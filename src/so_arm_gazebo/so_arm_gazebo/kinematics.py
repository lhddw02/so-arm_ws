"""Dependency-free FK for the supplied SO100 URDF, in metres and radians.

The origin rotations deliberately retain the URDF's decimal values instead of
replacing 1.57079 with pi/2. The TCP is the centre of the distal jaw pads.
"""

import math


ARM_JOINTS = (
    'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll',
)
GRIPPER_JOINT = 'gripper'
JOINT_LIMITS = {
    'shoulder_pan': (-2.0, 2.0),
    'shoulder_lift': (0.0, 3.5),
    'elbow_flex': (-3.14158, 0.0),
    'wrist_flex': (-2.5, 1.2),
    'wrist_roll': (-3.14158, 3.14158),
    'gripper': (-0.2, 2.0),
}
JOINT_ORIGINS = (
    ((0.0, -0.0452, 0.0165), (1.57079, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ((0.0, 0.1025, 0.0306), (-1.8, 0.0, 0.0), (1.0, 0.0, 0.0)),
    ((0.0, 0.11257, 0.028), (1.57079, 0.0, 0.0), (1.0, 0.0, 0.0)),
    ((0.0, 0.0052, 0.1349), (-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    ((0.0, -0.0601, 0.0), (0.0, 1.57079, 0.0), (0.0, 1.0, 0.0)),
)
TCP_OFFSET = (0.0, -0.100, 0.0)


def _identity():
    return [[float(row == column) for column in range(4)] for row in range(4)]


def _multiply(left, right):
    return [
        [sum(left[i][k] * right[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def _axis_rotation(axis, angle):
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    result = _identity()
    result[0][:3] = [x*x*v + c, x*y*v - z*s, x*z*v + y*s]
    result[1][:3] = [y*x*v + z*s, y*y*v + c, y*z*v - x*s]
    result[2][:3] = [z*x*v - y*s, z*y*v + x*s, z*z*v + c]
    return result


def _origin_transform(xyz, rpy):
    roll, pitch, yaw = rpy
    result = _multiply(
        _multiply(_axis_rotation((0.0, 0.0, 1.0), yaw),
                  _axis_rotation((0.0, 1.0, 0.0), pitch)),
        _axis_rotation((1.0, 0.0, 0.0), roll),
    )
    for index, value in enumerate(xyz):
        result[index][3] = value
    return result


def validate_joint_positions(joint_names, positions):
    """Reject nonfinite values, wrong dimensions and original-URDF limit errors."""
    if len(joint_names) != len(positions):
        raise ValueError('Joint names and positions have different lengths')
    for name, position in zip(joint_names, positions):
        if name not in JOINT_LIMITS:
            raise ValueError(f'Unknown joint: {name}')
        if not math.isfinite(position):
            raise ValueError(f'{name} has a nonfinite position')
        lower, upper = JOINT_LIMITS[name]
        if not lower <= position <= upper:
            raise ValueError(f'{name}={position} exceeds [{lower}, {upper}]')


def forward_kinematics_chain(positions, tcp_offset=TCP_OFFSET):
    """Expose each homogeneous transform in the five-joint arm chain.

    Every matrix uses column vectors: ``local = origin @ motion`` and
    ``cumulative = previous_cumulative @ local``. ``origin`` is the URDF
    parent-to-joint transform at zero angle; ``motion`` rotates about the joint's
    local URDF axis. The final ``base_tool`` adds the fixed TCP translation.

    ``joint_frames`` contains each joint's origin and unit rotation axis in base
    coordinates, before its own motion. Lists and plain floats make the returned
    dictionary directly JSON serializable. The jaw joint is outside this chain.
    """
    validate_joint_positions(ARM_JOINTS, positions)
    if len(tcp_offset) != 3 or not all(math.isfinite(value) for value in tcp_offset):
        raise ValueError('TCP offset must contain three finite coordinates')
    transform = _identity()
    steps = []
    joint_frames = []
    for name, position, (xyz, rpy, axis) in zip(ARM_JOINTS, positions, JOINT_ORIGINS):
        angle = float(position)
        origin = _origin_transform(xyz, rpy)
        motion = _axis_rotation(axis, angle)
        local = _multiply(origin, motion)
        joint_frame = _multiply(transform, origin)
        joint_frames.append({
            'name': name,
            'angle': angle,
            'joint_origin_in_base': [joint_frame[i][3] for i in range(3)],
            'axis_in_base': [
                sum(joint_frame[i][j] * axis[j] for j in range(3))
                for i in range(3)
            ],
        })
        # Keep the original FK multiplication order and numerical results.
        transform = _multiply(joint_frame, motion)
        steps.append({
            'name': name,
            'angle': angle,
            'origin': origin,
            'motion': motion,
            'local': local,
            'cumulative': transform,
        })
    tool_transform = _identity()
    for index, value in enumerate(tcp_offset):
        tool_transform[index][3] = float(value)
    base_tool = _multiply(transform, tool_transform)
    return {
        'steps': steps,
        'joint_frames': joint_frames,
        'base_tool': base_tool,
        'position': [base_tool[i][3] for i in range(3)],
        'rotation': [row[:3] for row in transform[:3]],
    }


def forward_kinematics(positions, tcp_offset=TCP_OFFSET):
    """Return TCP position and 3x3 orientation relative to the base link."""
    chain = forward_kinematics_chain(positions, tcp_offset)
    return tuple(chain['position']), tuple(tuple(row) for row in chain['rotation'])


def position_jacobian(positions, tcp_offset=TCP_OFFSET):
    """Return the geometric position Jacobian, 3 rows by 5 columns, in m/rad.

    Column i is ``axis_in_base x (tcp_in_base - joint_origin_in_base)``.
    The rows correspond to x/y/z and columns to ``ARM_JOINTS``. With the default
    TCP on the wrist-roll axis, that joint changes orientation but not position,
    so its column is zero up to floating-point roundoff. A TCP offset away from
    the axis generally gives a nonzero wrist-roll column. The jaw is excluded.
    """
    chain = forward_kinematics_chain(positions, tcp_offset)
    point = chain['position']
    columns = []
    for frame in chain['joint_frames']:
        axis = frame['axis_in_base']
        radius = [point[i] - frame['joint_origin_in_base'][i] for i in range(3)]
        columns.append([
            axis[1]*radius[2] - axis[2]*radius[1],
            axis[2]*radius[0] - axis[0]*radius[2],
            axis[0]*radius[1] - axis[1]*radius[0],
        ])
    return [[column[row] for column in columns] for row in range(3)]


def trajectory_duration(start, target, max_velocity=0.6, minimum_duration=2.0):
    """Bound the *peak* velocity of zero-velocity/acceleration quintic segments.

    JointTrajectoryController uses quintic interpolation when both endpoint
    velocities and accelerations are supplied. Its peak speed is 1.875*delta/T.
    """
    if len(start) != len(target) or not start:
        raise ValueError('Trajectory endpoints must have matching nonempty dimensions')
    if not 0.0 < max_velocity <= 1.0:
        raise ValueError('max_velocity must be in (0, 1], the URDF velocity limit')
    if not math.isfinite(minimum_duration) or minimum_duration <= 0.0:
        raise ValueError('minimum_duration must be positive and finite')
    delta = max(abs(a - b) for a, b in zip(start, target))
    if not math.isfinite(delta):
        raise ValueError('Trajectory positions must be finite')
    return max(minimum_duration, 1.875 * delta / max_velocity + 0.1)


def validate_plan(waypoints, source, destination, base_height,
                  approach_height, release_clearance):
    """Ensure edited configuration still targets this scene and points down."""
    expected = {
        'home': (source[0], source[1], base_height + approach_height),
        'pick': tuple(source),
        'lift': (source[0], source[1], base_height + approach_height),
        'transfer': (destination[0], destination[1], base_height + approach_height),
        'place': (destination[0], destination[1], destination[2] + release_clearance),
        'retreat': (destination[0], destination[1], base_height + approach_height),
    }
    for name, world_target in expected.items():
        point, rotation = forward_kinematics(waypoints[name])
        world_point = (point[0], point[1], point[2] + base_height)
        error = math.dist(world_point, world_target)
        if error > 0.002:
            raise ValueError(f'{name} TCP misses configured target by {error:.4f} m')
        down = tuple(-rotation[index][1] for index in range(3))
        if math.dist(down, (0.0, 0.0, -1.0)) > 0.0001:
            raise ValueError(f'{name} gripper does not point downward')
