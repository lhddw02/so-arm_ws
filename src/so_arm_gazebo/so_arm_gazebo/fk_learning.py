"""Connect joint sliders to Gazebo, and visualize measured forward kinematics.

The mathematical FK implementation is in kinematics.py. This node supplies its
measured joint angles, publishes its intermediate matrices, and compares its
result with robot_state_publisher's independent URDF/TF computation.
"""

from collections import deque
import json
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from .kinematics import (
    ARM_JOINTS, GRIPPER_JOINT, forward_kinematics_chain, position_jacobian,
    trajectory_duration, validate_joint_positions,
)


JOINTS = (*ARM_JOINTS, GRIPPER_JOINT)


def rotation_quaternion(matrix):
    """Convert a 3x3 rotation to ROS's xyzw quaternion for pose visualization."""
    trace = sum(matrix[i][i] for i in range(3))
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [(matrix[2][1] - matrix[1][2])/s,
             (matrix[0][2] - matrix[2][0])/s,
             (matrix[1][0] - matrix[0][1])/s, s/4.0]
    else:
        i = max(range(3), key=lambda index: matrix[index][index])
        j, k = (i+1) % 3, (i+2) % 3
        s = 2.0 * math.sqrt(max(0.0, 1.0 + matrix[i][i] - matrix[j][j] - matrix[k][k]))
        q = [0.0] * 4
        q[i] = s/4.0
        q[j] = (matrix[j][i] + matrix[i][j])/s
        q[k] = (matrix[k][i] + matrix[i][k])/s
        q[3] = (matrix[k][j] - matrix[j][k])/s
    norm = math.sqrt(sum(value*value for value in q))
    return [value/norm for value in q]


def rotation_rpy(matrix):
    """Display Rz(yaw) Ry(pitch) Rx(roll) angles; singularities use roll=0."""
    horizontal = math.hypot(matrix[0][0], matrix[1][0])
    pitch = math.atan2(-matrix[2][0], horizontal)
    if horizontal < 1e-8:
        return [0.0, pitch, math.atan2(-matrix[0][1], matrix[1][1])]
    return [math.atan2(matrix[2][1], matrix[2][2]), pitch,
            math.atan2(matrix[1][0], matrix[0][0])]


def duration(seconds):
    ns = round(seconds*1e9)
    return Duration(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)


def point(xyz):
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


class FkLearning(Node):
    def __init__(self):
        super().__init__('fk_learning')
        for key, value in {
            'base_height': 0.6, 'control_rate': 20.0, 'report_rate': 10.0,
            'max_joint_velocity': 0.6, 'minimum_motion_duration': 0.25,
            'startup_timeout': 60.0, 'trail_length': 250,
        }.items():
            self.declare_parameter(key, value)
        for key in ('control_rate', 'report_rate', 'startup_timeout'):
            if not math.isfinite(self.param(key)) or self.param(key) <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if not math.isfinite(self.param('base_height')) or self.param('trail_length') < 2:
            raise ValueError('Invalid base_height or trail_length')
        trajectory_duration([0.0], [0.0], self.param('max_joint_velocity'),
                            self.param('minimum_motion_duration'))
        self.positions = {}
        self.desired = {}
        self.last_sent = {}
        self.samples = deque(maxlen=300)
        self.trail = deque(maxlen=self.param('trail_length'))
        self.selected = ARM_JOINTS[0]
        self.state_time = None
        self.last_state_wall = None
        self.started = time.monotonic()
        self.ever_ready = False
        self.last_error = ''
        self.last_warn_wall = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, '/joint_states', self.on_measured,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, '/fk/desired_joint_states', self.on_desired, 10)
        # Match the dashboard's retained initial selection even if discovery
        # finishes after the dashboard publishes its first message.
        self.create_subscription(String, '/fk/selected_joint', self.on_selected,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       '/arm_controller/follow_joint_trajectory')
        self.jaw_client = ActionClient(self, FollowJointTrajectory,
                                       '/gripper_controller/follow_joint_trajectory')
        self.arm_pub = self.create_publisher(JointTrajectory,
                                            '/arm_controller/joint_trajectory', 10)
        self.jaw_pub = self.create_publisher(JointTrajectory,
                                            '/gripper_controller/joint_trajectory', 10)
        self.report_pub = self.create_publisher(String, '/fk/report', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/fk/tool_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fk/markers', 10)
        # Wall-clock timers can still report that simulation is paused/stale.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0/self.param('control_rate'), self.control, clock=steady)
        self.create_timer(1.0/self.param('report_rate'), self.report, clock=steady)
        self.get_logger().info('FK learning ready: sliders -> Gazebo -> measured joints -> matrices/TF')

    def param(self, name):
        return self.get_parameter(name).value

    def on_measured(self, message):
        for name, value in zip(message.name, message.position):
            if name in JOINTS and math.isfinite(value):
                self.positions[name] = value
                self.desired.setdefault(name, value)
        if all(name in self.positions for name in JOINTS):
            self.state_time = message.header.stamp
            self.last_state_wall = time.monotonic()
            stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
            self.samples.append((stamp, [self.positions[name] for name in JOINTS]))

    def on_desired(self, message):
        try:
            if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
                raise ValueError('JointState requires unique names and matching positions')
            updates = {name: value for name, value in zip(message.name, message.position)
                       if name in JOINTS}
            if not updates:
                raise ValueError('No SO-ARM joints in slider command')
            validate_joint_positions(list(updates), list(updates.values()))
            self.desired.update(updates)
            self.last_error = ''
        except ValueError as error:
            self.last_error = f'Rejected slider command: {error}'
            if time.monotonic() - self.last_warn_wall > 1.0:
                self.get_logger().warn(self.last_error)
                self.last_warn_wall = time.monotonic()

    def on_selected(self, message):
        if message.data in ARM_JOINTS:
            self.selected = message.data

    def readiness(self):
        if self.state_time is None:
            return False, 'Waiting for measured joint states'
        if not (self.arm_client.server_is_ready() and self.jaw_client.server_is_ready()):
            return False, 'Waiting for active arm and gripper controllers'
        if time.monotonic() - self.last_state_wall > 1.0:
            return False, 'Gazebo is paused or measured joint states are stale'
        return True, self.last_error or 'Live measured FK; drag a joint slider'

    def control(self):
        ready, _ = self.readiness()
        if not ready:
            if not self.ever_ready and time.monotonic() - self.started > self.param('startup_timeout'):
                raise RuntimeError('FK learning timed out waiting for Gazebo/controllers')
            return
        self.ever_ready = True
        for names, publisher in ((ARM_JOINTS, self.arm_pub), ((GRIPPER_JOINT,), self.jaw_pub)):
            target = [self.desired[name] for name in names]
            if all(abs(self.last_sent.get(name, math.inf) - value) < 1e-5
                   for name, value in zip(names, target)):
                continue
            start = [self.positions[name] for name in names]
            seconds = trajectory_duration(start, target, self.param('max_joint_velocity'),
                                          self.param('minimum_motion_duration'))
            trajectory = JointTrajectory()
            trajectory.joint_names = list(names)
            # Zero velocity/acceleration endpoints make the controller interpolate
            # quintically. Replacing a goal follows the slider from the measured pose.
            for values, elapsed in ((start, 0.0), (target, seconds)):
                trajectory.points.append(JointTrajectoryPoint(
                    positions=[float(v) for v in values], velocities=[0.0]*len(names),
                    accelerations=[0.0]*len(names), time_from_start=duration(elapsed)))
            publisher.publish(trajectory)
            self.last_sent.update(zip(names, target))

    def matched_state(self):
        """Match the TF snapshot to its measured joint sample before comparing FK."""
        values = [self.positions[name] for name in JOINTS]
        tf = None
        stamp = self.state_time
        try:
            candidate = self.tf_buffer.lookup_transform('world', 'tool0', Time())
            seconds = candidate.header.stamp.sec + candidate.header.stamp.nanosec*1e-9
            nearest = min(self.samples, key=lambda sample: abs(sample[0] - seconds))
            if abs(nearest[0] - seconds) < 1e-5:
                values = nearest[1]
                tf = candidate
                stamp = candidate.header.stamp
        except TransformException:
            pass
        return values, tf, stamp

    def report(self):
        ready, status = self.readiness()
        if self.state_time is None:
            self.report_pub.publish(String(data=json.dumps({'ready': False, 'status': status})))
            return
        values, tf, stamp = self.matched_state()
        chain = forward_kinematics_chain(values[:5])
        world_tool = [row[:] for row in chain['base_tool']]
        world_tool[2][3] += self.param('base_height')
        xyz = [world_tool[i][3] for i in range(3)]
        rotation = chain['rotation']
        quaternion = rotation_quaternion(rotation)
        tf_xyz, error, orientation_error = None, None, None
        if tf is not None:
            p, q = tf.transform.translation, tf.transform.rotation
            tf_xyz = [p.x, p.y, p.z]
            error = math.dist(xyz, tf_xyz)
            dot = sum(a*b for a, b in zip(quaternion, [q.x, q.y, q.z, q.w]))
            orientation_error = 2.0*math.acos(min(1.0, abs(dot)))
        elif ready:
            status += '; waiting for time-matched TF comparison'
        jacobian = position_jacobian(values[:5])
        data = {
            'ready': ready, 'status': status, 'joint_names': list(JOINTS),
            'desired_positions': [self.desired[name] for name in JOINTS],
            'measured_positions': values, 'chain': chain['steps'],
            'base_tool': chain['base_tool'], 'world_tool': world_tool,
            'tcp_world': xyz, 'rpy_world': rotation_rpy(rotation), 'tcp_tf_world': tf_xyz,
            'tf_position_error': error, 'tf_orientation_error': orientation_error,
            'jacobian': jacobian, 'selected_joint': self.selected,
        }
        self.report_pub.publish(String(data=json.dumps(data, allow_nan=False)))
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'world', stamp
        pose.pose.position = point(xyz)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, \
            pose.pose.orientation.w = quaternion
        self.pose_pub.publish(pose)
        if not self.trail or math.dist(xyz, self.trail[-1]) > 0.001:
            self.trail.append(xyz)
        self.publish_markers(chain, xyz, tf_xyz, jacobian, stamp)

    def marker(self, namespace, ident, kind, stamp, rgba, scale):
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = 'world', stamp
        marker.ns, marker.id, marker.type, marker.action = namespace, ident, kind, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.lifetime = duration(0.5)
        return marker

    def publish_markers(self, chain, xyz, tf_xyz, jacobian, stamp):
        markers = []
        height = self.param('base_height')
        frames = [[[float(i == j) for j in range(4)] for i in range(4)]]
        frames += [step['cumulative'] for step in chain['steps']] + [chain['base_tool']]
        names = ['base', 'shoulder', 'upper_arm', 'lower_arm', 'wrist', 'gripper_link', 'tool0']
        skeleton = self.marker('chain', 0, Marker.LINE_STRIP, stamp, (0.2, 0.9, 1.0, 1.0),
                               (0.004, 0.0, 0.0))
        for index, (matrix, name) in enumerate(zip(frames, names)):
            origin = [matrix[i][3] + (height if i == 2 else 0) for i in range(3)]
            skeleton.points.append(point(origin))
            for axis, color in enumerate(((1.0, 0.15, 0.1, 1.0), (0.1, 0.9, 0.2, 1.0),
                                          (0.15, 0.3, 1.0, 1.0))):
                arrow = self.marker('frames', 3*index+axis, Marker.ARROW, stamp, color,
                                    (0.0025, 0.005, 0.007))
                arrow.points = [point(origin), point([origin[i]+0.045*matrix[i][axis]
                                                      for i in range(3)])]
                markers.append(arrow)
            label = self.marker('frame_labels', index, Marker.TEXT_VIEW_FACING, stamp,
                                (0.95, 0.95, 0.95, 1.0), (0.0, 0.0, 0.013))
            label.pose.position = point([origin[0], origin[1], origin[2]+0.025])
            label.text = name
            markers.append(label)
        markers.append(skeleton)
        tool = self.marker('tcp', 0, Marker.SPHERE, stamp, (0.0, 1.0, 1.0, 1.0),
                           (0.014, 0.014, 0.014))
        tool.pose.position = point(xyz)
        markers.append(tool)
        if tf_xyz is not None:
            actual = self.marker('tcp', 1, Marker.SPHERE, stamp, (1.0, 0.7, 0.1, 0.65),
                                 (0.01, 0.01, 0.01))
            actual.pose.position = point(tf_xyz)
            markers.append(actual)
        trail = self.marker('trail', 0, Marker.LINE_STRIP, stamp, (0.0, 0.85, 0.95, 0.7),
                            (0.003, 0.0, 0.0))
        trail.points = [point(p) for p in self.trail]
        if len(trail.points) >= 2:
            markers.append(trail)
        selected = ARM_JOINTS.index(self.selected)
        frame = chain['joint_frames'][selected]
        origin = list(frame['joint_origin_in_base'])
        origin[2] += height
        axis = frame['axis_in_base']
        highlight = self.marker('selected', 0, Marker.ARROW, stamp, (1.0, 0.1, 0.9, 1.0),
                                (0.006, 0.012, 0.018))
        highlight.points = [point(origin), point([origin[i]+0.1*axis[i] for i in range(3)])]
        markers.append(highlight)
        displacement = [jacobian[i][selected]*math.radians(1.0)*10.0 for i in range(3)]
        derivative = self.marker('selected', 1, Marker.ARROW, stamp, (1.0, 0.1, 0.9, 1.0),
                                 (0.003, 0.006, 0.009))
        derivative.points = [point(xyz), point([xyz[i]+displacement[i] for i in range(3)])]
        # Wrist roll may leave position unchanged; do not create a zero-length arrow.
        if math.sqrt(sum(v*v for v in displacement)) > 1e-7:
            markers.append(derivative)
        label = self.marker('selected', 2, Marker.TEXT_VIEW_FACING, stamp,
                            (1.0, 0.5, 0.95, 1.0), (0.0, 0.0, 0.015))
        label.pose.position = point([xyz[0], xyz[1], xyz[2]+0.05])
        label.text = f'{self.selected}: +1 degree TCP derivative (x10)'
        markers.append(label)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = None
    result = 0
    try:
        node = FkLearning()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (ValueError, RuntimeError) as error:
        result = 1
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(error)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
