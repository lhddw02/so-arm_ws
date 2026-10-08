"""Run and verify one SO-ARM100 Gazebo pick/place cycle using ROS 2 actions."""

import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from gazebo_msgs.srv import GetEntityState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

from .kinematics import (
    ARM_JOINTS, GRIPPER_JOINT, trajectory_duration,
    validate_joint_positions, validate_plan,
)


DEFAULT_WAYPOINTS = {
    'home': [0.0, 1.6569670531, -1.5806712649, 1.1529205386, 0.0],
    'pick': [0.0, 1.7684468165, -1.0210583541, 0.4818278644, 0.0],
    'lift': [0.0, 1.6569670531, -1.5806712649, 1.1529205386, 0.0],
    'transfer': [0.6844335123, 1.6497761515, -1.5728717730, 1.1523105234, 0.0],
    'place': [0.6844355125, 1.7498382254, -1.0237518840, 0.5031285604, 0.0],
    'retreat': [0.6844335123, 1.6497761515, -1.5728717730, 1.1523105234, 0.0],
}


class DemoError(RuntimeError):
    """A required readiness, controller or physical scene check failed."""


def _duration(seconds):
    nanoseconds = round(seconds * 1_000_000_000)
    return Duration(sec=nanoseconds // 1_000_000_000,
                    nanosec=nanoseconds % 1_000_000_000)


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        defaults = {
            'startup_timeout': 60.0,
            'operation_timeout': 30.0,
            'max_joint_velocity': 0.6,
            'minimum_motion_duration': 2.0,
            'base_height': 0.60,
            'approach_height': 0.10,
            'release_clearance': 0.003,
            'object_name': 'pick_cube',
            'source_position': [0.0, -0.22, 0.610],
            'destination_position': [0.11, -0.18, 0.610],
            'position_tolerance': 0.015,
            'gripper_open': 0.8,
            'gripper_closed': 0.0,
            'settle_duration': 1.0,
        }
        for name, value in {**defaults, **DEFAULT_WAYPOINTS}.items():
            self.declare_parameter(name, value)
        self.waypoints = {name: self.param(name) for name in DEFAULT_WAYPOINTS}
        validate_plan(
            self.waypoints, self.param('source_position'),
            self.param('destination_position'), self.param('base_height'),
            self.param('approach_height'), self.param('release_clearance'),
        )
        for name in ('gripper_open', 'gripper_closed'):
            validate_joint_positions([GRIPPER_JOINT], [self.param(name)])
        for name in ('startup_timeout', 'operation_timeout', 'settle_duration',
                     'position_tolerance'):
            if not math.isfinite(self.param(name)) or self.param(name) <= 0.0:
                raise ValueError(f'{name} must be positive and finite')
        trajectory_duration([0.0], [0.0], self.param('max_joint_velocity'),
                            self.param('minimum_motion_duration'))
        self.positions = {}
        self.clock_seconds = None
        self.clock_has_advanced = False
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        # Humble has no qos_profile_clock constant. Match ClockQoS explicitly;
        # a best-effort reader also accepts a reliable Gazebo publisher.
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Clock, '/clock', self.on_clock, clock_qos)
        self.arm = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.grasp = self.create_client(SetBool, '/grasp/set_attached')
        self.entity = self.create_client(GetEntityState, '/gazebo/get_entity_state')

    def param(self, name):
        return self.get_parameter(name).value

    def on_joints(self, message):
        for name, value in zip(message.name, message.position):
            if math.isfinite(value):
                self.positions[name] = value

    def on_clock(self, message):
        current = message.clock.sec + message.clock.nanosec * 1e-9
        if self.clock_seconds is not None and current > self.clock_seconds:
            self.clock_has_advanced = True
        self.clock_seconds = current

    def wait_ready(self):
        deadline = time.monotonic() + self.param('startup_timeout')
        self.get_logger().info('Waiting for Gazebo clock, joint states, controllers and services')
        checks = {'ROS context': False}
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            checks = {
                'advancing /clock': self.clock_has_advanced,
                'all six joint states': all(
                    name in self.positions for name in (*ARM_JOINTS, GRIPPER_JOINT)),
                'arm action': self.arm.server_is_ready(),
                'gripper action': self.gripper.server_is_ready(),
                'grasp service': self.grasp.service_is_ready(),
                'Gazebo state service': self.entity.service_is_ready(),
            }
            if all(checks.values()):
                return
        missing = ', '.join(name for name, ready in checks.items() if not ready)
        raise DemoError(f'Startup timed out waiting for: {missing}')

    def wait_future(self, future, label, timeout=None):
        deadline = time.monotonic() + (timeout or self.param('operation_timeout'))
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise DemoError(f'{label} timed out (Gazebo must remain unpaused)')
        exception = future.exception()
        if exception is not None:
            raise DemoError(f'{label} failed: {exception}')
        return future.result()

    def move(self, client, joint_names, target, label):
        validate_joint_positions(joint_names, target)
        start = [self.positions[name] for name in joint_names]
        duration = trajectory_duration(
            start, target, self.param('max_joint_velocity'),
            self.param('minimum_motion_duration'),
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        for positions, elapsed in ((start, 0.0), (target, duration)):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in positions]
            point.velocities = [0.0] * len(joint_names)
            point.accelerations = [0.0] * len(joint_names)
            point.time_from_start = _duration(elapsed)
            goal.trajectory.points.append(point)
        goal.goal_time_tolerance = _duration(2.0)
        goal.goal_tolerance = [
            JointTolerance(name=name, position=0.035, velocity=0.1)
            for name in joint_names
        ]
        self.get_logger().info(f'{label}: {duration:.1f} simulated seconds')
        handle = self.wait_future(client.send_goal_async(goal), f'{label} acceptance')
        if handle is None or not handle.accepted:
            raise DemoError(f'{label}: controller rejected trajectory')
        try:
            result = self.wait_future(
                handle.get_result_async(), f'{label} execution',
                max(self.param('operation_timeout'), duration * 5.0 + 5.0),
            )
        except DemoError:
            try:
                self.wait_future(handle.cancel_goal_async(), f'{label} cancellation', 3.0)
            except DemoError as error:
                self.get_logger().error(str(error))
            raise
        # ROS action status 4 is STATUS_SUCCEEDED. Check both status and controller result.
        if result.status != 4 or result.result.error_code != 0:
            raise DemoError(
                f'{label}: action status={result.status}, '
                f'error={result.result.error_code}: {result.result.error_string}')

    def move_arm(self, name):
        self.move(self.arm, ARM_JOINTS, self.waypoints[name], name)

    def move_gripper(self, closed):
        self.move(self.gripper, [GRIPPER_JOINT],
                  [self.param('gripper_closed' if closed else 'gripper_open')],
                  'Close gripper' if closed else 'Open gripper')

    def set_attached(self, attached):
        request = SetBool.Request()
        request.data = attached
        response = self.wait_future(self.grasp.call_async(request), 'Grasp service')
        if not response.success:
            raise DemoError(f'Grasp service rejected request: {response.message}')
        self.get_logger().info(response.message)

    def object_state(self):
        request = GetEntityState.Request()
        request.name = self.param('object_name')
        request.reference_frame = 'world'
        response = self.wait_future(self.entity.call_async(request), 'Object state query')
        if not response.success:
            raise DemoError(f'Gazebo cannot find object {request.name}')
        return response.state

    def check_object(self, expected, label):
        state = self.object_state()
        p = state.pose.position
        actual = (p.x, p.y, p.z)
        error = math.dist(actual, expected)
        if not math.isfinite(error) or error > self.param('position_tolerance'):
            raise DemoError(
                f'{label}: cube is at {actual}, expected {expected}; error={error:.3f} m')
        return state

    def settle(self):
        start = self.clock_seconds
        deadline = time.monotonic() + self.param('operation_timeout')
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.clock_seconds - start >= self.param('settle_duration'):
                return
        raise DemoError('Simulation clock stopped while waiting for cube to settle')

    def run(self):
        self.wait_ready()
        self.settle()
        self.check_object(self.param('source_position'), 'Initial cube position')
        self.move_arm('home')
        self.move_gripper(False)
        self.move_arm('pick')
        self.move_gripper(True)
        self.set_attached(True)
        self.move_arm('lift')
        source = self.param('source_position')
        lift_target = (source[0], source[1],
                       self.param('base_height') + self.param('approach_height'))
        self.check_object(lift_target, 'Lift verification')
        self.get_logger().info('Verified cube lifted clear of the table')
        self.move_arm('transfer')
        self.move_arm('place')
        # Keep the assisted joint while the jaw opens, then release above the table.
        self.move_gripper(False)
        self.set_attached(False)
        self.move_arm('retreat')
        self.move_arm('home')
        self.settle()
        state = self.check_object(self.param('destination_position'), 'Placement verification')
        linear = state.twist.linear
        angular = state.twist.angular
        speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
        spin = math.sqrt(angular.x**2 + angular.y**2 + angular.z**2)
        if not math.isfinite(speed + spin) or speed > 0.03 or spin > 0.3:
            raise DemoError(f'Placed cube has not settled: speed={speed:.3f}, spin={spin:.3f}')
        self.get_logger().info(
            'PICK AND PLACE SUCCEEDED: lifted cube and verified destination/rest state; '
            f'cube=({state.pose.position.x:.4f}, {state.pose.position.y:.4f}, '
            f'{state.pose.position.z:.4f}) m, speed={speed:.5f} m/s')


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = PickAndPlace()
        node.run()
    except (DemoError, ValueError, KeyboardInterrupt) as error:
        exit_code = 1
        if node is not None:
            node.get_logger().error(f'PICK AND PLACE FAILED: {error}')
        else:
            print(f'PICK AND PLACE FAILED: {error}', file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
