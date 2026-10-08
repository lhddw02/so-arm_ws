#!/usr/bin/python3
"""Exercise real joint sliders, Gazebo motion, FK/TF, and the Qt dashboard."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewers', action='store_true',
                        help='Show Gazebo, RViz, sliders and dashboard on the current display')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    log_dir = workspace / 'log' / 'fk_smoke_test'
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', '74')
    os.environ.setdefault('GAZEBO_MASTER_URI', 'http://127.0.0.1:11376')
    os.environ['ROS_LOG_DIR'] = str(log_dir / 'ros')
    if not args.viewers:
        os.environ['QT_QPA_PLATFORM'] = 'offscreen'

    # System Python and the sourced workspace supply both ROS and Qt bindings.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    from gazebo_msgs.srv import GetEntityState
    from visualization_msgs.msg import MarkerArray
    from joint_state_publisher.joint_state_publisher import JointStatePublisher
    from joint_state_publisher_gui.joint_state_publisher_gui import JointStatePublisherGui
    from python_qt_binding.QtWidgets import QApplication
    from so_arm_gazebo.fk_dashboard import DashboardWindow
    from so_arm_gazebo.kinematics import ARM_JOINTS, GRIPPER_JOINT

    configuration = workspace / 'src/so_arm_gazebo/config/fk_learning.yaml'
    # These arguments apply to the real JSP node. The observer opts out below.
    rclpy.init(args=['--ros-args', '-r', '__node:=fk_sliders', '-r',
                    'joint_states:=/fk/desired_joint_states', '--params-file', str(configuration)])
    application = QApplication([])
    observer = Node('fk_smoke_observer', use_global_arguments=False)
    jsp = JointStatePublisher(None)
    sliders = JointStatePublisherGui('SO100 FK smoke test sliders', jsp)
    selected_pub = observer.create_publisher(
        String, '/fk/selected_joint',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    window = DashboardWindow(lambda name: selected_pub.publish(String(data=name)))
    sliders.show()
    window.show()
    latest = {}
    marker_count = [0]

    def receive(message):
        latest.clear()
        latest.update(json.loads(message.data))
        window.update_report(latest)

    observer.create_subscription(String, '/fk/report', receive, 10)
    observer.create_subscription(MarkerArray, '/fk/markers',
                                 lambda message: marker_count.__setitem__(0, len(message.markers)), 10)
    state_client = observer.create_client(GetEntityState, '/gazebo/get_entity_state')
    command_pub = observer.create_publisher(JointState, '/fk/desired_joint_states', 10)
    process = None
    log_file = log_dir / 'simulation.log'

    def pump():
        rclpy.spin_once(jsp, timeout_sec=0.0)
        rclpy.spin_once(observer, timeout_sec=0.02)
        application.processEvents()
        if process is not None and process.poll() is not None:
            raise RuntimeError(f'Launch exited unexpectedly; see {log_file}')

    def wait(predicate, label, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pump()
            if predicate():
                return
        raise RuntimeError(f'Timed out: {label}; latest status {latest.get("status")}')

    def check_pose():
        assert latest['tf_position_error'] < 1e-5, latest['tf_position_error']
        assert latest['tf_orientation_error'] < 1e-5, latest['tf_orientation_error']
        request = GetEntityState.Request(name='so100::tool0', reference_frame='world')
        future = state_client.call_async(request)
        wait(future.done, 'Gazebo tool link query')
        response = future.result()
        assert response.success, 'Gazebo tool0 link not found'
        p = response.state.pose.position
        physical = [p.x, p.y, p.z]
        error = math.dist(physical, latest['tcp_world'])
        assert error < 0.001, f'Gazebo vs analytical FK mismatch {error} m'
        print(f'  FK/TF/Gazebo agree: error {error*1000:.4f} mm', flush=True)

    def move(updates):
        target = dict(zip(latest['joint_names'], latest['desired_positions']))
        for name, value in updates.items():
            info = sliders.joint_map[name]
            info['slider'].setValue(sliders.valueToSlider(value, info['joint']))
            # Official JSP sliders quantize their joint range to 10,000 steps.
            target[name] = info['joint']['position']
        wait(lambda: latest.get('ready') and latest.get('tf_position_error') is not None
             and all(abs(latest['measured_positions'][latest['joint_names'].index(name)]-value)
                     < 0.0008 for name, value in target.items()), 'Sliders reaching measured pose')
        check_pose()

    passed = False
    try:
        with log_file.open('w') as output:
            command = ['ros2', 'launch', 'so_arm_gazebo', 'fk_learning.launch.py',
                       f'gui:={str(args.viewers).lower()}', 'sliders:=false',
                       'dashboard:=true', f'rviz:={str(args.viewers).lower()}']
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            wait(lambda: latest.get('ready') and latest.get('tf_position_error') is not None
                 and len(sliders.joint_map) == 6 and marker_count[0] > 20
                 and state_client.service_is_ready(), 'Learning scene readiness', 90.0)
            assert observer.count_publishers('/joint_states') == 1, 'Competing measured-state publishers'
            wait(lambda: latest.get('selected_joint') == 'shoulder_lift',
                 'Retained initial dashboard selection')
            print('Ready: actual JSP GUI, dashboard, measured-state markers and Gazebo', flush=True)
            check_pose()
            move({'shoulder_pan': 0.4})
            move({'shoulder_lift': 1.85, 'elbow_flex': -1.1, 'wrist_flex': 0.5})
            window.selected.setCurrentText('wrist_roll')
            wait(lambda: latest.get('selected_joint') == 'wrist_roll', 'Selected axis highlight')
            before = list(latest['tcp_world'])
            move({'wrist_roll': 1.0})
            assert math.dist(before, latest['tcp_world']) < 1e-5, 'Wrist roll should preserve TCP position'
            before = list(latest['tcp_world'])
            move({GRIPPER_JOINT: 0.2})
            assert math.dist(before, latest['tcp_world']) < 1e-5, 'Jaw should preserve TCP position'
            # Stop JSP briefly so this deliberately bad external command is observed.
            jsp.timer.cancel()
            command_pub.publish(JointState(name=['shoulder_pan'], position=[100.0]))
            wait(lambda: 'Rejected slider command' in latest.get('status', ''), 'Out-of-range rejection')
            jsp.timer.reset()
            move({'shoulder_pan': 2.0})
            move({'shoulder_pan': -2.0})
            move({'shoulder_pan': 0.0})
            window.grab().save(str(log_dir / 'dashboard_live.png'))
            sliders.grab().save(str(log_dir / 'sliders_live.png'))
            passed = True
    finally:
        sliders.close()
        window.close()
        observer.destroy_node()
        jsp.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None:
            for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                try:
                    if sig == signal.SIGINT:
                        # Let ROS launch signal its children once, then use the
                        # owned process group only as a termination fallback.
                        process.send_signal(sig)
                    else:
                        os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    continue
    if passed:
        print(f'PASS: slider motion, live matrices, TF/Gazebo agreement, markers, limits and jaw/roll invariance.\nLog: {log_file}')
        return 0
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
