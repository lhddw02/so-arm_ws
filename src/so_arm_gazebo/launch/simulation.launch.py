"""SO100 tabletop pick and place in Gazebo Classic with ROS 2 Humble."""

import os
import sys
from xml.dom import Node as XmlNode

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _continue_if_success(label, next_actions):
    """Start the next step only after this process reports successful completion."""
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            reason = f'{label} failed with exit code {event.returncode}'
            return [LogInfo(msg='ERROR: ' + reason), EmitEvent(event=Shutdown(reason=reason))]
        return [LogInfo(msg=f'{label} completed.')] + next_actions
    return on_exit


def _stop_if_publisher_exits(event, context):
    if context.is_shutdown:
        return []
    reason = f'robot_state_publisher stopped with exit code {event.returncode}'
    return [LogInfo(msg='ERROR: ' + reason), EmitEvent(event=Shutdown(reason=reason))]


def generate_launch_description():
    package = 'so_arm_gazebo'
    share = get_package_share_directory(package)
    controllers_file = os.path.join(share, 'config', 'controllers.yaml')
    document = xacro.process_file(
        os.path.join(share, 'urdf', 'so100.urdf.xacro'),
        mappings={'controller_config': controllers_file},
    )
    # Humble gazebo_ros2_control passes the description through the ROS argument
    # parser. Colons/newlines in XML comments can be interpreted as YAML syntax.
    def strip_comments(node):
        for child in list(node.childNodes):
            if child.nodeType == XmlNode.COMMENT_NODE:
                node.removeChild(child)
                child.unlink()
            else:
                strip_comments(child)
    strip_comments(document)
    robot_description = document.toxml()

    plugin_path = os.path.join(get_package_prefix(package), 'lib')
    inherited_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    if inherited_plugin_path:
        plugin_path += os.pathsep + inherited_plugin_path

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(share, 'worlds', 'pick_place.world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true',
            'pause': 'false',
            'server_required': 'true',
            'gui_required': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        prefix=sys.executable,
        output='screen',
        # The xacro anchors the robot base at world z=0.60 already.
        arguments=['-entity', 'so100', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0', '-timeout', '60'],
    )

    def controller_spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            prefix=sys.executable,
            output='screen',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60',
                       '--service-call-timeout', '30', '--switch-timeout', '30'],
        )

    joint_state_spawner = controller_spawner('joint_state_broadcaster')
    arm_spawner = controller_spawner('arm_controller')
    gripper_spawner = controller_spawner('gripper_controller')
    demo = Node(
        package=package,
        executable='pick_and_place',
        name='pick_and_place',
        output='screen',
        condition=IfCondition(LaunchConfiguration('demo')),
        parameters=[os.path.join(share, 'config', 'demo.yaml'), {'use_sim_time': True}],
    )

    # Register handlers before starting processes: no fixed startup sleeps.
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=robot_state_publisher, on_exit=_stop_if_publisher_exits)),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=_continue_if_success('Robot spawn', [joint_state_spawner]))),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=_continue_if_success('Joint state broadcaster activation', [arm_spawner]))),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_spawner,
            on_exit=_continue_if_success('Arm controller activation', [gripper_spawner]))),
        RegisterEventHandler(OnProcessExit(
            target_action=gripper_spawner,
            on_exit=_continue_if_success('Gripper controller activation', [demo]))),
        RegisterEventHandler(OnProcessExit(
            target_action=demo,
            on_exit=_continue_if_success('Pick and place demo', []))),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the Gazebo Classic GUI.'),
        DeclareLaunchArgument('demo', default_value='true',
                              description='Run one automatic pick and place cycle.'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),
        *handlers,
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
