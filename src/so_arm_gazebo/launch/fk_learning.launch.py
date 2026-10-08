"""Interactive SO100 forward-kinematics lab for ROS 2 Humble/Gazebo Classic."""

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


def _after_success(label, next_actions):
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            reason = f'{label} failed with exit code {event.returncode}'
            return [LogInfo(msg='ERROR: ' + reason), EmitEvent(event=Shutdown(reason=reason))]
        return [LogInfo(msg=f'{label} completed.')] + next_actions
    return on_exit


def _required_node_exited(label):
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        reason = f'{label} stopped with exit code {event.returncode}'
        return [LogInfo(msg='ERROR: ' + reason), EmitEvent(event=Shutdown(reason=reason))]
    return on_exit


def generate_launch_description():
    package = 'so_arm_gazebo'
    share = get_package_share_directory(package)
    document = xacro.process_file(
        os.path.join(share, 'urdf', 'so100.urdf.xacro'),
        mappings={'controller_config': os.path.join(share, 'config', 'controllers.yaml')},
    )

    # The lab has no object to grasp. Keep the original source xacro unchanged.
    for plugin in list(document.getElementsByTagName('plugin')):
        if plugin.getAttribute('name') == 'so_arm_grasp':
            plugin.parentNode.removeChild(plugin)
            plugin.unlink()

    # Humble gazebo_ros2_control forwards this through the ROS argument parser.
    # Remove XML comments containing YAML-significant punctuation/newlines.
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
    inherited = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    if inherited:
        plugin_path += os.pathsep + inherited
    configuration = os.path.join(share, 'config', 'fk_learning.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(share, 'worlds', 'fk_learning.world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true',
            'pause': 'false',
            'server_required': 'true',
            'gui_required': 'true',
        }.items(),
    )
    publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', prefix=sys.executable,
        output='screen',
        # The fixed world_to_base joint already raises the arm to z=0.60 m.
        arguments=['-entity', 'so100', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0', '-timeout', '60'],
    )

    def spawner(controller):
        return Node(
            package='controller_manager', executable='spawner', prefix=sys.executable,
            output='screen',
            arguments=[controller, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60',
                       '--service-call-timeout', '30', '--switch-timeout', '30'],
        )

    joint_states = spawner('joint_state_broadcaster')
    arm = spawner('arm_controller')
    gripper = spawner('gripper_controller')
    learner = Node(
        package=package, executable='fk_learning', name='fk_learning',
        prefix=sys.executable, output='screen', parameters=[configuration],
    )
    sliders = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        name='fk_sliders', prefix=sys.executable, output='screen',
        condition=IfCondition(LaunchConfiguration('sliders')),
        parameters=[configuration, {'robot_description': robot_description}],
        # Only the Gazebo broadcaster publishes actual /joint_states for TF/RViz.
        remappings=[('joint_states', '/fk/desired_joint_states')],
    )
    dashboard = Node(
        package=package, executable='fk_dashboard', name='fk_dashboard',
        prefix=sys.executable, output='screen',
        condition=IfCondition(LaunchConfiguration('dashboard')),
        parameters=[{'use_sim_time': True}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='fk_rviz', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(share, 'config', 'fk_learning.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=publisher, on_exit=_required_node_exited('Robot state publisher'))),
        RegisterEventHandler(OnProcessExit(
            target_action=learner, on_exit=_required_node_exited('FK learner'))),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=_after_success('Robot spawn', [joint_states]))),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_states, on_exit=_after_success('Joint state activation', [arm]))),
        RegisterEventHandler(OnProcessExit(
            target_action=arm, on_exit=_after_success('Arm controller activation', [gripper]))),
        RegisterEventHandler(OnProcessExit(
            target_action=gripper,
            on_exit=_after_success('Gripper controller activation',
                                   [learner, sliders, dashboard, rviz]))),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the Gazebo Classic window.'),
        DeclareLaunchArgument('sliders', default_value='true',
                              description='Show joint-angle sliders.'),
        DeclareLaunchArgument('dashboard', default_value='true',
                              description='Show the live FK values dashboard.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Show RViz with robot and FK markers.'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),
        *handlers, gazebo, publisher, spawn,
    ])
