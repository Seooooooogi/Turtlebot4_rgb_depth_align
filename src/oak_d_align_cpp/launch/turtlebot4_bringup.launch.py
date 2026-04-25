"""TurtleBot4 bringup using oak_d_align_cpp instead of vendor oakd.launch.py.

Mirrors `turtlebot4_bringup/launch/standard.launch.py` (robot, joy, rplidar,
description, conditional diagnostics) but excludes the vendor `oakd.launch.py`
include and adds our `oak_d_align_cpp` composable node in its own container.

Vendor immutability: this file lives in `oak_d_align_cpp` only; the vendor
launch file in `/opt/ros/humble/share/turtlebot4_bringup/` is not modified.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    lc = LaunchContext()

    diagnostics_enable = EnvironmentVariable('TURTLEBOT4_DIAGNOSTICS', default_value='1')
    namespace = EnvironmentVariable('ROBOT_NAMESPACE', default_value='')

    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    pkg_turtlebot4_diagnostics = get_package_share_directory('turtlebot4_diagnostics')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    pkg_oak_d_align_cpp = get_package_share_directory('oak_d_align_cpp')

    default_params_file = os.path.join(pkg_oak_d_align_cpp, 'config', 'oakd_params.yaml')

    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_bringup, 'config', 'turtlebot4.yaml']),
        description='TurtleBot4 robot param file (vendor)'
    )
    oakd_params_file_cmd = DeclareLaunchArgument(
        'oakd_params_file',
        default_value=default_params_file,
        description='oak_d_align_cpp parameter YAML'
    )

    param_file = LaunchConfiguration('param_file')
    oakd_params_file = LaunchConfiguration('oakd_params_file')

    namespaced_param_file = RewrittenYaml(
        source_file=param_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)

    turtlebot4_robot_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'robot.launch.py'])
    joy_teleop_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'joy_teleop.launch.py'])
    diagnostics_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_diagnostics, 'launch', 'diagnostics.launch.py'])
    rplidar_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'rplidar.launch.py'])
    description_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py'])

    # Standalone executable (option (c) — fallback from composable container).
    # Rationale: ComposableNodeContainer ↔ launch_ros LoadComposableNodes handshake
    # turned out to be non-deterministic on this robot (Load Library never invoked
    # on second restart). Container's only resident was OakdSender, so zero-copy
    # benefit was 0 anyway. ADR-004 updated.
    oakd_node = Node(
        package='oak_d_align_cpp',
        executable='oakd_sender',
        name='oakd_sender',
        parameters=[oakd_params_file],
        output='screen',
    )

    actions = [
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_robot_launch_file]),
            launch_arguments=[('model', 'standard'),
                              ('param_file', namespaced_param_file)]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([joy_teleop_launch_file]),
            launch_arguments=[('namespace', namespace)]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rplidar_launch_file])),

        # NOTE: vendor oakd.launch.py intentionally omitted. Replaced by
        # `oakd_node` (oak_d_align_cpp standalone executable).
        oakd_node,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_launch_file]),
            launch_arguments=[('model', 'standard')]),
    ]

    if diagnostics_enable.perform(lc) == '1':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([diagnostics_launch_file]),
            launch_arguments=[('namespace', namespace)]))

    ld = LaunchDescription()
    ld.add_action(param_file_cmd)
    ld.add_action(oakd_params_file_cmd)
    ld.add_action(GroupAction(actions))
    return ld
