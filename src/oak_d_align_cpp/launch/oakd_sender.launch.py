"""Standalone launch for oak_d_align_cpp (ComposableNode in its own container)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('oak_d_align_cpp')
    default_params_file = os.path.join(pkg_share, 'config', 'oakd_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to oakd_sender parameters YAML.',
    )

    container = ComposableNodeContainer(
        name='oakd_cpp_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='oak_d_align_cpp',
                plugin='oak_d_align_cpp::OakdSender',
                name='oakd_sender',
                parameters=[LaunchConfiguration('params_file')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        container,
    ])
