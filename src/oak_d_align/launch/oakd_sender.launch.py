from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('oak_d_align'),
        'config',
        'oakd_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='oak_d_align',
            executable='oakd_sender',
            name='oakd_sender',
            output='screen',
            parameters=[params_file],
        )
    ])
