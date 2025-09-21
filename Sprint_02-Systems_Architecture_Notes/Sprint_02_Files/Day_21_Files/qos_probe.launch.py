from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('planning101')
    params = os.path.join(pkg_share, 'config', 'qos_probe.yaml')

    return LaunchDescription([
        Node(
            package='planning101',
            executable='qos_probe',
            name='qos_probe',
            parameters=[params],
            output='screen',
        )
    ])
