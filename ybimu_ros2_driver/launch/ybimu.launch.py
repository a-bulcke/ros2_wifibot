import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ybimu_ros2_driver')
    return LaunchDescription([
        Node(
            package='ybimu_ros2_driver',
            executable='ybimu_node',
            name='ybimu_node',
            parameters=[os.path.join(pkg, 'config', 'ybimu.yaml')],
            output='screen',
        ),
    ])