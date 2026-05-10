#!/usr/bin/env python3
"""
Wifibot bringup — robot + YLidar X2 + caméra DSI
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('ros2wifibot')

    # ── Arguments ──────────────────────────────────────────────────────────
    use_lidar  = LaunchConfiguration('use_lidar',  default='true')
    use_camera = LaunchConfiguration('use_camera', default='true')

    declare_lidar  = DeclareLaunchArgument('use_lidar',  default_value='true',
                                           description='Activer le YLidar X2')
    declare_camera = DeclareLaunchArgument('use_camera', default_value='true',
                                           description='Activer la caméra DSI')

    # ── Node wifibot ────────────────────────────────────────────────────────
    wifibot_node = Node(
        package='ros2wifibot',
        executable='wifibot_node',
        name='wifibot_node',
        parameters=[os.path.join(pkg, 'config', 'wifibot.yaml')],
        output='screen',
    )

    # ── YLidar X2 ───────────────────────────────────────────────────────────
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=[os.path.join(pkg, 'config', 'ydlidar_x2.yaml')],
        output='screen',
        condition=IfCondition(use_lidar),
    )

    # ── Caméra DSI (via v4l2) ───────────────────────────────────────────────
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size':   [640, 480],
            'camera_frame_id': 'camera_link',
        }],
        output='screen',
        condition=IfCondition(use_camera),
    )

    # ── TF statiques ────────────────────────────────────────────────────────
    # base_link → laser_frame
    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_laser',
        arguments=['0.15', '0', '0.10',   # x y z  (lidar à l'avant, 10cm de haut)
                   '0', '0', '0',          # roll pitch yaw
                   'base_link', 'laser_frame'],
        condition=IfCondition(use_lidar),
    )

    # base_link → camera_link
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=['0.10', '0', '0.15',   # x y z
                   '0', '0', '0',
                   'base_link', 'camera_link'],
        condition=IfCondition(use_camera),
    )

    return LaunchDescription([
        declare_lidar,
        declare_camera,
        wifibot_node,
        lidar_node,
        camera_node,
        tf_lidar,
        tf_camera,
    ])