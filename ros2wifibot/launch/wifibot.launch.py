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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('ros2wifibot')
    imu_pkg = get_package_share_directory('ybimu_ros2_driver')

    # ── Arguments ──────────────────────────────────────────────────────────
    use_lidar  = LaunchConfiguration('use_lidar',  default='true')
    use_camera = LaunchConfiguration('use_camera', default='true')
    use_joy = LaunchConfiguration('use_joy')
    use_imu = LaunchConfiguration('use_imu')
    use_ekf = LaunchConfiguration('use_ekf')

    declare_lidar  = DeclareLaunchArgument('use_lidar',  default_value='true',
                                           description='Activer le YLidar X2')
    declare_camera = DeclareLaunchArgument('use_camera', default_value='true',
                                           description='Activer la caméra DSI')
    declare_use_joy = DeclareLaunchArgument('use_joy', default_value='false',
                                           description='Lancer la manette BT pour teleop')
    declare_use_imu = DeclareLaunchArgument('use_imu', default_value='true',
                                           description="Activer l'IMU Yahboom")
    declare_use_ekf = DeclareLaunchArgument('use_ekf', default_value='true',
                                           description="Activer la fusion EKF odom+IMU")


    # ── Node wifibot ────────────────────────────────────────────────────────
    wifibot_node = Node(
        package='ros2wifibot',
        executable='wifibot_node',
        name='wifibot_node',
        parameters=[
            os.path.join(pkg, 'config', 'wifibot.yaml'),
            {'publish_odom_tf': PythonExpression(["'", use_ekf, "' == 'false'"])},
        ],
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
#            'output_encoding': 'yuv422_yuy2',
            'camera_frame_id': 'camera_link',
        }],
        output='screen',
        condition=IfCondition(use_camera),
    )
    
    # ── Joystic BT  ─────────────────────────────────────────────────────────
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        parameters=[{
            #'device_id' : 0,
            'dev': '/dev/input/js0',
            'dev_ff': '/dev/input/event3',   # Feedback
            'deadzone' : 0.1,
            'autorepeat_rate' : 20.0,
        }],
        condition=IfCondition(use_joy),
    )
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[os.path.join(pkg, 'config', 'joy_teleop.yaml')],
        condition=IfCondition(use_joy),
    )
    
    # ── Capteurs Sharp IR ─────────────────────────────────────────────────────
    ir_distance_node = Node(
        package='ros2wifibot',
        executable='ir_distance_node.py',
        name='ir_distance_node',
        parameters=[{
            'left_channel': 1,
            'right_channel': 3,
            'voltage_scale': 1.0,
            'coeff_a': 65.0,
            'coeff_b': -1.10,
            'obstacle_threshold': 0.3,
        }],
        output='screen',
    )
    
    # ── Arret manette BT ───────────────────────────────────────────────────
    shutdown_node = Node(
        package='ros2wifibot',
        executable='shutdown_button_node.py',
        name='shutdown_button_node',
        parameters=[{
            'button_index': 8,       # BP Connect
            'hold_duration_sec': 3.0,
        }],
        condition=IfCondition(use_joy),
)   
    # ── IMU Yahboom 9dof ───────────────────────────────────────────────────
    imu_node = Node(
    package='ybimu_ros2_driver', executable='ybimu_node', name='ybimu_node',
    parameters=[os.path.join(imu_pkg, 'config', 'ybimu.yaml')],
    output='screen', condition=IfCondition(use_imu),
)
tf_imu = Node(
    package='tf2_ros', executable='static_transform_publisher', name='tf_base_to_imu',
    arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
    condition=IfCondition(use_imu),
)
    # ── Fusion odometrie robot (/odom) et IMU (/imu/data_raw) ───────────────
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
        output='screen', condition=IfCondition(use_ekf),
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
        declare_use_joy,
        declare_use_imu,
        declare_use_ekf,
        wifibot_node,
        lidar_node,
        camera_node,
        joy_node,
        teleop_node,
        ir_distance_node,
        shutdown_node,
        imu_node,
        tf_imu,
        ekf_node,
        tf_lidar,
        tf_camera,
    ])
