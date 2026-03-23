"""Plik uruchomieniowy dla systemu RoboMVP – pełna wersja z diagnostyką i TF.

Uruchamia węzły w logicznej kolejności:
  1. robomvp_tf_publisher  – statyczne TF (konieczne zanim inne węzły użyją tf2)
  2. camera_interface      – strumień kamer
  3. marker_detection      – detekcja AprilTag / QR
  4. marker_pose_estimator – pozycja 3D markerów
  5. robomvp_odometry      – odometria dead-reckoning
  6. robomvp_diagnostics   – diagnostyki systemu
  7. robomvp_main          – automat stanowy (centralny orkiestrator)
  8. robomvp_teleop        – ręczne sterowanie (domyślnie wyłączone)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg   = get_package_share_directory('robomvp')
    scene = os.path.join(pkg, 'config', 'scene.yaml')
    cam   = os.path.join(pkg, 'config', 'camera.yaml')

    args = [
        DeclareLaunchArgument('marker_type',             default_value='apriltag'),
        DeclareLaunchArgument('network_interface',        default_value='eth0'),
        DeclareLaunchArgument('require_robot_connection', default_value='false'),
        DeclareLaunchArgument('body_camera_device',       default_value='0'),
        DeclareLaunchArgument('head_camera_device',       default_value='-1'),
        DeclareLaunchArgument('teleop_enabled',           default_value='false'),
        DeclareLaunchArgument('step_period',              default_value='1.0'),
    ]

    nodes = [
        Node(
            package='robomvp', executable='robomvp_tf_publisher',
            name='robomvp_tf_publisher', output='screen',
        ),
        Node(
            package='robomvp', executable='camera_interface',
            name='camera_interface', output='screen',
            parameters=[{
                'publish_rate':        10.0,
                'body_camera_device':  LaunchConfiguration('body_camera_device'),
                'head_camera_device':  LaunchConfiguration('head_camera_device'),
            }],
        ),
        Node(
            package='robomvp', executable='marker_detection',
            name='marker_detection', output='screen',
            parameters=[{'marker_type': LaunchConfiguration('marker_type')}],
        ),
        Node(
            package='robomvp', executable='marker_pose_estimator',
            name='marker_pose_estimator', output='screen',
            parameters=[{'camera_config_path': cam, 'marker_size': 0.1}],
        ),
        Node(
            package='robomvp', executable='robomvp_odometry',
            name='robomvp_odometry', output='screen',
            parameters=[{'publish_rate': 50.0}],
        ),
        Node(
            package='robomvp', executable='robomvp_diagnostics',
            name='robomvp_diagnostics', output='screen',
            parameters=[{'publish_rate': 1.0}],
        ),
        Node(
            package='robomvp', executable='robomvp_main',
            name='robomvp_main', output='screen',
            parameters=[{
                'scene_config_path':       scene,
                'step_period':             LaunchConfiguration('step_period'),
                'network_interface':       LaunchConfiguration('network_interface'),
                'require_robot_connection': LaunchConfiguration('require_robot_connection'),
            }],
        ),
        Node(
            package='robomvp', executable='robomvp_teleop',
            name='robomvp_teleop', output='screen',
            parameters=[{
                'enabled':            LaunchConfiguration('teleop_enabled'),
                'max_linear_speed':   0.3,
                'max_angular_speed':  0.5,
            }],
        ),
    ]

    return LaunchDescription(args + nodes)
