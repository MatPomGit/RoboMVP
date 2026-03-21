"""Plik uruchomieniowy dla systemu RoboMVP.

Uruchamia węzły w kolejności:
1. camera_interface - publikuje obrazy z rzeczywistych kamer
2. marker_detection - wykrywa markery w obrazach
3. marker_pose_estimator - oblicza pozycje 3D markerów
4. robomvp_main - główny węzeł sterujący
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generuje opis uruchomienia dla systemu RoboMVP."""

    package_share_dir = get_package_share_directory('robomvp')
    config_dir = os.path.join(package_share_dir, 'config')
    scene_config = os.path.join(config_dir, 'scene.yaml')
    camera_config = os.path.join(config_dir, 'camera.yaml')

    marker_type_arg = DeclareLaunchArgument(
        'marker_type',
        default_value='apriltag',
        description='Typ markera: apriltag lub qr',
    )
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='eth0',
        description='Interfejs sieciowy Ethernet podłączony do robota (np. eth0, enp3s0)',
    )

    require_robot_connection_arg = DeclareLaunchArgument(
        'require_robot_connection',
        default_value='false',
        description='Wymagaj aktywnego połączenia z robotem (true/false).',
    )
    body_camera_device_arg = DeclareLaunchArgument(
        'body_camera_device',
        default_value='0',
        description='Indeks urządzenia V4L2 dla kamery ciała (np. 0 dla /dev/video0)',
    )
    head_camera_device_arg = DeclareLaunchArgument(
        'head_camera_device',
        default_value='-1',
        description='Indeks urządzenia V4L2 dla kamery głowy; -1 wyłącza kamerę głowy i używa obrazu z kamery ciała',
    )

    camera_interface_node = Node(
        package='robomvp',
        executable='camera_interface',
        name='camera_interface',
        output='screen',
        parameters=[{
            'publish_rate': 10.0,
            'body_camera_device': LaunchConfiguration('body_camera_device'),
            'head_camera_device': LaunchConfiguration('head_camera_device'),
        }],
    )

    marker_detection_node = Node(
        package='robomvp',
        executable='marker_detection',
        name='marker_detection',
        output='screen',
        parameters=[{
            'marker_type': LaunchConfiguration('marker_type'),
        }],
    )

    marker_pose_estimator_node = Node(
        package='robomvp',
        executable='marker_pose_estimator',
        name='marker_pose_estimator',
        output='screen',
        parameters=[{
            'camera_config_path': camera_config,
            'marker_size': 0.1,
        }],
    )

    robomvp_main_node = Node(
        package='robomvp',
        executable='robomvp_main',
        name='robomvp_main',
        output='screen',
        parameters=[{
            'scene_config_path': scene_config,
            'step_period': 1.0,
            'network_interface': LaunchConfiguration('network_interface'),
            'require_robot_connection': LaunchConfiguration('require_robot_connection'),
        }],
    )

    return LaunchDescription([
        marker_type_arg,
        network_interface_arg,
        require_robot_connection_arg,
        body_camera_device_arg,
        head_camera_device_arg,
        camera_interface_node,
        marker_detection_node,
        marker_pose_estimator_node,
        robomvp_main_node,
    ])
