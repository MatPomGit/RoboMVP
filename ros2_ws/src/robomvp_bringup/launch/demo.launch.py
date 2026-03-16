"""Plik uruchomieniowy dla demonstracji systemu RoboMVP.

Uruchamia węzły w kolejności:
1. camera_interface - publikuje obrazy kamer
2. marker_detection - wykrywa markery w obrazach
3. marker_pose_estimator - oblicza pozycje 3D markerów
4. robomvp_main - główny węzeł sterujący

Tryb wybieramy parametrem launch: mode:=demo_mode lub mode:=robot_mode
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generuje opis uruchomienia dla systemu RoboMVP."""

    # Ścieżka do katalogu konfiguracji
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__))
        ))),
        'config'
    )
    scene_config = os.path.join(config_dir, 'scene.yaml')
    camera_config = os.path.join(config_dir, 'camera.yaml')
    test_images = os.path.join(
        os.path.dirname(config_dir), 'data', 'test_images'
    )

    # Deklaracja argumentów uruchomienia
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='demo_mode',
        description='Tryb pracy: demo_mode (obrazy testowe) lub robot_mode (sprzęt)',
    )
    marker_type_arg = DeclareLaunchArgument(
        'marker_type',
        default_value='apriltag',
        description='Typ markera: apriltag lub qr',
    )

    # Węzeł interfejsu kamery
    camera_interface_node = Node(
        package='robomvp',
        executable='camera_interface',
        name='camera_interface',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'test_images_path': test_images,
            'publish_rate': 10.0,
        }],
    )

    # Węzeł detekcji markerów
    marker_detection_node = Node(
        package='robomvp',
        executable='marker_detection',
        name='marker_detection',
        output='screen',
        parameters=[{
            'marker_type': LaunchConfiguration('marker_type'),
        }],
    )

    # Węzeł estymacji pozy markerów
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

    # Główny węzeł sterujący
    robomvp_main_node = Node(
        package='robomvp',
        executable='robomvp_main',
        name='robomvp_main',
        output='screen',
        parameters=[{
            'scene_config_path': scene_config,
            'mode': LaunchConfiguration('mode'),
            'step_period': 1.0,
        }],
    )

    return LaunchDescription([
        mode_arg,
        marker_type_arg,
        camera_interface_node,
        marker_detection_node,
        marker_pose_estimator_node,
        robomvp_main_node,
    ])
