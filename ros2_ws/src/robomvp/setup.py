import os
from glob import glob
from os.path import join
from setuptools import setup

package_name = 'robomvp'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob('launch/*.py')),
        (
            join('share', package_name, 'config'),
            glob(join(os.path.dirname(__file__), '..', '..', '..', 'config', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboMVP Maintainer',
    maintainer_email='maintainer@robomvp.local',
    description='Minimalna aplikacja ROS2 MVP dla robota humanoidalnego Unitree G1 EDU',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_interface      = robomvp.camera_interface:main',
            'marker_detection      = robomvp.marker_detection:main',
            'marker_pose_estimator = robomvp.marker_pose_estimator:main',
            'robomvp_main          = robomvp.main_node:main',
            'robomvp_diagnostics   = robomvp.robomvp_diagnostics:main',
            'robomvp_odometry      = robomvp.robomvp_odometry:main',
            'robomvp_tf_publisher  = robomvp.robomvp_tf_publisher:main',
            'robomvp_teleop        = robomvp.robomvp_teleop:main',
        ],
    },
)
