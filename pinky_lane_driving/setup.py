# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
from glob import glob
import os

from setuptools import setup

setup(
    name='pinky_lane_driving', version='0.1.0',
    packages=['pinky_lane_driving'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/pinky_lane_driving']),
                ('share/pinky_lane_driving', ['package.xml']),
                (os.path.join('share', 'pinky_lane_driving', 'launch'),
                 glob(os.path.join('launch', '*.launch.py'))),
                (os.path.join('share', 'pinky_lane_driving', 'config'),
                 glob(os.path.join('config', '*.yaml'))
                 + glob(os.path.join('config', '*.json'))),
                (os.path.join('share', 'pinky_lane_driving', 'models'),
                 glob(os.path.join('models', '*.pt'))),
                ('share/pinky_lane_driving', ['requirements.txt'])],
    install_requires=['setuptools'],
    maintainer='SeungHoon Jeong', maintainer_email='jsh0116@users.noreply.github.com',
    description='Calibrated lane algorithms and fail-safe ROS adapters', license='Apache-2.0',
    entry_points={'console_scripts': ['lane_watchdog = pinky_lane_driving.ros_watchdog:main',
                                      'lane_control = pinky_lane_driving.ros_control:main',
                                      'lane_mission_server = '
                                      'pinky_lane_driving.lane_mission_server:main',
                                      'lane_perception = pinky_lane_driving.ros_perception:main',
                                      'lane_safety = pinky_lane_driving.ros_safety:main',
                                      'lane_calibrate_ground = '
                                      'pinky_lane_driving.ground_calibration:main']},
)
