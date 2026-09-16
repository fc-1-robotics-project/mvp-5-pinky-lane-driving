# Copyright 2026 SeungHoon Jeong
# SPDX-License-Identifier: Apache-2.0
from setuptools import setup

setup(
    name='pinky_lane_driving', version='0.1.0',
    packages=['pinky_lane_driving'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/pinky_lane_driving']),
                ('share/pinky_lane_driving', ['package.xml'])],
    install_requires=['setuptools'], tests_require=['pytest'],
    maintainer='SeungHoon Jeong', maintainer_email='jsh0116@users.noreply.github.com',
    description='Calibrated lane algorithms and fail-safe ROS adapters', license='Apache-2.0',
    entry_points={'console_scripts': ['lane_watchdog = pinky_lane_driving.ros_watchdog:main',
                                      'lane_control = pinky_lane_driving.ros_control:main',
                                      'lane_perception = pinky_lane_driving.ros_perception:main']},
)
