from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'pinky_fleet_safety'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.*')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pl3',
    maintainer_email='kyung133851@pinklab.art',
    description='Fail-closed velocity gate for fleet-controlled Pinky robots.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'drive_mode_mux=pinky_fleet_safety.drive_mode_mux:main',
            'pose_reporter=pinky_fleet_safety.pose_reporter:main',
            'velocity_gate=pinky_fleet_safety.velocity_gate:main',
        ],
    },
)
