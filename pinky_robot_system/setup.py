from glob import glob
import os

from setuptools import setup


package_name = 'pinky_robot_system'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SeungHoon Jeong',
    maintainer_email='jsh0116@users.noreply.github.com',
    description='Integrated real-robot Nav2 and lane-driving bringup',
    license='Apache-2.0',
)
