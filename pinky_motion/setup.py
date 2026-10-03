from setuptools import find_packages, setup


package_name = 'pinky_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seunghoon',
    maintainer_email='seunghoon7561@gmail.com',
    description="Publish Pinky's current map-frame pose.",
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_reporter=pinky_motion.pose_reporter:main',
            'pose_reporter_sh=pinky_motion.pose_reporter_sh:main',
            'pose_monitor=pinky_motion.pose_monitor:main',
            'pose_monitor_sh=pinky_motion.pose_monitor_sh:main',
        ],
    },
)
