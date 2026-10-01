"""Start the robot-local Nav2 and lane-driving system together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Assemble hardware, safety gates, lane driving, and Nav2."""
    lane_launch = PathJoinSubstitution([
        FindPackageShare('pinky_lane_driving'),
        'launch',
        'lane_driving.launch.py',
    ])
    nav_launch = PathJoinSubstitution([
        FindPackageShare('pinky_navigation'),
        'launch',
        'bringup_launch.xml',
    ])
    default_map = PathJoinSubstitution([
        FindPackageShare('pinky_navigation'),
        'map',
        'my_map.yaml',
    ])
    default_nav_params = PathJoinSubstitution([
        FindPackageShare('pinky_navigation'),
        'params',
        'nav2_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value=default_nav_params,
        ),
        DeclareLaunchArgument('start_nav2', default_value='true'),
        DeclareLaunchArgument('start_robot_description', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_motors', default_value='false'),
        DeclareLaunchArgument('start_battery', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('camera_role', default_value='viewfinder'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument(
            'camera_pixel_format',
            default_value='RGB888',
        ),
        DeclareLaunchArgument('camera_orientation', default_value='180'),
        DeclareLaunchArgument(
            'camera_frame_duration_us',
            default_value='33333',
        ),
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('perception_device', default_value='cpu'),
        DeclareLaunchArgument('perception_cpu_threads', default_value='1'),
        DeclareLaunchArgument('perception_imgsz', default_value='448'),
        DeclareLaunchArgument('perception_confidence', default_value='0.35'),
        DeclareLaunchArgument('perception_max_age_s', default_value='1.1'),
        DeclareLaunchArgument('lane_max_source_age_s', default_value='1.1'),
        DeclareLaunchArgument('start_lane_control', default_value='false'),
        DeclareLaunchArgument('lane_control_config', default_value=''),
        DeclareLaunchArgument('lane_dry_run', default_value='true'),
        DeclareLaunchArgument(
            'hardware_watchdog_confirmed',
            default_value='false',
        ),
        DeclareLaunchArgument('lane_start_enabled', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lane_launch),
            launch_arguments={
                'robot_id': LaunchConfiguration('robot_id'),
                'start_robot_bringup': 'true',
                'start_robot_description': LaunchConfiguration(
                    'start_robot_description',
                ),
                'start_lidar': LaunchConfiguration('start_lidar'),
                'start_motors': LaunchConfiguration('start_motors'),
                'start_battery': LaunchConfiguration('start_battery'),
                'start_camera': LaunchConfiguration('start_camera'),
                'camera_index': LaunchConfiguration('camera_index'),
                'camera_role': LaunchConfiguration('camera_role'),
                'camera_width': LaunchConfiguration('camera_width'),
                'camera_height': LaunchConfiguration('camera_height'),
                'camera_pixel_format': LaunchConfiguration(
                    'camera_pixel_format',
                ),
                'camera_orientation': LaunchConfiguration(
                    'camera_orientation',
                ),
                'camera_frame_duration_us': LaunchConfiguration(
                    'camera_frame_duration_us',
                ),
                'start_perception': LaunchConfiguration('start_perception'),
                'device': LaunchConfiguration('perception_device'),
                'cpu_threads': LaunchConfiguration('perception_cpu_threads'),
                'imgsz': LaunchConfiguration('perception_imgsz'),
                'confidence': LaunchConfiguration('perception_confidence'),
                'perception_max_age_s': LaunchConfiguration(
                    'perception_max_age_s',
                ),
                'max_source_age_s': LaunchConfiguration('lane_max_source_age_s'),
                'start_control': LaunchConfiguration('start_lane_control'),
                'control_config_path': LaunchConfiguration(
                    'lane_control_config',
                ),
                'dry_run': LaunchConfiguration('lane_dry_run'),
                'hardware_watchdog_confirmed': LaunchConfiguration(
                    'hardware_watchdog_confirmed',
                ),
                'lane_start_enabled': LaunchConfiguration(
                    'lane_start_enabled',
                ),
            }.items(),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(nav_launch),
            condition=IfCondition(LaunchConfiguration('start_nav2')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('nav_params_file'),
                'autostart': 'true',
            }.items(),
        ),
    ])
