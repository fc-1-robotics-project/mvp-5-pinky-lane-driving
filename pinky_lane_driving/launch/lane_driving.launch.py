"""Start the complete robot-local lane-driving stack in fail-closed mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Assemble bringup, perception, control, safety latch and watchdog."""
    model = PathJoinSubstitution([
        FindPackageShare('pinky_lane_driving'), 'models', 'best.pt'])
    bringup = PathJoinSubstitution([
        FindPackageShare('pinky_bringup'), 'launch', 'bringup_robot.launch.xml'])

    return LaunchDescription([
        DeclareLaunchArgument('start_robot_bringup', default_value='true'),
        DeclareLaunchArgument('robot_id', default_value='robot1'),
        DeclareLaunchArgument('start_robot_description', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument('start_motors', default_value='false'),
        DeclareLaunchArgument('start_battery', default_value='false'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('camera_role', default_value='viewfinder'),
        DeclareLaunchArgument('camera_frame_id', default_value='front_camera_link'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_pixel_format', default_value='RGB888'),
        DeclareLaunchArgument('camera_orientation', default_value='180'),
        DeclareLaunchArgument('camera_frame_duration_us', default_value='33333'),
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('pinky_bringup'), 'config',
                'pinky_camera.yaml',
            ]),
        ),
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=model),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('cpu_threads', default_value='1'),
        DeclareLaunchArgument('imgsz', default_value='448'),
        DeclareLaunchArgument('confidence', default_value='0.35'),
        DeclareLaunchArgument('perception_max_age_s', default_value='1.1'),
        DeclareLaunchArgument('start_control', default_value='false'),
        DeclareLaunchArgument('control_config_path', default_value=''),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('hardware_watchdog_confirmed', default_value='false'),
        DeclareLaunchArgument('command_timeout_s', default_value='0.2'),
        DeclareLaunchArgument(
            'lane_output_topic', default_value='cmd_vel_lane_candidate'),
        DeclareLaunchArgument('lane_start_enabled', default_value='false'),
        DeclareLaunchArgument('finish_topic', default_value='lane/finish'),
        DeclareLaunchArgument('max_source_age_s', default_value='1.1'),
        DeclareLaunchArgument('max_speed_mps', default_value='0.03'),
        DeclareLaunchArgument('max_omega_radps', default_value='0.6'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(bringup),
            condition=IfCondition(LaunchConfiguration('start_robot_bringup')),
            launch_arguments={
                'robot_id': LaunchConfiguration('robot_id'),
                'start_robot_description': LaunchConfiguration(
                    'start_robot_description'),
                'start_lidar': LaunchConfiguration('start_lidar'),
                'start_motors': LaunchConfiguration('start_motors'),
                'start_battery': LaunchConfiguration('start_battery'),
                'start_camera': LaunchConfiguration('start_camera'),
                'camera_index': LaunchConfiguration('camera_index'),
                'camera_role': LaunchConfiguration('camera_role'),
                'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                'camera_width': LaunchConfiguration('camera_width'),
                'camera_height': LaunchConfiguration('camera_height'),
                'camera_pixel_format': LaunchConfiguration('camera_pixel_format'),
                'camera_orientation': LaunchConfiguration('camera_orientation'),
                'camera_frame_duration_us': LaunchConfiguration(
                    'camera_frame_duration_us'),
                'camera_calibration_file': LaunchConfiguration(
                    'camera_calibration_file'),
            }.items(),
        ),
        Node(
            package='pinky_lane_driving',
            executable='lane_safety',
            name='lane_safety',
            output='screen',
            parameters=[{
                'start_enabled': ParameterValue(
                    LaunchConfiguration('lane_start_enabled'), value_type=bool),
            }],
        ),
        Node(
            package='pinky_lane_driving',
            executable='lane_mission_server',
            name='lane_mission_server',
            output='screen',
            parameters=[{
                'finish_topic': LaunchConfiguration('finish_topic'),
            }],
        ),
        Node(
            package='pinky_lane_driving',
            executable='lane_perception',
            name='lane_perception',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_perception')),
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'image_topic': 'camera/image_raw',
                'device': LaunchConfiguration('device'),
                'cpu_threads': ParameterValue(
                    LaunchConfiguration('cpu_threads'), value_type=int),
                'imgsz': ParameterValue(
                    LaunchConfiguration('imgsz'), value_type=int),
                'confidence': ParameterValue(
                    LaunchConfiguration('confidence'), value_type=float),
                'max_age_s': ParameterValue(
                    LaunchConfiguration('perception_max_age_s'), value_type=float),
            }],
        ),
        Node(
            package='pinky_lane_driving',
            executable='lane_control',
            name='lane_control',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_control')),
            parameters=[{
                'config_path': LaunchConfiguration('control_config_path'),
            }],
        ),
        Node(
            package='pinky_lane_driving',
            executable='lane_watchdog',
            name='lane_watchdog',
            output='screen',
            parameters=[{
                'dry_run': ParameterValue(
                    LaunchConfiguration('dry_run'), value_type=bool),
                'output_topic': LaunchConfiguration('lane_output_topic'),
                'hardware_watchdog_confirmed': ParameterValue(
                    LaunchConfiguration('hardware_watchdog_confirmed'),
                    value_type=bool),
                'command_timeout_s': ParameterValue(
                    LaunchConfiguration('command_timeout_s'), value_type=float),
                'max_source_age_s': ParameterValue(
                    LaunchConfiguration('max_source_age_s'), value_type=float),
                'max_speed_mps': ParameterValue(
                    LaunchConfiguration('max_speed_mps'), value_type=float),
                'max_omega_radps': ParameterValue(
                    LaunchConfiguration('max_omega_radps'), value_type=float),
            }],
        ),
    ])
