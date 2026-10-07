"""Launch live lane perception with the packaged segmentation model."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start lane perception against the Pinky front-camera topic."""
    default_model = PathJoinSubstitution([
        FindPackageShare('pinky_lane_driving'),
        'models',
        'best.pt',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=default_model),
        DeclareLaunchArgument('image_topic', default_value='camera/image_raw'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('confidence', default_value='0.25'),
        DeclareLaunchArgument('max_age_s', default_value='0.3'),
        Node(
            package='pinky_lane_driving',
            executable='lane_perception',
            name='lane_perception',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'image_topic': LaunchConfiguration('image_topic'),
                'device': LaunchConfiguration('device'),
                'imgsz': ParameterValue(
                    LaunchConfiguration('imgsz'), value_type=int),
                'confidence': ParameterValue(
                    LaunchConfiguration('confidence'), value_type=float),
                'max_age_s': ParameterValue(
                    LaunchConfiguration('max_age_s'), value_type=float),
            }],
        ),
    ])
