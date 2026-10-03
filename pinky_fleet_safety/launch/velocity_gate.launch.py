"""Launch the robot-local fleet velocity gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the velocity-gate launch description."""
    robot_id = LaunchConfiguration('robot_id')
    config = PathJoinSubstitution([
        FindPackageShare('pinky_fleet_safety'),
        'config',
        'velocity_gate.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot'),
        Node(
            package='pinky_fleet_safety',
            executable='drive_mode_mux',
            name='drive_mode_mux',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='pinky_fleet_safety',
            executable='velocity_gate',
            name='fleet_velocity_gate',
            output='screen',
            parameters=[config, {'robot_id': robot_id}],
        ),
    ])
