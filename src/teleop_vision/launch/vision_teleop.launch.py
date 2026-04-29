from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_vision',
            executable='teleop_mapper',
            name='teleop_mapper',
            output='screen',
            parameters=[{}]
        )
    ])