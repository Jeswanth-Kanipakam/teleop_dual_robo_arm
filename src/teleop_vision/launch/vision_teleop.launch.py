from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_device = LaunchConfiguration('camera_device')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='Camera device path or numeric camera index for OpenCV.'
        ),
        Node(
            package='teleop_vision',
            executable='teleop_mapper',
            name='teleop_mapper',
            output='screen',
            parameters=[{
                'camera_device': camera_device,
            }]
        )
    ])
