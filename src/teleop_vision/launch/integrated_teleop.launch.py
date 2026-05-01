import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def load_yaml(path):
    with open(path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    camera_device = LaunchConfiguration('camera_device')

    moveit_config_dir = get_package_share_directory(
        'teleop_moveit2_config'
    )
    moveit_config = MoveItConfigsBuilder(
        'dual_piper',
        package_name='teleop_moveit2_config'
    ).to_moveit_configs()

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_config_dir,
                'launch',
                'demo.launch.py'
            )
        )
    )

    left_servo_config = os.path.join(
        moveit_config_dir,
        'config',
        'left_servo_config.yaml'
    )
    left_servo_params = {
        'moveit_servo': load_yaml(left_servo_config)
    }

    right_servo_config = os.path.join(
        moveit_config_dir,
        'config',
        'right_servo_config.yaml'
    )
    right_servo_params = {
        'moveit_servo': load_yaml(right_servo_config)
    }

    left_servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        namespace='left_servo_node',
        parameters=[
            moveit_config.to_dict(),
            left_servo_params,
        ],
        output='screen'
    )

    right_servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        namespace='right_servo_node',
        parameters=[
            moveit_config.to_dict(),
            right_servo_params,
        ],
        output='screen'
    )

    teleop = Node(
        package='teleop_vision',
        executable='teleop_mapper',
        name='teleop_mapper',
        output='screen',
        parameters=[{
            'camera_device': camera_device,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='Camera device path or numeric camera index for OpenCV.'
        ),
        moveit_demo,
        left_servo,
        right_servo,
        teleop
    ])
