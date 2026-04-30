import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    moveit_config_dir = get_package_share_directory(
        'teleop_moveit2_config'
    )

    left_servo_config = os.path.join(config_path, 'config', 'left_servo_config.yaml')
    right_servo_config = os.path.join(config_path, 'config', 'right_servo_config.yaml')


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

    right_servo_config = os.path.join(
        moveit_config_dir,
        'config',
        'right_servo_config.yaml'
    )

    left_servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        namespace='left_servo_node',
        parameters=[left_servo_config],
        output='screen'
    )

    right_servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        namespace='right_servo_node',
        parameters=[right_servo_config],
        output='screen'
    )

    teleop = Node(
        package='teleop_vision',
        executable='teleop_mapper',
        name='teleop_mapper',
        output='screen'
    )

    return LaunchDescription([
        moveit_demo,
        left_servo,
        right_servo,
        teleop
    ])
