import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_share = get_package_share_directory("teleop_description")
    moveit_share = get_package_share_directory("teleop_moveit_config")

    urdf_path = os.path.join(desc_share, "urdf", "dual_robot.urdf.xacro")
    srdf_path = os.path.join(moveit_share, "config", "teleop.srdf")
    kinematics_yaml = os.path.join(moveit_share, "config", "kinematics.yaml")
    joint_limits_yaml = os.path.join(moveit_share, "config", "joint_limits.yaml")
    controllers_yaml = os.path.join(moveit_share, "config", "moveit_controllers.yaml")

    with open(srdf_path, "r") as f:
        robot_description_semantic = f.read()

    robot_description = ParameterValue(
        Command(["xacro", urdf_path]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "robot_description_semantic": robot_description_semantic,
                "robot_description_kinematics": kinematics_yaml,
                "joint_limits": joint_limits_yaml,
                "moveit_simple_controller_manager": controllers_yaml,
                "use_sim_time": False,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", os.path.join(desc_share, "rviz", "teleop.rviz")],
        ),
    ])