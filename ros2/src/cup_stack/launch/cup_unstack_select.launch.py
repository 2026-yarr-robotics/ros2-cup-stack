"""Launch click-selected cup pyramid unstacking with MoveItPy parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description()
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()
    )

    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("cup_stack"), "config", "moveit_py.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("nest_inc", default_value="0.0147"),
            Node(
                package="cup_stack",
                executable="cup_unstack_select",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {"nest_inc": LaunchConfiguration("nest_inc")},
                ],
            ),
        ]
    )
