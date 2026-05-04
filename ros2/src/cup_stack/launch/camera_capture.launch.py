"""Launch the camera capture node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "save_dir",
                default_value="~/captures",
                description="Directory to save captured images",
            ),
            Node(
                package="cup_stack",
                executable="camera_capture",
                output="screen",
                parameters=[{"save_dir": LaunchConfiguration("save_dir")}],
            ),
        ]
    )
