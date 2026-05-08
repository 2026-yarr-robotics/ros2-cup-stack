"""Launch file for the standalone gripper control node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cup_stack",
            executable="gripper_node",
            name="gripper_node",
            output="screen",
        ),
    ])
