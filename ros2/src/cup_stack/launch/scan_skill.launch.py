"""Launch the scan-skill controller.

This controller only shells out to the existing scan node, so it
needs no MoveItPy parameters here — the spawned
``scan.launch.py`` supplies them.  Bring up move_group first, exactly
as for the plain ``scan`` launch.  Overrides
(launch_file/launch_package/timeout_sec) are node parameters; pass
them with ``--ros-args -p name:=value``.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the scan-skill controller."""

    return LaunchDescription(
        [
            Node(
                package="cup_stack",
                executable="scan_skill",
                output="screen",
            ),
        ]
    )
