"""Controller node: run the scan step as a skill.

The skill itself just executes the existing ``scan`` node in its own
process, so this controller needs no MoveItPy bringup.  It exists so
scan can be sequenced uniformly alongside the other skills.
"""

import rclpy
from rclpy.node import Node

from cup_stack.skills.scan_skill import ScanSkill


def main(args=None):
    """Run the scan skill once and report the result."""

    rclpy.init(args=args)
    node = Node("scan_skill_node")
    node.declare_parameter("launch_file", "scan.launch.py")
    node.declare_parameter("launch_package", "cup_stack")
    node.declare_parameter("timeout_sec", 180.0)

    try:
        skill = ScanSkill(
            node.get_logger(),
            launch_package=str(
                node.get_parameter("launch_package").value
            ),
            launch_file=str(node.get_parameter("launch_file").value),
            timeout_sec=float(
                node.get_parameter("timeout_sec").value
            ),
        )
        node.get_logger().info(skill.describe())
        ok = skill.execute()
        node.get_logger().info(
            f"=== scan skill {'OK' if ok else 'FAILED'} ==="
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
