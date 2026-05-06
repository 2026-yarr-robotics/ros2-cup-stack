"""ROS 2 entry point for the cup unstack task."""

import rclpy
from rclpy.node import Node

from cup_stack.runtime import CupStackRuntime
from cup_stack.tasks.cup_unstack import CupUnstackTask


def main(args=None):
    rclpy.init(args=args)
    node = Node("cup_unstack_node")
    node.declare_parameter("nest_inc", 0.0247)

    try:
        nest_inc = float(node.get_parameter("nest_inc").value)
        runtime = CupStackRuntime(node, "cup_unstack_moveit_py")
        task = CupUnstackTask(runtime, nest_inc=nest_inc)
        task.try_execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
