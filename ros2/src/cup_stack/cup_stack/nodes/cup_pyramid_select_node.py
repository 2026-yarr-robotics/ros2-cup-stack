"""ROS 2 entry point for click-selected cup pyramid building."""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cup_stack.runtime import CupStackRuntime
from cup_stack.tasks.cup_pyramid import CupPyramidTask
from cup_stack.vision import CameraClickSelector


def main(args=None):
    rclpy.init(args=args)
    node = Node("cup_pyramid_select_node")
    node.declare_parameter("nest_inc", 0.0127)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        nest_inc = float(node.get_parameter("nest_inc").value)
        runtime = CupStackRuntime(node, "cup_pyramid_select_moveit_py")
        node.get_logger().info("[0] Moving HOME before camera selection")
        if not runtime.try_move_home():
            return

        selector = CameraClickSelector(node, runtime)
        selected = selector.select_point()
        if selected is None:
            node.get_logger().warn("No coordinate selected; exiting")
            return

        pick_x, pick_y, _ = selected
        task = CupPyramidTask(runtime, nest_inc=nest_inc)
        task.try_execute(pick_xy=(pick_x, pick_y), move_home=False)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
