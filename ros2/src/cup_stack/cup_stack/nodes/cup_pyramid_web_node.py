"""Web-triggered cup pyramid build using pixel coordinates from the dashboard."""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cup_stack.runtime import CupStackRuntime
from cup_stack.tasks.cup_pyramid import CupPyramidTask
from cup_stack.vision import CameraClickSelector


def main(args=None):
    rclpy.init(args=args)
    node = Node("cup_pyramid_web_node")
    node.declare_parameter("nest_inc", 0.0127)
    node.declare_parameter("pixel_x", -1)
    node.declare_parameter("pixel_y", -1)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        pixel_x = int(node.get_parameter("pixel_x").value)
        pixel_y = int(node.get_parameter("pixel_y").value)
        if pixel_x < 0 or pixel_y < 0:
            node.get_logger().error("Invalid pixel coordinates")
            return

        nest_inc = float(node.get_parameter("nest_inc").value)
        runtime = CupStackRuntime(node, "cup_pyramid_web_moveit_py")
        node.get_logger().info("[0] Moving HOME")
        if not runtime.try_move_home():
            return

        selector = CameraClickSelector(node, runtime)
        node.get_logger().info(
            f"Waiting for camera data (pixel_x={pixel_x}, pixel_y={pixel_y})…"
        )

        # Wait for camera frames to arrive
        import time

        for _ in range(100):
            if selector.ready:
                break
            time.sleep(0.1)

        if not selector.ready:
            node.get_logger().error("Camera data not available")
            return

        point = selector.pixel_to_base(pixel_x, pixel_y)
        if point is None:
            node.get_logger().error("Pixel-to-base conversion failed")
            return

        pick_x, pick_y, _ = point
        node.get_logger().info(
            f"Converted pixel({pixel_x},{pixel_y}) → "
            f"base({pick_x:.3f},{pick_y:.3f})"
        )

        task = CupPyramidTask(runtime, nest_inc=nest_inc)
        task.try_execute(pick_xy=(pick_x, pick_y), move_home=False)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
