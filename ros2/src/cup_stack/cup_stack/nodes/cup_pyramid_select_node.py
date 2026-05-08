"""Web-triggered cup pyramid: pixel click → depth → pick/place → execute."""

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cup_stack.config import CupStackConfig, MotionConfig
from cup_stack.runtime import CupStackRuntime
from cup_stack.tasks.cup_pyramid import CupPyramidTask
from cup_stack.vision import CameraClickSelector


def main(args=None):
    rclpy.init(args=args)
    node = Node("cup_pyramid_select_node")
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
            node.get_logger().error(
                "pixel_x / pixel_y not set — launch with pixel_x:=N pixel_y:=M"
            )
            return

        nest_inc = float(node.get_parameter("nest_inc").value)
        config = CupStackConfig()
        runtime = CupStackRuntime(node, "cup_pyramid_select_moveit_py", MotionConfig())

        node.get_logger().info("[0] Moving HOME")
        if not runtime.try_move_home():
            return
        runtime.sleep(config.home_sleep_sec)

        selector = CameraClickSelector(node, runtime)
        node.get_logger().info(
            f"Waiting for camera frames (pixel={pixel_x},{pixel_y})…"
        )
        for _ in range(100):
            if selector.ready:
                break
            time.sleep(0.1)

        if not selector.ready:
            node.get_logger().error("Camera not available after 10 s")
            return

        point = selector.pixel_to_base(pixel_x, pixel_y)
        if point is None:
            node.get_logger().error("Pixel-to-base conversion failed")
            return

        pick_x, pick_y, _ = point
        # Place center: X offset forward, Y offset to side (away from nested stack)
        place_xy = (
            pick_x + config.place_x_offset,
            pick_y - 1.5 * config.cup_spacing,
        )
        node.get_logger().info(
            f"pick=({pick_x:.3f},{pick_y:.3f})  "
            f"pyramid_center=({place_xy[0]:.3f},{place_xy[1]:.3f})"
        )

        task = CupPyramidTask(runtime, nest_inc=nest_inc, config=config)
        task.try_execute(
            pick_xy=(pick_x, pick_y),
            place_xy=place_xy,
            move_home=False,
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
