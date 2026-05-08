"""Cup unstack with camera-based pyramid center selection.

Two modes selected automatically by the pixel_x / pixel_y parameters:

  Local (default): pixel_x < 0
    Opens an OpenCV window.  User clicks the pyramid top/center to set
    the pyramid XY, then presses SPACE to execute.

  Web (pixel_x >= 0): pixel_x / pixel_y supplied via launch args
    Skips the OpenCV window; converts the pixel directly to a base-frame
    coordinate via depth image + hand-eye calibration, then executes.
"""

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cup_stack.config import MotionConfig
from cup_stack.runtime import CupStackRuntime
from cup_stack.tasks.cup_unstack import CupUnstackTask
from cup_stack.vision import CameraClickSelector


def main(args=None):
    rclpy.init(args=args)
    node = Node("cup_unstack_select_node")
    node.declare_parameter("nest_inc", 0.0127)
    node.declare_parameter("pixel_x", -1)
    node.declare_parameter("pixel_y", -1)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    selector = None
    try:
        pixel_x = int(node.get_parameter("pixel_x").value)
        pixel_y = int(node.get_parameter("pixel_y").value)
        web_mode = pixel_x >= 0 and pixel_y >= 0

        nest_inc = float(node.get_parameter("nest_inc").value)
        runtime = CupStackRuntime(node, "cup_unstack_select_moveit_py", MotionConfig())

        node.get_logger().info("[0] Moving HOME before selection")
        if not runtime.try_move_home():
            return

        selector = CameraClickSelector(node, runtime)

        if web_mode:
            # ── Web mode: pixel coords supplied by the dashboard ──
            node.get_logger().info(
                f"[Web] Waiting for camera frames (pixel={pixel_x},{pixel_y})…"
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

            pyramid_x, pyramid_y, _ = point

        else:
            # ── Local mode: OpenCV window for interactive selection ──
            node.get_logger().info("[Local] Click the pyramid center/top cup.")
            selected = selector.select_point(
                prompt="Click pyramid center/top cup. SPACE to start / ESC to cancel."
            )
            if selected is None:
                node.get_logger().warn("No coordinate selected; exiting")
                return
            pyramid_x, pyramid_y, _ = selected

        node.get_logger().info(
            f"pyramid_center=({pyramid_x:.3f},{pyramid_y:.3f})"
        )

        task = CupUnstackTask(runtime, nest_inc=nest_inc)
        done = threading.Event()

        task_thread = threading.Thread(
            target=lambda: (
                task.try_execute(pyramid_xy=(pyramid_x, pyramid_y)),
                done.set(),
            ),
            daemon=True,
        )
        task_thread.start()

        if not web_mode:
            selector.monitor(done)

        task_thread.join()

    finally:
        if selector is not None:
            selector.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
