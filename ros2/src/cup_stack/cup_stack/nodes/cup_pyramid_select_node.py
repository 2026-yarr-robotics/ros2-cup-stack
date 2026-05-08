"""ROS 2 entry point for click-selected cup pyramid building."""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cup_stack.config import CupStackConfig
from cup_stack.runtime import CupStackRuntime
from cup_stack.tasks.cup_pyramid import CupPyramidTask
from cup_stack.vision import CameraClickSelector


def main(args=None):
    rclpy.init(args=args)
    node = Node("cup_pyramid_select_node")
    node.declare_parameter("nest_inc", 0.012)
    node.declare_parameter("place_y_offset", CupStackConfig().cup_spacing)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        nest_inc = float(node.get_parameter("nest_inc").value)
        place_y_offset = float(node.get_parameter("place_y_offset").value)
        runtime = CupStackRuntime(node, "cup_pyramid_select_moveit_py")
        node.get_logger().info("[0] Moving HOME")
        if not runtime.try_move_home():
            return
        home_x, home_y = runtime.current_ee_xy()
        place_xy = (home_x, home_y + place_y_offset)
        node.get_logger().info(
            f"HOME: ({home_x:.3f}, {home_y:.3f})  "
            f"Pyramid center: ({place_xy[0]:.3f}, {place_xy[1]:.3f})"
        )

        selector = CameraClickSelector(node, runtime)
        selected = selector.select_point()
        if selected is None:
            node.get_logger().warn("No coordinate selected; exiting")
            return

        pick_x, pick_y, _ = selected
        task = CupPyramidTask(runtime, nest_inc=nest_inc)

        done = threading.Event()
        task_thread = threading.Thread(
            target=lambda: (
                task.try_execute(pick_xy=(pick_x, pick_y), place_xy=place_xy, move_home=False),
                done.set(),
            ),
            daemon=True,
        )
        task_thread.start()
        selector.monitor(done)
        task_thread.join()
    finally:
        selector.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
