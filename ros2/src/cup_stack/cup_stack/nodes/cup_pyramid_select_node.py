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

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        nest_inc = float(node.get_parameter("nest_inc").value)
        config = CupStackConfig()
        runtime = CupStackRuntime(node, "cup_pyramid_select_moveit_py")
        node.get_logger().info("[0] Moving HOME")
        if not runtime.try_move_home():
            return
        home_x, home_y = runtime.current_ee_xy()
        node.get_logger().info(f"HOME: ({home_x:.3f}, {home_y:.3f})")

        selector = CameraClickSelector(node, runtime)
        selected = selector.select_point()
        if selected is None:
            node.get_logger().warn("No coordinate selected; exiting")
            return

        pick_x, pick_y, _ = selected
        # place 중앙: pick 기준 x+offset, y-(1.5 × cup_spacing)
        # y 오프셋 = 1.5 × 0.079 = 0.1185m → 최근접 cycle(3번) 이격 0.107m > 컵직경 0.076m
        place_xy = (pick_x + config.place_x_offset, pick_y - 1.5 * config.cup_spacing)
        node.get_logger().info(
            f"Pick: ({pick_x:.3f}, {pick_y:.3f})  "
            f"Pyramid center: ({place_xy[0]:.3f}, {place_xy[1]:.3f})"
        )
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
