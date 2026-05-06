"""ROS 2 entry point for click-selected cup pyramid unstacking."""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cup_stack.runtime import CupStackRuntime
from cup_stack.tasks.cup_unstack import CupUnstackTask
from cup_stack.vision import CameraClickSelector


def main(args=None):
    rclpy.init(args=args)
    node = Node("cup_unstack_select_node")
    node.declare_parameter("nest_inc", 0.0127)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        nest_inc = float(node.get_parameter("nest_inc").value)
        runtime = CupStackRuntime(node, "cup_unstack_select_moveit_py")
        node.get_logger().info("[0] Moving HOME before camera selection")
        if not runtime.try_move_home():
            return

        selector = CameraClickSelector(node, runtime)
        selected = selector.select_point(
            prompt="Click pyramid center/top cup. ESC to cancel."
        )
        if selected is None:
            node.get_logger().warn("No coordinate selected; exiting")
            return

        pyramid_x, pyramid_y, _ = selected
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
        selector.monitor(done)
        task_thread.join()
    finally:
        selector.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
