"""ROS 2 node: move robot end-effector + gripper control via ROS services."""

import rclpy
from cup_stack.config import GripperConfig, MotionConfig
from cup_stack.runtime import CupStackRuntime
from cup_stack_interfaces.srv import MoveCartesian
from rclpy.node import Node

try:
    from cup_stack_interfaces.srv import GripperControl
    _GRIPPER_SRV_AVAILABLE = True
except ImportError:
    _GRIPPER_SRV_AVAILABLE = False


def main(args=None):
    rclpy.init(args=args)

    node = Node("move_cartesian_node")

    # Runtime singleton
    runtime = None
    try:
        runtime = CupStackRuntime(
            node,
            "move_cartesian",
            MotionConfig(),
        )
    except Exception as e:
        node.get_logger().error(f"Failed to initialize runtime: {e}")
        rclpy.shutdown()
        return

    def handle_move_cartesian(request: MoveCartesian.Request, response: MoveCartesian.Response):
        """Handle move_cartesian service request."""
        x, y, z = request.x, request.y, request.z
        mode = request.mode

        node.get_logger().info(f"Move request: x={x:.3f}, y={y:.3f}, z={z:.3f}, mode={mode}")

        # Get current pose for relative mode
        if mode == "relative":
            try:
                ee_matrix = runtime.current_ee_matrix()
                current_x = ee_matrix[0, 3]
                current_y = ee_matrix[1, 3]
                current_z = ee_matrix[2, 3]
                x += current_x
                y += current_y
                z += current_z
                node.get_logger().info(f"Relative: current=({current_x:.3f}, {current_y:.3f}, {current_z:.3f}), target=({x:.3f}, {y:.3f}, {z:.3f})")
            except Exception as e:
                node.get_logger().error(f"Failed to get current pose: {e}")
                response.success = False
                response.message = f"Failed to get current pose: {e}"
                return response

        # Execute move
        try:
            success = runtime.try_move_to_pose(x, y, z, safe_z_min=0.25, lin=True, strict=False)
            if success:
                response.success = True
                response.message = f"Moved to ({x:.3f}, {y:.3f}, {z:.3f})"
                node.get_logger().info(f"Move successful: ({x:.3f}, {y:.3f}, {z:.3f})")
            else:
                response.success = False
                response.message = "Planning failed"
                node.get_logger().error("Move failed: planning failed")
        except Exception as e:
            response.success = False
            response.message = str(e)
            node.get_logger().error(f"Move failed: {e}")

        return response

    def handle_gripper_control(request: GripperControl.Request, response: GripperControl.Response):
        cmd = request.command.strip().lower()
        node.get_logger().info(f"Gripper command: {cmd}")
        try:
            if cmd == "open":
                runtime.try_open_gripper(GripperConfig().open_sleep_sec)
                response.success = True
                response.message = "Gripper opened"
            elif cmd == "close":
                runtime.try_grip_cup(GripperConfig().grip_sleep_sec)
                response.success = True
                response.message = "Gripper closed"
            else:
                response.success = False
                response.message = f"Unknown command: {cmd!r}. Use 'open' or 'close'."
        except Exception as e:
            response.success = False
            response.message = str(e)
            node.get_logger().error(f"Gripper failed: {e}")
        return response

    # Create services
    svc_move = node.create_service(MoveCartesian, "/move_cartesian", handle_move_cartesian)
    svc_grip = None
    if _GRIPPER_SRV_AVAILABLE:
        svc_grip = node.create_service(GripperControl, "/gripper_control", handle_gripper_control)
        node.get_logger().info("move_cartesian + gripper_control services ready")
    else:
        node.get_logger().warn("GripperControl.srv not built — /gripper_control unavailable. Run colcon build.")
        node.get_logger().info("move_cartesian service ready")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        svc_move.destroy()
        if svc_grip:
            svc_grip.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
