"""ROS 2 node: standalone gripper control via Modbus TCP.

Does NOT create a MoveItPy instance, so it can run alongside cup_pyramid / cup_unstack
without conflicting over the planning scene monitor.
Registers /gripper_control service (same interface as move_cartesian_node).
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from cup_stack.config import CupStackConfig, GripperConfig
from cup_stack.onrobot import RG

try:
    from cup_stack_interfaces.srv import GripperControl
    _SRV_AVAILABLE = True
except ImportError:
    _SRV_AVAILABLE = False


def main(args=None):
    rclpy.init(args=args)
    node = Node("gripper_node")

    gripper_cfg = GripperConfig()
    cup_cfg = CupStackConfig()

    try:
        gripper: RG | None = RG(
            gripper_cfg.name,
            gripper_cfg.toolcharger_ip,
            gripper_cfg.toolcharger_port,
        )
        node.get_logger().info(
            f"Gripper connected: {gripper_cfg.name} @ {gripper_cfg.toolcharger_ip}:{gripper_cfg.toolcharger_port}"
        )
    except Exception as exc:
        node.get_logger().warning(f"Gripper not connected ({exc}) — service will return errors")
        gripper = None

    if not _SRV_AVAILABLE:
        node.get_logger().error("GripperControl.srv not built — run colcon build")
        node.destroy_node()
        rclpy.shutdown()
        return

    def handle_gripper(
        request: GripperControl.Request,
        response: GripperControl.Response,
    ) -> GripperControl.Response:
        if gripper is None:
            response.success = False
            response.message = "Gripper not connected (192.168.1.1:502)"
            return response

        cmd = request.command.strip().lower()
        try:
            if cmd == "open":
                gripper.move_gripper(gripper_cfg.open_width, gripper_cfg.force)
                time.sleep(cup_cfg.open_sleep_sec)
                response.success = True
                response.message = "Gripper opened"
            elif cmd == "close":
                gripper.move_gripper(gripper_cfg.grip_width, gripper_cfg.force)
                time.sleep(cup_cfg.grip_sleep_sec)
                response.success = True
                response.message = "Gripper closed"
            else:
                response.success = False
                response.message = f"Unknown command: {cmd!r}. Use 'open' or 'close'."
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            node.get_logger().error(f"Gripper error: {exc}")
        return response

    svc = node.create_service(GripperControl, "/gripper_control", handle_gripper)
    hw = "connected" if gripper is not None else "not connected (hardware unavailable)"
    node.get_logger().info(f"gripper_control service ready (gripper {hw})")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        svc.destroy()
        if gripper is not None:
            try:
                gripper.close_connection()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
