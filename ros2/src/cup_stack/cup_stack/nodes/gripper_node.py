"""ROS 2 node: standalone gripper control via Modbus TCP.

Does NOT create a MoveItPy instance, so it can run alongside cup_pyramid / cup_unstack
without conflicting over the planning scene monitor.
Registers /gripper_control service (cup_stack_interfaces/srv/GripperControl).
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from cup_stack.config import CupStackConfig, GripperConfig
from cup_stack.onrobot_sim import make_gripper, sim_backend_enabled

# Real-time width telemetry: the server subscribes to this and forwards it
# to the dashboard. RG.get_width() already returns millimetres.
WIDTH_TOPIC = "/gripper/width"
WIDTH_PUBLISH_HZ = 5.0

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
        gripper = make_gripper(
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

    # Periodically read and publish the live finger width (mm). The default
    # single-threaded executor serialises this with handle_gripper, so the
    # shared Modbus client needs no extra locking. Read failures (hardware
    # down / mid-motion) are skipped so the server's staleness check fires.
    # Sim backend: the Isaac gripper_bridge already publishes /gripper/width —
    # republishing here would duplicate the topic, so the timer is skipped.
    if sim_backend_enabled():
        node.get_logger().info(
            f"sim gripper backend — {WIDTH_TOPIC} is published by Isaac, skipping width timer"
        )
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            svc.destroy()
            node.destroy_node()
            rclpy.shutdown()
        return

    width_pub = node.create_publisher(Float32, WIDTH_TOPIC, 10)
    _read_warned = False

    def publish_width() -> None:
        nonlocal _read_warned
        if gripper is None:
            return
        try:
            width_mm = float(gripper.get_width())
        except Exception as exc:
            if not _read_warned:
                node.get_logger().warning(f"Gripper width read failed: {exc}")
                _read_warned = True
            return
        _read_warned = False
        width_pub.publish(Float32(data=width_mm))

    width_timer = node.create_timer(1.0 / WIDTH_PUBLISH_HZ, publish_width)
    node.get_logger().info(
        f"publishing gripper width on {WIDTH_TOPIC} at {WIDTH_PUBLISH_HZ:.0f} Hz"
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        width_timer.cancel()
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
