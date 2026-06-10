"""Simulation backend for the RG gripper: ROS topics instead of Modbus.

Drop-in replacement for onrobot.RG, selected with
CUP_STACK_GRIPPER_BACKEND=sim (default: modbus → real hardware). Used by the
Isaac Sim digital twin (yarr-isaac-playground): commands go out on
/gripper/target_width (Float32, mm); the achieved width comes back on
/gripper/width, published by the playground's gripper_bridge — the same topic
the FastAPI server already consumes, so the dashboard works unchanged.
"""
from __future__ import annotations

import os
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32

TARGET_TOPIC = "/gripper/target_width"
WIDTH_TOPIC = "/gripper/width"


def sim_backend_enabled() -> bool:
    return os.environ.get("CUP_STACK_GRIPPER_BACKEND", "modbus") == "sim"


class SimRG:
    """onrobot.RG interface against the Isaac digital twin.

    Same unit conventions as RG: move_gripper() takes width/force in 1/10 mm
    / 1/10 N (open_width 900 = 90 mm); get_width() returns millimetres.
    """

    def __init__(self, gripper, ip, port):
        if gripper not in ["rg2", "rg6"]:
            print("Please specify either rg2 or rg6.")
            return
        self.gripper = gripper
        self.max_width = 1100 if gripper == "rg2" else 1600
        self.max_force = 400 if gripper == "rg2" else 1200

        self._width_mm: float | None = None
        self._node = rclpy.create_node(f"sim_gripper_io_{os.getpid()}")
        self._pub = self._node.create_publisher(Float32, TARGET_TOPIC, 10)
        self._node.create_subscription(Float32, WIDTH_TOPIC, self._on_width, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()
        print(f"SimRG ({gripper}): publishing {TARGET_TOPIC}, reading {WIDTH_TOPIC}")

    def _on_width(self, msg: Float32) -> None:
        self._width_mm = float(msg.data)

    def open_connection(self):
        pass

    def close_connection(self):
        self._executor.shutdown()
        self._node.destroy_node()

    def get_width(self):
        """Achieved width in millimetres (last commanded if sim is silent)."""
        if self._width_mm is None:
            return 0.0
        return self._width_mm

    def get_status(self):
        return [0] * 16

    def move_gripper(self, width_val, force_val=400):
        """width_val in 1/10 mm, matching RG (force is ignored in sim)."""
        self._pub.publish(Float32(data=width_val / 10.0))


def make_gripper(name, ip, port):
    """Factory: RG (Modbus) or SimRG per CUP_STACK_GRIPPER_BACKEND."""
    if sim_backend_enabled():
        return SimRG(name, ip, port)
    from .onrobot import RG
    return RG(name, ip, port)
