"""Task that traces a scan line using joint-space moves only.

Sequence
  [0] PTP : 초기 위치 → pos1  (joint-space)
  [1] PTP : pos1      → pos2  (joint-space)
  [2] PTP : pos2      → 초기 위치  (joint-space)
"""

import math

import numpy as np
from moveit.core.robot_state import RobotState

from cup_stack.config import ScanConfig
from cup_stack.runtime import CupStackRuntime


class ScanTask:

    def __init__(
        self,
        runtime: CupStackRuntime,
        scan_config: ScanConfig | None = None,
    ) -> None:
        self.runtime = runtime
        self.cfg = scan_config or ScanConfig()
        self.logger = runtime.logger

    def _current_joints_rad(self) -> list[float]:
        monitor = self.runtime.robot.get_planning_scene_monitor()
        with monitor.read_only() as scene:
            positions = list(
                scene.current_state.get_joint_group_positions(
                    self.runtime.motion.group_name
                )
            )
        return positions

    def _log_joints(self, label: str, joints_rad: list[float]) -> None:
        parts = "  ".join(
            f"J{i}={math.degrees(r):.2f}°" for i, r in enumerate(joints_rad, 1)
        )
        self.logger.info(f"  {label}: {parts}")

    def _ptp(self, label: str, joints_rad: list[float]) -> bool:
        """joint-space PTP 이동. 실패 시 OMPL로 재시도."""
        rt = self.runtime
        self._log_joints(f"목표({label})", joints_rad)

        goal_state = RobotState(rt.robot_model)
        goal_state.set_joint_group_positions(rt.motion.group_name, joints_rad)
        goal_state.update()

        rt.arm.set_start_state_to_current_state()
        rt.arm.set_goal_state(robot_state=goal_state)
        plan_result = rt.arm.plan(parameters=rt.ptp_params)

        if not plan_result:
            self.logger.warn(f"  {label} PTP 실패 — OMPL 재시도")
            rt.arm.set_start_state_to_current_state()
            rt.arm.set_goal_state(robot_state=goal_state)
            plan_result = rt.arm.plan(parameters=rt.ompl_params)

        if not plan_result:
            self.logger.error(f"  {label} 계획 실패")
            return False

        rt.robot.execute(
            group_name=rt.motion.group_name,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        arrived = self._current_joints_rad()
        self._log_joints("도달", arrived)
        return True

    def try_execute(self) -> bool:
        self.logger.info("=== Scan task start ===")

        # 시작 전 현재 joint 상태 저장
        start_joints = self._current_joints_rad()
        self._log_joints("초기 joints", start_joints)

        # [0] PTP: 초기 위치 → pos1
        self.logger.info("[0] PTP → pos1")
        if not self._ptp("pos1", self.cfg.pos1_joints_rad):
            return False

        # [1] PTP: pos1 → pos2
        self.logger.info("[1] PTP → pos2")
        if not self._ptp("pos2", self.cfg.pos2_joints_rad):
            return False

        # [2] PTP: pos2 → 초기 위치
        self.logger.info("[2] PTP → 초기 위치")
        if not self._ptp("초기 위치", start_joints):
            return False

        self.logger.info("=== Scan task complete ===")
        return True
