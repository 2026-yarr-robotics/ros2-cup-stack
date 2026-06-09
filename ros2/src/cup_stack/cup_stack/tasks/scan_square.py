"""Task that traces an axis-aligned square with the camera fixed downward.

pos1 단일 자세 스캔(`scan.py`)과 달리, 카메라를 계속 하향(``DOWN_ORI``)으로
고정한 채 base_link XY 평면에서 사각형 네 꼭짓점을 순회한다. Z 는 HOME 자세의
EE 높이(런타임 FK)를 그대로 사용한다. 시작/복귀 위치는 scan 과 동일하게
시작 시점의 joint 자세를 캡처해 마지막에 복귀한다.

Sequence
  [0] PTP : 초기 위치 → 꼭짓점1            (pose goal, 카메라 하향, z=HOME)
  [1] LIN : 꼭짓점1 → 꼭짓점2              (직선 변)
  [2] LIN : 꼭짓점2 → 꼭짓점3
  [3] LIN : 꼭짓점3 → 꼭짓점4
  [4] LIN : 꼭짓점4 → 꼭짓점1              (둘레 닫기)
  [5] PTP : 꼭짓점1 → 초기 위치            (joint-space)
"""

import math
import time

import numpy as np
from moveit.core.robot_state import RobotState

from cup_stack.config import DOWN_ORI, ScanConfig
from cup_stack.runtime import CupStackRuntime


class ScanSquareTask:

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

    def _home_ee_z(self) -> float:
        """HOME joint 자세의 EE Z 높이 (이동 없이 FK 로 계산)."""
        rt = self.runtime
        home_state = RobotState(rt.robot_model)
        home_state.set_joint_group_positions(
            rt.motion.group_name, rt.motion.home_joints_rad
        )
        home_state.update()
        transform = np.asarray(
            home_state.get_global_link_transform(rt.motion.ee_link),
            dtype=float,
        )
        return float(transform[2, 3])

    def _dwell(self) -> None:
        dwell = max(0.0, float(self.cfg.dwell_sec))
        if dwell > 0.0:
            self.logger.info(f"  대기 {dwell:.2f}s")
            time.sleep(dwell)

    def _move_corner(self, idx: int, x: float, y: float, z: float, lin: bool) -> bool:
        """카메라 하향 고정 pose 이동. LIN 실패 시 PTP→OMPL 로 폴백."""
        mode = "LIN" if lin else "PTP"
        self.logger.info(
            f"  꼭짓점{idx} ({mode}): x={x:.3f} y={y:.3f} z={z:.3f}"
        )
        ok = self.runtime.try_move_to_pose(
            x, y, z,
            safe_z_min=self.runtime.workspace.z_min,
            ori=DOWN_ORI,
            lin=lin,
        )
        if not ok:
            self.logger.error(f"  꼭짓점{idx} 이동 실패")
            return False
        self._dwell()
        return True

    def _ptp_joints(self, label: str, joints_rad: list[float]) -> bool:
        """joint-space PTP 이동. 실패 시 OMPL 로 재시도 (초기 위치 복귀용)."""
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

        exec_result = rt.robot.execute(
            group_name=rt.motion.group_name,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        # MoveItPy returns falsy on ABORTED/FAILED/PREEMPTED — surface it so an
        # aborted move (e.g. controller action client not connected) does not
        # leak through as success while the robot never moved.
        if not exec_result:
            self.logger.error(f"  {label} 실행 실패 (ABORTED)")
            return False
        return True

    def try_execute(self) -> bool:
        self.logger.info("=== Square scan task start ===")

        # 시작 전 현재 joint 상태 저장 (2방향 스캔과 동일한 복귀 지점)
        start_joints = self._current_joints_rad()
        self._log_joints("초기 joints", start_joints)

        z = self._home_ee_z()
        corners = self.cfg.square_corners_xy
        self.logger.info(
            f"사각형 중심=({self.cfg.square_center_x:.3f}, "
            f"{self.cfg.square_center_y:.3f}) 한 변={self.cfg.square_size:.3f}m "
            f"z(HOME)={z:.3f}m, 카메라 하향 고정"
        )

        # [0] PTP: 초기 위치 → 꼭짓점1
        self.logger.info("[0] PTP → 꼭짓점1")
        x0, y0 = corners[0]
        if not self._move_corner(1, x0, y0, z, lin=False):
            return False

        # [1..3] LIN: 꼭짓점1 → 2 → 3 → 4 (직선 변)
        for i in range(1, len(corners)):
            self.logger.info(f"[{i}] LIN → 꼭짓점{i + 1}")
            x, y = corners[i]
            if not self._move_corner(i + 1, x, y, z, lin=True):
                return False

        # [4] LIN: 꼭짓점4 → 꼭짓점1 (둘레 닫기)
        self.logger.info(f"[{len(corners)}] LIN → 꼭짓점1 (둘레 닫기)")
        if not self._move_corner(1, x0, y0, z, lin=True):
            return False

        # [5] PTP: 꼭짓점1 → 초기 위치
        self.logger.info(f"[{len(corners) + 1}] PTP → 초기 위치")
        if not self._ptp_joints("초기 위치", start_joints):
            return False

        self.logger.info("=== Square scan complete ===")
        return True
