"""Task that traces a rectangular scan path at the pos1 Z height.

Sequence
  [0] PTP : start  → pos1  (joint-space, Pilz PTP)
  [1] LIN : pos1   → pos2  (Cartesian)
  [2] LIN : pos2   → pos3  (Cartesian)
  [3] LIN : pos3   → pos4  (Cartesian)
  [4] LIN : pos4   → pos1  (Cartesian)
"""

from moveit.core.robot_state import RobotState

from cup_stack.config import DOWN_ORI, ScanConfig
from cup_stack.runtime import CupStackRuntime


class ScanTask:

    def __init__(
        self,
        runtime: CupStackRuntime,
        scan_config: ScanConfig | None = None,
        safe_z_min: float = 0.25,
    ) -> None:
        self.runtime = runtime
        self.cfg = scan_config or ScanConfig()
        self.safe_z_min = safe_z_min
        self.logger = runtime.logger

    def _log_ee(self, label: str) -> None:
        tf = self.runtime.current_ee_matrix()
        self.logger.info(
            f"  {label} EE: x={tf[0,3]:.3f}  y={tf[1,3]:.3f}  z={tf[2,3]:.3f}"
        )

    def _move_to_pos1(self) -> bool:
        """PTP — joint 최단경로로 pos1 이동."""
        rt = self.runtime
        joints_deg = self.cfg.pos1_joints_deg
        self.logger.info(
            f"  목표 joints: "
            + "  ".join(f"J{i}={d:.2f}°" for i, d in enumerate(joints_deg, 1))
        )
        pos1_state = RobotState(rt.robot_model)
        pos1_state.set_joint_group_positions(
            rt.motion.group_name,
            self.cfg.pos1_joints_rad,
        )
        pos1_state.update()
        rt.arm.set_start_state_to_current_state()
        rt.arm.set_goal_state(robot_state=pos1_state)
        plan_result = rt.arm.plan(parameters=rt.ptp_params)
        if not plan_result:
            self.logger.warn("  pos1 PTP 실패 — OMPL 재시도")
            rt.arm.set_start_state_to_current_state()
            rt.arm.set_goal_state(robot_state=pos1_state)
            plan_result = rt.arm.plan(parameters=rt.ompl_params)
        if not plan_result:
            self.logger.error("  pos1 계획 실패")
            return False
        rt.robot.execute(
            group_name=rt.motion.group_name,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        self._log_ee("도달")
        return True

    def _lin(self, label: str, x: float, y: float, z: float) -> bool:
        """LIN — Cartesian 직선 이동 (PTP fallback 포함)."""
        self.logger.info(f"  목표: ({x:.3f}, {y:.3f}, {z:.3f})")
        if not self.runtime.try_move_to_pose(
            x, y, z,
            self.safe_z_min,
            ori=DOWN_ORI,
            lin=True,
            strict=False,
        ):
            self.logger.error(f"  {label} 이동 실패 — 중단")
            return False
        self._log_ee("도달")
        return True

    def try_execute(self) -> bool:
        self.logger.info("=== Scan task start ===")

        # 시작 전 현재 EE 위치 저장
        start_tf = self.runtime.current_ee_matrix()
        start_x = float(start_tf[0, 3])
        start_y = float(start_tf[1, 3])
        start_z = float(start_tf[2, 3])
        self.logger.info(f"시작 EE: x={start_x:.3f}  y={start_y:.3f}  z={start_z:.3f}")

        # [0] PTP: 현재 위치 → pos1
        self.logger.info("[0] PTP → pos1")
        if not self._move_to_pos1():
            return False

        # pos1 EE 위치에서 z 높이 확정
        tf = self.runtime.current_ee_matrix()
        z = float(tf[2, 3])

        # [1~3] LIN: pos2 → pos3 → pos4
        waypoints = [
            ("[1] LIN → pos2", *self.cfg.pos2_xy),
            ("[2] LIN → pos3", *self.cfg.pos3_xy),
            ("[3] LIN → pos4", *self.cfg.pos4_xy),
        ]
        for label, wx, wy in waypoints:
            self.logger.info(label)
            if not self._lin(label, wx, wy, z):
                return False

        # [4] LIN: pos4 → 시작 위치 복귀
        self.logger.info("[4] LIN → 시작 위치")
        if not self._lin("[4] LIN → 시작 위치", start_x, start_y, start_z):
            return False

        self.logger.info("=== Scan task complete ===")
        return True
