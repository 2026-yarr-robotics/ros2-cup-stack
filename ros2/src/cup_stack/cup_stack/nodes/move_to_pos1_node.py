"""ROS 2 entry point: pos1 (카메라 수직 하향) 으로 joint-space 이동."""

import rclpy
from rclpy.node import Node

from cup_stack.config import ScanConfig
from cup_stack.runtime import CupStackRuntime
from moveit.core.robot_state import RobotState


def main(args=None):
    rclpy.init(args=args)
    node = Node("move_to_pos1_node")
    logger = node.get_logger()

    try:
        scan_cfg = ScanConfig()
        runtime = CupStackRuntime(node, "move_to_pos1_moveit")

        logger.info(
            "pos1 joints (deg): "
            + ", ".join(
                f"J{i}={d:.4f}" for i, d in enumerate(scan_cfg.pos1_joints_deg, 1)
            )
        )

        pos1_state = RobotState(runtime.robot_model)
        pos1_state.set_joint_group_positions(
            runtime.motion.group_name,
            scan_cfg.pos1_joints_rad,
        )
        pos1_state.update()

        runtime.arm.set_start_state_to_current_state()
        runtime.arm.set_goal_state(robot_state=pos1_state)
        # Pilz PTP: joint 최단경로 보간 — OMPL 대비 한바퀴 도는 현상 없음
        plan_result = runtime.arm.plan(parameters=runtime.ptp_params)
        if not plan_result:
            logger.warn("pos1 PTP planning failed; retrying with OMPL")
            runtime.arm.set_start_state_to_current_state()
            runtime.arm.set_goal_state(robot_state=pos1_state)
            plan_result = runtime.arm.plan(parameters=runtime.ompl_params)
        if not plan_result:
            logger.error("pos1 계획 실패")
            return

        logger.info("계획 완료 — 실행 중...")
        runtime.robot.execute(
            group_name=runtime.motion.group_name,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        logger.info("pos1 도달 완료")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
