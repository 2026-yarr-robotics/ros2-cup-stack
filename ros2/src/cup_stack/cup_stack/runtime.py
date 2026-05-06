"""Runtime adapters for MoveItPy and the RG gripper."""

import time

import numpy as np

from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

from .config import DOWN_ORI, GripperConfig, MotionConfig
from .geometry import clamp_z
from .onrobot import RG


class CupStackRuntime:
    """Owns robot and gripper resources shared by cup stacking tasks."""

    def __init__(
        self,
        node,
        moveit_node_name: str,
        motion_config: MotionConfig | None = None,
        gripper_config: GripperConfig | None = None,
    ) -> None:
        self.node = node
        self.logger = node.get_logger()
        self.motion = motion_config or MotionConfig()
        self.gripper_config = gripper_config or GripperConfig()

        try:
            self.gripper = RG(
                self.gripper_config.name,
                self.gripper_config.toolcharger_ip,
                self.gripper_config.toolcharger_port,
            )
        except Exception as e:
            self.logger.warning(f"Gripper init failed — hardware not connected? ({e})")
            self.gripper = None

        self.robot = MoveItPy(node_name=moveit_node_name)
        self.arm = self.robot.get_planning_component(self.motion.group_name)
        self.robot_model = self.robot.get_robot_model()
        self.ompl_params = self._make_ompl_params()
        self.ptp_params = self._make_ptp_params()
        self.lin_params = self._make_lin_params()

    def _make_ompl_params(self) -> PlanRequestParameters:
        params = PlanRequestParameters(self.robot)
        params.planning_pipeline = "ompl"
        params.planner_id = "RRTConnect"
        params.max_velocity_scaling_factor = 0.7
        params.max_acceleration_scaling_factor = 0.5
        params.planning_time = 2.0
        return params

    def _make_ptp_params(self) -> PlanRequestParameters:
        params = PlanRequestParameters(self.robot)
        params.planning_pipeline = "pilz_industrial_motion_planner"
        params.planner_id = "PTP"
        params.max_velocity_scaling_factor = 0.7
        params.max_acceleration_scaling_factor = 0.5
        params.planning_time = 2.0
        return params

    def _make_lin_params(self) -> PlanRequestParameters:
        params = PlanRequestParameters(self.robot)
        params.planning_pipeline = "pilz_industrial_motion_planner"
        params.planner_id = "LIN"
        params.max_velocity_scaling_factor = 0.3
        params.max_acceleration_scaling_factor = 0.15
        params.planning_time = 2.0
        return params

    def try_move_home(self) -> bool:
        """Plan and execute the configured HOME joint state."""

        home_state = RobotState(self.robot_model)
        home_state.set_joint_group_positions(
            self.motion.group_name,
            self.motion.home_joints_rad,
        )
        home_state.update()
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=home_state)
        plan_result = self.arm.plan(parameters=self.ompl_params)
        if not plan_result:
            self.logger.error("HOME planning failed")
            return False

        self.robot.execute(
            group_name=self.motion.group_name,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        return True

    def try_move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        safe_z_min: float,
        ori: dict[str, float] | None = None,
        lin: bool = False,
        strict: bool = False,
    ) -> bool:
        """Plan and execute a pose move."""

        self.arm.set_start_state_to_current_state()
        pose = PoseStamped()
        pose.header.frame_id = self.motion.base_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(clamp_z(z, safe_z_min))
        orientation = ori or DOWN_ORI
        pose.pose.orientation.x = orientation["x"]
        pose.pose.orientation.y = orientation["y"]
        pose.pose.orientation.z = orientation["z"]
        pose.pose.orientation.w = orientation["w"]
        self.arm.set_goal_state(
            pose_stamped_msg=pose,
            pose_link=self.motion.ee_link,
        )

        plan_params = self.lin_params if lin else self.ptp_params
        plan_result = self.arm.plan(parameters=plan_params)
        if not plan_result and lin and not strict:
            self.logger.warn("LIN planning failed; retrying with PTP")
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(
                pose_stamped_msg=pose,
                pose_link=self.motion.ee_link,
            )
            plan_result = self.arm.plan(parameters=self.ptp_params)
        if not plan_result and not lin:
            self.logger.warn("PTP planning failed; retrying with OMPL")
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(
                pose_stamped_msg=pose,
                pose_link=self.motion.ee_link,
            )
            plan_result = self.arm.plan(parameters=self.ompl_params)
        if not plan_result:
            self.logger.error("Planning failed")
            return False

        self.robot.execute(
            group_name=self.motion.group_name,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        return True

    def current_ee_matrix(self) -> np.ndarray:
        """Return the current end-effector transform in the base frame."""

        monitor = self.robot.get_planning_scene_monitor()
        with monitor.read_only() as scene:
            transform = np.asarray(
                scene.current_state.get_global_link_transform(
                    self.motion.ee_link
                ),
                dtype=float,
            )
        return transform

    def current_ee_xy(self) -> tuple[float, float]:
        """Return the current end-effector XY position in the base frame."""

        transform = self.current_ee_matrix()
        return float(transform[0, 3]), float(transform[1, 3])

    def try_open_gripper(self, sleep_sec: float) -> bool:
        """Open the configured gripper."""

        self.gripper.move_gripper(
            self.gripper_config.open_width,
            self.gripper_config.force,
        )
        time.sleep(sleep_sec)
        return True

    def try_grip_cup(self, sleep_sec: float) -> bool:
        """Grip a cup with the configured width and force."""

        self.gripper.move_gripper(
            self.gripper_config.grip_width,
            self.gripper_config.force,
        )
        time.sleep(sleep_sec)
        self.log_grip_status()
        return True

    def try_release_cup(self, sleep_sec: float) -> bool:
        """Release a cup by opening the gripper."""

        return self.try_open_gripper(sleep_sec)

    def log_grip_status(self) -> None:
        """Read gripper status if available and log it."""

        try:
            width = self.gripper.get_width() / 10.0
            status = self.gripper.get_status()
            grip_ok = len(status) > 1 and status[1] == 1
            state = "OK" if grip_ok else "FAIL"
            self.logger.info(f"      width={width:.1f}mm, grip={state}")
        except Exception:
            self.logger.warn("      failed to read gripper status; continuing")

    def sleep(self, seconds: float) -> None:
        """Sleep for task timing gaps."""

        time.sleep(seconds)
