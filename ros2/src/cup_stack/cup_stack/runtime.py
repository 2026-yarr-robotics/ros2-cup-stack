"""Runtime adapters for MoveItPy and the RG gripper."""

import math
import time

import numpy as np

from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import Constraints, JointConstraint

from .config import DOWN_ORI, GripperConfig, MotionConfig, WorkspaceConfig
from .geometry import clamp_workspace, clamp_z
from .onrobot import RG


class CupStackRuntime:
    """Owns robot and gripper resources shared by cup stacking tasks."""

    def __init__(
        self,
        node,
        moveit_node_name: str,
        motion_config: MotionConfig | None = None,
        gripper_config: GripperConfig | None = None,
        workspace_config: WorkspaceConfig | None = None,
        moveit_namespace: str | None = None,
    ) -> None:
        self.node = node
        self.logger = node.get_logger()
        self.motion = motion_config or MotionConfig()
        self.gripper_config = gripper_config or GripperConfig()
        self.workspace = workspace_config or WorkspaceConfig()

        try:
            self.gripper = RG(
                self.gripper_config.name,
                self.gripper_config.toolcharger_ip,
                self.gripper_config.toolcharger_port,
            )
        except Exception as e:
            self.logger.warning(f"Gripper init failed — hardware not connected? ({e})")
            self.gripper = None

        # MoveItPy spins up its own rclcpp context via rclcpp::init(...) using
        # only the launch_arguments it builds internally, so the parent
        # process's --ros-args (including __ns:=/dsr01 from launch_ros) do NOT
        # propagate. The internal moveit_simple_controller_manager node then
        # lands at root and its FollowJointTrajectory action client never
        # binds to /dsr01/dsr_moveit_controller. Passing name_space puts the
        # primary MoveItPy node under the namespace, and a __ns remap goes
        # into the new context's global args so every internal rclcpp node
        # (planning_scene_monitor, controller manager, ...) inherits it.
        ns = moveit_namespace if moveit_namespace and moveit_namespace != "/" else None
        moveit_kwargs: dict = {"node_name": moveit_node_name}
        if ns:
            moveit_kwargs["name_space"] = ns
            moveit_kwargs["remappings"] = {"__ns": ns}
        self.robot = MoveItPy(**moveit_kwargs)
        self.arm = self.robot.get_planning_component(self.motion.group_name)
        self.robot_model = self.robot.get_robot_model()
        self.ompl_params = self._make_ompl_params()
        self.ptp_params = self._make_ptp_params()
        self.ptp_fast_params = self._make_ptp_fast_params()
        self.lin_params = self._make_lin_params()
        self.lin_fast_params = self._make_lin_fast_params()
        self.lin_slow_params = self._make_lin_slow_params()

    def _make_ompl_params(self) -> PlanRequestParameters:
        params = PlanRequestParameters(self.robot)
        params.planning_pipeline = "ompl"
        params.planner_id = "RRTConnect"
        params.max_velocity_scaling_factor = 0.4
        # Lowered from 0.2 -> 0.1 -> 0.08 to suppress harmonic-drive "click"
        # caused by the trapezoidal-profile jerk step at trajectory boundaries.
        # PTP path and its OMPL fallback share this value so behaviour is
        # consistent.
        params.max_acceleration_scaling_factor = 0.08
        params.planning_time = 2.0
        return params

    def _make_ptp_params(self) -> PlanRequestParameters:
        params = PlanRequestParameters(self.robot)
        params.planning_pipeline = "pilz_industrial_motion_planner"
        params.planner_id = "PTP"
        params.max_velocity_scaling_factor = 0.4
        params.max_acceleration_scaling_factor = 0.08
        params.planning_time = 2.0
        return params

    def _make_ptp_fast_params(self) -> PlanRequestParameters:
        # Opt-in fast PTP for free-space staging/travel segments (empty
        # approach, XY travel at clearance Z, post-release lift). PTP scales
        # joint velocities directly so no singularity blow-up is possible:
        # 0.6 caps the wrist at 135 deg/s (60% of the 225 limit) and J1/J2 at
        # 90 deg/s. Acc 0.15 stays under the 0.2 oscillation guard for the
        # position-only JTC (docs/speed_limits.md §4). NOT for the place
        # descend ([7a]/[7b]) — stacking-contact moves keep ptp_params.
        params = PlanRequestParameters(self.robot)
        params.planning_pipeline = "pilz_industrial_motion_planner"
        params.planner_id = "PTP"
        params.max_velocity_scaling_factor = 0.6
        params.max_acceleration_scaling_factor = 0.15
        params.planning_time = 2.0
        return params

    def _make_lin_params(self) -> PlanRequestParameters:
        params = PlanRequestParameters(self.robot)
        params.planning_pipeline = "pilz_industrial_motion_planner"
        params.planner_id = "LIN"
        # LIN is Cartesian: near a singularity joint velocity blows up to hold
        # the Cartesian speed. Even non-high moves can be near-singular at
        # extended reach — build_pyramid's far grid picks (x≈0.35, y≈±0.2) spiked
        # J3 to 212 deg/s (> 180 limit → alarm 1908) at the old 0.2 scale. Drop
        # the base LIN scale to 0.1 so all descend/lift moves keep headroom under
        # the joint limits (J1/J2=150, J3=180, J4-6=225 deg/s).
        params.max_velocity_scaling_factor = 0.1
        params.max_acceleration_scaling_factor = 0.08
        params.planning_time = 2.0
        return params

    def _make_lin_fast_params(self) -> PlanRequestParameters:
        # Opt-in fast LIN for non-contact lift/settle segments below
        # singular_z. Ceiling is set by the J3 evidence above: scale 0.2 hit
        # 212 deg/s at far reach, so 0.12 keeps the same worst case at
        # ~127 deg/s = 70% of the 180 limit (docs/speed_limits.md §4-1).
        # Do NOT raise past 0.17 (J3 limit point) — and contact moves
        # (pick descend, place) keep lin_params/ptp_params.
        params = PlanRequestParameters(self.robot)
        params.planning_pipeline = "pilz_industrial_motion_planner"
        params.planner_id = "LIN"
        params.max_velocity_scaling_factor = 0.12
        params.max_acceleration_scaling_factor = 0.12
        params.planning_time = 2.0
        return params

    def _make_lin_slow_params(self) -> PlanRequestParameters:
        # LIN profile for the worst near-singularity zone (high-Z, z >= singular_z,
        # near full vertical reach). The unstack 3m extraction spiked a wrist
        # joint to 256 deg/s (> 225 limit) at the old 0.2 scale. 0.08 puts that
        # worst case at ~102 deg/s (45% of the 225 limit, 2.2x headroom). Used
        # for the high pick descend/lift and the high place extra-lift.
        params = PlanRequestParameters(self.robot)
        params.planning_pipeline = "pilz_industrial_motion_planner"
        params.planner_id = "LIN"
        params.max_velocity_scaling_factor = 0.08
        params.max_acceleration_scaling_factor = 0.06
        params.planning_time = 2.0
        return params

    def try_move_home(self, z_offset_m: float = 0.0, x_offset_m: float = 0.0) -> bool:
        """Plan and execute the configured HOME joint state.

        Two-stage retract so a finished pyramid (or any cups) under the path is
        not knocked over: traverse in XY at a clearance Z to the HOME XY, then
        either descend to the HOME joint configuration or, when ``z_offset_m`` is
        positive, finish at a lifted Cartesian HOME pose (HOME Z + offset).
        This lets post-place recovery keep the camera/arm higher without changing
        the configured joint HOME used at startup.
        """

        home_state = RobotState(self.robot_model)
        home_state.set_joint_group_positions(
            self.motion.group_name,
            self.motion.home_joints_rad,
        )
        home_state.update()
        home_T = np.asarray(
            home_state.get_global_link_transform(self.motion.ee_link),
            dtype=float,
        )
        home_x, home_y, home_z = (
            float(home_T[0, 3]), float(home_T[1, 3]), float(home_T[2, 3]),
        )

        z_offset_m = max(0.0, float(z_offset_m))
        x_offset_m = float(x_offset_m)
        lifted_home_x = min(max(home_x + x_offset_m, self.workspace.x_min),
                            self.workspace.x_max)
        lifted_home_y = home_y
        lifted_home_z = min(home_z + z_offset_m, self.workspace.z_max)
        clearance_z = max(home_z, lifted_home_z)

        # Stage 1: move at a clearance Z to the HOME XY so the EE clears
        # anything built below. If a lifted HOME is requested, first raise in
        # place to the lifted target Z and finish there instead of descending to
        # the exact joint HOME.
        try:
            cur_T = self.current_ee_matrix()
            cur_x, cur_y, cur_z = (
                float(cur_T[0, 3]), float(cur_T[1, 3]), float(cur_T[2, 3]),
            )
        except Exception as exc:  # noqa: BLE001 - best-effort; fall back to direct
            self.logger.warn(f"current EE pose unavailable ({exc}); direct HOME move")
            cur_x, cur_y, cur_z = home_x, home_y, home_z

        if cur_z < clearance_z - 1e-3:
            self.logger.info(
                f"HOME[0] lift in place z={cur_z:.3f} → {clearance_z:.3f}"
            )
            if not self.try_move_to_pose(
                cur_x, cur_y, clearance_z, self.workspace.z_min,
                lin=True, slow=clearance_z >= 0.50,
            ):
                self.logger.warn(
                    "HOME[0] lift failed; falling back to current Z traverse"
                )
                clearance_z = cur_z

        traverse_z = max(cur_z, clearance_z)
        if traverse_z > home_z + 1e-3:
            self.logger.info(
                f"HOME[1] keep z={traverse_z:.3f} → HOME xy=({lifted_home_x:.3f},{lifted_home_y:.3f})"
            )
            if not self.try_move_to_pose(
                lifted_home_x, lifted_home_y, traverse_z, self.workspace.z_min,
                lin=True, slow=traverse_z >= 0.50,
            ):
                self.logger.warn(
                    "HOME[1] XY traverse failed; falling back to direct HOME"
                )

        if z_offset_m > 1e-6:
            self.logger.info(
                f"HOME[2] lifted cartesian home x={lifted_home_x:.3f} z={lifted_home_z:.3f} "
                f"(x_offset={x_offset_m:+.3f}, z_offset=+{z_offset_m:.3f})"
            )
            if abs(traverse_z - lifted_home_z) > 1e-3:
                return self.try_move_to_pose(
                    lifted_home_x, lifted_home_y, lifted_home_z, self.workspace.z_min,
                    lin=True, slow=lifted_home_z >= 0.50,
                )
            return True

        # Stage 2: descend to the exact HOME joint configuration.
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=home_state)
        plan_result = self.arm.plan(parameters=self.ompl_params)
        if not plan_result:
            self.logger.error("HOME planning failed")
            return False

        exec_result = self.robot.execute(
            group_name=self.motion.group_name,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        # MoveItPy returns truthy on SUCCESS, falsy on ABORTED/FAILED/PREEMPTED.
        # Without this check ABORTED executions (e.g. controller not connected)
        # were leaking through as success — the skill would proceed to the next
        # step on a robot that never moved.
        if not exec_result:
            self.logger.error("Trajectory execution failed")
            return False
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
        slow: bool = False,
        fast: bool = False,
    ) -> bool:
        """Plan and execute a pose move.

        ``slow`` selects the reduced-velocity LIN profile (lin_slow_params) for
        near-singularity high-Z moves so joint velocity stays under the limit.
        Only affects LIN moves (PTP already respects joint velocity limits).

        ``fast`` opts a free-space, non-contact segment into the raised
        staging/travel profile (ptp_fast_params / lin_fast_params — see
        docs/speed_limits.md). ``slow`` always wins over ``fast``.
        """

        cx, cy, cz = clamp_workspace(x, y, z, self.workspace, self.logger)
        cz = clamp_z(cz, safe_z_min)  # safe_z_min 을 추가 하한으로 적용
        self.arm.set_start_state_to_current_state()
        pose = PoseStamped()
        pose.header.frame_id = self.motion.base_frame
        pose.pose.position.x = cx
        pose.pose.position.y = cy
        pose.pose.position.z = cz
        orientation = ori or DOWN_ORI
        pose.pose.orientation.x = orientation["x"]
        pose.pose.orientation.y = orientation["y"]
        pose.pose.orientation.z = orientation["z"]
        pose.pose.orientation.w = orientation["w"]
        self.arm.set_goal_state(
            pose_stamped_msg=pose,
            pose_link=self.motion.ee_link,
        )

        if lin:
            if slow:
                plan_params = self.lin_slow_params
            elif fast:
                plan_params = self.lin_fast_params
            else:
                plan_params = self.lin_params
        else:
            plan_params = self.ptp_fast_params if fast else self.ptp_params
        plan_result = self.arm.plan(parameters=plan_params)
        if not plan_result and lin and not strict:
            self.logger.warn("LIN planning failed; retrying with PTP")
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(
                pose_stamped_msg=pose,
                pose_link=self.motion.ee_link,
            )
            plan_result = self.arm.plan(parameters=self.ptp_params)
        if not plan_result and not strict:
            # PTP (or LIN→PTP) all failed — last resort: OMPL
            # joint rotation constraint 로 한바퀴 회전 경로 차단
            self.logger.warn("Pilz planning failed; retrying with OMPL (joint-bounded)")
            constraints = self._joint_rotation_constraints()
            self.arm.set_path_constraints(constraints)
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(
                pose_stamped_msg=pose,
                pose_link=self.motion.ee_link,
            )
            plan_result = self.arm.plan(parameters=self.ompl_params)
            self.arm.set_path_constraints(Constraints())  # 반드시 초기화
        if not plan_result:
            self.logger.error("Planning failed")
            return False

        exec_result = self.robot.execute(
            group_name=self.motion.group_name,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        if not exec_result:
            self.logger.error("Trajectory execution failed")
            return False
        return True

    def _joint_rotation_constraints(
        self, tolerance: float = math.pi
    ) -> Constraints:
        """현재 joint 위치 기준 ±tolerance 이내로 제한하는 path constraint.

        OMPL fallback 시 적용해 한바퀴(360°) 회전 경로를 차단.
        tolerance 기본값 π(180°) → 각 관절이 현재 위치에서
        최대 반 바퀴 이내로만 이동 가능.
        """
        monitor = self.robot.get_planning_scene_monitor()
        with monitor.read_only() as scene:
            positions = np.array(
                scene.current_state.get_joint_group_positions(
                    self.motion.group_name
                ),
                dtype=float,
            )
        c = Constraints()
        for i, pos in enumerate(positions, 1):
            jc = JointConstraint()
            jc.joint_name = f"joint_{i}"
            jc.position = float(pos)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

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

    # Aliases used by place_cup_at (which times its own sleeps via r.sleep()).
    def gripper_open(self) -> bool:
        return self.try_open_gripper(0.0)

    def gripper_close(self) -> bool:
        return self.try_grip_cup(0.0)

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
