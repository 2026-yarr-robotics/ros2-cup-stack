"""Compute HOME joint angles for DOWN_ORI at a target XYZ position."""

import math
import threading

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanRequestParameters


def main(args=None):
    rclpy.init(args=args)
    node = Node("find_vertical_home")
    node.declare_parameter("target_x", float("nan"))
    node.declare_parameter("target_y", float("nan"))
    node.declare_parameter("target_z", float("nan"))

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot = MoveItPy(node_name="find_vertical_home_moveit")
    arm = robot.get_planning_component("manipulator")

    monitor = robot.get_planning_scene_monitor()
    with monitor.read_only() as scene:
        transform = np.asarray(
            scene.current_state.get_global_link_transform("link_6"),
            dtype=float,
        )

    x_cur, y_cur, z_cur = transform[0, 3], transform[1, 3], transform[2, 3]

    px = node.get_parameter("target_x").value
    py = node.get_parameter("target_y").value
    pz = node.get_parameter("target_z").value

    x = x_cur if math.isnan(px) else px
    y = y_cur if math.isnan(py) else py
    z = z_cur if math.isnan(pz) else pz

    node.get_logger().info(f"Current EE : ({x_cur:.4f}, {y_cur:.4f}, {z_cur:.4f})")
    node.get_logger().info(f"Target pose: ({x:.4f}, {y:.4f}, {z:.4f}) + DOWN_ORI")

    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 1.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0

    def try_plan(pipeline, planner_id):
        params = PlanRequestParameters(robot)
        params.planning_pipeline = pipeline
        params.planner_id = planner_id
        params.max_velocity_scaling_factor = 0.3
        params.max_acceleration_scaling_factor = 0.3
        params.planning_time = 5.0
        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=pose, pose_link="link_6")
        return arm.plan(parameters=params)

    plan_result = try_plan("pilz_industrial_motion_planner", "PTP")
    if not plan_result:
        plan_result = try_plan("ompl", "RRTConnect")

    if not plan_result:
        node.get_logger().error(
            f"IK 실패: ({x:.4f}, {y:.4f}, {z:.4f}) + DOWN_ORI 도달 불가"
        )
    else:
        traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
        traj = traj_msg.joint_trajectory
        joint_map = dict(zip(traj.joint_names, traj.points[-1].positions))
        ordered = [joint_map.get(f"joint_{i}", 0.0) for i in range(1, 7)]
        deg_vals = [math.degrees(r) for r in ordered]

        node.get_logger().info("=" * 50)
        node.get_logger().info(f"HOME joints for DOWN_ORI at ({x:.3f}, {y:.3f}, {z:.3f}):")
        node.get_logger().info("=" * 50)
        for i, d in enumerate(deg_vals, 1):
            node.get_logger().info(f"  joint_{i}: {d:.4f}")
        node.get_logger().info(
            f"\n  config.py 에 붙여넣기:\n"
            f"  home_joints_deg: tuple[float, ...] = "
            f"{tuple(round(d, 4) for d in deg_vals)}"
        )

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
