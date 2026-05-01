# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS2 Humble workspace for controlling a **Doosan M0609 collaborative robot** with MoveIt2 motion planning. The workspace contains:

- `src/doosan-robot2/` — upstream Doosan driver (git submodule, do not modify)
- `src/dsr_practice/` — local application package for custom pick-and-place and motion demos

## Environment Setup

All terminals must source these in order before running ROS2 commands:

```bash
source /opt/ros/humble/setup.bash
source /home/ssu/ws_moveit/install/setup.bash
source /home/ssu/ros2_ws/install/setup.bash
source /home/ssu/install/setup.bash
```

## Build Commands

```bash
# Build entire workspace (from /home/ssu/ros2_ws)
colcon build

# Build only the local practice package (much faster)
colcon build --packages-select dsr_practice

# Build Doosan driver for controller v3.x firmware
colcon build --cmake-args -DDRCF_VER=3
```

After rebuilding `dsr_practice`, re-source the workspace (`source install/setup.bash`) since it is an ament_python package.

## Running the Robot

**Simulation (virtual mode)** — starts the Doosan emulator + MoveIt2 + RViz:
```bash
cd /home/ssu/ros2_ws/src/dsr_practice
./bringup_sim.sh
```

**Real robot** — connects to physical robot over Ethernet:
```bash
cd /home/ssu/ros2_ws/src/dsr_practice
./bringup_real.sh [ROBOT_IP]   # default IP: 192.168.137.50
```

## Launching Applications (Terminal 2, after bringup)

```bash
ros2 launch dsr_practice pick_and_place.launch.py
ros2 launch dsr_practice keyboard_teleop.launch.py
ros2 launch dsr_practice mp_basic.launch.py
ros2 launch dsr_practice mp_waypoint.launch.py
ros2 launch dsr_practice mp_waypoint_pilz.launch.py
ros2 launch dsr_practice collision_obstacle.launch.py
```

## Running Tests

```bash
colcon test --packages-select dsr_practice
colcon test-result --verbose
```

Tests cover copyright, flake8 style, and pep257 docstrings. There are no functional/integration tests yet.

## Architecture

### Two-Layer Design

**Layer 1 — Doosan Driver** (`src/doosan-robot2/`, upstream, read-only):
- `dsr_msgs2` — all custom ROS2 message/service/action definitions (~70 services)
- `dsr_hardware2` — ros2_control `HardwareInterface` communicating via DRFL C++ library
- `dsr_controller2` — joint trajectory controller + state broadcaster
- `dsr_bringup2` — launch files for real/virtual/Gazebo/MuJoCo/MoveIt modes
- `dsr_description2` — URDF/XACRO + mesh models for all Doosan robot variants
- `dsr_moveit_config_m0609` — MoveIt2 SRDF, kinematics, joint limits, planner configs

**Layer 2 — Application** (`src/dsr_practice/`, local development):
- Python nodes using `moveit.planning.MoveItPy` for all motion
- `onrobot.py` — Modbus TCP driver for OnRobot RG2/RG6 gripper (IP `192.168.1.1:502`)
- All nodes load `config/moveit_py.yaml` via their launch files

### Motion Planning Pattern

All motion nodes follow this pattern:
```python
robot = MoveItPy(node_name="moveit_py")
arm = robot.get_planning_component("manipulator")  # group from SRDF

params = PlanRequestParameters(robot)
params.planning_pipeline = "ompl"      # or "pilz_industrial_motion_planner"
params.planner_id = "RRTConnectkConfigDefault"  # or "PTP", "LIN"

arm.set_start_state_to_current_state()
arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_6")
result = arm.plan(parameters=params)
robot.execute(group_name="manipulator", robot_trajectory=result.trajectory, blocking=True)
```

### Planning Pipelines (defined in `config/moveit_py.yaml`)

| Preset | Pipeline | Planner | Use case |
|--------|----------|---------|----------|
| `ompl_rrtc` | ompl | RRTConnect | Joint-space / home moves |
| `pilz_lin` | pilz_industrial_motion_planner | PTP | Precise cartesian placement |
| `ompl_rrt_star` | ompl | RRT* | Obstacle-dense environments |
| `chomp` | chomp | CHOMP | Trajectory smoothing |

### Frames & Geometry

- Planning group name (SRDF): `manipulator`
- Base frame: `base_link`
- End-effector link: `link_6`
- Home joints (deg): `[0, 0, 90, 0, 90, 0]`
- Safe workspace (base_link): X ≥ 0, Y ∈ [-0.3, 0.3], Z ≥ 0.27 m

### Gripper

`onrobot.py` wraps a Modbus TCP client (`pymodbus`). Widths are in raw units (1/10 mm):
- RG2: max width 1100 (110 mm), max force 400
- RG6: max width 1600 (160 mm), max force 1200

`pick_and_place.py` auto-detects gripper presence; falls back to simulation-only mode if connection fails.

## Key Files

| File | Purpose |
|------|---------|
| `src/dsr_practice/dsr_practice/pick_and_place.py` | Full pick-and-place workflow (9-step) |
| `src/dsr_practice/dsr_practice/keyboard_teleop.py` | Arrow-key IK teleoperation |
| `src/dsr_practice/dsr_practice/onrobot.py` | OnRobot RG gripper Modbus driver |
| `src/dsr_practice/config/moveit_py.yaml` | MoveItPy planner configuration |
| `src/dsr_practice/bringup_sim.sh` | Start virtual robot + MoveIt |
| `src/dsr_practice/bringup_real.sh` | Start real robot + MoveIt |
| `src/doosan-robot2/dsr_msgs2/` | All ROS2 service/message definitions |
| `src/doosan-robot2/dsr_bringup2/launch/` | Official bringup launch files |
