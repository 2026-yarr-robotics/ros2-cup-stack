# CLAUDE.md

This file provides guidance to Claude Code when working in this repository.

## Overview

This is a ROS 2 Humble workspace for running human-like speed stacking with a
Doosan M0609 collaborative robot and MoveIt 2.

- `src/doosan-robot2/` — upstream Doosan Robotics driver stack, tracked as a
  git submodule. Do not edit this directory unless explicitly requested.
- `src/cup_stack/` — local application package for cup stacking control.

## Workspace Setup

After cloning, initialize the Doosan driver submodule:

```bash
git submodule update --init --recursive
```

Install dependencies with `rosdep` from the workspace root when available:

```bash
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

Build the workspace:

```bash
./src/cup_stack/build_cup_stack.sh
source install/setup.bash
```

For controller firmware 3.x, pass the CMake option through the build script:

```bash
./src/cup_stack/build_cup_stack.sh --cmake-args -DDRCF_VER=3
```

## Running MoveIt Bringup

Simulation / virtual mode:

```bash
./src/cup_stack/bringup_sim.sh
```

Real robot mode:

```bash
./src/cup_stack/bringup_real.sh 192.168.1.100
```

Both scripts source `/opt/ros/humble/setup.bash` and then search upward from the
script path for this workspace's `install/setup.bash`.

## Running Cup Stack Tasks

Open a second terminal after bringup:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cup_stack cup_pyramid.launch.py nest_inc:=0.0127
ros2 launch cup_stack cup_unstack.launch.py nest_inc:=0.0127
```

`cup_unstack` assumes the current end-effector XY position is the pyramid
center, matching the final state of `cup_pyramid`.

## Architecture

**Layer 1 — Doosan Driver** (`src/doosan-robot2/`):

- `dsr_bringup2` — real/virtual/Gazebo/MoveIt launch files.
- `dsr_msgs2` — Doosan-specific ROS 2 messages, services, and actions.
- `dsr_hardware2`, `dsr_controller2` — ros2_control hardware and controllers.
- `dsr_description2` — URDF/Xacro and meshes.
- `dsr_moveit2/dsr_moveit_config_m0609` — MoveIt config for M0609.

**Layer 2 — Application** (`src/cup_stack/`):

- `cup_stack/runtime.py` — MoveItPy runtime and gripper adapter.
- `cup_stack/tasks/` — reusable task sequences.
- `cup_stack/nodes/` — thin ROS 2 executable wrappers.
- `launch/` — MoveItPy parameter loading and task launch files.
- `config/moveit_py.yaml` — MoveItPy planning scene and pipeline config.

## Motion Planning Pattern

Pose goals are created as `geometry_msgs/msg/PoseStamped` in `base_link` and
planned for the `link_6` end-effector through MoveItPy. HOME moves use
`moveit.core.robot_state.RobotState`.

Default planning values:

- Planning group: `manipulator`
- Base frame: `base_link`
- End-effector link: `link_6`
- HOME joints: `[0, 0, 90, 0, 90, 90]` degrees

## Testing

Use fast syntax checks when ROS dependencies are not available:

```bash
python3 -m compileall src/cup_stack
```

Use ament tests after dependencies are installed:

```bash
colcon test --packages-select cup_stack
colcon test-result --verbose
```
