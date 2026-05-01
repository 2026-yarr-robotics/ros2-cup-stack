# Repository Guidelines

## Project Structure & Module Organization

This repository contains a standalone ROS 2 workspace under `ros2/`. The
application source code lives under `ros2/src/cup_stack/cup_stack/`.

- `ros2/src/doosan-robot2/`: upstream Doosan Robotics driver stack submodule.
- `config/`: MoveItPy configuration, including `moveit_py.yaml`.
- `cup_stack/config.py`: robot frames, gripper settings, and cup geometry.
- `cup_stack/geometry.py`: pose and orientation helpers.
- `cup_stack/runtime.py`: MoveItPy execution, `PoseStamped` goal creation, and
  gripper commands.
- `cup_stack/tasks/`: reusable task sequences such as `cup_pyramid.py` and
  `cup_unstack.py`.
- `cup_stack/nodes/`: ROS 2 executable wrappers.
- `launch/`: launch files for running each task.
- `test/`: ament lint test placeholders.

## Build, Test, and Development Commands

Run commands from the repository root unless noted.

```bash
git submodule update --init --recursive
cd ros2
colcon build --symlink-install
```

Initializes the Doosan driver submodule and builds the workspace, creating
local `install/`, `build/`, and `log/` directories.

```bash
source ros2/install/setup.bash
ros2 launch cup_stack cup_pyramid.launch.py nest_inc:=0.0127
ros2 launch cup_stack cup_unstack.launch.py nest_inc:=0.0127
```

Runs the pyramid build and restore tasks after the Doosan MoveIt bringup is
already active.

```bash
python3 -m compileall ros2/src/cup_stack
```

Performs a fast Python syntax check.

## Coding Style & Naming Conventions

Use Python 3 with 4-space indentation. Keep modules small and task-specific.
Prefer explicit names such as `CupStackRuntime`, `CupPyramidTask`, and
`try_move_to_pose`. Use `try_*` for operations that execute robot actions and
return `bool`. Keep ROS executable wrappers thin; put shared behavior in
`runtime.py` or `tasks/`.

## Testing Guidelines

The package includes standard ament lint tests: `ament_flake8`,
`ament_pep257`, and `ament_copyright`. Add functional tests under
`ros2/src/cup_stack/test/` when behavior can be verified without robot hardware.
Name tests `test_*.py`. Before committing, run at least the compile check.

## Commit & Pull Request Guidelines

Commit history uses short Conventional Commit-style messages, for example
`feat: add cup stack ROS 2 tasks` and `docs: simplify project readme`. Keep
that style for new commits.

Pull requests should include a concise summary, commands run, and any robot or
simulation assumptions. For motion changes, mention affected task steps,
frames, safety constraints, and whether real hardware was used.

## Safety & Configuration Notes

Do not run motion tasks unless Doosan bringup, MoveIt, `/joint_states`, and the
OnRobot RG2 Modbus connection are confirmed. Keep hardware-specific values
centralized in `config.py`.
