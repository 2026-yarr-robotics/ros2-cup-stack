# cup_stack

`cup_stack`은 Doosan M0609 robot과 MoveItPy를 사용해 cup stacking task를
실행하는 ROS 2 `ament_python` package입니다. 기존 `dsr_practice`의
`cup_pyramid.py`, `cup_unstack.py` 동작을 재사용 가능한 task module과 얇은
ROS 2 node wrapper로 분리했습니다.

## Overview

이 package는 두 가지 main task를 제공합니다.

| Task | Executable | Launch file | Description |
| --- | --- | --- | --- |
| `CupPyramidTask` | `cup_pyramid` | `cup_pyramid.launch.py` | nested stack에서 6개 cup을 pick해서 3-2-1 pyramid로 place |
| `CupUnstackTask` | `cup_unstack` | `cup_unstack.launch.py` | 3-2-1 pyramid를 top-down 순서로 pick해서 nested stack으로 복귀 |

`DSR` prefix는 Doosan Robotics driver stack에서 사용하는 이름입니다.
예를 들어 `dsr_bringup2`, `dsr_msgs2`, `dsr_moveit_config_m0609`가 여기에
해당합니다. 이 repository는 driver package가 아니라 cup stacking
application package이므로 이름을 `cup_stack`으로 둡니다.

## Architecture

구조는 robot resource를 다루는 runtime layer와 실제 cup 동작을 표현하는
task layer를 분리합니다.

```text
cup_stack/
  src/cup_stack/
    config/
      moveit_py.yaml
    cup_stack/
      config.py
      geometry.py
      onrobot.py
      runtime.py
      nodes/
        cup_pyramid_node.py
        cup_unstack_node.py
      tasks/
        cup_pyramid.py
        cup_unstack.py
    launch/
      cup_pyramid.launch.py
      cup_unstack.launch.py
```

| Module | Responsibility |
| --- | --- |
| `config.py` | robot frame, gripper, cup geometry, timing parameter 정의 |
| `geometry.py` | z clamp, twist orientation 계산 |
| `runtime.py` | MoveItPy planning, `PoseStamped` goal 생성, gripper command 실행 |
| `tasks/cup_pyramid.py` | pyramid build sequence와 단계별 `try_*` method |
| `tasks/cup_unstack.py` | pyramid restore sequence와 단계별 `try_*` method |
| `nodes/*.py` | ROS 2 parameter를 읽고 task를 실행하는 entry point |

## ROS Interface

현재 node는 custom topic, service, action message를 직접 expose하지 않습니다.
외부 ROS interface는 launch parameter 중심입니다.

| Node | Executable | Parameter | Type | Default |
| --- | --- | --- | --- | --- |
| `cup_pyramid_node` | `cup_pyramid` | `nest_inc` | `float` | `0.0127` |
| `cup_unstack_node` | `cup_unstack` | `nest_inc` | `float` | `0.0127` |

MoveIt goal은 내부에서 `geometry_msgs/msg/PoseStamped`로 생성됩니다.

```text
PoseStamped
  header.frame_id = "base_link"
  pose.position.x = float
  pose.position.y = float
  pose.position.z = float
  pose.orientation.x = float
  pose.orientation.y = float
  pose.orientation.z = float
  pose.orientation.w = float
```

HOME move는 `moveit.core.robot_state.RobotState`를 사용하고, cartesian pose
move는 `PoseStamped`를 `MoveItPy` planning component에 전달합니다. Gripper는
OnRobot RG2/RG6 Modbus TCP command로 제어합니다.

## Task Interface

Python module에서 직접 호출할 때는 `CupStackRuntime`과 task class를 사용합니다.

```python
from cup_stack.runtime import CupStackRuntime
from cup_stack.tasks.cup_pyramid import CupPyramidTask

runtime = CupStackRuntime(node, "cup_pyramid_moveit_py")
task = CupPyramidTask(runtime, nest_inc=0.0127)
ok = task.try_execute()
```

각 task는 full sequence용 `try_execute()`와 단계별 control을 위한 `try_*`
method를 제공합니다. 실패 시 `False`를 반환하고 ROS logger에 실패 step을
남깁니다.

## Requirements

이 package는 ROS 2 Humble 기준의 Doosan MoveIt environment를 전제로 합니다.

| Dependency | Purpose |
| --- | --- |
| `rclpy` | ROS 2 Python node |
| `geometry_msgs` | `PoseStamped` goal |
| `moveit_py` | MoveItPy planning and execution |
| `dsr_moveit_config_m0609` | Doosan M0609 MoveIt config |
| `moveit_configs_utils` | launch-time MoveIt config builder |
| `python3-numpy` | transform and orientation math |
| `python3-pymodbus` | OnRobot RG gripper Modbus TCP |

Robot bringup은 이 package가 직접 수행하지 않습니다. 실행 전에 Doosan robot,
MoveIt, joint state publisher가 정상 동작해야 합니다.

## Build

```bash
cd /home/leo/development/ros2-cup-stack/cup_stack
colcon build --symlink-install
source install/setup.bash
```

개발 중 Python syntax만 빠르게 확인할 때는 다음 명령을 사용할 수 있습니다.

```bash
python3 -m compileall src/cup_stack
```

## Run

Pyramid build task:

```bash
ros2 launch cup_stack cup_pyramid.launch.py nest_inc:=0.0127
```

Pyramid restore task:

```bash
ros2 launch cup_stack cup_unstack.launch.py nest_inc:=0.0127
```

`cup_unstack`은 `cup_pyramid` 종료 위치를 기준으로 현재 end-effector FK를 읽고,
그 위치를 pyramid center로 가정합니다. 따라서 별도 위치에서 실행하면 source
stack 위치 계산이 달라질 수 있습니다.

## Safety Checklist

실제 robot에서 실행하기 전에 아래 항목을 확인해야 합니다.

| Check | Expected state |
| --- | --- |
| Robot mode | Doosan bringup이 real 또는 virtual mode로 정상 실행 |
| MoveIt | `dsr_moveit_config_m0609`와 planning pipeline 사용 가능 |
| Joint state | `/joint_states` publish 중 |
| End-effector frame | `link_6` transform 확인 가능 |
| Gripper | RG2 Modbus TCP `192.168.1.1:502` 연결 가능 |
| Workspace | cup, table, robot path 주변 충돌 위험 제거 |
| Emergency stop | operator가 즉시 접근 가능 |

## Development Notes

`skill`이라는 이름은 사용하지 않습니다. 이 package에서는 재사용 가능한 동작
단위를 `task`로 부릅니다.

기본 package path는 `src/cup_stack`이며, ROS package name도 `cup_stack`입니다.
Doosan driver package와 구분하기 위해 `dsr_*` prefix를 application package
이름에 붙이지 않습니다.

## License

이 project는 `MIT License`를 사용합니다. 자세한 내용은 `LICENSE` 파일을
확인하세요.
