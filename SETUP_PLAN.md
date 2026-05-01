# cup_stack structure plan

이 프로젝트는 기존 `dsr_practice`의 ament_python 패키지 양식을 유지하되,
컵 피라미드 쌓기와 되돌리기를 재사용 가능한 ROS 2 task 모듈로 분리한다.

`DSR`는 이 워크스페이스에서 Doosan Robotics/Doosan robot integration
stack의 드라이버 패키지 접두어로 쓰인다. 예: `dsr_bringup2`,
`dsr_msgs2`, `dsr_moveit_config_m0609`. 새 패키지는 드라이버가 아니라
컵 쌓기 애플리케이션이므로 패키지/폴더 이름을 `cup_stack`으로 둔다.

## 1. 패키지 골격

- 새 워크스페이스 폴더: `ros2/`
- ROS 2 패키지 경로: `ros2/src/cup_stack`
- 빌드 타입: `ament_python`
- 실행 진입점:
  - `cup_pyramid`
  - `cup_unstack`
- launch 파일:
  - `cup_pyramid.launch.py`
  - `cup_unstack.launch.py`

## 2. 모듈 분리 기준

- `config.py`: 로봇, 그리퍼, 컵 geometry 기본값
- `geometry.py`: z clamp, place twist quaternion, pyramid layout 계산
- `runtime.py`: MoveItPy, planner parameter, pose 이동, HOME 이동, 그리퍼 try 동작
- `tasks/cup_pyramid.py`: nested 컵을 3-2-1 피라미드로 쌓는 task
- `tasks/cup_unstack.py`: 3-2-1 피라미드를 nested 스택으로 되돌리는 task
- `nodes/*.py`: ROS 2 node와 task 모듈을 연결하는 얇은 실행 래퍼

## 3. Task 인터페이스

각 task는 외부 모듈에서 직접 호출할 수 있도록 `try_execute()`를 공개한다.
세부 단계도 `try_*` 메서드로 분리해서 상위 태스크 플래너가 필요한 단계만
호출하거나 테스트할 수 있게 한다.

예:

```python
runtime = CupStackRuntime(node, "cup_pyramid_moveit_py")
task = CupPyramidTask(runtime, nest_inc=0.0127)
ok = task.try_execute()
```

## 4. ROS 2 실행 방식

```bash
cd /home/leo/development/ros2-cup-stack/cup_stack/ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch cup_stack cup_pyramid.launch.py nest_inc:=0.0127
ros2 launch cup_stack cup_unstack.launch.py nest_inc:=0.0127
```

## 5. 검증 계획

- Python syntax 검증: `python3 -m compileall ros2/src/cup_stack`
- ROS 2 빌드 검증: `colcon build --symlink-install`
- 로봇 실행 전 확인:
  - Doosan bringup과 MoveIt config가 활성화되어 있는지 확인
  - `/joint_states`가 publish 되는지 확인
  - RG2 Modbus IP `192.168.1.1:502` 연결 확인
