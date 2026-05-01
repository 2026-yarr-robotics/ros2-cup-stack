# cup_stack 작동 구조

이 문서는 `ros2/src/cup_stack` 패키지의 폴더 구조, ROS 2 노드, 태스크 모듈, 메시지 흐름을 설명한다. `cup_stack`은 Doosan M0609 로봇과 RG 그리퍼를 사용해 스피드스택 컵을 6컵 피라미드로 쌓고 다시 원래 nested stack 형태로 되돌리는 제어 패키지다.

## 패키지 위치

```text
ros2/
└── src/
    ├── cup_stack/
    └── doosan-robot2/
```

`cup_stack`은 실제 작업 로직을 담는 ROS 2 `ament_python` 패키지이고, `doosan-robot2`는 DSR bringup, MoveIt 설정, 로봇 메시지 등 실행 기반을 제공하는 외부 레이어다.

## 주요 폴더 구조

```text
ros2/src/cup_stack/
├── cup_stack/
│   ├── nodes/          # ROS 2 실행 엔트리
│   ├── tasks/          # 컵 쌓기/되돌리기 동작 시퀀스
│   ├── config.py       # 로봇, 그리퍼, 컵 geometry 설정
│   ├── geometry.py     # orientation, z clamp 유틸
│   ├── onrobot.py      # RG 그리퍼 Modbus 제어
│   ├── runtime.py      # MoveItPy와 그리퍼 공통 실행 어댑터
│   └── vision.py       # 카메라 클릭 좌표 선택
├── config/
│   ├── moveit_py.yaml
│   └── T_gripper2camera.npy
├── launch/
├── test/
├── package.xml
└── setup.py
```

## 노드 구조

`setup.py`의 `console_scripts`로 4개 실행 파일을 제공한다.

| 실행 파일 | 노드 이름 | 역할 |
| --- | --- | --- |
| `cup_pyramid` | `cup_pyramid_node` | 현재 HOME TCP XY를 nested cup 위치로 보고 6컵 피라미드를 만든다. |
| `cup_pyramid_select` | `cup_pyramid_select_node` | 카메라 화면에서 nested cup 위치를 클릭한 뒤 피라미드를 만든다. |
| `cup_unstack` | `cup_unstack_node` | 현재 TCP XY를 피라미드 중심으로 보고 컵을 원래 nested stack으로 되돌린다. |
| `cup_unstack_select` | `cup_unstack_select_node` | 카메라 화면에서 피라미드 중심을 클릭한 뒤 원래 위치로 되돌린다. |

선택형 노드는 `MultiThreadedExecutor`를 백그라운드에서 돌려 카메라 토픽 콜백을 계속 처리하면서 OpenCV 클릭 UI를 유지한다.

## 태스크 구조

`tasks/cup_pyramid.py`의 `CupPyramidTask`는 6개의 컵을 nested stack에서 하나씩 집어 `3-2-1` 피라미드 위치에 배치한다. 기준 위치는 `pick_xy`가 주어지면 선택 좌표를 사용하고, 없으면 현재 end-effector XY를 사용한다.

`tasks/cup_unstack.py`의 `CupUnstackTask`는 피라미드의 상단부터 역순으로 컵을 집어 nested stack 위치로 되돌린다. 기준 위치는 `pyramid_xy`가 주어지면 선택 좌표를 사용하고, 없으면 현재 end-effector XY를 피라미드 중심으로 사용한다.

두 태스크는 직접 ROS 메시지를 publish/subscribe하지 않는다. 이동과 그리퍼 동작은 `CupStackRuntime`을 통해 수행한다.

## Runtime 구조

`CupStackRuntime`은 노드와 태스크 사이의 실행 계층이다.

- `MoveItPy`를 초기화하고 `manipulator` planning component를 사용한다.
- HOME 이동은 `RobotState` 목표로 계획한다.
- pose 이동은 `geometry_msgs/msg/PoseStamped` 목표를 만들고 MoveIt planning에 전달한다.
- 일반 XY 이동은 Pilz `PTP`, 수직 접근/상승은 Pilz `LIN`을 우선 사용한다.
- `LIN` 계획 실패 시 `strict=False`인 동작은 `PTP`로 재시도한다.
- RG 그리퍼는 `onrobot.RG`를 통해 Modbus로 open, grip, release 명령을 보낸다.

## 메시지와 토픽 구조

현재 패키지는 커스텀 ROS 메시지를 정의하지 않는다.

| 구분 | 타입 | 사용 위치 | 설명 |
| --- | --- | --- | --- |
| MoveIt pose goal | `geometry_msgs/msg/PoseStamped` | `runtime.py` | base frame 기준 목표 TCP pose를 MoveIt에 전달한다. |
| Camera info | `sensor_msgs/msg/CameraInfo` | `vision.py` | `fx`, `fy`, `ppx`, `ppy` 내부 파라미터를 읽는다. |
| Color image | `sensor_msgs/msg/Image` | `vision.py` | OpenCV 클릭 UI 표시용 RGB/BGR 이미지다. |
| Depth image | `sensor_msgs/msg/Image` | `vision.py` | 클릭 픽셀의 depth 값을 base 좌표로 변환할 때 사용한다. |

구독 토픽 기본값은 `CameraConfig`에 정의되어 있다.

```text
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
```

## 좌표 선택 흐름

`vision.py`의 `CameraClickSelector`는 색상 이미지에 OpenCV 창을 띄우고 사용자의 클릭을 받는다. 클릭 픽셀의 depth가 0이면 주변 반경에서 유효한 depth를 찾고, 가까운 표면을 우선하기 위해 유효 depth의 25 percentile 근처 값을 사용한다.

픽셀 좌표는 다음 순서로 변환된다.

```text
pixel + depth
→ camera frame 3D point
→ current base_to_ee
→ T_gripper2camera.npy
→ base frame point
```

선택형 stack은 base frame의 `(x, y)`를 nested cup 위치로 넘긴다. 선택형 unstack은 base frame의 `(x, y)`를 피라미드 중심으로 넘긴다. `z`는 클릭 좌표 계산에는 사용하지만 실제 컵 pick/place 높이는 `CupStackConfig`의 고정 geometry 값을 따른다.

## 실행 흐름

일반적인 실행 순서는 다음과 같다.

```bash
cd ros2
colcon build --symlink-install
source install/setup.bash
```

시뮬레이션 bringup:

```bash
ros2/src/cup_stack/bringup_sim.sh
```

실로봇 bringup:

```bash
ros2/src/cup_stack/bringup_real.sh 192.168.1.100
```

태스크 실행:

```bash
ros2 launch cup_stack cup_pyramid.launch.py
ros2 launch cup_stack cup_pyramid_select.launch.py
ros2 launch cup_stack cup_unstack.launch.py
ros2 launch cup_stack cup_unstack_select.launch.py
```

`nest_inc`는 nested cup 사이의 높이 증가량이며 launch 인자로 조정할 수 있다.

```bash
ros2 launch cup_stack cup_pyramid_select.launch.py nest_inc:=0.0127
```

## 동작 기준 좌표

`CupStackConfig`의 주요 기준은 다음과 같다.

- `pick_z_base`: nested stack 최하단 컵을 집을 때의 기준 높이
- `place_z_base`: 피라미드 바닥층 배치 높이
- `place_x_offset`: nested stack 기준 피라미드 중심을 X+ 방향으로 떨어뜨리는 거리
- `cup_spacing`: 피라미드 내 컵 사이 Y 간격
- `layer_height`: 피라미드 층간 높이
- `pyramid_places`: stack 순서의 피라미드 배치 오프셋
- `reverse_picks`: unstack 순서의 피라미드 픽업 오프셋

따라서 stack과 unstack은 같은 geometry 설정을 공유해야 한다. 컵 크기, 그리퍼 파지 위치, 작업대 높이가 바뀌면 `config.py`를 먼저 조정한다.

## 패키지 분리 기준

`cup_stack`에 남겨야 하는 코드는 스피드스택 컵 쌓기, 원래대로 되돌리기, 카메라 좌표 선택과 직접 관련된 코드다. 음성 제어, 일반 pick/place 데모, teleop, calibration tutorial, fallen cup 복구 로직은 현재 동작 범위 밖이므로 독립 패키지 유지에 필요하지 않다.
