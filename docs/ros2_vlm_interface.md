# ROS 2 토픽·메시지·서비스 & 카메라 좌표 변환 (VLM) 인터페이스

이 문서는 `server` ↔ `ROS 2` 간의 토픽·메시지·서비스 인터페이스와, 카메라 픽셀 좌표를 로봇 base frame 좌표로 변환하는 VLM 파이프라인을 정의한다.

---

## 목차

1. [ROS 2 토픽 및 메시지](#1-ros-2-토픽-및-메시지)
2. [ROS 2 서비스](#2-ros-2-서비스)
3. [노드 및 launch 파라미터](#3-노드-및-launch-파라미터)
4. [카메라 좌표 변환 (VLM) 인터페이스](#4-카메라-좌표-변환-vlm-인터페이스)

---

## 1. ROS 2 토픽 및 메시지

모든 ROS 2 통신은 rosbridge WebSocket(포트 9090)을 통해 서버의 `RosBridge` 싱글턴이 대행한다.

### 1.1 서버 → ROS 2 (구독)

| 토픽 | 메시지 타입 | 사용처 | throttle |
|------|-------------|--------|----------|
| `/joint_states` | `sensor_msgs/msg/JointState` | `RobotDomain` → `/ws/robot/state` | 100 ms |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `CameraStream` → `/ws/camera/handineye` | 33 ms |
| `/camera/fixed_camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `CameraStream` → `/ws/camera/handtoeye` | 33 ms |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `HandInEyeDomain` 내부 파라미터 | — |
| `/camera/fixed_camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `HandToEyeDomain` 내부 파라미터 | — |

#### `sensor_msgs/msg/JointState` 필드

```
std_msgs/Header header
string[]         name
float64[]        position   # 라디안
float64[]        velocity   # 라디안/s
float64[]        effort     # N·m
```

#### `sensor_msgs/msg/CameraInfo` 필드 (사용 부분)

```
float64[9] k   # 3×3 내부 파라미터 행렬 (row-major)
               # k[0]=fx, k[4]=fy, k[2]=ppx, k[5]=ppy
```

#### `sensor_msgs/msg/CompressedImage` 필드 (사용 부분)

```
string  format   # "jpeg" 또는 "png"
uint8[] data     # base64 인코딩된 이미지 데이터 (rosbridge JSON 전송 시)
```

---

### 1.2 ROS 2 노드 구독 토픽 (cup_stack 내부)

`CameraClickSelector`(`vision.py`)가 직접 구독하는 토픽.

| 토픽 | 메시지 타입 | 설명 |
|------|-------------|------|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `fx`, `fy`, `ppx`, `ppy` 추출 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | OpenCV 클릭 UI 표시용 BGR 이미지 |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 픽셀 depth 값 조회 |

기본값은 `config.py`의 `CameraConfig`에서 정의한다.

#### `sensor_msgs/msg/Image` 필드 (사용 부분)

```
string encoding     # color: "bgr8", depth: "passthrough" (16UC1, mm 단위)
uint8[] data
```

depth 이미지는 uint16, 단위는 밀리미터.

---

## 2. ROS 2 서비스

`move_cartesian_node`가 제공하는 서비스. 서버는 rosbridge의 `call_service`로 호출한다.

### 2.1 `/move_cartesian`

**서비스 타입**: `cup_stack_interfaces/srv/MoveCartesian`

**Request**

| 필드   | 타입     | 설명 |
|--------|----------|------|
| `x`    | `float64`| base_link 기준 X (m) |
| `y`    | `float64`| base_link 기준 Y (m) |
| `z`    | `float64`| base_link 기준 Z (m) |
| `mode` | `string` | `"absolute"` \| `"relative"` |

**Response**

| 필드      | 타입     | 설명 |
|-----------|----------|------|
| `success` | `bool`   | MoveItPy 플래닝 및 실행 성공 여부 |
| `message` | `string` | 결과 설명 |

`mode="relative"` 시 현재 EE 위치를 FK로 읽어 delta를 더한 뒤 실행한다.  
이동 실패 시에도 ROS 서비스 레벨은 성공(`success=false` 반환). HTTP 레벨에서 409가 발생하는 것은 서비스 연결 자체 실패(`RuntimeError`)인 경우다.

---

### 2.2 `/gripper_control`

**서비스 타입**: `cup_stack_interfaces/srv/GripperControl`

**Request**

| 필드      | 타입     | 값 |
|-----------|----------|----|
| `command` | `string` | `"open"` \| `"close"` |

**Response**

| 필드      | 타입     | 설명 |
|-----------|----------|------|
| `success` | `bool`   | 그리퍼 동작 성공 여부 |
| `message` | `string` | 결과 설명 (`"Gripper opened"`, `"Gripper hardware not connected (192.168.1.1:502)"` 등) |

그리퍼 하드웨어(IP `192.168.1.1:502`)가 미연결인 경우에도 서비스는 등록되며, `success=false`를 반환한다.  
`cup_stack_interfaces` 패키지가 빌드되지 않은 경우 서비스 자체가 등록되지 않는다.

---

## 3. 노드 및 launch 파라미터

### 3.1 노드 목록

| 실행 파일 | 노드 이름 | 역할 |
|-----------|-----------|------|
| `cup_pyramid` | `cup_pyramid_node` | HOME XY 기준 6컵 피라미드 빌드 |
| `cup_unstack` | `cup_unstack_node` | HOME XY 기준 피라미드 언스택 |
| `cup_pyramid_select` | `cup_pyramid_select_node` | OpenCV 클릭으로 nested stack 위치 선택 후 빌드 |
| `cup_unstack_select` | `cup_unstack_select_node` | OpenCV 클릭으로 피라미드 중심 선택 후 언스택 |
| `cup_pyramid_web` | `cup_pyramid_web_node` | 픽셀 좌표 파라미터로 피라미드 빌드 |
| `cup_unstack_web` | `cup_unstack_web_node` | 픽셀 좌표 파라미터로 피라미드 언스택 |
| `move_cartesian` | `move_cartesian_node` | `/move_cartesian` + `/gripper_control` ROS 서비스 제공 |
| `camera_capture` | `camera_capture_node` | 카메라 라이브 뷰 + 키입력으로 프레임 저장 |

---

### 3.2 launch 파라미터

#### 공통 (cup_pyramid / cup_unstack 계열)

| 파라미터   | 타입    | 기본값   | 설명 |
|------------|---------|----------|------|
| `nest_inc` | `float` | `0.0127` | nested cup 높이 증가량 (m) |

#### `cup_pyramid_web` / `cup_unstack_web` 전용

| 파라미터  | 타입  | 기본값 | 설명 |
|-----------|-------|--------|------|
| `pixel_x` | `int` | `-1`   | 대시보드 클릭 픽셀 X. 음수이면 에러 종료 |
| `pixel_y` | `int` | `-1`   | 대시보드 클릭 픽셀 Y. 음수이면 에러 종료 |

서버는 `/api/robot/task/start`의 `args`를 `ros2 launch cup_stack {task}.launch.py key:=value` 형식으로 전달한다.

#### `camera_capture` 전용

| 파라미터   | 타입     | 기본값        | 설명 |
|------------|----------|---------------|------|
| `save_dir` | `string` | `~/captures`  | PNG 저장 디렉토리. [S] 키로 캡처 |

#### `move_cartesian` 전용

launch 파라미터 없음. MoveItPy 파라미터(`dsr_moveit_config_m0609`)와 `link_6 → camera_link` 정적 TF를 자동 로드한다.

---

## 4. 카메라 좌표 변환 (VLM) 인터페이스

대시보드 카메라 화면의 픽셀 클릭 좌표를 로봇 base frame의 3D 좌표로 변환하는 파이프라인.  
`cup_pyramid_web` / `cup_unstack_web` 태스크에서 사용한다.

### 4.1 입출력

**입력**

| 항목      | 타입  | 설명 |
|-----------|-------|------|
| `pixel_x` | `int` | 카메라 이미지 픽셀 X (0이 왼쪽) |
| `pixel_y` | `int` | 카메라 이미지 픽셀 Y (0이 위쪽) |

**출력**

| 항목     | 타입    | 단위 | 설명 |
|----------|---------|------|------|
| `base_x` | `float` | m    | `base_link` 프레임 X |
| `base_y` | `float` | m    | `base_link` 프레임 Y |
| `base_z` | `float` | m    | `base_link` 프레임 Z (참고용, 실제 픽/플레이스 Z는 `CupStackConfig` 사용) |

변환 실패 시 `None` 반환.

---

### 4.2 변환 파이프라인

```
픽셀 (pixel_x, pixel_y)
        │
        ▼
[1] depth 탐색
    depth[pixel_y, pixel_x]가 0이면
    반경 30 px 내에서 유효 depth 탐색
    (유효값의 25th percentile 근처 값 선택)
        │
        ▼ z_mm (uint16, 밀리미터)
[2] 카메라 내부 파라미터 적용
    z_m = z_mm / 1000.0
    cam_x = (pixel_x - ppx) * z_m / fx
    cam_y = (pixel_y - ppy) * z_m / fy
    cam_point = [cam_x, cam_y, z_m, 1.0]
        │
        ▼
[3] 좌표 변환
    base_to_camera = T_ee_in_base  ×  T_gripper2camera
    base_point = base_to_camera  ×  cam_point
        │
        ▼
    (base_x, base_y, base_z)  [m, base_link 프레임]
```

---

### 4.3 캘리브레이션 파일

| 파일명 | 형식 | 단위 | 설명 |
|--------|------|------|------|
| `T_gripper2camera.npy` | `numpy` 4×4 `float64` | mm (번역 부분) | EE → 카메라 동차 변환. 로드 시 `[:3, 3] /= 1000.0`으로 m 변환 |

파일 위치: `cup_stack/ros2/src/cup_stack/config/T_gripper2camera.npy`

`move_cartesian.launch.py`에 정의된 정적 TF(`link_6 → camera_link`)도 동일 캘리브레이션에서 유래한다:
- Translation: `(0.0384, 0.05885, -0.005715)` m
- Quaternion (xyzw): `(0.002446, -0.003002, 0.999861, 0.016209)`

---

### 4.4 카메라 내부 파라미터 (CameraInfo K 행렬)

```
K = [fx,  0, ppx,
      0, fy, ppy,
      0,  0,   1]
```

| 기호  | K 인덱스 | 설명 |
|-------|----------|------|
| `fx`  | `K[0]`   | X축 초점 거리 (px) |
| `fy`  | `K[4]`   | Y축 초점 거리 (px) |
| `ppx` | `K[2]`   | 주점 X (px) |
| `ppy` | `K[5]`   | 주점 Y (px) |

---

### 4.5 전체 흐름 — 웹 카메라 선택 태스크

```
[Browser]
  1. 사용자가 Eye-in-Hand 카메라 피드 클릭
  2. displayX/Y → 실제 카메라 픽셀 (naturalWidth/Height 스케일링)
  3. handleCameraClick({ px, py })
  4. POST /api/robot/task/start
     body: { task: "cup_pyramid_web", args: { pixel_x: "320", pixel_y: "240" } }

[Server]
  5. LaunchManager.start("cup_pyramid_web", { pixel_x: "320", pixel_y: "240" })
  6. ros2 launch cup_stack cup_pyramid_web.launch.py pixel_x:=320 pixel_y:=240

[ROS 2 Node: cup_pyramid_web_node]
  7. node.get_parameter("pixel_x").value → 320
  8. CameraClickSelector.pixel_to_base(320, 240)
     depth → cam_point → T_gripper2camera → T_ee_in_base → (base_x, base_y, base_z)
  9. CupPyramidTask.try_execute(pick_xy=(base_x, base_y))
```
