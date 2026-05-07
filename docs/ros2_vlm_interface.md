# ROS 2 토픽·메시지 & 카메라 좌표 변환 (VLM) 인터페이스

이 문서는 `server` ↔ `ROS 2` 간의 토픽·메시지 인터페이스와, 카메라 픽셀 좌표를 로봇 base frame 좌표로 변환하는 VLM 파이프라인을 정의한다.

---

## 목차

1. [ROS 2 토픽 및 메시지](#1-ros-2-토픽-및-메시지)
2. [카메라 좌표 변환 (VLM) 인터페이스](#2-카메라-좌표-변환-vlm-인터페이스)

---

## 1. ROS 2 토픽 및 메시지

모든 ROS 2 통신은 rosbridge WebSocket(포트 9090)을 통해 서버의 `RosBridge` 싱글턴이 대행한다. 서버 노드는 ROS 2 노드를 직접 실행하지 않는다.

### 1.1 서버 → ROS 2 (구독)

| 토픽 | 메시지 타입 | 사용처 | throttle |
|------|-------------|--------|----------|
| `/joint_states` | `sensor_msgs/msg/JointState` | `RobotDomain` → `/ws/robot/state` | 100 ms |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `CameraStream` → `/ws/camera/handineye` | 33 ms |
| `/fixed_camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `CameraStream` → `/ws/camera/handtoeye` | 33 ms |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `HandInEyeDomain` 내부 파라미터 | — |
| `/fixed_camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `HandToEyeDomain` 내부 파라미터 | — |

#### `sensor_msgs/msg/JointState` 필드

```
std_msgs/Header header
string[]         name
float64[]        position   # 라디안
float64[]        velocity   # 라디안/s
float64[]        effort     # N·m
```

서버가 추출하는 필드: `name`, `position`, `velocity`, `effort`

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

`cup_stack` 노드들이 직접 구독하는 토픽 목록이다. `CameraClickSelector`(`vision.py`)에서 사용한다.

| 토픽 | 메시지 타입 | 설명 |
|------|-------------|------|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `fx`, `fy`, `ppx`, `ppy` 파라미터 추출 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | OpenCV 클릭 UI 표시용 BGR 이미지 |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 픽셀 depth 값 조회 |

기본값은 `config.py`의 `CameraConfig`에서 정의한다. 토픽 이름은 노드 파라미터로 변경 불가능하며, 코드 수정 또는 `CameraConfig` 인스턴스 재정의가 필요하다.

#### `sensor_msgs/msg/Image` 필드 (사용 부분)

```
std_msgs/Header header
uint32 height
uint32 width
string encoding     # color: "bgr8", depth: "passthrough" (16UC1, mm 단위)
uint8[] data
```

depth 이미지는 16-bit unsigned (uint16), 단위는 밀리미터.

---

### 1.3 launch 파라미터

#### 공통 파라미터

| 파라미터    | 타입    | 기본값   | 설명 |
|-------------|---------|----------|------|
| `nest_inc`  | `float` | `0.0127` | nested cup 높이 증가량 (m). 컵 겹침 두께에 맞춰 조정 |

#### `cup_pyramid_web` / `cup_unstack_web` 전용

| 파라미터  | 타입  | 기본값 | 설명 |
|-----------|-------|--------|------|
| `pixel_x` | `int` | `-1`   | 대시보드 클릭 픽셀 X. 음수이면 노드가 에러로 종료 |
| `pixel_y` | `int` | `-1`   | 대시보드 클릭 픽셀 Y. 음수이면 노드가 에러로 종료 |

서버는 `/api/robot/task/start`의 `args`를 `ros2 launch {pkg} {task}.launch.py pixel_x:={v} pixel_y:={v}` 형식으로 전달한다.

---

## 2. 카메라 좌표 변환 (VLM) 인터페이스

대시보드 카메라 화면의 픽셀 클릭 좌표를 로봇 base frame의 3D 좌표로 변환하는 파이프라인이다. `cup_pyramid_web` / `cup_unstack_web` 태스크에서 사용한다.

### 2.1 입출력

**입력**

| 항목      | 타입  | 설명 |
|-----------|-------|------|
| `pixel_x` | `int` | 카메라 이미지 픽셀 X (0이 왼쪽) |
| `pixel_y` | `int` | 카메라 이미지 픽셀 Y (0이 위쪽) |

**출력**

| 항목      | 타입    | 단위 | 설명 |
|-----------|---------|------|------|
| `base_x`  | `float` | m    | `base_link` 프레임 X 좌표 |
| `base_y`  | `float` | m    | `base_link` 프레임 Y 좌표 |
| `base_z`  | `float` | m    | `base_link` 프레임 Z 좌표 (참고용, 실제 픽/플레이스 Z는 `CupStackConfig` 사용) |

변환 실패 시 `None` 반환 (depth 정보 없음, 카메라 미준비 등).

---

### 2.2 변환 파이프라인

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
    cam_point = [cam_x, cam_y, z_m, 1.0]  (동차 좌표)
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

### 2.3 캘리브레이션 파일

| 파일명 | 형식 | 단위 | 설명 |
|--------|------|------|------|
| `T_gripper2camera.npy` | `numpy` 4×4 `float64` | mm (번역 부분) | EE → 카메라 동차 변환. 로드 시 `[:3, 3] /= 1000.0`으로 m 변환 |

파일 위치: `cup_stack/ros2/src/cup_stack/config/T_gripper2camera.npy`

---

### 2.4 카메라 내부 파라미터 (CameraInfo K 행렬)

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

서버 측(`HandInEyeDomain`, `HandToEyeDomain`)과 ROS 노드 측(`CameraClickSelector`) 모두 동일한 인덱싱 규칙을 사용한다.

---

### 2.5 전체 흐름 — 웹 카메라 선택 태스크

```
[Browser]
  1. 사용자가 Eye-in-Hand 카메라 피드 클릭
  2. displayX, displayY → 실제 카메라 픽셀 (naturalWidth/Height 스케일링)
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
