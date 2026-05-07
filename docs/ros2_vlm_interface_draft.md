# ROS 2 토픽·메시지·서비스 & 컵 감지(YOLO) 파이프라인 인터페이스 (Draft)

이 문서는 `server` ↔ `ROS 2` 간의 토픽·메시지·서비스 인터페이스와,  
카메라 픽셀 좌표를 로봇 base frame 좌표로 변환하는 파이프라인 및  
YOLO 기반 컵 객체 인식 파이프라인을 정의한다.

---

## 목차

1. [전체 토픽 맵](#1-전체-토픽-맵)
2. [커스텀 메시지 타입](#2-커스텀-메시지-타입)
3. [ROS 2 서비스](#3-ros-2-서비스)
4. [노드 목록](#4-노드-목록)
5. [Launch 파라미터](#5-launch-파라미터)
6. [카메라 좌표 변환 파이프라인](#6-카메라-좌표-변환-파이프라인)
7. [YOLO 컵 감지 파이프라인](#7-yolo-컵-감지-파이프라인)
8. [전체 데이터 흐름](#8-전체-데이터-흐름)

---

## 1. 전체 토픽 맵

모든 ROS 2 통신은 rosbridge WebSocket(포트 9090)을 통해 서버의 `RosBridge` 싱글턴이 대행한다.

### 1.1 서버 → ROS 2 구독 토픽

| 토픽 | 메시지 타입 | 용도 | throttle |
|------|-------------|------|----------|
| `/joint_states` | `sensor_msgs/msg/JointState` | `RobotDomain` → `/ws/robot/state` | 100 ms |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `CameraStream` → `/ws/camera/handineye` | 33 ms |
| `/camera/fixed_camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `CameraStream` → `/ws/camera/handtoeye` | 33 ms |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `HandInEyeDomain` 내부 파라미터 | — |
| `/camera/fixed_camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `HandToEyeDomain` 내부 파라미터 | — |
| `/cup_poses` | `cup_stack_interfaces/msg/CupPoseArray` | `CupDetectionDomain` → `/ws/cups`, `GET /api/robot/cups` | — |

### 1.2 ROS 2 내부 토픽 (rosbridge 미경유)

아래 토픽은 ROS 2 노드 간에만 흐르며 서버는 직접 구독하지 않는다.

| 토픽 | 퍼블리셔 | 구독자 | 설명 |
|------|----------|--------|------|
| `/camera/camera/color/image_raw` | Intel RealSense | `yolo_detector_node` | YOLO 입력용 비압축 컬러 이미지 |
| `/camera/camera/aligned_depth_to_color/image_raw` | Intel RealSense | `cup_pose_estimator_node` | depth → 3D 변환용 (uint16, mm) |
| `/camera/camera/color/camera_info` | Intel RealSense | `cup_pose_estimator_node`, `cup_pyramid_web_node`, `cup_unstack_web_node` | 카메라 내부 파라미터 |
| `/cup_detections` | `yolo_detector_node` | `cup_pose_estimator_node` | YOLO 2D 감지 결과 |
| `/cup_poses` | `cup_pose_estimator_node` | rosbridge (서버 구독), `cup_pyramid_web_node`, `cup_unstack_web_node` | 컵 3D 포즈 배열 |

---

### 1.3 표준 메시지 필드 요약

#### `sensor_msgs/msg/JointState`

```
std_msgs/Header header
string[]  name
float64[] position   # 라디안
float64[] velocity   # 라디안/s
float64[] effort     # N·m
```

#### `sensor_msgs/msg/CameraInfo` (사용 부분)

```
float64[9] k   # 3×3 내부 파라미터 (row-major)
               # k[0]=fx, k[4]=fy, k[2]=ppx, k[5]=ppy
```

#### `sensor_msgs/msg/CompressedImage` (사용 부분)

```
string  format   # "jpeg" 또는 "png"
uint8[] data     # base64 인코딩 (rosbridge JSON 전송 시)
```

#### `sensor_msgs/msg/Image` (사용 부분)

```
string encoding   # color: "bgr8", depth: "passthrough" (16UC1, mm 단위)
uint8[] data
```

depth 이미지: uint16, 단위 밀리미터.

---

## 2. 커스텀 메시지 타입

패키지: `cup_stack_interfaces`  
위치: `cup_stack/ros2/src/cup_stack_interfaces/msg/`

### 2.1 `CupDetection.msg`

YOLO 1개 컵 감지 결과 (픽셀 공간).

```
string    cup_id       # 프레임 내 고유 ID: "cup_0", "cup_1", ...
string    label        # YOLO 클래스 레이블 ("cup")
float32   confidence   # 신뢰도 [0.0, 1.0]
uint32[4] bbox         # [x_min, y_min, x_max, y_max] 픽셀 좌표
float32   cx           # bbox 중심 픽셀 X
float32   cy           # bbox 중심 픽셀 Y
```

### 2.2 `CupDetectionArray.msg`

한 프레임의 전체 2D 감지 결과.

```
std_msgs/Header  header      # stamp + frame_id = "camera_link"
CupDetection[]   detections
```

### 2.3 `CupPose.msg`

3D 좌표로 변환된 컵 1개 정보.

```
string              cup_id       # CupDetection과 동일 ID
string              label        # YOLO 클래스 레이블
float32             confidence   # 신뢰도
geometry_msgs/Point position     # base_link 기준 3D 좌표 (m)
float32             cx           # 원본 픽셀 중심 X (3D 변환 실패 시 픽셀만 사용 가능)
float32             cy           # 원본 픽셀 중심 Y
uint32[4]           bbox         # 원본 픽셀 bbox [x_min, y_min, x_max, y_max]
bool                pose_valid   # false이면 depth 조회 실패로 position이 유효하지 않음
```

### 2.4 `CupPoseArray.msg`

한 프레임의 전체 3D 컵 포즈 배열. 서버가 rosbridge로 이 토픽을 구독한다.

```
std_msgs/Header  header    # stamp + frame_id = "base_link"
CupPose[]        poses
```

> **설계 의도**  
> - `CupDetectionArray`는 순수 YOLO 출력(픽셀 공간)으로 노드 간 의존성을 낮춘다.  
> - `CupPoseArray`는 3D 변환까지 완료된 최종 결과로, 서버와 task 노드가 모두 이를 소비한다.  
> - `pose_valid=false` 컵은 bbox 오버레이 표시는 하되, 3D 좌표 기반 pick/place에서는 제외한다.

---

## 3. ROS 2 서비스

`move_cartesian_node`가 제공하는 서비스.  
서버는 rosbridge의 `call_service`로 호출한다.

### 3.1 `/move_cartesian`

**서비스 타입**: `cup_stack_interfaces/srv/MoveCartesian`

**Request**

| 필드 | 타입 | 설명 |
|------|------|------|
| `x` | `float64` | base_link 기준 X (m) |
| `y` | `float64` | base_link 기준 Y (m) |
| `z` | `float64` | base_link 기준 Z (m) |
| `mode` | `string` | `"absolute"` \| `"relative"` |

**Response**

| 필드 | 타입 | 설명 |
|------|------|------|
| `success` | `bool` | MoveItPy 플래닝 및 실행 성공 여부 |
| `message` | `string` | 결과 설명 |

`mode="relative"`: 현재 EE 위치를 FK로 읽어 delta를 더한 뒤 실행.  
이동 실패 시에도 ROS 서비스 레벨은 성공 반환 (`success=false`).

### 3.2 `/gripper_control`

**서비스 타입**: `cup_stack_interfaces/srv/GripperControl`

**Request**

| 필드 | 타입 | 값 |
|------|------|----|
| `command` | `string` | `"open"` \| `"close"` |

**Response**

| 필드 | 타입 | 설명 |
|------|------|------|
| `success` | `bool` | 그리퍼 동작 성공 여부 |
| `message` | `string` | 결과 설명 |

그리퍼 하드웨어(IP `192.168.1.1:502`) 미연결 시에도 서비스는 등록되며 `success=false` 반환.

---

## 4. 노드 목록

### 4.1 기존 노드

| 실행 파일 | 노드 이름 | 역할 |
|-----------|-----------|------|
| `cup_pyramid` | `cup_pyramid_node` | HOME XY 기준 6컵 피라미드 빌드 |
| `cup_unstack` | `cup_unstack_node` | HOME XY 기준 피라미드 언스택 |
| `cup_pyramid_select` | `cup_pyramid_select_node` | OpenCV 클릭으로 위치 선택 후 빌드 |
| `cup_unstack_select` | `cup_unstack_select_node` | OpenCV 클릭으로 피라미드 중심 선택 후 언스택 |
| `cup_pyramid_web` | `cup_pyramid_web_node` | 픽셀 좌표 파라미터로 피라미드 빌드 |
| `cup_unstack_web` | `cup_unstack_web_node` | 픽셀 좌표 파라미터로 피라미드 언스택 |
| `move_cartesian` | `move_cartesian_node` | `/move_cartesian` + `/gripper_control` 서비스 제공 |
| `camera_capture` | `camera_capture_node` | 카메라 라이브 뷰 + [S] 키로 PNG 저장 |

### 4.2 신규 노드 (YOLO 파이프라인)

| 실행 파일 | 노드 이름 | 역할 |
|-----------|-----------|------|
| `yolo_detector` | `yolo_detector_node` | 컬러 이미지에서 YOLO 컵 감지 → `/cup_detections` 퍼블리시 |
| `cup_pose_estimator` | `cup_pose_estimator_node` | `/cup_detections` + depth + camera_info → 3D 변환 → `/cup_poses` 퍼블리시 |

이 두 노드는 `cup_detection.launch.py`로 묶어 함께 기동한다.

---

## 5. Launch 파라미터

### 5.1 공통 (cup_pyramid / cup_unstack 계열)

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `nest_inc` | `float` | `0.0127` | nested cup 높이 증가량 (m) |

### 5.2 `cup_pyramid_web` / `cup_unstack_web` 전용

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `pixel_x` | `int` | `-1` | 클릭 픽셀 X. 음수이면 에러 종료 |
| `pixel_y` | `int` | `-1` | 클릭 픽셀 Y. 음수이면 에러 종료 |

서버는 `/api/robot/task/start` 또는 `/api/robot/cups/trigger`의 인자를  
`ros2 launch cup_stack {task}.launch.py key:=value` 형식으로 전달한다.

### 5.3 `move_cartesian` 전용

launch 파라미터 없음. MoveItPy 파라미터(`dsr_moveit_config_m0609`)와  
`link_6 → camera_link` 정적 TF를 자동 로드한다.

### 5.4 `camera_capture` 전용

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `save_dir` | `string` | `~/captures` | PNG 저장 디렉토리. [S] 키로 캡처 |

### 5.5 `yolo_detector` 전용 (신규)

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `model_path` | `string` | `share/cup_stack/config/cup_yolo.pt` | YOLO 모델 가중치 파일 경로 |
| `confidence_threshold` | `float` | `0.5` | 감지 최소 신뢰도. 이하 박스는 무시 |
| `device` | `string` | `"cpu"` | 추론 장치 (`"cpu"`, `"cuda:0"` 등) |
| `image_topic` | `string` | `/camera/camera/color/image_raw` | 입력 컬러 이미지 토픽 |
| `output_topic` | `string` | `/cup_detections` | 출력 감지 결과 토픽 |

### 5.6 `cup_pose_estimator` 전용 (신규)

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `detection_topic` | `string` | `/cup_detections` | 입력 감지 결과 토픽 |
| `depth_topic` | `string` | `/camera/camera/aligned_depth_to_color/image_raw` | 입력 depth 토픽 |
| `camera_info_topic` | `string` | `/camera/camera/color/camera_info` | 카메라 내부 파라미터 토픽 |
| `output_topic` | `string` | `/cup_poses` | 출력 3D 포즈 토픽 |
| `depth_search_radius` | `int` | `30` | depth=0인 픽셀 주변 탐색 반경 (px) |

---

## 6. 카메라 좌표 변환 파이프라인

대시보드 카메라 클릭 픽셀 또는 YOLO 감지 중심 픽셀을  
로봇 base_link 3D 좌표로 변환하는 공통 파이프라인.  
`cup_pyramid_web`, `cup_unstack_web`, `cup_pose_estimator_node`에서 사용한다.

### 6.1 입출력

**입력**

| 항목 | 타입 | 설명 |
|------|------|------|
| `pixel_x` | `int` | 컬러 이미지 픽셀 X (0이 왼쪽) |
| `pixel_y` | `int` | 컬러 이미지 픽셀 Y (0이 위쪽) |

**출력**

| 항목 | 타입 | 단위 | 설명 |
|------|------|------|------|
| `base_x` | `float` | m | `base_link` 기준 X |
| `base_y` | `float` | m | `base_link` 기준 Y |
| `base_z` | `float` | m | `base_link` 기준 Z (참고용; pick/place Z는 `CupStackConfig` 값 사용) |

변환 실패(유효 depth 없음) 시 `None` 반환.

### 6.2 변환 파이프라인

```
픽셀 (pixel_x, pixel_y)
        │
        ▼
[1] depth 탐색
    depth[pixel_y, pixel_x] == 0 이면
    반경 depth_search_radius px 내에서 유효 depth 탐색
    (유효값의 25th percentile 값 선택)
        │
        ▼ z_mm (uint16, mm)
[2] 카메라 내부 파라미터 적용
    z_m  = z_mm / 1000.0
    cam_x = (pixel_x - ppx) * z_m / fx
    cam_y = (pixel_y - ppy) * z_m / fy
    cam_point = [cam_x, cam_y, z_m, 1.0]
        │
        ▼
[3] 좌표 변환
    base_to_camera = T_ee_in_base × T_gripper2camera
    base_point = base_to_camera × cam_point
        │
        ▼
    (base_x, base_y, base_z)   [m, base_link 기준]
```

### 6.3 캘리브레이션 파일

| 파일명 | 형식 | 설명 |
|--------|------|------|
| `T_gripper2camera.npy` | NumPy 4×4 `float64` | EE → 카메라 동차 변환. 번역 부분 단위 mm, 로드 시 `/= 1000.0`으로 m 변환 |

파일 위치: `cup_stack/ros2/src/cup_stack/config/T_gripper2camera.npy`

`move_cartesian.launch.py`에 정의된 정적 TF (`link_6 → camera_link`):
- Translation: `(0.0384, 0.05885, -0.005715)` m
- Quaternion (xyzw): `(0.002446, -0.003002, 0.999861, 0.016209)`

### 6.4 카메라 내부 파라미터 (K 행렬)

```
K = [fx,  0, ppx,
      0, fy, ppy,
      0,  0,   1]
```

| 기호 | K 인덱스 | 설명 |
|------|----------|------|
| `fx` | `K[0]` | X축 초점 거리 (px) |
| `fy` | `K[4]` | Y축 초점 거리 (px) |
| `ppx` | `K[2]` | 주점 X (px) |
| `ppy` | `K[5]` | 주점 Y (px) |

---

## 7. YOLO 컵 감지 파이프라인

### 7.1 개요

```
/camera/camera/color/image_raw
        │
        ▼
[yolo_detector_node]
  - Ultralytics YOLOv8 (또는 v11) 모델 로드
  - 매 프레임 추론 → bbox, label, confidence 추출
  - cup_id 부여: "cup_0", "cup_1", ... (confidence 내림차순)
        │
        ▼ /cup_detections  (CupDetectionArray)
[cup_pose_estimator_node]
  - 각 detection의 (cx, cy) → depth 탐색 → 3D 변환 (섹션 6.2 파이프라인)
  - pose_valid = (depth > 0 및 변환 성공)
        │
        ▼ /cup_poses  (CupPoseArray, frame_id="base_link")
[rosbridge]
        │
[Server CupDetectionDomain]
  - 최신 CupPoseArray를 메모리에 보관
  - 10 Hz로 /ws/cups WebSocket 전송
  - GET /api/robot/cups REST 응답
  - POST /api/robot/cups/trigger: cup_id → pixel_x/y → task 시작
```

### 7.2 cup_id 부여 규칙

- 매 프레임마다 독립적으로 부여 (프레임 간 트래킹 없음)
- YOLO confidence 내림차순 정렬 후 `cup_0`, `cup_1`, … 순서 부여
- 동일 물리 컵이라도 프레임이 바뀌면 ID가 달라질 수 있음
- 트래킹이 필요하면 별도 `cup_tracker_node`를 추가로 구현

### 7.3 depth 탐색 전략

컵 중심 픽셀의 depth가 0(무효)인 경우:
1. 반경 `depth_search_radius`(기본 30px) 내 픽셀을 스캔
2. 유효 depth 값(>0) 중 **25th percentile** 값 선택 (이상값 제거 목적)
3. 여전히 유효 값이 없으면 `pose_valid=false`로 해당 컵의 position 무효 처리

### 7.4 `cup_detection.launch.py` 구성

```python
# cup_detection.launch.py (신규)
nodes:
  - yolo_detector_node
      params: model_path, confidence_threshold, device,
              image_topic, output_topic
  - cup_pose_estimator_node
      params: detection_topic, depth_topic, camera_info_topic,
              output_topic, depth_search_radius
```

서버는 이 launch 파일을 `task: "cup_detection"` (service 분류)으로 기동한다.

---

## 8. 전체 데이터 흐름

### 8.1 YOLO 감지 → 컵 선택 → task 시작

```
[Browser]
  1. /ws/cups 수신 → 카메라 피드 위에 bbox 오버레이 렌더링
  2. 사용자가 컵 클릭 (또는 목록에서 선택)
  3. POST /api/robot/cups/trigger
     body: { cup_id: "cup_0", task: "cup_pyramid_web" }

[Server]
  4. CupDetectionDomain에서 cup_0의 (cx, cy) 조회
  5. LaunchManager.start("cup_pyramid_web", { pixel_x: "320", pixel_y: "240" })
  6. ros2 launch cup_stack cup_pyramid_web.launch.py pixel_x:=320 pixel_y:=240

[ROS 2: cup_pyramid_web_node]
  7. pixel_to_base(320, 240) → (base_x, base_y, base_z)
  8. CupPyramidTask.try_execute(pick_xy=(base_x, base_y))
```

### 8.2 클릭 → task 시작 (기존 방식, 병행 지원)

```
[Browser]
  1. Eye-in-Hand 카메라 피드 클릭
  2. displayX/Y → 실제 픽셀 (naturalWidth/Height 스케일링)
  3. POST /api/robot/task/start
     body: { task: "cup_pyramid_web", args: { pixel_x: "320", pixel_y: "240" } }

[Server]
  4. LaunchManager.start("cup_pyramid_web", { pixel_x: "320", pixel_y: "240" })
  5. ros2 launch cup_stack cup_pyramid_web.launch.py pixel_x:=320 pixel_y:=240

[ROS 2: cup_pyramid_web_node]
  6. pixel_to_base(320, 240) → task 실행
```

### 8.3 직접 이동 (move_cartesian 서비스)

```
[Browser]
  1. POST /api/robot/move
     body: { x: 0.35, y: 0.02, z: 0.40, mode: "absolute" }

[Server]
  2. RosBridge.call_service("/move_cartesian", { x, y, z, mode })

[ROS 2: move_cartesian_node]
  3. MoveItPy.plan(target) → execute()
  4. Response: { success: true, message: "OK" }
```

### 8.4 전체 노드 기동 순서 (권장)

```
1. bringup_sim / bringup_real    (MoveItPy, rosbridge, 카메라 드라이버)
       ↓ (자동, 20초 후)
2. move_cartesian                (EE 이동 + 그리퍼 서비스)
       ↓ (수동 또는 대시보드)
3. cup_detection                 (yolo_detector + cup_pose_estimator)
       ↓ (대시보드에서 컵 선택 또는 직접 좌표 입력)
4. cup_pyramid_web / cup_unstack_web  (task 실행)
```
