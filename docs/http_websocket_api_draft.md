# HTTP REST & WebSocket API (Draft)

이 문서는 `server` ↔ `frontend` 간의 HTTP REST 엔드포인트와 WebSocket 스트리밍 인터페이스를 정의한다.  
YOLO 기반 컵 감지 파이프라인의 결과 전달 인터페이스를 포함한다.

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [HTTP REST API](#2-http-rest-api)
3. [WebSocket 스트리밍](#3-websocket-스트리밍)
4. [공통 스키마](#4-공통-스키마)
5. [서비스 포트 구성](#5-서비스-포트-구성)

---

## 1. 시스템 개요

```
Browser
  ├── HTTP REST  →  FastAPI server (각 domain 포트)
  └── WebSocket  →  FastAPI server  →  RosBridge (9090)
                                             └── ROS 2 topics / services
                                                   ├── Robot (joint states, move, gripper)
                                                   ├── Camera (compressed image stream)
                                                   └── Cup Detection (YOLO → 3D poses)
```

### 컵 감지 데이터 흐름

```
Camera ──color──▶ YOLO Detector ──────▶ /cup_detections
       ──depth──▶ Cup Pose Estimator ◀─┘
                       │
                       ▼ /cup_poses
                   RosBridge
                       │
                   Server (CupDetectionDomain)
                   ├── GET /api/robot/cups        (REST — 최신 스냅샷)
                   └── /ws/cups                   (WS  — 10 Hz 스트림)
                       │
                   Browser
                   ├── 카메라 피드 위 bbox 오버레이
                   ├── 컵 목록 + 3D 좌표 표시
                   └── 컵 선택 → task 트리거
```

---

## 2. HTTP REST API

기본 URL: `http://<host>:<port>`  
모든 요청 헤더: `Content-Type: application/json`

---

### 2.1 Robot — `/api/robot`

#### GET `/api/robot/status`

로봇 전체 상태를 반환한다.

**응답 200**

```json
{
  "joints": {
    "name":     ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
    "position": [0.0, 0.0, 1.5708, 0.0, 1.5708, 1.5708],
    "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "effort":   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  },
  "task":    { "name": "cup_pyramid_web", "status": "running" },
  "bringup": { "name": "bringup_real",    "status": "running" },
  "tasks": [
    { "name": "bringup_real", "command": "bringup_real", "status": "running", "pid": 12345 }
  ],
  "ee_position": { "x": 0.412, "y": -0.073, "z": 0.336 }
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `joints.position` | `float[]` | 라디안 (6축) |
| `task` | `ActiveTask` | action 태스크 기준; service 노드 제외 |
| `bringup` | `ActiveTask` | bringup 없으면 `{name: null, status: "idle"}` |
| `ee_position` | `EEPosition \| null` | 마지막 move 명령 좌표 (m). 없으면 `null` |

---

#### POST `/api/robot/bringup`

bringup 프로세스를 시작한다. 완료 20초 후 `move_cartesian` 서비스 노드가 자동 기동된다.

**요청 body**

```json
{ "mode": "real", "ip": "192.168.1.100" }
```

| 필드 | 기본값 | 설명 |
|------|--------|------|
| `mode` | `"sim"` | `"sim"` \| `"real"` |
| `ip` | `"192.168.1.100"` | `mode="real"`일 때 로봇 컨트롤러 IP |

**응답 200** → `TaskStartedResponse`

**에러**: 400 (잘못된 mode), 409 (이미 실행 중), 503 (domain 미초기화)

---

#### POST `/api/robot/task/start`

ROS 2 launch 태스크를 시작한다. action 태스크는 동시에 하나만 실행된다.

**요청 body**

```json
{ "task": "cup_pyramid_web", "args": { "pixel_x": "320", "pixel_y": "240" } }
```

| 필드 | 설명 |
|------|------|
| `task` | 아래 허용 태스크 목록 참고 |
| `args` | launch 인자 (`key:=value`로 전달). `cup_pyramid_web`/`cup_unstack_web`은 `pixel_x`, `pixel_y` 필수 |

**허용 태스크**

| 값 | 분류 | 설명 |
|----|------|------|
| `cup_pyramid` | action | HOME XY 기준 피라미드 빌드 |
| `cup_unstack` | action | HOME XY 기준 언스택 |
| `cup_pyramid_select` | action | OpenCV 클릭으로 위치 선택 후 피라미드 빌드 |
| `cup_unstack_select` | action | OpenCV 클릭으로 위치 선택 후 언스택 |
| `cup_pyramid_web` | action | 픽셀 좌표 기반 피라미드 빌드 |
| `cup_unstack_web` | action | 픽셀 좌표 기반 언스택 |
| `move_cartesian` | service | EE 이동 + 그리퍼 ROS 서비스 노드 |
| `cup_detection` | service | YOLO 감지 + Pose 추정 노드 (지속 실행) |
| `bringup_sim` | bringup | 시뮬레이션 bringup |
| `bringup_real` | bringup | 실로봇 bringup |

**응답 200** → `TaskStartedResponse`

**에러**: 400 (잘못된 task), 409 (action 중복 실행)

---

#### POST `/api/robot/task/stop`

태스크를 종료한다. 이미 정지된 태스크는 200으로 무시된다(멱등).

**요청 body**

```json
{ "name": "cup_pyramid_web" }
```

**응답 200** → `TaskStoppedResponse`

---

#### GET `/api/robot/task/log`

태스크 최근 stdout 로그를 반환한다.

**쿼리 파라미터**

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `name` | 필수 | 태스크 이름 |
| `tail` | `50` | 최대 줄 수 (1–500) |

**응답 200**

```json
{ "name": "cup_pyramid_web", "log": ["[INFO] Moving HOME", "[INFO] CYCLE 1/6"] }
```

---

#### GET `/api/robot/position`

마지막 move 명령 EE 좌표. 아직 move 명령이 없으면 404.

**응답 200** → `EEPosition`

---

#### GET `/api/robot/workspace/limits`

워크스페이스 안전 영역 한계를 반환한다.

**응답 200**

```json
{
  "x_min": -0.5, "x_max": 0.5,
  "y_min": -0.5, "y_max": 0.5,
  "z_min":  0.25, "z_max": 0.55,
  "grid_spacing": 0.05
}
```

---

#### POST `/api/robot/gripper`

그리퍼를 열거나 닫는다. `move_cartesian` 노드가 실행 중이어야 한다.

**요청 body**

```json
{ "command": "open" }
```

**응답 200**

```json
{ "success": true, "message": "Gripper opened" }
```

`success=false`는 하드웨어 미연결 또는 서비스 실패를 의미한다.

---

#### POST `/api/robot/move`

EE를 지정 좌표로 이동한다. 좌표는 workspace limits로 자동 클램핑된다.

**요청 body**

```json
{ "x": 0.4, "y": 0.0, "z": 0.35, "mode": "absolute" }
```

| 필드 | 필수 | 기본값 | 설명 |
|------|------|--------|------|
| `x`, `y`, `z` | 필수 | — | base_link 기준 좌표 (m) |
| `mode` | 선택 | `"absolute"` | `"absolute"` \| `"relative"` |

**응답 200**

```json
{ "success": true, "message": "OK", "position": { "x": 0.4, "y": 0.0, "z": 0.35 } }
```

---

### 2.2 Cup Detection — `/api/robot/cups`

#### GET `/api/robot/cups`

가장 최근 YOLO 감지 결과(스냅샷)를 반환한다.  
`cup_detection` 태스크가 실행 중이 아니거나 아직 첫 프레임이 수신되지 않으면 `cups` 배열이 빈 상태로 반환된다.

**응답 200** → `CupDetectionFrame`

```json
{
  "stamp": 1715165696.789,
  "frame_id": "base_link",
  "count": 2,
  "cups": [
    {
      "id":         "cup_0",
      "label":      "cup",
      "confidence": 0.95,
      "position":   { "x": 0.350, "y":  0.020, "z": 0.300 },
      "pixel":      { "x": 320,   "y": 240 },
      "bbox":       { "x_min": 300, "y_min": 220, "x_max": 340, "y_max": 260 }
    },
    {
      "id":         "cup_1",
      "label":      "cup",
      "confidence": 0.88,
      "position":   { "x": 0.320, "y": -0.050, "z": 0.300 },
      "pixel":      { "x": 280,   "y": 260 },
      "bbox":       { "x_min": 260, "y_min": 240, "x_max": 300, "y_max": 280 }
    }
  ]
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `stamp` | `float` | UNIX 타임스탬프 (초) |
| `frame_id` | `string` | 좌표 기준 프레임 (`"base_link"`) |
| `count` | `int` | 감지된 컵 수 |
| `cups[].id` | `string` | 프레임 내 고유 ID (`"cup_0"`, `"cup_1"`, …) |
| `cups[].label` | `string` | YOLO 클래스 레이블 |
| `cups[].confidence` | `float` | YOLO 신뢰도 [0.0, 1.0] |
| `cups[].position` | `EEPosition` | `base_link` 기준 3D 좌표 (m) |
| `cups[].pixel` | `PixelPoint` | 컬러 이미지 상의 bbox 중심 픽셀 좌표 |
| `cups[].bbox` | `BoundingBox` | 픽셀 단위 bbox |

---

#### POST `/api/robot/cups/trigger`

YOLO로 감지한 특정 컵을 기반으로 task를 시작한다.  
서버가 해당 `cup_id`의 현재 픽셀 좌표를 조회하여 `cup_pyramid_web` 또는 `cup_unstack_web` task를 시작한다.

**요청 body**

```json
{ "cup_id": "cup_0", "task": "cup_pyramid_web" }
```

| 필드 | 설명 |
|------|------|
| `cup_id` | 감지 결과의 `cups[].id` 값 |
| `task` | `"cup_pyramid_web"` \| `"cup_unstack_web"` |

**응답 200** → `TaskStartedResponse`

**에러**

| 코드 | 조건 |
|------|------|
| 400 | 유효하지 않은 `task` 값 |
| 404 | `cup_id`를 현재 감지 결과에서 찾을 수 없음 |
| 409 | action 태스크 중복 실행 |
| 503 | `cup_detection` 태스크가 실행 중이지 않음 |

---

### 2.3 Hand-in-Eye — `/api/handineye`

EE 장착 카메라의 `T_gripper2camera` 캘리브레이션 관리.

#### GET `/api/handineye/calibration`

**응답 200** → `CalibrationResponse`

```json
{
  "file":   "T_gripper2camera.npy",
  "matrix": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
  "shape":  [4, 4]
}
```

`matrix`는 mm 단위 4×4 동차 변환 행렬.

#### PUT `/api/handineye/calibration`

**요청 body** → `CalibrationUpdateRequest`

```json
{ "matrix": [[...]] }
```

**응답 200** → `CalibrationResponse`

---

### 2.4 Hand-to-Eye — `/api/handtoeye`

고정 카메라의 `T_base2camera` 캘리브레이션 관리. 스키마는 2.3과 동일.

#### GET `/api/handtoeye/calibration`

**응답 200** → `CalibrationResponse` (`file: "T_base2camera.npy"`)

#### PUT `/api/handtoeye/calibration`

**요청 body** / **응답 200** → 2.3과 동일.

---

## 3. WebSocket 스트리밍

모든 WS 경로는 `ws://<host>:<port>/<path>`로 접속한다.  
클라이언트는 연결 끊김 시 2초 후 자동 재연결한다.

---

### 3.1 로봇 상태 — `/ws/robot/state`

| 항목 | 값 |
|------|----|
| 방향 | server → client |
| 주기 | 100 ms (10 Hz) |
| 타입 | JSON |

**메시지** — `RobotStatusResponse`와 동일한 스키마 (섹션 2.1 `GET /api/robot/status` 참고).

---

### 3.2 카메라 영상 — `/ws/camera/{camera_name}`

| 항목 | 값 |
|------|----|
| 방향 | server → client |
| 주기 | ~30 fps (33 ms throttle) |
| 타입 | Binary (JPEG bytes) |

| `camera_name` | ROS 소스 토픽 |
|---------------|--------------|
| `handineye`   | `/camera/camera/color/image_raw/compressed` |
| `handtoeye`   | `/camera/fixed_camera/color/image_raw/compressed` |

- 클라이언트: `ws.binaryType = 'arraybuffer'` → 수신 바이트를 `Blob`으로 변환 → `URL.createObjectURL()` → `<img>` src 할당
- 이전 URL: `URL.revokeObjectURL()`로 즉시 해제
- JPEG 외 포맷은 서버에서 OpenCV로 JPEG(quality=80) 변환 후 전송

> **컵 오버레이**: 브라우저는 `<canvas>`를 `<img>` 위에 겹쳐 `/ws/cups`로 받은 bbox/label/confidence를 직접 그린다.  
> 서버는 원본 프레임만 전송하며 오버레이를 포함하지 않는다.

---

### 3.3 태스크 로그 — `/ws/task/log`

| 항목 | 값 |
|------|----|
| 방향 | server → client |
| 주기 | 500 ms (2 Hz) |
| 타입 | JSON |

**태스크 실행 중**

```json
{
  "task":   "cup_pyramid_web",
  "status": "running",
  "log":    ["[INFO] Moving HOME", "[INFO] CYCLE 1/6", "[INFO] GRIP"]
}
```

**태스크 없음**

```json
{ "task": null, "status": "idle", "log": [] }
```

| 필드 | 설명 |
|------|------|
| `task` | action 태스크 이름. 없으면 bringup 이름. 모두 없으면 `null` |
| `log` | 최근 5줄 stdout (MoveIt 내부 노이즈 필터링 적용) |

---

### 3.4 컵 감지 결과 — `/ws/cups`

| 항목 | 값 |
|------|----|
| 방향 | server → client |
| 주기 | 100 ms (10 Hz) |
| 타입 | JSON |

**메시지** — `CupDetectionFrame`

```json
{
  "stamp":    1715165696.789,
  "frame_id": "base_link",
  "count":    2,
  "cups": [
    {
      "id":         "cup_0",
      "label":      "cup",
      "confidence": 0.95,
      "position":   { "x": 0.350, "y":  0.020, "z": 0.300 },
      "pixel":      { "x": 320,   "y": 240 },
      "bbox":       { "x_min": 300, "y_min": 220, "x_max": 340, "y_max": 260 }
    }
  ]
}
```

- `cups` 배열이 비어 있으면 컵이 감지되지 않았음을 의미한다.
- `position`이 `null`인 컵은 depth 조회 실패로 2D 감지만 성공한 경우다.
- 동일 프레임이 반복 전송될 수 있다 (ROS 메시지 수신 주기와 WS 전송 주기가 다를 수 있음).

---

## 4. 공통 스키마

### 기본 타입

```typescript
type TaskStatus = "idle" | "running" | "stopping" | "failed"

interface EEPosition {
  x: number   // base_link X (m)
  y: number   // base_link Y (m)
  z: number   // base_link Z (m)
}

interface PixelPoint {
  x: number   // 픽셀 X
  y: number   // 픽셀 Y
}

interface BoundingBox {
  x_min: number
  y_min: number
  x_max: number
  y_max: number
}
```

### ActiveTask

```typescript
interface ActiveTask {
  name:   string | null
  status: TaskStatus
}
```

### TaskStartedResponse

```typescript
interface TaskStartedResponse {
  name:   string
  status: TaskStatus
  pid:    number | null   // agent 경유 태스크는 null
}
```

### TaskStoppedResponse

```typescript
interface TaskStoppedResponse {
  name:   string
  status: "stopped"
}
```

### CalibrationResponse / CalibrationUpdateRequest

```typescript
interface CalibrationResponse {
  file:   string            // 예: "T_gripper2camera.npy"
  matrix: number[][]        // 4×4, mm 단위
  shape:  [4, 4]
}

interface CalibrationUpdateRequest {
  matrix: number[][]        // 4×4, mm 단위
}
```

### CupInfo

```typescript
interface CupInfo {
  id:          string         // "cup_0", "cup_1", ...
  label:       string         // "cup"
  confidence:  number         // [0.0, 1.0]
  position:    EEPosition | null   // base_link 3D 좌표 (m); depth 실패 시 null
  pixel:       PixelPoint          // bbox 중심 픽셀 좌표
  bbox:        BoundingBox         // 픽셀 단위 bbox
}

interface CupDetectionFrame {
  stamp:    number           // UNIX 타임스탬프 (초, float)
  frame_id: string           // "base_link"
  count:    number
  cups:     CupInfo[]
}
```

### ErrorDetail

```typescript
interface ErrorDetail {
  detail: string
}
```

---

## 5. 서비스 포트 구성

Docker Compose (`server/docker-compose.yml`) 기준:

| 서비스 | 내부 포트 | 담당 엔드포인트 |
|--------|-----------|----------------|
| nginx (frontend) | 80 | React SPA + `/api/` 프록시 |
| robot FastAPI | 8001 | `/api/robot/*`, `/ws/robot/state`, `/ws/camera/*`, `/ws/task/log`, `/ws/cups` |
| handineye FastAPI | 8002 | `/api/handineye/*` |
| handtoeye FastAPI | 8003 | `/api/handtoeye/*` |
| rosbridge | 9090 | roslibpy WebSocket 연결 대상 |

rosbridge 환경 변수:

| 환경 변수 | 기본값 | 설명 |
|-----------|--------|------|
| `ROSBRIDGE_HOST` | `localhost` | rosbridge 호스트 |
| `ROSBRIDGE_PORT` | `9090` | rosbridge 포트 |
