# HTTP REST & WebSocket API

이 문서는 `server` ↔ `frontend` 간의 HTTP REST 엔드포인트와 WebSocket 스트리밍 인터페이스를 정의한다.

---

## 목차

1. [HTTP REST API](#1-http-rest-api)
2. [WebSocket 스트리밍](#2-websocket-스트리밍)
3. [서비스 포트 구성](#3-서비스-포트-구성)

---

## 1. HTTP REST API

기본 URL 접두사: `http://<host>:<port>`  
`Content-Type: application/json`을 요청 헤더에 명시한다.

### 1.1 Robot — `/api/robot`

#### GET `/api/robot/status`

로봇 상태, bringup 상태, 실행 중인 태스크 목록을 반환한다.

**응답 200**

```json
{
  "joints": {
    "name":     ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
    "position": [0.0, 0.0, 1.5708, 0.0, 1.5708, 1.5708],
    "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "effort":   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  },
  "task": {
    "name":   "cup_pyramid_web",
    "status": "running"
  },
  "bringup": {
    "name":   "bringup_real",
    "status": "running"
  },
  "tasks": [
    {
      "name":    "bringup_real",
      "command": "bringup_real",
      "status":  "running",
      "pid":     12345
    }
  ],
  "ee_position": { "x": 0.412, "y": -0.073, "z": 0.336 }
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `joints.position` | `float[]` | 라디안 단위 관절 각도 (6축) |
| `joints.velocity` | `float[]` | 라디안/s |
| `joints.effort`   | `float[]` | N·m |
| `task.status`     | `string`  | `idle` \| `running` \| `stopping` \| `failed` |
| `bringup`         | `object`  | 현재 bringup 태스크 상태 (없으면 `{name: null, status: "idle"}`) |
| `tasks[].pid`     | `int\|null` | 프로세스 종료 시 `null` |
| `ee_position`     | `object\|null` | 마지막으로 명령한 EE 좌표 (m). 아직 없으면 `null` |

---

#### POST `/api/robot/bringup`

로봇 bringup 프로세스를 시작한다. 성공 후 20초 뒤 `move_cartesian` 서비스 노드가 자동 기동된다.

**요청 body**

```json
{ "mode": "real", "ip": "192.168.1.100" }
```

| 필드  | 타입   | 기본값          | 설명                          |
|-------|--------|-----------------|-------------------------------|
| `mode`| `string` | `"sim"`       | `"sim"` \| `"real"`           |
| `ip`  | `string` | `"192.168.1.100"` | `mode="real"`일 때만 사용 |

**응답 200**

```json
{ "name": "bringup_real", "status": "running", "pid": 12345 }
```

**에러**

| 코드 | 조건 |
|------|------|
| 400  | 유효하지 않은 `mode` 값 |
| 409  | 이미 실행 중인 bringup이 있음 |
| 503  | Robot domain 미초기화 |

---

#### POST `/api/robot/task/start`

ROS 2 launch 태스크를 시작한다. action 태스크는 동시에 하나만 실행 가능하다.  
`move_cartesian`은 service 노드로 분류되어 action 제한을 받지 않는다.

**요청 body**

```json
{ "task": "cup_pyramid_web", "args": { "pixel_x": "320", "pixel_y": "240" } }
```

| 필드   | 타입              | 필수 | 설명 |
|--------|-------------------|------|------|
| `task` | `string`          | 필수 | 아래 허용 태스크 목록 참고 |
| `args` | `Record<string,string>` | 선택 | launch 인자 (`key:=value`로 전달) |

**허용 태스크 (`task` 값)**

| 값                   | 분류    | 설명 |
|----------------------|---------|------|
| `cup_pyramid`        | action  | HOME XY 기준 피라미드 빌드 |
| `cup_unstack`        | action  | HOME XY 기준 언스택 |
| `cup_pyramid_select` | action  | OpenCV 클릭으로 위치 선택 후 피라미드 빌드 |
| `cup_unstack_select` | action  | OpenCV 클릭으로 위치 선택 후 언스택 |
| `cup_pyramid_web`    | action  | 픽셀 좌표 기반 피라미드 빌드 |
| `cup_unstack_web`    | action  | 픽셀 좌표 기반 언스택 |
| `move_cartesian`     | service | EE 이동 + 그리퍼 ROS 서비스 노드 (지속 실행) |
| `bringup_sim`        | bringup | 시뮬레이션 bringup |
| `bringup_real`       | bringup | 실로봇 bringup |

**`cup_pyramid_web` / `cup_unstack_web` args**

| 키        | 타입     | 설명 |
|-----------|----------|------|
| `pixel_x` | `string` | 카메라 이미지 픽셀 X (정수 문자열) |
| `pixel_y` | `string` | 카메라 이미지 픽셀 Y (정수 문자열) |

**응답 200**

```json
{ "name": "cup_pyramid_web", "status": "running", "pid": 12346 }
```

`move_cartesian` 등 agent 경유 태스크는 `pid: null`.

**에러**

| 코드 | 조건 |
|------|------|
| 400  | 유효하지 않은 `task` 값 |
| 409  | action 태스크가 이미 실행 중 |

---

#### POST `/api/robot/task/stop`

실행 중인 태스크를 종료한다. 이미 종료된 태스크에 대한 요청은 200으로 무시된다.

**요청 body**

```json
{ "name": "cup_pyramid_web" }
```

**응답 200**

```json
{ "name": "cup_pyramid_web", "status": "stopped" }
```

**에러**

| 코드 | 조건 |
|------|------|
| 400  | `name` 필드 누락 |
| 404  | bringup 태스크 이름이 잘못됨 |

---

#### GET `/api/robot/task/log`

특정 태스크의 최근 stdout 로그를 반환한다.

**쿼리 파라미터**

| 파라미터 | 타입  | 기본값 | 설명 |
|----------|-------|--------|------|
| `name`   | `string` | 필수 | 태스크 이름 |
| `tail`   | `int`    | `50` | 반환할 최대 줄 수 |

**응답 200**

```json
{ "name": "cup_pyramid_web", "log": ["[INFO] Moving HOME", "..."] }
```

**에러**

| 코드 | 조건 |
|------|------|
| 400  | `name` 쿼리 파라미터 누락 |
| 404  | 해당 이름의 태스크가 없음 |

---

#### GET `/api/robot/position`

마지막으로 명령된 EE 좌표를 반환한다. `POST /api/robot/move`가 성공한 이후부터 유효하다.

**응답 200**

```json
{ "x": 0.412, "y": -0.073, "z": 0.336 }
```

**에러**

| 코드 | 조건 |
|------|------|
| 404  | 아직 move 명령을 내린 적 없음 |

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

모든 값의 단위는 **m**.

---

#### POST `/api/robot/gripper`

그리퍼를 열거나 닫는다. `move_cartesian` 노드의 `/gripper_control` ROS 서비스를 호출한다.

**요청 body**

```json
{ "command": "open" }
```

| 필드      | 타입     | 값                  |
|-----------|----------|---------------------|
| `command` | `string` | `"open"` \| `"close"` |

**응답 200**

```json
{ "success": true, "message": "Gripper opened" }
```

| 필드      | 타입     | 설명 |
|-----------|----------|------|
| `success` | `bool`   | `false`이면 그리퍼 하드웨어 미연결 또는 서비스 실패 |
| `message` | `string` | 결과 메시지 |

**에러**

| 코드 | 조건 |
|------|------|
| 400  | `command`가 `open`/`close`가 아님 |
| 409  | 서비스 호출 실패 (RuntimeError) |

---

#### POST `/api/robot/move`

EE를 지정한 좌표로 이동한다. `move_cartesian` 노드의 `/move_cartesian` ROS 서비스를 호출한다.  
좌표는 workspace limits 범위로 자동 클램핑된다.

**요청 body**

```json
{ "x": 0.4, "y": 0.0, "z": 0.35, "mode": "absolute" }
```

| 필드   | 타입     | 필수 | 기본값       | 설명 |
|--------|----------|------|--------------|------|
| `x`    | `float`  | 필수 | —            | base_link 기준 X (m) |
| `y`    | `float`  | 필수 | —            | base_link 기준 Y (m) |
| `z`    | `float`  | 필수 | —            | base_link 기준 Z (m) |
| `mode` | `string` | 선택 | `"absolute"` | `"absolute"` \| `"relative"` |

**응답 200**

```json
{
  "success": true,
  "message": "Moved to (0.400, 0.000, 0.350)",
  "position": { "x": 0.4, "y": 0.0, "z": 0.35 }
}
```

| 필드       | 타입        | 설명 |
|------------|-------------|------|
| `success`  | `bool`      | 플래닝 성공 여부 |
| `message`  | `string`    | 결과 메시지 |
| `position` | `object\|null` | 이동 후 EE 좌표. 실패 시 이전 값 유지 |

**에러**

| 코드 | 조건 |
|------|------|
| 400  | `x`, `y`, `z` 누락 또는 `mode` 값 오류 |
| 409  | 서비스 호출 실패 |

---

### 1.2 Hand-in-Eye — `/api/handineye`

EE에 장착된 카메라(`T_gripper2camera.npy`)에 대한 캘리브레이션 관리.

#### GET `/api/handineye/calibration`

저장된 `T_gripper2camera` 행렬을 반환한다.

**응답 200**

```json
{
  "file": "T_gripper2camera.npy",
  "matrix": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
  "shape": [4, 4]
}
```

| 필드     | 타입       | 설명 |
|----------|------------|------|
| `file`   | `string`   | 저장 파일명 |
| `matrix` | `float[][]`| 4×4 동차 변환 행렬 (mm 단위) |
| `shape`  | `int[]`    | 행렬 shape |

#### PUT `/api/handineye/calibration`

**요청 body**

```json
{ "matrix": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]] }
```

**응답 200** — GET과 동일한 스키마

**에러** — `400`: `matrix` 필드 누락

---

### 1.3 Hand-to-Eye — `/api/handtoeye`

고정 카메라(`T_base2camera.npy`)에 대한 캘리브레이션 관리. 스키마는 1.2와 동일하며 파일명만 다르다.

#### GET `/api/handtoeye/calibration`

**응답 200**

```json
{
  "file": "T_base2camera.npy",
  "matrix": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
  "shape": [4, 4]
}
```

#### PUT `/api/handtoeye/calibration`

요청/응답 스키마는 `/api/handineye/calibration` PUT과 동일.

---

## 2. WebSocket 스트리밍

모든 WebSocket 경로는 동일 호스트의 `ws://` 또는 `wss://`로 접속한다.  
클라이언트는 연결 끊김 시 2초 후 자동 재연결한다 (`useWebSocket.ts`).

### 2.1 로봇 상태 — `/ws/robot/state`

**방향**: server → client  
**주기**: 100 ms (10 Hz)  
**메시지 타입**: JSON

```json
{
  "joints": {
    "name":     ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
    "position": [0.0, 0.0, 1.5708, 0.0, 1.5708, 1.5708],
    "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "effort":   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  },
  "task": {
    "name":   "cup_pyramid_web",
    "status": "running"
  },
  "bringup": {
    "name":   "bringup_real",
    "status": "running"
  },
  "tasks": [
    {
      "name":    "bringup_real",
      "command": "bringup_real",
      "status":  "running",
      "pid":     12345
    }
  ],
  "ee_position": { "x": 0.412, "y": -0.073, "z": 0.336 }
}
```

- `joints.position` 값은 **라디안**. 프론트엔드는 `× 180/π`로 변환해 표시한다.
- `task`는 action 태스크 기준. service 노드(`move_cartesian`)는 `task`에 반영되지 않는다.
- `bringup.name = null`, `bringup.status = "idle"` — bringup 없음

---

### 2.2 카메라 영상 — `/ws/camera/{camera_name}`

**방향**: server → client  
**주기**: ~30 fps (rosbridge throttle_rate=33 ms)  
**메시지 타입**: Binary (JPEG bytes)

| `camera_name` | ROS 토픽 소스 |
|---------------|--------------|
| `handineye`   | `/camera/camera/color/image_raw/compressed` |
| `handtoeye`   | `/camera/fixed_camera/color/image_raw/compressed` |

- 클라이언트는 `ws.binaryType = 'arraybuffer'`로 설정하고, 수신 바이트를 `Blob`으로 변환 후 `URL.createObjectURL()`로 `<img>` src에 할당한다.
- 이전 프레임 URL은 메모리 누수 방지를 위해 `URL.revokeObjectURL()`로 해제한다.
- JPEG가 아닌 포맷(PNG 등)은 서버에서 OpenCV로 JPEG(quality=80)으로 변환 후 전송한다.

---

### 2.3 태스크 로그 — `/ws/task/log`

**방향**: server → client  
**주기**: 500 ms (2 Hz)  
**메시지 타입**: JSON

**실행 중 태스크 있음**

```json
{
  "task":   "cup_pyramid_web",
  "status": "running",
  "log":    ["[INFO] Moving HOME", "[INFO] CYCLE 1/6", "[INFO] GRIP"]
}
```

**실행 중 태스크 없음**

```json
{
  "task":   null,
  "status": "idle",
  "log":    []
}
```

| 필드     | 타입       | 설명 |
|----------|------------|------|
| `task`   | `string\|null` | 현재 action 태스크 이름 (없으면 bringup 이름) |
| `status` | `string`   | `"idle"` \| `"running"` \| `"stopping"` \| `"failed"` |
| `log`    | `string[]` | 가장 최근 5줄의 stdout 라인 (노이즈 필터링 적용) |

> 서버는 "returned 1 controllers in list", "Trajectory execution is managing controllers", "services ready" 등 MoveIt 내부 노이즈를 로그에서 제거한다.

---

## 3. 서비스 포트 구성

Docker Compose (`server/docker-compose.yml`) 기준:

| 서비스 | 내부 포트 | 엔드포인트 |
|--------|-----------|-----------|
| nginx (frontend) | 80 | React 정적 파일 + `/api/` 프록시 |
| robot FastAPI | 8001 | `/api/robot/*`, `/ws/robot/state`, `/ws/camera/*`, `/ws/task/log` |
| handineye FastAPI | 8002 | `/api/handineye/*` |
| handtoeye FastAPI | 8003 | `/api/handtoeye/*` |
| rosbridge | 9090 | WebSocket — roslibpy 연결 대상 |

rosbridge 연결 설정 환경 변수:

| 환경 변수       | 기본값      | 설명 |
|-----------------|-------------|------|
| `ROSBRIDGE_HOST`| `localhost` | rosbridge WebSocket 호스트 |
| `ROSBRIDGE_PORT`| `9090`      | rosbridge WebSocket 포트 |
