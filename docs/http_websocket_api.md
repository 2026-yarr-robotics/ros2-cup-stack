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

로봇 상태와 실행 중인 태스크 목록을 반환한다.

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
  "tasks": [
    {
      "name":    "bringup_real",
      "command": "bringup_real",
      "status":  "running",
      "pid":     12345
    }
  ]
}
```

| 필드 | 타입 | 설명 |
|------|------|------|
| `joints.position` | `float[]` | 라디안 단위 관절 각도 (6축) |
| `joints.velocity` | `float[]` | 라디안/s |
| `joints.effort`   | `float[]` | N·m |
| `task.status`     | `string`  | `idle` \| `running` \| `stopping` \| `failed` |
| `tasks[].pid`     | `int\|null` | 프로세스 종료 시 `null` |

---

#### POST `/api/robot/bringup`

로봇 bringup 프로세스를 시작한다.

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
| 409  | 이미 실행 중인 태스크가 있음 |
| 503  | Robot domain 미초기화 |

---

#### POST `/api/robot/task/start`

ROS 2 태스크 launch를 시작한다.

**요청 body**

```json
{ "task": "cup_pyramid_web", "args": { "pixel_x": "320", "pixel_y": "240" } }
```

| 필드   | 타입              | 필수 | 설명 |
|--------|-------------------|------|------|
| `task` | `string`          | 필수 | 아래 허용 태스크 목록 참고 |
| `args` | `Record<string,string>` | 선택 | launch 인자 (launch 파일에 `key:=value`로 전달) |

**허용 태스크 (`task` 값)**

| 값                   | 설명 |
|----------------------|------|
| `cup_pyramid`        | 현재 HOME XY를 nested stack 위치로 사용해 피라미드 빌드 |
| `cup_unstack`        | 현재 EE XY를 피라미드 중심으로 사용해 언스택 |
| `cup_pyramid_select` | OpenCV 창에서 클릭 좌표 선택 후 피라미드 빌드 |
| `cup_unstack_select` | OpenCV 창에서 클릭 좌표 선택 후 언스택 |
| `cup_pyramid_web`    | 대시보드 픽셀 좌표를 받아 피라미드 빌드 |
| `cup_unstack_web`    | 대시보드 픽셀 좌표를 받아 언스택 |
| `bringup_sim`        | 시뮬레이션 bringup |
| `bringup_real`       | 실로봇 bringup |

**`cup_pyramid_web` / `cup_unstack_web` args**

| 키        | 타입     | 설명 |
|-----------|----------|------|
| `pixel_x` | `string` | 카메라 이미지 픽셀 X 좌표 (정수 문자열) |
| `pixel_y` | `string` | 카메라 이미지 픽셀 Y 좌표 (정수 문자열) |

**응답 200**

```json
{ "name": "cup_pyramid_web", "status": "running", "pid": 12346 }
```

**에러**

| 코드 | 조건 |
|------|------|
| 400  | 유효하지 않은 `task` 값 |
| 409  | 이미 실행 중인 태스크가 있음 |

---

#### POST `/api/robot/task/stop`

실행 중인 태스크를 종료한다.

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
| 404  | 해당 이름의 태스크가 없음 |

---

#### GET `/api/robot/task/log`

특정 태스크의 stdout 로그를 반환한다.

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

### 1.2 Hand-in-Eye — `/api/handineye`

EE에 장착된 카메라(`T_gripper2camera.npy`)에 대한 캘리브레이션 관리 엔드포인트.

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

---

#### PUT `/api/handineye/calibration`

`T_gripper2camera` 행렬을 갱신·저장한다.

**요청 body**

```json
{
  "matrix": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
}
```

| 필드     | 타입       | 필수 | 설명 |
|----------|------------|------|------|
| `matrix` | `float[][]`| 필수 | 4×4 동차 변환 행렬 |

**응답 200** — GET과 동일한 스키마

**에러**

| 코드 | 조건 |
|------|------|
| 400  | `matrix` 필드 누락 |

---

### 1.3 Hand-to-Eye — `/api/handtoeye`

고정 카메라(`T_base2camera.npy`)에 대한 캘리브레이션 관리 엔드포인트.  
요청/응답 스키마는 1.2와 동일하며 파일명만 다르다.

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
  "tasks": [
    {
      "name":    "bringup_real",
      "command": "bringup_real",
      "status":  "running",
      "pid":     12345
    }
  ]
}
```

- `joints.position` 값은 **라디안** 단위. 프론트엔드는 `× 180/π`로 변환해 표시한다.
- `task.status`: `"idle"` \| `"running"` \| `"stopping"` \| `"failed"`
- 태스크가 없으면 `task.name = null`, `task.status = "idle"`

---

### 2.2 카메라 영상 — `/ws/camera/{camera_name}`

**방향**: server → client  
**주기**: ~30 fps (rosbridge throttle_rate=33 ms)  
**메시지 타입**: Binary (JPEG bytes)

| `camera_name` | ROS 토픽 소스 |
|---------------|--------------|
| `handineye`   | `/camera/camera/color/image_raw/compressed` |
| `handtoeye`   | `/fixed_camera/color/image_raw/compressed` |

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
| `task`   | `string\|null` | 현재 실행 중인 태스크 이름 |
| `status` | `string`   | `"idle"` \| `"running"` \| `"stopping"` \| `"failed"` |
| `log`    | `string[]` | 가장 최근 5줄의 stdout 라인 |

프론트엔드는 `log` 각 줄에서 `ERR`/`FAIL`, `OK`, `WARN` 키워드를 탐지해 로그 레벨(`ERR`, `OK`, `WARN`, `INFO`)을 결정한다.

---

## 3. 서비스 포트 구성

Docker Compose (`server/docker-compose.yml`) 기준:

| 서비스 | 내부 포트 | 설명 |
|--------|-----------|------|
| nginx (frontend) | 80 | React 정적 파일 서빙 + `/api/` 프록시 |
| robot FastAPI | 8001 | `/api/robot`, `/ws/robot/state`, `/ws/camera/*`, `/ws/task/log` |
| handineye FastAPI | 8002 | `/api/handineye` |
| handtoeye FastAPI | 8003 | `/api/handtoeye` |
| rosbridge | 9090 | WebSocket — roslibpy 연결 대상 |

rosbridge 연결 설정은 환경 변수로 오버라이드 가능하다:

| 환경 변수       | 기본값      | 설명 |
|-----------------|-------------|------|
| `ROSBRIDGE_HOST`| `localhost` | rosbridge WebSocket 호스트 |
| `ROSBRIDGE_PORT`| `9090`      | rosbridge WebSocket 포트 |
