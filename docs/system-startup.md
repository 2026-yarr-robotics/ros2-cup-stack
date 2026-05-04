# 컵 스태킹 로봇 시스템 실행 가이드

## 시스템 구성

```
Browser (yarr.simplyimg.com)
  └─ Cloudflare Pages (React 프론트엔드)
       ├─ HTTP REST  →  yarr-api.simplyimg.com
       └─ WebSocket  →  yarr-api.simplyimg.com
                             └─ nginx (포트 80)
                                  ├─ robot:8001    (FastAPI)
                                  ├─ handineye:8002 (FastAPI)
                                  └─ handtoeye:8003 (FastAPI)
                                       └─ rosbridge (포트 9090, WebSocket)
                                                └─ ROS 2 Humble
                                                     ├─ DSR M0609 (MoveIt2)
                                                     └─ RealSense D435i
```

---

## 통합 실행 스크립트 (권장)

프로젝트 루트의 `start.sh`가 tmux 세션으로 모든 서비스를 자동 실행합니다.

```bash
cd ~/development/cup-stack

# 기본 실행 (rosbridge + 카메라 + Docker 서버)
./start.sh

# 시뮬레이션 bringup 포함
./start.sh sim

# 실로봇 bringup 포함 (기본 IP: 192.168.1.100)
./start.sh real
./start.sh real 192.168.1.100
```

실행 후 tmux 창 구성:

| 번호 | 창 이름 | 역할 |
|------|---------|------|
| 0 | `rosbridge` | ROS ↔ WebSocket 브릿지 |
| 1 | `camera` | RealSense D435i 드라이버 |
| 2 | `server` | Docker Compose (nginx + FastAPI) |
| 3 | `bringup` | 로봇 bringup (sim/real 선택 시만) |

tmux 조작:
```bash
tmux attach -t cup-stack      # 세션 연결
# Ctrl+b → 0~3                # 창 전환
tmux kill-session -t cup-stack  # 전체 종료
```

---

## 서비스별 개별 실행

### 1. rosbridge (ROS ↔ WebSocket 브릿지)

```bash
bash ~/development/cup-stack/server/rosbridge.sh
```

rosbridge_suite가 없으면 자동 설치됩니다. 포트 9090에서 실행됩니다.

---

### 2. RealSense D435i 카메라

**Eye-in-Hand 카메라** (기본, 로봇 손목 장착):

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

퍼블리시 토픽:
- `/camera/camera/color/image_raw/compressed` — RGB 1280×720@30fps
- `/camera/camera/aligned_depth_to_color/image_raw` — Depth (색상 정렬)

**Eye-to-Hand 카메라** (고정 카메라, 별도 장치 필요):

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true camera_name:=fixed_camera camera_namespace:=fixed_camera
```

퍼블리시 토픽:
- `/fixed_camera/color/image_raw/compressed`
- `/fixed_camera/aligned_depth_to_color/image_raw`

---

### 3. Docker 서버 (nginx + FastAPI + cloudflared)

```bash
cd ~/development/cup-stack/server

# 전체 빌드 후 실행
docker compose up --build

# 빌드 없이 실행 (이미지가 최신 상태일 때)
docker compose up

# 특정 서비스만 재빌드
docker compose build robot
docker compose build handineye
docker compose build handtoeye

# 백그라운드 실행
docker compose up -d
```

서비스별 포트:

| 서비스 | 내부 포트 | 역할 |
|--------|----------|------|
| nginx | 80 | 리버스 프록시 |
| robot | 8001 | 로봇 제어 REST/WS |
| handineye | 8002 | Eye-in-Hand 캘리브레이션 + 카메라 |
| handtoeye | 8003 | Eye-to-Hand 캘리브레이션 + 카메라 |
| cloudflared | — | Cloudflare 터널 |

---

### 4. 로봇 Bringup (MoveIt2)

**빌드 먼저:**

```bash
cd ~/development/cup-stack/ros2-cup-stack

# 서브모듈 초기화 (최초 1회)
git submodule update --init --recursive

# 워크스페이스 빌드
bash ros2/src/cup_stack/build_cup_stack.sh

# 또는 직접 빌드
cd ros2
colcon build --symlink-install
source install/setup.bash
```

**시뮬레이션 실행:**

```bash
bash ~/development/cup-stack/ros2-cup-stack/ros2/src/cup_stack/bringup_sim.sh
```

**실로봇 실행:**

```bash
bash ~/development/cup-stack/ros2-cup-stack/ros2/src/cup_stack/bringup_real.sh 192.168.1.100
```

> rosbridge 및 Docker 서버가 먼저 실행된 상태여야 합니다.

---

### 5. ROS2 태스크 실행 (bringup 이후)

두 번째 터미널에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/development/cup-stack/ros2-cup-stack/ros2/install/setup.bash

# 6컵 피라미드 쌓기
ros2 launch cup_stack cup_pyramid.launch.py nest_inc:=0.0127

# 클릭으로 위치 선택 후 피라미드 쌓기
ros2 launch cup_stack cup_pyramid_select.launch.py nest_inc:=0.0127

# 피라미드 → 중첩 스택으로 해체
ros2 launch cup_stack cup_unstack.launch.py nest_inc:=0.0127

# 클릭으로 위치 선택 후 해체
ros2 launch cup_stack cup_unstack_select.launch.py nest_inc:=0.0127
```

---

## 빌드 요약

| 구성 요소 | 빌드 명령 | 필요 시점 |
|-----------|----------|----------|
| ROS 2 워크스페이스 | `bash build_cup_stack.sh` | 소스 변경 시 |
| Docker 서버 전체 | `docker compose build` | server/ 코드 변경 시 |
| robot 서비스만 | `docker compose build robot` | server/server/ 변경 시 |
| 프론트엔드 | `npm run build` (자동 배포) | Git push → Cloudflare Pages |

---

## API / WebSocket 엔드포인트

베이스 URL: `https://yarr-api.simplyimg.com`

| 경로 | 방식 | 설명 |
|------|------|------|
| `/health` | GET | 서버 헬스 체크 |
| `/api/robot/status` | GET | 로봇 joint 상태 |
| `/api/robot/task/start` | POST | 태스크 시작 |
| `/api/robot/task/stop` | POST | 태스크 중지 |
| `/ws/robot/state` | WS | 10 Hz joint 상태 스트림 |
| `/ws/task/log` | WS | 태스크 로그 스트림 |
| `/ws/camera/handineye` | WS | Eye-in-Hand JPEG 스트림 |
| `/ws/camera/handtoeye` | WS | Eye-to-Hand JPEG 스트림 |

---

## 문제 해결

**nginx 502 오류 (docker compose 재시작 후)**

nginx가 이전 IP를 캐시합니다. nginx만 재시작합니다:

```bash
docker compose restart nginx
```

**카메라 "WAITING" 상태**

ROS 토픽 퍼블리셔가 없는 경우입니다:

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /camera/camera/color/image_raw/compressed
# Publisher count: 0 이면 카메라 실행 필요
```

**rosbridge 연결 실패**

```bash
ss -tlnp | grep 9090   # 9090 포트 리스닝 여부 확인
bash ~/development/cup-stack/server/rosbridge.sh
```
