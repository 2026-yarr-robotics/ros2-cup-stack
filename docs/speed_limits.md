# 속도 한계 & 허용 범위 (M0609 / cup_stack)

> 작성 2026-06-12. pick 전 staging / pick 후 travel 속도 상향 검토의 기준 문서.
> 원칙: **떨림 없는 한계 내에서만 상향** — 속도보다 가속이 떨림의 주범.

## 1. 하드웨어/설정 절대 한계 (이 값은 변경 금지)

### 조인트 한계 — `dsr_moveit_config_m0609/config/joint_limits.yaml`

| 조인트 | max_velocity | (deg/s) | max_acceleration |
|---|---|---|---|
| joint_1, joint_2 | 2.618 rad/s | **150°/s** | 2.618 rad/s² |
| joint_3 | 3.14 rad/s | **180°/s** | 3.14 rad/s² |
| joint_4–6 (손목) | 3.927 rad/s | **225°/s** | 3.927 rad/s² |

### Pilz Cartesian 한계 — `dsr_moveit_config_m0609/config/pilz_cartesian_limits.yaml`

| 항목 | 값 |
|---|---|
| max_trans_vel | **1.0 m/s** |
| max_trans_acc | **2.25 m/s²** |
| max_trans_dec | −5.0 m/s² |
| max_rot_vel | 1.57 rad/s |

> Pilz scale 은 위 값에 대한 배율이다: LIN scale 0.1 = 100 mm/s, acc scale 0.08 = 0.18 m/s².

### 실측 임계 증거 (절대 기준점)

1. 싱귤러 존(z ≥ `singular_z` = 0.50 m): **LIN scale 0.2 로 unstack 추출 시
   손목 256°/s 스파이크** (225 한계 초과). 외삽 한계점 scale ≈ 0.175 →
   싱귤러 존 LIN 은 **scale 0.1 초과 금지**.
2. **far reach (x≈0.35, y≈±0.2), 일반 LIN**: scale 0.2 에서 **J3 212°/s
   (180 한계 초과 → alarm 1908)**. 외삽: J3 한계 도달점 **scale ≈ 0.17**,
   70% 가드(126°/s) 기준 **scale ≈ 0.12** → 비접촉 LIN(fast)도 **0.12 가
   권장 상한, 0.17 절대 금지** (`_make_lin_params` 주석).

## 2. 현재 운용값 (2026-06-12 기준)

| 프로파일 | 위치 | 용도 | vel / acc scale | 실속도 |
|---|---|---|---|---|
| Pilz PTP | `runtime.py _make_ptp_params` | 접촉 하강([7a/7b]) 포함 기본 PTP | 0.4 / 0.08 | 손목 ≤ 90°/s |
| **Pilz PTP fast** | `runtime.py _make_ptp_fast_params` | 비접촉 staging/travel ([1-low],[6],[9]) | **0.6 / 0.15** | 손목 ≤ 135°/s (한계의 60%) |
| Pilz LIN | `runtime.py _make_lin_params` | 접촉 하강([3]) 등 기본 LIN | 0.1 / 0.08 | 100 mm/s, 0.18 m/s² |
| **Pilz LIN fast** | `runtime.py _make_lin_fast_params` | 비접촉 lift/settle ([5],[5b], z<0.50) | **0.12 / 0.12** | 120 mm/s, far-reach J3 ~127°/s |
| Pilz LIN slow | `runtime.py _make_lin_slow_params` | 싱귤러 존 (z≥0.50, fast 보다 우선) | **0.08 / 0.06** | 80 mm/s, 손목 ~102°/s |
| OMPL fallback | `runtime.py _make_ompl_params` | 계획 실패 폴백 (보수 유지) | 0.4 / 0.08 | PTP 와 동일 거동 |
| native movel | server `domains/robot.py` (`vel/acc`) | `/move` staging | — | **250 mm/s / 400 mm/s²** |

> 2026-06-12: 권장 상향 적용됨 — fast 는 opt-in(`try_move_to_pose(fast=True)`),
> `slow` 가 `fast` 에 우선. 접촉 구간([3],[7a],[7b])은 기존 프로파일 유지.

## 3. 허용 범위 — 구간별 상향 가능치

| 구간 | 현재 | 권장 상향 | **한계치 (초과 금지)** | 근거 |
|---|---|---|---|---|
| ① PTP staging (pick 전) | 0.4 / 0.08 | 0.6 / 0.15 | **0.7 / 0.2** | joint-space 라 싱귤러 무관. 손목 157°/s = 한계의 70%. acc>0.2 → position-only JTC 오버슈트 진동 |
| ② LIN lift·settle, z<0.50 (pick 후, 비접촉) | 0.1 / 0.08 | 0.12 / 0.12 (120 mm/s) | **0.17** (J3 한계 도달점, §1-2) | ⚠️ far reach 에서 LIN 0.2 = J3 212°/s 실측 — 0.25 안은 철회. XY travel 의 실제 고속화는 PTP(①)가 담당 |
| ③ LIN 최종 하강·approach (적층 접촉) | 0.1 | **0.1 유지** | 0.15 (150 mm/s) | 속도 문제가 아니라 적층 충격 — 상향 시 피라미드 붕괴 위험 |
| ④ LIN slow, z≥0.50 싱귤러 존 | 0.05 / 0.04 | 0.08 / 0.06 | **0.1** (손목 ~128°/s, 마진 1.75×) | §1 실측: scale 0.175 에서 손목 한계 도달 |
| ⑤ native movel staging | 150 / 300 | 250 / 400 | **400 mm/s / 600 mm/s²** | 빈손 구간. TCP 한계 1.0 m/s 의 40% |

## 4. 떨림 방지 규칙 (수치 가드)

1. **피크 조인트 속도 ≤ 한계의 70%** — 손목(J4–6) 기준 **157°/s** 이하.
   한계 근접 시 안전 컨트롤러 개입 = 덜컹거림/강제정지.
2. **가속 스케일 ≤ 0.2** (Cartesian ≈ 0.45 m/s²) — JTC 가 position-only 인터페이스
   (jitter fix, doosan fork `754f6f9`)라 과도 가속은 추종 오차 → 진동으로 나타난다.
   vel 을 올리더라도 acc 는 0.15~0.2 에서 멈출 것.
3. **싱귤러 존(z ≥ 0.50 m) LIN scale ≤ 0.1** — 절대 상한 (§1 실측 외삽).
4. **컵 파지 중 TCP 가속 ≤ 0.5 m/s²** — 그리퍼 내 컵 흔들림/슬립 방지.
   (추정 시작점 — 실로봇 검증으로 보정할 것.)

## 5. 적용 위치 & 검증 절차

- 수정 지점: `cup_stack/runtime.py` `_make_ptp_params` / `_make_lin_params` /
  `_make_lin_slow_params`, server `server/domains/robot.py` 의 `vel`/`acc` 배열.
- 검증: **한 구간씩** 상향(권장 순서 ② → ① → ⑤ → ④) 후 매회:
  1. `/dsr01/joint_states` velocity 피크가 §4-1 가드 이내인지 확인
  2. `FollowJointTrajectory` ABORTED 로그 부재 확인
  3. 육안 진동·적층 정확도 확인 (② 상향 첫 run 은 컵 1개로)
- ③(적층 접촉 하강)은 사이클타임 이득이 작고 리스크가 커서 마지막까지 유지 권장.
