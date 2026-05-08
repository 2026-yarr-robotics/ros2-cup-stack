# SpeedStack — Pipeline & I/O Specification (Comprehensive)

LLM 노드 중심으로 파이프라인 전 구간의 입출력을 매우 상세히 정의한다.

## 목차

1. [시스템 아키텍처 개요](#1-시스템-아키텍처-개요)
2. [코어 데이터 타입](#2-코어-데이터-타입)
3. [LLM 입력 페이로드](#3-llm-입력-페이로드)
4. [LLM 출력 페이로드](#4-llm-출력-페이로드)
5. [End-to-End 워크스루: 정상 3단 피라미드](#5-end-to-end-워크스루-정상-3단-피라미드)
6. [End-to-End 워크스루: 외란 복구](#6-end-to-end-워크스루-외란-복구)
7. [엣지 케이스 시나리오](#7-엣지-케이스-시나리오)
8. [Validation 체크리스트](#8-validation-체크리스트)
9. [구현 참고 (API 호출 코드)](#9-구현-참고-api-호출-코드)

---

## 1. 시스템 아키텍처 개요

### 1.1 컴포넌트 구성

```
┌──────────────────┐                 ┌──────────────────┐
│  Perception Node │  /world_state   │ Robot Driver     │  /robot_state
│  (YOLO+ByteTrack)│ ──────────────► │ (M0609 + RG6)    │ ──────────────►
│                  │                 │                  │
└──────────────────┘                 └──────────────────┘
         │                                     │
         │                                     │
         ▼                                     ▼
┌──────────────────────────────────────────────────────────┐
│              Goal State Publisher (GSP)                  │
│  - Maintains: current_plan, current_goal, history        │
│  - Triggers LLM on:                                      │
│      (a) cold-start (no plan exists)                     │
│      (b) skill execution finished (success or fail)      │
│      (c) world_state change detected by Perception       │
└──────────────────────────────────────────────────────────┘
         │                                     ▲
         │ build payload                       │ LLM response
         ▼                                     │
┌──────────────────────────────────────────────────────────┐
│              LLM Node (Qwen3.6-35B-A3B)                  │
│  - Two prompts: Cold-start Planner / In-flight Decider   │
│  - JSON Schema enforced via Ollama format param          │
│  - Temperature 0.0                                       │
└──────────────────────────────────────────────────────────┘
         │
         │ decision + plan
         ▼
┌──────────────────────────────────────────────────────────┐
│              Skill Executor (MoveIt2 + RG6)              │
│  - Atomic skills: pick(cup_id), place(target_slot)       │
│  - Returns: ActionResult (success/fail + reason)         │
└──────────────────────────────────────────────────────────┘
```

### 1.2 LLM 호출 프로토콜

| 트리거 | 모드 | 설명 |
|---|---|---|
| 사용자 명령 입력 (current_plan == null) | **Cold-start** | 전체 plan 생성 |
| Skill 실행 완료 (success) | **In-flight** | continue / replan / done 결정 |
| Skill 실행 실패 (fail) | **In-flight** | replan 결정 (대부분) |
| Perception이 world_state 변화 감지 (skill 진행 중 아님) | **In-flight** | replan 결정 (대부분) |

GSP는 매 호출마다 입력 페이로드를 빌드해서 LLM에 보낸다.

### 1.3 ROS2 토픽 매핑 (참고)

| 토픽 | 발행자 | 구독자 | 메시지 |
|---|---|---|---|
| `/world_state` | Perception | GSP, Dashboard | `WorldState` |
| `/robot_state` | Robot Driver | GSP, Dashboard | `RobotState` |
| `/skill_request` | GSP | Skill Executor | `SkillRequest` |
| `/skill_result` | Skill Executor | GSP | `ActionResult` |
| `/llm_decision` | GSP (after LLM call) | Dashboard | `LLMDecision` |
| `/user_command` | Dashboard / CLI | GSP | `UserCommand` |

### 1.4 타이밍 모델

한 사이클의 시간 분배 (NFR-01 기준 4~7초):

```
[Skill 실행 시작]
   ↓ ~3-5s (pick or place motion)
[Skill 완료, ActionResult 수신]
   ↓ ~0.1s (perception scan + world_state update)
[GSP가 LLM 입력 페이로드 빌드]
   ↓ ~0.05s (JSON 직렬화)
[LLM 호출]
   ↓ ~0.5-1.0s (Qwen3.6-35B-A3B, MoE active 3B, JSON 강제)
[LLM 응답 수신, GSP가 next skill 결정]
   ↓ ~0.1s
[다음 Skill 실행 시작]
```

---

## 2. 코어 데이터 타입

### 2.1 `Cup` — 작업대 위 컵 1개

```json
{
  "id": 3,
  "color": "red",
  "state": "upright",
  "graspable": true
}
```

| 필드 | 타입 | 값 범위 | 설명 |
|---|---|---|---|
| `id` | int | 1~99 | YOLO+ByteTrack이 부여한 고유 ID. 단일 task 내에서 일관성 유지. 추적 실패 시 위치 기반 재할당 가능 (FR-005). |
| `color` | string | `"red"`, `"blue"`, `"green"`, `"yellow"` | YOLO 분류 결과. 미지원 색은 `"unknown"`. |
| `state` | string | `"upright"`, `"fallen"` | 컵의 자세. `fallen`은 옆으로 누워있음. |
| `graspable` | bool | `true` / `false` | Perception이 종합 판정: 직립 + 그리퍼 도달 가능 + 다른 컵에 가려지지 않음. |

> **graspable=false 사유 예시**: 누워있음, 다른 컵에 막혀있음, 작업 영역 밖, 이미 stack에 들어감, 그리퍼에 들려있음.

### 2.2 `Stack` — 피라미드 슬롯 6개 점유 상태

```json
{
  "L1_left":  {"cup_id": 5, "color": "red"},
  "L1_mid":   {"cup_id": 7, "color": "blue"},
  "L1_right": null,
  "L2_left":  null,
  "L2_right": null,
  "L3_top":   null
}
```

각 슬롯은 `null` (비어있음) 또는 `{"cup_id": int, "color": string}` 객체.

**슬롯 정의 (고정)**:

```
        ┌─────────┐
        │ L3_top  │     ← Layer 3 (1슬롯)
        └─────────┘
    ┌─────────┬─────────┐
    │ L2_left │L2_right │  ← Layer 2 (2슬롯)
    └─────────┴─────────┘
┌─────────┬─────────┬─────────┐
│ L1_left │ L1_mid  │L1_right │  ← Layer 1 (3슬롯)
└─────────┴─────────┴─────────┘
```

**빌드 순서 (불변)**: `L1_left → L1_mid → L1_right → L2_left → L2_right → L3_top`

### 2.3 `WorldState` — 작업 공간 전체 상태 (Perception 발행)

```json
{
  "cups_on_table": [
    {"id": 1, "color": "red",  "state": "upright", "graspable": true},
    {"id": 2, "color": "blue", "state": "upright", "graspable": true}
  ],
  "stack": {
    "L1_left":  null,
    "L1_mid":   null,
    "L1_right": null,
    "L2_left":  null,
    "L2_right": null,
    "L3_top":   null
  },
  "filled_slots": 0,
  "total_slots": 6
}
```

| 필드 | 설명 |
|---|---|
| `cups_on_table` | 작업대 위 (stack 안 아닌, 그리퍼에 들리지 않은) 컵들의 리스트. |
| `stack` | 슬롯별 점유 상태 (Section 2.2). |
| `filled_slots` | `stack`의 non-null 슬롯 개수. |
| `total_slots` | 항상 `6`. (Target pattern과 무관하게 슬롯은 6개로 고정.) |

> **컵의 "위치 분류" 규칙**:
> - 작업대에 자유로이 놓여있음 → `cups_on_table`
> - 그리퍼가 들고 있음 → `cups_on_table`에서 제외, `RobotState.gripper.holding_cup_id`로 표현
> - Stack의 어느 슬롯에 적재됨 → `cups_on_table`에서 제외, `stack[slot]`으로 표현
> - 한 컵은 동시에 한 곳에만 존재. 카테고리 간 이동은 명시적 action 결과로만 일어남.

### 2.4 `RobotState` — 로봇 상태 (Robot Driver 발행)

```json
{
  "gripper": {
    "holding_cup_id": 3,
    "force_n": 2.3
  }
}
```

| 필드 | 타입 | 설명 |
|---|---|---|
| `gripper.holding_cup_id` | int \| null | 들고 있는 컵 ID. 빈손이면 `null`. |
| `gripper.force_n` | float | RG6 force sensor 측정값 (Newton). 0 근처면 빈손, 1.5 이상이면 holding 강하게 의심. |

> **참고**: M0609 joint 6개의 raw 값은 LLM 의사결정에 의미 없으므로 입력에서 제외. 충돌 회피와 IK는 MoveIt2 담당.
> **holding 판정 합의 규칙 (FR-010)**: `force_n > 1.5` AND `Perception이 그리퍼 ROI에서 컵을 인식` 둘 다 만족해야 `holding_cup_id`를 set. 둘 중 하나만 만족하면 `null`로 설정 후 `failure_reason: GRIPPER_EMPTY` 발생.

### 2.5 `Plan` — 전체 작업 계획

```json
{
  "plan_id": "plan_001",
  "target_pattern": "pyramid_3level",
  "remaining_steps": [
    {"step": 5, "action": "pick",  "cup_id": 7},
    {"step": 6, "action": "place", "target_slot": "L1_right"},
    {"step": 7, "action": "pick",  "cup_id": 1},
    {"step": 8, "action": "place", "target_slot": "L2_left"}
  ]
}
```

| 필드 | 설명 |
|---|---|
| `plan_id` | GSP가 부여하는 식별자 (디버깅/로그용). 새 plan이 발행될 때마다 증가 (`plan_001`, `plan_002`, ...). |
| `target_pattern` | `"pyramid_1level"` / `"pyramid_2level"` / `"pyramid_3level"` |
| `remaining_steps` | 아직 실행되지 않은 step들의 리스트. 실행될 때마다 GSP가 앞에서부터 제거. |

### 2.6 `Step` — 원자적 작업 1개

두 형태 중 하나:

```json
{"step": 1, "action": "pick", "cup_id": 3}
{"step": 2, "action": "place", "target_slot": "L1_left"}
```

| 필드 | 설명 |
|---|---|
| `step` | 1부터 시작하는 순번. 전체 plan 내에서 고유. |
| `action` | `"pick"` 또는 `"place"`. |
| `cup_id` | (pick만) 집을 컵의 ID. |
| `target_slot` | (place만) 놓을 슬롯 이름. |

**규칙**: pick과 place는 반드시 교대로 등장 (pick → place → pick → place → ...). 첫 step은 보통 pick (그리퍼 빈손 시작).

### 2.7 `Goal` — GSP가 다음에 실행하려는 step

```json
{"step": 5, "action": "pick", "cup_id": 7}
```

`current_plan.remaining_steps[0]`과 동일. 별도 필드로 노출하는 이유: LLM이 "다음에 뭘 하려는지" 명시적으로 검증할 수 있게.

**모든 step 완료 후**: `current_goal`은 `null`이 됨 (이때 LLM은 done 판정).

### 2.8 `ActionResult` — Skill 실행 결과 (Skill Executor 발행)

```json
{
  "step": 4,
  "action": "place",
  "target_slot": "L1_mid",
  "result": "success",
  "failure_reason": null
}
```

```json
{
  "step": 5,
  "action": "pick",
  "cup_id": 7,
  "result": "fail",
  "failure_reason": "GRIPPER_EMPTY"
}
```

`failure_reason` 가능 값 (`result == "fail"`일 때만 non-null):

| 사유 | 트리거 | LLM 권장 반응 |
|---|---|---|
| `NO_IK` | 역기구학 해 없음 | replan, 다른 접근 자세로 재시도 |
| `COLLISION` | M0609 충돌 감지 | replan, 안전 자세 후 재계획 |
| `GRIPPER_EMPTY` | pick 후 force sensor가 빈손 판정 | replan, 같은 컵 재시도 또는 다른 컵으로 |
| `OBJECT_NOT_FOUND` | 목표 컵 인식 실패 | replan, 빠진 컵 제외하고 |
| `OBJECT_MOVED` | 접근 중 컵이 움직임 | replan, 갱신된 좌표로 |
| `EXECUTION_TIMEOUT` | 시간 초과 | replan, 보수적으로 |
| `DROP_DETECTED` | 운반 중 컵 떨어뜨림 | replan, 빈손 + 컵 위치 갱신 후 |

---

## 3. LLM 입력 페이로드

### 3.1 통합 스키마 (두 모드 공통)

```json
{
  "user_command": "string | null",
  "current_world_state": "WorldState",
  "previous_world_state": "WorldState | null",
  "robot_state": "RobotState",
  "current_plan": "Plan | null",
  "current_goal": "Step | null",
  "last_action_result": "ActionResult | null"
}
```

> **previous_world_state 정의**: 직전 skill이 실행 시작된 시점의 world_state 스냅샷. 외란이 없었다면 `current_world_state`와 deterministic하게 1단계 차이가 남. 외란 있으면 그 외 차이가 더 있음.

### 3.2 모드별 채움 규칙

| 필드 | Cold-start | In-flight (정상) | In-flight (외란) | In-flight (마지막 step 후) |
|---|---|---|---|---|
| `user_command` | ✅ 사용자 명령 | `null` | `null` | `null` |
| `current_world_state` | ✅ stack 모두 null | ✅ | ✅ | ✅ stack 모두 채워짐 |
| `previous_world_state` | `null` | ✅ 직전 skill 시작 시점 | ✅ 직전 skill 시작 시점 | ✅ |
| `robot_state` | ✅ 빈손 | ✅ | ✅ | ✅ 빈손 |
| `current_plan` | `null` | ✅ remaining_steps 있음 | ✅ remaining_steps 있음 | ✅ remaining_steps `[]` |
| `current_goal` | `null` | ✅ 다음 step | ✅ 다음 step | `null` |
| `last_action_result` | `null` | ✅ success | ✅ (보통 success, 또는 fail) | ✅ 마지막 step success |

### 3.3 필드별 상세 명세

#### `user_command` (string \| null)
- Cold-start에서만 채움. 그 외엔 `null`.
- 자연어. 예: `"Stack a 3-level pyramid"`, `"3단 쌓아줘"`, `"Make a 2-level pyramid with red cups on the bottom"`.
- 색상 제약 포함 가능.

#### `current_world_state` (WorldState)
- 항상 채움. Perception의 가장 최신 스냅샷.

#### `previous_world_state` (WorldState \| null)
- Cold-start에서 `null`.
- In-flight에서는 직전 skill이 실행되기 직전의 world_state.
- LLM이 `current` vs `previous` 차이를 보고 외란 감지.

#### `robot_state` (RobotState)
- 항상 채움. 가장 최신.

#### `current_plan` (Plan \| null)
- Cold-start에서 `null`.
- In-flight에서는 진행 중인 plan. `remaining_steps`는 아직 실행 안 한 step만.

#### `current_goal` (Step \| null)
- Cold-start에서 `null`.
- In-flight에서는 GSP가 다음에 실행하려는 step (= `current_plan.remaining_steps[0]`).
- 모든 step 완료 시 `null`.

#### `last_action_result` (ActionResult \| null)
- Cold-start에서 `null`.
- In-flight에서는 가장 최근에 실행된 step의 결과.

---

## 4. LLM 출력 페이로드

### 4.1 Cold-start 출력

```json
{
  "reasoning": "Building 3-level pyramid bottom-up using 6 graspable cups, no color constraint.",
  "plan": {
    "target_pattern": "pyramid_3level",
    "steps": [
      {"step": 1, "action": "pick",  "cup_id": 1},
      {"step": 2, "action": "place", "target_slot": "L1_left"},
      {"step": 3, "action": "pick",  "cup_id": 2},
      {"step": 4, "action": "place", "target_slot": "L1_mid"},
      {"step": 5, "action": "pick",  "cup_id": 3},
      {"step": 6, "action": "place", "target_slot": "L1_right"},
      {"step": 7, "action": "pick",  "cup_id": 4},
      {"step": 8, "action": "place", "target_slot": "L2_left"},
      {"step": 9, "action": "pick",  "cup_id": 5},
      {"step": 10, "action": "place", "target_slot": "L2_right"},
      {"step": 11, "action": "pick",  "cup_id": 6},
      {"step": 12, "action": "place", "target_slot": "L3_top"}
    ]
  }
}
```

**필드 제약**:
- `reasoning`: 한 문장, 최대 200자.
- `plan.target_pattern`: `pyramid_1level` (6 steps) / `pyramid_2level` (10 steps) / `pyramid_3level` (12 steps) 중 하나.
- `plan.steps`: 정확히 위 step 수. pick-place 교대.

### 4.2 In-flight 출력

```json
{
  "reasoning": "...",
  "decision": "continue" | "replan" | "done",
  "plan": null | "Plan"
}
```

`decision`별 출력 형태:

**continue**:
```json
{
  "reasoning": "Place L1_mid succeeded, no disturbance, next pick of cup 3 feasible.",
  "decision": "continue",
  "plan": null
}
```

**replan**:
```json
{
  "reasoning": "Cup 1 removed from L1_left unexpectedly, replanning to refill missing slot first.",
  "decision": "replan",
  "plan": {
    "target_pattern": "pyramid_3level",
    "steps": [
      {"step": 1, "action": "pick",  "cup_id": 1},
      {"step": 2, "action": "place", "target_slot": "L1_left"},
      {"step": 3, "action": "pick",  "cup_id": 3},
      {"step": 4, "action": "place", "target_slot": "L1_right"},
      {"step": 5, "action": "pick",  "cup_id": 4},
      {"step": 6, "action": "place", "target_slot": "L2_left"},
      {"step": 7, "action": "pick",  "cup_id": 5},
      {"step": 8, "action": "place", "target_slot": "L2_right"},
      {"step": 9, "action": "pick",  "cup_id": 6},
      {"step": 10, "action": "place", "target_slot": "L3_top"}
    ]
  }
}
```

**done**:
```json
{
  "reasoning": "Final place at L3_top succeeded, all 6 slots filled, no remaining steps.",
  "decision": "done",
  "plan": null
}
```

### 4.3 JSON Schema (Ollama format / vLLM guided_json)

#### Cold-start

```json
{
  "type": "object",
  "required": ["reasoning", "plan"],
  "additionalProperties": false,
  "properties": {
    "reasoning": {"type": "string", "maxLength": 200},
    "plan": {
      "type": "object",
      "required": ["target_pattern", "steps"],
      "additionalProperties": false,
      "properties": {
        "target_pattern": {
          "type": "string",
          "enum": ["pyramid_1level", "pyramid_2level", "pyramid_3level"]
        },
        "steps": {
          "type": "array",
          "minItems": 6,
          "maxItems": 12,
          "items": {
            "type": "object",
            "required": ["step", "action"],
            "properties": {
              "step": {"type": "integer", "minimum": 1},
              "action": {"type": "string", "enum": ["pick", "place"]},
              "cup_id": {"type": "integer", "minimum": 1},
              "target_slot": {
                "type": "string",
                "enum": ["L1_left", "L1_mid", "L1_right", "L2_left", "L2_right", "L3_top"]
              }
            }
          }
        }
      }
    }
  }
}
```

#### In-flight

```json
{
  "type": "object",
  "required": ["reasoning", "decision", "plan"],
  "additionalProperties": false,
  "properties": {
    "reasoning": {"type": "string", "maxLength": 200},
    "decision": {
      "type": "string",
      "enum": ["continue", "replan", "done"]
    },
    "plan": {
      "oneOf": [
        {"type": "null"},
        {
          "type": "object",
          "required": ["target_pattern", "steps"],
          "additionalProperties": false,
          "properties": {
            "target_pattern": {
              "type": "string",
              "enum": ["pyramid_1level", "pyramid_2level", "pyramid_3level"]
            },
            "steps": {
              "type": "array",
              "minItems": 1,
              "maxItems": 12,
              "items": {
                "type": "object",
                "required": ["step", "action"],
                "properties": {
                  "step": {"type": "integer", "minimum": 1},
                  "action": {"type": "string", "enum": ["pick", "place"]},
                  "cup_id": {"type": "integer", "minimum": 1},
                  "target_slot": {
                    "type": "string",
                    "enum": ["L1_left", "L1_mid", "L1_right", "L2_left", "L2_right", "L3_top"]
                  }
                }
              }
            }
          }
        }
      ]
    }
  }
}
```

> **Schema가 못 잡는 의미적 제약**: pick-place 교대 패턴, target_slot 빌드 순서, cup_id 중복 사용 금지, gripper holding 시 first step이 place인지. Section 8의 사후 validation으로 보강.

---

## 5. End-to-End 워크스루: 정상 3단 피라미드

작업대 초기 상태: 6개 컵 (red 3, blue 2, green 1).

```
[초기 작업대]
cups: id=1(red), id=2(red), id=3(red), id=4(blue), id=5(blue), id=6(green)
stack: 모두 비어있음
gripper: 빈손
```

### 사이클 0 — Cold-start

**트리거**: 사용자가 "Stack a 3-level pyramid" 입력. GSP가 `current_plan == null` 확인 후 Cold-start LLM 호출.

**LLM 입력 페이로드**:

```json
{
  "user_command": "Stack a 3-level pyramid",
  "current_world_state": {
    "cups_on_table": [
      {"id": 1, "color": "red",   "state": "upright", "graspable": true},
      {"id": 2, "color": "red",   "state": "upright", "graspable": true},
      {"id": 3, "color": "red",   "state": "upright", "graspable": true},
      {"id": 4, "color": "blue",  "state": "upright", "graspable": true},
      {"id": 5, "color": "blue",  "state": "upright", "graspable": true},
      {"id": 6, "color": "green", "state": "upright", "graspable": true}
    ],
    "stack": {
      "L1_left": null, "L1_mid": null, "L1_right": null,
      "L2_left": null, "L2_right": null,
      "L3_top": null
    },
    "filled_slots": 0,
    "total_slots": 6
  },
  "previous_world_state": null,
  "robot_state": {"gripper": {"holding_cup_id": null, "force_n": 0.0}},
  "current_plan": null,
  "current_goal": null,
  "last_action_result": null
}
```

**LLM 출력**:

```json
{
  "reasoning": "Building 3-level pyramid bottom-up using 6 graspable cups, no color constraint specified.",
  "plan": {
    "target_pattern": "pyramid_3level",
    "steps": [
      {"step": 1, "action": "pick",  "cup_id": 1},
      {"step": 2, "action": "place", "target_slot": "L1_left"},
      {"step": 3, "action": "pick",  "cup_id": 2},
      {"step": 4, "action": "place", "target_slot": "L1_mid"},
      {"step": 5, "action": "pick",  "cup_id": 3},
      {"step": 6, "action": "place", "target_slot": "L1_right"},
      {"step": 7, "action": "pick",  "cup_id": 4},
      {"step": 8, "action": "place", "target_slot": "L2_left"},
      {"step": 9, "action": "pick",  "cup_id": 5},
      {"step": 10, "action": "place", "target_slot": "L2_right"},
      {"step": 11, "action": "pick",  "cup_id": 6},
      {"step": 12, "action": "place", "target_slot": "L3_top"}
    ]
  }
}
```

**GSP 동작**: `plan_001` 저장. `remaining_steps`에 12개 step 모두 적재. step 1 (`pick cup_id=1`)을 Skill Executor에 발행.

### 사이클 1 — Step 1 (pick cup_id=1) 성공 후

**트리거**: Skill Executor가 `pick(cup_id=1)` success 반환.

**상태 변화**:
- `cup_id=1`이 `cups_on_table`에서 빠지고 `gripper.holding_cup_id=1`로 이동.

**LLM 입력 페이로드**:

```json
{
  "user_command": null,
  "current_world_state": {
    "cups_on_table": [
      {"id": 2, "color": "red",   "state": "upright", "graspable": true},
      {"id": 3, "color": "red",   "state": "upright", "graspable": true},
      {"id": 4, "color": "blue",  "state": "upright", "graspable": true},
      {"id": 5, "color": "blue",  "state": "upright", "graspable": true},
      {"id": 6, "color": "green", "state": "upright", "graspable": true}
    ],
    "stack": {
      "L1_left": null, "L1_mid": null, "L1_right": null,
      "L2_left": null, "L2_right": null,
      "L3_top": null
    },
    "filled_slots": 0,
    "total_slots": 6
  },
  "previous_world_state": {
    "cups_on_table": [
      {"id": 1, "color": "red",   "state": "upright", "graspable": true},
      {"id": 2, "color": "red",   "state": "upright", "graspable": true},
      {"id": 3, "color": "red",   "state": "upright", "graspable": true},
      {"id": 4, "color": "blue",  "state": "upright", "graspable": true},
      {"id": 5, "color": "blue",  "state": "upright", "graspable": true},
      {"id": 6, "color": "green", "state": "upright", "graspable": true}
    ],
    "stack": {
      "L1_left": null, "L1_mid": null, "L1_right": null,
      "L2_left": null, "L2_right": null,
      "L3_top": null
    },
    "filled_slots": 0,
    "total_slots": 6
  },
  "robot_state": {"gripper": {"holding_cup_id": 1, "force_n": 2.4}},
  "current_plan": {
    "plan_id": "plan_001",
    "target_pattern": "pyramid_3level",
    "remaining_steps": [
      {"step": 2,  "action": "place", "target_slot": "L1_left"},
      {"step": 3,  "action": "pick",  "cup_id": 2},
      {"step": 4,  "action": "place", "target_slot": "L1_mid"},
      {"step": 5,  "action": "pick",  "cup_id": 3},
      {"step": 6,  "action": "place", "target_slot": "L1_right"},
      {"step": 7,  "action": "pick",  "cup_id": 4},
      {"step": 8,  "action": "place", "target_slot": "L2_left"},
      {"step": 9,  "action": "pick",  "cup_id": 5},
      {"step": 10, "action": "place", "target_slot": "L2_right"},
      {"step": 11, "action": "pick",  "cup_id": 6},
      {"step": 12, "action": "place", "target_slot": "L3_top"}
    ]
  },
  "current_goal": {"step": 2, "action": "place", "target_slot": "L1_left"},
  "last_action_result": {
    "step": 1,
    "action": "pick",
    "cup_id": 1,
    "result": "success",
    "failure_reason": null
  }
}
```

**LLM 출력**:

```json
{
  "reasoning": "Pick of cup 1 succeeded, gripper now holds it, next step places it at L1_left as planned.",
  "decision": "continue",
  "plan": null
}
```

**GSP 동작**: step 2 (`place L1_left`) 발행.

### 사이클 2 — Step 2 (place L1_left) 성공 후

**상태 변화**: gripper 빈손, stack[L1_left] = {cup_id:1, color:red}, filled_slots=1.

**LLM 입력 페이로드** (요약 — current_world_state와 robot_state만 발췌):

```json
{
  "current_world_state": {
    "cups_on_table": [
      {"id": 2, "color": "red", "state": "upright", "graspable": true},
      {"id": 3, "color": "red", "state": "upright", "graspable": true},
      {"id": 4, "color": "blue", "state": "upright", "graspable": true},
      {"id": 5, "color": "blue", "state": "upright", "graspable": true},
      {"id": 6, "color": "green", "state": "upright", "graspable": true}
    ],
    "stack": {
      "L1_left":  {"cup_id": 1, "color": "red"},
      "L1_mid": null, "L1_right": null,
      "L2_left": null, "L2_right": null,
      "L3_top": null
    },
    "filled_slots": 1,
    "total_slots": 6
  },
  "robot_state": {"gripper": {"holding_cup_id": null, "force_n": 0.0}},
  "current_goal": {"step": 3, "action": "pick", "cup_id": 2},
  "last_action_result": {
    "step": 2, "action": "place", "target_slot": "L1_left",
    "result": "success", "failure_reason": null
  }
  /* previous_world_state, current_plan은 직전 사이클의 current에서 step 2 실행 직전 시점 그대로 */
}
```

**LLM 출력**:
```json
{
  "reasoning": "Place at L1_left succeeded as planned, no disturbance, proceed to pick cup 2.",
  "decision": "continue",
  "plan": null
}
```

### 사이클 3~11 — 압축 표현

각 사이클마다 동일한 패턴:
- 직전 step success → world_state가 expected delta만큼 변화
- LLM은 `decision: "continue"` 반환

**사이클 종료 시점의 stack 상태**:

| 사이클 | 직전 실행 step | stack 상태 | gripper |
|---|---|---|---|
| 3 | pick cup 2 | L1_left: cup 1 | holding cup 2 |
| 4 | place L1_mid | L1_left, L1_mid 채움 | 빈손 |
| 5 | pick cup 3 | (위와 동일) | holding cup 3 |
| 6 | place L1_right | L1 전체 채움 | 빈손 |
| 7 | pick cup 4 | L1 전체 채움 | holding cup 4 |
| 8 | place L2_left | L1 + L2_left 채움 | 빈손 |
| 9 | pick cup 5 | (위와 동일) | holding cup 5 |
| 10 | place L2_right | L1 + L2 전체 채움 | 빈손 |
| 11 | pick cup 6 | (위와 동일) | holding cup 6 |
| 12 | place L3_top | 모든 슬롯 채움 | 빈손 |

### 사이클 12 — Step 12 (place L3_top) 성공 후 = Done 판정

**상태 변화**: stack[L3_top] = {cup_id:6, color:green}. filled_slots = 6 = total_slots. remaining_steps = [].

**LLM 입력 페이로드**:

```json
{
  "user_command": null,
  "current_world_state": {
    "cups_on_table": [],
    "stack": {
      "L1_left":  {"cup_id": 1, "color": "red"},
      "L1_mid":   {"cup_id": 2, "color": "red"},
      "L1_right": {"cup_id": 3, "color": "red"},
      "L2_left":  {"cup_id": 4, "color": "blue"},
      "L2_right": {"cup_id": 5, "color": "blue"},
      "L3_top":   {"cup_id": 6, "color": "green"}
    },
    "filled_slots": 6,
    "total_slots": 6
  },
  "previous_world_state": {
    "cups_on_table": [
      {"id": 6, "color": "green", "state": "upright", "graspable": true}
    ],
    "stack": {
      "L1_left":  {"cup_id": 1, "color": "red"},
      "L1_mid":   {"cup_id": 2, "color": "red"},
      "L1_right": {"cup_id": 3, "color": "red"},
      "L2_left":  {"cup_id": 4, "color": "blue"},
      "L2_right": {"cup_id": 5, "color": "blue"},
      "L3_top":   null
    },
    "filled_slots": 5,
    "total_slots": 6
  },
  "robot_state": {"gripper": {"holding_cup_id": null, "force_n": 0.0}},
  "current_plan": {
    "plan_id": "plan_001",
    "target_pattern": "pyramid_3level",
    "remaining_steps": []
  },
  "current_goal": null,
  "last_action_result": {
    "step": 12, "action": "place", "target_slot": "L3_top",
    "result": "success", "failure_reason": null
  }
}
```

**LLM 출력**:

```json
{
  "reasoning": "Final place at L3_top succeeded, all 6 slots filled and remaining_steps empty, task complete.",
  "decision": "done",
  "plan": null
}
```

**GSP 동작**: 시스템 안전 자세 (home pose)로 복귀, task 종료.

---

## 6. End-to-End 워크스루: 외란 복구

같은 초기 조건이지만 **사이클 4 종료 직후 사용자가 L1_left에서 cup 1을 빼냄**.

### 사이클 0~4 — 정상 진행

Section 5와 동일. 사이클 4 종료 시점:

```
stack:
  L1_left:  cup 1 (red)
  L1_mid:   cup 2 (red)
  L1_right: null
  L2_*, L3_top: null
filled_slots: 2
gripper: 빈손
remaining_steps: [step 5 (pick cup 3), step 6 (place L1_right), ..., step 12]
```

### 외란 발생

사용자가 L1_left에서 cup 1을 집어다가 작업대에 다시 놓음. (외란 발생을 GSP가 감지하는 방법은 두 가지 — Perception이 `/world_state` 변화 감지 후 즉시 publish, 또는 사이클 5 시작 직전의 다음 LLM 호출에서 LLM이 알아챔. 여기선 **후자**로 가정한다.)

### 사이클 5 — Step 5 (pick cup 3) 실행 후, Replan 발생

**상태 변화 (외란 포함)**:
- step 5 (`pick cup_id=3`)는 정상 success
- 하지만 Perception이 cup 1이 L1_left에서 사라지고 작업대에 다시 나타남을 감지

**LLM 입력 페이로드**:

```json
{
  "user_command": null,
  "current_world_state": {
    "cups_on_table": [
      {"id": 1, "color": "red",   "state": "upright", "graspable": true},
      {"id": 4, "color": "blue",  "state": "upright", "graspable": true},
      {"id": 5, "color": "blue",  "state": "upright", "graspable": true},
      {"id": 6, "color": "green", "state": "upright", "graspable": true}
    ],
    "stack": {
      "L1_left":  null,
      "L1_mid":   {"cup_id": 2, "color": "red"},
      "L1_right": null,
      "L2_left": null, "L2_right": null,
      "L3_top": null
    },
    "filled_slots": 1,
    "total_slots": 6
  },
  "previous_world_state": {
    "cups_on_table": [
      {"id": 3, "color": "red",   "state": "upright", "graspable": true},
      {"id": 4, "color": "blue",  "state": "upright", "graspable": true},
      {"id": 5, "color": "blue",  "state": "upright", "graspable": true},
      {"id": 6, "color": "green", "state": "upright", "graspable": true}
    ],
    "stack": {
      "L1_left":  {"cup_id": 1, "color": "red"},
      "L1_mid":   {"cup_id": 2, "color": "red"},
      "L1_right": null,
      "L2_left": null, "L2_right": null,
      "L3_top": null
    },
    "filled_slots": 2,
    "total_slots": 6
  },
  "robot_state": {"gripper": {"holding_cup_id": 3, "force_n": 2.5}},
  "current_plan": {
    "plan_id": "plan_001",
    "target_pattern": "pyramid_3level",
    "remaining_steps": [
      {"step": 6,  "action": "place", "target_slot": "L1_right"},
      {"step": 7,  "action": "pick",  "cup_id": 4},
      {"step": 8,  "action": "place", "target_slot": "L2_left"},
      {"step": 9,  "action": "pick",  "cup_id": 5},
      {"step": 10, "action": "place", "target_slot": "L2_right"},
      {"step": 11, "action": "pick",  "cup_id": 6},
      {"step": 12, "action": "place", "target_slot": "L3_top"}
    ]
  },
  "current_goal": {"step": 6, "action": "place", "target_slot": "L1_right"},
  "last_action_result": {
    "step": 5, "action": "pick", "cup_id": 3,
    "result": "success", "failure_reason": null
  }
}
```

**LLM 분석 (reasoning에 들어갈 사고 흐름)**:
- last_action_result: step 5 pick cup 3 success.
- expected delta: cup 3이 cups_on_table에서 사라지고 gripper.holding_cup_id=3으로 이동.
- actual delta: cup 3 이동 ✓ (expected) **+ cup 1이 L1_left에서 사라지고 cups_on_table에 등장 (unexpected)**.
- 외란 감지 → replan 필요.
- 현재 빈 슬롯: L1_left, L1_right, L2_left, L2_right, L3_top.
- gripper에 cup 3을 들고 있음 → first step은 place여야 함.
- 빌드 순서상 다음 채울 슬롯은 L1_left (가장 왼쪽 빈 슬롯). cup 3을 L1_left에 둔다.
- 그 다음 L1_right (cup 1로), 그 다음 L2_*, L3_top.

**LLM 출력**:

```json
{
  "reasoning": "Cup 1 removed from L1_left unexpectedly while pick of cup 3 succeeded; replanning to place held cup 3 at L1_left and refill remaining slots.",
  "decision": "replan",
  "plan": {
    "target_pattern": "pyramid_3level",
    "steps": [
      {"step": 1, "action": "place", "target_slot": "L1_left"},
      {"step": 2, "action": "pick",  "cup_id": 1},
      {"step": 3, "action": "place", "target_slot": "L1_right"},
      {"step": 4, "action": "pick",  "cup_id": 4},
      {"step": 5, "action": "place", "target_slot": "L2_left"},
      {"step": 6, "action": "pick",  "cup_id": 5},
      {"step": 7, "action": "place", "target_slot": "L2_right"},
      {"step": 8, "action": "pick",  "cup_id": 6},
      {"step": 9, "action": "place", "target_slot": "L3_top"}
    ]
  }
}
```

> **포인트**: 새 plan은 `step` 번호를 1부터 다시 시작. plan_id는 GSP가 `plan_002`로 부여 (LLM 출력에는 없음).
> **포인트**: 첫 step이 `place` (gripper holding cup 3 → L1_left). 그 다음 pick은 cup 1 (작업대 위로 돌아온 컵).

**GSP 동작**: `plan_001`을 폐기하고 `plan_002` 저장. 새 step 1 (`place L1_left`) 발행.

### 사이클 6~14 — 새 plan으로 진행

각 사이클마다 `decision: "continue"` 반환되며 정상 진행. 사이클 14 (= 새 plan의 step 9 = `place L3_top` 성공) 후 LLM이 `decision: "done"` 반환하며 종료.

---

## 7. 엣지 케이스 시나리오

### 7.1 Pick 실패 (GRIPPER_EMPTY)

**시나리오**: 사이클 1에서 step 1 (`pick cup 1`) 실행했지만 force sensor가 빈손 판정.

**LLM 입력**: `last_action_result.result = "fail"`, `failure_reason = "GRIPPER_EMPTY"`. world_state에 cup 1은 여전히 cups_on_table에 있음 (graspable 그대로 true).

**LLM 출력 (예상)**:
```json
{
  "reasoning": "Pick of cup 1 failed with gripper empty, cup still graspable on table; replanning to retry pick.",
  "decision": "replan",
  "plan": {
    "target_pattern": "pyramid_3level",
    "steps": [
      {"step": 1, "action": "pick",  "cup_id": 1},
      {"step": 2, "action": "place", "target_slot": "L1_left"},
      /* ...rest unchanged from original plan... */
    ]
  }
}
```

> **참고**: 같은 컵 재시도가 기본. 같은 컵에서 N회 연속 실패하면 다른 컵 시도 로직을 추가하는 건 향후 확장 사항.

### 7.2 Place 실패 (DROP_DETECTED)

**시나리오**: 사이클 2에서 step 2 (`place L1_left`) 실행 중 컵이 미끄러져 떨어짐.

**LLM 입력**: `failure_reason = "DROP_DETECTED"`. gripper 빈손, world_state에 cup 1이 작업대로 돌아옴 (떨어진 자리). stack 변화 없음.

**LLM 출력 (예상)**:
```json
{
  "reasoning": "Cup 1 dropped during place; cup back on table and graspable; replanning to retry pick-place sequence.",
  "decision": "replan",
  "plan": {
    "target_pattern": "pyramid_3level",
    "steps": [
      {"step": 1, "action": "pick",  "cup_id": 1},
      {"step": 2, "action": "place", "target_slot": "L1_left"},
      /* ...rest of plan... */
    ]
  }
}
```

### 7.3 색상 제약 명령

**Cold-start 입력**:
```json
{
  "user_command": "Stack a 2-level pyramid with red cups on the bottom layer",
  "current_world_state": { /* 6 cups: 3 red + 3 blue */ }
}
```

**LLM 출력 (예상)**:
```json
{
  "reasoning": "2-level pyramid requested with red on bottom; assigning 3 red cups to L1, 2 blue cups to L2.",
  "plan": {
    "target_pattern": "pyramid_2level",
    "steps": [
      {"step": 1, "action": "pick",  "cup_id": 1},
      {"step": 2, "action": "place", "target_slot": "L1_left"},
      {"step": 3, "action": "pick",  "cup_id": 2},
      {"step": 4, "action": "place", "target_slot": "L1_mid"},
      {"step": 5, "action": "pick",  "cup_id": 3},
      {"step": 6, "action": "place", "target_slot": "L1_right"},
      {"step": 7, "action": "pick",  "cup_id": 4},
      {"step": 8, "action": "place", "target_slot": "L2_left"},
      {"step": 9, "action": "pick",  "cup_id": 5},
      {"step": 10, "action": "place", "target_slot": "L2_right"}
    ]
  }
}
```

> id 1, 2, 3은 red, id 4, 5는 blue로 가정.

### 7.4 1단 / 2단 피라미드

**1단 (3 cups)**: `target_pattern: "pyramid_1level"`, steps 6개, target_slot: L1_left → L1_mid → L1_right만 사용.

**2단 (5 cups)**: `target_pattern: "pyramid_2level"`, steps 10개, L3_top 사용 안 함.

Done 판정: `filled_slots == 3` (1단) / `5` (2단) / `6` (3단).

### 7.5 Cold-start 시 graspable=true 컵이 부족

**시나리오**: 작업대에 3단을 위한 6개 컵이 필요한데, 일부가 누워있어 graspable=false.

**처리 방향**:
- 이 케이스는 LLM이 처리하기 애매함 (사용자 의도 불명).
- 권장: GSP가 페이로드 빌드 전에 `count(graspable=true) < required` 체크해서 LLM 호출 전 에러 반환.
- 또는 LLM이 그래도 plan 생성을 시도하되 reasoning에 부족함 명시 → GSP가 plan validation에서 잡아냄.

### 7.6 Done 직전 외란

**시나리오**: step 12 직전에 사용자가 L2_left에서 컵을 빼냄.

**LLM 처리**: filled_slots는 6에서 5로 떨어짐 → done 조건 불충족 → replan. 새 plan으로 빠진 슬롯 다시 채움.

---

## 8. Validation 체크리스트

LLM 출력 받은 후 GSP에서 1차 검증. 실패 시 LLM 재호출 또는 HITL 에스컬레이션.

### 8.1 공통 (모든 출력)
- [ ] JSON 파싱 성공
- [ ] JSON Schema validation 통과
- [ ] `reasoning` 200자 이하

### 8.2 Cold-start 출력
- [ ] `target_pattern`과 step 수 매칭: 1level=6, 2level=10, 3level=12
- [ ] 홀수 step은 action="pick", 짝수 step은 action="place"
- [ ] `target_slot` 순서가 빌드 순서와 일치 (L1_left → L1_mid → L1_right → L2_left → L2_right → L3_top)
- [ ] 모든 picked `cup_id`가 입력 페이로드의 `cups_on_table`에서 graspable=true
- [ ] `cup_id` 중복 사용 없음
- [ ] User command의 색상 제약 honored

### 8.3 In-flight 출력
- [ ] `decision="continue"` → `plan == null`
- [ ] `decision="done"` → `plan == null` AND 다음 3조건 만족:
  - [ ] `current_world_state.filled_slots == target count for current_plan.target_pattern`
  - [ ] `current_plan.remaining_steps == []`
  - [ ] `last_action_result.result == "success"`
- [ ] `decision="replan"` → `plan != null` AND:
  - [ ] `plan.target_pattern == current_plan.target_pattern` (preserved)
  - [ ] step alternation 정상 (gripper holding이면 first=place, 아니면 first=pick)
  - [ ] 모든 picked `cup_id`가 graspable=true (gripper에 들린 컵 제외)
  - [ ] 모든 `target_slot`이 현재 null인 슬롯
  - [ ] target_slot 순서가 빌드 순서를 위반하지 않음 (남은 슬롯 한정)

### 8.4 Validation 실패 시 동작

| 실패 유형 | 권장 동작 |
|---|---|
| JSON 파싱 실패 | LLM 재호출 (1회) → 또 실패 시 HITL |
| Schema validation 실패 | LLM 재호출 (1회) → 또 실패 시 HITL |
| 의미적 제약 위반 (cup_id 중복 등) | LLM 재호출 with 에러 사유 명시 (1회) → 또 실패 시 HITL |
| Done 조건 위반 | replan으로 fallback (LLM 재호출) |

---

## 9. 구현 참고 (API 호출 코드)

### 9.1 Ollama 호출 (Python)

```python
import json
import requests

OLLAMA_URL = "http://localhost:11434/api/chat"
MODEL = "qwen3:35b-a3b"  # 정확한 모델명은 ollama pull 후 확인

def call_llm(system_prompt: str, payload: dict, schema: dict) -> dict:
    """
    system_prompt: Cold-start 또는 In-flight 프롬프트 텍스트
    payload: section 3에서 정의한 입력 페이로드 dict
    schema: section 4.3의 JSON Schema dict
    """
    response = requests.post(OLLAMA_URL, json={
        "model": MODEL,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": json.dumps(payload, ensure_ascii=False)}
        ],
        "format": schema,
        "options": {"temperature": 0.0},
        "stream": False
    }, timeout=10.0)

    response.raise_for_status()
    content = response.json()["message"]["content"]
    return json.loads(content)
```

### 9.2 vLLM 호출 (OpenAI 호환 API)

```python
from openai import OpenAI

client = OpenAI(base_url="http://localhost:8000/v1", api_key="dummy")

def call_llm(system_prompt: str, payload: dict, schema: dict) -> dict:
    response = client.chat.completions.create(
        model="qwen3-35b-a3b",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": json.dumps(payload, ensure_ascii=False)}
        ],
        temperature=0.0,
        extra_body={"guided_json": schema}
    )
    return json.loads(response.choices[0].message.content)
```

### 9.3 GSP 메인 루프 (의사 코드)

```python
class GoalStatePublisher:
    def __init__(self):
        self.current_plan = None
        self.previous_world_state = None

    def on_user_command(self, user_command: str):
        # Cold-start 트리거
        payload = self.build_payload(mode="cold_start", user_command=user_command)
        response = call_llm(COLD_START_PROMPT, payload, COLD_START_SCHEMA)
        if not self.validate(response, mode="cold_start"):
            self.escalate_to_hitl(reason="cold_start validation failed")
            return
        self.current_plan = response["plan"]
        self.current_plan["plan_id"] = self.next_plan_id()
        self.execute_next_step()

    def on_skill_result(self, action_result: ActionResult):
        # In-flight 트리거
        payload = self.build_payload(mode="in_flight", last_action_result=action_result)
        response = call_llm(IN_FLIGHT_PROMPT, payload, IN_FLIGHT_SCHEMA)
        if not self.validate(response, mode="in_flight"):
            self.escalate_to_hitl(reason="in_flight validation failed")
            return

        if response["decision"] == "continue":
            self.execute_next_step()
        elif response["decision"] == "replan":
            self.current_plan = response["plan"]
            self.current_plan["plan_id"] = self.next_plan_id()
            self.execute_next_step()
        elif response["decision"] == "done":
            self.finalize_task()

    def execute_next_step(self):
        next_step = self.current_plan["remaining_steps"][0]
        self.previous_world_state = self.get_current_world_state()
        self.publish_skill_request(next_step)
```

---

## 부록 A — 참고 매핑

### Target pattern별 사용 슬롯

| Pattern | 슬롯 | 컵 수 | Step 수 |
|---|---|---|---|
| `pyramid_1level` | L1_left, L1_mid, L1_right | 3 | 6 |
| `pyramid_2level` | + L2_left, L2_right | 5 | 10 |
| `pyramid_3level` | + L3_top | 6 | 12 |

### Failure reason 빈도/심각도 (참고)

| Reason | 예상 빈도 | 심각도 | 일반 권장 반응 |
|---|---|---|---|
| GRIPPER_EMPTY | 중 | 낮 | 같은 컵 재시도 (replan) |
| OBJECT_MOVED | 중 | 중 | 갱신된 좌표로 retry (replan) |
| DROP_DETECTED | 중 | 중 | 재시도 (replan) |
| OBJECT_NOT_FOUND | 하 | 중 | 다른 컵으로 (replan) |
| NO_IK | 하 | 중 | 다른 자세 시도 (replan) |
| EXECUTION_TIMEOUT | 하 | 중 | 재시도 (replan) |
| COLLISION | 하 | 상 | 안전 자세 후 보수적 replan, 또는 HITL |

### LLM 호출 횟수 추정

| 시나리오 | 호출 수 |
|---|---|
| 정상 3단 피라미드 (외란 없음) | 1 (cold-start) + 12 (각 step 후) = **13회** |
| 외란 1회 발생 (replan 1회) | 13 + 추가 step 만큼 = **14~22회** |
| Pick 실패 1회 | 13 + 1 (replan) + 1 (재시도 후 continue) = **15회** |

전체 task 완료까지 LLM 호출 비용 = (대략 13~25회) × (0.5~1.0초/회) = **6~25초** 추론 시간 누적. NFR-01의 3분 이내 완성 목표에 충분히 들어감.
