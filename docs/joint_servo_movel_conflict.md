# Pyramid skill "forced interrupt": JOINT_SERVO vs native movel

## Symptom

- `move` skill (cup-stack-server `POST /api/robot/move`) works fine on its own.
- During the **pyramid** sequence the motion intermittently gets a "forced
  interrupt": a staging move silently does nothing, so the next pick/place
  runs from the wrong pose and the sequence breaks.

## Two motion paths

| Path | Used by | Implementation | DRCF effect |
|------|---------|----------------|-------------|
| `move_l` / `move_j` (Doosan native) | `move` endpoint | `/dsr01/motion/move_line` service → `Drfl->movel` | needs `STATE_STANDBY` |
| Pilz `PTP` / `LIN` (MoveIt) | pyramid / pick skill | MoveItPy `plan()`→`execute()` → `FollowJointTrajectory` → `servoj_rt` stream | puts/keeps DRCF in `JOINT_SERVO` |

The closed loop interleaves them per cup:
```
move(staging, native movel) → skill/pyramid_step(MoveIt PTP/LIN) → move → skill → …
```

## Root cause (confirmed by live repro)

MoveIt executes via `servoj_rt` streaming, which leaves the controller in
**`JOINT_SERVO`**. A subsequent native `movel` issues event `eMoveL`, which the
DRCF state machine rejects from `JOINT_SERVO`:

```
[INFO] [dsr_controller2]: movel_cb() called and calling Drfl->movel
[WARN] [dsr_controller2]:  param : state[JOINT_SERVO] rejected event[eMoveL]
```

`Drfl->movel` then returns `false` → the move does not execute.

The DRCF does **not** auto-return to `STATE_STANDBY`: the Doosan HW interface
disables auto servo-off at bringup —
`dsr_hardware2/src/dsr_hw_interface2.cpp:280` `Drfl.set_auto_servo_off(0, 5.0)`
("Deactivate it"). After the `servoj_rt` stream stops there is only a slow
(~1.4 s observed) natural transition back to `STANDBY`.

### Why it is intermittent
`/skill/pyramid_step` replies **at place/release time (step 8), on purpose**, so
the LLM can plan the next step while the final lift runs in a background thread
(`skill_api_node.py`). `busy` stays `true` through the lift. If the caller's next
native `movel` arrives while that background lift is still streaming (or within
the ~1.4 s `JOINT_SERVO → STANDBY` window), it is rejected; if it arrives later,
it succeeds.

### Live evidence (one pyramid run)
6 native moves interleaved with the skill, 3 rejected. The rejected ones returned
in ~30 ms (no motion); the accepted ones took ~2–3 s (real motion):

| move | duration | result |
|------|----------|--------|
| (0.25,−0.2) initial | 2016 ms | ✅ STANDBY |
| (0.25, 0.0) after step | 32 ms | ❌ rejected (JOINT_SERVO) |
| (0.25, 0.2) after step | 31 ms | ❌ rejected |
| (0.35,−0.2) after step | 2923 ms | ✅ |
| (0.35, 0.0) after step | 39 ms | ❌ rejected |
| (0.35, 0.2) after step | 2412 ms | ✅ |

Skill-side counters for the same run: `movel_cb called = 6`, `rejected eMoveL = 3`.

## Secondary bug

cup-stack-server `move_to` (`server/server/domains/robot.py`) reports
`success:true` even when `move_line` returns `success=false` (the ~30 ms calls
above logged 200 OK / `success:true`). The rejection is not surfaced. This lives
in the `server` submodule, outside the ros2-cup-stack fix scope, but should be
fixed so a rejected movel is not reported as a completed move.

## Constraint on the fix

The early "placed" response is **intentional** — it lets the LLM start reasoning
about the next step while the cup is being released and lifted. The fix must
keep that overlap; it must not make `/skill/pyramid_step` fully synchronous.

## Proposed fix (ros2-cup-stack scope)

Keep the early response. After the **background lift completes**, deterministically
return the robot to `STATE_STANDBY` before clearing `busy`, so the LLM's next
native `movel` is accepted. Candidate mechanisms:

1. **`system/servo_off`** after the lift (`dsr_msgs2/srv/ServoOff`) — forces
   STANDBY immediately. Next skill's `servoj_rt` re-enables servo automatically.
   Trade-off: servo on/off cycles each cup (possible click/latency).
2. **`system/set_robot_mode`** to a standby/autonomous mode.
3. **Poll `system/get_robot_mode` until STANDBY** (with timeout) — no servo power
   change, just waits out the ~1.4 s natural transition before clearing `busy`.

`skill_api_node` can call these on `_runtime.node` (`call_async` + await the
future; the node is spun on the main thread).

### Residual edge case
If the next `movel` arrives **during the lift itself** (LLM decides faster than
the lift runs), it is still in `JOINT_SERVO` and will be rejected. Fully closing
this requires gating/retrying the movel until STANDBY, which touches
`dsr_controller2` (submodule) or the `server` `move_to` path — out of the
ros2-cup-stack scope. In practice LLM latency exceeds the lift, so option 1/3
above is expected to resolve the observed failures.

## Status

Diagnosis confirmed via live repro on the real M0609. Fix mechanism (servo_off vs
set_robot_mode vs poll-STANDBY) pending decision before implementation.
