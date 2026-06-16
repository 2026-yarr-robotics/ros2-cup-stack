# Real-time scheduling for the Doosan servo loop

## Symptom
Mid-motion the arm trips a **safety stop — red light / servo-off** with a sudden
joint **velocity spike** (Doosan alarm **1908 / 7051**). CPU load is moderate at
the time (e.g. 10/14 cores), so this is **not** raw CPU saturation.

## Root cause
`ros2_control_node` runs under **SCHED_OTHER** (normal time-sharing), so it gets
preempted and the `controller_manager` loop period `real_loop_dt` jitters wildly
(observed ~1 µs … ~100 ms). `dsr_hardware2::write()` computes

```
servo_time = real_loop_dt * 20;   // dsr_hw_interface2.cpp
Drfl.servoj_rt(pos, vel, acc, servo_time);
```

so a `real_loop_dt` spike produces a bad `servo_time` → commanded joint velocity
spike → Doosan safety-stops the arm. This is a **real-time scheduling** problem,
not a planning or CPU problem. Keeping the control loop on **SCHED_FIFO** holds
the period ~constant and removes the spike.

> Related but separate: `fix(motion): re-time plans that exceed joint velocity
> limits` guards the *planned* trajectory. It is complementary to — not a
> substitute for — RT scheduling, which stabilizes the *control-loop* timing.

## Why it was silently broken
`bringup_real.sh` tried `sudo chrt -f -p 80 <pid>` from a backgrounded
`promote_rt`. Two failures stacked:

1. **No RT limit on the host** — `ulimit -r` (RLIMIT_RTPRIO) = 0, and
   `/etc/security/limits.conf` had no active `rtprio` line.
2. **`sudo` needs a password** — a backgrounded call can't answer the prompt, so
   `chrt` failed.

Both were swallowed (`|| echo ... continuing`), so the loop kept running on
SCHED_OTHER unnoticed — the actual cause of the red-light velocity spikes.

## Fix (one-time, on the robot host)
```bash
cd ros2/src/cup_stack
./setup_rt.sh            # grants rtprio 95 + memlock unlimited to your user
                         # writes /etc/security/limits.d/99-ros2-rt.conf (sudo once)
# log out and back in (or reboot) so PAM applies the new limit
ulimit -r                # expect >= 95
```

After the grant, `bringup_real.sh` promotes `ros2_control_node` to
**SCHED_FIFO:80 without sudo** (a process may set RT priority on itself up to its
RLIMIT_RTPRIO), pins it to `RT_CPUS`, and **verifies** the policy actually
changed — printing `[RT] OK: ... SCHED_FIFO ...`, or a loud `[RT][ERROR]` (and,
with `RT_REQUIRED=1`, aborting) if it is still SCHED_OTHER.

## Verify
```bash
chrt -p "$(pgrep -f ros2_control_node)"          # policy: SCHED_FIFO, priority: 80
ps -eLo cls,rtprio,comm | grep ros2_control       # cls FF, rtprio 80
```

## Knobs (`bringup_real.sh`)
| env | default | meaning |
|---|---|---|
| `RT_PRIORITY` | `80` | SCHED_FIFO priority for `ros2_control_node` |
| `RT_CPUS` | `2,3` | CPUs to pin the control loop to |
| `SET_GOVERNOR` | `1` | set CPU governor to `performance` (needs sudo) |
| `RT_REQUIRED` | `0` | if `1`, abort bringup when RT promotion fails |

## Strongest result
Install a **PREEMPT_RT** kernel and isolate `RT_CPUS` from the scheduler via the
`isolcpus=` boot arg, so nothing else runs on the control-loop cores.
