#!/bin/bash
# Doosan M0609 MoveIt bringup - real hardware mode with RT scheduling.
#
# Usage: ./bringup_real.sh [ROBOT_IP] [ROBOT_PORT]
#        ./bringup_real.sh 192.168.1.100
#
# Why this exists
# ---------------
# The pick-stage hardware judder ("뚝딱거림") is driven by servo_time jitter
# in dsr_hardware2::write():
#
#     servo_time = real_loop_dt * 20;   // dsr_hw_interface2.cpp
#     Drfl.servoj_rt(pos, vel, acc, servo_time);
#
# real_loop_dt is the measured controller_manager loop period. On a stock
# (non-RT) kernel that period jitters by several ms because ros2_control_node
# gets preempted and the CPU changes frequency, so servo_time swings ~0.16-0.28s
# and the harmonic-drive servo profile wobbles cycle to cycle. The servo_time
# formula lives in the doosan-robot2 submodule and is off-limits, but we can
# stabilize its *input* (real_loop_dt) from here:
#
#   1. pin the CPU governor to performance      -> kills frequency-scaling jitter
#   2. promote ros2_control_node to SCHED_FIFO  -> stops it being preempted
#   3. pin it to dedicated cores                -> keeps the loop off busy cores
#
# This also reduces the dt-window drops in write() (dt outside [0.3,1.5]*period
# returns OK without sending a command), so fewer servoj_rt waypoints are lost.
#
# The governor step (1) needs sudo. The SCHED_FIFO + core-pin steps (2-3) run
# WITHOUT sudo once the host has RT limits — run ./setup_rt.sh once, then
# re-login. Promotion is best-effort, BUT step 2 now VERIFIES it actually took
# (reads back chrt -p) and errors loudly otherwise; set RT_REQUIRED=1 to make a
# missing RT promotion abort the bringup. Silently running on SCHED_OTHER is what
# let the servo loop jitter into a velocity-spike safety stop (red light). For
# the strongest result install a PREEMPT_RT kernel and isolate RT_CPUS via the
# isolcpus= boot arg. See docs/realtime.md.

set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

ROBOT_HOST=${1:-192.168.1.100}
ROBOT_PORT=${2:-12345}
MODEL=${MODEL:-m0609}

# RT knobs (override via env: RT_PRIORITY=90 RT_CPUS=3 ./bringup_real.sh ...)
RT_PRIORITY=${RT_PRIORITY:-80}
RT_CPUS=${RT_CPUS:-2,3}
SET_GOVERNOR=${SET_GOVERNOR:-1}

find_workspace_setup() {
    local dir="$SCRIPT_DIR"
    while [ "$dir" != "/" ]; do
        if [ -f "$dir/install/setup.bash" ]; then
            echo "$dir/install/setup.bash"
            return 0
        fi
        dir=$(dirname "$dir")
    done
    return 1
}

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

WORKSPACE_SETUP=$(find_workspace_setup || true)
if [ -n "$WORKSPACE_SETUP" ]; then
    # shellcheck source=/dev/null
    source "$WORKSPACE_SETUP"
else
    echo "[WARN] workspace install/setup.bash not found. Run colcon build first."
fi

# 1. CPU governor -> performance (removes frequency-scaling dt jitter).
if [ "$SET_GOVERNOR" = "1" ] && command -v cpupower >/dev/null 2>&1; then
    echo "[RT] setting CPU governor to performance"
    sudo cpupower frequency-set -g performance \
        || echo "[RT][WARN] governor set failed (need sudo / cpupower); continuing"
fi

# --- RT helpers ------------------------------------------------------------
# Apply SCHED_FIFO + CPU affinity to $1 WITHOUT ever blocking on a password:
#   1. plain `chrt`/`taskset` succeed on our own process once the user has an
#      RLIMIT_RTPRIO grant (run ./setup_rt.sh once + re-login); no sudo needed.
#   2. `sudo -n` is a non-interactive fallback, so a backgrounded promote_rt
#      never hangs on a sudo prompt — the old `sudo chrt` did, and silently
#      no-op'd, leaving the servo loop on SCHED_OTHER (the red-light root cause).
rt_apply() {
    local pid="$1"
    # -a = ALL threads. ros2_control_node runs its servo/read loop in a WORKER
    # thread, not the main thread, so `chrt -p` (no -a) promotes only the main
    # thread and leaves the loop on SCHED_OTHER — jitter persists. Observed:
    # 34/36 threads stayed SCHED_OTHER, joint_state/TF stalled up to 2.5s.
    chrt -a -f -p "$RT_PRIORITY" "$pid" 2>/dev/null \
        || sudo -n chrt -a -f -p "$RT_PRIORITY" "$pid" 2>/dev/null || true
    taskset -acp "$RT_CPUS" "$pid" >/dev/null 2>&1 \
        || sudo -n taskset -acp "$RT_CPUS" "$pid" >/dev/null 2>&1 || true
}

# True only when EVERY thread of $1 is on SCHED_FIFO. Checking just the main
# thread (the old behaviour) reported OK while the servo loop worker stayed on
# SCHED_OTHER — the silent gap that left the loop jittering.
rt_verify() {
    local t tid
    for t in /proc/"$1"/task/*/; do
        tid=$(basename "$t")
        chrt -p "$tid" 2>/dev/null | grep -q 'SCHED_FIFO' || return 1
    done
    return 0
}

# Warn before launch if we cannot get RT at all (the usual silent-no-op setup).
rt_preflight() {
    local soft
    soft=$(ulimit -r 2>/dev/null || echo 0)
    if [ "$soft" != "unlimited" ] && [ "${soft:-0}" -lt "$RT_PRIORITY" ] \
       && [ "$(id -u)" -ne 0 ] && ! sudo -n true 2>/dev/null; then
        echo "[RT][WARN] RLIMIT_RTPRIO=${soft} < ${RT_PRIORITY} and no passwordless sudo:"
        echo "[RT][WARN]   ros2_control_node cannot reach SCHED_FIFO; the servo loop will"
        echo "[RT][WARN]   jitter (risk: velocity-spike safety stop / red light)."
        echo "[RT][WARN]   Fix once:  ${SCRIPT_DIR}/setup_rt.sh   then log out and back in."
    fi
}

# 3 (deferred). Promote the control loop to RT once ros2_control_node is up,
# then VERIFY — a silent failure here is what trips the velocity-spike red light.
promote_rt() {
    local pid="" i pol
    for i in $(seq 1 60); do
        pid=$(pgrep -f ros2_control_node | head -n1 || true)
        if [ -n "$pid" ]; then
            echo "[RT] ros2_control_node pid=$pid -> SCHED_FIFO:${RT_PRIORITY} cpus=${RT_CPUS}"
            rt_apply "$pid"
            if rt_verify "$pid"; then
                echo "[RT] OK: $(chrt -p "$pid" 2>/dev/null | tr '\n' ' ')"
            else
                pol=$(chrt -p "$pid" 2>/dev/null | sed -n 's/.*policy: //p' | head -n1)
                echo "[RT][ERROR] ros2_control_node is NOT on SCHED_FIFO (policy=${pol:-unknown})."
                echo "[RT][ERROR]   Servo loop will jitter -> velocity spikes -> Doosan safety"
                echo "[RT][ERROR]   stop (red light / servo-off). Grant RT limits once:"
                echo "[RT][ERROR]   ${SCRIPT_DIR}/setup_rt.sh   then log out and back in."
                if [ "${RT_REQUIRED:-0}" = "1" ]; then
                    echo "[RT][FATAL] RT_REQUIRED=1: stopping bringup."
                    kill -INT "$LAUNCH_PID" 2>/dev/null || true
                fi
            fi
            return 0
        fi
        sleep 1
    done
    echo "[RT][WARN] ros2_control_node not found after 60s; RT promotion skipped"
}

rt_preflight

echo "[REAL] DSR ${MODEL} MoveIt bringup (mode=real host=${ROBOT_HOST} port=${ROBOT_PORT})"

# 2. Launch in the background so promote_rt can reach the spawned control node,
#    then forward Ctrl-C to the launch and block on it (foreground behaviour).
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
    model:="${MODEL}" \
    mode:=real \
    host:="${ROBOT_HOST}" \
    port:="${ROBOT_PORT}" &
LAUNCH_PID=$!

trap 'kill -INT "$LAUNCH_PID" 2>/dev/null' INT TERM

promote_rt &

wait "$LAUNCH_PID"
