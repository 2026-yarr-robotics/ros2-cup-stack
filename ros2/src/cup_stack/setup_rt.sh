#!/bin/bash
# One-time HOST setup: grant real-time scheduling limits so bringup_real.sh can
# promote ros2_control_node to SCHED_FIFO WITHOUT a per-run sudo password.
#
# Why
# ---
# On a stock kernel ros2_control_node runs SCHED_OTHER and gets preempted, so the
# controller_manager loop period (real_loop_dt) jitters by several ms.
# dsr_hardware2::write() turns that into servo_time = real_loop_dt*20 swings and
# feeds servoj_rt() inconsistent waypoints -> joint velocity spikes -> Doosan
# safety stop (red light / servo-off; alarm 1908 / 7051). Granting RLIMIT_RTPRIO
# lets the servo loop hold SCHED_FIFO so the loop period stays ~constant.
#
# Usage
# -----
#   ./setup_rt.sh                 # grant to the current login user
#   ./setup_rt.sh ssu             # grant to user 'ssu'
#   ./setup_rt.sh @realtime       # grant to group 'realtime' (members must join)
#   RT_PRIORITY_MAX=99 ./setup_rt.sh
#
# Run WITHOUT sudo (the script sudo's only for the file write). The new limit
# applies at the NEXT login (PAM reads limits.conf at session start) — log out
# and back in (or reboot) afterwards.
set -euo pipefail

RT_PRIORITY_MAX=${RT_PRIORITY_MAX:-95}    # ceiling; must be >= bringup RT_PRIORITY (80)

if [ "$(id -u)" -eq 0 ] && [ "$#" -eq 0 ]; then
    echo "[setup_rt][ERROR] Don't run as root without a target — TARGET would be 'root'."
    echo "                  Run without sudo, or pass a user/group:  ./setup_rt.sh <user|@group>"
    exit 1
fi

TARGET="${1:-$(id -un)}"
LIMITS_FILE=/etc/security/limits.d/99-ros2-rt.conf

CONTENT="# Installed by ros2-cup-stack/setup_rt.sh — real-time scheduling for the
# Doosan servo loop (ros2_control_node -> SCHED_FIFO). See bringup_real.sh and
# docs/realtime.md. Lets bringup promote the control loop without per-run sudo.
${TARGET}   -   rtprio    ${RT_PRIORITY_MAX}
${TARGET}   -   memlock   unlimited"

echo "[setup_rt] target=${TARGET}  rtprio<=${RT_PRIORITY_MAX}  memlock=unlimited"
echo "[setup_rt] -> ${LIMITS_FILE}  (writing via sudo)"
echo "------------------------------------------------------------"
printf '%s\n' "$CONTENT"
echo "------------------------------------------------------------"
printf '%s\n' "$CONTENT" | sudo tee "$LIMITS_FILE" >/dev/null
sudo chmod 0644 "$LIMITS_FILE"

case "$TARGET" in
    @*) echo "[setup_rt] NOTE: ensure your user is in group '${TARGET#@}':"
        echo "                  sudo usermod -aG ${TARGET#@} \"\$(id -un)\"" ;;
esac

echo "[setup_rt] installed."
echo "[setup_rt] this shell still shows the old limit: ulimit -r = $(ulimit -r)"
echo "[setup_rt] >>> LOG OUT and back in (or reboot), then verify:"
echo "[setup_rt]       ulimit -r                               # expect >= ${RT_PRIORITY_MAX}"
echo "[setup_rt]     and after starting bringup_real.sh:"
echo "[setup_rt]       chrt -p \$(pgrep -f ros2_control_node)   # expect SCHED_FIFO"
