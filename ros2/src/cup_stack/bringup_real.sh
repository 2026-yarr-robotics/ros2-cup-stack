#!/bin/bash
# Doosan M0609 MoveIt bringup - real robot mode.
# Usage: ./bringup_real.sh [ROBOT_IP] [--teach]
#   --teach / -t  : 직접 교시(MANUAL) 모드로 기동. Ctrl+C 종료 시 자율 모드로 복귀.

set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Argument parsing — IP is the first non-flag arg; --teach/-t can appear anywhere.
ROBOT_IP=${ROBOT_IP:-192.168.1.100}
TEACH_MODE=false
for arg in "$@"; do
    case "$arg" in
        --teach|-t) TEACH_MODE=true ;;
        -*) ;;
        *) ROBOT_IP="$arg" ;;
    esac
done

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

# Kill any existing bringup processes before starting
echo "[REAL] 기존 bringup 프로세스 정리 중..."
pkill -f "dsr_bringup2_moveit\.launch\.py" 2>/dev/null || true
pkill -f "dsr_bringup2_rviz\.launch\.py"   2>/dev/null || true
sleep 2

LAUNCH_ARGS=(
    model:=m0609
    mode:=real
    "host:=${ROBOT_IP}"
    port:=12345
)

if ! $TEACH_MODE; then
    echo "[REAL] DSR M0609 MoveIt bringup (mode=real, host=${ROBOT_IP})"
    exec ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py "${LAUNCH_ARGS[@]}"
fi

# ── 직접 교시 모드 ──────────────────────────────────────────────────────────
echo "[REAL] DSR M0609 MoveIt bringup (mode=real, host=${ROBOT_IP}) [직접 교시]"

ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py "${LAUNCH_ARGS[@]}" &
BRINGUP_PID=$!

cleanup() {
    echo ""
    echo "[TEACH] 자율 모드(AUTONOMOUS)로 복귀 중..."
    ros2 service call /system/set_robot_mode dsr_msgs2/srv/SetRobotMode \
        "{robot_mode: 1}" >/dev/null 2>&1 || true
    kill "$BRINGUP_PID" 2>/dev/null || true
    wait "$BRINGUP_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[TEACH] /system/set_robot_mode 서비스 대기 중..."
until ros2 service list 2>/dev/null | grep -q "/system/set_robot_mode"; do
    sleep 1
    # Exit immediately if the bringup process died unexpectedly.
    kill -0 "$BRINGUP_PID" 2>/dev/null || { echo "[ERROR] bringup 프로세스가 종료됨"; exit 1; }
done

echo "[TEACH] 직접 교시 모드(ROBOT_MODE_MANUAL=0) 활성화..."
ros2 service call /system/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 0}"
echo "[TEACH] 직접 교시 활성화 완료. 로봇을 손으로 이동할 수 있습니다. Ctrl+C 로 종료."

wait "$BRINGUP_PID"
