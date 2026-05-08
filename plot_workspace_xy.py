"""
M0609 XY 작업공간 시각화 (IK 서비스 기반)
DOWN_ORI = [x=0, y=1, z=0, w=0] 기준, 고정 Z 높이별로 도달 가능 영역 확인
"""

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped

# ── 설정 ────────────────────────────────────────────────────────────
GROUP_NAME   = "manipulator"
BASE_FRAME   = "base_link"
DOWN_ORI     = (0.0, 1.0, 0.0, 0.0)   # x, y, z, w

X_RANGE = np.arange(0.30, 0.90, 0.025)
Y_RANGE = np.arange(-0.50, 0.30, 0.025)

Z_LEVELS = {
    "safe_z (0.55)":       0.55,
    "pick_safe_z (0.45)":  0.45,
    "pick_z_base (0.308)": 0.308,
}

# 현재 작업 포인트 (로그에서 확인)
HOME_XY     = (0.609, -0.104)
PICK_XY     = (0.613, -0.092)
FAIL_XY     = (0.609, -0.183)
# ────────────────────────────────────────────────────────────────────


def check_ik(node: Node, cli, x: float, y: float, z: float) -> bool:
    req = GetPositionIK.Request()
    ik = PositionIKRequest()
    ik.group_name = GROUP_NAME
    ik.avoid_collisions = False
    ps = PoseStamped()
    ps.header.frame_id = BASE_FRAME
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    ps.pose.orientation.x, ps.pose.orientation.y, \
        ps.pose.orientation.z, ps.pose.orientation.w = DOWN_ORI
    ik.pose_stamped = ps
    req.ik_request = ik

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
    if future.result() is None:
        return False
    return future.result().error_code.val == 1   # SUCCESS = 1


def main():
    rclpy.init()
    node = Node("workspace_sampler")
    cli = node.create_client(GetPositionIK, "/compute_ik")

    node.get_logger().info("IK 서비스 대기 중...")
    if not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("/compute_ik 서비스 없음 — bringup 실행 중인지 확인")
        rclpy.shutdown()
        sys.exit(1)
    node.get_logger().info("IK 서비스 연결됨")

    ncols = len(Z_LEVELS)
    fig, axes = plt.subplots(1, ncols, figsize=(6 * ncols, 6), squeeze=False)
    fig.suptitle("M0609 XY 도달 가능 영역  (DOWN_ORI, base_link 기준)", fontsize=14)

    for col, (label, z) in enumerate(Z_LEVELS.items()):
        ax = axes[0][col]
        reachable = np.zeros((len(Y_RANGE), len(X_RANGE)), dtype=bool)

        total = len(X_RANGE) * len(Y_RANGE)
        done = 0
        for yi, y in enumerate(Y_RANGE):
            for xi, x in enumerate(X_RANGE):
                reachable[yi, xi] = check_ik(node, cli, x, y, z)
                done += 1
            pct = done / total * 100
            node.get_logger().info(f"[{label}] {pct:.0f}%  ({done}/{total})")

        # 격자 그리기
        ax.imshow(
            reachable,
            origin="lower",
            extent=[X_RANGE[0], X_RANGE[-1], Y_RANGE[0], Y_RANGE[-1]],
            aspect="equal",
            cmap="RdYlGn",
            vmin=0, vmax=1,
            alpha=0.7,
        )

        # 포인트 표시
        ax.plot(*HOME_XY, "b*", markersize=12, label=f"HOME {HOME_XY}")
        ax.plot(*PICK_XY, "g^", markersize=10, label=f"PICK {PICK_XY}")
        ax.plot(*FAIL_XY, "rx", markersize=14, markeredgewidth=3, label=f"FAIL {FAIL_XY}")

        # 피라미드 배치 y 오프셋 표시 (x=HOME_X 기준)
        spacing = 0.079
        for y_off, layer in [(-spacing, 0), (0.0, 0), (spacing, 0),
                              (-spacing/2, 1), (spacing/2, 1), (0.0, 2)]:
            py = HOME_XY[1] + y_off
            ax.axhline(py, color="orange", linewidth=0.6, linestyle="--", alpha=0.6)

        ax.set_title(f"z = {label}", fontsize=11)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.legend(fontsize=7, loc="upper left")
        ax.grid(True, linewidth=0.3, alpha=0.4)
        ax.set_xlim(X_RANGE[0], X_RANGE[-1])
        ax.set_ylim(Y_RANGE[0], Y_RANGE[-1])

    plt.tight_layout()
    out = "/home/ssu/development/cup-stack/ros2-cup-stack/workspace_xy.png"
    plt.savefig(out, dpi=150)
    node.get_logger().info(f"저장됨: {out}")
    print(f"\n[결과] {out}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
