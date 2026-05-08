"""현재 end-effector XYZ + 쿼터니언 출력 (TF2 기반)."""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node("print_ee_pose")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    # TF 메시지가 실제로 수신될 때까지 spin
    import time
    deadline = time.time() + 5.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if tf_buffer.can_transform("base_link", "link_6", rclpy.time.Time()):
            break
    else:
        node.get_logger().error("TF 수신 timeout — bringup이 실행 중인지 확인하세요.")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        t = tf_buffer.lookup_transform("base_link", "link_6", rclpy.time.Time())
        p = t.transform.translation
        q = t.transform.rotation
        print(f"\n=== EE Pose (base_link → link_6) ===")
        print(f"  x = {p.x:.6f} m")
        print(f"  y = {p.y:.6f} m")
        print(f"  z = {p.z:.6f} m")
        print(f"  qx={q.x:.4f}  qy={q.y:.4f}  qz={q.z:.4f}  qw={q.w:.4f}")
        print(f"=====================================\n")
    except Exception as e:
        node.get_logger().error(f"TF lookup 실패: {e}")
        node.get_logger().error("bringup이 실행 중인지 확인하세요.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
