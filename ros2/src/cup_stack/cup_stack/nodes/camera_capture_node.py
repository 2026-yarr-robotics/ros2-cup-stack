"""ROS 2 node: show live camera feed and capture frames with a keypress."""

import datetime
import os
import subprocess
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from cup_stack.config import CameraConfig

WINDOW = "Camera Capture  [S] save  [ESC] exit"


def main(args=None):
    rclpy.init(args=args)
    node = Node("camera_capture_node")
    node.declare_parameter("save_dir", os.path.expanduser("~/captures"))

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    config = CameraConfig()
    bridge = CvBridge()
    frame = {"color": None}

    def color_cb(msg: Image) -> None:
        frame["color"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    node.create_subscription(Image, config.color_topic, color_cb, 10)

    save_dir = os.path.expanduser(str(node.get_parameter("save_dir").value))
    os.makedirs(save_dir, exist_ok=True)
    node.get_logger().info(f"Saving captures to: {save_dir}")
    node.get_logger().info("Press [S] to capture, [ESC] to exit.")

    cv2.namedWindow(WINDOW)
    count = 0

    try:
        while rclpy.ok():
            img = frame["color"]
            if img is None:
                blank = __import__("numpy").zeros((480, 640, 3), dtype="uint8")
                cv2.putText(blank, "Waiting for camera...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow(WINDOW, blank)
            else:
                cv2.imshow(WINDOW, img)

            key = cv2.waitKey(20) & 0xFF
            if key == 27:  # ESC
                break
            if key in (ord("s"), ord("S")):
                if frame["color"] is not None:
                    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    path = os.path.join(save_dir, f"capture_{ts}_{count:03d}.png")
                    cv2.imwrite(path, frame["color"])
                    count += 1
                    node.get_logger().info(f"Saved: {path}")
                    subprocess.Popen(
                        ["paplay",
                         "/usr/share/sounds/freedesktop/stereo/camera-shutter.oga"],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
    finally:
        cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
