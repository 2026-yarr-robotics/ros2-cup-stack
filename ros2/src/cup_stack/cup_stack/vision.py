"""Camera click coordinate selection for cup stacking tasks."""

from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CameraInfo, Image

from .config import CameraConfig


class CameraClickSelector:
    """Select a base-frame point from aligned color/depth camera images."""

    def __init__(self, node, runtime, config: CameraConfig | None = None) -> None:
        self.node = node
        self.runtime = runtime
        self.config = config or CameraConfig()
        self.logger = node.get_logger()
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.selected_xyz = None
        self._window = None

        calib_path = (
            Path(get_package_share_directory("cup_stack"))
            / "config"
            / self.config.handeye_file
        )
        self.gripper_to_camera = np.load(str(calib_path)).astype(float)
        self.gripper_to_camera[:3, 3] /= 1000.0
        self.logger.info(f"Loaded hand-eye calibration: {calib_path}")

        node.create_subscription(
            CameraInfo,
            self.config.camera_info_topic,
            self._camera_info_cb,
            10,
        )
        node.create_subscription(Image, self.config.color_topic, self._color_cb, 10)
        node.create_subscription(Image, self.config.depth_topic, self._depth_cb, 10)

    @property
    def ready(self) -> bool:
        return (
            self.color_image is not None
            and self.depth_image is not None
            and self.intrinsics is not None
        )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "ppx": msg.k[2],
            "ppy": msg.k[5],
        }

    def _color_cb(self, msg: Image) -> None:
        self.color_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8",
        )

    def _depth_cb(self, msg: Image) -> None:
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough",
        )

    def find_valid_depth(self, cx: int, cy: int) -> tuple[int, int, int] | None:
        """Find a nearby non-zero depth sample, preferring nearer surfaces."""

        h, w = self.depth_image.shape[:2]
        if not (0 <= cx < w and 0 <= cy < h):
            return None

        center_z = self.depth_image[cy, cx]
        if center_z > 0:
            return cx, cy, int(center_z)

        radius = self.config.depth_search_radius_px
        x0, x1 = max(0, cx - radius), min(w, cx + radius + 1)
        y0, y1 = max(0, cy - radius), min(h, cy + radius + 1)
        patch = self.depth_image[y0:y1, x0:x1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None

        target_z = int(np.percentile(valid, 25))
        band = (patch >= target_z - 20) & (patch <= target_z + 20)
        ys, xs = np.where(band)
        if ys.size == 0:
            ys, xs = np.where(patch > 0)

        abs_xs = xs + x0
        abs_ys = ys + y0
        distances = (abs_xs - cx) ** 2 + (abs_ys - cy) ** 2
        idx = int(np.argmin(distances))
        return int(abs_xs[idx]), int(abs_ys[idx]), int(patch[ys[idx], xs[idx]])

    def pixel_to_base(self, x: int, y: int) -> tuple[float, float, float] | None:
        if not self.ready:
            self.logger.warn("Camera frames or intrinsics are not ready")
            return None

        depth = self.find_valid_depth(x, y)
        if depth is None:
            self.logger.warn("No valid depth near click")
            return None

        use_x, use_y, z_raw = depth
        if (use_x, use_y) != (x, y):
            self.logger.info(
                f"Using nearby depth pixel ({use_x}, {use_y}) z={z_raw}mm"
            )

        z_m = float(z_raw) / 1000.0
        fx = self.intrinsics["fx"]
        fy = self.intrinsics["fy"]
        ppx = self.intrinsics["ppx"]
        ppy = self.intrinsics["ppy"]
        cam_x = (use_x - ppx) * z_m / fx
        cam_y = (use_y - ppy) * z_m / fy
        cam_point = np.array([cam_x, cam_y, z_m, 1.0], dtype=float)

        base_to_camera = self.runtime.current_ee_matrix() @ self.gripper_to_camera
        base_point = base_to_camera @ cam_point
        return (
            float(base_point[0]),
            float(base_point[1]),
            float(base_point[2]),
        )

    def mouse_callback(self, event, x, y, flags, param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        point = self.pixel_to_base(x, y)
        if point is None:
            return
        self.selected_xyz = point
        bx, by, bz = point
        self.logger.info(f"Selected point: ({bx:.3f}, {by:.3f}, {bz:.3f})")

    def select_point(
        self,
        window_name: str = "Cup Stack Coordinate Select",
        prompt: str = "Click nested cup stack. ESC to cancel.",
    ):
        """Show camera frames until a point is selected or ESC is pressed."""

        self._window = window_name
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        try:
            while rclpy.ok(context=self.node.context):
                if self.color_image is None:
                    blank = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(
                        blank,
                        "Waiting for camera...",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )
                    cv2.imshow(window_name, blank)
                else:
                    display = self.color_image.copy()
                    if self.selected_xyz is None:
                        msg = prompt
                        color = (255, 255, 255)
                    else:
                        bx, by, _ = self.selected_xyz
                        msg = f"Selected ({bx:.3f}, {by:.3f}). ENTER to use."
                        color = (0, 255, 0)
                    cv2.putText(
                        display,
                        msg,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        color,
                        2,
                    )
                    cv2.imshow(window_name, display)

                key = cv2.waitKey(20) & 0xFF
                if key == 27:
                    return None
                if key in (10, 13) and self.selected_xyz is not None:
                    return self.selected_xyz
        finally:
            cv2.destroyWindow(window_name)
