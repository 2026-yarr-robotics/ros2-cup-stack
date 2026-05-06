"""Configuration values shared by cup stacking tasks."""

from dataclasses import dataclass, field
import math


_S = math.sqrt(0.5)

DOWN_ORI = {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0}
PICK_ORI = {"x": _S, "y": _S, "z": 0.0, "w": 0.0}


@dataclass(frozen=True)
class MotionConfig:
    """MoveIt planning and robot frame configuration."""

    group_name: str = "manipulator"
    base_frame: str = "base_link"
    ee_link: str = "link_6"
    home_joints_deg: tuple[float, ...] = (
        -13.2044,
        47.7017,
        13.3773,
        0.0267,
        118.9093,
        -13.2030,
    )

    @property
    def home_joints_rad(self) -> list[float]:
        return [math.radians(deg) for deg in self.home_joints_deg]


@dataclass(frozen=True)
class GripperConfig:
    """OnRobot RG gripper configuration."""

    name: str = "rg2"
    toolcharger_ip: str = "192.168.1.1"
    toolcharger_port: int = 502
    open_width: int = 750
    grip_width: int = 450
    force: int = 120
    open_sleep_sec: float = 0.8
    grip_sleep_sec: float = 1.0


@dataclass(frozen=True)
class CameraConfig:
    """Camera topics and hand-eye calibration config."""

    camera_info_topic: str = "/camera/camera/color/camera_info"
    color_topic: str = "/camera/camera/color/image_raw"
    depth_topic: str = "/camera/camera/aligned_depth_to_color/image_raw"
    handeye_file: str = "T_gripper2camera.npy"
    depth_search_radius_px: int = 30


@dataclass(frozen=True)
class CupStackConfig:
    """Geometry and timing values for six-cup stacking."""

    total_cups: int = 6
    safe_z: float = 0.55
    pick_safe_z: float = 0.45
    safe_z_min: float = 0.25
    pick_z_base: float = 0.308
    place_z_base: float = 0.308
    place_x_offset: float = 0.10
    cup_spacing: float = 0.079
    layer_height: float = 0.095
    place_twist_deg: float = 10.0
    open_sleep_sec: float = 0.8
    grip_sleep_sec: float = 1.5
    release_sleep_sec: float = 1.0
    home_sleep_sec: float = 0.5
    pyramid_places: tuple[tuple[float, int], ...] = field(init=False)
    reverse_picks: tuple[tuple[float, int], ...] = field(init=False)

    def __post_init__(self) -> None:
        spacing = self.cup_spacing
        object.__setattr__(
            self,
            "pyramid_places",
            (
                (-spacing, 0),
                (0.0, 0),
                (spacing, 0),
                (-spacing / 2.0, 1),
                (spacing / 2.0, 1),
                (0.0, 2),
            ),
        )
        object.__setattr__(
            self,
            "reverse_picks",
            (
                (0.0, 2),
                (-spacing / 2.0, 1),
                (spacing / 2.0, 1),
                (-spacing, 0),
                (0.0, 0),
                (spacing, 0),
            ),
        )
