"""Configuration values shared by cup stacking tasks."""

from dataclasses import dataclass, field
import math


_S = math.sqrt(0.5)

DOWN_ORI = {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0}
PICK_ORI = {"x": _S, "y": _S, "z": 0.0, "w": 0.0}


@dataclass(frozen=True)
class WorkspaceConfig:
    """Safe Cartesian workspace bounds (base_link frame, metres).

    click_pick_two.py 의 SAFE_* 상수와 동일한 값.
    수정 시 이 클래스만 변경하면 모든 플래너에 일괄 적용됨.
    """

    x_min: float = 0.0
    x_max: float = 0.80
    y_min: float = -0.30
    y_max: float =  0.30
    z_min: float =  0.25
    z_max: float =  0.80


@dataclass(frozen=True)
class MotionConfig:
    """MoveIt planning and robot frame configuration."""

    group_name: str = "manipulator"
    base_frame: str = "base_link"
    ee_link: str = "link_6"
    home_joints_deg: tuple[float, ...] = (
        -12.4849,
        24.8886,
        52.5965,
        0.0239,
        102.5033,
        -12.4914,
    )

    @property
    def home_joints_rad(self) -> list[float]:
        return [math.radians(deg) for deg in self.home_joints_deg]


@dataclass(frozen=True)
class ScanConfig:
    """Scan task configuration.

    pos1: 스캔 시작 joint 자세 (PTP). 현재 로봇 위치 기준.
    pos2: pos1에서 LIN으로 이동할 끝점 (x, y). z는 pos1 EE 높이 사용.

    수정 방법
      pos1 — 로봇을 원하는 자세로 이동 후 degree 값 교체:
              ros2 topic echo /joint_states --once
      pos2  — 로봇을 끝점으로 이동 후 EE (x, y) 교체:
              ros2 topic echo /ee_pose --once
    """

    # pos1: joint-space (PTP) — J1~J6 (단위: degree)
    pos1_joints_deg: tuple[float, ...] = (
         28.6133,   # J1
        -13.4030,   # J2
        100.3468,   # J3
         -0.1471,   # J4
         90.9877,   # J5
         25.3427,   # J6
    )

    # pos2: joint-space (PTP) — J1~J6 (단위: degree)
    pos2_joints_deg: tuple[float, ...] = (
         28.6133,   # J1
        -13.4030,   # J2
        100.3468,   # J3
         -0.1471,   # J4
          90.9877,  # J5
         25.3427,   # J6
    )

    @property
    def pos1_joints_rad(self) -> list[float]:
        return [math.radians(d) for d in self.pos1_joints_deg]

    @property
    def pos2_joints_rad(self) -> list[float]:
        return [math.radians(d) for d in self.pos2_joints_deg]


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
    pick_safe_z: float = 0.55
    safe_z_min: float = 0.25
    pick_z_base: float = 0.323
    place_z_base: float = 0.323
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
