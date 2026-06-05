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
        -0.7922,
        11.7405,
        72.7817,
        0.0234,
        95.4661,
        -0.8017,
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
         57.4585,   # J1
         10.3361,   # J2
         75.0595,   # J3
        -31.1839,   # J4
        127.2512,   # J5
         44.6653,   # J6
    )

    # pos2: joint-space (PTP) — J1~J6 (단위: degree)
    pos2_joints_deg: tuple[float, ...] = (
        -42.1911,   # J1
         12.8098,   # J2
         66.9024,   # J3
         25.0818,   # J4
        107.6920,   # J5
        -18.8562,   # J6
    )

    # 각 PTP 웨이포인트 도달 후 대기 시간 (초)
    dwell_sec: float = 5.0

    # ── 4방향 사각형 스캔 (scan square) ───────────────────────────────
    # 2방향(pos1/pos2) 스캔과 달리, 카메라를 계속 하향(DOWN_ORI)으로 고정한
    # 채 base_link XY 평면에서 축 정렬 사각형의 네 꼭짓점을 순회한다. Z 는
    # HOME 자세의 EE 높이(런타임 FK)를 그대로 사용하고, 시작/복귀 위치는
    # 2방향 스캔과 동일하게 시작 시점의 joint 자세를 캡처해 마지막에 복귀한다.
    #
    # 수정 방법: 로봇을 사각형 중심 위로 이동시킨 뒤 EE (x, y) 를 echo 해
    #   square_center_x/y 로 교체하고, 한 변 길이를 square_size 로 조정.
    #     ros2 topic echo /ee_pose --once
    square_center_x: float = 0.45   # 사각형 중심 X (base_link, m)
    square_center_y: float = 0.0    # 사각형 중심 Y (base_link, m)
    square_size: float = 0.20       # 사각형 한 변 길이 (m)

    @property
    def pos1_joints_rad(self) -> list[float]:
        return [math.radians(d) for d in self.pos1_joints_deg]

    @property
    def pos2_joints_rad(self) -> list[float]:
        return [math.radians(d) for d in self.pos2_joints_deg]

    @property
    def square_corners_xy(self) -> tuple[tuple[float, float], ...]:
        """사각형 네 꼭짓점 (x, y) — 중심 ± size/2, CCW 순회.

        순서: 좌하 → 우하 → 우상 → 좌상. ScanSquareTask 가 이 순서대로
        LIN 으로 변을 그린 뒤 첫 꼭짓점으로 돌아와 둘레를 닫는다.
        """
        cx, cy = self.square_center_x, self.square_center_y
        h = self.square_size / 2.0
        return (
            (cx - h, cy - h),
            (cx + h, cy - h),
            (cx + h, cy + h),
            (cx - h, cy + h),
        )


@dataclass(frozen=True)
class GripperConfig:
    """OnRobot RG gripper configuration."""

    name: str = "rg2"
    toolcharger_ip: str = "192.168.1.1"
    toolcharger_port: int = 502
    open_width: int = 900
    grip_width: int = 450
    force: int = 120
    open_sleep_sec: float = 0.8
    grip_sleep_sec: float = 1.0


@dataclass(frozen=True)
class CameraConfig:
    """Camera topics and hand-eye calibration config.

    Vision uses the eye-in-hand (gripper-mounted) camera, launched under
    the ``hand`` namespace (RealSense serial 140122076335).
    """

    camera_info_topic: str = "/hand/hand/color/camera_info"
    color_topic: str = "/hand/hand/color/image_raw"
    depth_topic: str = "/hand/hand/aligned_depth_to_color/image_raw"
    handeye_file: str = "T_gripper2camera.npy"
    depth_search_radius_px: int = 30


@dataclass(frozen=True)
class CupStackConfig:
    """Geometry and timing values for six-cup stacking."""

    total_cups: int = 6
    safe_z: float = 0.55
    pick_safe_z: float = 0.55
    safe_z_min: float = 0.25
    pick_z_base: float = 0.313
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
