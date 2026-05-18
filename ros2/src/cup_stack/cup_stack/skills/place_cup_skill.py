"""Skill: pick one cup and place it at a single pyramid slot.

Standalone — no import of the existing task/runtime/config code.
The motion choreography mirrors the reference ``CupPyramidTask``
cycle so behaviour is identical.
"""

from cup_stack.skills.base import PickSpec, RobotIO, Skill
from cup_stack.skills.config import DOWN_ORI, SkillStackConfig
from cup_stack.skills.geometry import make_twist_orientation
from cup_stack.skills.pyramid_slot import PyramidSlot


class PlaceCupSkill(Skill):
    """Place exactly one cup at its pyramid slot.

    The destination is fully determined by the pyramid centre and the
    slot, so the skill resolves its own place pose.  The start (pick)
    coordinate is the cup-middle XY of the source stack and is supplied
    by the controller node through :class:`PickSpec`.
    """

    def __init__(
        self,
        robot: RobotIO,
        slot: PyramidSlot,
        center_xy: tuple[float, float],
        config: SkillStackConfig | None = None,
    ) -> None:
        self.robot = robot
        self.slot = slot
        self.config = config or SkillStackConfig()
        self.logger = robot.logger
        self.name = slot.name
        self.center_x, self.center_y = center_xy

    # ---- destination, derived from the pyramid centre ----------------

    @property
    def place_x(self) -> float:
        """Absolute place X; gets the row offset only on the X axis."""

        if self.config.spread_axis == "x":
            return self.center_x + self.slot.lateral_offset
        return self.center_x

    @property
    def place_y(self) -> float:
        """Absolute place Y; gets the row offset only on the Y axis."""

        if self.config.spread_axis == "x":
            return self.center_y
        return self.center_y + self.slot.lateral_offset

    @property
    def place_z(self) -> float:
        """Absolute place Z for this slot's vertical layer."""

        return (
            self.config.place_z_base
            + self.slot.layer * self.config.layer_height
        )

    def describe(self) -> str:
        """Return a one-line human summary for plan logging."""

        return (
            f"{self.name}: tier {self.slot.tier} "
            f"place ({self.place_x:.3f},{self.place_y:.3f}) "
            f"z={self.place_z:.3f} (L{self.slot.layer})"
        )

    # ---- execution ---------------------------------------------------

    def execute(self, pick: PickSpec | None = None) -> bool:
        """Run the full pick -> travel -> place cycle for this cup."""

        if pick is None:
            self.logger.error(
                f"SKILL {self.name}: a PickSpec is required"
            )
            return False
        place_x, place_y, place_z = self.place_x, self.place_y, self.place_z
        pick_ori = pick.ori or DOWN_ORI

        self.logger.info("=" * 50)
        self.logger.info(
            f"SKILL {self.name}  "
            f"pick ({pick.x:.3f},{pick.y:.3f}) z={pick.z:.3f} -> "
            f"place ({place_x:.3f},{place_y:.3f}) "
            f"z={place_z:.3f} L{self.slot.layer}"
        )
        self.logger.info("=" * 50)

        if not self._pick(pick, pick_ori):
            return False
        if not self._travel(place_x, place_y):
            return False
        return self._place(place_x, place_y, place_z)

    def _pick(self, pick: PickSpec, pick_ori: dict[str, float]) -> bool:
        cfg = self.config
        self.logger.info("  [1] pick XY move @ PICK_SAFE_Z")
        if not self._step(
            self.robot.try_move_to_pose(
                pick.x, pick.y, cfg.pick_safe_z, cfg.safe_z_min,
                ori=pick_ori,
            ),
            1,
        ):
            return False

        self.logger.info("  [2] gripper OPEN")
        self.robot.try_open_gripper(cfg.open_sleep_sec)

        self.logger.info(f"  [3] pick descend -> z={pick.z:.3f}")
        if not self._step(
            self.robot.try_move_to_pose(
                pick.x, pick.y, pick.z, cfg.safe_z_min,
                ori=pick_ori, lin=True,
            ),
            3,
        ):
            return False

        self.logger.info("  [4] GRIP")
        if not self.robot.try_grip_cup(cfg.grip_sleep_sec):
            return False

        self.logger.info("  [5] lift -> PICK_SAFE_Z")
        return self._step(
            self.robot.try_move_to_pose(
                pick.x, pick.y, cfg.pick_safe_z, cfg.safe_z_min,
                ori=pick_ori, lin=True,
            ),
            5,
        )

    def _travel(self, place_x: float, place_y: float) -> bool:
        cfg = self.config
        # Keep PICK_SAFE_Z during the diagonal move (no extra lift).
        travel_z = cfg.pick_safe_z
        self.logger.info(
            f"  [6] target XY move ({place_x:.3f},{place_y:.3f}) "
            f"@ z={travel_z:.3f}"
        )
        return self._step(
            self.robot.try_move_to_pose(
                place_x, place_y, travel_z, cfg.safe_z_min,
            ),
            6,
        )

    def _place(
        self, place_x: float, place_y: float, place_z: float
    ) -> bool:
        cfg = self.config
        approach_z = cfg.pick_safe_z
        if approach_z > place_z:
            # Descending onto a lower layer (L0/L1).
            mid_z = place_z + (approach_z - place_z) / 2.0
        else:
            # Rising onto the top cap (L2): just above place_z.
            mid_z = place_z + 0.02

        half_twist = make_twist_orientation(cfg.place_twist_deg / 2.0)
        full_twist = make_twist_orientation(cfg.place_twist_deg)

        self.logger.info(f"  [7a] place mid -> z={mid_z:.3f}")
        if not self._step(
            self.robot.try_move_to_pose(
                place_x, place_y, mid_z, cfg.safe_z_min,
                ori=half_twist,
            ),
            "7a",
        ):
            return False

        self.logger.info(f"  [7b] place final -> z={place_z:.3f}")
        if not self._step(
            self.robot.try_move_to_pose(
                place_x, place_y, place_z, cfg.safe_z_min,
                ori=full_twist,
            ),
            "7b",
        ):
            return False

        self.logger.info("  [8] RELEASE")
        self.robot.try_release_cup(cfg.release_sleep_sec)

        lift_z = max(cfg.pick_safe_z, place_z + 0.02)
        self.logger.info(f"  [9] lift -> z={lift_z:.3f}")
        return self._step(
            self.robot.try_move_to_pose(
                place_x, place_y, lift_z, cfg.safe_z_min,
                ori=full_twist,
            ),
            9,
        )

    def _step(self, ok: bool, step: int | str) -> bool:
        if ok:
            return True
        self.logger.error(
            f"=== SKILL {self.name} STEP {step} failed; aborting ==="
        )
        return False
