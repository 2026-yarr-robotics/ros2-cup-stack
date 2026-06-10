"""Skill: pick one cup and place it at an externally provided XYZ.

Unlike :class:`cup_stack.skills.place_cup_skill.PlaceCupSkill`, this
skill does not derive its destination from a ``PyramidSlot`` /
``center_xy`` / ``spread_axis``: the place pose is supplied directly,
so the caller (server) owns slot geometry and yaw.  Motion choreography
mirrors ``PlaceCupSkill`` so behaviour stays identical.
"""

from dataclasses import dataclass
from typing import Callable

from cup_stack.skills.base import PickSpec, RobotIO, Skill
from cup_stack.skills.config import DOWN_ORI, SkillStackConfig
from cup_stack.skills.geometry import make_twist_orientation


@dataclass(frozen=True)
class PlaceSpec:
    """Absolute place pose (base_link, m). ``name`` is a log/debug tag."""

    x: float
    y: float
    z: float
    name: str = ""


class PlaceCupAtSkill(Skill):
    """Pick → travel → place a single cup at an absolute XYZ.

    The pick pose (and its ``ori``) come from ``PickSpec``; the place
    pose comes from ``PlaceSpec`` passed at construction.  No
    dependency on the pyramid slot table or yaw axis — the server
    decides those externally and hands over the resolved place XYZ.
    """

    def __init__(
        self,
        robot: RobotIO,
        place: PlaceSpec,
        config: SkillStackConfig | None = None,
    ) -> None:
        self.robot = robot
        self.place = place
        self.config = config or SkillStackConfig()
        self.logger = robot.logger
        self.name = place.name or "place_cup_at"

    def describe(self) -> str:
        return (
            f"{self.name}: place "
            f"({self.place.x:.3f},{self.place.y:.3f},{self.place.z:.3f})"
        )

    def execute(
        self,
        pick: PickSpec | None = None,
        on_placed: Callable[[], None] | None = None,
    ) -> bool:
        """Run the full pick → travel → place cycle for one cup.

        Mirrors :meth:`PlaceCupSkill.execute` step-for-step so motion
        choreography (orientations, linear/joint moves, sleep timings)
        stays identical — only the place pose source differs.

        ``on_placed`` is invoked right after the cup is released at its
        place pose (step 8), before the final lift.  Callers can use it
        to report completion at place time while the lift finishes.
        """
        if pick is None:
            self.logger.error(f"SKILL {self.name}: a PickSpec is required")
            return False

        cfg = self.config
        r = self.robot
        log = self.logger
        log.info(self.describe())

        pick_ori = pick.ori or DOWN_ORI
        half_twist = make_twist_orientation(cfg.place_twist_deg / 2.0)
        full_twist = make_twist_orientation(cfg.place_twist_deg)

        # ── Pick ─────────────────────────────────────────────────────
        # Lateral approach to the pick must clear any cups standing at the pick
        # site. For high picks (unstacking a pyramid's upper tiers — e.g. 3m at
        # z≈0.504, above pick_safe_z) a fixed pick_safe_z drags the open gripper
        # through the standing cups; approach one cup-body above the pick (mirrors
        # the place-travel lift), capped at travel_z_max, then descend straight down.
        pick_approach_z = max(cfg.pick_safe_z, pick.z + cfg.layer_height + 0.03)
        pick_approach_z = min(pick_approach_z, cfg.travel_z_max)
        log.info(f"  [1] pick XY move @ z={pick_approach_z:.3f}")
        if not r.try_move_to_pose(
            pick.x, pick.y, pick_approach_z, cfg.safe_z_min, ori=pick_ori,
        ):
            return False
        log.info("  [2] gripper OPEN")
        r.try_open_gripper(cfg.open_sleep_sec)
        log.info(f"  [3] pick descend -> z={pick.z:.3f}")
        if not r.try_move_to_pose(
            pick.x, pick.y, pick.z, cfg.safe_z_min, ori=pick_ori, lin=True,
            slow=pick.z >= cfg.singular_z,
        ):
            return False
        log.info("  [4] GRIP")
        if not r.try_grip_cup(cfg.grip_sleep_sec):
            return False
        log.info(f"  [5] lift -> z={pick_approach_z:.3f}")
        if not r.try_move_to_pose(
            pick.x, pick.y, pick_approach_z, cfg.safe_z_min,
            ori=pick_ori, lin=True, slow=pick_approach_z >= cfg.singular_z,
        ):
            return False

        # ── Travel ───────────────────────────────────────────────────
        # For upper-layer slots (2l/2r/3m) the place height is at or above
        # pick_safe_z, so a lateral move at pick_safe_z drags the held cup
        # straight through cups already placed on the layer below. Lift
        # vertically (LIN) one full layer above place.z first (cup body
        # ≈ layer_height), then travel laterally, then descend.
        #
        # Capped at travel_z_max (just below the 0.55 workspace safe zone):
        # uncapped, the top slot (3m, place_z=0.513) asked for 0.638, where
        # the M0609 with a down-facing EE is at the edge of its reach and
        # placement accuracy degrades. The cap still clears the tier-2 cup
        # tops (= 3m's place_z) by travel_z_max - place_z ≈ 32 mm.
        travel_z = max(cfg.pick_safe_z, self.place.z + cfg.layer_height + 0.03)
        travel_z = min(travel_z, cfg.travel_z_max)
        # Never travel below the support layer the cup will be placed onto.
        travel_z = max(travel_z, self.place.z + cfg.travel_clearance)
        if travel_z > cfg.pick_safe_z:
            log.info(f"  [5b] extra lift -> z={travel_z:.3f}")
            if not r.try_move_to_pose(
                pick.x, pick.y, travel_z, cfg.safe_z_min,
                ori=pick_ori, lin=True, slow=travel_z >= cfg.singular_z,
            ):
                return False
        log.info(
            f"  [6] target XY move ({self.place.x:.3f},{self.place.y:.3f}) "
            f"@ z={travel_z:.3f}"
        )
        if not r.try_move_to_pose(
            self.place.x, self.place.y, travel_z, cfg.safe_z_min,
        ):
            return False

        # ── Place ────────────────────────────────────────────────────
        approach_z = travel_z
        if approach_z > self.place.z:
            mid_z = self.place.z + (approach_z - self.place.z) / 2.0
        else:
            mid_z = self.place.z + 0.02

        log.info(f"  [7a] place mid -> z={mid_z:.3f}")
        if not r.try_move_to_pose(
            self.place.x, self.place.y, mid_z, cfg.safe_z_min,
            ori=half_twist,
        ):
            return False
        log.info(f"  [7b] place final -> z={self.place.z:.3f}")
        if not r.try_move_to_pose(
            self.place.x, self.place.y, self.place.z, cfg.safe_z_min,
            ori=full_twist,
        ):
            return False
        log.info("  [8] RELEASE")
        r.try_release_cup(cfg.release_sleep_sec)
        if on_placed is not None:
            try:
                on_placed()
            except Exception as exc:  # pragma: no cover - defensive
                log.warn(f"on_placed callback failed: {exc}")

        lift_z = max(cfg.pick_safe_z, self.place.z + 0.02)
        log.info(f"  [9] lift -> z={lift_z:.3f}")
        if not r.try_move_to_pose(
            self.place.x, self.place.y, lift_z, cfg.safe_z_min,
            ori=full_twist,
        ):
            return False
        return True
