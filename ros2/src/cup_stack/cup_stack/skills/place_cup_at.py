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
from cup_stack.skills.config import SkillStackConfig
from cup_stack.skills.geometry import make_twist_orientation, matrix_to_quaternion


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
        grip_twist_deg: float = 0.0,
    ) -> None:
        self.robot = robot
        self.place = place
        self.config = config or SkillStackConfig()
        self.logger = robot.logger
        self.name = place.name or "place_cup_at"
        # Yaw twist (deg) of the gripper-down grip orientation about the
        # vertical. 0.0 = the plain down orientation (DOWN_ORI ==
        # make_twist_orientation(0)). Callers set this to hold the wrist (J6)
        # at the same yaw as the joint HOME pose ([0,0,90,0,90,90], J6=90°),
        # so the wrist does not swing ~90° between HOME and every pick/place.
        # The place's small settle twist (place_twist_deg) is applied on top
        # of this base, so the relative place choreography is unchanged.
        self.grip_twist_deg = float(grip_twist_deg)

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

        # Base every orientation on the grip twist so the wrist holds a single
        # yaw through pick → travel → place (no mid-carry 90° swing). twist=0
        # reproduces the original DOWN_ORI base exactly.
        base = self.grip_twist_deg
        pick_ori = pick.ori or make_twist_orientation(base)
        half_twist = make_twist_orientation(base + cfg.place_twist_deg / 2.0)
        full_twist = make_twist_orientation(base + cfg.place_twist_deg)

        # Command the gripper OPEN up front (non-blocking — it is on a separate
        # Modbus link from the arm) so it opens *while* the arm travels to the
        # pick approach, instead of freezing at the pick top for the full open
        # dwell. The gripper is normally already open from the previous cup's
        # release; this just re-asserts it. A short settle floor after the
        # approach (step [2]) guarantees it is fully open before the descend.
        r.try_open_gripper(0.0)

        # ── Pick ─────────────────────────────────────────────────────
        # Lateral approach to the pick must clear any cups standing at the pick
        # site. For high picks (unstacking a pyramid's upper tiers — e.g. 3m at
        # z≈0.504, above pick_safe_z) a fixed pick_safe_z drags the open gripper
        # through the standing cups; approach one cup-body above the pick (mirrors
        # the place-travel lift), capped at travel_z_max, then descend straight down.
        pick_approach_z = max(cfg.pick_safe_z, pick.z + cfg.layer_height + 0.03)
        pick_approach_z = min(pick_approach_z, cfg.travel_z_max)
        if pick_approach_z >= cfg.singular_z:
            # Up-over approach for high picks (unstacking upper tiers 3m/2l/2r).
            # The arm starts at HOME (z≈0.45) each step; a direct diagonal to the
            # high pick over the standing pyramid would clip the cups. Ascend in
            # place to the clearance height first [0], then traverse to the pick
            # at that constant Z [1]. Straight LIN holds Z; slow profile keeps
            # joint velocity within limits near the singular zone.
            #
            # The wrist (J6) yaw at the start (HOME) differs ~90° from the pick
            # orientation. Do NOT spend that ~90° turn here in [0] as a
            # near-stationary spin (XY fixed, Z barely rising): hold the current
            # incoming wrist yaw through the lift, then fold the reorientation
            # into the [1] XY traverse so J6 turns *while* the arm is moving
            # laterally — no stopped-rotation dead time. (Later unstack cups
            # already arrive near the pick orientation, so [1]'s twist is small;
            # only the first cup, from HOME, carries the full swing.) Both moves
            # stay LIN+slow at the clearance Z, and try_move_to_pose's
            # joint-velocity guard re-times the rotating traverse if it nears the
            # limit — overlapping the turn never exceeds it.
            cur_T = r.current_ee_matrix()
            cur_x, cur_y = float(cur_T[0, 3]), float(cur_T[1, 3])
            cur_z = float(cur_T[2, 3])
            cur_ori = matrix_to_quaternion(cur_T)
            if cur_z < pick_approach_z - 1e-3:
                log.info(f"  [0] ascend in place (hold wrist) -> z={pick_approach_z:.3f}")
                if not r.try_move_to_pose(
                    cur_x, cur_y, pick_approach_z, cfg.safe_z_min,
                    ori=cur_ori, lin=True, slow=True,
                ):
                    log.warn("  [0] ascend-in-place failed; continuing")
            log.info(f"  [1] pick XY traverse + reorient @ z={pick_approach_z:.3f}")
            if not r.try_move_to_pose(
                pick.x, pick.y, pick_approach_z, cfg.safe_z_min,
                ori=pick_ori, lin=True, slow=True,
            ):
                return False
        else:
            # Low picks (bottom tier / source nest): a single free-space PTP
            # approach to above the pick is clear and faster.
            log.info(f"  [1] pick XY move @ z={pick_approach_z:.3f}")
            if not r.try_move_to_pose(
                pick.x, pick.y, pick_approach_z, cfg.safe_z_min, ori=pick_ori,
                fast=True,
            ):
                return False
        log.info("  [2] gripper OPEN settle")
        r.try_open_gripper(cfg.open_settle_sec)  # floor; open already overlapped approach
        log.info(f"  [3] pick descend -> z={pick.z:.3f}")
        if not r.try_move_to_pose(
            pick.x, pick.y, pick.z, cfg.safe_z_min, ori=pick_ori, lin=True,
            slow=pick.z >= cfg.singular_z,
        ):
            return False
        log.info("  [4] GRIP")
        if not r.try_grip_cup(cfg.grip_sleep_sec):
            return False
        # ── Travel height ────────────────────────────────────────────
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

        # [5] lift the picked cup clear of the pick site. [5] and [5b] share
        # the pick XY and form a colinear vertical run, so when the traverse
        # height (travel_z) is at or above the pick-clearance height we lift
        # straight to travel_z in a single move — the stop at pick_approach_z
        # was a redundant zero-velocity boundary on the way up (a needless
        # decel/accel "click"). Only when travel_z sits *below* pick_approach_z
        # is there a genuine up-then-down corner whose intermediate stop must
        # stay (merging it would cut the clearance corner).
        first_lift_z = max(pick_approach_z, travel_z)
        log.info(f"  [5] lift -> z={first_lift_z:.3f}")
        if not r.try_move_to_pose(
            pick.x, pick.y, first_lift_z, cfg.safe_z_min,
            ori=pick_ori, lin=True, slow=first_lift_z >= cfg.singular_z,
            fast=True,
        ):
            return False

        # ── Travel ───────────────────────────────────────────────────
        if travel_z < pick_approach_z and travel_z > cfg.pick_safe_z:
            log.info(f"  [5b] settle -> z={travel_z:.3f}")
            if not r.try_move_to_pose(
                pick.x, pick.y, travel_z, cfg.safe_z_min,
                ori=pick_ori, lin=True, slow=travel_z >= cfg.singular_z,
                fast=True,
            ):
                return False
        log.info(
            f"  [6] target XY move ({self.place.x:.3f},{self.place.y:.3f}) "
            f"@ z={travel_z:.3f}"
        )
        if not r.try_move_to_pose(
            self.place.x, self.place.y, travel_z, cfg.safe_z_min,
            fast=True,
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
            ori=full_twist, fast=True,
        ):
            return False
        return True
