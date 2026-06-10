"""Self-contained geometry/timing config for the skill subsystem.

Values mirror :mod:`cup_stack.config` (reference code) but this module
intentionally does **not** import it, so the skills package works
standalone and is unit-testable without ROS.
"""

from dataclasses import dataclass, field
import math


_S = math.sqrt(0.5)

# Gripper-down orientation, and the 90 deg-twisted variant used on
# alternate picks.  Copied from the reference config.
DOWN_ORI = {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0}
PICK_ORI = {"x": _S, "y": _S, "z": 0.0, "w": 0.0}


@dataclass(frozen=True)
class SkillStackConfig:
    """Geometry and timing values for the six-cup 3-2-1 pyramid.

    Mirrors the reference ``CupStackConfig`` field-for-field; kept
    separate so the skills package has no dependency on the existing
    task code.

    ``spread_axis`` selects the workbench axis a tier row spreads
    along: ``"y"`` (default, robot left/right) or ``"x"`` (robot
    forward/depth).  Tiers always stack upward on Z regardless.

    ``nested_count`` is how many cups are pre-nested in the source
    stack.  It sets the pick height of the top cup
    (``pick_z_base + (nested_count - 1) * nest_inc``); each pick then
    drops one ``nest_inc``.  Independent of ``total_cups`` (the number
    of cups actually placed into the pyramid).

    ``cup_grip_z_offset`` is the vertical distance from the cup's
    top-centre to the point the gripper actually grips.  Used to
    convert an externally supplied cup-top Z into an actual pick Z:
    ``pick_z = cup_top_z + cup_grip_z_offset``.  Calibrate this
    value against the physical cup geometry.
    """

    total_cups: int = 6
    nested_count: int = 6
    spread_axis: str = "y"
    pick_safe_z: float = 0.45
    safe_z_min: float = 0.25
    # Ceiling for the lateral travel altitude while holding a cup.  Kept
    # just below the 0.55 workspace safe zone (server WorkspaceLimits
    # z_max): above it the M0609 with a down-facing EE is at the edge of
    # its reach, so Pilz planning degrades and place accuracy suffers.
    # 0.545 still clears the tier-2 cup tops (gripper-equivalent 0.513)
    # by ~32 mm, so the held top cup cannot knock them over.
    travel_z_max: float = 0.545
    # Minimum clearance kept between travel_z and the slot's place_z so
    # the held cup never drags across the layer it will be placed onto.
    travel_clearance: float = 0.03
    # Above this gripper Z the M0609 (down-facing EE) is near full vertical
    # reach — close to a wrist/elbow singularity where a Cartesian LIN move
    # makes joint velocity spike past the 225 deg/s joint limit (controller
    # alarm 1908). LIN moves at/above this Z use the reduced-velocity profile
    # (runtime lin_slow_params) so joint speed stays within limits.
    singular_z: float = 0.50
    pick_z_base: float = 0.313
    cup_grip_z_offset: float = 0.10
    place_z_base: float = 0.318
    place_x_offset: float = 0.10
    cup_spacing: float = 0.078
    layer_height: float = 0.093
    place_twist_deg: float = 10.0
    open_sleep_sec: float = 0.8
    grip_sleep_sec: float = 1.5
    release_sleep_sec: float = 1.0
    home_sleep_sec: float = 0.5
    pyramid_places: tuple[tuple[float, int], ...] = field(init=False)

    def __post_init__(self) -> None:
        """Validate the spread axis and derive the slot table."""

        axis = str(self.spread_axis).lower()
        if axis not in ("x", "y"):
            raise ValueError(
                f"spread_axis must be 'x' or 'y', got "
                f"{self.spread_axis!r}"
            )
        object.__setattr__(self, "spread_axis", axis)
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
