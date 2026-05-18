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
    bottom-centre to the point the gripper actually grips.  Used to
    convert an externally supplied cup-bottom Z into an actual pick Z:
    ``pick_z = cup_bottom_z + cup_grip_z_offset``.  Calibrate this
    value against the physical cup geometry.
    """

    total_cups: int = 6
    nested_count: int = 6
    spread_axis: str = "y"
    pick_safe_z: float = 0.55
    safe_z_min: float = 0.25
    pick_z_base: float = 0.323
    cup_grip_z_offset: float = 0.10
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
