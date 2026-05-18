"""Controller-side plan: ordered pyramid skills + per-step start pose.

Standalone helper the controller node drives.  It binds every cup
skill to the chosen pyramid centre (so each destination is fixed)
and, given the node-supplied cup-middle XY, hands back the start
coordinate for each step — the pick height drops as the source stack
depletes, exactly as the reference monolithic task did.
"""

from cup_stack.skills.base import PickSpec, RobotIO
from cup_stack.skills.config import DOWN_ORI, PICK_ORI, SkillStackConfig
from cup_stack.skills.place_cup_skill import PlaceCupSkill
from cup_stack.skills.pyramid_slot import build_pyramid_slots


class PyramidStackPlan:
    """Ordered list of one-cup skills for a 3-2-1 pyramid build."""

    def __init__(
        self,
        robot: RobotIO,
        center_xy: tuple[float, float],
        nest_inc: float,
        config: SkillStackConfig | None = None,
    ) -> None:
        self.robot = robot
        self.config = config or SkillStackConfig()
        self.nest_inc = nest_inc
        self.center_xy = center_xy
        self.logger = robot.logger
        self.skills: list[PlaceCupSkill] = [
            PlaceCupSkill(robot, slot, center_xy, self.config)
            for slot in build_pyramid_slots(self.config)
        ]

    def __len__(self) -> int:
        """Return the number of cup skills in the plan."""

        return len(self.skills)

    def pick_spec(
        self, step_index: int, pick_x: float, pick_y: float
    ) -> PickSpec:
        """Start coordinate for ``step_index`` from the node's XY.

        Z drops by ``nest_inc`` per cup as the source stack empties,
        and the gripper orientation alternates exactly as the
        reference task did.
        """

        cfg = self.config
        pick_z = cfg.pick_z_base + (
            cfg.nested_count - step_index - 1
        ) * self.nest_inc
        ori = PICK_ORI if step_index % 2 == 1 else DOWN_ORI
        return PickSpec(x=pick_x, y=pick_y, z=pick_z, ori=ori)

    def log_plan(self) -> None:
        """Log the full per-cup plan, mirroring the reference layout."""

        cfg = self.config
        cx, cy = self.center_xy
        self.logger.info("=" * 60)
        self.logger.info(
            f"3-2-1 pyramid as {len(self.skills)} cup skills "
            f"(centre={cx:.3f},{cy:.3f}, spread={cfg.spread_axis}-axis)"
        )
        self.logger.info(
            f"nest_inc={self.nest_inc * 1000:.1f}mm, "
            f"layer_height={cfg.layer_height * 1000:.1f}mm, "
            f"nested_count={cfg.nested_count}"
        )
        self.logger.info("-" * 60)
        for i, skill in enumerate(self.skills):
            pick_z = cfg.pick_z_base + (
                cfg.nested_count - i - 1
            ) * self.nest_inc
            self.logger.info(
                f"  [{i + 1}] {skill.describe()}  "
                f"(pick z={pick_z:.3f}, stack {cfg.nested_count - i})"
            )
        self.logger.info("=" * 60)
