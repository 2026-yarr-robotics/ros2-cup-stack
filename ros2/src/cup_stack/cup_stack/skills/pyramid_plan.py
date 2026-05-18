"""Controller-side plan: ordered pyramid skills + per-step start pose.

Standalone helper the controller node drives.  It binds every cup
skill to the chosen pyramid centre (so each destination is fixed)
and, given the node-supplied ``SourceStack`` list, hands back the
start coordinate for each step.  Steps that share the same XY
automatically track stack depletion — Z drops by ``nest_inc`` each
time the same position is picked.
"""

from dataclasses import dataclass

from cup_stack.skills.base import PickSpec, RobotIO
from cup_stack.skills.config import DOWN_ORI, PICK_ORI, SkillStackConfig
from cup_stack.skills.place_cup_skill import PlaceCupSkill
from cup_stack.skills.pyramid_slot import build_pyramid_slots


@dataclass
class SourceStack:
    """One physical nested-cup source stack at a fixed XY location.

    ``nested_count`` is the number of cups in this stack at the start
    of the sequence.  Steps that share the same ``(x, y)`` draw from
    the same stack; Z drops automatically with each pick.
    """

    x: float
    y: float
    nested_count: int


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
        self, step_index: int, stacks: list[SourceStack]
    ) -> PickSpec:
        """Build a ``PickSpec`` for ``step_index`` from its ``SourceStack``.

        Z accounts for how many cups have already been taken from the
        same ``(x, y)`` position in prior steps.  Orientation alternates
        per step as the reference task does.
        """

        stack = stacks[step_index]
        prior = sum(
            1 for s in stacks[:step_index]
            if s.x == stack.x and s.y == stack.y
        )
        pick_z = self.config.pick_z_base + (
            stack.nested_count - prior - 1
        ) * self.nest_inc
        ori = PICK_ORI if step_index % 2 == 1 else DOWN_ORI
        return PickSpec(x=stack.x, y=stack.y, z=pick_z, ori=ori)

    def log_plan(self, stacks: list[SourceStack]) -> None:
        """Log the full per-cup plan with per-step pick coordinates."""

        cfg = self.config
        cx, cy = self.center_xy
        self.logger.info("=" * 60)
        self.logger.info(
            f"3-2-1 pyramid as {len(self.skills)} cup skills "
            f"(centre={cx:.3f},{cy:.3f}, spread={cfg.spread_axis}-axis)"
        )
        self.logger.info(
            f"nest_inc={self.nest_inc * 1000:.1f}mm, "
            f"layer_height={cfg.layer_height * 1000:.1f}mm"
        )
        self.logger.info("-" * 60)
        for i, skill in enumerate(self.skills):
            stack = stacks[i]
            prior = sum(
                1 for s in stacks[:i]
                if s.x == stack.x and s.y == stack.y
            )
            pick_z = cfg.pick_z_base + (
                stack.nested_count - prior - 1
            ) * self.nest_inc
            self.logger.info(
                f"  [{i + 1}] {skill.describe()}  "
                f"(pick=({stack.x:.3f},{stack.y:.3f},{pick_z:.3f}), "
                f"nested={stack.nested_count}, prior_picks={prior})"
            )
        self.logger.info("=" * 60)
