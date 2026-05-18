"""Controller node: run the 3-2-1 pyramid as ordered cup skills.

Each cup is one skill (tier1 left/mid/right, tier2 x2, tier3 cap).
This node owns the sequence and *presents* the start coordinate — the
cup-middle XY of the nested source stack.  Every skill already knows
its destination from the pyramid centre.

This node is the only integration point between the standalone
``cup_stack.skills`` package and the existing runtime: the runtime is
passed in by duck typing as a ``RobotIO``, so the skills themselves
stay free of any dependency on the reference code.
"""

import math

import rclpy
from rclpy.node import Node

from cup_stack.runtime import CupStackRuntime
from cup_stack.skills.config import SkillStackConfig
from cup_stack.skills.pyramid_plan import PyramidStackPlan


def _resolved(node: Node, name: str, fallback: float) -> float:
    """Return the float param, or ``fallback`` when it is NaN/unset."""

    value = float(node.get_parameter(name).value)
    return fallback if math.isnan(value) else value


def main(args=None):
    """Run the skill-based pyramid controller node."""

    rclpy.init(args=args)
    node = Node("cup_pyramid_skill_node")
    node.declare_parameter("nest_inc", 0.012)
    node.declare_parameter("pick_x", math.nan)
    node.declare_parameter("pick_y", math.nan)
    node.declare_parameter("place_x", math.nan)
    node.declare_parameter("place_y", math.nan)
    node.declare_parameter("move_home", True)
    node.declare_parameter("spread_axis", "y")
    node.declare_parameter("nested_count", 6)

    try:
        nest_inc = float(node.get_parameter("nest_inc").value)
        move_home = bool(node.get_parameter("move_home").value)
        log = node.get_logger()

        # Workbench axis the pyramid row spreads along ("x" or "y").
        spread_axis = str(node.get_parameter("spread_axis").value).lower()
        if spread_axis not in ("x", "y"):
            log.warn(
                f"spread_axis={spread_axis!r} invalid; using 'y'"
            )
            spread_axis = "y"

        # Cups pre-nested in the source stack -> top-cup pick height.
        nested_count = int(node.get_parameter("nested_count").value)
        config = SkillStackConfig(
            spread_axis=spread_axis, nested_count=nested_count
        )
        if nested_count < config.total_cups:
            log.warn(
                f"nested_count={nested_count} < {config.total_cups} "
                "cups to place; deepest picks may underrun the stack"
            )

        runtime = CupStackRuntime(node, "cup_pyramid_skill_moveit_py")

        if move_home:
            log.info("[0] Moving HOME")
            if not runtime.try_move_home():
                log.error("HOME failed; aborting")
                return
            runtime.sleep(config.home_sleep_sec)

        # Start coordinate: cup-middle XY, presented by this node.
        ee_x, ee_y = runtime.current_ee_xy()
        pick_x = _resolved(node, "pick_x", ee_x)
        pick_y = _resolved(node, "pick_y", ee_y)

        # Pyramid centre fixes every destination.
        center_x = _resolved(
            node, "place_x", pick_x + config.place_x_offset
        )
        center_y = _resolved(node, "place_y", pick_y)

        log.info(
            f"pick(cup-middle)=({pick_x:.3f},{pick_y:.3f})  "
            f"pyramid_center=({center_x:.3f},{center_y:.3f})  "
            f"spread={spread_axis}-axis  nested_count={nested_count}"
        )

        plan = PyramidStackPlan(
            runtime,
            (center_x, center_y),
            nest_inc=nest_inc,
            config=config,
        )
        plan.log_plan()

        for i, skill in enumerate(plan.skills):
            log.info(f"--- step {i + 1}/{len(plan)}: {skill.name} ---")
            pick = plan.pick_spec(i, pick_x, pick_y)
            if not skill.execute(pick):
                log.error(
                    f"skill {skill.name} failed; aborting sequence"
                )
                return

        log.info("[END] Moving to PICK_SAFE_Z")
        runtime.try_move_to_pose(
            center_x,
            center_y,
            config.pick_safe_z,
            config.safe_z_min,
        )
        log.info("=== Cup pyramid (skills) complete ===")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
