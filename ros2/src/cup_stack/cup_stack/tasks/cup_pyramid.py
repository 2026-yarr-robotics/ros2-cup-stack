"""Task that builds a six-cup 3-2-1 pyramid from a nested stack."""

from cup_stack.config import DOWN_ORI, PICK_ORI, CupStackConfig
from cup_stack.geometry import make_twist_orientation
from cup_stack.runtime import CupStackRuntime


class CupPyramidTask:
    """Build a cup pyramid using the current HOME XY as the source stack."""

    def __init__(
        self,
        runtime: CupStackRuntime,
        nest_inc: float,
        config: CupStackConfig | None = None,
    ) -> None:
        self.runtime = runtime
        self.nest_inc = nest_inc
        self.config = config or CupStackConfig()
        self.logger = runtime.logger

    def try_execute(
        self,
        pick_xy: tuple[float, float] | None = None,
        move_home: bool = True,
    ) -> bool:
        """Execute the full pyramid build task."""

        self.log_plan()
        if move_home:
            self.logger.info("[0] Moving HOME")
            if not self.runtime.try_move_home():
                return False
            self.runtime.sleep(self.config.home_sleep_sec)

        if pick_xy is None:
            pick_x, pick_y = self.runtime.current_ee_xy()
        else:
            pick_x, pick_y = pick_xy
        pyramid_cx = pick_x + self.config.place_x_offset
        pyramid_cy = pick_y
        target_x, target_y = pyramid_cx, pyramid_cy

        for index in range(self.config.total_cups):
            y_offset, layer = self.config.pyramid_places[index]
            target_x = pyramid_cx
            target_y = pyramid_cy + y_offset
            if not self.try_cycle(
                cycle_index=index,
                pick_x=pick_x,
                pick_y=pick_y,
                target_x=target_x,
                target_y=target_y,
                y_offset=y_offset,
                layer=layer,
            ):
                return False

        self.logger.info("\n[END] Moving to SAFE_Z")
        self.runtime.try_move_to_pose(
            target_x,
            target_y,
            self.config.safe_z,
            self.config.safe_z_min,
            ori=DOWN_ORI,
        )
        self.logger.info("=== Cup pyramid complete ===")
        return True

    def try_cycle(
        self,
        cycle_index: int,
        pick_x: float,
        pick_y: float,
        target_x: float,
        target_y: float,
        y_offset: float,
        layer: int,
    ) -> bool:
        """Execute one pick-place cycle."""

        cycle = cycle_index + 1
        pick_z = self.config.pick_z_base + (
            self.config.total_cups - cycle_index - 1
        ) * self.nest_inc
        place_z = self.config.place_z_base + layer * self.config.layer_height
        pick_ori = PICK_ORI if cycle_index % 2 == 1 else DOWN_ORI

        self.logger.info(f"\n{'=' * 50}")
        self.logger.info(
            f"CYCLE {cycle}/{self.config.total_cups}  "
            f"pick(stack={self.config.total_cups - cycle_index}, "
            f"z={pick_z:.3f}) -> "
            f"place y_off={y_offset * 1000:+.0f}mm L{layer} z={place_z:.3f}"
        )
        self.logger.info(f"{'=' * 50}")

        if not self.try_move_to_pick_safe(cycle, pick_x, pick_y, pick_ori):
            return False
        if not self.try_pick_cup(cycle, pick_x, pick_y, pick_z, pick_ori):
            return False
        if not self.try_lift_from_pick(cycle, pick_x, pick_y, pick_ori):
            return False
        if not self.try_move_to_place_safe(cycle, target_x, target_y):
            return False
        if not self.try_place_cup(cycle, target_x, target_y, place_z):
            return False
        return self.try_lift_from_place(cycle, target_x, target_y)

    def try_move_to_pick_safe(
        self,
        cycle: int,
        pick_x: float,
        pick_y: float,
        pick_ori: dict[str, float],
    ) -> bool:
        self.logger.info("  [1] pick XY move @ SAFE_Z")
        return self._require(
            self.runtime.try_move_to_pose(
                pick_x,
                pick_y,
                self.config.safe_z,
                self.config.safe_z_min,
                ori=pick_ori,
            ),
            1,
            cycle,
        )

    def try_pick_cup(
        self,
        cycle: int,
        pick_x: float,
        pick_y: float,
        pick_z: float,
        pick_ori: dict[str, float],
    ) -> bool:
        self.logger.info("  [2] gripper OPEN")
        self.runtime.try_open_gripper(self.config.open_sleep_sec)
        self.logger.info(f"  [3] pick descend -> z={pick_z:.3f}")
        if not self._require(
            self.runtime.try_move_to_pose(
                pick_x,
                pick_y,
                pick_z,
                self.config.safe_z_min,
                ori=pick_ori,
                lin=True,
            ),
            3,
            cycle,
        ):
            return False
        self.logger.info("  [4] GRIP")
        return self.runtime.try_grip_cup(self.config.grip_sleep_sec)

    def try_lift_from_pick(
        self,
        cycle: int,
        pick_x: float,
        pick_y: float,
        pick_ori: dict[str, float],
    ) -> bool:
        self.logger.info("  [5] lift -> SAFE_Z")
        return self._require(
            self.runtime.try_move_to_pose(
                pick_x,
                pick_y,
                self.config.safe_z,
                self.config.safe_z_min,
                ori=pick_ori,
                lin=True,
            ),
            5,
            cycle,
        )

    def try_move_to_place_safe(
        self,
        cycle: int,
        target_x: float,
        target_y: float,
    ) -> bool:
        self.logger.info(
            f"  [6] target XY move ({target_x:.3f}, {target_y:.3f}) @ SAFE_Z"
        )
        return self._require(
            self.runtime.try_move_to_pose(
                target_x,
                target_y,
                self.config.safe_z,
                self.config.safe_z_min,
            ),
            6,
            cycle,
        )

    def try_place_cup(
        self,
        cycle: int,
        target_x: float,
        target_y: float,
        place_z: float,
    ) -> bool:
        mid_z = place_z + (self.config.safe_z - place_z) / 2.0
        half_twist = make_twist_orientation(self.config.place_twist_deg / 2.0)
        full_twist = make_twist_orientation(self.config.place_twist_deg)

        self.logger.info(f"  [7a] place mid descend -> z={mid_z:.3f}")
        if not self._require(
            self.runtime.try_move_to_pose(
                target_x,
                target_y,
                mid_z,
                self.config.safe_z_min,
                ori=half_twist,
            ),
            "7a",
            cycle,
        ):
            return False

        self.logger.info(f"  [7b] place final descend -> z={place_z:.3f}")
        if not self._require(
            self.runtime.try_move_to_pose(
                target_x,
                target_y,
                place_z,
                self.config.safe_z_min,
                ori=full_twist,
            ),
            "7b",
            cycle,
        ):
            return False

        self.logger.info("  [8] RELEASE")
        return self.runtime.try_release_cup(self.config.release_sleep_sec)

    def try_lift_from_place(
        self,
        cycle: int,
        target_x: float,
        target_y: float,
    ) -> bool:
        self.logger.info("  [9] lift -> SAFE_Z")
        return self._require(
            self.runtime.try_move_to_pose(
                target_x,
                target_y,
                self.config.safe_z,
                self.config.safe_z_min,
                ori=DOWN_ORI,
            ),
            9,
            cycle,
        )

    def log_plan(self) -> None:
        self.logger.info("=" * 60)
        self.logger.info("3-2-1 cup pyramid build (6 cups)")
        self.logger.info(
            f"nest_inc={self.nest_inc * 1000:.1f}mm, "
            f"layer_height={self.config.layer_height * 1000:.1f}mm"
        )
        self.logger.info("-" * 60)
        for index in range(self.config.total_cups):
            pick_z = self.config.pick_z_base + (
                self.config.total_cups - index - 1
            ) * self.nest_inc
            y_offset, layer = self.config.pyramid_places[index]
            place_z = (
                self.config.place_z_base
                + layer * self.config.layer_height
            )
            self.logger.info(
                f"  cycle {index + 1}: pick z={pick_z:.3f}"
                f"(stack {self.config.total_cups - index}) "
                "-> "
                f"place y_off={y_offset * 1000:+.0f}mm "
                f"L{layer} z={place_z:.3f}"
            )
        self.logger.info("=" * 60)

    def _require(self, ok: bool, step: int | str, cycle: int) -> bool:
        if ok:
            return True
        self.logger.error(
            f"=== CYCLE {cycle} STEP {step} failed; aborting ==="
        )
        return False
