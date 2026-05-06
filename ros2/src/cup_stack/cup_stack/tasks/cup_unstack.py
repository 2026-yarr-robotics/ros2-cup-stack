"""Task that restores a six-cup pyramid back into a nested stack."""

from cup_stack.config import CupStackConfig
from cup_stack.runtime import CupStackRuntime


class CupUnstackTask:
    """Pick cups from a pyramid and return them to the source stack."""

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
        pyramid_xy: tuple[float, float] | None = None,
    ) -> bool:
        """Execute the full pyramid unstack task."""

        self.log_plan()
        if pyramid_xy is None:
            self.logger.info("[0] Reading current FK from pyramid center")
            pyramid_cx, pyramid_cy = self.runtime.current_ee_xy()
        else:
            pyramid_cx, pyramid_cy = pyramid_xy
            self.logger.info("[0] Using selected pyramid center")
        nest_x = pyramid_cx - self.config.place_x_offset
        nest_y = pyramid_cy
        self.logger.info(
            f"    pyramid_center=({pyramid_cx:.3f},{pyramid_cy:.3f})  "
            f"nest=({nest_x:.3f},{nest_y:.3f})"
        )

        target_x, target_y = nest_x, nest_y
        for index in range(self.config.total_cups):
            y_offset, layer = self.config.reverse_picks[index]
            src_x = pyramid_cx
            src_y = pyramid_cy + y_offset
            target_x, target_y = nest_x, nest_y
            if not self.try_cycle(
                cycle_index=index,
                src_x=src_x,
                src_y=src_y,
                nest_x=nest_x,
                nest_y=nest_y,
                y_offset=y_offset,
                layer=layer,
            ):
                return False

        self.logger.info("\n[END] Holding SAFE_Z")
        self.runtime.try_move_to_pose(
            target_x,
            target_y,
            self.config.safe_z,
            self.config.safe_z_min,
        )
        self.logger.info("=== Cup unstack complete ===")
        return True

    def try_cycle(
        self,
        cycle_index: int,
        src_x: float,
        src_y: float,
        nest_x: float,
        nest_y: float,
        y_offset: float,
        layer: int,
    ) -> bool:
        """Execute one pyramid-to-nest cycle."""

        cycle = cycle_index + 1
        src_z = self.config.place_z_base + layer * self.config.layer_height
        dst_z = self.config.pick_z_base + cycle_index * self.nest_inc

        self.logger.info(f"\n{'=' * 50}")
        self.logger.info(
            f"CYCLE {cycle}/{self.config.total_cups}  "
            f"pyramid pick ({src_x:.3f},{src_y:.3f}) z={src_z:.3f} -> "
            f"nest place z={dst_z:.3f}(stack {cycle})"
        )
        self.logger.info(f"{'=' * 50}")

        if not self.try_move_to_pyramid_pick_safe(cycle, src_x, src_y):
            return False
        if not self.try_pick_from_pyramid(cycle, src_x, src_y, src_z):
            return False
        if not self.try_lift_from_pyramid(cycle, src_x, src_y):
            return False
        if not self.try_move_to_nest_safe(cycle, nest_x, nest_y):
            return False
        if not self.try_place_on_nest(cycle, nest_x, nest_y, dst_z):
            return False
        return self.try_lift_from_nest(cycle, nest_x, nest_y)

    def try_move_to_pyramid_pick_safe(
        self,
        cycle: int,
        src_x: float,
        src_y: float,
    ) -> bool:
        self.logger.info("  [1] pyramid pick XY move @ SAFE_Z")
        return self._require(
            self.runtime.try_move_to_pose(
                src_x,
                src_y,
                self.config.safe_z,
                self.config.safe_z_min,
            ),
            1,
            cycle,
        )

    def try_pick_from_pyramid(
        self,
        cycle: int,
        src_x: float,
        src_y: float,
        src_z: float,
    ) -> bool:
        self.logger.info("  [2] gripper OPEN")
        self.runtime.try_open_gripper(self.config.open_sleep_sec)
        self.logger.info(f"  [3] pick descend -> z={src_z:.3f}")
        if not self._require(
            self.runtime.try_move_to_pose(
                src_x,
                src_y,
                src_z,
                self.config.safe_z_min,
                lin=True,
            ),
            3,
            cycle,
        ):
            return False
        self.logger.info("  [4] GRIP")
        return self.runtime.try_grip_cup(self.config.grip_sleep_sec)

    def try_lift_from_pyramid(
        self,
        cycle: int,
        src_x: float,
        src_y: float,
    ) -> bool:
        self.logger.info("  [5] lift -> SAFE_Z")
        return self._require(
            self.runtime.try_move_to_pose(
                src_x,
                src_y,
                self.config.safe_z,
                self.config.safe_z_min,
                lin=True,
            ),
            5,
            cycle,
        )

    def try_move_to_nest_safe(
        self,
        cycle: int,
        nest_x: float,
        nest_y: float,
    ) -> bool:
        self.logger.info(
            f"  [6] nest XY move ({nest_x:.3f},{nest_y:.3f}) @ PICK_SAFE_Z"
        )
        return self._require(
            self.runtime.try_move_to_pose(
                nest_x,
                nest_y,
                self.config.pick_safe_z,
                self.config.safe_z_min,
            ),
            6,
            cycle,
        )

    def try_place_on_nest(
        self,
        cycle: int,
        nest_x: float,
        nest_y: float,
        dst_z: float,
    ) -> bool:
        self.logger.info(f"  [7] nest place descend -> z={dst_z:.3f}")
        if not self._require(
            self.runtime.try_move_to_pose(
                nest_x,
                nest_y,
                dst_z,
                self.config.safe_z_min,
                lin=True,
                strict=True,
            ),
            7,
            cycle,
        ):
            return False
        self.logger.info("  [8] RELEASE")
        return self.runtime.try_release_cup(self.config.release_sleep_sec)

    def try_lift_from_nest(
        self,
        cycle: int,
        nest_x: float,
        nest_y: float,
    ) -> bool:
        self.logger.info("  [9] lift -> PICK_SAFE_Z")
        return self._require(
            self.runtime.try_move_to_pose(
                nest_x,
                nest_y,
                self.config.pick_safe_z,
                self.config.safe_z_min,
            ),
            9,
            cycle,
        )

    def log_plan(self) -> None:
        self.logger.info("=" * 60)
        self.logger.info("Pyramid to nested cup stack (6 cups)")
        self.logger.info(
            f"nest_inc={self.nest_inc * 1000:.1f}mm, "
            f"layer_height={self.config.layer_height * 1000:.1f}mm"
        )
        self.logger.info("-" * 60)
        for index in range(self.config.total_cups):
            y_offset, layer = self.config.reverse_picks[index]
            pick_z = (
                self.config.place_z_base
                + layer * self.config.layer_height
            )
            place_z = self.config.pick_z_base + index * self.nest_inc
            self.logger.info(
                f"  cycle {index + 1}: pyramid pick "
                f"y_off={y_offset * 1000:+.0f}mm L{layer} z={pick_z:.3f}"
                f" -> nest place z={place_z:.3f}(stack {index + 1})"
            )
        self.logger.info("=" * 60)

    def _require(self, ok: bool, step: int | str, cycle: int) -> bool:
        if ok:
            return True
        self.logger.error(
            f"=== CYCLE {cycle} STEP {step} failed; aborting ==="
        )
        return False
