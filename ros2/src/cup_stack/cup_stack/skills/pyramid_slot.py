"""Pyramid slot model: where each cup goes, relative to the centre."""

from dataclasses import dataclass

from cup_stack.skills.config import SkillStackConfig


@dataclass(frozen=True)
class PyramidSlot:
    """One cup target in the pyramid, relative to its centre.

    name is a stable id, e.g. ``tier1_left`` / ``tier3_top``.  tier is
    the 1-based tier number (1 = bottom, widest row) and column is the
    0-based position within the tier, left -> right.  lateral_offset
    is the in-row distance from the pyramid centre (metres); it is
    applied to whichever workbench axis ``config.spread_axis``
    selects.  layer is the 0-based vertical layer that selects the
    place-Z height.
    """

    name: str
    tier: int
    column: int
    lateral_offset: float
    layer: int


_COLUMN_NAMES = {
    1: ("top",),
    2: ("left", "right"),
    3: ("left", "mid", "right"),
}


def build_pyramid_slots(
    config: SkillStackConfig | None = None,
) -> tuple[PyramidSlot, ...]:
    """Build ordered pyramid slots from ``config.pyramid_places``.

    Placement order is preserved exactly: tier 1 left/mid/right, then
    tier 2's two cups, then the tier 3 cap.  Each returned slot is one
    cup unit, in the order it should be stacked.
    """

    cfg = config or SkillStackConfig()
    places = cfg.pyramid_places

    layer_counts: dict[int, int] = {}
    for _, layer in places:
        layer_counts[layer] = layer_counts.get(layer, 0) + 1

    seen: dict[int, int] = {}
    slots: list[PyramidSlot] = []
    for lateral_offset, layer in places:
        column = seen.get(layer, 0)
        seen[layer] = column + 1
        tier = layer + 1
        size = layer_counts[layer]
        labels = _COLUMN_NAMES.get(size)
        label = labels[column] if labels else f"c{column}"
        slots.append(
            PyramidSlot(
                name=f"tier{tier}_{label}",
                tier=tier,
                column=column,
                lateral_offset=lateral_offset,
                layer=layer,
            )
        )
    return tuple(slots)
