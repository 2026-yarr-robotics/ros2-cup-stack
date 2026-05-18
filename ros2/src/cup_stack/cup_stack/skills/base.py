"""Base abstractions shared by cup stacking skills.

The skills package is standalone: it never imports the existing
runtime/task code.  Instead it talks to the robot through the
:class:`RobotIO` structural interface, which the controller node
satisfies by passing in any object that has these members (the
existing ``CupStackRuntime`` already does, by duck typing).
"""

from dataclasses import dataclass
from typing import Protocol, runtime_checkable


@dataclass(frozen=True)
class PickSpec:
    """Start coordinate for one skill, presented by the controller node.

    The pyramid centre fixes every destination, so a skill resolves
    its own place pose.  The pick side cannot be derived that way: it
    depends on how many cups remain in the source stack, so the node
    hands it over here.

    x, y are the cup-middle XY of the nested source stack (base_link,
    metres).  z is the descend height for this cup and drops as the
    stack depletes.  ori is the gripper orientation for the pick, or
    None to use DOWN_ORI.
    """

    x: float
    y: float
    z: float
    ori: dict[str, float] | None = None


@runtime_checkable
class RobotIO(Protocol):
    """Minimal robot/gripper surface a skill needs to drive hardware.

    Deliberately matches the public method names of the reference
    runtime so an existing runtime instance can be passed straight in,
    while keeping this package free of any import on it.
    """

    @property
    def logger(self):
        """Logger with ``info``/``warn``/``error`` methods."""

    def try_move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        safe_z_min: float,
        ori: dict[str, float] | None = None,
        lin: bool = False,
        strict: bool = False,
    ) -> bool:
        """Plan and execute a Cartesian pose move."""

    def try_open_gripper(self, sleep_sec: float) -> bool:
        """Open the gripper."""

    def try_grip_cup(self, sleep_sec: float) -> bool:
        """Close the gripper onto a cup."""

    def try_release_cup(self, sleep_sec: float) -> bool:
        """Release a held cup."""


class Skill:
    """One independently executable step of a stacking sequence."""

    name: str

    def describe(self) -> str:
        """Return a one-line human summary for plan logging."""

        raise NotImplementedError

    def execute(self, pick: PickSpec | None = None) -> bool:
        """Run the skill; return True on success, False to abort.

        ``pick`` is the start coordinate for cup-placement skills.
        Skills that do not pick a cup (e.g. scan, which just runs the
        existing node) ignore it.
        """

        raise NotImplementedError
