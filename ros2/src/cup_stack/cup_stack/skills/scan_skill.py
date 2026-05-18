"""Skill: run the existing scan node as a sub-process.

Per the requirement, scan is exposed as a skill but is **not**
re-implemented here — this skill simply launches the existing
``cup_stack`` scan node (``ros2 launch cup_stack scan.launch.py``)
and reports whether it finished cleanly.  It therefore stays free of
any import on the reference task/runtime/config code (only the
standard library is used), in line with the rest of this package.
"""

import subprocess

from cup_stack.skills.base import PickSpec, Skill


class _PrintLogger:
    """Minimal ROS-free logger used when none is supplied."""

    def info(self, msg: str) -> None:
        """Print an info line."""

        print(msg)

    def warn(self, msg: str) -> None:
        """Print a warning line."""

        print(f"WARN: {msg}")

    def error(self, msg: str) -> None:
        """Print an error line."""

        print(f"ERROR: {msg}")


class ScanSkill(Skill):
    """Run the existing scan node and wait for it to complete.

    The skill shells out to ``ros2 launch`` so the proven scan node
    runs unchanged in its own process (its own MoveItPy instance),
    fully isolated from the controller.
    """

    def __init__(
        self,
        logger=None,
        launch_package: str = "cup_stack",
        launch_file: str = "scan.launch.py",
        ros2_bin: str = "ros2",
        extra_args: list[str] | None = None,
        timeout_sec: float = 180.0,
        success_marker: str = "Scan task complete",
    ) -> None:
        self.name = "scan"
        self.logger = logger or _PrintLogger()
        self.launch_package = launch_package
        self.launch_file = launch_file
        self.ros2_bin = ros2_bin
        self.extra_args = list(extra_args or [])
        self.timeout_sec = timeout_sec
        self.success_marker = success_marker

    @property
    def command(self) -> list[str]:
        """Return the argv that runs the existing scan node."""

        return [
            self.ros2_bin,
            "launch",
            self.launch_package,
            self.launch_file,
            *self.extra_args,
        ]

    def describe(self) -> str:
        """Return a one-line human summary for plan logging."""

        return f"{self.name}: run existing node `{' '.join(self.command)}`"

    def execute(self, pick: PickSpec | None = None) -> bool:
        """Launch the scan node, stream its log, report success."""

        cmd = self.command
        self.logger.info(
            f"[scan skill] running existing node: {' '.join(cmd)}"
        )
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=self.timeout_sec,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            self.logger.error(
                f"[scan skill] timed out after {self.timeout_sec:.0f}s"
            )
            for line in (exc.output or "").splitlines():
                self.logger.info(f"  [scan] {line}")
            return False
        except FileNotFoundError:
            self.logger.error(
                f"[scan skill] '{self.ros2_bin}' not found on PATH"
            )
            return False

        out = result.stdout or ""
        for line in out.splitlines():
            self.logger.info(f"  [scan] {line}")

        saw_marker = bool(self.success_marker) and (
            self.success_marker in out
        )
        ok = result.returncode == 0 and (
            saw_marker or not self.success_marker
        )
        self.logger.info(
            f"[scan skill] exit={result.returncode} "
            f"marker={saw_marker} -> {'OK' if ok else 'FAIL'}"
        )
        return ok
