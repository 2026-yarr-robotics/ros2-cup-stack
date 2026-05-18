"""HTTP client for the CupStack Skill API server.

Usable as a library or as a CLI tool::

    ros2 run cup_stack skill_api_client status
    ros2 run cup_stack skill_api_client pick 0.40 0.00 0.36
    ros2 run cup_stack skill_api_client pyramid 0.50 0.00 0.40 0.00
    ros2 run cup_stack skill_api_client scan
"""

import argparse
import json
import sys

try:
    import requests
except ImportError as exc:
    raise SystemExit(
        "api_client requires requests: pip install requests"
    ) from exc

_DEFAULT_HOST = "localhost"
_DEFAULT_PORT = 8765


class SkillApiClient:
    """Thin HTTP wrapper around the CupStack Skill API server."""

    def __init__(
        self, host: str = _DEFAULT_HOST, port: int = _DEFAULT_PORT
    ) -> None:
        self.base = f"http://{host}:{port}"

    def status(self) -> dict:
        """Return server liveness and busy state."""

        return requests.get(f"{self.base}/status", timeout=5).json()

    def pick(
        self,
        x: float,
        y: float,
        z: float | None = None,
        ori: dict | None = None,
        cup_bottom_z: float | None = None,
    ) -> dict:
        """Pick a cup from the given coordinate.

        Pass ``z`` for an explicit gripper Z, or ``cup_bottom_z`` for
        the cup-bottom centre Z (the server applies ``cup_grip_z_offset``
        to compute the actual gripper height).  Exactly one must be set.
        """

        if z is None and cup_bottom_z is None:
            raise ValueError("provide z or cup_bottom_z")
        return requests.post(
            f"{self.base}/skill/pick",
            json={"x": x, "y": y, "z": z,
                  "cup_bottom_z": cup_bottom_z, "ori": ori},
            timeout=60,
        ).json()

    def pyramid(
        self,
        center_x: float,
        center_y: float,
        pick_x: float,
        pick_y: float,
        nested_count: int = 6,
        spread_axis: str = "y",
        nest_inc: float = 0.012,
        per_step: list[dict] | None = None,
    ) -> dict:
        """Run the full 6-cup pyramid sequence.

        ``per_step`` is an optional list of
        ``{"x": ..., "y": ..., "nested_count": ...}`` dicts (one per
        step, length 6) that override the global pick position and
        nested count for each individual step.
        """

        return requests.post(
            f"{self.base}/skill/pyramid",
            json={
                "center_x": center_x,
                "center_y": center_y,
                "pick_x": pick_x,
                "pick_y": pick_y,
                "nested_count": nested_count,
                "spread_axis": spread_axis,
                "nest_inc": nest_inc,
                "per_step": per_step,
            },
            timeout=300,
        ).json()

    def scan(self) -> dict:
        """Launch the scan node and wait for it to finish."""

        return requests.post(
            f"{self.base}/skill/scan", json={}, timeout=300
        ).json()


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    """Run the CLI client."""

    parser = argparse.ArgumentParser(
        description="CupStack Skill API client"
    )
    parser.add_argument("--host", default=_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=_DEFAULT_PORT)
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="check server liveness")

    p_pick = sub.add_parser("pick", help="pick a cup from XYZ")
    p_pick.add_argument("x", type=float)
    p_pick.add_argument("y", type=float)
    p_pick.add_argument(
        "z", type=float, nargs="?", default=None,
        help="gripper Z (raw); omit when using --cup-bottom-z",
    )
    p_pick.add_argument(
        "--cup-bottom-z", type=float, default=None,
        metavar="Z",
        help="cup-bottom centre Z; server adds cup_grip_z_offset",
    )

    p_pyr = sub.add_parser("pyramid", help="run the full pyramid sequence")
    p_pyr.add_argument("center_x", type=float)
    p_pyr.add_argument("center_y", type=float)
    p_pyr.add_argument("pick_x", type=float)
    p_pyr.add_argument("pick_y", type=float)
    p_pyr.add_argument("--nested-count", type=int, default=6)
    p_pyr.add_argument("--spread-axis", default="y")
    p_pyr.add_argument("--nest-inc", type=float, default=0.012)

    sub.add_parser("scan", help="run the scan node")

    ns = parser.parse_args(args)
    client = SkillApiClient(ns.host, ns.port)

    if ns.cmd == "status":
        result = client.status()
    elif ns.cmd == "pick":
        result = client.pick(
            ns.x, ns.y,
            z=ns.z,
            cup_bottom_z=ns.cup_bottom_z,
        )
    elif ns.cmd == "pyramid":
        result = client.pyramid(
            ns.center_x, ns.center_y, ns.pick_x, ns.pick_y,
            nested_count=ns.nested_count,
            spread_axis=ns.spread_axis,
            nest_inc=ns.nest_inc,
        )
    elif ns.cmd == "scan":
        result = client.scan()

    print(json.dumps(result, indent=2))
    if isinstance(result, dict) and result.get("success") is False:
        sys.exit(1)


if __name__ == "__main__":
    main()
