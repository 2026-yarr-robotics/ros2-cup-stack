"""HTTP API server node: expose cup skills as REST endpoints.

Runs a FastAPI/uvicorn server in a daemon thread alongside the ROS 2
/ MoveItPy runtime.  Only one skill executes at a time; concurrent
requests receive ``409 Conflict``.

Endpoints
---------
GET  /status           -- liveness and busy check
POST /skill/pick       -- pick a cup from an explicit XYZ
POST /skill/pyramid    -- run the full 6-cup pyramid sequence
POST /skill/scan       -- launch the existing scan node
"""

import threading

import rclpy
from rclpy.node import Node

try:
    import uvicorn
    from fastapi import FastAPI, HTTPException
    from pydantic import BaseModel
except ImportError as exc:
    raise SystemExit(
        "skill_api_node requires fastapi and uvicorn: "
        "pip install fastapi uvicorn"
    ) from exc

from cup_stack.runtime import CupStackRuntime
from cup_stack.skills.config import SkillStackConfig
from cup_stack.skills.pick_cup_skill import PickCupSkill
from cup_stack.skills.pyramid_plan import PyramidStackPlan, SourceStack
from cup_stack.skills.scan_skill import ScanSkill


# ---------------------------------------------------------------------------
# Request / response models
# ---------------------------------------------------------------------------

class PickRequest(BaseModel):
    """Body for POST /skill/pick."""

    x: float
    y: float
    z: float
    ori: dict | None = None


class PerStepSource(BaseModel):
    """Source stack for one pyramid step."""

    x: float
    y: float
    nested_count: int


class PyramidRequest(BaseModel):
    """Body for POST /skill/pyramid."""

    center_x: float
    center_y: float
    pick_x: float
    pick_y: float
    nested_count: int = 6
    spread_axis: str = "y"
    nest_inc: float = 0.012
    per_step: list[PerStepSource] | None = None


class SkillResponse(BaseModel):
    """Uniform response for all skill endpoints."""

    success: bool
    skill: str
    detail: str = ""


# ---------------------------------------------------------------------------
# FastAPI app — _runtime is injected by the ROS 2 node before uvicorn starts
# ---------------------------------------------------------------------------

app = FastAPI(title="CupStack Skill API")
_runtime: CupStackRuntime | None = None
_lock = threading.Lock()


def _check_busy() -> None:
    if not _lock.acquire(blocking=False):
        raise HTTPException(status_code=409, detail="a skill is already running")


@app.get("/status")
def status() -> dict:
    """Return server liveness and whether a skill is currently running."""

    return {"ready": _runtime is not None, "busy": _lock.locked()}


@app.post("/skill/pick", response_model=SkillResponse)
def skill_pick(req: PickRequest) -> SkillResponse:
    """Pick a cup from the given XYZ coordinate."""

    _check_busy()
    try:
        skill = PickCupSkill(_runtime, req.x, req.y, req.z, ori=req.ori)
        ok = skill.execute()
        return SkillResponse(success=ok, skill="pick")
    finally:
        _lock.release()


@app.post("/skill/pyramid", response_model=SkillResponse)
def skill_pyramid(req: PyramidRequest) -> SkillResponse:
    """Run the full 6-cup 3-2-1 pyramid sequence."""

    _check_busy()
    try:
        config = SkillStackConfig(
            spread_axis=req.spread_axis,
            nested_count=req.nested_count,
        )
        plan = PyramidStackPlan(
            _runtime,
            (req.center_x, req.center_y),
            nest_inc=req.nest_inc,
            config=config,
        )
        if req.per_step and len(req.per_step) == len(plan):
            stacks = [
                SourceStack(x=s.x, y=s.y, nested_count=s.nested_count)
                for s in req.per_step
            ]
        else:
            stacks = [
                SourceStack(
                    x=req.pick_x, y=req.pick_y,
                    nested_count=req.nested_count,
                )
                for _ in plan.skills
            ]
        plan.log_plan(stacks)
        for i, skill in enumerate(plan.skills):
            pick = plan.pick_spec(i, stacks)
            if not skill.execute(pick):
                return SkillResponse(
                    success=False, skill="pyramid",
                    detail=f"step {i} ({skill.name}) failed",
                )
        return SkillResponse(success=True, skill="pyramid")
    finally:
        _lock.release()


@app.post("/skill/scan", response_model=SkillResponse)
def skill_scan() -> SkillResponse:
    """Launch the existing scan node and wait for completion."""

    _check_busy()
    try:
        skill = ScanSkill(logger=_runtime.logger)
        ok = skill.execute()
        return SkillResponse(success=ok, skill="scan")
    finally:
        _lock.release()


# ---------------------------------------------------------------------------
# ROS 2 node entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    """Run the skill API server node."""

    rclpy.init(args=args)
    node = Node("skill_api_node")
    node.declare_parameter("host", "0.0.0.0")
    node.declare_parameter("port", 8765)
    node.declare_parameter("move_home", False)

    global _runtime
    try:
        host = str(node.get_parameter("host").value)
        port = int(node.get_parameter("port").value)
        move_home = bool(node.get_parameter("move_home").value)
        log = node.get_logger()

        _runtime = CupStackRuntime(node, "skill_api_moveit_py")

        if move_home:
            log.info("Moving HOME before starting API server")
            if not _runtime.try_move_home():
                log.error("HOME failed; aborting")
                return

        log.info(f"Starting Skill API on http://{host}:{port}")
        threading.Thread(
            target=uvicorn.run,
            kwargs={
                "app": app, "host": host, "port": port,
                "log_level": "info",
            },
            daemon=True,
        ).start()

        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
