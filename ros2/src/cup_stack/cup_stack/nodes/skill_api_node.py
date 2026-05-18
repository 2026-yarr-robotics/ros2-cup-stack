"""HTTP API server node: expose cup skills as REST endpoints.

Runs a FastAPI/uvicorn server in a daemon thread alongside the ROS 2
/ MoveItPy runtime.  Only one skill executes at a time; concurrent
requests receive ``409 Conflict``.

Endpoints
---------
GET  /             -- pick frontend (HTML)
GET  /status       -- liveness, busy, and cup_grip_z_offset
POST /skill/pick   -- pick a cup; accepts gripper Z or cup-bottom Z
POST /skill/pyramid -- run the full 6-cup pyramid sequence
POST /skill/scan   -- launch the existing scan node
"""

import threading

import rclpy
from rclpy.node import Node

try:
    import uvicorn
    from fastapi import FastAPI, HTTPException
    from fastapi.responses import HTMLResponse
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
# Pick frontend (served at GET /)
# ---------------------------------------------------------------------------

_FRONTEND_HTML = """\
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<title>CupStack Pick</title>
<style>
body{font-family:monospace;max-width:400px;
     margin:48px auto;padding:0 20px}
h2{margin-bottom:4px}
p.sub{margin:0 0 16px;color:#555;font-size:.9em}
label{display:block;margin-top:10px;font-size:.85em;color:#333}
input[type=number]{
  width:100%;box-sizing:border-box;
  padding:7px 8px;margin-top:3px;
  border:1px solid #bbb;border-radius:4px;font-size:1em}
#preview{
  margin-top:10px;padding:8px 10px;border-radius:4px;
  font-size:.85em;background:#f1f3f4;color:#333}
button{
  display:block;width:100%;margin-top:14px;
  padding:11px;font-size:1em;cursor:pointer;
  background:#1a73e8;color:#fff;border:none;border-radius:4px}
button:hover{background:#1558b0}
button:disabled{background:#aaa;cursor:default}
#result{
  margin-top:16px;padding:10px 12px;border-radius:4px;
  font-size:.9em;display:none}
.ok{background:#e6f4ea;color:#137333}
.err{background:#fce8e6;color:#c5221f}
.busy{background:#fef7e0;color:#b06000}
pre{margin:6px 0 0;font-size:.85em;max-height:160px;overflow:auto}
</style>
</head>
<body>
<h2>Pick Skill</h2>
<p class="sub">컵 바닥 중심 좌표 입력 (m)</p>
<label>X (m)</label>
<input id="x" type="number" step="0.001" value="0.400"
       oninput="updatePreview()">
<label>Y (m)</label>
<input id="y" type="number" step="0.001" value="0.000"
       oninput="updatePreview()">
<label>Z — 컵 바닥 (m)</label>
<input id="z" type="number" step="0.001" value="0.100"
       oninput="updatePreview()">
<div id="preview">Gripper Z: —</div>
<button id="btn" onclick="run()">Pick</button>
<div id="result"><pre id="out"></pre></div>
<script>
let _offset=null;
async function init(){
  try{
    const s=await fetch('/status');
    const j=await s.json();
    _offset=j.cup_grip_z_offset??null;
    updatePreview();
  }catch(_){}
}
function updatePreview(){
  const z=parseFloat(document.getElementById('z').value);
  const el=document.getElementById('preview');
  if(_offset!==null&&!isNaN(z)){
    const gz=(z+_offset).toFixed(4);
    el.textContent=
      'Gripper Z = '+z.toFixed(3)+' + '+_offset.toFixed(3)
      +' = '+gz+' m';
  }else{
    el.textContent='Gripper Z: —';
  }
}
async function run(){
  const btn=document.getElementById('btn');
  const rd=document.getElementById('result');
  const out=document.getElementById('out');
  btn.disabled=true;
  rd.className='busy';rd.style.display='block';
  out.textContent='Running...';
  const body={
    x:+document.getElementById('x').value,
    y:+document.getElementById('y').value,
    cup_bottom_z:+document.getElementById('z').value
  };
  try{
    const r=await fetch('/skill/pick',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify(body)
    });
    const j=await r.json();
    rd.className=j.success?'ok':'err';
    out.textContent=JSON.stringify(j,null,2);
  }catch(e){
    rd.className='err';
    out.textContent='Network error: '+e;
  }finally{
    btn.disabled=false;
  }
}
init();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# Request / response models
# ---------------------------------------------------------------------------

class PickRequest(BaseModel):
    """Body for POST /skill/pick.

    Supply either ``z`` (raw gripper Z) or ``cup_bottom_z`` (cup-bottom
    centre Z, converted server-side using ``cup_grip_z_offset``).
    """

    x: float
    y: float
    z: float | None = None
    cup_bottom_z: float | None = None
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
# FastAPI app — _runtime and _cup_grip_z_offset injected before uvicorn starts
# ---------------------------------------------------------------------------

app = FastAPI(title="CupStack Skill API")
_runtime: CupStackRuntime | None = None
_lock = threading.Lock()
_cup_grip_z_offset: float = SkillStackConfig().cup_grip_z_offset


def _check_busy() -> None:
    if not _lock.acquire(blocking=False):
        raise HTTPException(
            status_code=409, detail="a skill is already running"
        )


@app.get("/", response_class=HTMLResponse)
def frontend() -> str:
    """Serve the pick skill HTML frontend."""

    return _FRONTEND_HTML


@app.get("/status")
def status() -> dict:
    """Return server liveness, busy state, and cup_grip_z_offset."""

    return {
        "ready": _runtime is not None,
        "busy": _lock.locked(),
        "cup_grip_z_offset": _cup_grip_z_offset,
    }


@app.post("/skill/pick", response_model=SkillResponse)
def skill_pick(req: PickRequest) -> SkillResponse:
    """Pick a cup from the given coordinate.

    Accepts either ``z`` (raw gripper Z) or ``cup_bottom_z`` (cup-bottom
    centre Z).  When ``cup_bottom_z`` is given the actual gripper Z is
    ``cup_bottom_z + cup_grip_z_offset`` (configurable node parameter).
    """

    if req.z is None and req.cup_bottom_z is None:
        raise HTTPException(
            status_code=422,
            detail="provide 'z' (gripper Z) or 'cup_bottom_z'",
        )
    pick_z = (
        req.z
        if req.z is not None
        else req.cup_bottom_z + _cup_grip_z_offset
    )
    _check_busy()
    try:
        skill = PickCupSkill(_runtime, req.x, req.y, pick_z, ori=req.ori)
        ok = skill.execute()
        return SkillResponse(
            success=ok, skill="pick",
            detail=f"gripper_z={pick_z:.4f}",
        )
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
    node.declare_parameter(
        "cup_grip_z_offset", SkillStackConfig().cup_grip_z_offset
    )

    global _runtime, _cup_grip_z_offset
    try:
        host = str(node.get_parameter("host").value)
        port = int(node.get_parameter("port").value)
        move_home = bool(node.get_parameter("move_home").value)
        _cup_grip_z_offset = float(
            node.get_parameter("cup_grip_z_offset").value
        )
        log = node.get_logger()
        log.info(f"cup_grip_z_offset={_cup_grip_z_offset:.4f} m")

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
