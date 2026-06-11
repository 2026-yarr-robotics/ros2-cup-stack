"""HTTP API server node: expose cup skills as REST endpoints.

Runs a FastAPI/uvicorn server in a daemon thread alongside the ROS 2
/ MoveItPy runtime.  Only one skill executes at a time; concurrent
requests receive ``409 Conflict``.

Endpoints
---------
GET  /             -- pick frontend (HTML)
GET  /status       -- liveness, busy, and cup_grip_z_offset
POST /skill/pick   -- pick a cup; accepts gripper Z or cup-top Z
POST /skill/pyramid -- run the full 6-cup pyramid sequence
POST /skill/scan   -- launch the existing 2-direction scan node
POST /skill/scan_square -- launch the 4-corner square scan node
"""

import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
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
from cup_stack.skills.config import DOWN_ORI, SkillStackConfig
from cup_stack.skills.pick_cup_skill import PickCupSkill
from cup_stack.skills.place_cup_at import PlaceCupAtSkill, PlaceSpec
from cup_stack.skills.pyramid_plan import PyramidStackPlan, SourceStack
from cup_stack.skills.base import PickSpec
from cup_stack.tasks.scan import ScanTask
from cup_stack.tasks.scan_square import ScanSquareTask

# Relative under /dsr01; resolves to /dsr01/dsr_moveit_controller/...
_FJT_ACTION_NAME = "dsr_moveit_controller/follow_joint_trajectory"


# ---------------------------------------------------------------------------
# Pick frontend (served at GET /)
# ---------------------------------------------------------------------------

_FRONTEND_HTML = """\
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<title>CupStack Skill CLI</title>
<style>
*{box-sizing:border-box}
body{
  background:#1e1e1e;color:#d4d4d4;
  font-family:'Courier New',monospace;
  margin:0;padding:24px;min-height:100vh}
h2{color:#9cdcfe;margin:0 0 4px}
p.sub{margin:0 0 16px;color:#6a9955;font-size:.85em}
#help{
  background:#252526;border:1px solid #3c3c3c;
  border-radius:6px;padding:12px 16px;
  margin-bottom:16px;font-size:.82em;line-height:1.8;color:#ce9178}
#help b{color:#dcdcaa}
#log{
  background:#252526;border:1px solid #3c3c3c;
  border-radius:6px;padding:12px 16px;
  min-height:200px;max-height:420px;overflow-y:auto;
  margin-bottom:12px;font-size:.85em;line-height:1.6}
.line-cmd{color:#9cdcfe}
.line-ok{color:#4ec9b0}
.line-err{color:#f44747}
.line-busy{color:#dcdcaa}
.line-info{color:#6a9955}
.line-hint{color:#808080}
#input-row{display:flex;gap:8px}
#cmd{
  flex:1;background:#3c3c3c;color:#d4d4d4;
  border:1px solid #555;border-radius:4px;
  padding:8px 12px;font-family:inherit;font-size:.95em}
#cmd:focus{outline:none;border-color:#007acc}
#run{
  background:#0e639c;color:#fff;border:none;
  border-radius:4px;padding:8px 18px;
  font-family:inherit;font-size:.95em;cursor:pointer}
#run:hover{background:#1177bb}
#run:disabled{background:#555;cursor:default}
</style>
</head>
<body>
<h2>CupStack Skill CLI</h2>
<p class="sub"># skill API command terminal</p>
<div id="help">
<b>pick</b>  x  y  z_top            &nbsp;— 컵 윗면 Z 기준 pick<br>
<b>pick</b>  x  y  --z  z_gripper    &nbsp;— 그리퍼 Z 직접 지정<br>
<b>status</b>                         &nbsp;— 서버 상태 / offset 확인<br>
<b>scan</b>                           &nbsp;— 스캔 실행<br>
<b>help</b>                           &nbsp;— 이 도움말
</div>
<div id="log"></div>
<div id="input-row">
  <input id="cmd" placeholder="pick 0.40 0.00 0.10" autofocus
         onkeydown="if(event.key==='Enter')exec()">
  <button id="run" onclick="exec()">Run</button>
</div>
<script>
let _offset=null;
const log=document.getElementById('log');
const inp=document.getElementById('cmd');
const btn=document.getElementById('run');
const hist=[];let hIdx=-1;

function print(text,cls){
  const d=document.createElement('div');
  d.className='line-'+cls;
  d.textContent=text;
  log.appendChild(d);
  log.scrollTop=log.scrollHeight;
}

async function init(){
  try{
    const j=await(await fetch('/status')).json();
    _offset=j.cup_grip_z_offset??null;
    print(
      '# server ready  cup_grip_z_offset='
      +(_offset!==null?_offset.toFixed(3)+'m':'?'),
      'info'
    );
    print('# type help for usage','hint');
  }catch(e){
    print('# server unreachable: '+e,'err');
  }
}

function parseArgs(tokens){
  const kv={};const pos=[];
  for(let i=0;i<tokens.length;i++){
    if(tokens[i].startsWith('--')){
      kv[tokens[i].slice(2)]=tokens[++i];
    }else{pos.push(tokens[i]);}
  }
  return{pos,kv};
}

async function exec(){
  const raw=inp.value.trim();
  if(!raw)return;
  hist.unshift(raw);hIdx=-1;
  inp.value='';
  print('> '+raw,'cmd');
  btn.disabled=true;

  const tokens=raw.split(/\s+/);
  const cmd=tokens[0].toLowerCase();
  const rest=tokens.slice(1);

  try{
    if(cmd==='help'){
      print('pick x y z_top | pick x y --z z_gripper','hint');
      print('status | scan','hint');
    }else if(cmd==='status'){
      const j=await(await fetch('/status')).json();
      _offset=j.cup_grip_z_offset??_offset;
      print(JSON.stringify(j,null,2),'ok');
    }else if(cmd==='scan'){
      print('running scan...','hint');
      const j=await(await fetch('/skill/scan',{
        method:'POST',headers:{'Content-Type':'application/json'},
        body:'{}'})).json();
      print(JSON.stringify(j,null,2),j.success?'ok':'err');
    }else if(cmd==='pick'){
      const{pos,kv}=parseArgs(rest);
      if(pos.length<2){
        print('usage: pick x y z_top  |  pick x y --z z_gripper','err');
      }else{
        const x=parseFloat(pos[0]);
        const y=parseFloat(pos[1]);
        const body={x,y};
        if('z'in kv){
          body.z=parseFloat(kv.z);
          print(
            'gripper_z='+body.z.toFixed(4),'hint');
        }else if(pos.length>=3){
          body.cup_top_z=parseFloat(pos[2]);
          const gz=body.cup_top_z+(_offset??0);
          print(
            'cup_top_z='+body.cup_top_z.toFixed(4)
            +' + offset='+(_offset??'?')
            +' → gripper_z='+gz.toFixed(4),'hint');
        }else{
          print('provide z_top or --z z_gripper','err');
          btn.disabled=false;return;
        }
        print('running pick...','hint');
        const j=await(await fetch('/skill/pick',{
          method:'POST',
          headers:{'Content-Type':'application/json'},
          body:JSON.stringify(body)})).json();
        print(JSON.stringify(j,null,2),j.success?'ok':'err');
      }
    }else{
      print('unknown command: '+cmd+' (try help)','err');
    }
  }catch(e){
    print('error: '+e,'err');
  }finally{
    btn.disabled=false;
    inp.focus();
  }
}

inp.addEventListener('keydown',e=>{
  if(e.key==='ArrowUp'){
    hIdx=Math.min(hIdx+1,hist.length-1);
    inp.value=hist[hIdx]??'';e.preventDefault();
  }else if(e.key==='ArrowDown'){
    hIdx=Math.max(hIdx-1,-1);
    inp.value=hIdx<0?'':hist[hIdx];e.preventDefault();
  }
});

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

    Supply one of ``z`` (raw gripper Z), ``cup_top_z`` (cup-top centre
    Z, converted to gripper Z via ``+ cup_grip_z_offset``), or
    ``nested_count`` (server derives gripper Z from
    ``pick_z_base + (nested_count - 1) * nest_inc``).
    """

    x: float
    y: float
    z: float | None = None
    cup_top_z: float | None = None
    nested_count: int | None = None
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


class PyramidStepRequest(BaseModel):
    """Body for POST /skill/pyramid_step — one cup, absolute place pose.

    Server-side state (pyramid center, yaw, pick_z) is resolved by the
    HTTP caller (cup-stack-server), so this endpoint receives only
    fully-resolved absolute coordinates.  ``slot`` is a logging tag only.
    """

    x: float
    y: float
    pick_z: float
    place_x: float
    place_y: float
    place_z: float
    slot: str = ""
    ori: dict | None = None


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
# Geometry used when /skill/pick is called with ``nested_count`` instead of
# an explicit Z.  Defaults mirror SkillStackConfig + cup_pyramid.launch.py.
_pick_z_base: float = SkillStackConfig().pick_z_base
_nest_inc: float = 0.012
# Set True once dsr_moveit_controller/follow_joint_trajectory action server
# is reachable.  /status's ``ready`` flag and skill endpoints gate on this so
# the spawner-vs-skill_api startup race no longer leaks ABORTED picks.
_controller_ready: bool = False


def _check_busy() -> None:
    if not _lock.acquire(blocking=False):
        raise HTTPException(
            status_code=409, detail="a skill is already running"
        )


def _require_ready() -> None:
    if _runtime is None or not _controller_ready:
        raise HTTPException(
            status_code=503,
            detail=(
                "skill_api not ready: "
                f"runtime={'ok' if _runtime is not None else 'pending'}, "
                f"controller={'ok' if _controller_ready else 'pending'}"
            ),
        )


@app.get("/", response_class=HTMLResponse)
def frontend() -> str:
    """Serve the pick skill HTML frontend."""

    return _FRONTEND_HTML


@app.get("/status")
def status() -> dict:
    """Return server liveness, busy state, and pick-Z geometry."""

    return {
        "ready": _runtime is not None and _controller_ready,
        "runtime_ready": _runtime is not None,
        "controller_ready": _controller_ready,
        "busy": _lock.locked(),
        "cup_grip_z_offset": _cup_grip_z_offset,
        "pick_z_base": _pick_z_base,
        "nest_inc": _nest_inc,
    }


def _resolve_pick_z(req: "PickRequest") -> tuple[float, str]:
    """Pick precedence: ``z`` > ``cup_top_z`` > ``nested_count``."""

    if req.z is not None:
        return req.z, f"gripper_z={req.z:.4f}"
    if req.cup_top_z is not None:
        gz = req.cup_top_z + _cup_grip_z_offset
        return gz, f"cup_top_z={req.cup_top_z:.4f} → gripper_z={gz:.4f}"
    if req.nested_count is not None:
        if req.nested_count < 1:
            raise HTTPException(
                status_code=422, detail="'nested_count' must be >= 1"
            )
        gz = _pick_z_base + (req.nested_count - 1) * _nest_inc
        return gz, f"nested_count={req.nested_count} → gripper_z={gz:.4f}"
    raise HTTPException(
        status_code=422,
        detail="provide 'z', 'cup_top_z', or 'nested_count'",
    )


@app.post("/skill/pick", response_model=SkillResponse)
def skill_pick(req: PickRequest) -> SkillResponse:
    """Pick a cup from the given coordinate.

    Accepts ``z`` (raw gripper Z), ``cup_top_z`` (cup-top centre Z,
    converted via ``+ cup_grip_z_offset``), or ``nested_count``
    (gripper Z = ``pick_z_base + (nested_count - 1) * nest_inc``).
    """

    pick_z, detail = _resolve_pick_z(req)
    _require_ready()
    _check_busy()
    try:
        skill = PickCupSkill(_runtime, req.x, req.y, pick_z, ori=req.ori)
        ok = skill.execute()
        return SkillResponse(success=ok, skill="pick", detail=detail)
    finally:
        _lock.release()


@app.post("/skill/pyramid", response_model=SkillResponse)
def skill_pyramid(req: PyramidRequest) -> SkillResponse:
    """Run the full 6-cup 3-2-1 pyramid sequence."""

    _require_ready()
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


@app.post("/skill/pyramid_step", response_model=SkillResponse)
def skill_pyramid_step(req: PyramidStepRequest) -> SkillResponse:
    """Pick one cup and place it at a server-supplied absolute XYZ.

    The HTTP caller (cup-stack-server) holds the pyramid config
    (center, yaw, pick_z) and computes the absolute place pose for the
    requested slot; this endpoint runs one PlaceCupAtSkill.

    The response is sent as soon as the cup is **released at its place
    pose** (step 8) — not after the final lift — so the caller gets the
    completion status at place time.  The lift keeps running in a
    background thread; ``busy`` stays ``true`` (and concurrent skill
    requests get 409) until the whole motion finishes.
    """

    _require_ready()
    _check_busy()

    detail = (
        f"slot={req.slot or '?'} "
        f"pick=({req.x:.3f},{req.y:.3f},{req.pick_z:.3f}) "
        f"place=({req.place_x:.3f},{req.place_y:.3f},{req.place_z:.3f})"
    )

    placed = threading.Event()
    finished = threading.Event()
    outcome = {"ok": False, "error": ""}

    def _run() -> None:
        try:
            pick = PickSpec(
                x=req.x, y=req.y, z=req.pick_z, ori=req.ori or DOWN_ORI,
            )
            place = PlaceSpec(
                x=req.place_x, y=req.place_y, z=req.place_z,
                name=req.slot or "pyramid_step",
            )
            skill = PlaceCupAtSkill(_runtime, place)
            ok = skill.execute(pick, on_placed=placed.set)
            outcome["ok"] = ok
            # After a successful place, return the arm to HOME (the same joint
            # HOME as the startup move_home) so the exo camera sees the placed
            # cup — not the arm hovering over it — before the caller reads the
            # world. Without this the verifier never marks the slot occupied and
            # the LLM loop stalls on "pending world update". Best-effort: a home
            # failure is logged but still reported as a successful place.
            if ok and not _runtime.try_move_home(z_offset_m=0.08, x_offset_m=-0.05):
                _runtime.logger.warn("post-place lifted move_home failed (continuing)")
        except Exception as exc:  # noqa: BLE001 - report via response
            outcome["error"] = str(exc)
            _runtime.logger.error(f"pyramid_step failed: {exc}")
        finally:
            finished.set()
            _lock.release()

    threading.Thread(target=_run, daemon=True).start()

    # Reply only after the FULL motion finishes: place + lift + return HOME.
    # (Previously replied at release with the lift running async, but the caller
    # reads the world immediately after the reply, so the arm must already be
    # clear of the exo camera by then.) Runs synchronously → call this endpoint
    # over localhost, not the Cloudflare tunnel (which 504s past ~60s).
    finished.wait()
    err = f" error={outcome['error']}" if outcome["error"] else ""
    suffix = " (placed + homed)" if outcome["ok"] else err
    return SkillResponse(
        success=outcome["ok"], skill="pyramid",
        detail=f"{detail}{suffix}",
    )


@app.post("/skill/scan", response_model=SkillResponse)
def skill_scan() -> SkillResponse:
    """Run the 2-direction scan reusing skill_api's own MoveItPy runtime.

    Was launching scan.launch.py as a subprocess, which spun up a SECOND
    MoveItPy in the same /<ns>; its planning-scene-monitor collided with this
    node's live MoveItPy and the scan node died at init ("Unable to configure
    planning scene monitor"). Running ScanTask against the existing _runtime
    avoids the contention entirely.
    """

    _require_ready()
    _check_busy()
    try:
        ok = ScanTask(_runtime).try_execute()
        return SkillResponse(success=ok, skill="scan")
    finally:
        _lock.release()


@app.post("/skill/scan_square", response_model=SkillResponse)
def skill_scan_square() -> SkillResponse:
    """Run the 4-corner square scan reusing skill_api's own MoveItPy runtime.

    Camera stays fixed downward; the EE traces an axis-aligned rectangle in
    XY at the HOME EE height, then returns to the start pose. Like /skill/scan,
    this reuses _runtime instead of launching a second MoveItPy.
    """

    _require_ready()
    _check_busy()
    try:
        ok = ScanSquareTask(_runtime).try_execute()
        return SkillResponse(success=ok, skill="scan_square")
    finally:
        _lock.release()


# ---------------------------------------------------------------------------
# ROS 2 node entry point
# ---------------------------------------------------------------------------

def _watch_controller_ready(node: Node) -> None:
    """Set ``_controller_ready`` once the FollowJointTrajectory server appears.

    The spawner that loads ``dsr_moveit_controller`` runs in parallel with
    skill_api_node and lags MoveItPy init by ~10 s.  Until the action server
    is up, any pick MoveIt sends out aborts with
    ``Action client not connected to action server``.  Poll on a short
    timeout so /status reflects controller availability in near-real-time.
    """
    global _controller_ready
    client = ActionClient(node, FollowJointTrajectory, _FJT_ACTION_NAME)
    log = node.get_logger()
    log.info(f"waiting for {_FJT_ACTION_NAME} action server")
    try:
        while rclpy.ok():
            if client.wait_for_server(timeout_sec=1.0):
                _controller_ready = True
                log.info(
                    f"{_FJT_ACTION_NAME} is up; skill_api now accepting picks"
                )
                return
    finally:
        client.destroy()


def main(args=None) -> None:
    """Run the skill API server node."""

    global _runtime, _cup_grip_z_offset, _pick_z_base, _nest_inc
    rclpy.init(args=args)
    node = Node("skill_api_node")
    node.declare_parameter("host", "0.0.0.0")
    node.declare_parameter("port", 8765)
    node.declare_parameter("move_home", False)
    cfg = SkillStackConfig()
    node.declare_parameter("cup_grip_z_offset", cfg.cup_grip_z_offset)
    node.declare_parameter("pick_z_base", cfg.pick_z_base)
    node.declare_parameter("nest_inc", _nest_inc)
    try:
        host = str(node.get_parameter("host").value)
        port = int(node.get_parameter("port").value)
        move_home = bool(node.get_parameter("move_home").value)
        _cup_grip_z_offset = float(
            node.get_parameter("cup_grip_z_offset").value
        )
        _pick_z_base = float(node.get_parameter("pick_z_base").value)
        _nest_inc = float(node.get_parameter("nest_inc").value)
        log = node.get_logger()
        log.info(
            f"cup_grip_z_offset={_cup_grip_z_offset:.4f} m  "
            f"pick_z_base={_pick_z_base:.4f} m  "
            f"nest_inc={_nest_inc:.4f} m"
        )

        _runtime = CupStackRuntime(
            node,
            "skill_api_moveit_py",
            moveit_namespace=node.get_namespace(),
        )

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
        threading.Thread(
            target=_watch_controller_ready,
            args=(node,),
            daemon=True,
        ).start()

        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
