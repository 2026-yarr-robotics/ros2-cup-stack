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
<b>pick</b>  x  y  z_bottom          &nbsp;— 컵 바닥 Z 기준 pick<br>
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
      print('pick x y z_bottom | pick x y --z z_gripper','hint');
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
        print('usage: pick x y z_bottom  |  pick x y --z z_gripper','err');
      }else{
        const x=parseFloat(pos[0]);
        const y=parseFloat(pos[1]);
        const body={x,y};
        if('z'in kv){
          body.z=parseFloat(kv.z);
          print(
            'gripper_z='+body.z.toFixed(4),'hint');
        }else if(pos.length>=3){
          body.cup_bottom_z=parseFloat(pos[2]);
          const gz=body.cup_bottom_z+(_offset??0);
          print(
            'cup_bottom_z='+body.cup_bottom_z.toFixed(4)
            +' + offset='+(_offset??'?')
            +' → gripper_z='+gz.toFixed(4),'hint');
        }else{
          print('provide z_bottom or --z z_gripper','err');
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
