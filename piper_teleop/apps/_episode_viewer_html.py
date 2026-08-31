"""Self-contained HTML template for the offline 3D episode viewer.

Placeholders filled by apps.viz_episode: __TITLE__ and __DATA_JSON__ (a JSON
object with the trajectory, workspace box and precomputed stats). Everything
else — orbit camera, playback, triads, base plane grid — is plain JS on a 2D
canvas so the file opens anywhere with zero dependencies.
"""

TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>__TITLE__</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin:0; background:#101014; color:#e6e6ea; font:13px/1.45 system-ui,-apple-system,'Segoe UI',sans-serif; height:100vh; display:flex; flex-direction:column; overflow:hidden; }
  header { padding:8px 14px; border-bottom:1px solid #2a2a30; background:#16161c; display:flex; gap:14px; align-items:baseline; flex:none; }
  header h1 { margin:0; font-size:15px; font-weight:600; }
  header .sub { color:#9a9aa5; font-size:12px; }
  .wrap { flex:1; display:flex; min-height:0; }
  .main { flex:1; display:flex; flex-direction:column; min-width:0; }
  #view { flex:1; min-height:0; cursor:grab; touch-action:none; }
  #view.dragging { cursor:grabbing; }
  #strip { flex:none; height:72px; border-top:1px solid #2a2a30; cursor:crosshair; }
  aside { width:340px; flex:none; border-left:1px solid #2a2a30; background:#16161c; overflow-y:auto; padding:10px 12px; }
  .group { border-bottom:1px solid #26262c; padding:8px 0; }
  .group h2 { margin:0 0 6px; font-size:11px; text-transform:uppercase; letter-spacing:.08em; color:#8f8f9a; }
  .row { display:flex; align-items:center; gap:8px; margin:5px 0; flex-wrap:wrap; }
  .row label { color:#b9b9c2; }
  input[type=range] { flex:1; min-width:60px; accent-color:#6ab0ff; }
  input[type=number] { width:68px; background:#0d0d11; color:#e6e6ea; border:1px solid #33333b; border-radius:4px; padding:3px 5px; }
  button { background:#26262e; color:#e6e6ea; border:1px solid #3a3a44; border-radius:4px; padding:4px 10px; cursor:pointer; }
  button:hover { border-color:#6ab0ff; }
  button.primary { background:#2b4a6f; border-color:#3d6ea5; }
  label.chk { display:inline-flex; align-items:center; gap:5px; color:#b9b9c2; margin-right:6px; cursor:pointer; }
  .mono, code { font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace; font-size:12px; }
  #readout { white-space:pre; overflow-x:auto; }
  #stats { white-space:pre-wrap; color:#9a9aa5; font-size:11px; }
  #cmd { display:block; background:#0d0d11; border:1px solid #2c2c34; border-radius:4px; padding:6px; margin-top:4px; color:#a9d1ff; word-break:break-all; white-space:pre-wrap; user-select:all; }
  .hint { color:#77777f; font-size:11px; }
</style>
</head>
<body>
<header><h1>__TITLE__</h1><span class="sub" id="subinfo"></span></header>
<div class="wrap">
  <div class="main">
    <canvas id="view"></canvas>
    <canvas id="strip"></canvas>
  </div>
  <aside>
    <div class="group">
      <div class="row">
        <button id="play" class="primary">&#9654; Play</button>
        <span class="mono" id="frameinfo"></span>
      </div>
      <div class="row"><input id="frame" type="range" min="0" max="0" value="0" step="1" style="width:100%"></div>
      <div class="row"><label>Speed</label><input id="speed" type="range" min="0.1" max="4" step="0.1" value="1"><span class="mono" id="speedv">1.0x</span></div>
    </div>
    <div class="group">
      <h2>View</h2>
      <div class="row">
        <button data-view="iso">Iso</button><button data-view="top">Top</button>
        <button data-view="front">Front</button><button data-view="side">Side</button>
        <button id="fit">Fit</button>
      </div>
      <div class="row" id="showArms"></div>
      <div class="row">
        <label class="chk"><input type="checkbox" id="tTriads" checked>triads</label>
        <label class="chk"><input type="checkbox" id="tObs">obs trail</label>
        <label class="chk"><input type="checkbox" id="tGrid" checked>base plane</label>
        <label class="chk"><input type="checkbox" id="tBox" checked>workspace box</label>
      </div>
      <div class="row">
        <label class="chk"><input type="checkbox" id="tJumps" checked>jump marks</label>
        <label class="chk"><input type="checkbox" id="tOutside" checked>outside-ws marks</label>
        <label class="chk"><input type="checkbox" id="tDrop" checked>drop line</label>
      </div>
      <div class="row"><label>Triad every</label><input id="stride" type="number" min="1" step="1"><label>frames, size</label><input id="axscale" type="range" min="0.4" max="3" step="0.1" value="1" style="width:64px"></div>
      <div class="hint">drag orbit &middot; shift/right-drag pan &middot; wheel zoom &middot; dbl-click fit &middot; space play &middot; &larr;/&rarr; step</div>
    </div>
    <div class="group">
      <h2>Offset / workspace preview</h2>
      <div class="row">
        <label>dx</label><input id="dx" type="number" step="0.005">
        <label>dy</label><input id="dy" type="number" step="0.005">
        <label>dz</label><input id="dz" type="number" step="0.005">
      </div>
      <div class="row">
        <button id="suggest">Suggest offset</button>
        <button id="zero">Zero</button>
        <label class="chk"><input type="checkbox" id="tClamp">clamp preview</label>
      </div>
      <div class="row mono" id="clampinfo"></div>
      <code id="cmd" class="mono"></code>
    </div>
    <div class="group">
      <h2>Current frame</h2>
      <div id="readout" class="mono"></div>
    </div>
    <div class="group">
      <h2>Data checks</h2>
      <div id="stats" class="mono"></div>
    </div>
  </aside>
</div>
<script>
'use strict';
const DATA = __DATA_JSON__;
const TS = DATA.ts, N = TS.length, FPS = DATA.fps;
const SIDES = Object.keys(DATA.arms);
const WS = DATA.ws;
const HUE = { right:[205,330], left:[60,150], arm:[205,330],
              cmd:[205,330], meas:[20,55] };
const el = id => document.getElementById(id);

// ---- per-arm state ---------------------------------------------------------
const arms = {};
for (const s of SIDES) {
  const a = DATA.arms[s];
  arms[s] = { P:a.ap, Q:a.aq, OP:a.op, OQ:a.oq, G:a.g, GW:a.gw,
              obsEq:a.obs_eq, jumps:a.jumps, off:a.off,
              T:null, OT:null, outside:null, nOut:0, show:true, segColor:[] };
  const h = HUE[s] || HUE.arm;
  for (let i = 0; i < N; i++)
    arms[s].segColor.push('hsl(' + (h[0] + (h[1]-h[0]) * i / Math.max(N-1,1)).toFixed(1) + ',85%,62%)');
}

let bmin = [1e9,1e9,1e9], bmax = [-1e9,-1e9,-1e9];
function accum(p){ for (let k=0;k<3;k++){ if(p[k]<bmin[k])bmin[k]=p[k]; if(p[k]>bmax[k])bmax[k]=p[k]; } }
for (const s of SIDES) for (const p of arms[s].P) accum(p);
accum([0,0,0]);
const DIAG = Math.hypot(bmax[0]-bmin[0], bmax[1]-bmin[1], bmax[2]-bmin[2]) || 0.3;
const axLen = Math.min(Math.max(0.09*DIAG, 0.015), 0.1);

// ---- rotation helpers (same conventions as piper_teleop.xr_math) -----------
function quatMat(q){ // wxyz -> 3x3, columns are the x/y/z tool axes
  const w=q[0], x=q[1], y=q[2], z=q[3];
  const n = w*w+x*x+y*y+z*z;
  if (n < 1e-12) return [[1,0,0],[0,1,0],[0,0,1]];
  const s = 2/n;
  const xx=x*x*s, yy=y*y*s, zz=z*z*s, xy=x*y*s, xz=x*z*s, yz=y*z*s, wx=w*x*s, wy=w*y*s, wz=w*z*s;
  return [[1-(yy+zz), xy-wz, xz+wy],[xy+wz, 1-(xx+zz), yz-wx],[xz-wy, yz+wx, 1-(xx+yy)]];
}
function rpyDeg(m){ // ZYX (R = Rz Ry Rx), pitch in [-90, 90]
  const pitch = Math.asin(Math.min(1, Math.max(-1, -m[2][0])));
  let roll, yaw;
  if (Math.abs(m[2][0]) < 0.9999999) { roll = Math.atan2(m[2][1], m[2][2]); yaw = Math.atan2(m[1][0], m[0][0]); }
  else { roll = 0; yaw = Math.atan2(-m[0][1], m[1][1]); }
  const d = 180/Math.PI;
  return [roll*d, pitch*d, yaw*d];
}

// ---- offset / clamp preview ------------------------------------------------
function getOffset(){ return [parseFloat(el('dx').value)||0, parseFloat(el('dy').value)||0, parseFloat(el('dz').value)||0]; }
function updateTransform(){
  const off = getOffset(), clampOn = el('tClamp').checked && WS;
  for (const s of SIDES) {
    const A = arms[s];
    A.T = new Array(N); A.OT = new Array(N); A.outside = new Array(N); A.nOut = 0;
    for (let i = 0; i < N; i++) {
      const p = A.P[i];
      let t = [p[0]+off[0], p[1]+off[1], p[2]+off[2]];
      let out = false;
      if (WS) for (let k = 0; k < 3; k++)
        if (t[k] < WS.min[k]-1e-9 || t[k] > WS.max[k]+1e-9) { out = true; break; }
      A.outside[i] = out; if (out) A.nOut++;
      if (clampOn) for (let k = 0; k < 3; k++) t[k] = Math.min(Math.max(t[k], WS.min[k]), WS.max[k]);
      A.T[i] = t;
      const o = A.OP[i];
      A.OT[i] = [o[0]+off[0], o[1]+off[1], o[2]+off[2]];
    }
  }
  if (WS) {
    const info = SIDES.filter(s => arms[s].show).map(s =>
      s + ': ' + arms[s].nOut + '/' + N + ' (' + (100*arms[s].nOut/N).toFixed(0) + '%) outside ws'
        + (el('tClamp').checked && arms[s].nOut ? ' -> clamped' : ''));
    el('clampinfo').textContent = info.join('   ');
  }
  updateCmd();
}
function updateCmd(){
  const off = getOffset();
  let cmd = 'piper-replay --root ' + DATA.root + ' --episode ' + DATA.episode;
  if (DATA.schema === 'modality') {
    cmd += ' --arm-side ' + SIDES[0] + ' --absolute';
    if (off.some(v => Math.abs(v) > 1e-9)) cmd += ' --offset ' + off.map(v => v.toFixed(3)).join(',');
  }
  el('cmd').textContent = cmd;
}

// ---- camera ----------------------------------------------------------------
const canvas = el('view'), ctx = canvas.getContext('2d');
const strip = el('strip'), sctx = strip.getContext('2d');
let W = 0, H = 0, SW = 0, SH = 0;
function resize(){
  const dpr = window.devicePixelRatio || 1;
  W = canvas.clientWidth; H = canvas.clientHeight;
  canvas.width = Math.max(1, Math.round(W*dpr)); canvas.height = Math.max(1, Math.round(H*dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  SW = strip.clientWidth; SH = strip.clientHeight;
  strip.width = Math.max(1, Math.round(SW*dpr)); strip.height = Math.max(1, Math.round(SH*dpr));
  sctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
const ro = new ResizeObserver(resize); ro.observe(canvas); ro.observe(strip);

const cam = { yaw:-45*Math.PI/180, pitch:27*Math.PI/180, dist:1.0, target:[0,0,0] };
let eye, fwd, right, upv, focal;
function computeBasis(){
  const cy=Math.cos(cam.yaw), sy=Math.sin(cam.yaw), cp=Math.cos(cam.pitch), sp=Math.sin(cam.pitch);
  eye = [cam.target[0]+cam.dist*cp*cy, cam.target[1]+cam.dist*cp*sy, cam.target[2]+cam.dist*sp];
  fwd = [-cp*cy, -cp*sy, -sp];
  let rx = fwd[1], ry = -fwd[0];               // cross(fwd, z-up)
  let rn = Math.hypot(rx, ry);
  if (rn < 1e-9) { rx = -sy; ry = cy; rn = 1; }
  right = [rx/rn, ry/rn, 0];
  upv = [right[1]*fwd[2], -right[0]*fwd[2], right[0]*fwd[1] - right[1]*fwd[0]];
  focal = 1.3 * Math.min(W, H);
}
function proj(p){
  const dx = p[0]-eye[0], dy = p[1]-eye[1], dz = p[2]-eye[2];
  const z = dx*fwd[0] + dy*fwd[1] + dz*fwd[2];
  if (z < 1e-4) return null;
  return [W/2 + (dx*right[0]+dy*right[1]+dz*right[2])/z*focal,
          H/2 - (dx*upv[0]+dy*upv[1]+dz*upv[2])/z*focal, z];
}
function fitView(){
  let mn = [1e9,1e9,1e9], mx = [-1e9,-1e9,-1e9];
  for (const s of SIDES) { if (!arms[s].show) continue;
    for (const p of arms[s].T) for (let k = 0; k < 3; k++) {
      if (p[k] < mn[k]) mn[k] = p[k];
      if (p[k] > mx[k]) mx[k] = p[k];
    }
  }
  if (mn[0] > mx[0]) { mn = [-0.2,-0.2,0]; mx = [0.4,0.2,0.4]; }
  for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], 0); mx[k] = Math.max(mx[k], 0); }
  cam.target = [(mn[0]+mx[0])/2, (mn[1]+mx[1])/2, (mn[2]+mx[2])/2];
  const r = Math.hypot(mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2])/2 + axLen*2;
  cam.dist = Math.max(0.25, 3.0*r);
}
const VIEWS = { iso:[-45,27], top:[-90,89], front:[0,8], side:[-90,8] };

// ---- draw helpers ----------------------------------------------------------
function line3(a, b, color, width, alpha, dash){
  const pa = proj(a), pb = proj(b); if (!pa || !pb) return;
  ctx.globalAlpha = alpha === undefined ? 1 : alpha;
  ctx.strokeStyle = color; ctx.lineWidth = width || 1;
  if (dash) ctx.setLineDash(dash);
  ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); ctx.stroke();
  if (dash) ctx.setLineDash([]);
  ctx.globalAlpha = 1;
}
function dot3(p, color, r){
  const q = proj(p); if (!q) return null;
  ctx.fillStyle = color; ctx.beginPath(); ctx.arc(q[0], q[1], r, 0, 6.2832); ctx.fill();
  return q;
}
function text3(p, str, color){
  const q = proj(p); if (!q) return;
  ctx.fillStyle = color; ctx.fillText(str, q[0]+4, q[1]-4);
}

// grid extent: cover trajectory, origin and the workspace box, on 0.1 m ticks
const gpad = 0.15;
let gx0 = Math.min(bmin[0], 0) - gpad, gx1 = Math.max(bmax[0], 0) + gpad;
let gy0 = Math.min(bmin[1], 0) - gpad, gy1 = Math.max(bmax[1], 0) + gpad;
if (WS) { gx0 = Math.min(gx0, WS.min[0]-0.05); gx1 = Math.max(gx1, WS.max[0]+0.05);
          gy0 = Math.min(gy0, WS.min[1]-0.05); gy1 = Math.max(gy1, WS.max[1]+0.05); }
gx0 = Math.floor(gx0*10)/10; gx1 = Math.ceil(gx1*10)/10;
gy0 = Math.floor(gy0*10)/10; gy1 = Math.ceil(gy1*10)/10;

function drawGrid(){
  if (!el('tGrid').checked) return;
  const corners = [[gx0,gy0,0],[gx1,gy0,0],[gx1,gy1,0],[gx0,gy1,0]].map(proj);
  if (corners.every(c => c)) {
    ctx.globalAlpha = 0.08; ctx.fillStyle = '#5a7fb0';
    ctx.beginPath(); ctx.moveTo(corners[0][0], corners[0][1]);
    for (let i = 1; i < 4; i++) ctx.lineTo(corners[i][0], corners[i][1]);
    ctx.closePath(); ctx.fill(); ctx.globalAlpha = 1;
  }
  const step = 0.05;
  for (let i = 0; i <= Math.round((gx1-gx0)/step); i++) {
    const x = gx0 + i*step;
    const major = Math.abs(x - Math.round(x/0.25)*0.25) < 1e-9;
    line3([x,gy0,0], [x,gy1,0], major ? '#4a4a55' : '#33333c', 1, major ? 0.8 : 0.45);
  }
  for (let i = 0; i <= Math.round((gy1-gy0)/step); i++) {
    const y = gy0 + i*step;
    const major = Math.abs(y - Math.round(y/0.25)*0.25) < 1e-9;
    line3([gx0,y,0], [gx1,y,0], major ? '#4a4a55' : '#33333c', 1, major ? 0.8 : 0.45);
  }
  const ax = Math.max(gx1, 0.2), ay = Math.max(gy1, 0.2);
  line3([0,0,0], [ax,0,0], '#e35d5d', 2, 0.9);
  line3([0,0,0], [0,ay,0], '#57c957', 2, 0.9);
  line3([0,0,0], [0,0,0.15], '#6a8dff', 2, 0.9);
  text3([ax,0,0], '+x', '#e35d5d');
  text3([0,ay,0], '+y', '#57c957');
  text3([0,0,0.15], '+z', '#6a8dff');
  text3([gx0+0.02, gy0+0.02, 0], 'base plane z=0', '#707080');
}
function drawBox(){
  if (!WS || !el('tBox').checked) return;
  const m = WS.min, M = WS.max;
  const c = [[m[0],m[1],m[2]],[M[0],m[1],m[2]],[M[0],M[1],m[2]],[m[0],M[1],m[2]],
             [m[0],m[1],M[2]],[M[0],m[1],M[2]],[M[0],M[1],M[2]],[m[0],M[1],M[2]]];
  const E = [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];
  for (const e of E) line3(c[e[0]], c[e[1]], '#d9a05f', 1.2, 0.75, [5,4]);
  text3([m[0],m[1],M[2]], 'Piper workspace', '#d9a05f');
}
function drawTriad(p, q, L, w, alpha){
  const m = quatMat(q);
  const cols = ['#ff5252', '#43d17a', '#5b8cff'];   // tool x / y / z
  for (let k = 0; k < 3; k++)
    line3(p, [p[0]+m[0][k]*L, p[1]+m[1][k]*L, p[2]+m[2][k]*L], cols[k], w, alpha);
}
function drawArm(s){
  const A = arms[s]; if (!A.show) return;
  if (el('tObs').checked && !A.obsEq)
    for (let i = 1; i < N; i++) line3(A.OT[i-1], A.OT[i], '#8a8f98', 1.1, 0.5);
  for (let i = 1; i < N; i++) line3(A.T[i-1], A.T[i], A.segColor[i], 2, 0.95);
  if (el('tTriads').checked) {
    const stride = Math.max(1, parseInt(el('stride').value) || 1);
    const L = axLen * 0.7 * parseFloat(el('axscale').value);
    for (let i = 0; i < N; i += stride) drawTriad(A.T[i], A.Q[i], L, 1.1, 0.55);
  }
  if (el('tOutside').checked && WS)
    for (let i = 0; i < N; i++) if (A.outside[i]) dot3(A.T[i], '#ff5757', 1.6);
  if (el('tJumps').checked)
    for (const j of A.jumps) {
      const q = proj(A.T[j]); if (!q) continue;
      ctx.strokeStyle = '#ff4040'; ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.arc(q[0], q[1], 6, 0, 6.2832); ctx.stroke();
    }
  const q0 = dot3(A.T[0], '#4ade80', 4);
  if (q0) { ctx.fillStyle = '#4ade80'; ctx.fillText('start', q0[0]+6, q0[1]); }
  const qe = proj(A.T[N-1]);
  if (qe) { ctx.fillStyle = '#f87171'; ctx.fillRect(qe[0]-3.5, qe[1]-3.5, 7, 7); ctx.fillText('end', qe[0]+6, qe[1]); }
}
function drawCurrent(){
  for (const s of SIDES) {
    const A = arms[s]; if (!A.show) continue;
    const p = A.T[cur];
    if (el('tDrop').checked) {
      line3(p, [p[0],p[1],0], '#ffffff', 1, 0.3, [3,4]);
      line3([p[0]-0.01,p[1],0], [p[0]+0.01,p[1],0], '#ffffff', 1, 0.4);
      line3([p[0],p[1]-0.01,0], [p[0],p[1]+0.01,0], '#ffffff', 1, 0.4);
    }
    drawTriad(p, A.Q[cur], axLen * 1.8 * parseFloat(el('axscale').value), 2.8, 1);
    dot3(p, A.outside[cur] ? '#ff5757' : '#ffffff', 3.2);
  }
}

// ---- strip chart: z and gripper over time ----------------------------------
let stripDrag = false;
function drawStrip(){
  sctx.fillStyle = '#121218'; sctx.fillRect(0, 0, SW, SH);
  let zmin = 1e9, zmax = -1e9, gmax = 1e-9;
  for (const s of SIDES) { const A = arms[s]; if (!A.show) continue;
    for (let i = 0; i < N; i++) {
      const z = A.T[i][2];
      if (z < zmin) zmin = z; if (z > zmax) zmax = z;
      if (A.G[i] > gmax) gmax = A.G[i];
    }
  }
  if (zmin > zmax) return;
  if (zmax - zmin < 1e-6) zmax = zmin + 1e-6;
  const X = i => 10 + i/Math.max(N-1,1)*(SW-20);
  const Yz = z => SH-8 - (z-zmin)/(zmax-zmin)*(SH-16);
  const Yg = g => SH-8 - g/gmax*(SH-16);
  if (zmin < 0 && zmax > 0) {
    sctx.strokeStyle = '#3a3a44'; sctx.beginPath();
    sctx.moveTo(10, Yz(0)); sctx.lineTo(SW-10, Yz(0)); sctx.stroke();
  }
  for (const s of SIDES) { const A = arms[s]; if (!A.show) continue;
    sctx.strokeStyle = A.segColor[Math.floor(N/2)]; sctx.lineWidth = 1.3;
    sctx.beginPath();
    for (let i = 0; i < N; i++) { const x = X(i), y = Yz(A.T[i][2]); i ? sctx.lineTo(x,y) : sctx.moveTo(x,y); }
    sctx.stroke();
    sctx.strokeStyle = '#e8c95a'; sctx.setLineDash([2,3]); sctx.beginPath();
    for (let i = 0; i < N; i++) { const x = X(i), y = Yg(A.G[i]); i ? sctx.lineTo(x,y) : sctx.moveTo(x,y); }
    sctx.stroke(); sctx.setLineDash([]);
  }
  sctx.globalAlpha = 0.7; sctx.strokeStyle = '#ffffff';
  sctx.beginPath(); sctx.moveTo(X(cur), 2); sctx.lineTo(X(cur), SH-2); sctx.stroke();
  sctx.globalAlpha = 1;
  sctx.font = '10px system-ui';
  sctx.fillStyle = '#8f8f9a'; sctx.fillText('z (m)', 12, 12);
  sctx.fillStyle = '#e8c95a'; sctx.fillText('gripper', 52, 12);
}
function stripSeek(e){
  const r = strip.getBoundingClientRect();
  const t = (e.clientX - r.left - 10) / Math.max(r.width - 20, 1);
  playing = false; el('play').innerHTML = '&#9654; Play';
  setFrame(Math.round(Math.min(1, Math.max(0, t)) * (N-1)));
}
strip.addEventListener('pointerdown', e => { stripDrag = true; strip.setPointerCapture(e.pointerId); stripSeek(e); });
strip.addEventListener('pointermove', e => { if (stripDrag) stripSeek(e); });
strip.addEventListener('pointerup', () => stripDrag = false);

// ---- readout ---------------------------------------------------------------
function updateReadout(){
  el('frameinfo').textContent = 'f ' + cur + '/' + (N-1) + '  t=' + TS[cur].toFixed(2) + 's';
  let txt = '';
  for (const s of SIDES) {
    const A = arms[s]; if (!A.show) continue;
    const p = A.T[cur], e = rpyDeg(quatMat(A.Q[cur])), q = A.Q[cur];
    let dp = 0;
    if (cur > 0) {
      const a = A.P[cur], b = A.P[cur-1];
      dp = Math.hypot(a[0]-b[0], a[1]-b[1], a[2]-b[2]) * 1000;
    }
    txt += s + '  pos ' + p.map(v => v.toFixed(3)).join(', ') + ' m'
        + (A.outside[cur] ? '  [outside ws]' : '') + '\n'
        + '   rpy ' + e.map(v => v.toFixed(1)).join(', ') + ' deg\n'
        + '   quat wxyz ' + q.map(v => v.toFixed(3)).join(', ') + '\n'
        + '   grip raw ' + A.G[cur].toFixed(3) + ' -> ' + (A.GW[cur]*1000).toFixed(1) + ' mm'
        + '   dpos ' + dp.toFixed(1) + ' mm\n';
  }
  el('readout').textContent = txt;
}

// ---- main loop -------------------------------------------------------------
let cur = 0, frameF = 0, playing = false, speed = 1, lastT = performance.now();
function setFrame(i){
  cur = Math.min(N-1, Math.max(0, Math.round(i)));
  frameF = cur; el('frame').value = cur;
}
function draw(){
  computeBasis();
  ctx.fillStyle = '#101014'; ctx.fillRect(0, 0, W, H);
  ctx.font = '11px system-ui';
  drawGrid(); drawBox();
  for (const s of SIDES) drawArm(s);
  drawCurrent();
  drawStrip(); updateReadout();
}
function tick(now){
  const dt = (now - lastT)/1000; lastT = now;
  if (playing) {
    frameF += dt * FPS * speed;
    if (frameF > N-1) frameF = 0;
    cur = Math.round(frameF); el('frame').value = cur;
  }
  draw();
  requestAnimationFrame(tick);
}

// ---- interactions ----------------------------------------------------------
let dragMode = null, lastX = 0, lastY = 0;
canvas.addEventListener('contextmenu', e => e.preventDefault());
canvas.addEventListener('pointerdown', e => {
  dragMode = (e.button === 2 || e.shiftKey) ? 'pan' : 'orbit';
  lastX = e.clientX; lastY = e.clientY;
  canvas.setPointerCapture(e.pointerId); canvas.classList.add('dragging');
});
canvas.addEventListener('pointermove', e => {
  if (!dragMode) return;
  const dx = e.clientX - lastX, dy = e.clientY - lastY;
  lastX = e.clientX; lastY = e.clientY;
  if (dragMode === 'orbit') {
    cam.yaw -= dx*0.008;
    cam.pitch = Math.min(1.55, Math.max(-1.55, cam.pitch + dy*0.008));
  } else {
    const k = cam.dist/focal;
    for (let i = 0; i < 3; i++) cam.target[i] += -right[i]*dx*k + upv[i]*dy*k;
  }
});
canvas.addEventListener('pointerup', () => { dragMode = null; canvas.classList.remove('dragging'); });
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  cam.dist = Math.min(20, Math.max(0.05, cam.dist * Math.exp(e.deltaY*0.0012)));
}, {passive:false});
canvas.addEventListener('dblclick', fitView);

el('play').addEventListener('click', () => {
  playing = !playing;
  el('play').innerHTML = playing ? '&#10074;&#10074; Pause' : '&#9654; Play';
  lastT = performance.now();
});
el('frame').max = N-1;
el('frame').addEventListener('input', e => {
  playing = false; el('play').innerHTML = '&#9654; Play';
  setFrame(+e.target.value);
});
el('speed').addEventListener('input', e => {
  speed = +e.target.value; el('speedv').textContent = speed.toFixed(1) + 'x';
});
document.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => {
  const v = VIEWS[b.dataset.view];
  cam.yaw = v[0]*Math.PI/180; cam.pitch = v[1]*Math.PI/180;
}));
el('fit').addEventListener('click', fitView);
for (const id of ['dx','dy','dz']) el(id).addEventListener('input', updateTransform);
el('tClamp').addEventListener('change', updateTransform);
el('suggest').addEventListener('click', () => {
  const o = arms[SIDES[0]].off;
  el('dx').value = o[0].toFixed(3); el('dy').value = o[1].toFixed(3); el('dz').value = o[2].toFixed(3);
  updateTransform();
});
el('zero').addEventListener('click', () => {
  for (const id of ['dx','dy','dz']) el(id).value = '0.000';
  updateTransform();
});
window.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' && e.target.type !== 'range') return;
  if (e.code === 'Space') { e.preventDefault(); el('play').click(); }
  else if (e.key === 'ArrowRight') { playing = false; setFrame(cur + (e.shiftKey ? 10 : 1)); }
  else if (e.key === 'ArrowLeft') { playing = false; setFrame(cur - (e.shiftKey ? 10 : 1)); }
  else if (e.key === 'Home') setFrame(0);
  else if (e.key === 'End') setFrame(N-1);
});

// ---- init ------------------------------------------------------------------
if (SIDES.length > 1) {
  for (const s of SIDES) {
    const l = document.createElement('label');
    l.className = 'chk';
    const cb = document.createElement('input');
    cb.type = 'checkbox'; cb.checked = true;
    cb.addEventListener('change', ev => { arms[s].show = ev.target.checked; updateTransform(); });
    l.appendChild(cb); l.appendChild(document.createTextNode(' show ' + s));
    el('showArms').appendChild(l);
  }
} else el('showArms').style.display = 'none';

el('stride').value = DATA.stride;
el('dx').value = DATA.initOffset[0].toFixed(3);
el('dy').value = DATA.initOffset[1].toFixed(3);
el('dz').value = DATA.initOffset[2].toFixed(3);
el('tObs').checked = SIDES.some(s => !arms[s].obsEq);
if (SIDES.every(s => arms[s].obsEq))
  el('tObs').parentElement.title = 'observation pose equals action pose in this dataset';
if (!WS) {
  for (const id of ['tBox','tClamp','tOutside']) { el(id).checked = false; el(id).disabled = true; }
}
el('subinfo').textContent = DATA.schema + ' | ' + N + ' frames | '
  + (TS[N-1]-TS[0]).toFixed(1) + ' s @ ' + FPS.toFixed(1) + ' Hz | ' + SIDES.join(' + ');
el('stats').textContent = DATA.statsText;
resize(); updateTransform(); fitView();
requestAnimationFrame(tick);
</script>
</body>
</html>
"""
