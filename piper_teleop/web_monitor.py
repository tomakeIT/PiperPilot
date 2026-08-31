"""Minimal local web dashboard for data collection.

Mirrors the in-headset HUD in a browser tab on the host (and optionally other
devices on the LAN): recording state, episode timer, episodes saved, clutch
state, input rate, plus a small live view of every recording camera.

Read-only and dependency-free (stdlib http.server; cv2 only for JPEG
encoding). Never on the critical path: it polls the same snapshots the HUD
uses and a dead client cannot stall the recorder.

Endpoints:
    GET /                the dashboard page (self-contained HTML)
    GET /status          JSON from Recorder.status_dict()
    GET /cam/<name>.jpg  latest frame of that camera
    GET /calib/datasets  JSON list of LeRobot datasets under datasets_root
    GET /calib/ref/<name>.jpg?dataset=D   reference frame (episode 0, frame 0)
                         of camera <name> from dataset D — the "camera
                         calibrate" overlay uses these to re-align a moved
                         camera against an existing dataset
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs

_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>piper-collect</title>
<style>
 body{margin:0;background:#101014;color:#e8e8ee;font:16px/1.4 system-ui,sans-serif}
 main{max-width:980px;margin:0 auto;padding:28px 24px}
 #state{font-size:60px;font-weight:700;font-variant-numeric:tabular-nums}
 .ready #state{color:#5ade7a}.recording #state{color:#ff5252}.homing #state{color:#ffbe3c}
 #dot{display:none}
 .recording #dot{display:inline-block;width:.45em;height:.45em;border-radius:50%;
   background:#ff5252;margin-right:.28em;animation:blink 1s steps(1) infinite}
 @keyframes blink{50%{opacity:0}}
 #sub{color:#9a9aa6;margin-top:6px;font-variant-numeric:tabular-nums}
 #err{color:#ffbe3c;min-height:1.4em;margin-top:8px}
 #limits{margin-top:16px;display:flex;flex-wrap:wrap;gap:6px;min-height:1.8em}
 .chip{padding:3px 12px;border-radius:14px;background:#1d1d24;color:#63636e;
   font:13px ui-monospace,monospace;border:1px solid #2a2a32}
 .chip.on{background:#5b2020;color:#ffc9c9;border-color:#a33}
 .chip b{font-weight:600}
 #joints{margin-top:12px;max-width:520px}
 .jrow{display:flex;align-items:center;gap:10px;margin:4px 0;
   font:12px ui-monospace,monospace;color:#9a9aa6}
 .jrow>span:first-child{width:20px}
 .jtrack{position:relative;flex:1;height:10px;background:#1d1d24;border-radius:5px}
 .jtrack::before,.jtrack::after{content:'';position:absolute;top:0;bottom:0;width:7%;
   background:#3b2126;border-radius:5px;left:0}
 .jtrack::after{left:auto;right:0}
 .jmark{position:absolute;top:-2px;width:4px;height:14px;background:#6ab0ff;
   border-radius:2px}
 .jmark.hot{background:#ff5252}
 .jval{width:52px;text-align:right}
 #cams{display:flex;gap:14px;flex-wrap:wrap;margin-top:26px}
 figure{margin:0}
 figcaption{color:#9a9aa6;font-size:13px;margin-top:5px;font-family:ui-monospace,monospace}
 img{width:300px;max-width:100%;aspect-ratio:4/3;border-radius:6px;background:#000;object-fit:contain}
 #calib{margin-top:26px;display:flex;gap:10px;align-items:center;flex-wrap:wrap;
   font:13px ui-monospace,monospace;color:#9a9aa6}
 #calib[hidden],#calib-ctrl[hidden]{display:none}
 #calib-ctrl{display:inline-flex;gap:6px;align-items:center}
 #calib select,#calib button{background:#1d1d24;color:#c8c8d2;border:1px solid #2a2a32;
   border-radius:6px;font:inherit;padding:4px 10px;max-width:340px}
 #calib button.on{background:#1d2a3f;color:#9ecbff;border-color:#35507a}
 #calib input[type=range]{width:120px;accent-color:#6ab0ff}
 .camwrap{position:relative;line-height:0}
 .camwrap .ref{position:absolute;left:0;top:0;display:none;pointer-events:none}
 #cams.calib .ref{display:block}
 #cams.diff .ref{mix-blend-mode:difference}
 #cams.calib img{width:480px}
 #meta{margin-top:26px;color:#9a9aa6;font-size:13px;font-family:ui-monospace,monospace;
   overflow-wrap:anywhere}
</style></head><body><main>
 <div id="state"><span id="dot"></span><span id="txt">--</span></div>
 <div id="sub"></div><div id="err"></div>
 <div id="limits"></div>
 <div id="joints"></div>
 <div id="calib" hidden>
  <span>camera calibrate</span>
  <select id="calib-ds"><option value="">off</option></select>
  <span id="calib-ctrl" hidden>
   <button id="calib-onion" class="on" title="reference frame at adjustable opacity">overlay</button>
   <button id="calib-diff" title="|live - reference|: image goes black when aligned">difference</button>
   <input id="calib-alpha" type="range" min="10" max="90" value="50" title="reference opacity">
  </span>
 </div>
 <div id="cams">__CAM_FIGURES__</div>
 <div id="meta"></div>
<script>
const CAMS = __CAM_LIST__;
async function tick(){
  try{
    const s = await (await fetch('/status')).json();
    document.body.className = s.state;
    const mm = String(Math.floor(s.elapsed_s/60)).padStart(2,'0');
    const ss = (s.elapsed_s%60).toFixed(1).padStart(4,'0');
    document.getElementById('txt').textContent =
      s.state==='recording' ? `REC ep${s.episode_index}  ${mm}:${ss}` :
      s.state==='homing'    ? 'HOMING …' :
      s.eps_saved==null     ? 'READY' :
                              `READY · ${s.eps_saved} saved`;
    const parts = [];
    if (s.eps_saved != null)
      parts.push(`${s.eps_saved} episodes · ${s.total_frames} frames`);
    parts.push(s.engaged ? 'ENGAGED' : 'idle');
    parts.push(`input ${s.input_hz.toFixed(0)} Hz`);
    document.getElementById('sub').textContent = parts.join(' · ');
    document.getElementById('err').textContent = s.error || '';
    renderLimits(s.limits || {});
    renderJoints(s.joints);
    document.getElementById('meta').textContent = `${s.task} — ${s.root}`;
  }catch(e){ document.getElementById('err').textContent = 'monitor connection lost'; }
  setTimeout(tick, 300);
}
// Safety clamps that fired: lit chip = shaping the motion right now,
// ·N = total hits this session.
const LIMIT_LABELS = {ws:'workspace', vel:'lin vel', angvel:'ang vel',
                      pitch:'pitch', tether:'tether', ik:'IK fail',
                      jlim:'joint limit', jvel:'joint vel'};
function renderLimits(groups){
  const chips = [];
  for (const g of Object.keys(groups))
    for (const [n, s] of Object.entries(groups[g])){
      const label = LIMIT_LABELS[n] || n;
      chips.push(`<span class="chip${s.active ? ' on' : ''}">${label}` +
        (s.active && s.detail ? ` <b>${s.detail}</b>` : '') +
        ` ·${s.count}</span>`);
    }
  document.getElementById('limits').innerHTML = chips.join('');
}
function renderJoints(j){
  const div = document.getElementById('joints');
  if (!j){ div.innerHTML = ''; return; }
  const rows = [];
  for (let i = 0; i < j.q.length; i++){
    const lo = j.lo[i], hi = j.hi[i], q = j.q[i];
    const f = Math.min(1, Math.max(0, (q - lo) / (hi - lo)));
    const hot = (q - lo) < 3*Math.PI/180 || (hi - q) < 3*Math.PI/180;
    rows.push(`<div class=jrow><span>j${i+1}</span><div class=jtrack>` +
      `<div class="jmark${hot ? ' hot' : ''}" style="left:calc(${(f*100).toFixed(1)}% - 2px)"></div>` +
      `</div><span class=jval>${(q*180/Math.PI).toFixed(0)}&deg;</span></div>`);
  }
  div.innerHTML = rows.join('');
}
// Camera calibrate: overlay a reference frame (episode 0, frame 0) from a
// previous dataset on each live view so a moved camera can be re-aligned.
// "overlay" = onion skin at adjustable opacity for coarse placement;
// "difference" = |live - ref| blend, the view goes black when aligned.
const calib = {diff:false};
async function calibInit(){
  try{
    const r = await fetch('/calib/datasets');
    if (!r.ok) return;
    const list = (await r.json()).datasets || [];
    if (!list.length) return;
    const sel = document.getElementById('calib-ds');
    for (const d of list){
      const o = document.createElement('option');
      o.value = o.textContent = d;
      sel.appendChild(o);
    }
    for (const c of CAMS){
      const ref = document.getElementById('ref-'+c);
      if (!ref) continue;   // e.g. dataset has no video for this camera
      ref.onerror = () => { ref.style.visibility = 'hidden'; };
      ref.onload  = () => { ref.style.visibility = 'visible'; };
    }
    sel.onchange = calibApply;
    document.getElementById('calib-onion').onclick = () => calibMode(false);
    document.getElementById('calib-diff').onclick = () => calibMode(true);
    document.getElementById('calib-alpha').oninput = calibApply;
    document.getElementById('calib').hidden = false;
  }catch(e){}
}
function calibMode(diff){
  calib.diff = diff;
  document.getElementById('calib-onion').classList.toggle('on', !diff);
  document.getElementById('calib-diff').classList.toggle('on', diff);
  calibApply();
}
function calibApply(){
  const ds = document.getElementById('calib-ds').value;
  const cams = document.getElementById('cams');
  cams.classList.toggle('calib', !!ds);
  cams.classList.toggle('diff', !!ds && calib.diff);
  document.getElementById('calib-ctrl').hidden = !ds;
  document.getElementById('calib-alpha').disabled = calib.diff;
  const a = document.getElementById('calib-alpha').value/100;
  for (const c of CAMS){
    const ref = document.getElementById('ref-'+c);
    if (!ref) continue;
    const want = ds ? '/calib/ref/'+c+'.jpg?dataset='+encodeURIComponent(ds) : '';
    if (ref.dataset.src !== want){
      ref.dataset.src = want;
      if (want) ref.src = want; else ref.removeAttribute('src');
    }
    ref.style.opacity = calib.diff ? 1 : a;
  }
}
tick();
calibInit();
setInterval(() => {
  for (const c of CAMS){
    const img = document.getElementById('cam-'+c);
    if (img) img.src = '/cam/'+c+'.jpg?t='+Date.now();
  }
}, 400);
</script></main></body></html>
"""


class WebMonitor:
    """Serves the collection dashboard in a daemon thread."""

    def __init__(self, status_fn, cameras, host: str = "127.0.0.1",
                 port: int = 8780, jpeg_quality: int = 70,
                 datasets_root: str | None = None):
        """status_fn: zero-arg callable returning a JSON-able status dict
        (Recorder.status_dict during collection; a slimmer adapter during
        plain teleop — eps_saved/total_frames may be null there).
        datasets_root: directory scanned for LeRobot datasets by the
        camera-calibrate overlay; None hides the feature."""
        self.status_fn = status_fn
        self.cameras = {c.name: c for c in cameras}
        self.host = host
        self.port = int(port)
        self.jpeg_quality = int(jpeg_quality)
        self.datasets_root = datasets_root
        self._calib_cache: dict[tuple[str, str], bytes] = {}
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    # -- lifecycle -----------------------------------------------------------

    def start(self) -> bool:
        """Bind and serve; returns False (with a warning) if the port is
        taken — monitoring must never block collection."""
        monitor = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # silence per-request stderr spam
                pass

            def do_GET(self):
                try:
                    monitor._route(self)
                except (BrokenPipeError, ConnectionResetError):
                    pass  # browser tab closed mid-response

        try:
            self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as e:
            print(f"[monitor] disabled: cannot bind {self.host}:{self.port} ({e})")
            return False
        self._httpd.daemon_threads = True
        self.port = self._httpd.server_address[1]  # resolves port=0 -> real port
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, name="web-monitor", daemon=True)
        self._thread.start()
        print(f"[monitor] dashboard: http://{self.host}:{self.port}")
        return True

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None

    # -- request handling ------------------------------------------------------

    def _route(self, req: BaseHTTPRequestHandler) -> None:
        path = req.path.split("?", 1)[0]
        if path == "/":
            self._send(req, 200, "text/html; charset=utf-8", self._page().encode())
        elif path == "/status":
            body = json.dumps(self.status_fn()).encode()
            self._send(req, 200, "application/json", body)
        elif path.startswith("/cam/") and path.endswith(".jpg"):
            self._serve_cam(req, path[len("/cam/"):-len(".jpg")])
        elif path == "/calib/datasets":
            body = json.dumps({"datasets": self._list_datasets()}).encode()
            self._send(req, 200, "application/json", body)
        elif path.startswith("/calib/ref/") and path.endswith(".jpg"):
            query = req.path.partition("?")[2]
            dataset = (parse_qs(query).get("dataset") or [""])[0]
            self._serve_calib_ref(req, path[len("/calib/ref/"):-len(".jpg")], dataset)
        else:
            self._send(req, 404, "text/plain", b"not found")

    def _page(self) -> str:
        figures = "".join(
            f'<figure><div class="camwrap"><img id="cam-{n}" alt="{n}">'
            f'<img class="ref" id="ref-{n}" alt=""></div>'
            f'<figcaption>{n}</figcaption></figure>'
            for n in self.cameras)
        return (_PAGE
                .replace("__CAM_FIGURES__", figures)
                .replace("__CAM_LIST__", json.dumps(list(self.cameras))))

    def _serve_cam(self, req: BaseHTTPRequestHandler, name: str) -> None:
        cam = self.cameras.get(name)
        fr = cam.latest() if cam is not None else None
        if fr is None:
            self._send(req, 404, "text/plain", b"no frame")
            return
        import cv2
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(fr.rgb, cv2.COLOR_RGB2BGR),
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self._send(req, 500, "text/plain", b"encode failed")
            return
        self._send(req, 200, "image/jpeg", buf.tobytes())

    # -- camera calibrate ------------------------------------------------------

    def _list_datasets(self) -> list[str]:
        """LeRobot dataset dirs under datasets_root, newest first."""
        if self.datasets_root is None:
            return []
        root = Path(self.datasets_root).expanduser()
        if not root.is_dir():
            return []
        found = [(d.stat().st_mtime, d.name) for d in root.iterdir()
                 if (d / "meta" / "info.json").is_file()]
        return [name for _, name in sorted(found, reverse=True)]

    def _serve_calib_ref(self, req: BaseHTTPRequestHandler, cam: str,
                         dataset: str) -> None:
        data = self._calib_ref_jpeg(dataset, cam)
        if data is None:
            self._send(req, 404, "text/plain", b"no reference frame")
        else:
            self._send(req, 200, "image/jpeg", data)

    def _calib_ref_jpeg(self, dataset: str, cam: str) -> bytes | None:
        """First frame of the dataset's first episode for camera `cam`,
        JPEG-encoded and cached (a decoded reference never changes)."""
        if self.datasets_root is None or not dataset or not cam:
            return None
        if dataset != Path(dataset).name or dataset in (".", ".."):
            return None  # dataset must be a plain dir name inside the root
        if (data := self._calib_cache.get((dataset, cam))) is not None:
            return data
        chunk = Path(self.datasets_root).expanduser() / dataset / "videos" / "chunk-000"
        vdir = chunk / f"observation.images.{cam}"
        if not vdir.is_dir():  # tolerate other video-key conventions
            hits = sorted(p for p in chunk.glob(f"*{cam}")
                          if p.is_dir()) if chunk.is_dir() else []
            if not hits:
                return None
            vdir = hits[0]
        vids = sorted(vdir.glob("episode_*.mp4"))
        if not vids:
            return None
        import cv2
        cap = cv2.VideoCapture(str(vids[0]))
        ok, bgr = cap.read()
        cap.release()
        if not ok:
            return None
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return None
        data = buf.tobytes()
        self._calib_cache[(dataset, cam)] = data
        return data

    @staticmethod
    def _send(req: BaseHTTPRequestHandler, code: int, ctype: str, body: bytes) -> None:
        req.send_response(code)
        req.send_header("Content-Type", ctype)
        req.send_header("Content-Length", str(len(body)))
        req.send_header("Cache-Control", "no-store")
        req.end_headers()
        req.wfile.write(body)
