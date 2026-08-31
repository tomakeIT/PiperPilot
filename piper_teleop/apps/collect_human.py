"""RGB-D snapshot capture from a single RealSense (photo-style, no video).

Streams one RealSense D400, shows a live browser UI (color + colorized
depth), and saves a still RGB-D pair every time you hit the shoot button:

Three files per shot — RGB, raw depth, and a human-viewable depth heatmap:

    <root>/<name>/
      calib.json                # intrinsics / extrinsics / depth scale
      shots.csv                 # per-shot wall time + device timestamps
      000000_color.png          # 8-bit RGB, lossless
      000000_depth.png          # 16-bit raw z16 — THE data (meters = v * 0.001)
      000000_depth_viz.png      # 8-bit colorized depth, for eyeballing only
      000000_ir1.png, _ir2.png  # optional (--ir)

All PNG, all lossless. Load raw depth with the UNCHANGED flag or OpenCV
silently hands back an 8-bit BGR image:

    depth_m = cv2.imread(p, cv2.IMREAD_UNCHANGED).astype(np.float32) * 0.001

Depth is aligned to the color camera by default (each depth pixel matches the
same color pixel; deproject with the color intrinsics). With --raw-depth the
native depth frame is stored instead and the depth intrinsics plus the
depth->color extrinsics in calib.json apply.

Web page (default http://127.0.0.1:8790): shoot / undo buttons; space
shoots, u deletes the last shot.

Run inside the piper_teleop conda env (pyrealsense2, cv2):
    piper-human --name my_scene        # or: python -m piper_teleop.apps.collect_human
"""

from __future__ import annotations

import argparse
import csv
import json
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

# Pass --serial to pin capture to a particular camera. An empty value selects
# the first connected RealSense device.
DEFAULT_SERIAL = ""

# (width, height, fps) tried in order until the pipeline delivers frames.
# USB2 ports can't sustain color+depth 640x480@30 — 15 fps is the reliable
# ceiling there; USB3 takes the first profile.
FALLBACK_PROFILES = [(640, 480, 30), (640, 480, 15), (480, 270, 30)]

DEPTH_VIZ_MAX_MM = 2000  # colormap far end; near end is always 0

PREVIEW_FPS = 15         # browser stream rate (capture still runs at full fps)
PREVIEW_QUALITY = 75     # JPEG quality for the stream

# z16 -> 8-bit ramp, built once. A lookup beats the equivalent float32 math by
# ~2x and keeps the preview off the float path entirely.
_DEPTH_LUT = np.clip(np.arange(65536) * (255.0 / DEPTH_VIZ_MAX_MM), 0, 255).astype(np.uint8)


def colorize_depth(depth: np.ndarray) -> np.ndarray:
    """Raw z16 -> BGR heatmap for human eyes (near = blue, far = red,
    invalid = black). Fixed range, so shots stay comparable to each other.
    Lossy and for viewing only — never load this back as depth."""
    import cv2

    cmap = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
    bgr = cv2.applyColorMap(_DEPTH_LUT[depth], cmap)
    bgr[depth == 0] = 0
    return bgr


# ---------------------------------------------------------------------------
# capture


@dataclass
class FrameSet:
    color: np.ndarray            # (H, W, 3) uint8 RGB
    depth: np.ndarray            # (H, W) uint16, units of depth_scale meters
    ir: list[np.ndarray]         # 0 or 2 grayscale (H, W) uint8
    mono: float                  # host time.monotonic() at arrival
    device_ts_ms: float          # RealSense frameset timestamp
    frame_number: int            # color sensor frame counter


def _intrinsics_dict(intr) -> dict:
    return {
        "width": intr.width, "height": intr.height,
        "fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
        "model": str(intr.model).split(".")[-1], "coeffs": list(intr.coeffs),
    }


class RealSenseCapture:
    """Grab thread for one device; exposes the latest frame set."""

    def __init__(self, serial: str, width: int, height: int, fps: int | None,
                 align: bool = True, ir: bool = False):
        import pyrealsense2 as rs

        if not serial:
            devices = list(rs.context().devices)
            if not devices:
                raise RuntimeError("no RealSense camera found")
            serial = devices[0].get_info(rs.camera_info.serial_number)
        self._rs = rs
        self.serial = serial
        self.align_enabled = align
        self.ir_enabled = ir

        usb = self._usb_descriptor(serial)
        self.usb = usb or "?"
        if fps is None:  # auto: USB2 can't sustain color+depth at 30 fps
            fps = 15 if usb.startswith("2") else 30

        profiles = [(width, height, fps)]
        profiles += [p for p in FALLBACK_PROFILES if p not in profiles]
        self.profile, err = None, None
        for w, h, f in profiles:
            try:
                self._start_pipeline(w, h, f)
                self.profile = (w, h, f)
                break
            except RuntimeError as e:
                err = e
                print(f"[capture] {w}x{h}@{f} failed ({e}), trying next profile")
        if self.profile is None:
            raise RuntimeError(f"could not start RealSense {serial}: {err}")

        self.width, self.height, self.fps = self.profile
        print(f"[capture] RealSense {serial} (USB {self.usb}) "
              f"{self.width}x{self.height}@{self.fps} align={align} ir={ir}")

        self._align = rs.align(rs.stream.color) if align else None
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._latest: FrameSet | None = None
        self._grab_times: list[float] = []
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="rs-grab", daemon=True)
        self._thread.start()

    def _usb_descriptor(self, serial: str) -> str:
        rs = self._rs
        for d in rs.context().devices:
            if d.get_info(rs.camera_info.serial_number) == serial:
                if d.supports(rs.camera_info.usb_type_descriptor):
                    return d.get_info(rs.camera_info.usb_type_descriptor)
        return ""

    def _start_pipeline(self, w: int, h: int, fps: int) -> None:
        rs = self._rs
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        if self.ir_enabled:
            cfg.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, fps)
            cfg.enable_stream(rs.stream.infrared, 2, w, h, rs.format.y8, fps)
        started = self._pipeline.start(cfg)
        try:  # a profile can start yet never deliver on a saturated bus
            self._pipeline.wait_for_frames(timeout_ms=5000)
        except RuntimeError:
            self._pipeline.stop()
            raise RuntimeError("pipeline started but no frames arrived")
        self._started_profile = started

    def calibration(self) -> dict:
        rs = self._rs
        prof = self._started_profile
        color_sp = prof.get_stream(rs.stream.color).as_video_stream_profile()
        depth_sp = prof.get_stream(rs.stream.depth).as_video_stream_profile()
        extr = depth_sp.get_extrinsics_to(color_sp)
        dev = prof.get_device()
        return {
            "serial": self.serial,
            "device_name": dev.get_info(rs.camera_info.name),
            "usb": self.usb,
            "width": self.width, "height": self.height, "fps": self.fps,
            "depth_scale_m": dev.first_depth_sensor().get_depth_scale(),
            "depth_aligned_to_color": self.align_enabled,
            "color_intrinsics": _intrinsics_dict(color_sp.get_intrinsics()),
            "depth_intrinsics": _intrinsics_dict(depth_sp.get_intrinsics()),
            "depth_to_color_extrinsics": {
                "rotation_row_major_3x3": list(extr.rotation),
                "translation_m": list(extr.translation),
            },
            "depth_viz_max_mm": DEPTH_VIZ_MAX_MM,
            "note": ("*_depth.png are raw z16 (read with cv2.IMREAD_UNCHANGED); "
                     "meters = value * depth_scale_m, 0 = invalid. "
                     "If depth_aligned_to_color, deproject depth pixels with "
                     "color_intrinsics; otherwise use depth_intrinsics and the "
                     "depth->color extrinsics. *_depth_viz.png is an 8-bit "
                     "colormap over 0..depth_viz_max_mm — for viewing only."),
        }

    def _loop(self) -> None:
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=2000)
            except RuntimeError:
                continue
            if self._align is not None:
                frames = self._align.process(frames)
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color or not depth:
                continue
            ir = []
            if self.ir_enabled:
                for idx in (1, 2):
                    f = frames.get_infrared_frame(idx)
                    if f:
                        ir.append(np.asanyarray(f.get_data()).copy())
            fs = FrameSet(
                color=np.asanyarray(color.get_data()).copy(),
                depth=np.asanyarray(depth.get_data()).copy(),
                ir=ir,
                mono=time.monotonic(),
                device_ts_ms=frames.get_timestamp(),
                frame_number=color.get_frame_number(),
            )
            with self._lock:
                self._latest = fs
                self._grab_times = [t for t in self._grab_times if fs.mono - t < 2.0]
                self._grab_times.append(fs.mono)
                self._new_frame.notify_all()

    def latest(self) -> FrameSet | None:
        with self._lock:
            return self._latest

    def wait_next(self, after: int, timeout: float = 2.0) -> FrameSet | None:
        """Block until a frame newer than `after` arrives (or timeout).
        Lets preview streams idle instead of polling."""
        with self._new_frame:
            if self._latest is None or self._latest.frame_number == after:
                self._new_frame.wait(timeout)
            return self._latest

    def grab_hz(self) -> float:
        with self._lock:
            n = len(self._grab_times)
            if n < 2:
                return 0.0
            span = self._grab_times[-1] - self._grab_times[0]
        return (n - 1) / span if span > 0 else 0.0

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=3.0)
        try:
            self._pipeline.stop()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# shot store


class ShotStore:
    """Numbered RGB-D stills in one flat directory + shots.csv index."""

    CSV_HEADER = ["idx", "wall_time", "host_mono_s", "device_ts_ms",
                  "device_frame_number"]

    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        self._lock = threading.Lock()
        self._rows: list[list] = []
        csv_path = out_dir / "shots.csv"
        if csv_path.is_file():
            with open(csv_path, newline="") as f:
                self._rows = [r for r in csv.reader(f)][1:]
        existing = [int(p.name.split("_")[0]) for p in out_dir.glob("*_color.png")]
        self.next_idx = max(existing, default=-1) + 1

    @property
    def count(self) -> int:
        return len(self._rows)

    def shoot(self, fs: FrameSet) -> str:
        import cv2

        with self._lock:
            idx = self.next_idx
            stem = f"{idx:06d}"
            cv2.imwrite(str(self.out_dir / f"{stem}_color.png"),
                        cv2.cvtColor(fs.color, cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(self.out_dir / f"{stem}_depth.png"), fs.depth)
            cv2.imwrite(str(self.out_dir / f"{stem}_depth_viz.png"),
                        colorize_depth(fs.depth))
            for i, img in enumerate(fs.ir):
                cv2.imwrite(str(self.out_dir / f"{stem}_ir{i + 1}.png"), img)
            self._rows.append([idx, datetime.now().isoformat(timespec="milliseconds"),
                               f"{fs.mono:.4f}", f"{fs.device_ts_ms:.2f}",
                               fs.frame_number])
            self.next_idx = idx + 1
            self._write_csv()
        print(f"[shot] saved {stem} ({self.count} total)")
        return stem

    def undo(self) -> str | None:
        with self._lock:
            if not self._rows:
                return None
            idx = int(self._rows[-1][0])
            stem = f"{idx:06d}"
            for p in self.out_dir.glob(f"{stem}_*"):
                p.unlink()
            self._rows.pop()
            self.next_idx = idx
            self._write_csv()
        print(f"[shot] deleted {stem}")
        return stem

    def last_stem(self) -> str | None:
        with self._lock:
            return f"{int(self._rows[-1][0]):06d}" if self._rows else None

    def _write_csv(self) -> None:
        with open(self.out_dir / "shots.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.CSV_HEADER)
            w.writerows(self._rows)


# ---------------------------------------------------------------------------
# web UI

_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>piper-human</title>
<style>
 body{margin:0;background:#101014;color:#e8e8ee;font:16px/1.4 system-ui,sans-serif}
 main{max-width:980px;margin:0 auto;padding:28px 24px}
 #state{font-size:48px;font-weight:700;font-variant-numeric:tabular-nums;color:#5ade7a}
 #state.flash{color:#fff}
 #sub{color:#9a9aa6;margin-top:6px;font-variant-numeric:tabular-nums}
 #err{color:#ffbe3c;min-height:1.4em;margin-top:8px}
 #btns{margin-top:18px;display:flex;gap:12px}
 button{font:600 18px system-ui;padding:12px 30px;border-radius:10px;border:0;cursor:pointer}
 #shoot{background:#1d5c31;color:#d9ffe4;font-size:22px;padding:14px 44px}
 #undo{background:#1d1d24;color:#9a9aa6;border:1px solid #2a2a32}
 button:active{transform:translateY(1px)}
 #cams{display:flex;gap:14px;flex-wrap:wrap;margin-top:26px}
 figure{margin:0}
 figcaption{color:#9a9aa6;font-size:13px;margin-top:5px;font-family:ui-monospace,monospace}
 img{width:440px;max-width:100%;border-radius:6px;background:#000}
 #meta{margin-top:26px;color:#9a9aa6;font-size:13px;font-family:ui-monospace,monospace;
   overflow-wrap:anywhere}
 kbd{background:#1d1d24;border:1px solid #2a2a32;border-radius:4px;padding:1px 6px}
</style></head><body><main>
 <div id="state">--</div>
 <div id="sub"></div><div id="err"></div>
 <div id="btns">
  <button id="shoot">shoot</button>
  <button id="undo">undo last</button>
 </div>
 <div id="cams">__CAM_FIGURES__</div>
 <div id="meta"></div>
 <p style="color:#63636e;font-size:13px"><kbd>space</kbd> shoot &nbsp; <kbd>u</kbd> undo</p>
<script>
const CAMS = __CAM_LIST__;
async function control(action){
  try{ await fetch('/control', {method:'POST', headers:{'Content-Type':'application/json'},
                               body: JSON.stringify({action})}); }catch(e){}
  if (action === 'shoot'){
    const el = document.getElementById('state');
    el.classList.add('flash'); setTimeout(() => el.classList.remove('flash'), 150);
  }
  tickOnce();
}
document.getElementById('shoot').onclick = () => control('shoot');
document.getElementById('undo').onclick = () => control('undo');
document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  if (e.code === 'Space'){ e.preventDefault(); control('shoot'); }
  if (e.key === 'u') control('undo');
});
async function tickOnce(){
  try{
    const s = await (await fetch('/status')).json();
    document.getElementById('state').textContent = `${s.shots} shots`;
    const parts = [`grab ${s.grab_hz.toFixed(1)} Hz`];
    if (s.last_shot) parts.unshift(`last: ${s.last_shot}`);
    document.getElementById('sub').textContent = parts.join(' · ');
    document.getElementById('err').textContent = s.error || '';
    document.getElementById('meta').textContent =
      `${s.out_dir} — ${s.device} @${s.fps}fps usb${s.usb}` +
      (s.aligned ? ' — depth aligned to color' : ' — raw depth');
  }catch(e){ document.getElementById('err').textContent = 'connection lost'; }
}
async function tick(){ await tickOnce(); setTimeout(tick, 500); }
tick();
// Views are MJPEG streams — the browser renders them on its own, so there is
// no refresh timer here. If one drops (server restart), re-point it.
for (const c of CAMS){
  const img = document.getElementById('cam-'+c);
  if (img) img.onerror = () => setTimeout(() => { img.src = '/stream/'+c+'.mjpg?r='+Date.now(); }, 1000);
}
</script></main></body></html>
"""


class SnapshotServer:
    def __init__(self, capture: RealSenseCapture, store: ShotStore,
                 host: str, port: int):
        self.capture = capture
        self.store = store
        self.host, self.port = host, port
        self.error = ""
        self._httpd: ThreadingHTTPServer | None = None
        self._jpeg_cache: dict[str, tuple[int, bytes]] = {}
        self._jpeg_lock = threading.Lock()

    def _control(self, action: str) -> None:
        if action == "shoot":
            fs = self.capture.latest()
            if fs is None:
                self.error = "no frame available"
                return
            self.error = ""
            self.store.shoot(fs)
        elif action == "undo":
            self.store.undo()

    def _status(self) -> dict:
        return {
            "shots": self.store.count,
            "last_shot": self.store.last_stem(),
            "grab_hz": self.capture.grab_hz(),
            "out_dir": str(self.store.out_dir),
            "device": f"RealSense {self.capture.serial}",
            "fps": self.capture.fps,
            "usb": self.capture.usb,
            "aligned": self.capture.align_enabled,
            "error": self.error,
        }

    def _encode(self, name: str, fs: FrameSet) -> bytes | None:
        """JPEG for one view, memoized per (view, frame) so N viewers of the
        same frame cost one encode."""
        import cv2

        with self._jpeg_lock:
            hit = self._jpeg_cache.get(name)
            if hit is not None and hit[0] == fs.frame_number:
                return hit[1]

        if name == "color":
            bgr = cv2.cvtColor(fs.color, cv2.COLOR_RGB2BGR)
        elif name == "depth":
            bgr = colorize_depth(fs.depth)
        elif name.startswith("ir") and fs.ir:
            idx = int(name[2:]) - 1
            if idx >= len(fs.ir):
                return None
            bgr = cv2.cvtColor(fs.ir[idx], cv2.COLOR_GRAY2BGR)
        else:
            return None
        ok, buf = cv2.imencode(".jpg", bgr,
                               [int(cv2.IMWRITE_JPEG_QUALITY), PREVIEW_QUALITY])
        if not ok:
            return None
        data = buf.tobytes()
        with self._jpeg_lock:
            self._jpeg_cache[name] = (fs.frame_number, data)
        return data

    def _cam_jpeg(self, name: str) -> bytes | None:
        fs = self.capture.latest()
        return self._encode(name, fs) if fs is not None else None

    def _stream(self, req: BaseHTTPRequestHandler, name: str) -> None:
        """multipart/x-mixed-replace: one connection, frames pushed as they
        arrive. Replaces per-frame polling — no reconnects, no img.src
        flicker, and the browser paces itself."""
        if name not in ("color", "depth", "ir1", "ir2"):
            self._send(req, 404, "text/plain", b"not found")
            return
        boundary = "frame"
        req.send_response(200)
        req.send_header("Content-Type",
                        f"multipart/x-mixed-replace; boundary={boundary}")
        req.send_header("Cache-Control", "no-store")
        req.end_headers()
        last_id, period = -1, 1.0 / PREVIEW_FPS
        next_due = time.monotonic()
        while True:
            fs = self.capture.wait_next(last_id)
            if fs is None:
                continue          # camera hiccup; keep the connection open
            last_id = fs.frame_number
            now = time.monotonic()
            if now < next_due:    # throttle to PREVIEW_FPS
                continue
            next_due = now + period
            data = self._encode(name, fs)
            if data is None:
                continue
            req.wfile.write(
                f"--{boundary}\r\nContent-Type: image/jpeg\r\n"
                f"Content-Length: {len(data)}\r\n\r\n".encode())
            req.wfile.write(data)
            req.wfile.write(b"\r\n")   # raises when the tab closes -> thread exits

    # -- http ----------------------------------------------------------------

    def serve_forever(self) -> None:
        server = self
        cams = ["color", "depth"] + (["ir1", "ir2"] if self.capture.ir_enabled else [])

        class Handler(BaseHTTPRequestHandler):
            # keep-alive: HTTP/1.0 (the stdlib default) tears down the TCP
            # connection after every response, which is what made the polling
            # UI stutter. Every response here sends an accurate Content-Length.
            protocol_version = "HTTP/1.1"
            # TCP_NODELAY. Headers and body go out as separate send() calls, so
            # with Nagle on, each response waits on the peer's delayed ACK —
            # a flat ~40 ms per request on loopback. This is the single
            # biggest win for UI responsiveness.
            disable_nagle_algorithm = True

            def log_message(self, *args):
                pass

            def do_GET(self):
                try:
                    server._get(self, cams)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_POST(self):
                try:
                    server._post(self)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._httpd.daemon_threads = True
        self.port = self._httpd.server_address[1]
        print(f"[web] snapshot UI: http://{self.host}:{self.port}")
        try:
            self._httpd.serve_forever()
        except KeyboardInterrupt:
            pass

    def shutdown(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()

    def _get(self, req: BaseHTTPRequestHandler, cams: list[str]) -> None:
        path = req.path.split("?", 1)[0]
        if path == "/":
            figures = "".join(
                f'<figure><img id="cam-{n}" src="/stream/{n}.mjpg" alt="{n}">'
                f'<figcaption>{n}</figcaption></figure>'
                for n in cams)
            page = (_PAGE.replace("__CAM_FIGURES__", figures)
                         .replace("__CAM_LIST__", json.dumps(cams)))
            self._send(req, 200, "text/html; charset=utf-8", page.encode())
        elif path == "/status":
            self._send(req, 200, "application/json", json.dumps(self._status()).encode())
        elif path.startswith("/stream/") and path.endswith(".mjpg"):
            self._stream(req, path[len("/stream/"):-len(".mjpg")])
        elif path.startswith("/cam/") and path.endswith(".jpg"):
            data = self._cam_jpeg(path[len("/cam/"):-len(".jpg")])
            if data is None:
                self._send(req, 404, "text/plain", b"no frame")
            else:
                self._send(req, 200, "image/jpeg", data)
        else:
            self._send(req, 404, "text/plain", b"not found")

    def _post(self, req: BaseHTTPRequestHandler) -> None:
        if req.path.split("?", 1)[0] != "/control":
            self._send(req, 404, "text/plain", b"not found")
            return
        length = int(req.headers.get("Content-Length") or 0)
        try:
            action = json.loads(req.rfile.read(length) or b"{}").get("action", "")
        except json.JSONDecodeError:
            action = ""
        if action not in ("shoot", "undo"):
            self._send(req, 400, "text/plain", b"bad action")
            return
        self._control(action)
        self._send(req, 200, "application/json", json.dumps(self._status()).encode())

    @staticmethod
    def _send(req: BaseHTTPRequestHandler, code: int, ctype: str, body: bytes) -> None:
        req.send_response(code)
        req.send_header("Content-Type", ctype)
        req.send_header("Content-Length", str(len(body)))
        req.send_header("Cache-Control", "no-store")
        req.end_headers()
        req.wfile.write(body)


# ---------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--serial", default=DEFAULT_SERIAL,
                    help="RealSense serial (default: first connected device)")
    ap.add_argument("--root", default="~/piper_datasets", help="dataset root directory")
    ap.add_argument("--name", default=None,
                    help="dataset dir name (default: human_<YYYYMMDD>)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=None,
                    help="default: 30 on USB3, 15 on USB2 (auto)")
    ap.add_argument("--raw-depth", action="store_true",
                    help="store native depth instead of aligning it to color")
    ap.add_argument("--ir", action="store_true",
                    help="also save both IR cameras per shot (needs USB3 bandwidth)")
    ap.add_argument("--host", default="127.0.0.1",
                    help="web UI bind address (0.0.0.0 for LAN access)")
    ap.add_argument("--port", type=int, default=8790)
    args = ap.parse_args()

    name = args.name or f"human_{datetime.now():%Y%m%d}"
    out_dir = Path(args.root).expanduser() / name
    out_dir.mkdir(parents=True, exist_ok=True)

    capture = RealSenseCapture(args.serial, args.width, args.height, args.fps,
                               align=not args.raw_depth, ir=args.ir)
    (out_dir / "calib.json").write_text(json.dumps(capture.calibration(), indent=2))
    print(f"[main] calib.json written; shots dir: {out_dir}")

    store = ShotStore(out_dir)
    server = SnapshotServer(capture, store, args.host, args.port)
    try:
        server.serve_forever()
    finally:
        print("[main] shutting down")
        capture.stop()
        print(f"[main] {store.count} shots in {out_dir}")


if __name__ == "__main__":
    main()
