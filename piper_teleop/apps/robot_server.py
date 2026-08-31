"""Thin robot HTTP server: hardware interface for an external inference loop.

    piper-serve                       # real arm + cameras (config backend)
    piper-serve --sim                 # fake arm + fake cameras, no hardware
    piper-serve --position            # force firmware position backend

Owns the arm, the cameras and the safety envelope; knows nothing about
policies. An external inference process drives it over localhost HTTP:

  GET  /health          {"status": "ok"|"fault", "cameras": [...], "sim": bool}
  GET  /status          executor counters (cheap, no images — poll this)
  GET  /obs             observation (infer.py build_obs schema, images as
                        base64 jpeg) + the same status block
  POST /chunk           {"actions": [[x,y,z,qw,qx,qy,qz,grip], ...],
                         "replace": bool}  -> queued for the 30 Hz executor;
                        replace=True drops not-yet-executed actions first
                        (receding-horizon handover). Non-blocking.
  POST /home            {"joints": [6]? } -> blocking firmware-planned move
                        to home_joints + settle; clears the queue first.
  POST /stop            clear queue, hold current pose.

Executor semantics: one action per tick at recording.fps (30 Hz); position
workspace-clamped like infer.py; arm fault (arm_status not in (0, -1))
latches a fault state and stops consuming until /home clears it. An empty
queue holds the last pose (starved ticks are counted and reported so the
client can detect pacing bugs).
"""

from __future__ import annotations

import argparse
import base64
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from .. import xr_math
from ..cameras import make_cameras
from ..config import load_config
from ..piper_arm import DEFAULT_HOME_JOINTS, make_arm
from .replay import pose7_to_pose6

HOME_TOL_RAD = 0.03
HOME_TIMEOUT_S = 25.0


class RobotServer:
    def __init__(self, cfg, sim: bool, jpeg_quality: int = 85):
        self.cfg = cfg
        self.sim = sim
        self.jpeg_quality = jpeg_quality
        self.cameras = make_cameras(cfg, fake=sim)
        self.arm = make_arm(cfg)
        time.sleep(2.0)  # RealSense / MIT settle (same as infer.py)

        self.ws_min = np.array(cfg.teleop.limits.workspace_min)
        self.ws_max = np.array(cfg.teleop.limits.workspace_max)
        self.period = 1.0 / float(cfg.recording.fps)
        self.home_joints = list(cfg.arm.home_joints or DEFAULT_HOME_JOINTS)

        self._lock = threading.Lock()       # queue/counter state
        self._arm_lock = threading.Lock()   # serialize CAN access (executor vs HTTP threads)
        self._queue: deque[np.ndarray] = deque()
        self._pushed = 0          # total actions ever accepted
        self._executed = 0        # total actions ever executed
        self._starved = 0         # active ticks that found an empty queue
        self._active = False      # between first /chunk and /stop|/home|fault
        self._homing = False
        self._fault: str | None = None
        self._arm_status = -1

        self._runner = threading.Thread(target=self._exec_loop,
                                        name="robot-executor", daemon=True)
        self._runner.start()

    # -- 30 Hz executor ------------------------------------------------------

    def _exec_loop(self) -> None:
        next_t = time.monotonic()
        while True:
            with self._lock:
                action = None
                if not self._homing and self._fault is None and self._queue:
                    action = self._queue.popleft()
                elif self._active and not self._homing and self._fault is None:
                    self._starved += 1

            if action is not None:
                pos = np.clip(action[:3], self.ws_min, self.ws_max)
                pose7 = np.concatenate([pos, action[3:7]])
                with self._arm_lock:
                    self.arm.command_eef(pose7_to_pose6(pose7))
                    self.arm.command_gripper(float(action[7]))
                    st = self.arm.get_state()
                with self._lock:
                    self._executed += 1
                    self._arm_status = int(st.arm_status)
                    if st.arm_status not in (0, -1):
                        self._fault = f"arm_status={st.arm_status}"
                        self._queue.clear()
                        self._active = False
                        print(f"\n[serve] FAULT: {self._fault} — queue cleared, "
                              "POST /home to recover")

            next_t += self.period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    # -- request handlers ----------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            return {
                "pushed": self._pushed,
                "executed": self._executed,
                "pending": len(self._queue),
                "starved_steps": self._starved,
                "active": self._active,
                "homing": self._homing,
                "fault": self._fault,
                "arm_status": self._arm_status,
            }

    def obs(self) -> dict:
        import cv2  # lazy, matches package convention

        with self._arm_lock:
            st = self.arm.get_state()
        out = {
            "state": {
                "observation.arm_joint": st.joints.tolist(),
                "observation.arm_eef": np.concatenate(
                    [st.eef_pos, st.eef_quat_wxyz]).tolist(),
                "observation.gripper": [st.gripper_width],
            },
            "images": {},
            # capture age per camera (ms) so the client can detect a FROZEN
            # feed: a disconnected RealSense keeps returning its last frame
            # from latest() — presence alone can't catch that.
            "image_age_ms": {},
            "mono": st.mono,
            "status": self.status(),
        }
        now = time.monotonic()
        for cam in self.cameras:
            fr = cam.latest()
            if fr is None:
                continue
            ok, jpeg = cv2.imencode(
                ".jpg", cv2.cvtColor(fr.rgb, cv2.COLOR_RGB2BGR),
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                out["images"][cam.name] = base64.b64encode(jpeg.tobytes()).decode()
                out["image_age_ms"][cam.name] = round((now - fr.mono) * 1000.0, 1)
        return out

    def push_chunk(self, actions: list, replace: bool) -> dict:
        arr = np.asarray(actions, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 8 or not np.isfinite(arr).all():
            raise ValueError(f"actions must be finite (N, 8), got {arr.shape}")
        with self._lock:
            if self._fault is not None:
                raise RuntimeError(f"fault latched ({self._fault}); /home first")
            if self._homing:
                raise RuntimeError("homing in progress")
            if replace:
                self._queue.clear()
            first_index = self._pushed
            for row in arr:
                self._queue.append(row)
            self._pushed += len(arr)
            self._active = True
        return {"accepted": int(len(arr)), "first_index": first_index}

    def home(self, joints: list | None, open_gripper: bool = True) -> dict:
        target = np.asarray(joints or self.home_joints, dtype=float)
        with self._lock:
            self._queue.clear()
            self._active = False
            self._homing = True
        try:
            # MIT backend exits to firmware position-hold inside move_joints,
            # so the keepalive cannot re-stream the pre-home target.
            with self._arm_lock:
                self.arm.move_joints(target.tolist())
            deadline = time.monotonic() + HOME_TIMEOUT_S
            while time.monotonic() < deadline:
                with self._arm_lock:
                    st = self.arm.get_state()
                if st.joints_valid and float(np.max(np.abs(st.joints - target))) < HOME_TOL_RAD:
                    if open_gripper:
                        # Release AFTER reaching home so a still-held object
                        # drops at a predictable spot; episodes must start with
                        # the gripper open like every demo does (demos rest at
                        # max_width: trigger released = 0.07, feedback ~0.0696).
                        # The command is fire-and-forget and 50 Hz rate-limited,
                        # so a single send can be dropped or die mid-travel —
                        # repeat until the measured width confirms it opened.
                        # command_gripper clamps to max_width - open_margin
                        # (end-stop stall protection) — confirm against the
                        # clamped target, not the nominal max.
                        open_w = float(self.cfg.gripper.max_width)
                        margin = float(self.cfg.gripper.get("open_margin", 0.002))
                        open_deadline = time.monotonic() + 2.0
                        while time.monotonic() < open_deadline:
                            with self._arm_lock:
                                self.arm.command_gripper(open_w)
                                st = self.arm.get_state()
                            if st.gripper_valid and st.gripper_width >= open_w - margin - 0.004:
                                break
                            time.sleep(0.1)
                    time.sleep(0.5)  # settle (replay.py pattern)
                    with self._lock:
                        self._fault = None
                    return {"homed": True, "joints": st.joints.tolist()}
                time.sleep(0.2)
            with self._arm_lock:
                st = self.arm.get_state()
            return {"homed": False, "joints": st.joints.tolist(),
                    "error": f"not within {HOME_TOL_RAD} rad after {HOME_TIMEOUT_S}s"}
        finally:
            with self._lock:
                self._homing = False

    def stop_motion(self) -> dict:
        with self._lock:
            dropped = len(self._queue)
            self._queue.clear()
            self._active = False
        return {"stopped": True, "dropped": dropped}

    def shutdown(self) -> None:
        for cam in self.cameras:
            cam.stop()
        self.arm.stop()


def main() -> None:
    ap = argparse.ArgumentParser(description="Robot HTTP server for external inference")
    ap.add_argument("--config", default=None)
    ap.add_argument("--sim", action="store_true", help="fake arm + fake cameras")
    ap.add_argument("--position", action="store_true", help="force position backend")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8800)
    args = ap.parse_args()

    overrides: dict = {}
    if args.sim:
        overrides["arm"] = {"backend": "fake"}
    elif args.position:
        overrides["arm"] = {"backend": "piper"}
    cfg = load_config(args.config, overrides)
    server = RobotServer(cfg, sim=args.sim)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # silence per-request stderr spam
            pass

        def _reply(self, code: int, payload: dict) -> None:
            body = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            try:
                if self.path == "/health":
                    st = server.status()
                    self._reply(200, {
                        "status": "fault" if st["fault"] else "ok",
                        "sim": server.sim,
                        "cameras": [c.name for c in server.cameras],
                        "fps": 1.0 / server.period,
                    })
                elif self.path == "/status":
                    self._reply(200, server.status())
                elif self.path == "/obs":
                    self._reply(200, server.obs())
                else:
                    self._reply(404, {"error": "unknown path"})
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length") or 0)
                req = json.loads(self.rfile.read(length)) if length else {}
                if self.path == "/chunk":
                    self._reply(200, server.push_chunk(
                        req["actions"], bool(req.get("replace", False))))
                elif self.path == "/home":
                    self._reply(200, server.home(req.get("joints"),
                                                 bool(req.get("open_gripper", True))))
                elif self.path == "/stop":
                    self._reply(200, server.stop_motion())
                else:
                    self._reply(404, {"error": "unknown path"})
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:  # report to client, keep serving
                print(f"[serve] {self.path} error: {exc!r}")
                self._reply(400, {"error": repr(exc)})

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.daemon_threads = True
    print(f"[serve] robot server on http://{args.host}:{args.port}  "
          f"(sim={args.sim}, cams={[c.name for c in server.cameras]}, "
          f"{1.0 / server.period:.0f}Hz)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[serve] interrupted")
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
