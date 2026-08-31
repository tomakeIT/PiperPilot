"""Rerun-based live visualization of quest poses / arm state (rerun >= 0.23 API)."""

from __future__ import annotations

import numpy as np

from . import xr_math
from .piper_arm import ArmState
from .quest_client import QuestState

AXIS_COLORS = ([230, 60, 60], [60, 200, 60], [70, 110, 240])  # x red, y green, z blue


class RerunViz:
    def __init__(self, app_id: str = "piper_teleop", spawn: bool = True, serve: bool = False):
        import os
        import shutil
        import sys

        import rerun as rr

        self.rr = rr
        rr.init(app_id)
        if spawn and not serve:
            # The pip-installed viewer binary lives next to the interpreter;
            # absolute-path invocations (Makefile / scripts) don't have the
            # conda env's bin on PATH.
            env_bin = os.path.dirname(sys.executable)
            if (shutil.which("rerun") is None
                    and os.path.exists(os.path.join(env_bin, "rerun"))):
                os.environ["PATH"] = env_bin + os.pathsep + os.environ.get("PATH", "")
            if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
                print("[viz] no display detected (SSH/headless) — "
                      "serving the web viewer instead")
                serve = True
        if serve:
            self._serve_web()
        elif spawn:
            try:
                rr.spawn()
            except Exception as e:
                print(f"[viz] cannot spawn the native viewer ({e}) — "
                      "serving the web viewer instead")
                self._serve_web()
        self._trail: dict[str, list[np.ndarray]] = {}
        self._setup_scene()

    def _serve_web(self) -> None:
        uri = self.rr.serve_grpc()
        self.rr.serve_web_viewer(open_browser=False, connect_to=uri)
        print(f"[viz] rerun web viewer: http://localhost:9090/?url={uri}")

    def _setup_scene(self) -> None:
        rr = self.rr
        rr.log("xr", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)
        rr.log("robot", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        lines = []
        for i in range(-5, 6):
            lines.append([[i * 0.2, -1.0, 0.0], [i * 0.2, 1.0, 0.0]])
            lines.append([[-1.0, i * 0.2, 0.0], [1.0, i * 0.2, 0.0]])
        rr.log("robot/floor", rr.LineStrips3D(lines, colors=[[80, 80, 80]] * len(lines)),
               static=True)

    def _log_axes(self, path: str, pos: np.ndarray, rot: np.ndarray, scale: float = 0.08):
        """Pose as an RGB axis triad (version-robust: plain line strips)."""
        strips = [[pos, pos + rot[:, i] * scale] for i in range(3)]
        self.rr.log(path, self.rr.LineStrips3D(
            [np.asarray(s) for s in strips], colors=list(AXIS_COLORS), radii=0.004))

    def _log_trail(self, path: str, pos: np.ndarray, maxlen: int = 300):
        trail = self._trail.setdefault(path, [])
        trail.append(pos.copy())
        if len(trail) > maxlen:
            del trail[: len(trail) - maxlen]
        if len(trail) >= 2:
            self.rr.log(path, self.rr.LineStrips3D(
                [np.array(trail)], colors=[[200, 200, 60]], radii=0.002))

    def log_quest(self, qs: QuestState) -> None:
        rr = self.rr
        for name, hand in (("head", qs.head), ("left", qs.left), ("right", qs.right)):
            if not hand.valid:
                continue
            rot = xr_math.quat_xyzw_to_matrix(hand.quat_xyzw)
            self._log_axes(f"xr/{name}", hand.pos, rot)
            if name != "head":
                self._log_trail(f"xr/{name}_trail", hand.pos)
                rr.log(f"inputs/{name}/trigger", rr.Scalars([hand.trigger]))
                rr.log(f"inputs/{name}/squeeze", rr.Scalars([hand.squeeze]))
        rr.log("inputs/session", rr.TextLog(f"state={qs.session_state} frame={qs.frame}"))

    def log_mapped_hand(self, pos_robot: np.ndarray, rot_robot: np.ndarray) -> None:
        """Controller pose after XR->robot mapping (sanity check of axes)."""
        self._log_axes("robot/hand_mapped", pos_robot, rot_robot, scale=0.1)

    def log_arm(self, st: ArmState, target_pose6: np.ndarray | None = None) -> None:
        rr = self.rr
        if st.eef_valid:
            pos, rot = xr_math.pose6_to_matrix(st.eef_pose6)
            self._log_axes("robot/eef_measured", pos, rot, scale=0.12)
            self._log_trail("robot/eef_trail", pos)
        if target_pose6 is not None and np.any(target_pose6):
            pos, rot = xr_math.pose6_to_matrix(target_pose6)
            self._log_axes("robot/eef_target", pos, rot, scale=0.08)
        if st.gripper_valid:
            rr.log("inputs/gripper_width", rr.Scalars([st.gripper_width]))

    def log_images(self, frames: dict[str, np.ndarray]) -> None:
        for name, rgb in frames.items():
            self.rr.log(f"cameras/{name}", self.rr.Image(rgb[::2, ::2]))
