"""Episode recorder: samples state/action at recording.fps (default 30 Hz),
muxes camera frames, drives episode lifecycle from controller buttons (or
keyboard).

Button UX (control hand):
    A/X  (primary)   toggle episode recording  -> short haptic buzz on start,
                                                  longer buzz on save
    B/Y  (secondary) discard current episode   -> triple short buzz
Non-control hand:
    X/A held >= 1 s  discard the in-flight episode, or delete the LAST episode
                     saved this session -> triple short buzz (weak single buzz
                     if there is nothing to delete)
Headset background color: dark blue idle, dark red while recording.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from . import xr_math
from .cameras import CameraBase
from .lerobot_writer import LeRobotWriter
from .limits import merged_active_str
from .piper_arm import ArmBase
from .quest_client import QuestReader
from .teleop_controller import TeleopController

# (r, g, b, alpha) — alpha is the tint opacity over the passthrough feed.
IDLE_COLOR = (0.0, 0.0, 0.0, 0.0)          # invisible: pure passthrough
RECORDING_COLOR = (0.6, 0.02, 0.02, 0.25)  # light red tint while recording


class Recorder:
    def __init__(self, quest: QuestReader | None, arm: ArmBase, controller,
                 cameras: list[CameraBase], writer: LeRobotWriter, cfg):
        self.quest = quest
        self.arm = arm
        self.controller = controller
        self.cameras = cameras
        self.writer = writer
        self.task = cfg.recording.task
        self.fps = float(cfg.recording.fps)

        self._episode = None
        self._ep_t0 = 0.0
        self._last_cam_ids: dict[str, int] = {}
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._last_error = ""

    # -- state -------------------------------------------------------------

    @property
    def recording(self) -> bool:
        return self._episode is not None

    def start(self) -> None:
        self._running = True
        self._set_color(*IDLE_COLOR)
        self._thread = threading.Thread(target=self._loop, name="recorder", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        with self._lock:
            if self._episode is not None:
                self.writer.finish_episode()
                self._episode = None


    # -- headset feedback (no-ops without a Quest) ---------------------------

    def _set_color(self, r: float, g: float, b: float, a: float = 0.35) -> None:
        if self.quest is not None:
            self.quest.set_color(r, g, b, a)

    def _haptic(self, amp: float, ms: int) -> None:
        if self.quest is not None:
            self.quest.send_haptic("both", amp=amp, ms=ms)

    # -- episode control (thread-safe; called from main/CLI too) ---------------

    def toggle_recording(self) -> None:
        with self._lock:
            if self._episode is None:
                self._episode = self.writer.start_episode(self.task)
                self._ep_t0 = time.monotonic()
                self._last_cam_ids = {}
                self._haptic(amp=0.8, ms=120)
                self._set_color(*RECORDING_COLOR)
                print(f"\n[rec] ▶ episode {self._episode.episode_index} started "
                      f"(task: {self.task!r})")
            else:
                ep = self._episode
                self._episode = None
                row = self.writer.finish_episode()
                self._haptic(amp=0.5, ms=200)
                self._set_color(*IDLE_COLOR)
                if row:
                    print(f"[rec] ■ episode {row['episode_index']} saved: "
                          f"{row['length']} frames "
                          f"({row['length'] / self.fps:.1f} s)")

    def discard(self) -> None:
        with self._lock:
            if self._episode is None:
                return
            self._episode = None
            self.writer.discard_episode()
            for _ in range(3):
                self._haptic(amp=1.0, ms=60)
                time.sleep(0.12)
            self._set_color(*IDLE_COLOR)
            print("[rec] ✗ episode discarded")

    def delete_last_or_discard(self) -> None:
        """Delete gesture (non-control hand X/A held 1 s, or keyboard 'x'):
        discard the in-flight episode if recording, otherwise delete the most
        recent episode saved THIS session (earlier sessions are protected)."""
        if self.recording:
            self.discard()
            return
        with self._lock:
            row = self.writer.delete_last_episode()
        if row is None:
            self._haptic(amp=0.4, ms=80)
            print("\n[rec] nothing to delete (no episode saved this session)")
            return
        for _ in range(3):
            self._haptic(amp=1.0, ms=60)
            time.sleep(0.12)
        print(f"\n[rec] ✗ episode {row['episode_index']} deleted "
              f"({row['length']} frames)")

    # -- recording.fps sampling loop ----------------------------------------------

    def _loop(self) -> None:
        period = 1.0 / self.fps
        next_t = time.monotonic()
        while self._running:
            next_t += period
            try:
                self._process_buttons()
                self._sample()
                self._last_error = ""
            except Exception as e:  # disk full, encoder error, … — stay alive
                self._last_error = f"recorder: {e}"
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    def _process_buttons(self) -> None:
        ev = self.controller.poll_events()
        if ev.primary:
            self.toggle_recording()
        if ev.secondary:
            self.discard()
        # getattr: SpaceMouse events don't carry the Quest-only delete gesture.
        if getattr(ev, "delete_hold", False):
            self.delete_last_or_discard()

    def _sample(self) -> None:
        with self._lock:
            ep = self._episode
            if ep is None:
                return
            t_rel = time.monotonic() - self._ep_t0

            st = self.arm.get_state()
            cmd_pose6, cmd_grip = self.controller.latest_action()
            # Before any teleop command exists, mirror the measured state so
            # actions stay absolute and well-defined.
            if cmd_pose6 is None:
                cmd_pose6 = st.eef_pose6.copy()
            if cmd_grip is None:
                cmd_grip = st.gripper_width

            cmd_pos, cmd_rot = xr_math.pose6_to_matrix(cmd_pose6)
            cmd_quat = xr_math.quat_xyzw_to_wxyz(xr_math.matrix_to_quat_xyzw(cmd_rot))

            ep.add_frame(t_rel, {
                "observation.arm_joint": st.joints,
                "observation.arm_eef": np.concatenate([st.eef_pos, st.eef_quat_wxyz]),
                "observation.gripper": np.array([st.gripper_width]),
                "action.arm_eef": np.concatenate([cmd_pos, cmd_quat]),
                "action.gripper": np.array([cmd_grip]),
            })

            for cam in self.cameras:
                fr = cam.latest()
                if fr is None:
                    continue
                if self._last_cam_ids.get(cam.name) == fr.frame_id:
                    continue  # no new frame since last tick
                self._last_cam_ids[cam.name] = fr.frame_id
                ep.add_video_frame(cam.name, fr.rgb, max(0.0, fr.mono - self._ep_t0))

            qs = self.quest.get() if self.quest is not None else None
            if qs is not None:
                ep.add_raw_quest({
                    "t_rel": t_rel,
                    "quest_t": qs.t,
                    "frame": qs.frame,
                    "left": {"pos": qs.left.pos.tolist(),
                             "quat_xyzw": qs.left.quat_xyzw.tolist(),
                             "trigger": qs.left.trigger, "squeeze": qs.left.squeeze},
                    "right": {"pos": qs.right.pos.tolist(),
                              "quat_xyzw": qs.right.quat_xyzw.tolist(),
                              "trigger": qs.right.trigger, "squeeze": qs.right.squeeze},
                })

    def status_image(self):
        """640x140 RGB HUD for the in-headset status panel: current state
        (REC / READY / HOMING), episode timer, and episodes-saved count."""
        import cv2
        import numpy as np

        img = np.zeros((140, 640, 3), dtype=np.uint8)
        img[:] = (18, 18, 24)
        st = self.controller.status
        ep = self._episode

        if st.homing:
            headline, color = "HOMING ...", (255, 190, 40)
        elif ep is not None:
            t = time.monotonic() - self._ep_t0
            headline = f"REC  ep{ep.episode_index}   {int(t // 60):02d}:{t % 60:04.1f}"
            color = (255, 70, 70)
            if int(t * 2) % 2 == 0:  # blinking dot
                cv2.circle(img, (600, 40), 14, (255, 60, 60), -1)
        else:
            headline, color = f"READY  -  {len(self.writer.episodes)} eps saved", (90, 230, 90)

        cv2.putText(img, headline, (18, 58), cv2.FONT_HERSHEY_SIMPLEX,
                    1.5, color, 3, cv2.LINE_AA)
        sub = (f"{len(self.writer.episodes)} saved | "
               f"{'ENGAGED' if st.engaged else 'idle'} | "
               f"input {self.controller.input_rate_hz():.0f}Hz")
        sub_color = (200, 200, 210)
        lim = merged_active_str(self.limit_groups())
        if self._last_error or st.last_error:
            sub = (self._last_error or st.last_error)[:52]
        elif lim:  # which safety clamp is shaping the motion right now
            sub = ("LIMIT " + lim)[:52]
            sub_color = (255, 190, 40)
        cv2.putText(img, sub, (18, 112), cv2.FONT_HERSHEY_SIMPLEX,
                    0.95, sub_color, 2, cv2.LINE_AA)
        return img

    def limit_groups(self) -> dict:
        """{group: LimitTracker.snapshot()} for every tracker in the stack."""
        groups = {}
        if getattr(self.controller, "limits", None) is not None:
            groups["teleop"] = self.controller.limits.snapshot()
        if getattr(self.arm, "limits", None) is not None:
            groups["arm"] = self.arm.limits.snapshot()
        return groups

    def _joint_ranges(self) -> dict | None:
        """Measured joints against the backend's joint limits, or None."""
        jl = getattr(self.arm, "joint_limits", None)
        if jl is None:
            return None
        st = self.arm.get_state()
        if not st.joints_valid:
            return None
        return {"q": [float(x) for x in st.joints],
                "lo": [float(x) for x in jl[0]],
                "hi": [float(x) for x in jl[1]]}

    def status_dict(self) -> dict:
        """JSON-able snapshot of the recorder state (web monitor, tooling)."""
        st = self.controller.status
        ep = self._episode  # local copy: may be cleared concurrently
        groups = self.limit_groups()
        return {
            "limits": groups,
            "limits_active": merged_active_str(groups),
            "joints": self._joint_ranges(),
            "state": ("homing" if st.homing
                      else "recording" if ep is not None else "ready"),
            "episode_index": ep.episode_index if ep is not None else None,
            "elapsed_s": (time.monotonic() - self._ep_t0) if ep is not None else 0.0,
            "frames": len(ep) if ep is not None else 0,
            "eps_saved": len(self.writer.episodes),
            "total_frames": self.writer.total_frames,
            "engaged": bool(st.engaged),
            "input_hz": float(self.controller.input_rate_hz()),
            "space": self._anchor_state(),
            "calib_step": self._calib_step(),
            "task": self.task,
            "root": str(self.writer.root),
            "error": (self._last_error or st.last_error or ""),
        }

    def _anchor_state(self) -> str:
        """"anchor" (pinned), "anchor-lost" (saved but not located -> axes may
        be rotated!), or the raw reporting space ("stage"/"local")."""
        if self.quest is None or (qs := self.quest.get()) is None:
            return ""
        if qs.anchor_have:
            return "anchor" if qs.space == "anchor" else "anchor-lost"
        return qs.space

    def _calib_step(self) -> int:
        if self.quest is None or (qs := self.quest.get()) is None:
            return 0
        return qs.calib_step

    def status_line(self) -> str:
        st = self.controller.status
        calib = self._calib_step()
        if calib:
            return (f"CALIBRATING X — pull trigger at point {calib}/2 "
                    "(P1→P2 line = robot +X; menu or c cancels)")
        parts = [
            f"input {self.controller.input_rate_hz():5.1f}Hz",
            "ENGAGED" if st.engaged else "idle   ",
            f"grip {st.gripper_width * 1000:4.0f}mm",
        ]
        anc = self._anchor_state()
        if anc == "anchor-lost":
            parts.append("ANCHOR LOST — axes may be rotated")
        elif anc:
            parts.append(f"[{anc}]")
        ep = self._episode  # local copy: may be cleared concurrently
        if ep is not None:
            parts.append(f"● REC ep{ep.episode_index} {len(ep)} frames")
        else:
            parts.append(f"{len(self.writer.episodes)} eps saved")
        lim = merged_active_str(self.limit_groups())
        if lim:
            parts.append(f"LIM[{lim}]")
        if self._last_error:
            parts.append(self._last_error[:40])
        elif st.last_error:
            parts.append(st.last_error[:40])
        return " | ".join(parts)
