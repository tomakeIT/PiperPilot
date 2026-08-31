"""Clutch-based teleop: map Quest controller deltas to absolute EEF targets.

Scheme (standard VR teleop):
- Hold the grip (squeeze) button on the control hand to ENGAGE the clutch.
  On engage, the controller pose and the current arm EEF pose are latched.
- While engaged, controller displacement/rotation (mapped XR->robot frame)
  is applied on top of the latched EEF pose and streamed via move_p.
- Release the clutch to freeze the arm; reposition your hand freely and
  re-engage (ratcheting through the workspace).
- The trigger analog drives the gripper (pressed = closed).

Safety: one-euro filtered input, workspace box clamp, per-tick velocity
caps, pitch guard (move_p rejects |pitch| > 90 deg), stale-stream watchdog.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from . import xr_math
from .limits import LimitTracker
from .piper_arm import DEFAULT_HOME_JOINTS, ArmBase
from .quest_client import QuestReader


@dataclass
class TeleopStatus:
    engaged: bool = False
    tracked: bool = False
    stale: bool = False
    homing: bool = False
    target_pose6: np.ndarray = field(default_factory=lambda: np.zeros(6))
    gripper_width: float = 0.0
    ticks: int = 0
    last_error: str = ""


@dataclass
class ButtonEvents:
    """Rising-edge events sampled by the control loop."""

    primary: bool = False    # A/X on control hand -> toggle recording
    secondary: bool = False  # B/Y on control hand -> discard episode
    menu: bool = False       # left menu -> disengage + hold
    delete_hold: bool = False  # non-control hand X/A held >= 1 s -> discard
                               # in-flight episode / delete last saved one


class TeleopController:
    """Runs the control loop in a thread; exposes latched action for recording."""

    def __init__(self, quest: QuestReader, arm: ArmBase, cfg):
        self.quest = quest
        self.arm = arm
        self.cfg = cfg
        self.hand = cfg.quest.control_hand
        t = cfg.teleop
        self.rate_hz = float(t.rate_hz)
        self.pos_scale = float(t.pos_scale)
        self.engage_thr = float(t.clutch_engage)
        self.release_thr = float(t.clutch_release)
        self.map_m = xr_math.xr_to_robot_matrix(math.radians(float(t.yaw_offset_deg)))
        self.ws_min = np.array(t.limits.workspace_min, dtype=float)
        self.ws_max = np.array(t.limits.workspace_max, dtype=float)
        self.max_lin_step = float(t.limits.max_lin_vel) / self.rate_hz
        self.max_ang_step = float(t.limits.max_ang_vel) / self.rate_hz
        self.pitch_max = math.radians(float(t.limits.pitch_abs_max_deg))
        self.max_cmd_deviation = float(t.limits.max_cmd_deviation)
        self.stale_timeout = float(cfg.quest.stale_timeout_s)
        f = t.filter
        self._pos_filter = xr_math.OneEuroFilter(f.min_cutoff, f.beta, f.d_cutoff)
        self._quat_filter = xr_math.QuatOneEuroFilter(f.min_cutoff, f.beta, f.d_cutoff)

        self.grip_max_width = float(cfg.gripper.max_width)

        self.limits = LimitTracker()
        print(f"[teleop] envelope: ws x[{self.ws_min[0]:.2f},{self.ws_max[0]:.2f}] "
              f"y[{self.ws_min[1]:.2f},{self.ws_max[1]:.2f}] "
              f"z[{self.ws_min[2]:.2f},{self.ws_max[2]:.2f}] m | "
              f"vel {t.limits.max_lin_vel} m/s, {t.limits.max_ang_vel} rad/s | "
              f"pitch +-{t.limits.pitch_abs_max_deg} deg | "
              f"tether {self.max_cmd_deviation} m")

        self.status = TeleopStatus()
        self._engaged = False
        # Latched frames at clutch engage.
        self._c0_pos = np.zeros(3)
        self._c0_rot = np.eye(3)
        self._e0_pos = np.zeros(3)
        self._e0_rot = np.eye(3)
        # Current absolute command (also the recorded action).
        self._lock = threading.Lock()
        self._cmd_pose6: np.ndarray | None = None
        self._cmd_grip: float | None = None
        self._prev_buttons = {"primary": False, "secondary": False, "menu": False}
        self._events_lock = threading.Lock()
        self._pending_events = ButtonEvents()

        # Homing: hold the NON-control hand's secondary button (Y/B) >= 1 s
        # while disengaged, or call request_home() (keyboard 'h').
        self._home_joints = list(cfg.arm.get("home_joints") or DEFAULT_HOME_JOINTS)
        self._home_request = False
        self._homing_deadline = 0.0
        self._home_hold_start: float | None = None
        self._home_hold_fired = False
        # Delete gesture: NON-control hand's primary button (X/A) held >= 1 s.
        self._del_hold_start: float | None = None
        self._del_hold_fired = False

        self._running = False
        self._thread: threading.Thread | None = None

    # -- public API ------------------------------------------------------------

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="teleop-ctl", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def latest_action(self) -> tuple[np.ndarray | None, float | None]:
        """Absolute commanded (pose6, gripper width); None until first command."""
        with self._lock:
            pose = None if self._cmd_pose6 is None else self._cmd_pose6.copy()
            return pose, self._cmd_grip

    def poll_events(self) -> ButtonEvents:
        """Fetch-and-clear rising-edge button events."""
        with self._events_lock:
            ev = self._pending_events
            self._pending_events = ButtonEvents()
            return ev

    def input_rate_hz(self) -> float:
        return self.quest.rate_hz()

    def request_home(self) -> None:
        """Move the arm to its home joints (executed by the control loop when
        the clutch is disengaged)."""
        self._home_request = True

    # -- control loop ------------------------------------------------------------

    def _loop(self) -> None:
        period = 1.0 / self.rate_hz
        next_t = time.monotonic()
        while self._running:
            next_t += period
            try:
                self._tick()
            except Exception as e:  # never let one bad tick kill the loop
                self.status.last_error = f"tick: {e}"
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()  # fell behind; resync

    def _tick(self) -> None:
        st = self.status
        st.ticks += 1

        # Homing runs regardless of the quest stream (keyboard-triggered too).
        if self._handle_homing():
            return

        qs = self.quest.get()
        stale = self.quest.age() > self.stale_timeout
        st.stale = stale
        if qs is None or stale:
            self._disengage("stream stale")
            return

        hand = qs.hand(self.hand)
        st.tracked = hand.tracked
        self._emit_button_events(qs)
        self._detect_home_gesture(qs)
        self._detect_delete_gesture(qs)

        # Gripper always live while the hand is tracked (independent of clutch).
        if hand.valid:
            width = self.grip_max_width * (1.0 - float(hand.trigger))
            try:
                self.arm.command_gripper(width)
            except Exception as e:
                st.last_error = f"gripper: {e}"
            else:
                with self._lock:
                    self._cmd_grip = width
                st.gripper_width = width

        if not (hand.valid and hand.tracked):
            self._disengage("hand not tracked")
            return

        # Filter the raw controller pose (in XR space).
        now = time.monotonic()
        c_pos = self._pos_filter(hand.pos, now)
        c_quat = self._quat_filter(hand.quat_xyzw, now)
        c_rot = xr_math.quat_xyzw_to_matrix(c_quat)

        # Clutch logic with hysteresis.
        menu_pressed = qs.left.menu or qs.right.menu
        if self._engaged:
            if hand.squeeze < self.release_thr or menu_pressed:
                self._disengage("clutch released")
                return
        else:
            if hand.squeeze > self.engage_thr and not menu_pressed:
                arm_state = self.arm.get_state()
                if not arm_state.eef_valid:
                    st.last_error = "no EEF feedback; cannot engage"
                    return
                if arm_state.arm_status not in (0, -1):
                    st.last_error = (f"arm fault status={arm_state.arm_status} "
                                     "— run `make home` first")
                    return
                # Folded/rest pose sits outside the workspace box; Cartesian
                # targets from there get rejected by the firmware (status 4).
                if (np.any(arm_state.eef_pos < self.ws_min - 0.02)
                        or np.any(arm_state.eef_pos > self.ws_max + 0.02)):
                    st.last_error = "EEF outside workspace — run `make home` first"
                    return
                self._c0_pos = c_pos.copy()
                self._c0_rot = c_rot.copy()
                # Latch priority: the arm's actively-held target (impedance
                # backends sag below it — latching the measured pose would
                # ratchet the arm down on every engage), then our last
                # command, then the measured pose.
                held = self.arm.held_target_pose6()
                with self._lock:
                    cmd = self._cmd_pose6
                if held is not None:
                    e0 = held
                elif cmd is not None:
                    e0 = cmd
                else:
                    e0 = arm_state.eef_pose6
                self._e0_pos, self._e0_rot = xr_math.pose6_to_matrix(e0)
                self._engaged = True
                st.engaged = True
            return  # first engaged tick commands next round

        # --- engaged: compute absolute target ---
        # Arm fault check: stop streaming immediately on any non-normal status.
        arm_state = self.arm.get_state()
        if arm_state.arm_status not in (0, -1):  # -1 = no status feedback yet
            self._disengage(f"arm fault status={arm_state.arm_status}")
            return

        dp = self.map_m @ (c_pos - self._c0_pos) * self.pos_scale
        target_pos = self._e0_pos + dp
        dr = self.map_m @ c_rot @ self._c0_rot.T @ self.map_m.T
        target_rot = dr @ self._e0_rot

        # Workspace + per-tick step limits relative to previous command.
        with self._lock:
            prev = self._cmd_pose6
        if prev is not None:
            prev_pos, prev_rot = xr_math.pose6_to_matrix(prev)
            step = target_pos - prev_pos
            n = float(np.linalg.norm(step))
            if n > self.max_lin_step:
                target_pos = prev_pos + step * (self.max_lin_step / n)
                self.limits.hit("vel", f"{n * self.rate_hz:.1f}m/s")
            ang = math.acos(max(-1.0, min(1.0,
                (float(np.trace(target_rot @ prev_rot.T)) - 1.0) / 2.0)))
            if ang > self.max_ang_step:
                self.limits.hit("angvel", f"{ang * self.rate_hz:.1f}rad/s")
            target_rot = xr_math.clamp_angle_step(prev_rot, target_rot, self.max_ang_step)
        below = target_pos < self.ws_min - 1e-9
        above = target_pos > self.ws_max + 1e-9
        if below.any() or above.any():
            self.limits.hit("ws", " ".join(
                "xyz"[i] + ("-" if below[i] else "+")
                for i in range(3) if below[i] or above[i]))
        target_pos = np.clip(target_pos, self.ws_min, self.ws_max)

        # Deviation guard: never command far from where the arm actually is
        # (prevents a catch-up lunge when the target ran ahead of the arm,
        # e.g. dragged against a workspace edge or an unreachable pose).
        if arm_state.eef_valid:
            dev = target_pos - arm_state.eef_pos
            dn = float(np.linalg.norm(dev))
            if dn > self.max_cmd_deviation:
                target_pos = arm_state.eef_pos + dev * (self.max_cmd_deviation / dn)
                self.limits.hit("tether", f"{dn * 1000:.0f}mm")

        roll, pitch, yaw = xr_math.matrix_to_rpy(target_rot)
        if abs(pitch) > self.pitch_max:
            self.limits.hit("pitch", "+" if pitch > 0 else "-")
        pitch = float(np.clip(pitch, -self.pitch_max, self.pitch_max))
        pose6 = np.array([target_pos[0], target_pos[1], target_pos[2], roll, pitch, yaw])

        try:
            self.arm.command_eef(pose6.tolist())
        except Exception as e:  # keep the loop alive; surface the error
            st.last_error = f"command_eef: {e}"
            return
        with self._lock:
            self._cmd_pose6 = pose6
        st.target_pose6 = pose6
        st.engaged = True

    def _handle_homing(self) -> bool:
        """Returns True while a homing move is in flight (blocks teleop)."""
        st = self.status
        now = time.monotonic()
        if self._home_request:
            self._home_request = False
            if self._engaged:
                st.last_error = "release the clutch before homing"
            else:
                try:
                    self.arm.move_joints(self._home_joints)
                except Exception as e:
                    st.last_error = f"homing: {e}"
                    return False
                self._homing_deadline = now + 10.0
                with self._lock:
                    self._cmd_pose6 = None  # target is stale after homing
                st.homing = True
                st.last_error = "homing…"
        if st.homing:
            arm_state = self.arm.get_state()
            done = (arm_state.joints_valid and float(np.max(np.abs(
                arm_state.joints - np.asarray(self._home_joints)))) < 0.03)
            if done or now > self._homing_deadline:
                st.homing = False
                st.last_error = "home reached" if done else "homing timed out"
                self.quest.send_haptic("both", amp=0.4, ms=150)
            return True
        return False

    def _detect_delete_gesture(self, qs) -> None:
        """Non-control hand primary (X/A) held >= 1 s -> emit a delete_hold
        event (the recorder discards the in-flight episode, or deletes the
        last one saved this session)."""
        other = qs.hand("left" if self.hand == "right" else "right")
        now = time.monotonic()
        if other.primary:
            if self._del_hold_start is None:
                self._del_hold_start = now
            elif not self._del_hold_fired and now - self._del_hold_start >= 1.0:
                self._del_hold_fired = True
                with self._events_lock:
                    self._pending_events.delete_hold = True
        else:
            self._del_hold_start = None
            self._del_hold_fired = False

    def _detect_home_gesture(self, qs) -> None:
        """Non-control hand secondary (Y/B) held >= 1 s while disengaged."""
        other = qs.hand("left" if self.hand == "right" else "right")
        now = time.monotonic()
        if other.secondary and not self._engaged:
            if self._home_hold_start is None:
                self._home_hold_start = now
            elif not self._home_hold_fired and now - self._home_hold_start >= 1.0:
                self._home_hold_fired = True
                self.quest.send_haptic(
                    "left" if self.hand == "right" else "right", amp=0.9, ms=300)
                self.request_home()
        else:
            self._home_hold_start = None
            self._home_hold_fired = False

    def _disengage(self, reason: str) -> None:
        if self._engaged:
            self.status.last_error = reason
        self._engaged = False
        self.status.engaged = False
        self._pos_filter.reset()
        self._quat_filter.reset()
        # No new move_p commands are sent; firmware holds the last target.
        # Invalidate the latched command: after a pause the arm may have been
        # moved externally or faulted, so the next engage must latch onto the
        # MEASURED pose instead of snapping back to a stale target (the
        # recorder falls back to the measured state while this is None).
        with self._lock:
            self._cmd_pose6 = None

    def _emit_button_events(self, qs) -> None:
        hand = qs.hand(self.hand)
        cur = {
            "primary": hand.primary,
            "secondary": hand.secondary,
            "menu": qs.left.menu or qs.right.menu,
        }
        with self._events_lock:
            for k, v in cur.items():
                if v and not self._prev_buttons[k]:
                    setattr(self._pending_events, k, True)
        self._prev_buttons = cur
