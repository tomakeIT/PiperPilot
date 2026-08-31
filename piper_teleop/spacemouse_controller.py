"""SpaceMouse rate-control teleop: puck deflection -> EEF velocity.

Unlike the Quest clutch (position control), the SpaceMouse integrates a
velocity command: the arm moves while the puck is deflected and stops when
released. Same safety envelope as the Quest controller (workspace box,
per-tick step caps via max velocities, pitch guard).

Default axis mapping (tune signs in configs: spacemouse.axis_signs):
    puck forward  (+y_sm) -> robot +x      puck right (+x_sm) -> robot -y
    puck up       (+z_sm) -> robot +z
    tilt fwd/back (pitch) -> rotate about robot y
    tilt left/rgt (roll)  -> rotate about robot x
    twist         (yaw)   -> rotate about robot z

Buttons (SpaceMouse Compact, 2 buttons):
    left  (0): toggle gripper open/close
    right (1): SHORT press  -> toggle episode recording (ButtonEvents.primary)
               LONG  press  -> home the arm (>= home_hold_s, default 1 s)
    both together: discard episode (ButtonEvents.secondary)
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np

from . import xr_math
from .limits import LimitTracker
from .piper_arm import DEFAULT_HOME_JOINTS, ArmBase
from .spacemouse_client import SpaceMouseReader
from .teleop_controller import ButtonEvents, TeleopStatus


class SpaceMouseController:
    """Same consumer-facing surface as TeleopController:
    start/stop, latest_action(), poll_events(), status, input_rate_hz()."""

    def __init__(self, sm: SpaceMouseReader, arm: ArmBase, cfg):
        self.sm = sm
        self.arm = arm
        s = cfg.spacemouse
        t = cfg.teleop
        self.rate_hz = float(s.rate_hz)
        self.deadzone = float(s.deadzone)
        self.max_lin_vel = float(s.max_lin_vel)
        self.max_ang_vel = float(s.max_ang_vel)
        self.axis_signs = np.array(s.axis_signs, dtype=float)
        assert self.axis_signs.shape == (6,)
        self.stale_timeout = float(s.stale_timeout_s)

        self.ws_min = np.array(t.limits.workspace_min, dtype=float)
        self.ws_max = np.array(t.limits.workspace_max, dtype=float)
        self.pitch_max = math.radians(float(t.limits.pitch_abs_max_deg))
        self.max_cmd_deviation = float(t.limits.max_cmd_deviation)
        self.chord_window_s = float(s.chord_window_s)

        self.grip_max_width = float(cfg.gripper.max_width)
        self.gripper_button = int(s.gripper_button)
        self.record_button = int(s.record_button)

        self.limits = LimitTracker()
        self.status = TeleopStatus()
        self._target_pos: np.ndarray | None = None
        self._target_rot: np.ndarray | None = None
        self._grip_closed = False

        self._lock = threading.Lock()
        self._cmd_pose6: np.ndarray | None = None
        self._cmd_grip: float | None = None
        self._events_lock = threading.Lock()
        self._pending_events = ButtonEvents()
        self._prev_buttons: tuple = ()
        # Button state machine (see module docstring):
        # - gripper toggle is deferred by chord_window_s so a staggered
        #   two-button press becomes "discard" without side effects;
        # - record fires on RELEASE of the record button (short press);
        #   holding it >= home_hold_s triggers homing instead.
        self.home_hold_s = float(s.get("home_hold_s", 1.0))
        self._grip_deadline: float | None = None
        self._rec_down_since: float | None = None
        self._rec_consumed = False
        self._chord_fired = False

        self._home_joints = list(cfg.arm.get("home_joints") or DEFAULT_HOME_JOINTS)
        self._home_request = False
        self._homing_deadline = 0.0

        self._running = False
        self._thread: threading.Thread | None = None

    # -- consumer surface (same as TeleopController) ---------------------------

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="sm-ctl", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def latest_action(self):
        with self._lock:
            pose = None if self._cmd_pose6 is None else self._cmd_pose6.copy()
            return pose, self._cmd_grip

    def poll_events(self) -> ButtonEvents:
        with self._events_lock:
            ev = self._pending_events
            self._pending_events = ButtonEvents()
            return ev

    def input_rate_hz(self) -> float:
        return self.sm.rate_hz()

    def request_home(self) -> None:
        """Move the arm to its home joints (keyboard 'h' in the collect app)."""
        self._home_request = True

    # -- control loop --------------------------------------------------------------

    def _loop(self) -> None:
        period = 1.0 / self.rate_hz
        next_t = time.monotonic()
        while self._running:
            next_t += period
            try:
                self._tick(period)
            except Exception as e:  # never let one bad tick kill the loop
                self.status.last_error = f"tick: {e}"
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    def _apply_deadzone(self, v: np.ndarray) -> np.ndarray:
        out = np.zeros_like(v)
        for i, x in enumerate(v):
            if abs(x) > self.deadzone:
                out[i] = math.copysign((abs(x) - self.deadzone) / (1 - self.deadzone), x)
        return out

    def _tick(self, dt: float) -> None:
        st = self.status
        st.ticks += 1
        now = time.monotonic()
        if self._home_request:
            self._home_request = False
            try:
                self.arm.move_joints(self._home_joints)
                self._homing_deadline = now + 10.0
                self._target_pos = None  # re-latch from measured after homing
                st.homing = True
                st.last_error = "homing…"
            except Exception as e:
                st.last_error = f"homing: {e}"
        if st.homing:
            arm_state = self.arm.get_state()
            done = (arm_state.joints_valid and float(np.max(np.abs(
                arm_state.joints - np.asarray(self._home_joints)))) < 0.03)
            if done or now > self._homing_deadline:
                st.homing = False
                st.last_error = "home reached" if done else "homing timed out"
            return

        state = self.sm.get()
        age = self.sm.age()
        if age == float("inf"):
            st.stale = True
            return  # never received a report yet
        st.stale = False
        st.tracked = True

        self._emit_button_events(state.buttons)

        # The device only reports while being touched; without a fresh report
        # the puck is (or may be) released -> treat the twist as zero instead
        # of integrating a stale velocity.
        if age > self.stale_timeout:
            state.twist[:] = 0.0

        # Latch the starting target from the measured pose on first use.
        if self._target_pos is None:
            arm_state = self.arm.get_state()
            if not arm_state.eef_valid:
                st.last_error = "no EEF feedback yet"
                return
            if arm_state.arm_status not in (0, -1):
                st.last_error = (f"arm fault status={arm_state.arm_status} "
                                 "— run `make home` first")
                return
            if (np.any(arm_state.eef_pos < self.ws_min - 0.02)
                    or np.any(arm_state.eef_pos > self.ws_max + 0.02)):
                st.last_error = "EEF outside workspace — run `make home` first"
                return
            held = self.arm.held_target_pose6()
            latch = held if held is not None else arm_state.eef_pose6
            self._target_pos, self._target_rot = xr_math.pose6_to_matrix(latch)
            self._grip_closed = False

        twist = self._apply_deadzone(state.twist) * self.axis_signs
        x_sm, y_sm, z_sm, roll_sm, pitch_sm, yaw_sm = twist
        lin = np.array([y_sm, -x_sm, z_sm]) * self.max_lin_vel
        ang = np.array([roll_sm, pitch_sm, yaw_sm]) * self.max_ang_vel
        active = bool(np.any(np.abs(twist) > 0.0))
        st.engaged = active

        if active:
            arm_state = self.arm.get_state()
            if arm_state.arm_status not in (0, -1):
                st.last_error = f"arm fault status={arm_state.arm_status}"
                st.engaged = False
                return
            raw = self._target_pos + lin * dt
            below = raw < self.ws_min - 1e-9
            above = raw > self.ws_max + 1e-9
            if below.any() or above.any():
                self.limits.hit("ws", " ".join(
                    "xyz"[i] + ("-" if below[i] else "+")
                    for i in range(3) if below[i] or above[i]))
            self._target_pos = np.clip(raw, self.ws_min, self.ws_max)
            # Deviation guard: keep the integrated target tethered to the
            # measured pose so the arm can never wind up chasing a runaway
            # target (e.g. after being blocked by an obstacle).
            if arm_state.eef_valid:
                dev = self._target_pos - arm_state.eef_pos
                dn = float(np.linalg.norm(dev))
                if dn > self.max_cmd_deviation:
                    self._target_pos = (arm_state.eef_pos
                                        + dev * (self.max_cmd_deviation / dn))
                    self.limits.hit("tether", f"{dn * 1000:.0f}mm")
            self._target_rot = xr_math.rotvec_to_matrix(ang * dt) @ self._target_rot

            roll, pitch, yaw = xr_math.matrix_to_rpy(self._target_rot)
            if abs(pitch) > self.pitch_max:
                self.limits.hit("pitch", "+" if pitch > 0 else "-")
            pitch = float(np.clip(pitch, -self.pitch_max, self.pitch_max))
            pose6 = np.array([*self._target_pos, roll, pitch, yaw])
            try:
                self.arm.command_eef(pose6.tolist())
            except Exception as e:
                st.last_error = f"command_eef: {e}"
                return
            with self._lock:
                self._cmd_pose6 = pose6
            st.target_pose6 = pose6

    def _emit_button_events(self, buttons: tuple) -> None:
        """Chord/hold state machine. Staggered human presses must not fire
        the single-button side effects (see module docstring):
        - both buttons held (in any order)        -> discard, nothing else
        - gripper button, alone, after the window -> gripper toggle
        - record button released short, alone     -> record toggle
        - record button held >= home_hold_s       -> home the arm
        """
        now = time.monotonic()
        prev = self._prev_buttons if len(self._prev_buttons) == len(buttons) \
            else tuple(0 for _ in buttons)
        rising = [b and not p for b, p in zip(buttons, prev)]
        self._prev_buttons = tuple(buttons)

        gb, rb = self.gripper_button, self.record_button
        g = bool(buttons[gb]) if gb < len(buttons) else False
        r = bool(buttons[rb]) if rb < len(buttons) else False
        r_released = (not r) and (rb < len(prev) and bool(prev[rb]))

        # Chord: both held together (any press order) -> discard once.
        if g and r:
            if not self._chord_fired:
                self._chord_fired = True
                self._grip_deadline = None
                self._rec_consumed = True  # suppress record on release
                with self._events_lock:
                    self._pending_events.secondary = True
            return
        if not g and not r:
            self._chord_fired = False

        # Gripper: defer by the chord window, then toggle.
        if gb < len(rising) and rising[gb] and not self._chord_fired:
            self._grip_deadline = now + self.chord_window_s
        if self._grip_deadline is not None and now >= self._grip_deadline:
            self._grip_deadline = None
            if not self._chord_fired:
                self._toggle_gripper()

        # Record button: short press fires on release; long hold homes.
        if rb < len(rising) and rising[rb]:
            self._rec_down_since = now
            self._rec_consumed = False
        if r and self._rec_down_since is not None and not self._rec_consumed \
                and now - self._rec_down_since >= self.home_hold_s:
            self._rec_consumed = True
            self.request_home()
        if r_released:
            if self._rec_down_since is not None and not self._rec_consumed:
                with self._events_lock:
                    self._pending_events.primary = True
            self._rec_down_since = None
            self._rec_consumed = False

    def _toggle_gripper(self) -> None:
        self._grip_closed = not self._grip_closed
        width = 0.0 if self._grip_closed else self.grip_max_width
        try:
            self.arm.command_gripper(width)
        except Exception as e:
            self.status.last_error = f"gripper: {e}"
            return
        with self._lock:
            self._cmd_grip = width
        self.status.gripper_width = width
