"""Arm abstraction: real Piper via pyAgxArm, or a fake arm for dry runs.

Real backend notes (pyAgxArm 1.0.0):
- move_p([x,y,z,r,p,y]) streams absolute EEF targets; IK runs in the arm
  firmware; target-overwrite semantics make 50-200 Hz streaming safe.
- All units m / rad; base frame; R = Rz(yaw) @ Ry(pitch) @ Rx(roll);
  pitch is hard-limited to [-pi/2, pi/2] by the SDK.
- Gripper: move_gripper_m(width_m, force_N), feedback via get_gripper_status().
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from . import xr_math


# Safe unfolded working pose (rad): shoulder up, elbow bent, gripper level
# and pointing straight forward (per MDH FK, flange +Z = (1, 0, -0.01) here;
# at this j2/j3 the j5 axis is vertical, so any j5 != 0 yaws the gripper
# sideways). The factory rest pose is folded OUTSIDE the joint range; move
# here first. Override via config arm.home_joints or `piper-home --joints`.
DEFAULT_HOME_JOINTS = [0.0, 0.85, -0.75, 0.0, 0.0, 0.0]


@dataclass
class ArmState:
    """Snapshot of measured arm + gripper state (None-free; check flags)."""

    joints: np.ndarray = field(default_factory=lambda: np.zeros(6))
    joints_valid: bool = False
    eef_pose6: np.ndarray = field(default_factory=lambda: np.zeros(6))  # x,y,z,r,p,y
    eef_valid: bool = False
    gripper_width: float = 0.0
    gripper_force: float = 0.0
    gripper_valid: bool = False
    arm_status: int = -1        # 0 = NORMAL (see pyAgxArm get_arm_status)
    mono: float = 0.0

    @property
    def eef_pos(self) -> np.ndarray:
        return self.eef_pose6[:3]

    @property
    def eef_quat_wxyz(self) -> np.ndarray:
        r = xr_math.rpy_to_matrix(*self.eef_pose6[3:6])
        return xr_math.quat_xyzw_to_wxyz(xr_math.matrix_to_quat_xyzw(r))


class ArmBase:
    limits = None        # LimitTracker when the backend tracks its own guards
    joint_limits: tuple | None = None  # (lower, upper) rad, for range displays

    def get_state(self) -> ArmState:
        raise NotImplementedError

    def held_target_pose6(self):
        """Pose the arm is actively holding (commanded target), or None.
        Impedance backends sag below their target under gravity — the clutch
        must latch onto the TARGET, not the sagged measured pose, or every
        engage ratchets the arm downward."""
        return None

    def describe(self) -> dict:
        """Hardware/controller provenance for dataset metadata."""
        return {"backend": type(self).__name__}

    def command_eef(self, pose6: list[float]) -> None:
        raise NotImplementedError

    def command_gripper(self, width: float) -> None:
        raise NotImplementedError

    def move_joints(self, joints: list[float]) -> None:
        raise NotImplementedError

    def command_joints(self, joints: list[float]) -> None:
        """Streamable joint target (joint-space replay). Default: firmware
        point move each tick; PiperArmMIT overrides with impedance targets."""
        self.move_joints(list(joints))

    def stop(self) -> None:
        raise NotImplementedError

    def emergency_stop(self) -> None:
        raise NotImplementedError


class PiperArm(ArmBase):
    """Real arm through pyAgxArm (CAN)."""

    def __init__(self, cfg):
        from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

        model = getattr(ArmModel, cfg.arm.model)
        fw = getattr(PiperFW, cfg.arm.firmware)
        config = create_agx_arm_config(
            robot=model,
            firmeware_version=fw,  # (sic) SDK kwarg spelling
            interface=cfg.arm.interface,
            channel=cfg.arm.channel,
            bitrate=cfg.arm.bitrate,
        )
        self.robot = AgxArmFactory.create_arm(config)
        self.robot.connect()
        time.sleep(0.2)  # let the RX thread populate caches

        fw_info = None
        try:
            msg = self.robot.get_firmware()
            fw_info = getattr(msg, "msg", msg)
        except Exception:
            pass
        self.firmware_info = dict(fw_info) if isinstance(fw_info, dict) else fw_info
        self._cfg_arm = cfg.arm.to_dict()
        print(f"[piper] connected on {cfg.arm.channel}; firmware: {fw_info}; "
              f"configured PiperFW.{cfg.arm.firmware}")

        self.gripper = None
        self._grip_cfg = cfg.gripper
        if cfg.gripper.enabled:
            self.gripper = self.robot.init_effector(self.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)

        deadline = time.monotonic() + 5.0
        while not self.robot.enable():
            if time.monotonic() > deadline:
                raise RuntimeError("failed to enable arm within 5 s — check power/CAN")
            time.sleep(0.01)
        if cfg.arm.enable_soft_joint_limits:
            self.robot.set_joint_limits_enabled(True)
        self.robot.set_speed_percent(int(cfg.arm.speed_percent))
        print("[piper] enabled")

        self._last_grip_cmd = 0.0
        self._grip_period = 1.0 / float(cfg.gripper.rate_hz)
        self._grip_force = float(cfg.gripper.force)

        # Joint range for status displays (SDK preset; the MIT backend
        # narrows this to its config-intersected working limits).
        try:
            from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD
            preset = ROBOT_JOINT_LIMIT_PRESET_RAD[cfg.arm.model.lower()]
            self.joint_limits = (
                np.array([preset[f"joint{i}"][0] for i in range(1, 7)]),
                np.array([preset[f"joint{i}"][1] for i in range(1, 7)]))
        except Exception:
            pass

    def get_state(self) -> ArmState:
        st = ArmState(mono=time.monotonic())
        ja = self.robot.get_joint_angles()
        if ja is not None:
            st.joints = np.asarray(ja.msg, dtype=float)
            st.joints_valid = True
        fp = self.robot.get_flange_pose()
        if fp is not None:
            st.eef_pose6 = np.asarray(fp.msg, dtype=float)
            st.eef_valid = True
        arm_st = self.robot.get_arm_status()
        if arm_st is not None:
            st.arm_status = int(arm_st.msg.arm_status)
        if self.gripper is not None:
            gs = self.gripper.get_gripper_status()
            if gs is not None:
                st.gripper_width = float(gs.msg.value)  # width mode -> meters
                st.gripper_force = float(gs.msg.force)
                st.gripper_valid = True
        return st

    def describe(self) -> dict:
        import pyAgxArm

        return {
            "backend": self._cfg_arm.get("backend", "piper"),
            "arm_model": self._cfg_arm.get("model"),
            "firmware": self.firmware_info,
            "configured_firmware_enum": self._cfg_arm.get("firmware"),
            "pyAgxArm_version": pyAgxArm.__version__,
            "can_channel": self._cfg_arm.get("channel"),
            "speed_percent": self._cfg_arm.get("speed_percent"),
        }

    def command_eef(self, pose6: list[float]) -> None:
        self.robot.move_p(list(map(float, pose6)))

    def command_gripper(self, width: float) -> None:
        if self.gripper is None:
            return
        now = time.monotonic()
        if now - self._last_grip_cmd < self._grip_period:
            return
        self._last_grip_cmd = now
        # Never command the mechanical end stop: the physical stroke tops out
        # ~0.4mm short of max_width (feedback ~0.0696 vs 0.07), so a "fully
        # open" target keeps the position controller stalled OUTWARD at the
        # force limit indefinitely — open is the resting state ~90% of the
        # time, hence the hot gripper motor. A 2mm margin puts the target
        # inside the reachable stroke (motor settles, current drops); 68mm vs
        # 69.6mm of opening is task-irrelevant.
        open_margin = float(self._grip_cfg.get("open_margin", 0.002))
        width = float(np.clip(width, 0.0, self._grip_cfg.max_width - open_margin))
        self.gripper.move_gripper_m(value=width, force=self._grip_force)

    def move_joints(self, joints: list[float]) -> None:
        self.robot.move_j(list(map(float, joints)))

    def stop(self) -> None:
        try:
            self.robot.disconnect()
        except Exception:
            pass

    def emergency_stop(self) -> None:
        self.robot.electronic_emergency_stop()


class PiperArmMIT(PiperArm):
    """Compliant variant: EEF targets -> host-side IK -> per-joint MIT
    (impedance) commands. The arm behaves like a 6-joint spring-damper
    around the streamed joint targets: T = kp*(q_des - q) + kd*(0 - v) + t_ff.

    Safety sequencing (all required for MIT mode):
    - the mode frame is sent ONCE (auto mode-resend disabled) before streaming;
    - the first joint target is latched from the MEASURED joints (no jumps);
    - per-tick joint steps are clamped by impedance.max_joint_vel;
    - stop() hands the arm back to firmware position hold (move_j at the
      measured pose) so it cannot slump when the process exits.
    """

    def __init__(self, cfg):
        super().__init__(cfg)
        imp = cfg.impedance
        self.kp = [float(x) for x in imp.kp]
        self.kd = [float(x) for x in imp.kd]
        self.t_ff = [float(x) for x in imp.t_ff]
        assert len(self.kp) == len(self.kd) == len(self.t_ff) == 6
        self._max_step = float(imp.max_joint_vel) / float(cfg.teleop.rate_hz)

        from .ik import MDHIK
        lower = np.radians(imp.joint_limits_deg.lower)
        upper = np.radians(imp.joint_limits_deg.upper)
        # Intersect with the SDK's per-model limits (2 deg margin) so the IK
        # can never command what the SDK/firmware would clamp — a clamped
        # joint fighting the impedance loop buzzes and drifts.
        try:
            from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD
            preset = ROBOT_JOINT_LIMIT_PRESET_RAD[cfg.arm.model.lower()]
            margin = math.radians(2.0)
            sdk_lo = np.array([preset[f"joint{i}"][0] for i in range(1, 7)]) + margin
            sdk_hi = np.array([preset[f"joint{i}"][1] for i in range(1, 7)]) - margin
            lower = np.maximum(lower, sdk_lo)
            upper = np.minimum(upper, sdk_hi)
            print(f"[piper-mit] joint limits (deg): "
                  f"{[(round(math.degrees(a)), round(math.degrees(b))) for a, b in zip(lower, upper)]}")
        except Exception as e:
            print(f"[piper-mit] WARNING: SDK limit preset unavailable ({e}); "
                  "using config limits only")
        self._ik = MDHIK(self.robot.fk, lower, upper,
                         damping=float(imp.ik.damping), iters=int(imp.ik.iters),
                         pos_tol=float(imp.ik.pos_tol))
        self.joint_limits = (lower.copy(), upper.copy())
        from .limits import LimitTracker
        self.limits = LimitTracker()
        self._q_target: np.ndarray | None = None
        self._mit_active = False
        self._mit_entered = 0.0
        self._ramp_s = float(imp.get("soft_start_s", 0.8))
        # Gravity feedforward: snapshot of the holding torques measured while
        # the firmware position servo is still active (= exact gravity +
        # friction hold at the entry pose). Applied UNRAMPED from the first
        # MIT frame -> torque-continuous handover: no clunk, no droop.
        self._gravity_ff = bool(imp.get("gravity_ff", True))
        self._tff_snapshot = np.zeros(6)
        self._tff_checked = True
        # Keepalive: while MIT is active but teleop is idle (clutch released),
        # keep streaming hold frames so the gain ramp completes and the
        # firmware always has fresh impedance targets (spring behavior in
        # idle is then well-defined, not "whatever the last frame said").
        self._mit_lock = threading.Lock()
        self._last_cmd_mono = 0.0
        self._keeper = threading.Thread(target=self._keepalive_loop,
                                        name="mit-keepalive", daemon=True)
        self._keeper_running = True
        self._keeper.start()
        print(f"[piper-mit] impedance backend ready (kp={self.kp}, kd={self.kd})")
        if bool(imp.get("engage_on_start", True)):
            deadline = time.monotonic() + 3.0
            entered = False
            while time.monotonic() < deadline:
                with self._mit_lock:
                    entered = self._enter_mit()
                if entered:
                    break
                time.sleep(0.05)
            if not entered:
                print("[piper-mit] WARNING: no joint feedback yet — MIT will "
                      "engage on the first EEF command instead")

    def _keepalive_loop(self) -> None:
        while self._keeper_running:
            time.sleep(0.02)  # 50 Hz
            with self._mit_lock:
                if not self._mit_active or self._q_target is None:
                    continue
                if time.monotonic() - self._last_cmd_mono > 0.04:
                    try:
                        self._send_mit(self._q_target)
                    except Exception:
                        pass
                # One-shot sanity check after the ramp: a wrong-signed torque
                # snapshot would DOUBLE the gravity deflection instead of
                # cancelling it. If the arm deflected a lot, drop the
                # feedforward and fall back to plain spring behavior.
                if (not self._tff_checked
                        and time.monotonic() - self._mit_entered > self._ramp_s + 0.7):
                    self._tff_checked = True
                    st = self.get_state()
                    if st.joints_valid:
                        err = float(np.max(np.abs(st.joints - self._q_target)))
                        if err > 0.2:
                            print(f"[piper-mit] WARNING: {err:.2f} rad deflection "
                                  "after entry — disabling gravity feedforward "
                                  "(check torque sign convention)")
                            self._tff_snapshot[:] = 0.0

    def _enter_mit(self) -> bool:
        st = self.get_state()
        if not st.joints_valid:
            return False
        self._q_target = st.joints.copy()

        # Sample the position servo's holding torques BEFORE switching modes.
        self._tff_snapshot[:] = 0.0
        if self._gravity_ff:
            samples = []
            for _ in range(5):
                row = []
                for i in range(1, 7):
                    ms = self.robot.get_motor_states(i)
                    row.append(float(ms.msg.torque) if ms is not None else np.nan)
                samples.append(row)
                time.sleep(0.02)
            tau = np.nan_to_num(np.nanmean(np.array(samples), axis=0))
            self._tff_snapshot = np.clip(tau, -6.0, 6.0)
            self._tff_checked = False
            print(f"[piper-mit] gravity hold torques (N*m): "
                  f"{[round(float(x), 2) for x in self._tff_snapshot]}")

        self.robot.set_auto_set_motion_mode_enabled(False)
        self.robot.set_motion_mode('mit')
        # Soft start: gains ramp 0 -> 100% over soft_start_s, so the handover
        # from the firmware position servo cannot produce a torque step (the
        # audible "clunk"). First frames hold the measured pose regardless.
        self._mit_entered = time.monotonic()
        self._send_mit(self._q_target)
        self._mit_active = True
        print("[piper-mit] MIT mode active (holding measured joints, "
              f"gains ramping over {self._ramp_s:.1f}s)")
        return True

    def _send_mit(self, q: np.ndarray) -> None:
        ramp = 1.0
        if self._ramp_s > 0:
            ramp = min(1.0, max(0.05, (time.monotonic() - self._mit_entered)
                                / self._ramp_s))
        for i in range(6):
            # Gains ramp (soft start); the gravity snapshot does NOT ramp —
            # it replaces exactly what the position servo was outputting, so
            # full strength from frame one keeps the handover torque-smooth.
            t_ff = float(np.clip(self.t_ff[i] + self._tff_snapshot[i], -8.0, 8.0))
            self.robot.move_mit(i + 1, p_des=float(q[i]), v_des=0.0,
                                kp=self.kp[i] * ramp, kd=self.kd[i] * ramp,
                                t_ff=t_ff)

    def command_eef(self, pose6: list[float]) -> None:
        with self._mit_lock:
            if not self._mit_active and not self._enter_mit():
                raise RuntimeError("no joint feedback yet; cannot enter MIT mode")
            self._last_cmd_mono = time.monotonic()
            q_sol, ok = self._ik.solve(pose6, self._q_target)
            if not ok:
                # Unreachable/singular target: hold the previous joint target
                # rather than chase a bad solution.
                self.limits.hit("ik")
                self._send_mit(self._q_target)
                return
            self._note_joint_guards(q_sol)
            step = np.clip(q_sol - self._q_target, -self._max_step, self._max_step)
            self._q_target = self._q_target + step
            self._send_mit(self._q_target)

    def _note_joint_guards(self, q_sol: np.ndarray) -> None:
        """Caller must hold _mit_lock. Records which joint-level guards are
        shaping the command: solutions pinned at a joint limit, and per-tick
        joint steps saturating max_joint_vel."""
        at_lo = q_sol <= self._ik.lower + 1e-6
        at_hi = q_sol >= self._ik.upper - 1e-6
        if at_lo.any() or at_hi.any():
            self.limits.hit("jlim", " ".join(
                f"j{i + 1}" + ("-" if at_lo[i] else "+")
                for i in range(6) if at_lo[i] or at_hi[i]))
        step = np.abs(q_sol - self._q_target)
        if np.any(step > self._max_step + 1e-12):
            self.limits.hit("jvel", f"j{int(step.argmax()) + 1}")

    def held_target_pose6(self):
        with self._mit_lock:
            if not self._mit_active or self._q_target is None:
                return None
            q = self._q_target.copy()
        return np.asarray(self.robot.fk([float(x) for x in q]), dtype=float)

    def command_joints(self, joints: list[float]) -> None:
        # Same path as command_eef minus the IK: clamp to the (SDK-
        # intersected) joint limits, step-clamp, stream as MIT targets.
        with self._mit_lock:
            if not self._mit_active and not self._enter_mit():
                raise RuntimeError("no joint feedback yet; cannot enter MIT mode")
            self._last_cmd_mono = time.monotonic()
            q = np.clip(np.asarray(joints, dtype=float),
                        self._ik.lower, self._ik.upper)
            self._note_joint_guards(q)
            step = np.clip(q - self._q_target, -self._max_step, self._max_step)
            self._q_target = self._q_target + step
            self._send_mit(self._q_target)

    def move_joints(self, joints: list[float]) -> None:
        # Leave MIT for a firmware-planned move (homing etc.).
        with self._mit_lock:
            self._exit_mit_to_position_hold()
            self._q_target = None
        super().move_joints(joints)

    def _exit_mit_to_position_hold(self) -> None:
        """Caller must hold _mit_lock."""
        if not self._mit_active:
            return
        self._mit_active = False
        self.robot.set_auto_set_motion_mode_enabled(True)
        st = self.get_state()
        if st.joints_valid:
            try:
                self.robot.move_j([float(x) for x in st.joints])  # position hold
            except Exception:
                pass

    def stop(self) -> None:
        self._keeper_running = False
        try:
            with self._mit_lock:
                self._exit_mit_to_position_hold()
        finally:
            super().stop()

    def emergency_stop(self) -> None:
        with self._mit_lock:
            self._mit_active = False
        super().emergency_stop()


class FakeArm(ArmBase):
    """Simulated arm: first-order lag toward commanded targets. For pipeline
    testing (viz, recording, format) without hardware."""

    def __init__(self, cfg):
        self._lock = threading.Lock()
        self._target = np.array([0.30, 0.0, 0.25, 0.0, 1.2, 0.0])
        self._pose = self._target.copy()
        self._joints = np.zeros(6)
        self._grip_target = float(cfg.gripper.max_width)
        self._grip = self._grip_target
        self._max_width = float(cfg.gripper.max_width)
        self._last = time.monotonic()
        print("[fake-arm] simulated arm backend active")

    def _step(self) -> None:
        now = time.monotonic()
        dt = min(now - self._last, 0.1)
        self._last = now
        alpha = min(1.0, dt / 0.08)  # ~80 ms time constant
        self._pose += alpha * (self._target - self._pose)
        self._grip += alpha * (self._grip_target - self._grip)

    def get_state(self) -> ArmState:
        with self._lock:
            self._step()
            st = ArmState(mono=time.monotonic())
            st.joints = self._joints.copy()
            st.joints_valid = True
            st.eef_pose6 = self._pose.copy()
            st.eef_valid = True
            st.gripper_width = float(self._grip)
            st.gripper_valid = True
            st.arm_status = 0
            return st

    def command_eef(self, pose6: list[float]) -> None:
        with self._lock:
            self._target = np.asarray(pose6, dtype=float)

    def command_gripper(self, width: float) -> None:
        with self._lock:
            self._grip_target = float(np.clip(width, 0.0, self._max_width))

    def move_joints(self, joints: list[float]) -> None:
        with self._lock:
            self._joints = np.asarray(joints, dtype=float)

    def stop(self) -> None:
        pass

    def emergency_stop(self) -> None:
        with self._lock:
            self._target = self._pose.copy()


def make_arm(cfg) -> ArmBase:
    backend = cfg.arm.backend
    if backend == "fake":
        return FakeArm(cfg)
    if backend == "piper_mit":
        return PiperArmMIT(cfg)
    return PiperArm(cfg)
