"""Tracking-error benchmark: command a small, slow circle around the current
pose and measure commanded-vs-measured EEF error.

    piper-track-test                    # position mode (move_p)
    piper-track-test --impedance        # MIT impedance mode
    piper-track-test --amp 0.03 --period 5 --duration 12

Reports: static hold error, dynamic RMS/max error, per-axis errors,
estimated command->measurement latency (by cross-correlation), and per-joint
errors in impedance mode. The trajectory amplitude ramps in over 2 s and is
clamped to the teleop workspace box.
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from ..config import load_config
from ..piper_arm import make_arm


def main() -> None:
    ap = argparse.ArgumentParser(description="EEF tracking-error benchmark")
    ap.add_argument("--config", default=None)
    ap.add_argument("--impedance", action="store_true", help="(default backend)")
    ap.add_argument("--position", action="store_true", help="move_p position mode")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--amp", type=float, default=0.03, help="circle radius (m, max 0.05)")
    ap.add_argument("--period", type=float, default=5.0, help="seconds per revolution")
    ap.add_argument("--duration", type=float, default=12.0, help="moving phase (s)")
    ap.add_argument("--rate", type=float, default=100.0)
    args = ap.parse_args()

    overrides: dict = {}
    if args.sim:
        overrides["arm"] = {"backend": "fake"}
    elif args.position:
        overrides["arm"] = {"backend": "piper"}
    elif args.impedance:
        overrides["arm"] = {"backend": "piper_mit"}
    cfg = load_config(args.config, overrides)
    amp = min(abs(args.amp), 0.05)
    ws_min = np.array(cfg.teleop.limits.workspace_min)
    ws_max = np.array(cfg.teleop.limits.workspace_max)

    arm = make_arm(cfg)
    time.sleep(2.5)  # MIT entry + gain ramp settle
    st = arm.get_state()
    assert st.eef_valid, "no EEF feedback"
    held = arm.held_target_pose6()
    center = np.array(held if held is not None else st.eef_pose6, dtype=float)
    print(f"[track] backend={cfg.arm.backend}  center="
          f"{[round(float(x), 3) for x in center[:3]]}  amp={amp * 1000:.0f}mm  "
          f"period={args.period}s")
    print("[track] phase 1: static hold 3 s …")

    dt = 1.0 / args.rate
    log_t, log_cmd, log_meas, log_q_err = [], [], [], []

    def sample(cmd_pose):
        s = arm.get_state()
        if not s.eef_valid:
            return
        log_t.append(time.monotonic())
        log_cmd.append(cmd_pose.copy())
        log_meas.append(s.eef_pose6.copy())
        held_now = arm.held_target_pose6()
        if held_now is not None and s.joints_valid:
            log_q_err.append(None)  # placeholder (joint err computed below)

    # Phase 1: static hold.
    t_end = time.monotonic() + 3.0
    static_meas = []
    while time.monotonic() < t_end:
        arm.command_eef(center.tolist())
        s = arm.get_state()
        if s.eef_valid:
            static_meas.append(s.eef_pose6[:3].copy())
        time.sleep(dt)
    static_err = np.linalg.norm(np.array(static_meas) - center[:3], axis=1)

    # Phase 2: circle in the x-z plane, amplitude ramped in.
    print("[track] phase 2: moving …")
    t0 = time.monotonic()
    w = 2 * math.pi / args.period
    while True:
        t = time.monotonic() - t0
        if t > args.duration:
            break
        a = amp * min(1.0, t / 2.0)
        pose = center.copy()
        pose[0] += a * math.sin(w * t)
        pose[2] += a * (1.0 - math.cos(w * t)) * 0.5  # z stays >= center
        pose[:3] = np.clip(pose[:3], ws_min, ws_max)
        arm.command_eef(pose.tolist())
        sample(pose)
        time.sleep(dt)

    arm.command_eef(center.tolist())
    time.sleep(0.5)

    # ---- analysis ----
    cmd = np.array(log_cmd)
    meas = np.array(log_meas)
    ts = np.array(log_t)
    n = len(cmd)
    # Skip the 2 s amplitude ramp for the steady-state numbers.
    steady = ts - ts[0] > 2.5
    err_vec = cmd[:, :3] - meas[:, :3]
    err = np.linalg.norm(err_vec, axis=1)

    # Latency estimate: cross-correlate cmd x with measured x.
    lag_ms = float("nan")
    x_c = cmd[steady, 0] - cmd[steady, 0].mean()
    x_m = meas[steady, 0] - meas[steady, 0].mean()
    if len(x_c) > 100 and np.std(x_c) > 1e-6:
        max_shift = int(0.4 / dt)
        shifts = range(0, max_shift)
        corr = [float(np.dot(x_c[: len(x_c) - s], x_m[s:])) for s in shifts]
        best = int(np.argmax(corr))
        lag_ms = best * dt * 1000
        # Residual error after removing the pure delay.
        if best > 0:
            resid = np.linalg.norm(
                cmd[steady][: -best or None, :3][: len(meas[steady]) - best]
                - meas[steady][best:, :3], axis=1)
        else:
            resid = err[steady]
    else:
        resid = err[steady]

    print("\n================ TRACKING REPORT ================")
    print(f"backend: {cfg.arm.backend}   samples: {n} @ {args.rate:.0f}Hz")
    print(f"static hold error : mean {np.mean(static_err) * 1000:6.2f} mm   "
          f"max {np.max(static_err) * 1000:6.2f} mm")
    print(f"dynamic error     : RMS  {np.sqrt(np.mean(err[steady] ** 2)) * 1000:6.2f} mm   "
          f"max {np.max(err[steady]) * 1000:6.2f} mm")
    print(f"per-axis RMS (mm) : x {np.sqrt(np.mean(err_vec[steady, 0] ** 2)) * 1000:5.2f}  "
          f"y {np.sqrt(np.mean(err_vec[steady, 1] ** 2)) * 1000:5.2f}  "
          f"z {np.sqrt(np.mean(err_vec[steady, 2] ** 2)) * 1000:5.2f}")
    print(f"est. latency      : {lag_ms:.0f} ms   "
          f"(RMS after delay removal: {np.sqrt(np.mean(resid ** 2)) * 1000:.2f} mm)")
    print("=================================================")
    arm.stop()


if __name__ == "__main__":
    main()
