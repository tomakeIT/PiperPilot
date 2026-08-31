"""Unfold / home the Piper arm into its working range (joint-space move).

The factory rest pose is folded slightly OUTSIDE the joint operating range
(j2 < 0 deg, j3 > 0 deg), so Cartesian move_p from rest is rejected by the
firmware with TARGET_POS_EXCEEDS_LIMIT(0x4). Run this once after power-on
(or whenever teleop reports "run piper-home"):

    piper-home                # default safe unfold pose, 30% speed
    piper-home --speed 20 --joints 0 0.85 -0.75 0 0.6 0
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from ..config import load_config
from ..piper_arm import DEFAULT_HOME_JOINTS, make_arm

# SDK demo convention (`move_j([0]*6)`): compact pose near the base — good
# for parking/rest, but OUTSIDE the teleop workspace box (flange x ~ 0.10 m).
ZERO_JOINTS = [0.0] * 6


def main() -> None:
    ap = argparse.ArgumentParser(description="Home/unfold the Piper arm")
    ap.add_argument("--config", default=None)
    ap.add_argument("--joints", type=float, nargs=6, default=None,
                    help="target joint angles (rad)")
    ap.add_argument("--zero", action="store_true",
                    help="go to the all-zero SDK rest pose (parking; teleop "
                         "cannot start from there)")
    ap.add_argument("--speed", type=int, default=30, help="speed percent (default 30)")
    ap.add_argument("--sim", action="store_true")
    args = ap.parse_args()

    # Homing is a firmware-planned joint move — always use the plain position
    # backend (entering/leaving MIT here would be pointless mode churn).
    overrides = {"arm": {"speed_percent": args.speed, "backend": "piper"}}
    if args.sim:
        overrides["arm"]["backend"] = "fake"
    cfg = load_config(args.config, overrides)
    if args.zero:
        target = ZERO_JOINTS
    else:
        target = args.joints or cfg.arm.get("home_joints") or DEFAULT_HOME_JOINTS

    arm = make_arm(cfg)
    st = arm.get_state()
    print(f"[home] current joints (deg): "
          f"{[round(math.degrees(x), 1) for x in st.joints]}")
    print(f"[home] target  joints (deg): "
          f"{[round(math.degrees(x), 1) for x in target]}")
    print(f"[home] moving at {args.speed}% speed — stand clear!")
    arm.move_joints(list(target))

    deadline = time.monotonic() + 25.0
    reached = False
    while time.monotonic() < deadline:
        time.sleep(0.2)
        st = arm.get_state()
        if not st.joints_valid:
            continue
        err = float(np.max(np.abs(st.joints - np.asarray(target))))
        print(f"\r[home] max joint err: {math.degrees(err):6.2f} deg   ",
              end="", flush=True)
        if err < 0.03:  # ~1.7 deg
            reached = True
            break
    print()

    st = arm.get_state()
    print(f"[home] final joints (deg): "
          f"{[round(math.degrees(x), 1) for x in st.joints]}")
    print(f"[home] flange pose: {[round(float(x), 3) for x in st.eef_pose6]}")
    print(f"[home] arm_status: {st.arm_status}")
    ws_min = np.array(cfg.teleop.limits.workspace_min)
    ws_max = np.array(cfg.teleop.limits.workspace_max)
    inside = bool(np.all(st.eef_pos >= ws_min) and np.all(st.eef_pos <= ws_max))
    print(f"[home] EEF inside teleop workspace box: {inside}")
    if not reached:
        print("[home] WARNING: target not reached within 25 s — check "
              "arm_status above (fault?) and whether the arm is obstructed.")
    elif not inside:
        print("[home] WARNING: home pose is outside the configured workspace "
              "box — adjust teleop.limits.workspace_* or the home joints.")
    else:
        print("[home] OK — arm is holding position; teleop can start now.")
    arm.stop()


if __name__ == "__main__":
    main()
