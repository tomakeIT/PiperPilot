"""Inference runtime: observation building -> policy -> chunked execution.

    piper-infer --policy hold                          # loop smoke test
    piper-infer --policy replay:<dataset_root>:0       # closed-loop replay
    piper-infer --policy http://gpu-host:8901          # remote model server
    piper-infer --policy ... --sim --no-cameras        # no hardware

Loop structure (standard chunked-action VLA deployment):
  every exec_horizon steps: build obs (latest camera frames + arm state)
  -> policy.predict(obs, horizon) -> execute steps at the dataset rate
  (30 Hz) via command_eef/command_gripper. Absolute actions, same safety
  envelope as teleop (backend step clamps, arm-fault abort).
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from .. import xr_math
from ..cameras import make_cameras
from ..config import load_config
from ..piper_arm import make_arm
from ..policy import ReplayPolicy, make_policy
from .replay import pose7_to_pose6


def main() -> None:
    ap = argparse.ArgumentParser(description="Run a policy on the Piper")
    ap.add_argument("--policy", required=True,
                    help="hold | replay:<root>[:<ep>] | http://host:port")
    ap.add_argument("--task", default=None, help="language instruction")
    ap.add_argument("--config", default=None)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--position", action="store_true")
    ap.add_argument("--no-cameras", action="store_true")
    ap.add_argument("--rate", type=float, default=30.0, help="execution rate (dataset fps)")
    ap.add_argument("--exec-horizon", type=int, default=15,
                    help="steps executed per policy query (15 @ 30Hz = 0.5 s)")
    ap.add_argument("--horizon", type=int, default=30, help="steps requested per query")
    ap.add_argument("--max-seconds", type=float, default=120.0)
    args = ap.parse_args()

    overrides: dict = {}
    if args.sim:
        overrides["arm"] = {"backend": "fake"}
    elif args.position:
        overrides["arm"] = {"backend": "piper"}
    cfg = load_config(args.config, overrides)
    task = args.task or cfg.recording.task
    ws_min = np.array(cfg.teleop.limits.workspace_min)
    ws_max = np.array(cfg.teleop.limits.workspace_max)

    policy = make_policy(args.policy)
    if isinstance(policy, ReplayPolicy):
        # Replay must consume its chunks fully (receding-horizon re-query
        # would skip the tail of every chunk); real policies keep the
        # exec_horizon < horizon overlap.
        args.exec_horizon = args.horizon
    cameras = [] if args.no_cameras else make_cameras(cfg, fake=args.sim)
    arm = make_arm(cfg)
    time.sleep(2.0)

    def build_obs() -> dict:
        st = arm.get_state()
        obs = {
            "task": task,
            "state": {
                "observation.arm_joint": st.joints.tolist(),
                "observation.arm_eef": np.concatenate(
                    [st.eef_pos, st.eef_quat_wxyz]).tolist(),
                "observation.gripper": [st.gripper_width],
            },
            "images": {},
        }
        for cam in cameras:
            fr = cam.latest()
            if fr is not None:
                obs["images"][cam.name] = fr.rgb
        return obs

    print(f"[infer] policy={args.policy}  task={task!r}  "
          f"{args.rate:.0f}Hz, chunk {args.exec_horizon}/{args.horizon}")
    policy.reset()
    period = 1.0 / args.rate
    t_end = time.monotonic() + args.max_seconds
    next_t = time.monotonic()
    chunk: np.ndarray | None = None
    ci = 0
    steps = 0
    try:
        while time.monotonic() < t_end:
            if chunk is None or ci >= min(args.exec_horizon, len(chunk)):
                t_q = time.monotonic()
                chunk = policy.predict(build_obs(), args.horizon)
                ci = 0
                lat = (time.monotonic() - t_q) * 1000
                print(f"\r[infer] step {steps:5d}  query {lat:5.1f} ms   ",
                      end="", flush=True)
                next_t = time.monotonic()  # re-sync clock after query latency

            a = chunk[ci]
            ci += 1
            steps += 1
            pos = np.clip(a[:3], ws_min, ws_max)
            rot = xr_math.quat_xyzw_to_matrix(xr_math.quat_wxyz_to_xyzw(a[3:7]))
            arm.command_eef(xr_math.matrix_to_pose6(pos, rot))
            arm.command_gripper(float(a[7]))

            st = arm.get_state()
            if st.arm_status not in (0, -1):
                print(f"\n[infer] ABORT: arm fault status={st.arm_status}")
                break
            if isinstance(policy, ReplayPolicy) and policy.done and ci >= len(chunk):
                print("\n[infer] replay policy finished")
                break

            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        print("\n[infer] interrupted")
    finally:
        for cam in cameras:
            cam.stop()
        arm.stop()
        print(f"[infer] done: {steps} steps executed")


if __name__ == "__main__":
    main()
