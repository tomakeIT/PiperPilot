"""Replay a recorded episode's actions on the arm (open-loop) and report how
well the replayed trajectory reproduces the recorded one.

Two dataset schemas are auto-detected:

- **native** (this toolkit's recordings): `action.arm_eef` (pos + quat wxyz,
  absolute, arm base frame) + `action.gripper` (width, m). Replayed
  ABSOLUTE by default.
- **modality** (GR00T-style `action` vector + meta/modality.json, e.g. the
  RoboMIND `agilex_cobot_magic` dual-arm sets): pick an arm with
  `--arm-side`. These come from a different robot and frame convention, so
  they replay RELATIVE by default — the recorded motion (deltas from its
  first pose) is applied on top of this arm's current pose — and raw-unit
  gripper values are rescaled to the configured width.

Control space: EEF actions by default; `--joints` replays the recorded joint
trajectory instead (native datasets only — impedance backend streams MIT
joint targets, position backend streams move_j). `--viz` opens a live rerun
3D view.

    piper-replay --root ~/piper_datasets/<task> [--episode 0] [--speed 0.7]
    piper-replay --root ~/piper_datasets/<task> --joints --viz
    piper-replay --root data/50_pourtea --episode 38 --arm-side right --sim

Safety: homes to the config home joints first (--no-home to skip — the host
IK warm-starts from the current joints, so different start postures land in
different joint branches and reproduce differently), then approaches the
first target over 3 s; target positions are clamped to the configured
workspace box (the clamped fraction is reported); per-tick step clamps come
from the arm backend; any arm fault aborts.

Every run saves a per-tick command/execution log (disable with --no-log) to
<root>/replay_logs/ and reports which backend guards fired (IK failures,
joint limits, joint-velocity saturation). Inspect a log in 3D with
`piper-viz --replay-log <file.npz>` — commanded vs measured trails.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

from .. import xr_math
from ..config import load_config
from ..piper_arm import DEFAULT_HOME_JOINTS, make_arm


def pose7_to_pose6(pose7: np.ndarray) -> list[float]:
    """[pos3, quat_wxyz4] -> [x,y,z,roll,pitch,yaw]."""
    rot = xr_math.quat_xyzw_to_matrix(xr_math.quat_wxyz_to_xyzw(pose7[3:7]))
    return xr_math.matrix_to_pose6(pose7[:3], rot)


def _quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    return xr_math.quat_xyzw_to_matrix(xr_math.quat_wxyz_to_xyzw(q))


def available_episodes(root: Path) -> list[int]:
    return sorted(int(p.stem.split("_")[-1])
                  for p in root.glob("data/chunk-*/episode_*.parquet"))


def load_episode(root: Path, episode: int, arm_side: str = "right"):
    """Returns (actions7_wxyz, grip_raw, obs7_wxyz, joints, ts, schema).
    joints is the recorded joint trajectory (N,6) or None if the dataset has
    no joint channel (e.g. modality/EEF-only sets)."""
    info_path = root / "meta" / "info.json"
    chunks = 1000
    if info_path.exists():
        chunks = int(json.load(open(info_path)).get("chunks_size", 1000))
    pq = root / "data" / f"chunk-{episode // chunks:03d}" / f"episode_{episode:06d}.parquet"
    if not pq.exists():
        avail = available_episodes(root)
        raise SystemExit(f"[replay] episode {episode} not found under {root}\n"
                         f"[replay] available episodes: {avail}")
    df = pd.read_parquet(pq)
    ts = df["timestamp"].to_numpy(dtype=float)

    if "action.arm_eef" in df.columns:  # native schema
        actions = np.vstack(df["action.arm_eef"]).astype(float)
        grip = np.vstack(df["action.gripper"]).astype(float)[:, 0]
        obs = np.vstack(df["observation.arm_eef"]).astype(float)
        joints = (np.vstack(df["observation.arm_joint"]).astype(float)
                  if "observation.arm_joint" in df.columns else None)
        return actions, grip, obs, joints, ts, "native"

    if "action" in df.columns:  # GR00T/modality schema
        mod = json.load(open(root / "meta" / "modality.json"))

        def part(group: str, key: str):
            name = f"{arm_side}_{key}"
            if name not in mod[group]:
                raise SystemExit(f"[replay] modality.json has no '{name}' in "
                                 f"'{group}' (keys: {list(mod[group])})")
            m = mod[group][name]
            return slice(int(m["start"]), int(m["end"])), m

        def pose7_from(mat: np.ndarray, group: str) -> np.ndarray:
            psl, _ = part(group, "position")
            osl, om = part(group, "orientation")
            quat = mat[:, osl]
            if om.get("rotation_type", "quat_xyzw") == "quat_xyzw":
                quat = quat[:, [3, 0, 1, 2]]  # -> wxyz
            return np.concatenate([mat[:, psl], quat], axis=1)

        a = np.vstack(df["action"]).astype(float)
        s = np.vstack(df["observation.state"]).astype(float)
        gsl, _ = part("action", "gripper")
        # Some modality sets (e.g. newer RoboMIND conversions) expose the
        # measured joint trajectory too — that enables --joints replay.
        joints = None
        jkey = f"{arm_side}_arm_joint"
        if jkey in mod.get("state", {}):
            jm = mod["state"][jkey]
            joints = s[:, int(jm["start"]):int(jm["end"])]
        return pose7_from(a, "action"), a[:, gsl][:, 0], \
            pose7_from(s, "state"), joints, ts, "modality"

    raise SystemExit(f"[replay] unrecognized schema in {pq.name}: "
                     f"columns {list(df.columns)}")


def build_targets(actions7: np.ndarray, relative: bool,
                  start_pose6: np.ndarray | None = None,
                  scale: float = 1.0) -> np.ndarray:
    """(N,7) wxyz actions -> (N,6) pose6 targets.

    relative: replay the recorded DELTAS from the episode's first action pose
    on top of start_pose6 (cross-robot / cross-frame safe), with translation
    deltas optionally scaled (shrink motions recorded on a larger robot):
        p(t) = p_start + scale * (p_rec(t) - p_rec(0))
        R(t) = R_rec(t) @ R_rec(0)^T @ R_start
    """
    if not relative:
        return np.array([pose7_to_pose6(a) for a in actions7])
    assert start_pose6 is not None
    p_start = np.asarray(start_pose6[:3], dtype=float)
    r_start = xr_math.rpy_to_matrix(*start_pose6[3:6])
    p0 = actions7[0, :3]
    r0_t = _quat_wxyz_to_matrix(actions7[0, 3:7]).T
    out = np.zeros((len(actions7), 6))
    for i, a in enumerate(actions7):
        rot = _quat_wxyz_to_matrix(a[3:7]) @ r0_t @ r_start
        out[i] = xr_math.matrix_to_pose6(p_start + scale * (a[:3] - p0), rot)
    return out


def map_gripper(grip: np.ndarray, max_width: float,
                raw_max: float | None = None) -> tuple[np.ndarray, str]:
    """Dataset gripper values -> command widths (m). Values already within
    the configured width pass through; raw-unit values (e.g. RoboMIND's
    0..~5) are rescaled linearly with full-open = raw_max (default: the
    episode's max)."""
    if grip.max() <= max_width * 1.5:
        return np.clip(grip, 0.0, max_width), "width (m), pass-through"
    top = float(raw_max) if raw_max else float(grip.max())
    return (np.clip(grip / top, 0.0, 1.0) * max_width,
            f"raw units rescaled: {top:.3g} -> {max_width:.3f} m")


def dataset_conventions(root: Path) -> dict:
    """Dataset-specific replay conventions from <root>/meta/piper_replay.json:
    {"rot_offset_deg": [R,P,Y], "offset_m": [dx,dy,dz]}. Lets a downloaded
    dataset carry its own frame fixes instead of flags on every command —
    dataloaders should read the same file."""
    p = root / "meta" / "piper_replay.json"
    return json.load(open(p)) if p.exists() else {}


def dataset_rot_offset(root: Path) -> list[float] | None:
    vals = dataset_conventions(root).get("rot_offset_deg")
    return None if vals is None else [float(v) for v in vals]


def apply_rot_offset(actions7: np.ndarray, rpy_deg) -> np.ndarray:
    """Post-multiply a constant TOOL-side rotation onto every recorded
    orientation — fixes cross-robot gripper-frame conventions (e.g. RoboMIND
    agilex tool-x = Piper tool-z). A constant tool-side rotation cancels in
    relative replay's delta composition, so this only changes ABSOLUTE
    orientation replay."""
    c = xr_math.rpy_to_matrix(*np.radians([float(v) for v in rpy_deg]))
    out = actions7.copy()
    for a in out:
        r = _quat_wxyz_to_matrix(a[3:7]) @ c
        a[3:7] = xr_math.quat_xyzw_to_wxyz(xr_math.matrix_to_quat_xyzw(r))
    return out


def save_replay_log(path: Path, ts, targets, grip, meas_eef, eef_valid,
                    meas_joints, joints_valid, actions7, obs7, meta: dict) -> None:
    """Per-tick command/execution record of a replay run (piper-viz
    --replay-log renders it; meta is a JSON-able dict)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, ts=ts, targets=targets, grip=grip,
                        meas_eef=meas_eef, eef_valid=eef_valid,
                        meas_joints=meas_joints, joints_valid=joints_valid,
                        rec_actions7=actions7, rec_obs7=obs7,
                        meta=json.dumps(meta))


def main() -> None:
    ap = argparse.ArgumentParser(description="Replay a recorded episode")
    ap.add_argument("--root", required=True)
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--speed", type=float, default=1.0, help="time scale (0.5 = half speed)")
    ap.add_argument("--config", default=None)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--position", action="store_true", help="force position backend")
    ap.add_argument("--arm-side", choices=["left", "right"], default="right",
                    help="dual-arm (modality) datasets: which arm to replay")
    ap.add_argument("--relative", action="store_true",
                    help="force relative-to-current-pose replay")
    ap.add_argument("--absolute", action="store_true",
                    help="force absolute poses even for modality datasets")
    ap.add_argument("--anchor", default=None, metavar="X,Y,Z[,R,P,Y]",
                    help="anchor relative replay to this FIXED pose (m, RPY "
                         "deg, base frame) instead of the arm's current pose "
                         "— same trajectory every run, wherever the arm "
                         "starts. Omit RPY to keep the current orientation.")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="scale translation deltas in relative replay "
                         "(shrink motions recorded on a larger robot)")
    ap.add_argument("--offset", default=None, metavar="DX,DY,DZ",
                    help="add a fixed translation offset (m, Piper base "
                         "frame) to EEF targets before workspace clamping")
    ap.add_argument("--rot-offset", default=None, metavar="R,P,Y",
                    help="constant TOOL-side rotation (deg, post-multiplied "
                         "onto each recorded orientation) fixing cross-robot "
                         "orientation conventions in ABSOLUTE replay (cancels "
                         "out in relative mode). Overrides the dataset's own "
                         "meta/piper_replay.json convention; RoboMIND agilex "
                         "uses -90,0,-90 (gripper rotation is stored relative "
                         "to the neutral pose)")
    ap.add_argument("--gripper-max", type=float, default=None,
                    help="raw gripper value meaning fully open (modality "
                         "datasets; default: the episode's max)")
    ap.add_argument("--joints", action="store_true",
                    help="replay the recorded JOINT trajectory instead of EEF "
                         "actions (native datasets only)")
    ap.add_argument("--viz", action="store_true",
                    help="live rerun 3D visualization during playback")
    ap.add_argument("--log", default=None, metavar="PATH",
                    help="save the per-tick command/execution log here "
                         "(default: <root>/replay_logs/ep<N>_<time>.npz)")
    ap.add_argument("--no-log", action="store_true",
                    help="do not save a replay log")
    ap.add_argument("--no-home", action="store_true",
                    help="skip the initial homing move (default: move to the "
                         "config home joints first — the host IK warm-starts "
                         "from the current joints, so replays from different "
                         "start postures land in different joint branches "
                         "and execute differently)")
    args = ap.parse_args()

    overrides: dict = {}
    if args.sim:
        overrides["arm"] = {"backend": "fake"}
    elif args.position:
        overrides["arm"] = {"backend": "piper"}
    cfg = load_config(args.config, overrides)

    root = Path(args.root).expanduser()
    actions, grip_raw, obs, joints, ts, schema = load_episode(
        root, args.episode, args.arm_side)
    if args.joints and joints is None:
        raise SystemExit("[replay] --joints: this dataset exposes no joint "
                         "channel for this arm")
    if args.joints and (args.offset is not None or args.rot_offset is not None):
        raise SystemExit("[replay] --offset/--rot-offset only apply to EEF replay")
    rot_vals, rot_src = None, ""
    if args.rot_offset is not None:
        rot_vals = [float(v) for v in args.rot_offset.split(",")]
        rot_src = "--rot-offset"
    elif not args.joints:
        rot_vals = dataset_rot_offset(root)
        rot_src = "dataset meta/piper_replay.json"
    if rot_vals is not None:
        if len(rot_vals) != 3:
            raise SystemExit(f"[replay] rot offset needs 3 comma-separated "
                             f"degrees (R,P,Y), got {rot_vals} from {rot_src}")
        actions = apply_rot_offset(actions, rot_vals)
        print(f"[replay] tool-side rotation offset rpy "
              f"{[round(v, 1) for v in rot_vals]} deg ({rot_src})")
    relative = (args.relative or args.anchor is not None
                or (schema == "modality" and not args.absolute))
    if args.anchor is not None and args.absolute:
        raise SystemExit("[replay] --anchor and --absolute are mutually exclusive")
    n = len(actions)
    duration = (ts[-1] - ts[0]) / max(args.speed, 0.1)
    print(f"[replay] episode {args.episode}: {n} steps, "
          f"{ts[-1] - ts[0]:.1f}s recorded -> {duration:.1f}s at {args.speed:.1f}x")
    print(f"[replay] schema {schema}"
          + (f", arm {args.arm_side}" if schema == "modality" else "")
          + (", JOINT space" if args.joints else
             f", {'RELATIVE to current pose' if relative else 'ABSOLUTE'}")
          + f", backend {cfg.arm.backend}")

    grip, grip_note = map_gripper(grip_raw, float(cfg.gripper.max_width),
                                  args.gripper_max)
    print(f"[replay] gripper: {grip_note}")

    arm = make_arm(cfg)
    time.sleep(2.0)  # MIT entry settle

    homed = False
    if not args.no_home:
        home = [float(x) for x in (cfg.arm.get("home_joints")
                                   or DEFAULT_HOME_JOINTS)]
        print(f"[replay] homing to "
              f"{[round(d) for d in np.degrees(home)]} deg for a "
              "reproducible IK start — stand clear! (skip with --no-home)")
        arm.move_joints(home)
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            s = arm.get_state()
            if s.joints_valid and float(np.max(np.abs(
                    s.joints - np.asarray(home)))) < 0.03:
                homed = True
                break
            time.sleep(0.05)
        if not homed:
            print("[replay] WARNING: homing timed out — continuing from the "
                  "current pose (run may not be reproducible)")
        time.sleep(0.5)  # let the pose feedback settle

    st = arm.get_state()
    assert st.eef_valid, "no EEF feedback"

    viz = None
    if args.viz:
        from ..visualize import RerunViz
        viz = RerunViz(spawn=True)

    n_clamped = None
    if args.joints:
        targets = joints.copy()
        start = st.joints.copy()
        gap = np.degrees(np.abs(targets[0] - start).max())
        print(f"[replay] approaching start joints (max {gap:.0f} deg away) "
              "in 3 s — stand clear!")
        # Pre-flight: recorded joints vs this arm's limits (config ∩ SDK
        # preset). External data may use postures this arm cannot reach —
        # those get clamped by the backend and the motion distorts.
        try:
            from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD
            preset = ROBOT_JOINT_LIMIT_PRESET_RAD[cfg.arm.model.lower()]
            sdk = np.array([list(v) for v in preset.values()], dtype=float)
            lo = np.maximum(np.radians(cfg.impedance.joint_limits_deg.lower),
                            sdk[:, 0])
            hi = np.minimum(np.radians(cfg.impedance.joint_limits_deg.upper),
                            sdk[:, 1])
            bad = ((targets < lo) | (targets > hi))
            if bad.any():
                per_joint = bad.mean(axis=0) * 100
                worst = ", ".join(f"j{i + 1}:{p:.0f}%"
                                  for i, p in enumerate(per_joint) if p > 0)
                print(f"[replay] WARNING: {bad.any(axis=1).mean() * 100:.0f}% "
                      f"of frames exceed this arm's joint limits ({worst}) — "
                      "they will be CLAMPED and the posture will distort")
        except Exception as e:
            print(f"[replay] (joint-limit pre-check skipped: {e})")
    else:
        anchor = st.eef_pose6.copy()
        if args.anchor is not None:
            vals = [float(v) for v in args.anchor.split(",")]
            if len(vals) not in (3, 6):
                raise SystemExit("[replay] --anchor needs 3 or 6 comma-"
                                 "separated numbers (x,y,z[,R,P,Y deg])")
            anchor[:3] = vals[:3]
            if len(vals) == 6:
                anchor[3:6] = np.radians(vals[3:6])
            print(f"[replay] fixed anchor: pos {np.round(anchor[:3], 3)} "
                  f"rpy {np.round(np.degrees(anchor[3:6]), 1)} deg")
        targets = build_targets(actions, relative,
                                anchor if relative else None,
                                scale=args.scale)
        if relative and args.scale != 1.0:
            print(f"[replay] translation deltas scaled x{args.scale:g}")
        offset_vec = np.zeros(3)
        off_src = None
        if args.offset is not None:
            offset_vec = np.array([float(v) for v in args.offset.split(",")],
                                  dtype=float)
            if offset_vec.shape != (3,):
                raise SystemExit("[replay] --offset needs 3 comma-separated "
                                 "numbers (dx,dy,dz)")
            off_src = "--offset"
        elif not relative:
            conv = dataset_conventions(root).get("offset_m")
            if conv is not None:
                offset_vec = np.array([float(v) for v in conv], dtype=float)
                off_src = "dataset meta/piper_replay.json"
        if off_src is not None:
            targets[:, :3] += offset_vec
            print(f"[replay] translation offset applied: "
                  f"{np.round(offset_vec, 3)} m ({off_src})")

        # Workspace clamp — external data may leave this arm's safe box.
        ws_min = np.array(cfg.teleop.limits.workspace_min, dtype=float)
        ws_max = np.array(cfg.teleop.limits.workspace_max, dtype=float)
        clipped = np.clip(targets[:, :3], ws_min, ws_max)
        n_clamped = int((np.abs(clipped - targets[:, :3]) > 1e-9).any(axis=1).sum())
        targets[:, :3] = clipped
        if n_clamped:
            print(f"[replay] WARNING: {n_clamped}/{n} targets "
                  f"({100 * n_clamped / n:.0f}%) clamped to the workspace box — "
                  f"the reproduced motion will be distorted at the edges")

        start = st.eef_pose6.copy()
        gap = np.linalg.norm(targets[0][:3] - start[:3])
        print(f"[replay] approaching start pose ({gap * 1000:.0f} mm away) "
              "in 3 s — stand clear!")

    command = arm.command_joints if args.joints else \
        (lambda pose: arm.command_eef(list(pose)))

    # Approach: measured -> first target, linear, 3 s.
    t0 = time.monotonic()
    while True:
        a = min(1.0, (time.monotonic() - t0) / 3.0)
        command((start + a * (targets[0] - start)).tolist())
        arm.command_gripper(float(grip[0]))
        if a >= 1.0:
            break
        time.sleep(1 / 50)

    print("[replay] playing …")
    t0 = time.monotonic()
    meas_eef = np.zeros((n, 6))
    eef_valid = np.zeros(n, dtype=bool)
    meas_joints = np.zeros((n, 6))
    joints_valid = np.zeros(n, dtype=bool)
    aborted = False
    i = 0
    while i < n and not aborted:
        t = (time.monotonic() - t0) * args.speed
        # Index by recorded timestamp (robust to tick jitter in the data).
        while i < n and ts[i] - ts[0] <= t:
            command(targets[i].tolist())
            arm.command_gripper(float(grip[i]))
            s = arm.get_state()
            if s.eef_valid:
                meas_eef[i] = s.eef_pose6
                eef_valid[i] = True
            if s.joints_valid:
                meas_joints[i] = s.joints
                joints_valid[i] = True
            if viz is not None:
                viz.log_arm(s, None if args.joints else targets[i])
            if s.arm_status not in (0, -1):
                print(f"\n[replay] ABORT: arm fault status={s.arm_status}")
                aborted = True
                break
            i += 1
        time.sleep(0.002)
        if i % 60 == 0:
            print(f"\r[replay] {i}/{n}", end="", flush=True)

    if not aborted:
        print(f"\r[replay] {n}/{n} done")
    print("=============== REPLAY REPORT ===============")
    if aborted:
        print(f"ABORTED at tick {i}/{n} — stats cover the completed part")
    if args.joints:
        rec = joints[joints_valid]
        rep = meas_joints[joints_valid]
        if len(rec):
            err = np.degrees(np.linalg.norm(rep - rec, axis=1))
            print(f"vs RECORDED joints : RMS {np.sqrt(np.mean(err ** 2)):6.2f} deg   "
                  f"max {err.max():6.2f} deg")
    else:
        # Reproduction error: replayed measured positions vs the recorded
        # ones (in relative mode the recorded trajectory is re-anchored to
        # the pose the arm actually started from).
        rec = obs[:, :3].copy()
        if relative:
            rec = (rec - rec[0]) * args.scale + anchor[:3]
        else:
            rec = rec + offset_vec  # compare in the same shifted frame
        rec = rec[eef_valid]
        rep = meas_eef[eef_valid, :3]
        if len(rec):
            err = np.linalg.norm(rep - rec, axis=1)
            cmd_err = np.linalg.norm(targets[eef_valid, :3] - rep, axis=1)
            print(f"vs RECORDED observation : RMS "
                  f"{np.sqrt(np.mean(err ** 2)) * 1000:6.1f} mm   "
                  f"max {err.max() * 1000:6.1f} mm")
            print(f"vs COMMANDED action     : RMS "
                  f"{np.sqrt(np.mean(cmd_err ** 2)) * 1000:6.1f} mm   "
                  f"max {cmd_err.max() * 1000:6.1f} mm")
    # Which backend guards shaped the motion (IK failures, joint limits,
    # joint-velocity saturation) — the usual culprits when the workspace
    # box never clamped but the arm still did not follow.
    guards = {}
    if getattr(arm, "limits", None) is not None:
        guards = arm.limits.snapshot()
    if guards:
        print("arm guards fired: " + ", ".join(
            f"{k} x{v['count']}" + (f" ({v['detail']})" if v["detail"] else "")
            for k, v in guards.items()))
    else:
        print("arm guards fired: none recorded")
    print("=============================================")

    if not args.no_log:
        log_path = (Path(args.log).expanduser() if args.log else
                    root / "replay_logs" /
                    f"ep{args.episode:06d}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz")
        save_replay_log(
            log_path, ts, targets, grip, meas_eef, eef_valid,
            meas_joints, joints_valid, actions, obs,
            {"argv": sys.argv[1:], "root": str(root), "episode": args.episode,
             "schema": schema, "arm_side": args.arm_side,
             "mode": "joints" if args.joints else "eef",
             "backend": cfg.arm.backend, "relative": relative,
             "speed": args.speed, "scale": args.scale,
             "offset": (offset_vec.tolist() if not args.joints else None),
             "rot_offset": rot_vals, "rot_offset_src": rot_src or None,
             "anchor": args.anchor, "n_clamped": n_clamped,
             "homed": homed,
             "aborted": aborted, "ticks_done": int(i),
             "arm_guards": guards})
        print(f"[replay] log: {log_path}")
        print(f"[replay] inspect: piper-viz --replay-log {log_path}")
    arm.stop()


if __name__ == "__main__":
    main()
