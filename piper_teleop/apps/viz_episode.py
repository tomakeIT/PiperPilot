"""Offline 3D episode viewer: check recorded EEF data before replaying it.

Loads an episode through the same loader as piper-replay (native and
GR00T/modality schemas) and writes a fully self-contained interactive HTML
viewer (no dependencies — open it in any browser):

- orbit / pan / zoom 3D view with the robot base plane z=0 drawn as a grid;
- the full EEF trajectory (time-color-coded), orientation triads along it,
  playback with time scrubbing and a per-frame pose readout (quat + RPY);
- the configured Piper workspace box with an offset/clamp preview and the
  matching piper-replay command line;
- data-sanity checks: frame-to-frame position/rotation jumps, quaternion
  norms, gripper range, workspace fit with a suggested --offset.

    piper-viz --root data/50_pourtea --episode 1 --arm-side right
    piper-viz --root data/50_pourtea --episode 1 --arm-side both --open
    piper-viz --root ~/piper_datasets/<task> --episode 0
    piper-viz --replay-log data/50_pourtea/replay_logs/ep000001_xxx.npz

--replay-log renders a piper-replay execution log instead: the commanded
targets (cmd, blue) and the measured execution (meas, orange) as two trails
in one scene, plus a divergence report (tracking error, guard hits).
"""

from __future__ import annotations

import argparse
import json
import webbrowser
from pathlib import Path

import numpy as np

from .. import xr_math
from ..config import load_config
from ._episode_viewer_html import TEMPLATE
from .replay import load_episode, map_gripper

POS_JUMP_M = 0.02      # flag consecutive-frame position steps above this
ANG_JUMP_DEG = 15.0    # ... and rotation steps above this


def _r6(a) -> list:
    return np.round(np.asarray(a, dtype=float), 6).tolist()


def suggest_offset(pos: np.ndarray, ws_min: np.ndarray, ws_max: np.ndarray,
                   margin: float = 0.01) -> tuple[np.ndarray, bool]:
    """Smallest per-axis translation that moves every position inside the
    workspace box (with a safety margin). When the trajectory span exceeds
    the box on some axis, fall back to centering. Returns (offset, fits)."""
    lo = (ws_min + margin) - pos.min(axis=0)
    hi = (ws_max - margin) - pos.max(axis=0)
    if (lo <= hi).all():
        return np.clip(0.0, lo, hi), True
    return (ws_min + ws_max) / 2 - (pos.min(axis=0) + pos.max(axis=0)) / 2, False


def _quat_step_deg(q: np.ndarray) -> np.ndarray:
    if len(q) < 2:
        return np.zeros(0)
    dots = np.clip(np.abs((q[1:] * q[:-1]).sum(axis=1)), -1.0, 1.0)
    return np.degrees(2.0 * np.arccos(dots))


def _arm_report(side: str, act7: np.ndarray, obs7: np.ndarray,
                grip: np.ndarray, grip_note: str,
                ws_min: np.ndarray, ws_max: np.ndarray) -> tuple[dict, list[str]]:
    p, q = act7[:, :3], act7[:, 3:7]
    dp = np.linalg.norm(np.diff(p, axis=0), axis=1)
    dang = _quat_step_deg(q)
    jumps = sorted(set((np.where(dp > POS_JUMP_M)[0] + 1).tolist())
                   | set((np.where(dang > ANG_JUMP_DEG)[0] + 1).tolist()))
    qn = np.linalg.norm(q, axis=1)
    inside = ((p >= ws_min - 1e-9) & (p <= ws_max + 1e-9)).all(axis=1).mean()
    off, fits = suggest_offset(p, ws_min, ws_max)
    po = p + off
    inside_off = ((po >= ws_min - 1e-9) & (po <= ws_max + 1e-9)).all(axis=1).mean()
    obs_eq = bool(np.allclose(act7, obs7, atol=1e-9))

    def rng(v):
        return f"[{v.min():+.3f}, {v.max():+.3f}]"

    lines = [f"[{side}]",
             f"  pos (m)  x {rng(p[:, 0])}  y {rng(p[:, 1])}  z {rng(p[:, 2])}",
             f"  |quat| in [{qn.min():.4f}, {qn.max():.4f}]"]
    if len(dp):
        lines.append(f"  max step {dp.max() * 1000:.1f} mm @ f{int(dp.argmax()) + 1}, "
                     f"rot {dang.max():.1f} deg @ f{int(dang.argmax()) + 1}")
        j = "none" if not jumps else (f"{len(jumps)} frames: "
                                      + ", ".join(map(str, jumps[:10]))
                                      + ("…" if len(jumps) > 10 else ""))
        lines.append(f"  jumps (>{POS_JUMP_M * 1000:.0f} mm or >{ANG_JUMP_DEG:.0f} deg): {j}")
    lines.append(f"  gripper raw [{grip.min():.3f}, {grip.max():.3f}] -> {grip_note}")
    lines.append("  observation pose == action pose"
                 if obs_eq else
                 f"  obs vs action pose: max gap "
                 f"{np.abs(act7[:, :3] - obs7[:, :3]).max() * 1000:.1f} mm")
    lines.append(f"  Piper workspace: {inside * 100:.0f}% of frames inside raw; "
                 f"suggested offset {off[0]:+.3f},{off[1]:+.3f},{off[2]:+.3f}"
                 + (f" -> {inside_off * 100:.0f}% inside"
                    if fits else " (span exceeds box; centered)"))
    return {"jumps": jumps, "off": np.round(off, 3).tolist(), "obs_eq": obs_eq}, lines


def generate(root: Path, episode: int, arm_side: str, cfg,
             out: Path | None = None, stride: int = 0,
             init_offset=(0.0, 0.0, 0.0)) -> tuple[Path, str]:
    ws_min = np.array(cfg.teleop.limits.workspace_min, dtype=float)
    ws_max = np.array(cfg.teleop.limits.workspace_max, dtype=float)
    gmax = float(cfg.gripper.max_width)

    sides = ["right", "left"] if arm_side == "both" else [arm_side]
    arms: dict = {}
    ts = None
    schema = None
    report: list[str] = []
    for side in sides:
        try:
            act7, grip, obs7, _joints, ts, schema = load_episode(root, episode, side)
        except SystemExit:
            if arm_side == "both" and arms:
                report.append(f"[{side}] not in this dataset — skipped")
                continue
            raise
        if schema == "native":
            side = "arm"
        gw, grip_note = map_gripper(grip, gmax)
        stats, lines = _arm_report(side, act7, obs7, grip, grip_note, ws_min, ws_max)
        report.extend(lines)
        arms[side] = {"ap": _r6(act7[:, :3]), "aq": _r6(act7[:, 3:7]),
                      "op": _r6(obs7[:, :3]), "oq": _r6(obs7[:, 3:7]),
                      "g": _r6(grip), "gw": _r6(gw), **stats}
        if schema == "native":
            break

    n = len(ts)
    dur = float(ts[-1] - ts[0])
    fps = (n - 1) / dur if dur > 0 else 30.0
    report.insert(0, f"{root} ep{episode} [{schema}] {n} frames, "
                     f"{dur:.1f} s @ {fps:.1f} Hz")

    data = {"root": str(root), "episode": episode, "schema": schema,
            "fps": round(fps, 3), "ts": _r6(ts), "arms": arms,
            "ws": {"min": ws_min.tolist(), "max": ws_max.tolist()},
            "stride": int(stride) if stride else max(1, round(n / 70)),
            "initOffset": [round(float(v), 6) for v in init_offset],
            "statsText": "\n".join(report)}
    out = out or root / f"episode_{episode:06d}_viz.html"
    html = (TEMPLATE
            .replace("__TITLE__", f"{root.name} ep{episode} EEF 3D")
            .replace("__DATA_JSON__", json.dumps(data, separators=(",", ":"))))
    out.write_text(html)
    return out, "\n".join(report)


def _pose6_to_pose7(pose6_arr: np.ndarray) -> np.ndarray:
    """(N,6) [pos, rpy] -> (N,7) [pos, quat wxyz]."""
    out = np.zeros((len(pose6_arr), 7))
    out[:, :3] = pose6_arr[:, :3]
    for i, p in enumerate(pose6_arr):
        rot = xr_math.rpy_to_matrix(p[3], p[4], p[5])
        out[i, 3:7] = xr_math.quat_xyzw_to_wxyz(xr_math.matrix_to_quat_xyzw(rot))
    return out


def generate_replay_log(log_path: Path, cfg, out: Path | None = None,
                        stride: int = 0) -> tuple[Path, str]:
    """Visualize a piper-replay log: commanded targets vs measured execution
    as two trails in the same 3D scene, with a where-did-it-diverge report."""
    ws_min = np.array(cfg.teleop.limits.workspace_min, dtype=float)
    ws_max = np.array(cfg.teleop.limits.workspace_max, dtype=float)
    gmax = float(cfg.gripper.max_width)

    z = np.load(log_path)
    meta = json.loads(str(z["meta"]))
    ts = np.asarray(z["ts"], dtype=float)
    grip = np.asarray(z["grip"], dtype=float)
    meas = np.asarray(z["meas_eef"], dtype=float)
    ev = np.asarray(z["eef_valid"], dtype=bool)
    n = len(ts)
    if not ev.any():
        raise SystemExit("[viz] log has no valid EEF feedback to display")
    # Fill invalid feedback rows (hold last / first valid) so trails stay
    # continuous; the count is reported below.
    first = meas[int(np.argmax(ev))].copy()
    last = first
    for i in range(n):
        if ev[i]:
            last = meas[i]
        else:
            meas[i] = last
    meas7 = _pose6_to_pose7(meas)

    report = [f"{log_path.name} [{meta.get('backend', '?')}] "
              f"{meta.get('mode', '?')} replay, {n} frames"
              + (", ABORTED at tick " + str(meta.get("ticks_done"))
                 if meta.get("aborted") else "")]
    gw, grip_note = map_gripper(grip, gmax)
    arms: dict = {}

    if meta.get("mode") == "eef":
        targets = np.asarray(z["targets"], dtype=float)
        cmd7 = _pose6_to_pose7(targets)
        derr = np.linalg.norm(cmd7[ev, :3] - meas7[ev, :3], axis=1)
        dots = np.clip(np.abs((cmd7[ev, 3:7] * meas7[ev, 3:7]).sum(axis=1)), -1, 1)
        rerr = np.degrees(2 * np.arccos(dots))
        bias = (meas7[ev, :3] - cmd7[ev, :3]).mean(axis=0) * 1000
        worst = int(np.argmax(derr))
        report += [
            f"  cmd vs meas: RMS {np.sqrt(np.mean(derr ** 2)) * 1000:.1f} mm, "
            f"max {derr.max() * 1000:.1f} mm @ f{worst}",
            f"  mean bias (meas-cmd) {bias[0]:+.1f},{bias[1]:+.1f},{bias[2]:+.1f} mm; "
            f"rot RMS {np.sqrt(np.mean(rerr ** 2)):.1f} deg, max {rerr.max():.1f} deg",
        ]
        # Did offset+clamp alter the recorded reference at all?
        if not meta.get("relative") and meta.get("n_clamped") is not None:
            report.append(f"  workspace-clamped targets: {meta['n_clamped']}/{n}")
        stats, lines = _arm_report("cmd", cmd7, cmd7, grip, grip_note,
                                   ws_min, ws_max)
        report += lines
        arms["cmd"] = {"ap": _r6(cmd7[:, :3]), "aq": _r6(cmd7[:, 3:7]),
                       "op": _r6(cmd7[:, :3]), "oq": _r6(cmd7[:, 3:7]),
                       "g": _r6(grip), "gw": _r6(gw), **stats}
    else:
        report.append("  (joint-space replay: commanded joints not shown in "
                      "the EEF view)")

    n_invalid = int((~ev).sum())
    if n_invalid:
        report.append(f"  invalid EEF feedback rows (held last pose): {n_invalid}")
    guards = meta.get("arm_guards") or {}
    report.append("  arm guards fired: " + (", ".join(
        f"{k} x{v['count']}" + (f" ({v['detail']})" if v.get("detail") else "")
        for k, v in guards.items()) if guards else "none recorded"))
    stats, lines = _arm_report("meas", meas7, meas7, grip, grip_note,
                               ws_min, ws_max)
    report += lines
    arms["meas"] = {"ap": _r6(meas7[:, :3]), "aq": _r6(meas7[:, 3:7]),
                    "op": _r6(meas7[:, :3]), "oq": _r6(meas7[:, 3:7]),
                    "g": _r6(grip), "gw": _r6(gw), **stats}

    dur = float(ts[-1] - ts[0]) if n > 1 else 0.0
    data = {"root": meta.get("root", ""), "episode": meta.get("episode", 0),
            "schema": f"replay-log:{meta.get('backend', '?')}",
            "fps": round((n - 1) / dur, 3) if dur > 0 else 30.0,
            "ts": _r6(ts), "arms": arms,
            "ws": {"min": ws_min.tolist(), "max": ws_max.tolist()},
            "stride": int(stride) if stride else max(1, round(n / 70)),
            "initOffset": [0.0, 0.0, 0.0],
            "statsText": "\n".join(report)}
    out = out or log_path.with_name(log_path.stem + "_viz.html")
    html = (TEMPLATE
            .replace("__TITLE__", f"{log_path.stem} cmd vs meas")
            .replace("__DATA_JSON__", json.dumps(data, separators=(",", ":"))))
    out.write_text(html)
    return out, "\n".join(report)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Write an interactive 3D HTML viewer for a recorded episode")
    ap.add_argument("--root", default=None)
    ap.add_argument("--replay-log", default=None, metavar="PATH",
                    help="visualize a piper-replay log (.npz): commanded vs "
                         "measured trails instead of a dataset episode")
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--arm-side", choices=["left", "right", "both"], default="right",
                    help="dual-arm (modality) datasets: which arm(s) to show")
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default=None,
                    help="output HTML path (default: <root>/episode_NNNNNN_viz.html)")
    ap.add_argument("--stride", type=int, default=0,
                    help="frames between orientation triads (default: auto)")
    ap.add_argument("--offset", default=None, metavar="DX,DY,DZ",
                    help="initial offset preview (same meaning as piper-replay --offset)")
    ap.add_argument("--open", action="store_true", help="open the result in a browser")
    args = ap.parse_args()

    off = [0.0, 0.0, 0.0]
    if args.offset is not None:
        off = [float(v) for v in args.offset.split(",")]
        if len(off) != 3:
            raise SystemExit("[viz] --offset needs 3 comma-separated numbers")
    out_path = Path(args.out).expanduser() if args.out else None
    if args.replay_log:
        out, report = generate_replay_log(Path(args.replay_log).expanduser(),
                                          load_config(args.config),
                                          out_path, args.stride)
    elif args.root:
        out, report = generate(Path(args.root).expanduser(), args.episode,
                               args.arm_side, load_config(args.config),
                               out_path, args.stride, off)
    else:
        ap.error("--root or --replay-log is required")
    print(report)
    print(f"[viz] wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")
    if args.open:
        webbrowser.open(out.resolve().as_uri())


if __name__ == "__main__":
    main()
