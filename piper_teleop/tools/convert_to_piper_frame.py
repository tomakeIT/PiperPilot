"""One-shot dataset conversion: rewrite a modality (GR00T/RoboMIND) dataset
into the Piper frame using its own meta/piper_replay.json conventions.

Why convert instead of transforming at load time: downstream trainers compute
and cache normalization statistics from the raw parquet (and bake them into
checkpoints), so a load-time transform leaves stats — and everything deployed
from them — describing the wrong frame. Converting once at the data boundary
makes model frame == robot frame by construction; dataloaders, inference and
eval then need no frame code at all.

What it does, for every pose block (both arms) in observation.state / action:
- position  += offset_m                       (into the Piper workspace)
- quat_xyzw -> R @ C -> quat_xyzw             (tool-side rotation fix)
- gripper    raw units -> width in m          (per-channel max = fully open;
                                               disable with --keep-gripper-raw)
- joints pass through unchanged.

The output root gets meta/ copied WITHOUT stats.json / relative_stats.json /
delta_stats.json (regenerate from converted values — conventions differ per
consumer) and WITHOUT piper_replay.json (the conversion is baked in; keeping
it would make piper-replay apply it twice). Provenance goes to
meta/frame_conversion.json. Videos are symlinked (--copy-videos to copy).

    piper-convert --root data/50_pourtea            # -> data/50_pourtea_piper
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import pandas as pd

from .. import xr_math
from ..apps.replay import dataset_conventions

STATS_FILES = ("stats.json", "relative_stats.json", "delta_stats.json")


def _pose_slices(mod: dict, group: str):
    """Yield (kind, slice) for every pose-related subkey of a modality group:
    kind in {'position', 'orientation', 'gripper'} for any arm prefix."""
    for name, m in mod[group].items():
        for kind in ("position", "orientation", "gripper"):
            if name == kind or name.endswith("_" + kind):
                yield kind, name, slice(int(m["start"]), int(m["end"]))


def convert(root: Path, out: Path, gripper_max_width: float = 0.07,
            keep_gripper_raw: bool = False, copy_videos: bool = False) -> Path:
    conv = dataset_conventions(root)
    if "rot_offset_deg" not in conv or "offset_m" not in conv:
        raise SystemExit(f"[convert] {root}/meta/piper_replay.json must define "
                         "rot_offset_deg and offset_m (nothing to convert)")
    C = xr_math.rpy_to_matrix(*np.radians([float(v) for v in conv["rot_offset_deg"]]))
    off = np.array([float(v) for v in conv["offset_m"]])
    mod = json.load(open(root / "meta" / "modality.json"))
    key_of = {"state": "observation.state", "action": "action"}

    parquets = sorted(root.glob("data/chunk-*/episode_*.parquet"))
    if not parquets:
        raise SystemExit(f"[convert] no episode parquets under {root}/data")

    # Pass 1: per-channel gripper full-open value (dataset-wide max).
    grip_max: dict[tuple, float] = {}
    if not keep_gripper_raw:
        for pq in parquets:
            df = pd.read_parquet(pq)
            for group, key in key_of.items():
                mat = np.vstack(df[key]).astype(float)
                for kind, name, sl in _pose_slices(mod, group):
                    if kind == "gripper":
                        k = (group, name)
                        grip_max[k] = max(grip_max.get(k, 0.0),
                                          float(mat[:, sl].max()))

    # Pass 2: rewrite parquets.
    for pq in parquets:
        df = pd.read_parquet(pq)
        for group, key in key_of.items():
            mat = np.vstack(df[key]).astype(float)
            for kind, name, sl in _pose_slices(mod, group):
                if kind == "position":
                    mat[:, sl] += off
                elif kind == "orientation":
                    if mod[group][name].get("rotation_type") != "quat_xyzw":
                        raise SystemExit(f"[convert] {group}.{name}: only "
                                         "quat_xyzw orientations supported")
                    for i in range(len(mat)):
                        r = xr_math.quat_xyzw_to_matrix(mat[i, sl])
                        mat[i, sl] = xr_math.matrix_to_quat_xyzw(r @ C)
                elif kind == "gripper" and not keep_gripper_raw:
                    top = grip_max[(group, name)]
                    if top > gripper_max_width * 1.5:  # raw units, not meters
                        mat[:, sl] = (np.clip(mat[:, sl] / top, 0.0, 1.0)
                                      * gripper_max_width)
            df[key] = list(mat)
        dst = out / pq.relative_to(root)
        dst.parent.mkdir(parents=True, exist_ok=True)
        df.to_parquet(dst)

    # meta/: copy everything except stats (stale) and the conventions sidecar
    # (baked in now — keeping it would make piper-replay apply it twice).
    (out / "meta").mkdir(parents=True, exist_ok=True)
    for f in sorted((root / "meta").iterdir()):
        if f.name in STATS_FILES or f.name == "piper_replay.json":
            continue
        shutil.copy2(f, out / "meta" / f.name)
    (out / "meta" / "frame_conversion.json").write_text(json.dumps({
        "source": str(root),
        "rot_offset_deg": conv["rot_offset_deg"],
        "offset_m": conv["offset_m"],
        "gripper": ("raw (unchanged)" if keep_gripper_raw else {
            "max_width_m": gripper_max_width,
            "raw_full_open": {f"{g}.{n}": v for (g, n), v in grip_max.items()},
        }),
        "note": "poses rewritten into the Piper base/flange frame; stats files "
                "removed on purpose — regenerate from converted values",
    }, indent=1))

    vids = root / "videos"
    if vids.exists():
        dst = out / "videos"
        if copy_videos:
            shutil.copytree(vids, dst, dirs_exist_ok=True)
        elif not dst.exists():
            dst.symlink_to(vids.resolve(), target_is_directory=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Rewrite a modality dataset into the Piper frame "
                    "(one-shot, driven by meta/piper_replay.json)")
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", default=None,
                    help="output root (default: <root>_piper)")
    ap.add_argument("--gripper-max-width", type=float, default=0.07,
                    help="width (m) that the raw full-open gripper value maps to")
    ap.add_argument("--keep-gripper-raw", action="store_true",
                    help="do not rescale gripper channels to meters")
    ap.add_argument("--copy-videos", action="store_true",
                    help="copy videos/ instead of symlinking")
    args = ap.parse_args()

    root = Path(args.root).expanduser()
    out = Path(args.out).expanduser() if args.out else \
        root.with_name(root.name + "_piper")
    if out.exists() and any(out.iterdir()):
        raise SystemExit(f"[convert] output {out} already exists — remove it "
                         "or pass a different --out")
    convert(root, out, args.gripper_max_width, args.keep_gripper_raw,
            args.copy_videos)
    n = len(list(out.glob("data/chunk-*/episode_*.parquet")))
    print(f"[convert] wrote {n} episodes to {out}")
    print("[convert] stats files intentionally NOT copied — downstream "
          "consumers regenerate them from the converted values")
    print(f"[convert] verify: piper-viz --root {out} --episode 0")


if __name__ == "__main__":
    main()
