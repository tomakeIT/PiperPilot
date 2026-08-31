"""Dataset finalization: generate relative_stats.json / delta_stats.json for
trainers that consume relative / delta action representations.

Semantics (HORIZON=30 future window, per-episode grouping):

- relative (joint-like columns):  rel[i] = action[i : i+H] - state[i]
- relative (EEF columns):         T_rel  = T_state(i)^-1 @ T_action(i+k)
      R_rel = R_s^T R_a ;  t_rel = R_s^T (t_a - t_s) ;  quat back to wxyz
- delta (joint-like):  d[0] = a[0]-state ; d[k] = a[k]-a[k-1]
- delta (EEF):         chained relative poses along the horizon
Stats = {mean,std,min,max,q01,q99} over the stacked (N, H, dim) arrays,
plus standard per-row stats for every observation.* column.

Usage:
    piper-finalize --root ~/piper_datasets/<task> [--horizon 30]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from .. import xr_math

EEF_COLUMNS = {"action.arm_eef": "observation.arm_eef"}
JOINT_COLUMNS = {"action.gripper": "observation.gripper"}


def _stack(col: pd.Series) -> np.ndarray:
    return np.vstack([np.asarray(x, dtype=np.float32) for x in col])


def _column_stats(arr: np.ndarray) -> dict:
    return {
        "mean": np.mean(arr, axis=0).tolist(),
        "std": np.std(arr, axis=0).tolist(),
        "min": np.min(arr, axis=0).tolist(),
        "max": np.max(arr, axis=0).tolist(),
        "q01": np.quantile(arr, 0.01, axis=0).tolist(),
        "q99": np.quantile(arr, 0.99, axis=0).tolist(),
    }


def _future(actions: np.ndarray, i: int, horizon: int) -> np.ndarray:
    fut = actions[i : i + horizon]
    if len(fut) < horizon:
        pad = np.repeat(fut[-1:], horizon - len(fut), axis=0)
        fut = np.vstack([fut, pad])
    return fut


def _pose7_to_pr(pose7: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """[pos3, quat_wxyz4] -> (pos, R)."""
    q_xyzw = xr_math.quat_wxyz_to_xyzw(pose7[3:7])
    return pose7[:3], xr_math.quat_xyzw_to_matrix(q_xyzw)


def _relative_pose7(action7: np.ndarray, state7: np.ndarray) -> np.ndarray:
    t_a, r_a = _pose7_to_pr(action7)
    t_s, r_s = _pose7_to_pr(state7)
    r_rel = r_s.T @ r_a
    t_rel = r_s.T @ (t_a - t_s)
    q_rel = xr_math.quat_xyzw_to_wxyz(xr_math.matrix_to_quat_xyzw(r_rel))
    return np.concatenate([t_rel, q_rel]).astype(np.float32)


def _episode_relative(actions: np.ndarray, states: np.ndarray, horizon: int,
                      eef: bool, delta: bool) -> list[np.ndarray]:
    out = []
    for i in range(len(actions)):
        fut = _future(actions, i, horizon)
        if not eef:
            if delta:
                d = np.zeros_like(fut)
                d[0] = fut[0] - states[i]
                d[1:] = fut[1:] - fut[:-1]
                out.append(d)
            else:
                out.append(fut - states[i])
        else:
            rel = np.zeros_like(fut)
            if delta:
                prev = states[i]
                for k in range(horizon):
                    rel[k] = _relative_pose7(fut[k], prev)
                    prev = fut[k]
            else:
                for k in range(horizon):
                    rel[k] = _relative_pose7(fut[k], states[i])
            out.append(rel)
    return out


def compute_horizon_stats(root: Path, horizon: int, delta: bool) -> dict:
    parquets = sorted(root.glob("data/*/*.parquet"))
    if not parquets:
        raise SystemExit(f"no parquet files under {root}/data")
    all_df = pd.concat([pd.read_parquet(p) for p in parquets], ignore_index=True)

    stats: dict[str, dict] = {}
    pairs = {**JOINT_COLUMNS, **EEF_COLUMNS}
    for action_col, state_col in pairs.items():
        if action_col not in all_df.columns or state_col not in all_df.columns:
            print(f"  skip {action_col} (missing)")
            continue
        eef = action_col in EEF_COLUMNS
        collected: list[np.ndarray] = []
        for _, ep in all_df.groupby("episode_index"):
            actions = _stack(ep[action_col])
            states = _stack(ep[state_col])
            collected.extend(_episode_relative(actions, states, horizon, eef, delta))
        stats[action_col] = _column_stats(np.stack(collected))
        print(f"  {action_col}: {'delta' if delta else 'relative'} over "
              f"{len(collected)} windows")

    for col in [c for c in all_df.columns if c.startswith("observation.")]:
        try:
            stats[col] = _column_stats(_stack(all_df[col]))
        except Exception as e:
            print(f"  warn: stats for {col} failed: {e}")
    return stats


def main() -> None:
    ap = argparse.ArgumentParser(description="Finalize a Piper LeRobot dataset")
    ap.add_argument("--root", required=True)
    ap.add_argument("--horizon", type=int, default=30)
    args = ap.parse_args()

    root = Path(args.root).expanduser()
    meta = root / "meta"
    assert (meta / "info.json").exists(), f"not a dataset root: {root}"

    print("[finalize] relative_stats.json …")
    rel = compute_horizon_stats(root, args.horizon, delta=False)
    with open(meta / "relative_stats.json", "w") as f:
        json.dump(rel, f, indent=2)

    print("[finalize] delta_stats.json …")
    dlt = compute_horizon_stats(root, args.horizon, delta=True)
    with open(meta / "delta_stats.json", "w") as f:
        json.dump(dlt, f, indent=2)

    print(f"[finalize] done: {meta}/relative_stats.json, delta_stats.json")


if __name__ == "__main__":
    main()
