"""LeRobot v2.0 dataset writer with GR00T-style modality metadata.

Layout:

    <root>/
    ├── meta/
    │   ├── info.json          # fps, features, data_path/video_path, totals
    │   ├── modality.json      # subkey -> (start, end, original_key, ...)
    │   ├── episodes.jsonl     # {"episode_index", "length", "tasks", "task_index"}
    │   ├── tasks.jsonl        # {"task_index", "task"}
    │   └── stats.json         # per-column {mean, std, min, max, q01, q99}
    ├── data/chunk-XXX/episode_XXXXXX.parquet
    ├── videos/chunk-XXX/<video_key>/episode_XXXXXX.mp4
    └── extras/                # non-standard: raw quest stream dumps (debug)

Piper feature columns (float32 lists per cell; conventions: meters/radians,
quaternions wxyz, absolute actions, arm-base frame):
    observation.arm_joint [6]   measured joint positions
    observation.arm_eef   [7]   measured flange pose: pos(3) + quat_wxyz(4)
    observation.gripper   [1]   measured width (m)
    action.arm_eef        [7]   commanded absolute EEF pose
    action.gripper        [1]   commanded width (m)
plus bookkeeping scalars: timestamp (float32), frame_index, episode_index,
index, task_index (int64).

Video PTS is seconds relative to episode start — the trainer fetches frames
by parquet `timestamp`, so both clocks must share the episode-start origin.
"""

from __future__ import annotations

import json
import shutil
from fractions import Fraction
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

FEATURE_COLUMNS: dict[str, int] = {
    "observation.arm_joint": 6,
    "observation.arm_eef": 7,
    "observation.gripper": 1,
    "action.arm_eef": 7,
    "action.gripper": 1,
}

MODALITY_JSON = {
    "state": {
        "arm_joint": {
            "start": 0, "end": 6,
            "original_key": "observation.arm_joint",
            "dtype": "float32", "absolute": True,
        },
        "eef_pos": {
            "start": 0, "end": 3,
            "original_key": "observation.arm_eef",
            "dtype": "float32", "absolute": True,
        },
        "eef_rot": {
            "start": 3, "end": 7,
            "original_key": "observation.arm_eef",
            "rotation_type": "quat_wxyz",
            "dtype": "float32", "absolute": True,
        },
        "gripper": {
            "start": 0, "end": 1,
            "original_key": "observation.gripper",
            "dtype": "float32", "absolute": True,
        },
    },
    "action": {
        "eef_pos": {
            "start": 0, "end": 3,
            "original_key": "action.arm_eef",
            "dtype": "float32", "absolute": True,
        },
        "eef_rot": {
            "start": 3, "end": 7,
            "original_key": "action.arm_eef",
            "rotation_type": "quat_wxyz",
            "dtype": "float32", "absolute": True,
        },
        "gripper": {
            "start": 0, "end": 1,
            "original_key": "action.gripper",
            "dtype": "float32", "absolute": True,
        },
    },
    "video": {},      # filled per camera: name -> {original_key: observation.images.<name>}
    "annotation": {
        "task_index": {},
    },
}


class VideoEncoder:
    """Streaming mp4 encoder with explicit millisecond PTS (PyAV)."""

    def __init__(self, path: Path, width: int, height: int, fps: int, video_cfg: dict):
        import av

        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._container = av.open(str(path), mode="w")
        self._stream = self._container.add_stream(video_cfg.get("codec", "libx264"), rate=fps)
        self._stream.width = width
        self._stream.height = height
        self._stream.pix_fmt = video_cfg.get("pix_fmt", "yuv420p")
        self._stream.codec_context.time_base = Fraction(1, 1000)
        self._stream.codec_context.options = {
            "crf": str(video_cfg.get("crf", 23)),
            "preset": str(video_cfg.get("preset", "veryfast")),
        }
        self.n_frames = 0
        self._last_pts = -1

    def add(self, rgb: np.ndarray, t_rel: float) -> None:
        import av

        pts = int(round(t_rel * 1000))
        if pts <= self._last_pts:
            return  # never emit non-monotonic PTS
        self._last_pts = pts
        frame = av.VideoFrame.from_ndarray(rgb, format="rgb24")
        frame.pts = pts
        frame.time_base = Fraction(1, 1000)
        for packet in self._stream.encode(frame):
            self._container.mux(packet)
        self.n_frames += 1

    def close(self) -> None:
        for packet in self._stream.encode():
            self._container.mux(packet)
        self._container.close()


class EpisodeBuffer:
    """Accumulates one episode; owned by LeRobotWriter between start/finish."""

    def __init__(self, episode_index: int, task: str, video_paths: dict[str, Path],
                 encoders: dict[str, VideoEncoder], raw_quest_path: Path | None):
        self.episode_index = episode_index
        self.task = task
        self.rows: list[dict] = []
        self.video_paths = video_paths
        self.encoders = encoders
        self._raw_quest_file = open(raw_quest_path, "w") if raw_quest_path else None

    def add_frame(self, t_rel: float, obs_action: dict[str, np.ndarray]) -> None:
        row = {"timestamp": np.float32(t_rel)}
        for col, dim in FEATURE_COLUMNS.items():
            v = obs_action.get(col)
            if v is None:
                v = np.zeros(dim, dtype=np.float32)
            v = np.asarray(v, dtype=np.float32)
            assert v.shape == (dim,), f"{col}: expected ({dim},), got {v.shape}"
            row[col] = v
        self.rows.append(row)

    def add_video_frame(self, cam_name: str, rgb: np.ndarray, t_rel: float) -> None:
        enc = self.encoders.get(cam_name)
        if enc is not None:
            enc.add(rgb, t_rel)

    def add_raw_quest(self, line: dict) -> None:
        if self._raw_quest_file is not None:
            self._raw_quest_file.write(json.dumps(line) + "\n")

    def close_files(self) -> None:
        for enc in self.encoders.values():
            enc.close()
        if self._raw_quest_file is not None:
            self._raw_quest_file.close()

    def __len__(self) -> int:
        return len(self.rows)


class LeRobotWriter:
    def __init__(self, root: str | Path, fps: int, video_specs: list[dict],
                 chunks_size: int = 1000, robot_type: str = "piper",
                 video_cfg: dict | None = None, save_raw_quest: bool = False):
        """video_specs: [{name, width, height, fps}], name -> observation.images.<name>."""
        self.root = Path(root).expanduser()
        self.fps = fps
        self.chunks_size = chunks_size
        self.robot_type = robot_type
        self.video_cfg = video_cfg or {}
        self.save_raw_quest = save_raw_quest
        self.video_specs = {v["name"]: v for v in video_specs}

        # Lazy on-disk creation: nothing is written until the first episode
        # starts, so a pure-teleop session (collect without ever recording)
        # leaves no dataset directory behind.
        self._pending_session: dict | None = None

        self.episodes: list[dict] = []   # episodes.jsonl rows
        self.tasks: dict[str, int] = {}  # task string -> task_index
        self.total_frames = 0
        self._load_existing()
        self._active: EpisodeBuffer | None = None
        # Episodes below this index belong to previous sessions (appending to
        # an existing dataset) and are protected from delete_last_episode().
        self._session_start_index = len(self.episodes)

    # -- resume support ------------------------------------------------------

    def _load_existing(self) -> None:
        ep_path = self.root / "meta" / "episodes.jsonl"
        if ep_path.exists():
            with open(ep_path) as f:
                self.episodes = [json.loads(line) for line in f if line.strip()]
            self.total_frames = sum(e["length"] for e in self.episodes)
        tasks_path = self.root / "meta" / "tasks.jsonl"
        if tasks_path.exists():
            with open(tasks_path) as f:
                for line in f:
                    if line.strip():
                        d = json.loads(line)
                        self.tasks[d["task"]] = d["task_index"]
        if self.episodes:
            print(f"[writer] resuming dataset at {self.root}: "
                  f"{len(self.episodes)} episodes, {self.total_frames} frames")

    @property
    def next_episode_index(self) -> int:
        return len(self.episodes)

    def _chunk(self, ep_idx: int) -> str:
        return f"chunk-{ep_idx // self.chunks_size:03d}"

    def video_key(self, cam_name: str) -> str:
        return f"observation.images.{cam_name}"

    # -- episode lifecycle ------------------------------------------------------

    def _ensure_root(self) -> None:
        """Create the dataset skeleton and flush the pending session record
        (deferred from record_session for fresh datasets)."""
        (self.root / "meta").mkdir(parents=True, exist_ok=True)
        (self.root / "extras").mkdir(exist_ok=True)
        if self._pending_session is not None:
            pending, self._pending_session = self._pending_session, None
            self._write_session(pending)

    def start_episode(self, task: str) -> EpisodeBuffer:
        assert self._active is None, "previous episode not finished"
        self._ensure_root()
        ep_idx = self.next_episode_index
        encoders: dict[str, VideoEncoder] = {}
        video_paths: dict[str, Path] = {}
        for name, spec in self.video_specs.items():
            path = (self.root / "videos" / self._chunk(ep_idx) / self.video_key(name)
                    / f"episode_{ep_idx:06d}.mp4")
            video_paths[name] = path
            encoders[name] = VideoEncoder(path, spec["width"], spec["height"],
                                          spec["fps"], self.video_cfg)
        raw_path = None
        if self.save_raw_quest:
            raw_path = self.root / "extras" / f"episode_{ep_idx:06d}.quest.jsonl"
        self._active = EpisodeBuffer(ep_idx, task, video_paths, encoders, raw_path)
        return self._active

    def discard_episode(self) -> None:
        ep = self._active
        if ep is None:
            return
        self._active = None
        ep.close_files()
        for path in ep.video_paths.values():
            path.unlink(missing_ok=True)
        print(f"[writer] episode {ep.episode_index} discarded ({len(ep)} frames)")

    def finish_episode(self) -> dict | None:
        ep = self._active
        if ep is None:
            return None
        self._active = None
        ep.close_files()
        if len(ep) == 0:
            for path in ep.video_paths.values():
                path.unlink(missing_ok=True)
            print("[writer] empty episode dropped")
            return None

        ep_idx = ep.episode_index
        task_index = self.tasks.setdefault(ep.task, len(self.tasks))

        n = len(ep.rows)
        df = pd.DataFrame({
            col: [r[col] for r in ep.rows] for col in FEATURE_COLUMNS
        })
        df["timestamp"] = np.array([r["timestamp"] for r in ep.rows], dtype=np.float32)
        df["frame_index"] = np.arange(n, dtype=np.int64)
        df["episode_index"] = np.int64(ep_idx)
        df["index"] = np.arange(self.total_frames, self.total_frames + n, dtype=np.int64)
        df["task_index"] = np.int64(task_index)

        pq_dir = self.root / "data" / self._chunk(ep_idx)
        pq_dir.mkdir(parents=True, exist_ok=True)
        pq.write_table(pa.Table.from_pandas(df), pq_dir / f"episode_{ep_idx:06d}.parquet")

        row = {
            "episode_index": ep_idx,
            "length": n,
            "tasks": [ep.task],
            "task_index": task_index,
        }
        if ep.encoders:
            # convention: a plain int (primary camera frame count).
            # Per-camera counts go under a non-canonical key for our own QA.
            counts = {self.video_key(name): enc.n_frames
                      for name, enc in ep.encoders.items()}
            row["video_frame_length"] = min(counts.values())
            row["video_frame_lengths"] = counts
        self.episodes.append(row)
        self.total_frames += n
        self._write_meta()
        print(f"[writer] episode {ep_idx} saved: {n} frames, "
              f"{ {k: v.n_frames for k, v in ep.encoders.items()} } video frames")
        return row

    def delete_last_episode(self) -> dict | None:
        """Delete the most recently SAVED episode: its parquet, videos and raw
        dumps are removed and all meta files are rewritten. Only episodes
        saved by this writer instance can be deleted — when appending to an
        existing dataset, earlier sessions' episodes are protected. Returns
        the deleted episodes.jsonl row, or None if there is nothing eligible."""
        if self._active is not None:
            return None  # an episode is in flight; discard that one instead
        if len(self.episodes) <= self._session_start_index:
            return None  # nothing saved this session
        row = self.episodes.pop()
        ep_idx = row["episode_index"]
        (self.root / "data" / self._chunk(ep_idx)
         / f"episode_{ep_idx:06d}.parquet").unlink(missing_ok=True)
        for name in self.video_specs:
            (self.root / "videos" / self._chunk(ep_idx) / self.video_key(name)
             / f"episode_{ep_idx:06d}.mp4").unlink(missing_ok=True)
        (self.root / "extras" / f"episode_{ep_idx:06d}.quest.jsonl").unlink(missing_ok=True)
        self.total_frames -= row["length"]
        self._write_meta()
        print(f"[writer] episode {ep_idx} deleted ({row['length']} frames)")
        return row

    # -- meta files --------------------------------------------------------------

    def _features(self) -> dict:
        feats: dict[str, dict] = {}
        for name, spec in self.video_specs.items():
            feats[self.video_key(name)] = {
                "dtype": "video",
                "shape": [spec["height"], spec["width"], 3],
                "names": ["height", "width", "channel"],
                "video_info": {"video.fps": spec["fps"]},
            }
        for col, dim in FEATURE_COLUMNS.items():
            feats[col] = {"dtype": "float32", "shape": [dim], "names": None}
        for col in ("timestamp",):
            feats[col] = {"dtype": "float32", "shape": [1], "names": None}
        for col in ("frame_index", "episode_index", "index", "task_index"):
            feats[col] = {"dtype": "int64", "shape": [1], "names": None}
        return feats

    def _write_meta(self) -> None:
        meta = self.root / "meta"

        modality = json.loads(json.dumps(MODALITY_JSON))  # deep copy
        for name in self.video_specs:
            modality["video"][name] = {"original_key": self.video_key(name)}
        with open(meta / "modality.json", "w") as f:
            json.dump(modality, f, indent=2)

        info = {
            "codebase_version": "v2.0",
            "robot_type": self.robot_type,
            "fps": self.fps,
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "chunks_size": self.chunks_size,
            "total_episodes": len(self.episodes),
            "total_frames": self.total_frames,
            "total_tasks": len(self.tasks),
            "splits": {"train": f"0:{len(self.episodes)}"},
            "features": self._features(),
        }
        with open(meta / "info.json", "w") as f:
            json.dump(info, f, indent=2)

        with open(meta / "episodes.jsonl", "w") as f:
            for row in self.episodes:
                f.write(json.dumps(row) + "\n")

        with open(meta / "tasks.jsonl", "w") as f:
            for task, idx in sorted(self.tasks.items(), key=lambda kv: kv[1]):
                f.write(json.dumps({"task_index": idx, "task": task}) + "\n")

        self._write_stats(meta)

    def _write_stats(self, meta: Path) -> None:
        """Recompute meta/stats.json across all saved parquets (cheap at this scale)."""
        parquets = sorted(self.root.glob("data/*/*.parquet"))
        if not parquets:
            return
        frames = [pd.read_parquet(p) for p in parquets]
        all_df = pd.concat(frames, ignore_index=True)
        stats: dict[str, dict] = {}
        for col in list(FEATURE_COLUMNS) + ["timestamp"]:
            if col not in all_df.columns:
                continue
            if col == "timestamp":
                arr = all_df[col].to_numpy(dtype=np.float32).reshape(-1, 1)
            else:
                arr = np.vstack([np.asarray(x, dtype=np.float32) for x in all_df[col]])
            stats[col] = {
                "mean": np.mean(arr, axis=0).tolist(),
                "std": np.std(arr, axis=0).tolist(),
                "min": np.min(arr, axis=0).tolist(),
                "max": np.max(arr, axis=0).tolist(),
                "q01": np.quantile(arr, 0.01, axis=0).tolist(),
                "q99": np.quantile(arr, 0.99, axis=0).tolist(),
            }
        with open(meta / "stats.json", "w") as f:
            json.dump(stats, f, indent=2)

    # -- collection provenance ---------------------------------------------

    # Fields that must stay constant across sessions of one dataset: a policy
    # trained on this data is implicitly bound to them.
    _BINDING_KEYS = (
        ("control", "backend"),
        ("control", "impedance", "kp"),
        ("control", "impedance", "kd"),
        ("control", "impedance", "t_ff"),
        ("control", "impedance", "gravity_ff"),
        ("arm", "firmware"),
        ("recording", "fps"),
    )

    @staticmethod
    def _dig(d: dict, path: tuple):
        for k in path:
            if not isinstance(d, dict) or k not in d:
                return None
            d = d[k]
        return d

    def record_session(self, session_meta: dict) -> None:
        """Register a collection-session provenance record (controller
        backend, arm/firmware, control gains, cameras, …) and warn loudly if
        a binding-relevant parameter changed vs. the previous session.

        On a fresh dataset the write is deferred until the first episode
        starts (see _ensure_root) so never-recorded sessions leave no files;
        when appending to an existing dataset it is written immediately, so
        the binding check runs before any teleop happens."""
        if not (self.root / "meta").exists():
            self._pending_session = session_meta
            return
        self._write_session(session_meta)

    def _write_session(self, session_meta: dict) -> None:
        meta_dir = self.root / "meta"
        sessions_path = meta_dir / "collection_sessions.jsonl"

        prev = None
        if sessions_path.exists():
            lines = [ln for ln in sessions_path.read_text().splitlines() if ln.strip()]
            if lines:
                prev = json.loads(lines[-1])
        if prev is not None:
            diffs = []
            for path in self._BINDING_KEYS:
                a, b = self._dig(prev, path), self._dig(session_meta, path)
                if a != b:
                    diffs.append(f"  {'.'.join(path)}: {a!r} -> {b!r}")
            if diffs:
                print("[writer] " + "!" * 62)
                print("[writer] WARNING: control parameters CHANGED vs the previous")
                print("[writer] session of this dataset — a policy is bound to its")
                print("[writer] controller. Consider a fresh --root instead:")
                for d in diffs:
                    print("[writer]" + d)
                print("[writer] " + "!" * 62)

        with open(sessions_path, "a") as f:
            f.write(json.dumps(session_meta) + "\n")
        with open(meta_dir / "collection_meta.json", "w") as f:
            json.dump(session_meta, f, indent=2)

    def wipe(self) -> None:
        """Dangerous: delete the whole dataset root."""
        shutil.rmtree(self.root, ignore_errors=True)
