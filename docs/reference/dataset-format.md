# Dataset format

Every recording session with `piper-collect` writes a **LeRobot v2.0 dataset**:
parquet tables for states and actions, mp4 videos for cameras, and JSON
metadata — including GR00T-style modality metadata and a provenance trail of
exactly which controller and hardware produced the data. The same dataset root
is consumed by [replay](../guide/replay.md), the
[inference runtime](../guide/inference.md), and any LeRobot-compatible
trainer.

This page is the precise spec of what lands on disk. For how to record data in
the first place, see the [data collection guide](../guide/data-collection.md).

## Directory layout

```
<root>/
├── meta/
│   ├── info.json                  # fps, features, path templates, totals
│   ├── modality.json              # GR00T-style modality map
│   ├── episodes.jsonl             # one line per episode
│   ├── tasks.jsonl                # task string <-> task_index
│   ├── stats.json                 # per-column statistics
│   ├── collection_meta.json       # provenance: latest session
│   └── collection_sessions.jsonl  # provenance: every session, append-only
├── data/
│   └── chunk-000/
│       ├── episode_000000.parquet
│       └── episode_000001.parquet
├── videos/
│   └── chunk-000/
│       ├── observation.images.cam_front/
│       │   └── episode_000000.mp4
│       └── observation.images.cam_wrist/
│           └── episode_000000.mp4
└── extras/                        # non-standard debug files (raw Quest stream)
```

Episodes are numbered with six digits (`episode_000000`) and grouped into
chunks of 1000 episodes by default (`chunk-000`, `chunk-001`, ...). Each
camera gets its own video directory named after its feature key,
`observation.images.<cam>` (camera names such as `cam_front` / `cam_wrist` /
`cam_back` are pinned to RealSense serials in the
[configuration](configuration.md)).

The writer supports resuming: if you point a new collection session at an
existing root, it reads `meta/episodes.jsonl` and `meta/tasks.jsonl` and
continues episode and frame indices from where the last session stopped.
Discarded and empty episodes leave nothing behind — their video files are
deleted and no metadata row is appended.

## Parquet schema

One parquet file per episode, one row per control tick (30 Hz). Feature cells
are float32 vectors.

| Column | Dim | Units / frame | Semantics |
|---|---|---|---|
| `observation.arm_joint` | 6 | rad | Measured joint positions |
| `observation.arm_eef` | 7 | m + quat wxyz, arm-base frame | Measured flange pose: pos (3) + quaternion wxyz (4) |
| `observation.gripper` | 1 | m | Measured gripper width |
| `action.arm_eef` | 7 | m + quat wxyz, arm-base frame | Commanded **absolute** EEF target pose |
| `action.gripper` | 1 | m | Commanded gripper width |

Plus the standard LeRobot bookkeeping columns:

| Column | dtype | Semantics |
|---|---|---|
| `timestamp` | float32 | Seconds since episode start (shared clock with video PTS) |
| `frame_index` | int64 | Row index within the episode, starting at 0 |
| `episode_index` | int64 | Episode number within the dataset |
| `index` | int64 | Global row index across the whole dataset |
| `task_index` | int64 | Index into `meta/tasks.jsonl` |

!!! note "Conventions"
    - **Units**: meters, radians, seconds. Gripper width in meters.
    - **Quaternions**: `wxyz` order, everywhere.
    - **Frame**: EEF poses (observed and commanded) are expressed in the
      arm-base frame.
    - **Actions are absolute**: `action.arm_eef` is the commanded target pose
      itself, not a delta. If your trainer wants relative or delta actions,
      use the statistics produced by [`piper-finalize`](#finalization-piper-finalize)
      rather than re-deriving conventions.

## Timing and alignment

Proprioception/action rows are written at 30 Hz and cameras record at 30 fps
(640x480), but the format does **not** rely on those rates matching exactly:

- All streams are stamped from one shared monotonic clock, converted to
  seconds relative to the episode start.
- Parquet rows carry that time in the `timestamp` column (float32 seconds).
- Video frames carry it as their PTS, in milliseconds (the mp4 stream uses a
  1/1000 time base). The encoder silently drops any frame whose PTS would not
  be monotonically increasing.

Because parquet `timestamp` and video PTS share the same episode-start origin,
a trainer fetches the video frame for a row **by timestamp** — there is no
index-based pairing and no resampling. This is why minor rate jitter is
harmless: if a camera hiccups or a control tick slips, every sample still
carries its true capture time, and lookups stay correct instead of drifting by
accumulated off-by-one errors.

!!! tip "Trainer-side rates"
    The data is recorded at 30 Hz, so set `action_frequency=30` on the trainer
    side. Subsampling with a stride of 3 gives a 10 Hz decision rate.

For QA, `meta/episodes.jsonl` records the encoded video frame count per
episode, so you can spot cameras that dropped significantly below the row
count.

## meta/ files

| File | Contents |
|---|---|
| `info.json` | Dataset header: `codebase_version: "v2.0"`, `robot_type`, `fps`, the `data_path` / `video_path` templates, `chunks_size`, totals (episodes / frames / tasks), a `splits` entry (`train: "0:N"`), and the `features` map with dtype and shape for every column and video stream. |
| `modality.json` | GR00T-style modality map — see below. |
| `episodes.jsonl` | One JSON line per episode: `episode_index`, `length` (row count), `tasks` (list containing the task string), `task_index`; when cameras are present, also `video_frame_length` (minimum frame count across cameras) and per-camera `video_frame_lengths`. |
| `tasks.jsonl` | One JSON line per distinct task string: `{"task_index", "task"}`. |
| `stats.json` | Per-column `{mean, std, min, max, q01, q99}` over all saved episodes, for every feature column plus `timestamp`. Recomputed after every saved episode. |
| `collection_meta.json` / `collection_sessions.jsonl` | Provenance records — see [Provenance](#provenance). |

### modality.json

`modality.json` describes how named sub-modalities slice into the parquet
columns, in the style GR00T-family trainers expect. Each entry gives a
`(start, end)` slice, the `original_key` (parquet column), the dtype, and
flags such as `rotation_type` and `absolute`.

??? info "Excerpt: state and action entries"

    ```json
    {
      "state": {
        "arm_joint": {"start": 0, "end": 6,
                      "original_key": "observation.arm_joint",
                      "dtype": "float32", "absolute": true},
        "eef_pos":   {"start": 0, "end": 3,
                      "original_key": "observation.arm_eef",
                      "dtype": "float32", "absolute": true},
        "eef_rot":   {"start": 3, "end": 7,
                      "original_key": "observation.arm_eef",
                      "rotation_type": "quat_wxyz",
                      "dtype": "float32", "absolute": true},
        "gripper":   {"start": 0, "end": 1,
                      "original_key": "observation.gripper",
                      "dtype": "float32", "absolute": true}
      },
      "action": {
        "eef_pos": {"start": 0, "end": 3, "original_key": "action.arm_eef",
                    "dtype": "float32", "absolute": true},
        "eef_rot": {"start": 3, "end": 7, "original_key": "action.arm_eef",
                    "rotation_type": "quat_wxyz",
                    "dtype": "float32", "absolute": true},
        "gripper": {"start": 0, "end": 1, "original_key": "action.gripper",
                    "dtype": "float32", "absolute": true}
      }
    }
    ```

The `video` section maps each camera name to its
`observation.images.<cam>` key, and the `annotation` section declares
`task_index`.

## Provenance

A policy trained on this data is implicitly bound to the controller that
produced it — the same EEF targets yield different contact behavior under
different backends and gains (see [Control modes](control-modes.md)). The
writer therefore records how each session was collected:

- `meta/collection_sessions.jsonl` — append-only, one JSON line per
  collection session.
- `meta/collection_meta.json` — the most recent session record, for quick
  inspection.

Each record captures the input device, the arm model and firmware, the
control backend and its gains, and the camera serials.

### Binding keys and the change warning

When a new session is appended, the writer compares it against the previous
session on these binding-critical keys:

| Key | Why it binds the policy |
|---|---|
| `control.backend` | Impedance vs. position control changes contact dynamics |
| `control.impedance.kp` | Stiffness — tracking tightness and contact force |
| `control.impedance.kd` | Damping |
| `control.impedance.t_ff` | Feed-forward torque |
| `control.impedance.gravity_ff` | Gravity feed-forward |
| `arm.firmware` | Firmware version paired with the arm |
| `recording.fps` | The data rate everything downstream assumes |

!!! warning "Do not mix controllers in one dataset"
    If any of these values changed since the previous session, the writer
    prints a prominent warning listing each differing key and its old and new
    value, and suggests starting a fresh `--root` instead. Mixing sessions
    recorded under different control parameters produces a dataset whose
    action-to-motion mapping is inconsistent.

## Finalization: piper-finalize

Some trainers consume **relative** or **delta** action representations
instead of absolute poses. `piper-finalize` derives the normalization
statistics for those representations without modifying the parquet data:

```bash
piper-finalize --root ~/piper_datasets/<task> [--horizon 30]
```

It writes two files into `meta/`, computed over a future window of
`--horizon` steps (default 30) with per-episode grouping:

- `relative_stats.json` — each window of future actions expressed relative to
  the state at the window start. Gripper-like columns use plain subtraction
  (`action[i:i+H] - state[i]`); EEF poses use the proper SE(3) relative
  transform `T_state(i)^-1 @ T_action(i+k)`, with the rotation converted back
  to a wxyz quaternion.
- `delta_stats.json` — frame-to-frame differences chained along the horizon:
  the first step relative to the current state, each subsequent step relative
  to the previous action.

Windows that run past the end of an episode are padded by repeating the last
action. Both files use the same `{mean, std, min, max, q01, q99}` shape as
`stats.json`, and both also include standard per-row statistics for every
`observation.*` column. See the [CLI reference](cli.md) for the command
summary.

## extras/

The `extras/` directory holds non-standard debug files that trainers ignore.
When raw Quest capture is enabled (the writer's `save_raw_quest` option), each
episode gets an `episode_XXXXXX.quest.jsonl` file containing the raw Quest
stream messages, one JSON object per line. This lets you replay or inspect
exactly what the headset sent when debugging input-mapping issues, without
touching the training-facing data.

## Video encoding

Videos are encoded as mp4 with libx264 by default (`yuv420p` pixel format,
CRF 23, preset `veryfast`), streaming during recording with explicit
millisecond PTS as described in [Timing and alignment](#timing-and-alignment).
