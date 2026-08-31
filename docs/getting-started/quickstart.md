# Quick start (no hardware)

This page walks you through the whole PiperTeleopTools pipeline in about ten
minutes — with **no robot arm, no headset, and no cameras**. Everything runs
against a simulated ("fake") arm and fake cameras on your Linux machine.

## What you'll have at the end

A real **LeRobot v2.0 dataset** on disk — parquet proprioception/action rows at
30 Hz, mp4 camera videos, and full metadata — recorded from a simulated
session. It is byte-for-byte the same format the real-hardware pipeline
produces, so everything you learn here transfers directly.

## Prerequisites

- A Linux host (Ubuntu 22.04 is the verified platform).
- `git`.
- Conda via miniforge (the install script defaults to `~/miniforge3/bin/conda`;
  override `CONDA_BIN` when needed).

That's it. No CAN adapter, no Quest, no SpaceMouse, no RealSense.

## Clone and create the environment

`make env` also installs the AgileX arm SDK (pyAgxArm) as an editable local
checkout, so clone that first:

```bash
git clone https://github.com/agilexrobotics/pyAgxArm ~/pyAgxArm
git clone <repository-url>
cd PiperTeleopTools
make env
conda activate piper_teleop
```

`make env` creates the `piper_teleop` conda env, installs the dependencies,
pyAgxArm, and this package (registering the `piper-*` console commands), and
ends with an import smoke test that prints `all imports OK` — the step-by-step
breakdown is in [Installation](installation.md).

Most of the time is package download, so duration depends on your connection.
A `pyrealsense2 import issue` line in the smoke test is harmless here — you
have no cameras attached.

!!! note "Non-default paths"
    The script honors `PYAGXARM_PATH` (default `~/pyAgxArm`) and `CONDA_BIN`
    (default `~/miniforge3/bin/conda`) if your checkout or conda live
    elsewhere. It exits with an error if it can't find pyAgxArm.

## Start a simulated collection session

```bash
make collect-sim
```

This runs the full data-collection app (`piper-collect`) with `--sim` and the
task string `"sim test task"`. Instead of real hardware it starts:

- a **fake arm** backend (no CAN bus needed),
- **fake cameras** in place of the RealSense streams,
- **keyboard controls** in the terminal.

You should see startup lines like:

```text
[collect] dataset root: /home/<you>/piper_datasets/sim_test_task_20260724_161500
[collect] task: 'sim test task'
[collect] A/X=start/stop  B/Y=discard  |  hold other hand's Y/B 1s=home  |  keys: space=start/stop d=discard h=home q=quit
```

followed by a single status line that refreshes in place:

```text
[collect] input   0.0Hz | idle    | grip    0mm | 0 eps saved
```

The fields are: input-device rate (0 Hz — nothing is connected), clutch state,
gripper width, and the episode counter. While an episode is recording, the
last field switches to something like `● REC ep0 156 frames`.

## Record an episode

The keyboard controls during collection:

| Key | Action |
| --- | --- |
| ++space++ | Start / stop-and-save the episode |
| ++d++ | Discard the current episode |
| ++h++ | Home the arm |
| ++q++ | Quit |

1. Press ++space++ to start recording. The status line switches to `● REC`.
2. Wait about 10 seconds. At 30 Hz that's roughly 300 frames.
3. Press ++space++ again to stop and save the episode.
4. Press ++q++ to quit. The app prints a summary as it shuts down:

```text
[collect] shutting down
[collect] dataset: /home/<you>/piper_datasets/sim_test_task_20260724_161500 — 1 episodes, 312 frames
```

!!! note "The arm doesn't move — that's expected"
    With no Quest or SpaceMouse connected, the fake arm simply holds still.
    The point of this exercise is the pipeline — controllers, recorder,
    cameras, dataset writer — not motion.

!!! tip "Want something to look at?"
    `piper-collect --sim --viz` runs the same app against the fake
    arm with a **rerun** 3D visualization window, so you can see the simulated
    arm rendered live.

## Inspect the result

The dataset lands under `~/piper_datasets/sim_test_task_<timestamp>` (the
default recording root plus the task name plus a start timestamp, so repeated
runs never collide). Its layout:

```text
~/piper_datasets/sim_test_task_<timestamp>/
├── meta/       info.json, modality.json, episodes.jsonl, tasks.jsonl,
│               stats.json, collection_meta.json, collection_sessions.jsonl
├── data/       chunk-000/episode_000000.parquet          # 30 Hz rows
└── videos/     chunk-000/observation.images.<cam>/episode_000000.mp4
```

`meta/collection_meta.json` records the session's provenance — input device,
arm backend, control parameters — even for this simulated run. The parquet
columns (joint angles, EEF pose, gripper, absolute actions) and all
conventions are documented in the
[Dataset format reference](../reference/dataset-format.md).

## Next steps

- [Installation](installation.md) — set up the real arm, CAN bus, and your
  input device.
- [Your first teleop session](first-teleop.md) — drive the real arm for the
  first time.
