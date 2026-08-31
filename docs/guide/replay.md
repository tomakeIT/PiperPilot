# Replaying episodes

`piper-replay` plays a recorded episode back on the real arm, open-loop, and
then prints a report of how closely the arm reproduced the original
trajectory. It reads the episode straight from the dataset's parquet files —
no teleop hardware (Quest or SpaceMouse) is needed.

## Why replay

Replay closes the loop on data collection in three ways:

- **Sanity-check a dataset physically.** Watching the arm re-execute a
  demonstration is the fastest way to confirm that what you recorded is what
  actually happened — correct motion, correct gripper timing, no glitches.
- **Verify the arm can reproduce demonstrations.** The recorded actions are
  absolute EEF targets (see
  [Dataset format](../reference/dataset-format.md)). If the arm cannot track
  them accurately in open loop, a policy trained on them will inherit that
  error floor.
- **Regression-test control changes.** After changing gains, firmware, or the
  control backend, replay a known-good episode and compare the report numbers
  against earlier runs.

## Run a replay

!!! note "Home the arm first"
    After every arm power-on, run `make home` to unfold the arm from the
    factory folded pose into the working pose. Starting a replay from the
    folded pose fails with `TARGET_POS_EXCEEDS_LIMIT(4)`.

```bash
piper-replay --root ~/piper_datasets/<task> --episode 0 --speed 0.7
```

### Flags

| Flag | Default | Meaning |
|---|---|---|
| `--root <ds>` | required | Dataset root directory (the folder containing `data/` and `meta/`) |
| `--episode N` | `0` | Episode index to replay |
| `--speed X` | `1.0` | Time scale; `0.5` = half speed |
| `--config FILE` | none | Config overlay applied on top of `default.yaml` (see [Configuration](../reference/configuration.md)) |
| `--sim` | off | Dry run on the `fake` backend: parses the episode and exercises the timing loop with no hardware |
| `--position` | off | Force the `piper` firmware position backend instead of the default MIT impedance backend |
| `--arm-side {left,right}` | `right` | Dual-arm (modality-schema) datasets: which arm's channel to replay |
| `--relative` / `--absolute` | auto | Override the replay anchoring (see below) |
| `--gripper-max X` | episode max | Raw gripper value meaning "fully open", for datasets whose gripper is not in meters |
| `--joints` | off | Replay the recorded **joint** trajectory instead of EEF actions (native datasets, or modality sets exposing `<side>_arm_joint`; a pre-flight check warns if recorded joints exceed this arm's limits) |
| `--viz` | off | Live rerun 3D view of the arm and target during playback |
| `--anchor X,Y,Z[,R,P,Y]` | arm's current pose | Anchor relative replay to this **fixed** pose (m, RPY in deg, base frame) — deterministic: same trajectory every run |
| `--scale X` | `1.0` | Scale translation deltas in relative replay (shrink motions recorded on a larger robot to fit the workspace) |

## EEF vs joint space

By default the recorded `action.arm_eef` targets are replayed through the
same EEF command path teleop uses (host IK in impedance mode, firmware IK in
`--position` mode). With `--joints`, the recorded `observation.arm_joint`
trajectory is streamed directly — impedance mode streams MIT joint targets,
position mode streams `move_j` — which removes IK from the loop and
reproduces the exact joint-space motion, including null-space posture. The
report then shows joint-space error in degrees. Modality datasets (e.g.
RoboMIND) carry no joint channel, so `--joints` is refused for them.

## External and dual-arm datasets

`piper-replay` also plays back GR00T/modality-style LeRobot datasets that were
recorded on *other* robots — for example the RoboMIND `agilex_cobot_magic`
dual-arm sets, whose `action` is a 16-dim vector described by
`meta/modality.json`. The schema is auto-detected; pick the arm with
`--arm-side`:

```bash
piper-replay --root data/50_pourtea --episode 38 --arm-side right --sim
```

Because such data lives in a different robot's frame convention, it is
replayed **relative** by default: the recorded motion — every pose expressed
as a delta from the episode's first action pose — is applied on top of this
arm's *current* pose, and the approach phase is a no-op. Home the arm first
so the motion unfolds from the middle of the workspace. Raw-unit gripper
values are rescaled linearly to the configured `gripper.max_width`
(override the full-open raw value with `--gripper-max`).

For a deterministic playback that does not depend on where the arm happens
to be, pass `--anchor` (a fixed base-frame pose the episode's first action is
pinned to) and, if the source robot's motion span exceeds this arm's
workspace, `--scale` to shrink the translations:

```bash
piper-replay --root data/50_pourtea --episode 53 --arm-side right \
             --anchor "0.40,0,0.30" --scale 0.7
```

Target positions are always clamped to the configured workspace box; if any
targets get clamped, the run prints how many, so you know where the
reproduction was distorted. If the requested episode's parquet is missing
(sampled subsets ship only a few episodes), the error lists the episode
indices actually present.

!!! warning "Cross-robot replay reproduces motion, not the task"
    The trajectory is re-anchored to your arm and clipped to your workspace —
    useful for inspecting data quality and exercising the control stack, but
    scene geometry, object positions, and the second arm's contribution are
    not reproduced.

!!! tip "Dry-run first"
    `piper-replay --root <ds> --sim` checks that the episode loads and the
    timing loop runs before you involve the real arm.

## What happens, step by step

1. **Load.** The tool reads
   `data/chunk-XXX/episode_XXXXXX.parquet` from the dataset root and extracts
   the absolute `action.arm_eef` targets, `action.gripper` openings,
   `observation.arm_eef` measurements, and timestamps. It prints a summary:
   step count, recorded duration, playback duration at your `--speed`, and
   the arm backend in use.
2. **Connect.** It brings up the arm backend (MIT impedance by default),
   waits 2 s for mode entry to settle, and verifies EEF feedback is valid.
3. **Approach.** It interpolates linearly from the arm's current measured
   pose to the episode's *first action pose* over 3 s, printing how far away
   that start pose is in millimetres — with a `stand clear!` warning.
4. **Playback.** It steps through the actions paced by the *recorded
   timestamps* (scaled by `--speed`), which makes playback robust to tick
   jitter in the data. Each step commands the absolute EEF pose and gripper
   opening, and samples the arm's measured EEF position for the report. Any
   arm fault aborts playback immediately and stops the arm.
5. **Report.** After the last step it prints the reproduction-error report
   (below) and stops the arm.

## Safety notes

!!! danger "The arm follows the recording blindly"
    Replay is open-loop: there is no perception and no reaction to obstacles.
    Clear the workspace of everything that was not present during recording —
    including your hands — before starting. The approach phase alone can
    sweep a large arc if the arm starts far from the episode's first pose.

- Replay commands the same arm backends as teleop, so the per-tick step
  clamps come from the backend and any arm fault aborts the run — the same
  safety envelope described in [Control modes](../reference/control-modes.md).
- Start at reduced speed (for example `--speed 0.7` or `--speed 0.5`) the
  first time you replay a dataset, and only move to `1.0` once the motion
  looks right.
- Keep in mind that `--position` makes the arm stiff: the firmware position
  servo tracks tightly but is not compliant on contact.

## Reading the report

At the end of playback you get:

```text
=============== REPLAY REPORT ===============
vs RECORDED observation : RMS  ...  mm   max  ...  mm
vs COMMANDED action     : RMS  ...  mm
=============================================
```

| Line | What it measures |
|---|---|
| `vs RECORDED observation` | Distance between the EEF positions measured *during replay* and the positions measured *during the original recording*, at matching steps — RMS and worst-case max. This is the headline "did the arm do the same thing again" number. |
| `vs COMMANDED action` | Distance between the replayed measured positions and the commanded action targets — pure tracking error of the current controller, independent of how well the recording itself tracked. |

### What is normal

Expect the numbers to sit in the same ballpark as the measured tracking
benchmarks for the control mode you are using (Piper-X, firmware S-V1.8-9;
see [Benchmarks](../reference/benchmarks.md)):

- **Position mode** (`--position`): about 1.3 mm dynamic RMS.
- **Impedance mode** (default, shipped soft gains): about 14 mm dynamic RMS,
  with an 11 mm static hold error.

So an impedance-mode replay reporting an RMS in the low tens of millimetres
is normal, not a bug.

### What a large error suggests

If the report is far above those ballparks:

- **The controller changed since recording.** Different impedance gains, a
  different firmware, or a different backend than the one used during
  collection all shift the error. The dataset's
  `meta/collection_meta.json` records the backend and gains used at
  recording time (see [Dataset format](../reference/dataset-format.md)), so
  compare it against your current config.
- **You are replaying in a different mode than recorded.** For example,
  replaying an impedance-mode recording with `--position` (or the reverse)
  changes the tracking behaviour — see
  [Control modes](../reference/control-modes.md) for how the two modes
  differ.

Once a dataset replays cleanly, the next step is closing the loop with a
policy — see [Running inference](inference.md).
