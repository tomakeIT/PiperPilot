# CLI reference

Every workflow in PiperPilot is driven from the terminal, in two
equivalent ways: short `make` targets (what the install and quickstart pages
use) and the `piper-*` console commands they wrap. This page lists both — the
make cheat sheet first, then the full flag tables for each console command.

## Make target cheat sheet

Run `make help` at the repo root to print a short version of this list.

### One-time setup

| Target | What it does |
|---|---|
| `make toolchain` | Install the Android SDK/NDK/JDK17/Gradle toolchain (~2.5 GB) — needed only to build the Quest APK |
| `make env` | Create the `piper_teleop` conda env, install dependencies and pyAgxArm |
| `make can` | Bring up `can0` at 1 Mbps (uses `sudo`) |
| `make spacemouse` | Run the SpaceMouse setup script (`install/04_spacemouse_setup.sh`) |
| `make apk` | Build the Quest APK |

### Daily use

| Target | Wraps | What it does |
|---|---|---|
| `make connect` | `scripts/quest_connect.sh -p` | Install/start the Quest APK and set up the `adb forward` wired link (`make install-apk` is an alias) |
| `make home` | `piper-home` | Unfold the arm from the folded rest pose into the working pose |
| `make view` | `piper-view` | Live 3D visualization of Quest controller poses (no arm needed) |
| `make collect TASK="..."` | `piper-collect --task "..."` | THE app: teleop + recording (recording starts only when you press A / space) |
| `make collect-sm TASK="..."` | `piper-collect --input spacemouse --task "..."` | Same, with the SpaceMouse |
| `make collect-sim` | `piper-collect --sim --task "sim test task"` | End-to-end dry run, no hardware |
| `make test` | `pytest tests/ -v` | Run the unit/integration tests |

The collect targets use `default.yaml` alone; add an overlay per run with
`make collect CONFIG=first_run.yaml`. There is no separate teleop entry —
running collect without pressing record IS teleop, and nothing is written to
disk until an episode is saved.

## Make targets are thin wrappers

Each daily-use target just runs the corresponding Python module with the
`piper_teleop` env's interpreter, e.g. `make collect` executes
`python -m piper_teleop.apps.collect_data`. You can always activate the env
and call the console command directly instead — every flag below is available
that way, while the make targets expose only the fixed combinations shown
above:

```bash
conda activate piper_teleop
piper-collect --input spacemouse --viz
```

!!! note "PYTHONPATH is cleared by the wrappers"
    The Makefile invokes Python with `PYTHONPATH=` (empty) so a sourced ROS
    environment cannot leak its Python modules or pytest plugins into the
    `piper_teleop` env. If you run the console commands directly from a shell
    that has sourced ROS, keep that in mind.

## Shared flags (`add_common_args`)

`piper-collect` and the other teleop-stack tools (`piper-infer`,
`piper-track-test`) share one flag set, defined once in
`piper_teleop/apps/teleop_arm.py`:

| Flag | Default | Meaning |
|---|---|---|
| `--config FILE` | none | YAML config overlay applied on top of the base config (see [Configuration](configuration.md)) |
| `--input {quest,spacemouse}` | from config (`quest`) | Teleop input device |
| `--sim` | off | Fake arm backend — full pipeline, no hardware |
| `--impedance` | off | Force MIT impedance control (compliant; already the default backend) |
| `--position` | off | Force firmware position control via `move_p` (tightest tracking) |
| `--viz` | off | Enable rerun visualization |
| `--serve` | off | Rerun web viewer instead of a spawned window (headless hosts) |
| `--hand {left,right}` | from config | Quest control hand |

`--sim`, `--position`, and `--impedance` select the arm backend (`fake`,
`piper`, `piper_mit`); `--sim` wins if combined. See
[Control modes](control-modes.md) for what the backends mean in practice.

!!! tip "Dry-run anything"
    Every command that touches the arm accepts `--sim`, so you can rehearse the
    whole pipeline — teleop, recording, replay, inference — with no robot and
    no CAN bus attached.

## piper-view

Visualize live Quest controller poses in rerun — the first thing to run after
`make connect`, before any arm is involved.

```bash
piper-view [--config FILE] [--host HOST] [--port PORT] [--serve]
```

| Flag | Default | Meaning |
|---|---|---|
| `--config FILE` | none | YAML config overlay |
| `--host HOST` | from config | Pose stream host |
| `--port PORT` | from config | Pose stream port |
| `--serve` | off | Rerun web viewer (headless host) |

It prints the stream rate and session state every 2 s and draws both
controllers plus the control hand mapped into robot coordinates.

```bash
make connect        # once: install/start APK + adb forward
piper-view          # then watch the controllers move in rerun
```

## piper-collect

Full data collection: teleop + cameras + LeRobot recording. Takes the
[shared flags](#shared-flags-add_common_args) plus:

```bash
piper-collect --task "..." [--root DIR] [--no-cameras] [shared flags]
```

| Flag | Default | Meaning |
|---|---|---|
| `--task TEXT` | from config | Language instruction stored with every episode |
| `--root DIR` | `<recording.root>/<task_with_underscores>_<YYYYMMDD_HHMMSS>` | Dataset root directory (task slugified, truncated to 48 chars, plus a start timestamp; pass an explicit `--root` to append to an existing dataset) |
| `--no-cameras` | off | Record without cameras |

During collection: **A/X** = start / stop-and-save episode, **B/Y** = discard,
hold the non-control hand's **Y/B** for 1 s = home. Keyboard: ++space++ =
start/stop, ++d++ = discard, ++h++ = home, ++q++ = quit. Full walkthrough in
the [data collection guide](../guide/data-collection.md).

Examples:

```bash
piper-collect --task "pick up the red block" --root ~/piper_datasets/pick_red
piper-collect --sim --task test      # end-to-end dry run, no hardware
```

Session provenance (input device, arm description, control backend and gains,
camera serials) is written into the dataset metadata automatically — see
[Dataset format](dataset-format.md).

## piper-home

Unfold the arm from the factory folded rest pose into the working pose. The
folded pose is slightly outside the joint operating range, so Cartesian moves
from rest are rejected by the firmware — run this once after every power-on
(or whenever teleop tells you to).

```bash
piper-home [--config FILE] [--joints J1 J2 J3 J4 J5 J6] [--zero] [--speed PCT] [--sim]
```

| Flag | Default | Meaning |
|---|---|---|
| `--config FILE` | none | YAML config overlay |
| `--joints J1..J6` | config `home_joints` or built-in default | Target joint angles (rad) |
| `--zero` | off | Go to the all-zero SDK rest pose (parking; teleop cannot start from there) |
| `--speed PCT` | 30 | Speed percent for the move |
| `--sim` | off | Fake arm backend |

Homing is always a firmware-planned joint move (position backend), regardless
of your configured teleop backend. The tool waits up to 25 s for the joints to
converge, then reports whether the end effector landed inside the teleop
workspace box — teleop can only engage from inside it.

```bash
piper-home                                       # default unfold, 30% speed
piper-home --speed 20 --joints 0 0.85 -0.75 0 0.6 0
```

!!! warning "Stand clear"
    The arm swings through free space during homing. Keep the workspace clear
    before you run it.

## piper-track-test

Tracking-error benchmark: commands a small, slow circle around the current
pose and measures commanded-vs-measured EEF error. This is the tool behind the
numbers in [Benchmarks](benchmarks.md).

```bash
piper-track-test [--config FILE] [--impedance | --position | --sim]
                 [--amp M] [--period S] [--duration S] [--rate HZ]
```

| Flag | Default | Meaning |
|---|---|---|
| `--config FILE` | none | YAML config overlay |
| `--impedance` | off | MIT impedance mode (already the default backend) |
| `--position` | off | `move_p` position mode |
| `--sim` | off | Fake arm backend |
| `--amp M` | 0.03 | Circle radius in meters (clamped to 0.05) |
| `--period S` | 5.0 | Seconds per revolution |
| `--duration S` | 12.0 | Length of the moving phase (s) |
| `--rate HZ` | 100.0 | Command rate |

The amplitude ramps in over 2 s and the trajectory is clamped to the teleop
workspace box. The report includes static hold error, dynamic RMS/max error,
per-axis RMS, and an estimated command-to-measurement latency obtained by
cross-correlation.

```bash
piper-track-test --position                      # benchmark position mode
piper-track-test --amp 0.03 --period 5 --duration 12
```

## piper-replay

Replay a recorded episode's actions on the arm (open-loop) and report how well
the replayed trajectory reproduces the recorded one. See the
[replay guide](../guide/replay.md) for context.

```bash
piper-replay --root DIR [--episode N] [--speed X] [--config FILE] [--sim] [--position]
             [--arm-side left|right] [--relative|--absolute] [--gripper-max X]
```

| Flag | Default | Meaning |
|---|---|---|
| `--root DIR` | required | Dataset root directory |
| `--episode N` | 0 | Episode index to replay |
| `--speed X` | 1.0 | Time scale (0.5 = half speed) |
| `--config FILE` | none | YAML config overlay |
| `--sim` | off | Fake arm — parse/timing dry run |
| `--position` | off | Force the position backend |
| `--arm-side` | `right` | Dual-arm modality datasets: arm channel to replay |
| `--relative` / `--absolute` | auto | Anchoring: native datasets replay absolute, modality (external) datasets replay relative to the current pose |
| `--gripper-max X` | episode max | Raw gripper value meaning fully open (external datasets) |
| `--joints` | off | Replay the recorded joint trajectory (native datasets, or modality sets with a `<side>_arm_joint` channel; warns when recorded joints exceed this arm's limits) |
| `--viz` | off | Live rerun 3D visualization during playback |
| `--anchor X,Y,Z[,R,P,Y]` | current pose | Fixed anchor pose for relative replay (m, RPY deg) — deterministic across runs |
| `--scale X` | 1.0 | Scale translation deltas in relative replay |

Both dataset schemas are auto-detected: native (`action.arm_eef` +
`action.gripper`) and GR00T/modality (16-dim `action` + `meta/modality.json`,
e.g. RoboMIND `agilex_cobot_magic`). See the
[replay guide](../guide/replay.md#external-and-dual-arm-datasets).

Safety behavior: the arm approaches the episode's first action pose linearly
over 3 s before playback, per-tick step clamps come from the arm backend, and
any arm fault aborts the replay. Afterwards it prints RMS/max position error
versus the recorded observations and versus the commanded actions.

```bash
piper-replay --root ~/piper_datasets/pick_red --episode 0 --speed 0.7
piper-replay --root ~/piper_datasets/pick_red --sim      # no hardware
```

## piper-infer

Policy-inference runtime: builds observations (latest camera frames + arm
state), queries a policy, and executes the returned action chunks at the
dataset rate. See the [inference guide](../guide/inference.md).

```bash
piper-infer --policy SPEC [--task TEXT] [--config FILE] [--sim] [--position]
            [--no-cameras] [--rate HZ] [--exec-horizon N] [--horizon N]
            [--max-seconds S]
```

| Flag | Default | Meaning |
|---|---|---|
| `--policy SPEC` | required | `hold` \| `replay:<root>[:<ep>]` \| `http://host:port` |
| `--task TEXT` | from config | Language instruction passed to the policy |
| `--config FILE` | none | YAML config overlay |
| `--sim` | off | Fake arm backend |
| `--position` | off | Force the position backend |
| `--no-cameras` | off | Run without cameras |
| `--rate HZ` | 30.0 | Execution rate (the dataset fps) |
| `--exec-horizon N` | 15 | Steps executed per policy query (15 @ 30 Hz = 0.5 s) |
| `--horizon N` | 30 | Steps requested per query |
| `--max-seconds S` | 120.0 | Run length limit |

Actions are absolute EEF poses + gripper width, executed under the same safety
envelope as teleop: positions are clipped to the workspace box, backend step
clamps apply, and any arm fault aborts. With a `replay:` policy the
exec-horizon is forced equal to the horizon so chunks are consumed fully.

```bash
piper-infer --policy hold                            # loop smoke test
piper-infer --policy replay:~/piper_datasets/pick_red:0
piper-infer --policy http://gpu-host:8901 --task "pick up the red block"
piper-infer --policy hold --sim --no-cameras         # no hardware at all
```

## piper-finalize

Generate `relative_stats.json` and `delta_stats.json` in a dataset's `meta/`
directory, for trainers that consume relative or delta action representations
(statistics over a future-action window, grouped per episode).

```bash
piper-finalize --root DIR [--horizon N]
```

| Flag | Default | Meaning |
|---|---|---|
| `--root DIR` | required | Dataset root (must contain `meta/info.json`) |
| `--horizon N` | 30 | Future window length used for the relative/delta stats |

Each stats entry contains `mean`, `std`, `min`, `max`, `q01`, and `q99`; the
files also include standard per-row stats for every `observation.*` column.
The exact relative/delta semantics are described in
[Dataset format](dataset-format.md).

```bash
piper-finalize --root ~/piper_datasets/pick_red
```

## See also

- [Installation](../getting-started/installation.md) — what the one-time make
  targets set up
- [Quickstart](../getting-started/quickstart.md) — the daily targets in order
- [Configuration](configuration.md) — what `--config` overlays can change
- [Troubleshooting](../troubleshooting.md) — when a command refuses to start
