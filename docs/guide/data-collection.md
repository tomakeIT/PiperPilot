# Collecting datasets

`piper-collect` is the day-to-day recording tool: everything works exactly as
in plain teleop, plus a recorder and a dataset writer running alongside. Three
pieces cooperate:

- **Teleop** — the same controller stack you already know from
  [your first teleop session](../getting-started/first-teleop.md). Nothing
  about clutching, homing, or the safety envelope changes.
- **Recorder** — samples the arm state and the commanded action at 30 Hz and
  muxes in the newest frame from each camera (streams are timestamp-aligned on
  a shared clock).
- **Writer** — turns those samples into a LeRobot v2.0 dataset on disk:
  parquet rows, mp4 videos, and metadata.

The unit of work is the **episode**: one demonstration from start to
stop-and-save (or discard). Teleop and collection share this single entry —
running it without ever pressing record is plain teleop, and **nothing is
written to disk until the first episode is saved**, so launching it commits
you to nothing. You stay in one long-running session and bank
episodes one after another.

## Start a collection run

```bash
make collect TASK="pick up the red block"
# equivalent:
piper-collect --task "pick up the red block"
# choose the dataset location explicitly:
piper-collect --task "pick up the red block" --root ~/piper_datasets/pick_red
```

If you omit `--root`, the dataset lands in
`<recording.root>/<task>_<timestamp>` with spaces replaced by underscores
(truncated to 48 characters) — e.g. the task above becomes
`.../pick_up_the_red_block_20260724_155857`. The timestamp keeps repeated
runs from colliding; to **append** to an existing dataset, pass its path as
`--root` explicitly. `recording.root` comes from the
[configuration](../reference/configuration.md).

Variants:

| Command | Use |
|---|---|
| `make collect TASK="..."` | Quest teleop + recording |
| `make collect-sm TASK="..."` | SpaceMouse input (`--input spacemouse`) |
| `make collect-sim` | End-to-end dry run without hardware (`--sim`) |

!!! note "Home first"
    After every arm power-on, run `make home` before collecting — controllers
    refuse to engage while the arm is outside the workspace box.

## The episode loop

Drive the arm as usual; the episode lifecycle sits on a few extra buttons.

=== "Quest"

    | Input (control hand) | Action |
    |---|---|
    | **A / X** | Start episode / stop and save |
    | **B / Y** | Discard current episode |
    | Non-control hand **Y/B** held 1 s | Home (clutch released) |
    | Non-control hand **X/A** held 1 s | Discard current episode, or delete the **last saved** episode when not recording |

    Haptic confirmations tell you what happened without looking away: a short
    buzz on start, a longer buzz on save, and three short buzzes on discard or
    delete (a single weak buzz means there was nothing to delete).

    The in-headset HUD panel mirrors the recorder state:

    - **REC** with the episode index, a running `mm:ss` timer, and a blinking
      dot while an episode is being recorded;
    - **READY** with the count of episodes saved so far when idle;
    - **HOMING ...** while the arm returns to the working pose.

    While recording, the passthrough view also gets a light **red tint**;
    idle is pure passthrough. You always know whether the take is live.
    See the [Quest guide](quest.md) for the full button map.

=== "SpaceMouse"

    | Input | Action |
    |---|---|
    | Right button, short press | Start episode / stop and save |
    | Right button, held 1 s | Home |
    | Both buttons | Discard current episode |
    | Left button | Gripper toggle (as in teleop) |

    See the [SpaceMouse guide](spacemouse.md) for the full mapping.

=== "Keyboard"

    The terminal running `piper-collect` also accepts keys:

    | Key | Action |
    |---|---|
    | ++space++ | Start episode / stop and save |
    | ++d++ | Discard current episode |
    | ++x++ | Discard current episode, or delete the last saved episode |
    | ++h++ | Home |
    | ++q++ | Quit |

Saving writes the episode's parquet and video files immediately and updates
the dataset metadata. Discarding deletes the episode's video files and keeps
nothing. Episodes with zero frames are dropped automatically.

!!! note "Deleting after the fact is session-scoped"
    The delete gesture removes the most recently saved episode — files and
    metadata — and can be repeated to walk back several bad takes. It only
    reaches episodes saved in the **current session**: when appending to an
    existing dataset, earlier sessions' episodes are protected.

## The web dashboard

While `piper-collect` runs, it serves a read-only status page at
**`http://127.0.0.1:8780`** (the URL is printed at startup). It mirrors the
in-headset HUD for anyone at the computer:

- a large color-coded state banner — red **REC** with episode index and a
  running timer, green **READY** with the episodes-saved count, amber
  **HOMING**;
- episodes saved, total frames, clutch state, and input rate;
- a small live view of every recording camera;
- the task string, dataset root, and any recorder error.

### Camera calibrate

The **camera calibrate** row above the live views re-aligns a moved camera
against an existing dataset. Pick a dataset from the dropdown (any LeRobot
dataset under `recording.root`, newest first) and each live view gets that
dataset's reference frame — episode 0, frame 0 of the matching camera —
layered on top, in one of two blend modes:

- **overlay** — the reference at adjustable opacity (onion skin), for coarse
  placement: nudge the camera until the table edges and fixtures line up;
- **difference** — a per-pixel `|live − reference|` blend: everything that
  matches goes black, misalignment shows up as bright ghost edges, so fine-tune
  the mount until the static scene is as dark as possible (the arm and any
  moved objects will stay visible — judge by the background, not by them).

Configure it under the `monitor:` block — port, or `host: 0.0.0.0` to watch
from another device on the LAN (unauthenticated; this exposes the camera
views). Set `enabled: false` to turn it off. See
[Configuration](../reference/configuration.md#monitor).

## Appending across sessions

Re-running `piper-collect` with the same `--root` **appends**: the writer
detects the existing dataset, prints how many episodes it is resuming from,
and continues numbering where it left off. This is the normal way to grow a
dataset over several days.

Every session is recorded in the dataset's provenance files:
`meta/collection_meta.json` holds the latest session record, and
`meta/collection_sessions.jsonl` appends one record per session — tool
version, input device, arm model and firmware, control backend and gains,
camera serials, and the recording settings.

If a *binding-critical* parameter differs from the previous session of the
same dataset — the control backend, the impedance gains (`kp`, `kd`, `t_ff`,
`gravity_ff`), the arm firmware, or the recording fps — the writer prints a
prominent warning:

```text
WARNING: control parameters CHANGED vs the previous
session of this dataset — a policy is bound to its
controller. Consider a fresh --root instead:
  control.impedance.kp: [...] -> [...]
```

!!! warning "Why this matters"
    The actions in the dataset are absolute EEF targets, but how the arm
    *responds* to those targets depends on the controller — its gains,
    latency, and compliance. A policy trained on the data implicitly learns
    that response. Mixing sessions recorded with different controller
    behavior in one dataset gives the policy inconsistent dynamics to imitate.
    When in doubt, start a fresh `--root`.

## Recording tips

- Home between episodes if the arm has drifted from a comfortable start pose
  (hold the non-control hand's **Y/B** for 1 s, or press ++h++).
- Keep demonstrations smooth — the recorded actions are your commanded
  targets, jitter included.
- Discard bad takes immediately (**B/Y** or ++d++); it is cheaper than
  cleaning the dataset later.
- Prefer several short, focused episodes over one long meandering one.

## After collecting: finalize

```bash
piper-finalize --root ~/piper_datasets/pick_up_the_red_block
```

`piper-finalize` writes `meta/relative_stats.json` and `meta/delta_stats.json`
— normalization statistics for **relative** and **delta** action
representations, computed over a future action window per frame (default
horizon 30, `--horizon` to change). You only need this if your trainer
consumes relative or delta actions; the standard `meta/stats.json` is already
maintained automatically after every saved episode.

## What's in the dataset

| Path | Contents |
|---|---|
| `meta/` | `info.json`, `modality.json`, `episodes.jsonl`, `tasks.jsonl`, `stats.json`, plus the provenance files above |
| `data/chunk-XXX/episode_XXXXXX.parquet` | 30 Hz observation/action rows |
| `videos/chunk-XXX/observation.images.<cam>/episode_XXXXXX.mp4` | 30 fps camera videos, PTS aligned to the parquet timestamps |
| `extras/` | Optional raw Quest controller stream (`episode_XXXXXX.quest.jsonl`) when `recording.save_raw_quest` is enabled — useful for debugging input issues |

Columns, units, and conventions (absolute actions, quaternion `wxyz`, meters
and radians) are specified in the
[dataset format reference](../reference/dataset-format.md). When you have a
few episodes, verify them with [replay](replay.md) before training.
