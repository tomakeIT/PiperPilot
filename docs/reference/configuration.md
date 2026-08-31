# Configuration

Every tool in PiperPilot (`piper-collect`, `piper-replay`, `piper-infer`, ...) reads the same YAML configuration. This page explains how the layering works, describes the two overlay files that ship with the repository, and then documents every key in the default configuration, block by block.

## How configuration works

Configuration is built in three layers, each applied on top of the previous one:

1. **`piper_teleop/configs/default.yaml`** — always loaded first. It contains every key with a working default.
2. **`--config file.yaml`** — an optional *overlay*. Only the keys you set in the overlay are overridden; everything else keeps its default. Paths starting with `~` are expanded.
3. **CLI flags** — applied last, on top of both files.

The merge is a *deep* merge (`_deep_update` in `piper_teleop/config.py`): nested dictionaries are merged key by key, so an overlay containing only `teleop.limits.max_lin_vel` touches exactly that one value. Any non-dictionary value — scalars **and lists** — replaces the default wholesale.

!!! note "Lists are replaced, not merged"
    If your overlay sets `impedance.kp`, you must give all six values. If it sets `cameras`, it replaces the entire camera list — you cannot patch a single camera entry.

The CLI flags that map onto config keys (see [CLI reference](cli.md) for the full flag list):

| Flag | Effect on config |
|---|---|
| `--input quest\|spacemouse` | sets `input` |
| `--sim` | sets `arm.backend: fake` (no hardware) |
| `--position` | sets `arm.backend: piper` (move_p position control) |
| `--impedance` | sets `arm.backend: piper_mit` (the default backend anyway) |
| `--hand left\|right` | sets `quest.control_hand` |

If you pass more than one backend flag, `--sim` wins, then `--position`, then `--impedance`.

## Shipped overlays

Two overlay files live in the repository root.

=== "first_run.yaml"

    A conservative overlay for your **first sessions on the real arm**. Use it until you are comfortable, then drop it (defaults are full speed/scale):

    ```bash
    make collect CONFIG=first_run.yaml
    piper-collect --config first_run.yaml   # equivalent
    piper-collect --config first_run.yaml --task "..."
    ```

    | Key | Overlay value | Default | Effect |
    |---|---|---|---|
    | `arm.speed_percent` | `50` | `100` | firmware-side global speed governor at half |
    | `teleop.pos_scale` | `0.6` | `1.0` | hand motion damped to 60% |
    | `teleop.limits.max_lin_vel` | `0.4` | `0.8` | m/s cap halved |
    | `teleop.limits.max_ang_vel` | `1.5` | `2.5` | rad/s cap reduced |
    | `spacemouse.max_lin_vel` | `0.08` | `0.15` | m/s at full deflection |
    | `spacemouse.max_ang_vel` | `0.5` | `0.9` | rad/s at full deflection |

    See [Your first teleop session](../getting-started/first-teleop.md) for the walkthrough that uses it.

=== "gravity_test.yaml"

    A diagnostic probe that answers one question: **does the firmware gravity-compensate in MIT mode?** It sets `kp` near zero (no spring to hold the arm up) with moderate `kd` (any fall stays slow and damped):

    ```bash
    piper-collect --impedance --config gravity_test.yaml
    ```

    The procedure and how to interpret the result are described in
    [Control modes & tuning](control-modes.md#the-gravity-test-probe).

    | Key | Overlay value |
    |---|---|
    | `impedance.kp` | `[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]` |
    | `impedance.kd` | `[1.0, 1.5, 1.2, 0.5, 0.5, 0.3]` |
    | `impedance.engage_on_start` | `true` |

## Key reference

All units are meters, radians, and seconds unless noted otherwise.

### `input`

| Key | Default | Meaning |
|---|---|---|
| `input` | `quest` | teleop input device: `quest` or `spacemouse` |

### `quest`

Settings for the Meta Quest 3 link (see the [Quest guide](../guide/quest.md)).

| Key | Default | Meaning |
|---|---|---|
| `host` | `127.0.0.1` | via `adb forward tcp:8735 tcp:8735` (wired USB) |
| `port` | `8735` | pose/JSON stream port |
| `control_hand` | `right` | which controller drives the arm |
| `stale_timeout_s` | `0.25` | freeze arm target if no fresh pose within this window |

#### `quest.video`

Streams RealSense views into the headset as floating panels over passthrough. Video uses a separate port so the pose stream keeps absolute priority.

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | stream camera panels into the headset |
| `port` | `8736` | MJPEG video port (separate from the pose stream) |
| `fps` | `15` | panel stream frame rate |
| `quality` | `70` | JPEG quality |
| `max_width` | `640` | maximum panel frame width |

### `teleop`

The Quest clutch-based teleop controller.

| Key | Default | Meaning |
|---|---|---|
| `rate_hz` | `100` | control loop rate (move_p streaming) |
| `pos_scale` | `1.0` | controller motion → arm motion scale |
| `clutch_engage` | `0.7` | squeeze (grip) analog threshold to engage |
| `clutch_release` | `0.3` | hysteresis release threshold |
| `yaw_offset_deg` | `0.0` | extra rotation about robot +z applied to the XR→robot mapping (tune if the operator does not face the same direction as robot +x) |

#### `teleop.filter`

One-euro filter on the controller pose.

| Key | Default | Meaning |
|---|---|---|
| `min_cutoff` | `1.2` | Hz; lower = smoother but laggier at rest |
| `beta` | `0.02` | speed coefficient; higher = less lag during fast motion |
| `d_cutoff` | `1.0` | derivative cutoff |

#### `teleop.limits`

!!! danger "Safety envelope"
    These keys define the safety envelope for the real arm: the workspace box, per-tick velocity clamps, and the `max_cmd_deviation` tether that prevents catch-up lunges after blocking or faults. Widen them only deliberately, with clear space around the robot. Controllers refuse to engage while the end effector is outside the workspace box — run `make home` after power-on first.

| Key | Default | Meaning |
|---|---|---|
| `workspace_min` | `[0.15, -0.35, 0.03]` | EEF position box lower corner, robot base frame |
| `workspace_max` | `[0.55, 0.35, 0.50]` | box upper corner (Piper reach ~0.62 m — stay inside) |
| `max_lin_vel` | `0.8` | m/s cap on target displacement per tick |
| `max_ang_vel` | `2.5` | rad/s cap on target rotation per tick |
| `pitch_abs_max_deg` | `88.0` | keep clear of move_p's hard pitch limit (90 deg) |
| `max_cmd_deviation` | `0.12` | m; target is tethered to the measured EEF pose (prevents catch-up lunges after blocking/faults) |

### `spacemouse`

Rate control: puck deflection → EEF velocity (see the [SpaceMouse guide](../guide/spacemouse.md)).

| Key | Default | Meaning |
|---|---|---|
| `device` | `""` | pyspacemouse device name; `""` = auto-detect |
| `rate_hz` | `100` | control loop rate |
| `deadzone` | `0.10` | normalized deflection below this is ignored |
| `max_lin_vel` | `0.15` | m/s at full deflection |
| `max_ang_vel` | `0.9` | rad/s at full deflection |
| `axis_signs` | `[1, 1, 1, -1, -1, -1]` | flip individual axes: x, y, z, roll, pitch, yaw (rotations inverted to match operator preference) |
| `gripper_button` | `0` | left button: toggle gripper |
| `record_button` | `1` | right button: short press = start/stop episode, long press (`home_hold_s`) = home; both buttons = discard |
| `chord_window_s` | `0.2` | single-button actions wait this long for a chord partner |
| `home_hold_s` | `1.0` | hold the record button this long to home the arm |
| `stale_timeout_s` | `0.15` | no HID report within this window → twist treated as zero |

### `arm`

The pyAgxArm backend and CAN link.

!!! danger "arm.firmware must match the robot"
    `firmware` is a pyAgxArm `PiperFW` enum name and **must match** what `robot.get_firmware()` reports — on mismatch the tools refuse to start. Verified pairing on this arm: firmware `S-V1.8-9` → `V189`. See [Troubleshooting](../troubleshooting.md) if startup fails here.

| Key | Default | Meaning |
|---|---|---|
| `backend` | `piper_mit` | `piper_mit` = MIT impedance (default: host IK + per-joint spring-damper, compliant on contact, bounded-force safety); `piper` = move_p position control (firmware IK, tightest tracking; CLI `--position`); `fake` = full pipeline without hardware (CLI `--sim`) — see [Control modes](control-modes.md) |
| `model` | `PIPER_X` | pyAgxArm `ArmModel` enum name (`PIPER` / `PIPER_X` / ...) |
| `firmware` | `V189` | pyAgxArm `PiperFW` enum name — must match `robot.get_firmware()` |
| `interface` | `socketcan` | CAN interface type |
| `channel` | `can0` | CAN channel |
| `bitrate` | `1000000` | CAN bitrate |
| `speed_percent` | `100` | firmware-side global speed governor |
| `enable_soft_joint_limits` | `true` | enable soft joint limits |
| `home_joints` | `null` | e.g. `[0.0, 0.8, -0.6, 0.0, 0.6, 0.0]`; `null` = don't auto-home |

### `impedance`

Used by the `piper_mit` backend only. Torque law: `T = kp*(q_des - q) - kd*v + t_ff`. See [Control modes](control-modes.md) for how this compares to position mode, and [Benchmarks](benchmarks.md) for measured tracking numbers with the shipped gains.

!!! warning "kp/kd are provenance-bound"
    The control backend and gains (`backend`, `kp`, `kd`, `t_ff`, `gravity_ff`, plus `arm.firmware` and `recording.fps`) are recorded in each dataset's provenance metadata. Changing any of them between sessions of the same dataset prints a prominent warning — your demonstrations would mix different arm dynamics. See [Dataset format](dataset-format.md).

| Key | Default | Meaning |
|---|---|---|
| `kp` | `[5, 25, 5, 8, 8, 5]` | N·m/rad — near SDK reference (10); tune on-robot: droops under gravity → raise; hums/buzzes → lower |
| `kd` | `[0.3, 1.2, 0.3, 0.45, 0.45, 0.3]` | damping; too high amplifies velocity sensor noise into audible hum (SDK ref 0.8) |
| `t_ff` | `[0, 0, 0, 0, 0, 0]` | manual feed-forward trim (N·m), added on top |
| `gravity_ff` | `true` | snapshot the position servo's holding torques at MIT entry → no droop at/near entry pose, torque-smooth handover (auto-disabled if the post-entry deflection check fails) |
| `soft_start_s` | `0.8` | gain ramp at MIT entry (kills handover clunk) |
| `engage_on_start` | `true` | enter impedance hold right at startup (`false` = only on first clutch engage) |
| `max_joint_vel` | `1.5` | rad/s per-joint target step clamp |

#### `impedance.ik`

Host-side damped-least-squares IK used in MIT mode.

| Key | Default | Meaning |
|---|---|---|
| `iters` | `4` | IK iterations per tick |
| `damping` | `0.05` | DLS lambda |
| `pos_tol` | `0.002` | m; position tolerance |

#### `impedance.joint_limits_deg`

Auto-intersected with the SDK per-model preset (PIPER_X: j4/j5 are ±89 in the SDK).

| Key | Default |
|---|---|
| `lower` | `[-148, 2, -168, -87, -87, -178]` |
| `upper` | `[148, 178, -2, 87, 87, 178]` |

### `gripper`

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | enable the gripper |
| `max_width` | `0.07` | 0.07 or 0.10 depending on installed gripper stroke |
| `force` | `2.0` | newtons |
| `rate_hz` | `50` | gripper command rate (separate from arm rate) |

### `cameras`

A **list** of RealSense color streams. Each entry has `name`, `serial`, `width`, `height`, `fps`.

!!! warning "Pin serials on multi-camera rigs"
    Empty serials auto-select unused cameras in enumeration order. For stable
    front/wrist/back names, set your own serials in a git-ignored local overlay.
    Overriding `cameras` replaces the entire list.

| Name | Default serial | Resolution |
|---|---|---|
| `cam_front` | `""` (auto) | 640x480 @ 30 |
| `cam_wrist` | `""` (auto) | 640x480 @ 30 |
| `cam_back` | `""` (auto) | 640x480 @ 30 |

### `monitor`

Local web dashboard served by `piper-collect` (see
[Collecting datasets](../guide/data-collection.md)).

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | serve the dashboard while collecting |
| `host` | `127.0.0.1` | bind address; `0.0.0.0` allows other LAN devices to view (unauthenticated — this exposes the camera views to the LAN) |
| `port` | `8780` | HTTP port |

### `recording`

LeRobot dataset output (see [Data collection](../guide/data-collection.md) and [Dataset format](dataset-format.md)).

| Key | Default | Meaning |
|---|---|---|
| `fps` | `30` | proprio/action rate; matches camera fps so every row has a near-simultaneous fresh frame — configure your trainer with `action_frequency=30` (e.g. sampling stride 3 for a 10 Hz decision rate) |
| `root` | `~/piper_datasets` | dataset root directory |
| `task` | `"pick up the object and place it in the box"` | default language instruction (override with `--task`) |
| `robot_type` | `piper` | robot type string written into the dataset |
| `chunks_size` | `1000` | episodes per dataset chunk |
| `save_raw_quest` | `true` | also dump the raw quest stream per episode (debug) |

#### `recording.video`

ffmpeg encode settings for the camera mp4 files.

| Key | Default | Meaning |
|---|---|---|
| `codec` | `libx264` | video codec |
| `pix_fmt` | `yuv420p` | pixel format |
| `crf` | `23` | constant rate factor (quality) |
| `preset` | `veryfast` | encoder speed/size preset |

### `viz`

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `false` | rerun visualization during teleop/collect |
| `spawn` | `true` | spawn a local rerun viewer (`false` + `--serve` for headless) |
