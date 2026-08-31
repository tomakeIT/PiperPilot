# Quest 3 teleoperation

The Quest 3 is the highest-fidelity input device for PiperPilot: you move
your hand, the arm follows. A native C++/OpenXR app (no Unity) runs on the
headset and streams both controller poses at 90 Hz over a wired USB link
(`adb forward` — measured 90–92 Hz in practice). You see the real room through
color passthrough, with up to three floating camera panels and a recording
status HUD overlaid in front of you, and the controllers give haptic feedback
when you start or save an episode.

This page covers building and installing the app, connecting each session, the
controller bindings, and the config keys that shape the experience. For the
recording workflow itself, see [Data collection](data-collection.md).

## One-time setup

You need a Quest 3 in **developer mode** with a USB data cable (see the
hardware table in [Installation](../getting-started/installation.md)).

1. Install the udev rule so `adb` can talk to the headset without root:

    ```bash
    bash install/05_quest_udev.sh
    ```

2. Install the Android toolchain and build the APK:

    ```bash
    make toolchain   # Android SDK/NDK/JDK17/Gradle (~2.5 GB, APK builds only)
    make apk         # builds quest_app/app/build/outputs/apk/debug/app-debug.apk
    ```

The app installs as the package `com.pipertools.questteleop`. You do not need
`make toolchain` on machines that only run teleop — it is required only to
build the APK.

## Every session: connect

Plug the headset in via USB, then:

```bash
make connect
```

This runs `scripts/quest_connect.sh -p`, which brings up the whole wired link:

1. Finds `adb` (prefers the SDK copy in `~/android-sdk/platform-tools`) and an
   authorized USB device.
2. Installs the APK if it is not already on the headset.
3. Wakes the headset and launches the app.
4. Sets up `adb forward` for both TCP ports: **8735** (pose/action stream) and
   **8736** (camera video).
5. Disables the proximity sensor, so the headset keeps tracking while resting
   on a table.

!!! warning "Authorize USB debugging"
    The first time you plug in, put the headset on and accept the
    **Allow USB debugging** dialog. Until you do, `adb devices` reports the
    headset as `unauthorized` and the script exits with an error.

Running the script directly gives you two flags:

| Flag | Effect |
|---|---|
| `-i` | Force-reinstall the APK (use after `make apk` rebuilds it) |
| `-p` | Disable the proximity sensor (what `make connect` passes for you) |

To re-enable the proximity sensor later:
`adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable`.

Verify the link with `make view` (controller poses in a rerun window) and
watch the app log with `adb logcat -s QuestTeleop`. If nothing arrives, see
[Troubleshooting](../troubleshooting.md).

## Controls

The **control hand** drives the arm — right by default, switchable with
`quest.control_hand` or the `--hand left` CLI flag.

| Input | Action |
|---|---|
| **grip** (hold) | Clutch: while held, controller motion maps 1:1 (incremental) to the end effector; release freezes the target |
| **trigger** (analog) | Gripper open/close, continuous |
| **A** / **X** | Start / stop-and-save episode (haptic confirmation, view tints red) |
| **B** / **Y** | Discard the current episode |
| Non-control hand **Y**/**B**, hold 1 s | Home the arm back to the working pose |
| Non-control hand **X**/**A**, hold 1 s | Discard the current episode — or, when not recording, delete the last episode saved this session |
| Left **menu** | Force clutch release |
| **Meta** button, long press | Re-place the floating panels in front of you |

The episode buttons act during `piper-collect` sessions — see
[Data collection](data-collection.md).

### The clutch, explained

The grip button works like a clutch pedal, not an on/off teleport:

- **Engage.** Squeeze the grip past the analog threshold (0.7 by default). From
  that moment, every controller displacement is added to the arm target,
  scaled by `teleop.pos_scale`. The arm never jumps to where your hand is — it
  only follows your *motion*.
- **Release.** Let go (below the 0.3 hysteresis threshold) and the target
  freezes exactly where it is. Move your hand anywhere; the arm ignores it.
- **Ratchet.** Re-grip from a comfortable position and continue. Long reaches
  become several short grip-move-release strokes, like ratcheting a wrench —
  your arm stays in a comfortable envelope while the robot covers the full
  workspace.

!!! tip
    If the clutch will not engage, the target is probably outside the
    workspace safety box — home the arm first (non-control hand **Y**/**B**
    held 1 s, or `make home`). If it engages but feels stuck, press the left
    **menu** button to force a clean release and re-grip.

## What you see in the headset

Everything renders over color passthrough, so you keep direct eye contact with
the robot:

- **Up to 3 camera panels**, each labeled with its camera name from the config
  (e.g. `cam_front`, `cam_wrist`). A panel hides itself if its stream goes
  stale for more than 3 s.
- **A status HUD panel** showing the recording state: a **REC** headline with
  the episode index and a running timer while recording, **READY** with the
  episodes-saved count between episodes, and **HOMING** while the arm homes.

Long-press the **Meta** button at any time to re-place all panels in front of
wherever you are currently facing.

## Streaming cameras into the headset

The recording cameras stream into the panels automatically whenever
`quest.video.enabled` is `true` (the default) — during collection and plain
teleop alike (both are `piper-collect`; nothing is recorded until you press
**A**).

!!! note "Pose stream always wins"
    Video runs on its own TCP port (8736), separate from the pose stream
    (8735), and it drops frames rather than queueing them. A slow or busy
    video link can never add latency to the 90 Hz pose stream that drives the
    arm.

## Calibrating the X axis (and keeping it across reboots)

The headset's STAGE space has no absolute yaw: gravity fixes up/down, but
the horizontal axes follow the guardian boundary, which silently rotates
whenever the boundary is redrawn or the room is not recognized after a
reboot. Symptom: pushing the controller forward moves the arm diagonally,
by a different angle each boot.

**Two-point calibration** (keyboard **`c`** in `piper-collect`):

1. Press **`c`**. The headset tints blue, the controller buzzes, and teleop
   input is paused (nothing you press in VR reaches the arm or recorder).
2. Hold the controller at a first point (e.g. above the robot base) and
   pull the **trigger**.
3. Move the controller at least 15 cm along the direction the robot's
   **+X** should point and pull the **trigger** again. A faint arrow
   previews the axis live while you move.
4. Done: the P1→P2 line (horizontal projection) becomes robot +X, the
   frame is saved as a **persistent spatial anchor**, both controllers
   buzz, and the status line shows `[anchor]`. The arrow stays on the
   floor plane, pinned to the room.

The saved anchor is restored on every later boot — same axes, no re-tuning
— as long as the headset recognizes the room. Re-press **`c`** anytime to
recalibrate (the old anchor is replaced); the **menu button** or pressing
**`c`** again cancels.

`teleop.yaw_offset_deg` still works as a manual trim on top, and **`a`**
(pin current frame as-is) / **`A`** (clear anchor) remain for the manual
workflow — but two-point calibration is the intended path.

If the status line shows `ANCHOR LOST — axes may be rotated`, the headset
has not recognized the room (yet): look around the workspace for a few
seconds — the app retries every 10 s. If it never recovers (e.g. the space
was deleted in headset settings), just recalibrate with **`c`**. Keep the
room lighting and layout roughly stable — anchors survive exactly as well
as Quest relocalization does.

## Config keys that matter

| Key | Default | Meaning |
|---|---|---|
| `quest.control_hand` | `right` | Which controller drives the arm |
| `quest.video.enabled` | `true` | Stream camera panels into the headset |
| `quest.video.port` | `8736` | Video TCP port (forwarded by `make connect`) |
| `quest.video.fps` | `15` | Panel frame rate |
| `quest.video.quality` | `70` | JPEG quality of the panel stream |
| `quest.video.max_width` | `640` | Downscale cap for panel frames |
| `teleop.pos_scale` | `1.0` | Controller motion → arm motion scale (0.6 in `first_run.yaml`) |
| `teleop.yaw_offset_deg` | `0.0` | Manual trim about robot +z on top of the XR→robot mapping. Prefer two-point X calibration (press `c`) — leave this at 0 when using it |

Put overrides in a YAML overlay and pass `--config file.yaml`; only the keys
you set are overridden. The full key list lives in the
[Configuration reference](../reference/configuration.md).
