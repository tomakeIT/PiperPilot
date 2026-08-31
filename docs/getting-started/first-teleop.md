# First run on the real arm

This page walks you through your first careful teleoperation session on a
real Piper arm: check the hardware link, unfold the arm from its factory
pose, and drive it with a conservative speed/scale overlay. It assumes you
have completed [Installation](installation.md).

## Safety first

!!! danger "The arm will move"
    The default control mode is MIT impedance — compliant, with contact force
    bounded to roughly 20 N at the shipped gains — but the arm still moves
    with real momentum. Before you power on:

    - Clear the workspace: no cables, cups, monitors, or hands inside the
      reachable volume.
    - Keep one hand near the arm's power switch (or e-stop, if your setup has
      one) for the whole first session.
    - Stand where you can see the arm but outside its reach.

The software adds its own safety envelope (workspace box, velocity/step
clamps, a 0.12 m target-deviation tether, stale-stream watchdogs,
fault-triggered disengage), but treat it as a backstop, not a substitute for
attention.

## Pre-flight checklist

Work through these in order every time you set up:

1. **CAN link up.** Run `make can` (needs sudo; brings up `can0` at 1 Mbps),
   then verify the bitrate:

    ```bash
    ip -details link show can0    # look for "bitrate 1000000"
    ```

2. **Arm powered on.** If the arm has no power or the CAN link is wrong, the
   software fails to enable the arm within 5 s and exits with
   `failed to enable arm within 5 s — check power/CAN`.

3. **Firmware pairing.** The config key `arm.firmware` (a pyAgxArm enum name)
   must match the firmware the robot actually reports. The verified pairing
   is a Piper-X running firmware S-V1.8-9 with `arm.firmware: V189`. On a
   mismatch the software refuses to start — fix the config, not the check.

4. **Input device ready.**

    === "Quest"

        Plug in the headset over USB and run `make connect`. It installs and
        starts the Quest app, sets up the `adb forward` port forwarding, and
        disables the proximity sensor. Run it every time you plug the
        headset in.

    === "SpaceMouse"

        Plug the SpaceMouse into USB. If this is the first time on this
        machine, run `make spacemouse` once (sudo) to install udev rules and
        run a read test.

## Unfold the arm: `make home`

Run this after **every** arm power-on, before anything else:

```bash
make home
```

The factory rest pose is folded slightly outside the joint operating range,
so Cartesian motion from there is rejected by the firmware with
`TARGET_POS_EXCEEDS_LIMIT(0x4)`. The folded pose is also outside the teleop
workspace box, so the controllers refuse to engage and report
`EEF outside workspace — run 'make home' first`.

Homing is a firmware-planned joint-space move at 30% speed by default. Stand
clear: the arm unfolds to a working pose (shoulder up, elbow bent, gripper
level and pointing forward) while the console prints the remaining joint
error, then confirms the end-effector is inside the workspace box:
`OK — arm is holding position; teleop can start now.`

## Start teleop with the conservative overlay

For your first sessions, add the shipped `first_run.yaml` overlay. It layers
on top of the default config and only overrides the keys it sets:

```bash
make collect CONFIG=first_run.yaml
# equivalent: piper-collect --config first_run.yaml
```

This is the teleop app: recording starts only if you press **A** / ++space++,
and nothing is written to disk until an episode is saved.

In short: arm speed governed to 50%, hand motion scaled to 60%, and all
velocity caps roughly halved (for both Quest and SpaceMouse). The full
key-by-key table is in the
[configuration reference](../reference/configuration.md#shipped-overlays).
Plain `make collect` uses `default.yaml` alone — full speed and 1:1 scale.

Once you are comfortable, drop the `--config` flag to get full speed and
1:1 scale. The overlay also works with `piper-collect`.

## Engage and move

=== "Quest"

    The control hand is the right controller by default
    (`quest.control_hand`).

    1. Hold **grip** to clutch in. From that moment your controller's motion
       maps 1:1 (scaled by `teleop.pos_scale`) onto the end-effector as an
       incremental offset.
    2. Move gently — small, slow translations first, then small rotations.
    3. Release **grip** to freeze the target where it is. The arm holds
       position.
    4. Reposition your hand comfortably and re-grip to continue from the
       frozen target — this "ratchet" lets you cover the whole workspace
       without contorting your arm.
    5. Squeeze **trigger** (analog) to close the gripper; it tracks the
       trigger continuously.

    If you ever need to bail out, the left-hand **menu** button forces a
    clutch release. Full bindings are in the [Quest guide](../guide/quest.md).

=== "SpaceMouse"

    The SpaceMouse is a rate-control device: puck deflection commands
    end-effector *velocity*, and releasing the puck stops the arm.

    1. Nudge the puck gently — deflection below the 0.10 deadzone is
       ignored, and full deflection maps to the configured maximum velocity
       (0.08 m/s with the first-run overlay).
    2. Press the **left button** to toggle the gripper open/closed.

    If a direction feels backwards, see the
    [SpaceMouse guide](../guide/spacemouse.md) for `spacemouse.axis_signs`.

## Reading the status line

`piper-collect` prints a single continuously-updating status line:

```text
[collect] input  90.9Hz | ENGAGED | grip   55mm | 0 eps saved
```

| Field | Meaning |
| --- | --- |
| `ENGAGED` / `idle` | Whether the clutch is engaged (arm follows input) |
| `input=...Hz` | Input device rate — expect around 90 Hz from the Quest |
| `target=(x,y,z)` | Commanded end-effector target position, meters, base frame |
| `grip=...mm` | Gripper width |
| trailing text | Last error, e.g. the "run `make home` first" message |

## What impedance mode feels like

The default `piper_mit` backend makes the arm behave like a spring-damper
around the streamed target, so it feels springy rather than rigid:

- **Slight sag or lag is normal.** With the shipped soft gains, measured
  static hold error is 11 mm, dynamic tracking is 14.1 mm RMS, and latency
  is around 90 ms. Pushing on the arm deflects it and it springs back.
- **Contact is safe by design.** Force is bounded by the stiffness times the
  0.12 m target tether — about 20 N with the shipped gains.

If your task needs tight free-space precision and the workspace is clear of
contact, try firmware position control instead:

```bash
piper-collect --position --config first_run.yaml
```

Position mode tracks at 1.34 mm dynamic RMS with ~10 ms latency, but it is a
stiff position servo — it does not yield on contact, so keep the workspace
clear. See [Control modes](../reference/control-modes.md) for how the two
backends work and how to tune the impedance gains.

## Next steps

- Something failed? [Troubleshooting](../troubleshooting.md) covers the
  common first-run errors: `TARGET_POS_EXCEEDS_LIMIT(4)` (arm still folded —
  run `make home`), the enable-arm timeout, missing Quest data, and more.
- Ready to record demonstrations? Continue to
  [Data collection](../guide/data-collection.md).
