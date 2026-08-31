# SpaceMouse teleoperation

A 3Dconnexion SpaceMouse lets you drive the Piper arm from your desk — no
headset, no room setup, no clutch choreography. You nudge the puck, the
end-effector moves; you let go, it stops. This page covers setup, the button
mapping, and how to tune the feel.

## When to use it

Reach for the SpaceMouse instead of the [Quest](quest.md) when:

- you don't have a headset handy (or don't want to wear one),
- you need precise, slow adjustments — full puck deflection tops out at a
  gentle 0.15 m/s by default,
- you're doing long desk sessions where holding a controller in mid-air
  would be tiring.

The key difference is the control mode:

| | Quest | SpaceMouse |
|---|---|---|
| Mode | Position (clutch) | **Rate** (velocity) |
| While engaged | Controller motion maps 1:1 to the EEF | Puck deflection commands EEF **velocity** |
| On release | Target freezes where it is | Arm **stops** |
| Clutch | Hold **grip** to engage | None — deflection *is* engagement |

Because it is rate control, there is nothing to re-grip or ratchet: the arm
integrates your velocity command while the puck is deflected and holds still
the moment you release it. The same safety envelope as the Quest applies —
workspace box, velocity caps, pitch guard, and the 0.12 m target-to-measured
tether (see [Control modes](../reference/control-modes.md)).

## Setup

One-time, after [installation](../getting-started/installation.md):

```bash
make spacemouse
```

This runs `install/04_spacemouse_setup.sh`, which:

1. installs a udev rule (`/etc/udev/rules.d/99-spacemouse.rules`, needs sudo)
   granting non-root access to 3Dconnexion devices, and
2. runs a 5-second read test — deflect the puck and press the buttons; you
   should see live axis values and a report rate.

If the read test fails right after installing the rule, **unplug and replug**
the SpaceMouse so the rule applies to the device node.

!!! note "No spacenavd needed"
    The driver reads HID reports directly through the Python `hidapi`
    package. You do not need to install spacenavd — and if it *is* running,
    it can grab the device and block the driver. Stop it with:

    ```bash
    sudo systemctl stop spacenavd
    ```

## Run

Power on the arm and home it first (`make home` — the controller refuses to
engage while the EEF is outside the workspace box):

```bash
piper-collect --input spacemouse    # or: make collect-sm
```

For data collection (see [Data collection](data-collection.md) for the full
workflow):

```bash
piper-collect --input spacemouse    # or: make collect-sm TASK="..."
```

The `--input` flag overrides the `input:` key in the config, so you can
switch between Quest and SpaceMouse without editing any file.

## Controls

| Input | Action |
|---|---|
| Puck deflection | EEF velocity (translate + rotate); release = stop |
| **Left** button | Toggle gripper open/close |
| **Right** button, short press | Start / stop-and-save episode |
| **Right** button, hold 1 s | Home the arm |
| **Both** buttons together | Discard current episode |

Default axis mapping: puck forward → robot +x, puck right → robot −y, puck
up → robot +z; tilting forward/back rotates about robot y, tilting
left/right rotates about robot x, and twisting rotates about robot z.

??? info "Why single-button actions feel slightly deferred"
    With only two buttons, "both together" has to be distinguishable from
    two staggered single presses. The controller waits a short chord window
    (`chord_window_s`, 0.2 s by default) after the gripper button goes down
    before toggling — if the other button arrives inside that window, the
    press becomes a discard instead. Similarly, the record button fires on
    **release** (a short press), because holding it for 1 s means "home".
    The tiny delay is deliberate, not lag.

During `piper-collect` the keyboard also works: ++space++ = start/stop,
++d++ = discard, ++h++ = home, ++q++ = quit.

## Tuning the feel

All knobs live in the `spacemouse:` block of the config. Put your changes in
an overlay file and pass `--config my.yaml` — see
[Configuration](../reference/configuration.md).

| Key | Default | What it does |
|---|---|---|
| `deadzone` | `0.10` | Normalized deflection below this is ignored; raise it if the arm creeps when your hand rests on the puck |
| `max_lin_vel` | `0.15` | Linear speed (m/s) at full deflection |
| `max_ang_vel` | `0.9` | Angular speed (rad/s) at full deflection |
| `axis_signs` | `[1, 1, 1, -1, -1, -1]` | Per-axis sign flips, in order `x, y, z, roll, pitch, yaw` |
| `stale_timeout_s` | `0.15` | No HID report within this window → twist treated as zero |

!!! tip "Flipping an axis that feels backwards"
    If, say, pushing the puck forward moves the arm the wrong way, flip the
    corresponding entry in `axis_signs` from `1` to `-1` (or vice versa) in
    your overlay. The order is `x, y, z, roll, pitch, yaw` of the *puck*
    axes. The shipped defaults invert the three rotation axes to match
    operator preference.

The device only sends reports while it is being touched, so `stale_timeout_s`
guarantees the commanded velocity drops to zero shortly after you let go —
the arm can never keep integrating a stale deflection.

## Supported devices

The driver auto-detects the first supported device it finds (set
`spacemouse.device` to a name to pin one). Recognized models:

| Model | VID:PID |
|---|---|
| SpaceMouse Compact | `256f:c635` |
| SpaceMouse Wireless (cabled) | `256f:c62e` |
| SpaceMouse Wireless (new) | `256f:c63a` |
| 3Dconnexion Universal Receiver | `256f:c652` |
| SpaceNavigator | `046d:c626` |
| SpaceNavigator for Notebooks | `046d:c628` |

!!! warning "Verified on the Compact"
    The HID report parsing was written and verified against the
    **SpaceMouse Compact**. The other models are recognized and are expected
    to share the same report layout (wireless models pack translation and
    rotation into a single combined report, which is handled), but they have
    not all been tested — a different model may need small report-parsing
    tweaks in `piper_teleop/spacemouse_client.py`.

If the driver reports `no supported SpaceMouse found` or cannot open the
device, re-run `make spacemouse`, replug, and check spacenavd — more in
[Troubleshooting](../troubleshooting.md).
