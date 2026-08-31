# Troubleshooting

Find your symptom below and apply the fix. Entries are grouped by area, in
the order you typically meet them. If nothing matches, start by
[collecting logs](#getting-logs) — most problems announce themselves there.

## Quest connection

### `adb devices` shows no device, or `unauthorized`

**Cause:** missing udev permissions on the host, or the USB-debugging prompt
inside the headset was never accepted.

```bash
bash install/05_quest_udev.sh   # one-time, sudo: udev rule + adb server restart
```

Then put on the headset, accept the **Allow USB debugging** dialog, and check
again:

```bash
adb devices
```

!!! tip
    Use a USB *data* cable, and re-run `make connect` every time you plug the
    Quest back in — the `adb forward` port mappings do not survive a replug.

### App is running but no pose data arrives

**Cause:** the port forwards are not active, or the headset went to sleep
(the proximity sensor stops tracking when nothing wears the headset).

```bash
make connect        # (re)install + launch app, adb forward 8735/8736,
                    # wake headset, disable proximity sensor
```

`make connect` runs `scripts/quest_connect.sh`, which forwards TCP 8735
(pose stream) and TCP 8736 (video panels) over the cable. To re-enable the
proximity sensor later:
`adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable`.

If poses still do not arrive, watch the app log: `adb logcat -s QuestTeleop`.

### Controllers report `valid` but `tracked=false`

**Cause:** the controllers are tracked by the headset's cameras — the headset
is not looking at them.

**Fix:** point the headset toward the operating area (e.g. rest it on a table
facing the workspace, with the proximity sensor disabled by `make connect`).
See the [Quest guide](guide/quest.md) for the full setup.

## Arm and CAN

### `failed to enable arm within 5 s — check power/CAN`

**Cause:** the arm is not powered, or the CAN interface is down / at the wrong
bitrate.

```bash
make can                        # bring up can0 at 1 Mbps (sudo, idempotent)
ip -details link show can0      # confirm: state UP, bitrate 1000000
```

If the script prints `can0 not found`, plug in the USB-CAN adapter
(gs_usb/candleLight) first.

### `TARGET_POS_EXCEEDS_LIMIT(4)` right after starting teleop

**Cause:** the arm is still in the factory folded pose, which sits slightly
outside the joint operating range, so the firmware rejects Cartesian targets.

```bash
make home           # unfold to the working pose (required after every power-on)
```

The controllers also refuse to engage outside the workspace box, so make
homing the first thing you do after switching the arm on.

### Startup refuses because of a firmware mismatch

**Cause:** `arm.firmware` in the config does not match what the arm reports.
On connect the console prints both sides:

```text
[piper] connected on can0; firmware: ...; configured PiperFW.V189
```

**Fix:** set `arm.firmware` to the pyAgxArm `PiperFW` enum matching your arm.
The verified pairing is firmware `S-V1.8-9` → `arm.firmware: V189`. See
[Configuration](reference/configuration.md).

## Impedance behaviour

The default backend is `piper_mit` (MIT impedance). All gains live under the
`impedance:` key; see [Control modes](reference/control-modes.md).

### The arm hums or buzzes, or is hard to push

**Cause:** gains too high — too much `kd` in particular amplifies velocity
sensor noise into an audible hum.

**Fix:** lower `impedance.kp` and `impedance.kd` (shipped soft defaults:
`kp: [5, 25, 5, 8, 8, 5]`, `kd: [0.3, 1.2, 0.3, 0.45, 0.45, 0.3]`).

### The arm droops under gravity

**Cause:** stiffness too low for the pose, or gravity feedforward inactive.

**Fix:** raise `impedance.kp`, and keep `impedance.gravity_ff: true` — it
snapshots the position servo's holding torques at impedance entry, so there is
no droop at or near the entry pose. Note that ~11 mm static hold error is
expected with the shipped soft gains.

### Tracking feels sluggish or loose

**Cause:** this is the soft-gain trade-off: ~14 mm dynamic RMS and ~90 ms
latency with the shipped gains, versus 1.34 mm / ~10 ms in position mode.

**Fix:** raise `impedance.kp` for tighter tracking, or switch to firmware
position control with `--position` for high-precision free-space tasks.
Measure the difference yourself with `piper-track-test` — see
[Benchmarks](reference/benchmarks.md).

## SpaceMouse

### Device open fails

**Cause:** no permission on the hidraw/USB device node.

```bash
make spacemouse     # one-time, sudo: udev rules + 5 s read test
```

Then **re-plug** the SpaceMouse and repeat the read test (deflect the puck —
you should see live axis values and a rate readout).

### spacenavd conflict

**Cause:** the Python driver reads HID reports directly; a running `spacenavd`
daemon can grab the device first.

```bash
sudo systemctl stop spacenavd
```

## Cameras

### Camera views are swapped or misnamed

**Cause:** camera names are pinned to device serial numbers in the config, and
the serials do not match your cameras.

**Fix:** edit the `cameras:` list (`cam_front` / `cam_wrist` / `cam_back`) so
each `serial:` matches the physically mounted camera. See
[Configuration](reference/configuration.md). List the serials of the
connected cameras with:

```bash
python -c "import pyrealsense2 as rs; \
  [print(d.get_info(rs.camera_info.serial_number), d.get_info(rs.camera_info.name)) \
   for d in rs.context().devices]"
```

### A camera fails to start on a USB 2 port

**Cause:** bandwidth. On a USB 2.1 port, 640x480 @ 30 fps is the maximum for a
RealSense color stream.

**Fix:** keep `width: 640`, `height: 480`, `fps: 30` (the recording default)
for that camera, or move it to a USB 3 port.

## Tests and Python

### pytest fails with errors from external plugins

**Cause:** globally installed pytest plugins leak into the test run.

```bash
make test           # runs the 31 unit/integration tests with an isolated PYTHONPATH
```

## Getting logs

- **Quest app log** (connection, tracking, streaming events):

    ```bash
    adb logcat -s QuestTeleop
    ```

- **Console status line:** `piper-collect` continuously prints one live line —
  engage state (`ENGAGED`/`idle`), input rate in Hz, current target position,
  gripper width in mm, and the most recent error message. `piper-collect`
  prints an equivalent `[collect]` line while recording. If something
  misbehaves mid-session, the error field here is the first place to look.
