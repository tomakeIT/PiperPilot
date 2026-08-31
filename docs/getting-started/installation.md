# Installation

This page walks through the full installation for real hardware. Each piece of
hardware has its own install step, so start with the matrix below and only run
the steps your setup needs.

## What do you need for what

| Your setup | Required steps |
|---|---|
| Simulation only (`make collect-sim`) | `make env` |
| Real Piper arm | `make env` + `make can` |
| Quest 3 teleoperation | above + developer mode, `bash install/05_quest_udev.sh`, `make toolchain`, `make apk`, `make connect` |
| SpaceMouse teleoperation | above (arm steps) + `make spacemouse` |
| RealSense cameras | nothing extra — the drivers ship with `make env` |

`make toolchain` and `make apk` exist to build the Quest APK; the toolchain is
not needed for anything else.

## Host prerequisites

- Linux. Ubuntu 22.04 is the verified platform.
- Conda. The install scripts default to `$HOME/miniforge3/bin/conda`; override
  `CONDA_BIN` when conda is installed elsewhere.
- Git, to clone this repository and the pyAgxArm SDK.

```bash
git clone https://github.com/tomakeIT/PiperTeleopTools.git
cd PiperTeleopTools
```

## Python environment

```bash
make env
```

This runs `install/02_python_env.sh`, which:

1. Creates the `piper_teleop` conda env (Python 3.11) if it does not exist.
2. Installs the pip dependencies from `install/requirements.txt` (numpy,
   pandas, pyarrow, av, opencv-python, pyrealsense2, rerun-sdk, python-can,
   hidapi, pytest, and friends).
3. Installs the AgileX arm SDK **pyAgxArm** as an editable package from
   `$PYAGXARM_PATH` (default `~/pyAgxArm`). The script exits with an error if
   that checkout is missing — clone it first:

    ```bash
    git clone https://github.com/agilexrobotics/pyAgxArm ~/pyAgxArm
    ```

4. Installs this repository as an editable package, which provides the CLI
   entry points (`piper-collect`, `piper-home`, and the rest —
   see the [CLI reference](../reference/cli.md)).
5. Writes a `pip freeze` lock file into `install/` and runs an import
   self-check that prints the pyAgxArm version and the number of connected
   RealSense devices.

When it finishes you should see `=== PYTHON ENV READY ===` followed by
`all imports OK`. Activate the env with `conda activate piper_teleop` before
calling the CLI commands or the runtime/test `make` targets. Alternatively,
override the interpreter explicitly, for example `make test PY=/path/to/python`.

## Arm and CAN

The Piper connects over a USB-CAN adapter (gs_usb / candleLight) at 1 Mbps.
Bring the interface up with:

```bash
make can
```

This runs `install/03_can_setup.sh` (needs sudo), which configures `can0` with
`bitrate 1000000` and brings the link up. The script is idempotent — if `can0`
is already up at 1 Mbps it prints `can0 already up at 1 Mbps` and exits. If the
interface does not exist at all, it errors and asks you to plug in the USB-CAN
adapter.

!!! warning "Re-run after replug or reboot"
    The CAN link configuration does not persist. Run `make can` again after
    you replug the USB-CAN adapter or reboot the host.

Verify the link:

```bash
ip -details link show can0
```

The output must show `bitrate 1000000`. An `enable arm` timeout during teleop
almost always means the arm is powered off or this link is down — see
[Troubleshooting](../troubleshooting.md).

## Quest 3

Skip this section if you use a SpaceMouse instead. For day-to-day usage and
controls, see the [Quest guide](../guide/quest.md).

### 1. Enable developer mode

The headset must have developer mode enabled so the host can sideload and talk
to the app over USB debugging. Follow Meta's official developer documentation
to enable it for your headset.

### 2. USB debugging permissions (one-time, sudo)

```bash
bash install/05_quest_udev.sh
```

This writes a udev rule for Meta/Oculus USB devices (vendor ID 2833), restarts
the adb server, and lists connected devices. If your headset shows as
`unauthorized`, put it on, accept the **Allow USB debugging** dialog, and run
`adb devices` again.

### 3. Android toolchain (one-time, ~2.5 GB, APK build only)

```bash
make toolchain
```

This runs `install/01_android_toolchain.sh`, which installs JDK 17 (in a
separate `jdk17` conda env), the Android cmdline-tools, SDK platform 32,
build-tools 34.0.0, NDK 26.3.11579264, CMake 3.22.1, and Gradle 8.7 — all
under `$HOME/android-sdk`, no sudo required. It downloads roughly 2.5 GB.

!!! note
    You only need the toolchain to **build** the APK. It is not used at
    runtime.

### 4. Build and connect

```bash
make apk       # build the Quest APK
make connect   # install/start the APK + adb forward (run after every replug)
```

`make connect` installs and starts the app on the headset, sets up the
`adb forward` port forwarding for the wired link, and disables the proximity
sensor. Run it every time you plug the Quest back in.

## SpaceMouse

```bash
make spacemouse
```

This runs `install/04_spacemouse_setup.sh` (needs sudo, one-time). It installs
a udev rule granting non-root access to 3Dconnexion devices, reloads udev, and
then runs a 5-second read test — deflect the puck and press buttons and you
should see live axis values and an update rate printed.

!!! tip "Replug after installing the rule"
    If the read test fails right after installing the udev rule, unplug and
    replug the SpaceMouse, then run `make spacemouse` again. The driver reads
    HID directly (no spacenavd needed); if spacenavd is installed and grabs
    the device, stop it with `sudo systemctl stop spacenavd`.

## Cameras

Nothing to install: `pyrealsense2` is part of the dependencies from `make env`.
The default config auto-selects connected cameras. For multi-camera rigs, pin
your serial numbers to `cam_front` / `cam_wrist` / `cam_back` in a local config
overlay — see the [Configuration reference](../reference/configuration.md).

## Verify the installation

```bash
make test
```

This runs the full unit/integration suite with an isolated `PYTHONPATH` and
pytest plugin autoloading disabled, so a sourced ROS environment or external
pytest plugins cannot interfere. Success looks like pytest reporting all
tests `passed`. If pytest fails with errors from external plugins, make sure you
ran it via `make test` rather than calling pytest directly.

## Next steps

Power on the arm, run `make home` to unfold it from the factory folded pose,
and continue with [Your first teleop session](first-teleop.md).
