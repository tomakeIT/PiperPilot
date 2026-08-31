# Benchmarks

This page lists the measured tracking performance of both control backends on
real hardware, explains exactly how the numbers were produced, and shows you
how to reproduce them on your own arm with `piper-track-test`.

## Measured results

All numbers below were measured on one Piper-X arm running firmware S-V1.8-9,
using the shipped default gains, with the benchmark trajectory built into
`piper-track-test` (a 3 cm circle at 5 s per lap).

| Metric | Position mode (`piper`) | Impedance mode (`piper_mit`, shipped soft gains) |
| --- | --- | --- |
| Static hold error | ≈ 0 mm | 11 mm |
| Dynamic RMS (3 cm circle, 5 s/lap) | 1.34 mm | 14.1 mm |
| Estimated latency | ~10 ms | ~90 ms |

Two related measurements for context:

- **Quest pose link:** the wired controller-pose stream runs at a measured
  90–92 Hz. Video panels do not affect it — they use a separate TCP port and
  drop frames rather than queue.
- **Contact force in impedance mode:** bounded at roughly the cartesian
  stiffness times the 0.12 m target tether (`max_cmd_deviation`), which works
  out to about 20 N with the shipped gains.

!!! note "Impedance numbers scale with kp"
    The impedance-mode figures are a property of the shipped *soft* gains, not
    of the controller itself. Raising `kp` tightens tracking (and raises the
    contact-force bound); lowering it makes the arm softer and the error
    larger. Position mode has no such trade-off because the firmware runs a
    stiff position servo. See [Control modes](control-modes.md) for the gain
    model and tuning advice.

## Method: what piper-track-test does

`piper-track-test` commands a small, slow circle around the arm's current pose
and compares the commanded end-effector position against the measured one. The
run has two phases:

1. **Static hold (3 s).** The tool repeatedly commands the starting pose and
   records the measured position. The *static hold error* is the distance
   between the two — effectively how far the arm sags or offsets when asked to
   stand still.
2. **Moving phase (12 s by default).** The tool traces a circle in the x–z
   plane around the starting pose. The circle radius is 3 cm by default
   (capped at 5 cm), one lap takes 5 s by default, and the amplitude ramps in
   over the first 2 s. Every commanded position is clamped to the teleop
   workspace box, and the z coordinate never goes below the starting height.

Commands and measurements are sampled at 100 Hz by default. The report skips
the first 2.5 s of the moving phase (the amplitude ramp) so the steady-state
numbers are not diluted by the gentle start. It then prints:

- static hold error (mean and max, mm),
- dynamic error (RMS and max, mm),
- per-axis RMS error (x, y, z, mm),
- estimated command-to-measurement latency, plus the residual RMS error after
  removing that pure delay.

**How latency is estimated:** the tool cross-correlates the commanded x
position against the measured x position over the steady-state window, testing
time shifts from 0 up to 0.4 s, and reports the shift with the highest
correlation. At the default 100 Hz sample rate this gives the estimate a
resolution of 10 ms. It also reports the dynamic RMS *after* shifting the
measurement back by that delay — useful for separating "the arm is late" from
"the arm takes a different path".

## Reproduce the numbers

Power on the arm, bring up the CAN link, and home the arm first (`make home`
after every power-on). Then:

=== "Impedance mode (default)"

    ```bash
    piper-track-test
    ```

    Uses the `piper_mit` MIT-impedance backend with your configured gains.
    The tool waits 2.5 s after connecting for the impedance entry and gain
    ramp to settle before it starts measuring.

=== "Position mode"

    ```bash
    piper-track-test --position
    ```

    Uses the `piper` backend (firmware position control via `move_p`).

=== "Simulation (no hardware)"

    ```bash
    piper-track-test --sim
    ```

    Runs the same trajectory against the `fake` backend. Useful for checking
    the tool itself, not for meaningful numbers.

You can vary the trajectory and sampling:

| Flag | Default | Meaning |
| --- | --- | --- |
| `--amp` | `0.03` | Circle radius in meters (clamped to 0.05 max) |
| `--period` | `5.0` | Seconds per revolution |
| `--duration` | `12.0` | Length of the moving phase in seconds |
| `--rate` | `100.0` | Command/sample rate in Hz |
| `--config` | – | Apply a config overlay (see [Configuration](configuration.md)) |
| `--impedance` | – | Force the `piper_mit` backend (the default backend) |
| `--position` | – | Force the `piper` backend (`move_p`) |
| `--sim` | – | Force the `fake` simulation backend |

The full flag reference lives in the [CLI reference](cli.md).

!!! tip "Compare apples to apples"
    If you tune `impedance.kp`/`kd` for your task, re-run `piper-track-test`
    with the same config overlay you teleoperate with. The benchmark uses the
    same `--config` mechanism as the other tools, so the numbers you measure
    describe exactly the controller your dataset will record.

## What the numbers mean for data collection

- **Position mode** (1.34 mm dynamic RMS, ~10 ms) is the right choice for
  high-precision free-space tasks where the arm should go exactly where you
  point and nothing will be touched along the way.
- **Impedance mode** (14.1 mm dynamic RMS, ~90 ms at soft gains) is the
  default for a reason: for contact-rich tasks the compliance and the bounded
  contact force matter far more than millimeter-level tracking, and the error
  shrinks if you raise `kp`.

The tracking error is between *commanded* and *measured* pose. Your datasets
record both (`action.arm_eef` and `observation.arm_eef`), so a policy trained
on this data sees the same controller behavior the benchmark measures — which
is also why the control backend and gains are written into each dataset's
provenance metadata. See [Control modes](control-modes.md) for choosing a
backend and [Dataset format](dataset-format.md) for what gets recorded.

!!! warning "Honest notes"
    - All headline numbers come from **one** Piper-X unit on firmware
      S-V1.8-9 with the shipped default gains. Your arm, firmware, gains, and
      payload will shift the results — treat the table as a reference point,
      not a spec sheet.
    - Latency is *estimated* by cross-correlating commanded vs. measured x
      position, with 10 ms resolution at the default 100 Hz rate. It is a
      whole-loop figure (command path + controller + mechanics), not a bus or
      network measurement.
    - The benchmark circles around whatever pose the arm holds when the tool
      starts, so a different starting pose can give different numbers —
      especially for the impedance-mode static hold error.
