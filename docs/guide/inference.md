# Running policies

`piper-infer` closes the loop: it feeds live observations from the robot to a
trained policy and executes the actions the policy returns. It is the same
runtime whether the "policy" is a real model on a GPU server, a recorded
episode used as a self-test, or a do-nothing hold.

In one sentence: every ~0.5 s the runtime sends the latest observation to the
policy, receives a chunk of absolute actions, and executes them at 30 Hz —
details in [How the loop works](#how-the-loop-works) below.

## The command

```bash
piper-infer --policy <spec>
```

`--policy` accepts three kinds of spec:

| Spec | What it runs |
|---|---|
| `hold` | Holds the current pose for every requested chunk. A safety no-op that smoke-tests the full loop without a model. |
| `replay:<root>[:<ep>]` | Feeds the recorded actions of episode `<ep>` (default 0) from dataset `<root>` chunk by chunk. End-to-end self-test with ground-truth actions — recommended first. |
| `http://host:port` | Queries a remote model server over HTTP (protocol below). |

Flags from `piper_teleop/apps/infer.py`:

| Flag | Default | Meaning |
|---|---|---|
| `--policy` | required | `hold` \| `replay:<root>[:<ep>]` \| `http://host:port` |
| `--task` | task from config | Language instruction included in each observation |
| `--config` | — | Config overlay file (see [Configuration](../reference/configuration.md)) |
| `--sim` | off | Simulated arm (`fake` backend), no hardware needed |
| `--position` | off | Firmware position-control backend instead of the default impedance backend |
| `--no-cameras` | off | Run without cameras (observations carry no images) |
| `--rate` | `30.0` | Execution rate in Hz (the dataset fps) |
| `--exec-horizon` | `15` | Steps executed per policy query (15 @ 30 Hz = 0.5 s) |
| `--horizon` | `30` | Steps requested per query |
| `--max-seconds` | `120.0` | Stop after this many seconds |

!!! note "Replay consumes whole chunks"
    With a `replay:` policy the runtime automatically sets
    `exec_horizon = horizon`, because a receding-horizon re-query would skip
    the tail of every replayed chunk. Real policies keep the
    `exec_horizon < horizon` overlap.

## How the loop works

The runtime follows the standard chunked-action VLA deployment pattern:

1. Every `--exec-horizon` steps (default 15 steps at 30 Hz, i.e. every
   ~0.5 s), it builds an **observation**: the latest frame from each camera
   plus the arm state (joints, end-effector pose, gripper width).
2. It sends the observation to the policy and asks for a **chunk** of
   `--horizon` actions (default 30).
3. It executes the chunk step by step at the dataset rate (30 Hz) via
   `command_eef` / `command_gripper`, then re-queries the policy before the
   chunk runs out (receding horizon: only the first `exec_horizon` of the 30
   returned steps are executed before fresh actions replace the rest).

Actions are **absolute** targets, 8 numbers per step:
`[pos x y z, quat w x y z, gripper width]` — exactly the
`action.arm_eef` + `action.gripper` columns of the recorded datasets (see
[Dataset format](../reference/dataset-format.md)).

## Validate the loop before training anything

The replay policy lets you prove the entire inference path — observation
building, chunking, timing, execution — against ground-truth actions before
any model exists. Run it in simulation first, then on hardware:

=== "Simulation"

    ```bash
    piper-infer --policy replay:~/piper_datasets/my_task:0 --sim --no-cameras
    ```

=== "Hardware"

    ```bash
    piper-infer --policy replay:~/piper_datasets/my_task:0
    ```

If the arm re-traces the demonstration, your deployment loop is correct and
any later problem is the model, not the plumbing. See
[Replaying episodes](replay.md) for the open-loop counterpart
(`piper-replay`).

!!! tip
    Power-cycled the arm? Home it first, as before teleop — see
    [Your first teleop session](../getting-started/first-teleop.md).

## Serving your own model

The runtime talks to your model through one framework-agnostic HTTP endpoint.
Implement it server-side, next to the model, in whatever stack you like.

**Request** — `POST <url>/predict` with a JSON body:

```json
{
  "task": "pick up the cube",
  "state": {
    "observation.arm_joint": [0.0, 0.4, -0.5, 0.0, 0.8, 0.0],
    "observation.arm_eef":   [0.30, 0.00, 0.20, 1.0, 0.0, 0.0, 0.0],
    "observation.gripper":   [0.04]
  },
  "images": {"cam_front": "<base64 jpeg>", "cam_wrist": "<base64 jpeg>"},
  "horizon": 30
}
```

- `state` keys match the dataset schema: `observation.arm_joint` (6, rad),
  `observation.arm_eef` (7 = position in m + quaternion **wxyz**, base
  frame), `observation.gripper` (1, width in m).
- `images` maps camera name to a base64-encoded JPEG (the client encodes at
  quality 85).

**Response** — a JSON object with an `actions` array of up to `horizon`
steps, each 8 floats, absolute, at the dataset rate:

```json
{"actions": [[0.30, 0.00, 0.20, 1.0, 0.0, 0.0, 0.0, 0.04], ...]}
```

Each step is `[x, y, z, qw, qx, qy, qz, grip]`. The client rejects anything
that is not an (H, 8) array.

!!! warning "Answer fast"
    The HTTP client uses a 2-second timeout, and every millisecond of query
    latency is time the arm spends without fresh actions (the runtime prints
    the per-query latency). Keep the server on a fast link to the robot host.

## The stdlib reference server

A minimal, dependency-free server that already speaks this protocol ships
with the package:

```bash
# on the GPU box
python -m piper_teleop.tools.example_policy_server --port 8901

# on the robot host
piper-infer --policy http://<gpu-host>:8901
```

Its `predict()` is a stub that holds the current pose for the requested
horizon — useful as a wire-format test. To serve a real model, replace
`predict()`: decode the base64 JPEGs, run your VLA, and return the (H, 8)
absolute-action chunk.

## Safety

Policy actions pass through the **same safety envelope as teleoperation**:

- Target positions are clamped to the configured workspace box before being
  sent to the arm.
- The arm backend applies its per-step clamps, and the loop aborts
  immediately on an arm fault.
- `--max-seconds` (default 120 s) bounds every run; ++ctrl+c++ stops it
  cleanly and hands the arm back in position hold.

!!! danger "Deploy with the training-time control mode"
    A policy is implicitly bound to the controller it was trained under.
    Every dataset records its control backend and gains in
    `meta/collection_meta.json` — deploy with the same control mode and gains
    (impedance vs. `--position`) that collected the data. See
    [Control modes](../reference/control-modes.md) and
    [Dataset format](../reference/dataset-format.md).
