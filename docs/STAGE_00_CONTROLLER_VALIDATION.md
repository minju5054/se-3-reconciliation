# Stage 0-B: Jackal Execution/Controller Validation

## Purpose and boundaries

Stage 0-B determines why the Stage 0-A ideal-unicycle command produced a large Jackal turn
error, then establishes an execution layer suitable for later LightNav trajectory playback.
It is simulator/controller validation, not research evidence. It does not run LightNav or
implement OLD/NEW chunks, correspondences, optimization, GTSAM, ROS 2 control, Nav2, MPC, or
obstacle avoidance.

The repository facts, observed controller-validation results, and future research method are
kept separate below. The official Jackal asset is the execution platform, not a trajectory
generator.

## Repository implementation

The input remains an arbitrary finite `N x 3` array in `[x, y, yaw]` order. Positions are
metres in the Isaac Sim world XY plane; yaw is counter-clockwise about world `+Z`, in radians,
wrapped to `[-pi, pi)`. Primitive and composite references are generated deterministically
from the configured constant `[v, omega]` profiles and the Stage 0-A Euler unicycle equation.

The Isaac runtime resolves `Clearpath/Jackal/jackal.usd` through the runtime asset API. It
discovers the articulation, four revolute wheel DOFs, wheel sides, radius, and lateral wheel
spacing from the loaded USD instead of guessing joint names. The official asset is never
overwritten and this validation applies no runtime drive, friction, damping, or effort
override.

The selected wheel command converter is Isaac Sim 6.0.1's non-deprecated experimental API:

```python
isaacsim.robot.experimental.wheeled_robots.controllers.DifferentialController
```

Its two outputs are mapped to the two left and two right runtime-discovered wheel DOFs. The
installed controller source uses the same equations as the original implementation:

```text
left  = (v - omega * wheel_separation / 2) / wheel_radius
right = (v + omega * wheel_separation / 2) / wheel_radius
```

`StanleyControl` was not selected because the installed implementation outputs bicycle-model
steering rather than left/right skid-steer wheel velocities. The installed
`WheelBasePoseController` is in a deprecated extension and performs point-to-point turn/drive,
not arbitrary SE(2) path tracking.

## Formula comparison

With the runtime-derived radius `0.0979999974 m` and separation `0.3755899966 m`, custom and
official outputs were identical in every required case (maximum absolute difference
`0 rad/s`):

| Case | `[v, omega]` | Custom `[left, right]` rad/s | Official `[left, right]` rad/s |
|---|---:|---:|---:|
| stop | `[0, 0]` | `[0, 0]` | `[0, 0]` |
| straight | `[0.5, 0]` | `[5.102041, 5.102041]` | `[5.102041, 5.102041]` |
| pure rotation | `[0, 0.25]` | `[-0.479069, 0.479069]` | `[-0.479069, 0.479069]` |
| arc | `[0.4, 0.25]` | `[3.602564, 4.560702]` | `[3.602564, 4.560702]` |
| opposite rotation | `[0, -0.25]` | `[0.479069, -0.479069]` | `[0.479069, -0.479069]` |

Therefore, **wheel conversion formula bug is not supported**. The official implementation is
still used downstream to avoid duplicating Isaac's wheel conversion API.

## Primitive diagnostic results

The final GUI diagnostic session was
`stage0b-official-primitives-20260903`. Each primitive used `0.5 s` stop, `4.0 s` active
motion, and `0.5 s` stop, sampled at `0.1 s`. These are validation observations, not research
results.

| Primitive | Desired body rate | Measured active mean | Wheel RMSE rad/s | Position RMSE m | Yaw RMSE rad |
|---|---:|---:|---:|---:|---:|
| straight | `v=0.3` | `v=0.29605` | `0.04842` | `0.01407` | `0.00072` |
| rotate left | `omega=0.25` | `omega=0.07607` | `0.29289` | `0.02545` | `0.42457` |
| rotate right | `omega=-0.25` | `omega=-0.03696` | `0.10103` | `0.02820` | `0.52153` |
| arc | `v=0.3, omega=0.2` | `v=0.29348, omega=0.00800` | `0.06885` | `0.22668` | `0.46727` |

The straight displacement was `1.19522 m` with negligible yaw drift. Left and right pure
rotations achieved only `0.30430 rad` and `-0.14785 rad`, respectively, instead of magnitude
`1.0 rad`. The arc's expected kinematic radius was `1.5 m`, while the measured effective
radius from mean body rates was `36.67637 m`.

During the arc, the aggregate wheel velocity error was small while desired yaw rate
`0.2 rad/s` produced only `0.00800 rad/s`. Pure rotation additionally exposed front-wheel
tracking loss and left/right asymmetry. These observations separate two effects:

1. wheel targets can track adequately for straight/arc translation;
2. ideal differential kinematics using physical lateral spacing does not predict the
   four-wheel skid-steer contact yaw response, and pure rotation also stresses wheel/contact
   tracking asymmetrically.

The most likely Stage 0-A root cause is therefore skid-steer tire/ground contact and the
ideal-unicycle/physical-track-width mismatch, not a sign or algebra error in wheel conversion.
No arbitrary effective-track calibration or asset-physics tuning was applied.

## Closed-loop follower

Because open-loop yaw execution is inadequate, `TrajectoryFollower` closes the loop around
the measured world SE(2) pose:

```text
reference N x 3 path -> monotonic nearest progress -> distance lookahead
-> position/heading feedback [v, omega] -> official DifferentialController
-> four runtime-discovered wheel targets -> Jackal -> measured world SE(2)
```

Progress never regresses and never advances solely because simulation time elapsed. The
nearest search is bounded to a configurable future window; lookahead is based on cumulative
path distance. Large heading errors cause rotate-in-place behavior, linear speed is reduced
by heading alignment, and the final pose gets explicit yaw alignment. This is a lightweight
feedback follower, not MPC or graph optimization.

The final GUI run `stage0b-closed-loop-accepted-20260903` reached the goal without exhausting
the `18 s` limit. Its immutable reference has shape `(101, 3)` and its actual history has
shape `(143, 3)`. Each actual row is paired with an explicitly stored monotonic nearest
`reference_index`; the code does not interpolate, resample, or force equal lengths.

| Metric | Stage 0-A open-loop baseline | Stage 0-B closed loop |
|---|---:|---:|
| position RMSE | not recorded | `0.05035 m` |
| final position error | `1.27615 m` | `0.07405 m` |
| yaw RMSE | not recorded | `0.05689 rad` |
| final yaw error | `0.69047 rad` | `0.07721 rad` |

All preliminary engineering criteria passed: final position `<0.20 m`, position RMSE
`<0.15 m`, final yaw `<0.10 rad`, and yaw RMSE `<0.10 rad`. This is an integration gate, not
a research claim or a general controller-performance guarantee.

## Telemetry, metrics, and output

Every actual sample records simulation time, commanded body `[v, omega]`, four canonical
wheel targets, four directly measured articulation joint velocities, actual world SE(2), and
finite-difference measured body speed/yaw rate. Each immutable run directory contains:

```text
reference_trajectory.npy
actual_trajectory.npy
samples.csv
telemetry.csv
metrics.json
metadata.json
```

`samples.csv` stores `reference_index` explicitly. Metrics include position RMSE/mean/max/final,
yaw RMSE/mean-absolute/max/final, desired-versus-measured body-rate RMSE, aggregate and per-DOF
wheel velocity RMSE, displacement, and yaw change. Session-level `summary.json` contains the
formula comparison and validated run rows. All generated data under
`data/stage0/controller_validation/` are ignored by Git.

The runtime inspection recorded force-type angular velocity drives with zero stiffness,
damping `1e7`, unbounded max force/max joint velocity, wheel mass about `0.477 kg`, and wheel
physics material static/dynamic friction `0.2`. Non-finite USD limits are serialized as
strings so metadata remains strict JSON.

## Visualization and commands

DebugDraw shows the full reference, live accumulated actual history, reference heading
markers, and endpoint markers above the ground. This geometry is visual only. GUI is the
default and remains open after completion unless `--no-hold` is supplied.

Run all open-loop primitives, then the closed-loop composite:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_jackal_controller_validation.sh
./scripts/isaac/run_jackal_controller_validation.sh \
  --controller closed_loop --scenarios composite
```

Validate and compare saved sessions without Isaac Sim:

```bash
.venv/bin/python scripts/summarize_controller_validation.py \
  data/stage0/controller_validation/<primitive_session_id> \
  data/stage0/controller_validation/<closed_loop_session_id>
```

Click either NPY file in VS Code to use the existing trajectory graph custom editor, or use
`scripts/view_trajectory_npy.py`. The viewer supports unequal reference/actual lengths and
does not interpolate them.

## Limitations and next step

- The follower was validated on a flat default ground plane and one deterministic composite
  path, without obstacles or disturbances.
- Desired-versus-measured angular velocity remains substantially different; pose feedback
  compensates over time but does not make skid-steer dynamics ideal.
- The engineering thresholds do not establish safety, generality, or research efficacy.
- DebugDraw is transient and the generated validation recordings are local-only.

The execution layer is sufficiently stable to begin the next, separately scoped step:
recorded LightNav `N x 3` trajectory integration with explicit frame/timestamp semantics.
Reconciliation factors and optimization remain unimplemented.
