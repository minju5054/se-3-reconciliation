# SE(3) Reconciliation Research

This repository studies how to reconcile successive navigation action chunks while
preserving the intent of a newly predicted trajectory. The final target is a local SE(2)
back end that uses OLD/NEW segment correspondences and relative-transform measurements to
distribute corrections over the editable future.

The platform roles are deliberately separate:

- **Isaac Sim 6.0.1** is the simulator.
- **ROS 2 Jazzy** is the robotics middleware and uses the machine's system Python 3.12.
- **LightNav-0** is the upstream navigation-VLA baseline that generates trajectory chunks.
- The reconciliation method developed here is **not LightNav**.

The current research-experiment scope remains **EXP-01 only**: characterize the discontinuity
produced by a naive raw OLD→NEW switch under asynchronous LightNav inference. There is no
interpolation, smoothing, correspondence factor, GTSAM dependency, or graph optimization in
this stage.

Before connecting LightNav, **Stage 0** validates the standalone Isaac Sim trajectory
pipeline with the official Clearpath Jackal asset. It generates a deterministic SE(2)
reference from configured unicycle commands, executes those commands through all four
runtime-discovered wheel joints, records the actual world SE(2) trajectory, and displays
both paths with non-physical DebugDraw geometry. Stage 0 is simulation pipeline validation,
not research evidence.

## Local layout and environments

Keep the repositories separate:

```text
~/Workspace/
├── se-3-reconciliation/       # this repository, Python 3.12 .venv
└── external/
    └── LightNav-0/            # upstream checkout, Python 3.11 .venv
```

The environments do not replace `/usr/bin/python3` and do not modify ROS 2, the NVIDIA
driver, CUDA, or Isaac Sim. Reproduce the research environment with:

```bash
cd ~/Workspace/se-3-reconciliation
uv venv --python /usr/bin/python3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

LightNav must be installed in its own checkout according to its upstream instructions; do
not install it here. The setup used for EXP-01 is recorded in `docs/WORK_LOG.md`.

## Stage 0 Jackal GUI smoke test

Run the default GUI workflow with one command:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_jackal_trajectory_demo.sh
```

The launcher uses `~/isaacsim/python.sh`, not the research `.venv`. It scrubs inherited
ROS/CUDA library environment variables only for that child process, then lets the Isaac
launcher establish its own runtime paths. The GUI stays open after recording so the ground,
Jackal, reference path, and accumulated actual path can be inspected together; close the
window to exit. Outputs are written to an immutable ignored directory below
`data/stage0/jackal_trajectory/<run_id>/`.

Validate any saved run from the research environment:

```bash
.venv/bin/python scripts/validate_stage0_output.py \
  data/stage0/jackal_trajectory/<run_id>
```

Configuration is in `configs/stage0_jackal_trajectory.yaml`; the full contract and observed
limitations are documented in [Stage 0](docs/STAGE_00_JACKAL_TRAJECTORY.md).

## EXP-01 data workflow

Never overwrite a raw VLA recording. Store uncommitted inputs below `data/exp01/raw/`, and
write derived chunk JSON below `data/exp01/derived/`. A raw waypoint file is an arbitrary
`N x 3` `.npy` array or JSON array in `[x, y, yaw]` order. It needs a separate metadata JSON:

```json
{
  "observation_time": 123.0,
  "ready_time": 123.43,
  "inference_latency": 0.43,
  "waypoint_dt": 0.25,
  "robot_pose_at_observation": [1.0, 2.0, 0.1],
  "frame": "robot_local_at_observation",
  "source": "LightNav-0:<checkpoint-or-run-id>"
}
```

The `waypoint_dt` value comes from the collection/controller configuration; LightNav rows
have no intrinsic time base. Build validated derived OLD and NEW chunks separately:

```bash
.venv/bin/python scripts/exp01_build_chunks.py \
  --poses data/exp01/raw/old_poses.npy \
  --metadata data/exp01/raw/old_metadata.json \
  --output data/exp01/derived/old_chunk.json
```

Then run the raw-switch analysis:

```bash
.venv/bin/python scripts/exp01_analyze_raw_switch.py \
  --old data/exp01/derived/old_chunk.json \
  --new data/exp01/derived/new_chunk.json \
  --output results/exp01/raw_switch_metrics.json \
  --plot results/exp01/raw_switch.png
```

The exact same command can be smoke-tested with
`tests/fixtures/exp01_synthetic/{old_chunk,new_chunk}.json`. That fixture is synthetic test
data and is **not research evidence**.

## Conventions

A LightNav-local waypoint is transformed only with the robot pose captured at that chunk's
observation time:

```text
T_world_waypoint = T_world_robot_at_observation * T_robot_waypoint
```

NEW is never transformed with the robot pose at NEW ready time. Waypoint row `i` is assigned
`observation_time + (i + 1) * waypoint_dt`; rows strictly earlier than `ready_time` are stale,
while a row exactly at `ready_time` is usable. See [EXP-01](docs/EXPERIMENT_01.md) for the
complete protocol and limitations.

## Tests

```bash
.venv/bin/python -m pytest
```

The suite covers SE(2) identity and inverse/compose consistency, angle wrapping, local-to-world
transforms, arbitrary horizons, timing boundary cases, I/O validation, and transition metrics.
