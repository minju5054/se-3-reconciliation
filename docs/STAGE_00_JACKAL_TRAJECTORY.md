# Stage 0: Isaac Sim Jackal Trajectory Smoke Test

## Purpose and scope

Stage 0 validates the simulation plumbing required before LightNav integration:

1. arbitrary `N x 3` SE(2) `[x, y, yaw]` representation;
2. four-wheel Clearpath Jackal execution in Isaac Sim 6.0.1;
3. world-pose reduction from 3D position/quaternion to world SE(2);
4. explicit, comparable reference/actual sampling and durable recording;
5. simultaneous 3D viewport visualization of both trajectories.

This is simulation pipeline validation, not research evidence. LightNav, OLD/NEW chunks,
correspondences, optimization, GTSAM, ROS control, Nav2, MPC, and obstacle avoidance are not
used. The Jackal is the execution platform; it is not the trajectory generator.

## Runtime and asset isolation

The demo must run through Isaac Sim's Python runtime:

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_jackal_trajectory_demo.sh
```

The launcher defaults to `~/isaacsim/python.sh`. Before launching it, the child environment
only unsets inherited `LD_LIBRARY_PATH`, `PYTHONPATH`, CUDA, ROS, AMENT, CMake, colcon, and
RMW variables known to cause library collisions. It does not modify the parent shell or
system configuration. Isaac's launcher then establishes its own paths, which are not
subsequently removed.

GUI mode is the default. After data are saved, the application remains open for visual
inspection until the user closes the window. `--no-hold` is available for an automated GUI
smoke run; `--headless` exists only as an explicit diagnostic option and is never the default.

The code obtains the asset root from `get_assets_root_path()` and resolves the configured
official relative asset path `Clearpath/Jackal/jackal.usd` beneath `/Isaac/Robots`. No asset
root URL or machine-specific absolute asset path is hardcoded.

## Reference trajectory generation

`configs/stage0_jackal_trajectory.yaml` defines `physics_dt`, `sample_dt`, initial SE(2)
pose, and a sequence of constant-command segments. The default deterministic sequence is:

1. initial stop;
2. straight motion;
3. gentle left turn;
4. straight motion;
5. final stop.

For each sample interval, pure research-side Python applies Euler unicycle kinematics:

```text
x[k+1]   = x[k] + v[k] cos(yaw[k]) sample_dt
y[k+1]   = y[k] + v[k] sin(yaw[k]) sample_dt
yaw[k+1] = wrap(yaw[k] + omega[k] sample_dt)
```

Segment durations must be integral multiples of `sample_dt`. This deliberate restriction
prevents hidden partial steps or interpolation. No horizon such as 10 or 24 is assumed.
The final reference row always has a zero command.

This reference is a deterministic smoke-test path, not VLA output.

## SE(2), frame, and timing conventions

All saved poses have shape `N x 3` or `M x 3` and order `[x, y, yaw]`:

- `x`, `y`: metres in the Isaac Sim world frame;
- `yaw`: radians about world `+Z`, counter-clockwise positive, wrapped to `[-pi, pi)`.

Isaac returns a 3D world position and scalar-first quaternion `(w, x, y, z)`. The runtime
keeps world X/Y and extracts world-Z yaw using the quaternion yaw formula. No coordinate
swap, projection into a robot-local frame, or silent transform is performed.

After a configured settling period, execution-relative time is set to zero. Reference pose
`k` and actual pose `k` are paired at `k * sample_dt`. Command `k` is applied for exactly
`sample_dt / physics_dt` physics steps and advances the system to sample `k+1`. Actual
physics time is recorded from Isaac and checked against the reference time grid. No
interpolation is performed; the writer and validator reject mismatched sample counts.

## Jackal articulation and wheel commands

After loading, the runtime traverses the referenced USD for exactly one
`UsdPhysics.ArticulationRootAPI` prim, initializes `SingleArticulation`, and prints the real
articulation path, DOF count, and DOF names. It requires the official asset to report four
unique revolute DOFs.

Wheel names are not encoded in the controller. For each runtime DOF, the code resolves its
revolute-joint prim, reads the child wheel body and the joint's lateral `localPos0`, and
classifies positive Y as left and negative Y as right. It requires two wheels per side.
Wheel radius is read from each wheel body's USD collision-cylinder radius and cross-checked
across all four wheels. Wheel separation is the difference between mean left and right joint
lateral positions. Both values and their sources are saved in metadata.

The differential/skid-steer command relation is:

```text
left_wheel_rad_s  = (v - omega * wheel_separation / 2) / wheel_radius
right_wheel_rad_s = (v + omega * wheel_separation / 2) / wheel_radius
```

The resulting left/right targets are mapped by discovered DOF index to both front and rear
wheels, then applied as an `ArticulationAction` velocity target. This is open-loop smoke
execution, not MPC or trajectory tracking.

## Actual-pose recording and visualization

At every sample boundary, the runtime reads the Jackal articulation world pose and appends
canonical `[world_x, world_y, world_yaw]` to `actual_trajectory` together with the same
execution-relative simulation timestamp.

The `isaacsim.util.debug_draw` interface renders visualization-only geometry:

- reference: connected line, decimated waypoint points, and prominent endpoints;
- actual: a connected history line that grows after every sample;
- start/end points: larger markers, supplemented by explicit `REFERENCE START/END` and
  `ACTUAL START/END` console logs so interpretation does not depend on color alone.

Reference and actual use slightly different configurable Z offsets above the ground to
avoid z-fighting. DebugDraw geometry has no collision and no effect on physics. The initial
viewport camera eye/target are configurable and frame the default trajectory.

## Data output and validation

Every run creates a new immutable directory:

```text
data/stage0/jackal_trajectory/<run_id>/
├── reference_trajectory.npy   # N x 3 [x, y, yaw]
├── actual_trajectory.npy      # M x 3 [x, y, yaw]
├── samples.csv
└── metadata.json
```

Existing run directories are never overwritten. `samples.csv` contains:

```text
sample_index,sim_time_s,ref_x,ref_y,ref_yaw,actual_x,actual_y,actual_yaw
```

Metadata records stage/run/time provenance, Git SHA, Isaac version, resolved asset details,
articulation prim, actual DOF names, timing, frame/yaw conventions, motion profile,
runtime-discovered wheel parameters/mapping, command equations, visualization height, and
sanity metrics clearly labeled as non-research results.

Validate a saved run independently of Isaac Sim:

```bash
.venv/bin/python scripts/validate_stage0_output.py \
  data/stage0/jackal_trajectory/<run_id>
```

The validator checks files, metadata schema, finite `N x 3` arrays, CSV schema/counts,
strictly increasing time, exact CSV/NPY pose equality, and any recorded motion smoke checks.
It never interpolates. Configured smoke thresholds require measurable displacement in at
least one commanded straight segment and measurable yaw change in a commanded turning
segment; these are pipeline checks, not tracking-quality claims.

## Success criteria

The smoke test passes when:

- GUI mode opens with a ground plane and official Jackal articulation;
- runtime reports and commands four wheel DOFs, with two wheels per side;
- actual X/Y displacement occurs during commanded motion;
- actual yaw changes during the turning segment;
- reference and accumulating actual paths are drawn together in the viewport;
- reference/actual NPY, CSV, and metadata files are produced;
- the independent output validator returns `valid: true`;
- the full research test suite, including existing EXP-01 tests, passes.

Perfect reference tracking is explicitly not required.

## Limitations and next step

- Control is open loop, and skid-steer lateral tire friction can make actual yaw differ
  substantially from the ideal unicycle reference.
- DebugDraw is transient viewport geometry and is not saved into the USD stage.
- Pose-estimation noise, controller quality, obstacle interaction, and navigation safety are
  outside Stage 0.
- A GUI run verifies the pipeline but is not an experimental result.

The next stage is to replace the deterministic generator input with recorded LightNav
trajectory chunks while preserving the same explicit SE(2), timing, frame, recording, and
visualization contracts. That future integration must not add graph reconciliation until it
is separately requested.
