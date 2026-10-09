# S3 Isaac relative-factor ablation 01

## Frozen protocol

This is one historical DEVELOPMENT handoff, S3
`episode_008_repeat_01/handoff_023`, restored in actual Isaac Sim with a logical
SE(2) USD Xform. Historical S3 trajectories are timing-controlled **offline**
rollouts. They are authentication/parity inputs, not prior Isaac measurements.
The new comparison uses `SimulationApp`, `World.reset`, exact pose write/readback,
and `World.step(render=False)`. No wheel or actuator dynamics are introduced.

Starting main / fetched origin/main:
`dc400b15f32b38251a14d5226e225ea0e6cbb84d`.
The historical boundary result is authenticated against commit
`4e26e547cbda3cbfee8dc842a40091388cde7543` and SHA256
`ed1309b6da2532b51dcdc4b57da9cf0ff1fa228e5a1ab54a5e026c5218405ee7`.
Its source chain includes C3 Entry-Suffix, B-to-entry Bridge, Vector Bridge V2,
and Boundary Row Ablation. The saved-only historical validator verifies the
underlying correspondence/transport/multisource/R00/R01 chains as well.

All source paths and hashes come from those authenticated manifests. No source
search, new acquisition, LightNav request, RGB read, instruction change or
historical method rerun is permitted. Unrelated stage0 configs and the GPU
plotting script remain untouched.

### State, frames and scene

All scientific numbers below are loaded from saved records, not reconstructed
from the rounded prose. Machine-readable copies are in the result namespace.

- A = `[17.46420298487384, 32.23488093333458, -0.9543308190733302]`.
- P = `[17.824839307518847, 31.823914057090693, -0.7146940940889608]`.
- B = `[17.828542610626915, 31.820715881084176, -0.7099805094359564]`.
- B tick 1804; B clock 30.10000156983733 s; dt 0.01666666753590107 s.
- Physical u_minus = `[0.2935884395558631, 0.282815064430321]`.
- Applied u_B_plus = controller previous_control =
  `[0.49358844870431834, 0.7828150725091488]`.
- Generation 326, reference version 24, FRESH chunk_024.
- C3 arc 0.46193137914572885 m; fraction 0.3662231723212672;
  segment 2, beta 0.9725845948085713.
- E* = `[17.89727685765006, 31.99213199452277, -0.3242954780329643]`.
- Retained suffix has eight rows. Seven downstream rows retain their original
  Native-installed world/raw identities before graph deformation.

World: XY metres, yaw radians CCW, +Z up. Observation-local: +x forward,
+y left. Original FRESH is `F=A*raw`; raw arrays remain immutable. A, readiness
and B/switch clocks remain distinct and are copied in source authentication.
There are no intrinsic waypoint timestamps. Derived local adapter arrays use
`A^-1*X` for representation only; they are never anchored at B.

The original saved world array equals `A*raw` bit-for-bit. The historical Native
installation has a maximum 2.7755575615628914e-17 rad yaw representation difference
(same XY). Preserve both arrays: original saved world is the full primary metric
target; C3 and B_ENTRY use exact historical Native-installed rows. RAW installation
must equal historical Native installation byte-for-byte. Existing adapter
roundtrip tolerance is 1e-12, not a new scientific parity tolerance.

S3 has **no added cart**. Restore the authenticated original Hospital scene/config,
not the recent successive-source added-cart scene. Compare every nonanonymous USD
layer's composed-text hash with the saved export; verify visible Hospital root
and exact identity world transform on reset. Existing scene constructor creates
a camera/render product but this experiment performs zero RGB reads/model calls.

### Controller and schedule

Use the unchanged official MPC source SHA256
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`,
checkout `c6f40e3220edbf7011e4f17eaf2c865416737d4d`.
The authenticated source supplies effective v limit **0.8 m/s**; no recent
0.4 m/s intervention is applied. H=5, nearest+1 selector, Q=[10,10,1], R=[.1,.1],
max omega=3 rad/s, acceleration limits 2 m/s² and 5 rad/s², controller rate10Hz
and internal MPC dt .1s remain unchanged. These controller internals do not
assign times to LightNav rows.

Copy the historical schedule bytes. The harness view only exposes its saved
lists: submit ticks 1806 through 1980 (30 saved submissions), releases 1807 through
1981. Finish at state1984 after180 intervals, 3.0000001564621925 s. The historical
54-interval window is 0.9000000469386578 s. No timing is regenerated from a nominal
formula. Official solve host time pauses the simulation; results release only at
saved ticks, with original generation/stale-result and previous_control semantics.

Each rollout uses a new controller, empty pending/future queues, exact restored
memory and held command. Each Isaac method resets World and the agent. Restoring
the saved clock requires1806 zero-motion physics steps from zero. The unchanged
`prime_clock` routine has a1000-step guard, so the S3 wrapper invokes bounded
1000+806-step batches, with total budget B_tick+2. These initialization steps are
excluded from execution metrics and never issue MPC commands. No fake clock is
assigned to satisfy a measurement.

### Methods and graph algebra

Order: RAW, B_ENTRY, FULL_GRAPH, GRAPH_NO_R. No Hermite execution.

- RAW: entire immutable original FRESH, using the exact original raw adapter rows.
- B_ENTRY: unchanged historical staging `[B,E*,original suffix]`, with frozen
  vertex/duplicate handling and downstream identity.
- Both graphs reuse current `ProgressGraph` (not the historical V2 bridge).
  Input S is the eight-row frozen suffix; X0=B and Xlast=Slast are fixed;
  only six internal poses are editable.
- Initialization is the current spatial taper
  `X_j=Exp((1-s_j)Log(B*S0^-1))*S_j`, then exact overwrite of B/end. Save both
  `.npy` hashes and require equality.

FULL retains current T/R/A residual arithmetic exactly. T is the wrapped world
first-edge direction minus actual P→B direction, divided by15deg. R is
`Log((S_j^-1*S_(j+1))^-1*(X_j^-1*X_(j+1))) / [.10m,.10m,10deg]`
for every suffix edge. A is
`s_j*Log(S_j^-1*X_j) / [.10m,.10m,10deg]`
for editable internal nodes; s is normalized suffix XY arc. All lambdas1.

NO_R removes the entire R block from the optimization vector/cost. It keeps T/A,
initialization, solver, fixed endpoints and safety identical. No substitute
smoothness, spacing, GP, obstacle, velocity, acceleration or other factor.
Both solutions receive the same post-solve relative-edge diagnostic; NO_R's R
number is explicitly **diagnostic relative-edge distortion, not optimized cost**.
Reference yaw/relative-pose deformation does not directly prescribe robot omega.

Use unchanged right-local `solve_least_squares` and SolverConfig:80iterations,
FD1e-6, damping1e-3/increase10/decrease.3/max1e12, gradient/step1e-9,
cost1e-12. Require convergence, finite states/residuals, edge lengths>1e-12,
exact endpoints and safety. Reject unsafe improving proposals. No last-iterate
fallback, repair, alternate initialization, tuning or retry.

### Safety and RAW gate

Full supplied reference (including B edges if B is a supplied row) must pass the
historical direct Hospital checker: circular footprint .20m, required edge
clearance .05m, reserve1e-7m, unchanged workspace/uncertainty conventions.
RAW has no invented B→first-row connector. Runtime remains abort-only: a failing
command is unapplied, its rollout retains the valid prefix, subsequent metrics
remain censored/null where coverage is incomplete. No fabricated continuation.

After the two planning solves, RAW executes first. Compare every historical
state/command, B, clock, submit/application, selected H5 rows, previous_control,
generation/reference identity, guard decision/clearance and full termination.
Freeze existing tolerances: pose/clock/command/memory/guard1e-10, selected reference
1e-12, rtol0; initial B exact. If RAW parity fails, execute no remaining method.
Scene/authentication/restoration/infrastructure failure is TECHNICAL_BLOCKED.
Invalid graph planning is retained explicitly and never executed as a fallback.

### Metrics, censoring and interpretation (predeclared)

Primary target: the **entire original FRESH**. Reuse forward-only continuous
projection and shortest-angle yaw. Reuse current AUC boundary interpolation.
Primary position/yaw AUC horizon .30s; secondary .90s; full3s descriptive.
Own modified references never replace this target.

T_turn50 uses the unchanged severity-audit function: psi_F is the tangent of the
original C3 containing segment, theta_B is actual B heading, delta=wrap(psi_F-theta_B).
For S3, psi_F=-0.38134998359916367 and delta=0.3286305258367927 rad.
`q(t)=sign(delta)*wrap(theta(t)-theta_B)/abs(delta)`.
The first saved sample q>=.5 must have a following saved sample also q>=.5, both
within the horizon. Near-zero <=1e-12rad is N/A; incomplete coverage is
INCOMPLETE_WINDOW; unreached full window is CENSORED_NOT_REACHED. No interpolated
latency or replacement with cap. Primary latency ordering uses .30s; report .90s
and full observations separately. Incomplete windows have no latency ordering.
One integration tick minus the existing1e-9 scalar equality tolerance is the
minimum discrete ordering difference; no practical-significance threshold.

Secondary: first new omega, linear/angular command TV, maxima, swept clearance,
abort/overlap, original-FRESH endpoint dwell (.10m/15deg for complete following
.30s sampled interval), endpoint error at the historical nominal3s cap.
Endpoint dwell is not navigation goal arrival or task completion.
Higher angular command change is not automatically worse.

Report FULL−RAW, FULL−B_ENTRY, NO_R−FULL and NO_R−B_ENTRY signed differences.
No scalar winner score. Precedence:

1. TECHNICAL_BLOCKED: authentication/restoration/infrastructure failure.
2. RAW_ISAAC_PARITY_FAIL.
3. GRAPH_PLANNING_FAILED: either variant fails frozen validity/convergence.
4. RELATIVE_FACTOR_LIMITING_EVIDENCE: safe NO_R has earlier primary T50 by a
   saved tick (or FULL censored) and smaller .30positionAUC by>1e-9.
5. FULL_RELATIVE_FACTOR_RESPONSE_SUPPORTED: safe FULL satisfies the reverse.
6. B_ENTRY_REMAINS_SUFFICIENT: neither graph improves both primary latency
   ordering and .30positionAUC relative to B_ENTRY.
7. RESPONSE_DEFORMATION_TRADEOFF: remaining improvement conflicts with yaw/safety;
   report deformation magnitudes without inventing a deformation cutoff.
8. MIXED_MECHANISM_EVIDENCE: remaining valid cases.

### Freeze and call budget

Before real planning/Isaac: authenticate chain, save exact references/initials,
source/config/schedule, zero-numerical-solve official installation preflight,
tests, review, freeze manifests/code hashes, commit and normal push. Commit title:
`Freeze S3 Isaac relative-factor ablation`.

Then exactly one FULL and one NO_R solve. Maximum one SimulationApp, one rollout
per eligible method, 30 MPC solves each if the full schedule completes (max120).
Exclusive start markers prevent repeat execution. LightNav/RGB/acquisition/
Hermite/retries0. A failed RAW gate leaves later methods unexecuted. No code or
config change after results. Saved-only validation is guarded against scientific
solves/runtime construction. Large traces remain in ignored
`data/s3_isaac_relative_factor_ablation_01/primary_20261009`.

Exactly two final PNGs:

- `results/s3_isaac_relative_factor_ablation_01/figures/s3_isaac_execution_comparison.png`
- `results/s3_isaac_relative_factor_ablation_01/figures/s3_response_and_deformation.png`

## Commands

From repository root; `.venv/bin/python` is the existing repository environment.
No system/external Python environment is changed.

```bash
git fetch origin main
git status --short
git branch --show-current
git rev-parse HEAD origin/main
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_boundary_row_ablation04.py --run data/b_to_entry_boundary_row_ablation_04/primary_20261004T000000Z --check-only
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_s3_isaac_relative_factor_ablation01.py --run data/s3_isaac_relative_factor_ablation_01/primary_20261009 --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/s3-mpl .venv/bin/python -m pytest -q tests/test_s3_isaac_relative_factor_ablation01.py
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_s3_isaac_relative_factor_ablation01.py --run data/s3_isaac_relative_factor_ablation_01/primary_20261009 --mode freeze
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_s3_isaac_relative_factor_ablation01.py --run data/s3_isaac_relative_factor_ablation_01/primary_20261009 --mode verify
.venv/bin/python -m compileall -q src scripts tests
git diff --check
git diff --cached --check
# After freeze commit and normal push only:
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/s3-mpl .venv/bin/python scripts/run_s3_isaac_relative_factor_ablation01.py --run data/s3_isaac_relative_factor_ablation_01/primary_20261009 --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_s3_isaac_relative_factor_ablation01.py --run data/s3_isaac_relative_factor_ablation_01/primary_20261009 --write
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/s3-mpl .venv/bin/python scripts/report_s3_isaac_relative_factor_ablation01.py --run data/s3_isaac_relative_factor_ablation_01/primary_20261009
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/s3-mpl .venv/bin/python scripts/report_s3_isaac_relative_factor_ablation01.py --run data/s3_isaac_relative_factor_ablation_01/primary_20261009 --check
```

The initial zero-solve prepare exposed a tuple/list JSON provenance comparison;
this was fixed before freeze, then only the unfinished restoration_preflight
stage was completed. No scientific start/solve/Isaac occurred. A historical
validator invocation without `--check-only` reached its existing exclusive-output
guard; no historical file was changed. Synthetic test solves are not evidence.

## Pre-freeze validation

Focused25 passed (11.45s); relevant regression1302 passed (416.98s). Historical
S3 validator, compileall and diff checks pass. The regression command was:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/s3-mpl .venv/bin/python -m pytest -q tests/test_s3_isaac_relative_factor_ablation01.py tests/test_successive*.py tests/test_long_continuous_obstacle_reveal_source01.py tests/test_continuous_obstacle*.py tests/test_online_mpc_adapter.py tests/test_osa03*.py tests/test_robotless*.py tests/test_join_online02*.py tests/test_se2*.py tests/test_b_to_entry*.py tests/test_spatial*.py tests/test_local_se2*.py tests/test_canonical_se2_graph.py tests/test_relative_factor_multisource01.py
```

## Repository-confirmed results

Pending pushed freeze and bounded execution. No S3 scientific solve or Isaac
launch has occurred at protocol preparation.

## Limitations and claim boundary

One historical development source/handoff, fixed controller/source, controlled
actual-Isaac logical Xform execution. No wheel dynamics or online latency result.
Reference optimization does not establish execution improvement; tracking does
not establish navigation task success. No population, real-world, obstacle
avoidance, universal graph/R-factor, VLA or LightNav improvement claim. Do not
proceed to another source, weak-R tuning or new environment after this experiment.
