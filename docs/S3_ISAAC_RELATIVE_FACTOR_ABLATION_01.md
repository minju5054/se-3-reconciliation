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

**TECHNICAL_BLOCKED. No actual-Isaac rollout occurred.** Both graph solves also failed convergence. The reported final iterates are invalid planning candidates, not executable references.

Scientific freeze `0f56a781177880d148383c3120491a3c991234f5` was normally pushed before either solve. Source and code hashes still match the freeze. RAW parity is **NOT_RUN**, not PASS or FAIL.

### Infrastructure blocker and accounting

`scripts/isaac/s3_relative_ablation01.py` imported `prime_s3_clock` through the new graph/metric module. Its transitive `boundary_row_ablation04` import required `shapely`, unavailable in the Isaac Python startup environment. The worker failed before importing/constructing SimulationApp. The source-specific zero-solve preflight covered official controller installation; it did not catch this Isaac-side import dependency.

One launcher attempt; **zero SimulationApp constructions, zero RAW/B_ENTRY/FULL_GRAPH/GRAPH_NO_R rollouts, zero MPC solves**. FULL and NO_R each consumed their one allowed planning solve. LightNav, RGB/model requests, acquisition, Hermite and retries all0. The raw driver field `Isaac_launches=1` denotes its launch attempt; the marker-based validator and `call_accounting.json` distinguish the actual SimulationApp count0. See `technical_failure.json` for the preserved traceback.

The intended execution comparison was not completed. This follows the frozen technical-stop branch. No post-outcome scientific code/config change, fallback, tuning, environment modification, repeated solve or repeated launch. README remains unchanged because no execution mechanism result was established.

### Planning outcomes

| Method | Calls | Iterations | Converged | Termination | Executable reference |
|---|---:|---:|---|---|---|
| FULL_GRAPH | 1 | 80 | false | maximum_iterations | None |
| GRAPH_NO_R | 1 | 80 | false | maximum_iterations | None |

Factor costs at the final **nonconverged** candidate; these objectives contain different blocks and cannot be compared as execution scores.

| Candidate | T | Optimized R | A | Optimized total | R diagnostic |
|---|---:|---:|---:|---:|---:|
| FULL_GRAPH | 3.46874275276e-08 | 2.78742433608 | 0.453145401249 | 3.24056977202 | 2.78742433608 |
| GRAPH_NO_R | 8.32372101936e-12 | absent | 0.000211715701951 | 0.000211715710274 | 10.5415636744 |

NO_R R is **diagnostic relative-edge distortion, not optimized cost**. Initial FULL T/R/A = 0.48157187939767576 / 3.3628749022934885 / 2.7941456526192088. Initial NO_R has identical T/A and no R. Initial arrays are byte-identical.

### Deformation of invalid planning candidates

| Diagnostic | FULL_GRAPH | GRAPH_NO_R |
|---|---:|---:|
| Node translation RMS [m] | 0.113672237619 | 0.0923534423031 |
| Node translation max [m] | 0.184730185155 | 0.184730576196 |
| Node yaw RMS [rad] | 0.168529362673 | 0.189749380267 |
| Node yaw max [rad] | 0.385685031403 | 0.385685031403 |
| Endpoint displacement [m] | 0 | 0 |
| Endpoint yaw correction [rad] | 0 | 0 |
| First edge direction [rad] | -0.712288542776 | -0.712336546449 |
| First edge length [m] | 4.46275091758e-06 | 5.74882132922e-06 |
| Total XY arc [m] | 0.871992853999 | 0.898162650366 |
| Minimum edge length [m] | 4.46275091758e-06 | 5.74882132922e-06 |
| Relative edge translation RMS [m] | 0.0459376116746 | 0.0923036587906 |
| Relative edge translation max [m] | 0.0746960839447 | 0.244178101387 |
| Relative edge yaw RMS [rad] | 0.075509862299 | 0.141138888418 |
| Relative edge yaw max [rad] | 0.145948152805 | 0.373209958854 |

Per-node corrections (world XY metres and absolute wrapped yaw radians):

| Node | FULL XY | NO_R XY | FULL yaw | NO_R yaw |
|---:|---:|---:|---:|---:|
| 0 | 0.184683189977 | 0.184683189977 | 0.385685031403 | 0.385685031403 |
| 1 | 0.184730185155 | 0.184730576196 | 0.239736878598 | 0.373209961075 |
| 2 | 0.147535456978 | 3.5527136788e-15 | 0.129884888841 | 2.22101759206e-09 |
| 3 | 0.0985436105872 | 3.5527136788e-15 | 0.0590299203313 | 0 |
| 4 | 0.0545032654431 | 0 | 0.0233661907037 | 0 |
| 5 | 0.0247435812871 | 0 | 0.00899656341982 | 0 |
| 6 | 0.00880305983624 | 0 | 0.00298686983657 | 0 |
| 7 | 0 | 0 | 0 | 0 |

Both final candidates have first edges of only a few micrometres. The frozen numerical noncollapse threshold was 1e-12m; no stronger threshold was introduced afterward. Both nevertheless fail the mandatory convergence requirement, so neither was returned as a valid reference.

### Primary and secondary execution results

| Method | Position AUC .30 [m s] | Yaw AUC .30 [rad s] | T_turn50 .30 | T_turn50 .90 | Position/yaw AUC .90 |
|---|---|---|---|---|---|
| RAW | N/A | N/A | NOT_RUN | NOT_RUN | N/A |
| B_ENTRY | N/A | N/A | NOT_RUN | NOT_RUN | N/A |
| FULL_GRAPH | N/A | N/A | NOT_RUN | NOT_RUN | N/A |
| GRAPH_NO_R | N/A | N/A | NOT_RUN | NOT_RUN | N/A |

All full-horizon metrics, first method-specific command, TV, max v/omega, endpoint dwell and endpoint error are N/A. No time was substituted with a cap. Signed FULL−RAW, FULL−B_ENTRY, NO_R−FULL and NO_R−B_ENTRY differences are all null; there is no latency ordering or schedule-comparable execution.

### Safety

| Method | Reference edge clearance [m] | Reference status | Execution clearance / abort / overlap |
|---|---:|---|---|
| RAW | 0.131349284934 | REFERENCE_SAFE | N/A — not executed |
| B_ENTRY | 0.21148985508 | REFERENCE_SAFE | N/A — not executed |
| FULL_GRAPH | N/A | GRAPH_PLANNING_FAILED | N/A — not executed |
| GRAPH_NO_R | N/A | GRAPH_PLANNING_FAILED | N/A — not executed |

RAW/B_ENTRY reference safety was authenticated/preflight-checked. Neither graph has an accepted final reference; do not describe its execution as safe or unsafe. No unsafe command was applied because no execution began.

### Research interpretation

**This run does not establish that R limits useful transition correction.** It shows different nonconverged candidate deformations under the frozen objectives. NO_R has larger relative-edge distortion concentrated near the beginning, but no valid solution or robot response comparison. Lower NO_R cost cannot demonstrate tracking improvement. Planning failure is reported independently of the higher-precedence infrastructure blocker. No weak-R variant or additional experiment was performed.

### Figures and saved-only validation

[World/reference view](../results/s3_isaac_relative_factor_ablation_01/figures/s3_isaac_execution_comparison.png) and [response/deformation view](../results/s3_isaac_relative_factor_ablation_01/figures/s3_response_and_deformation.png). Exactly two final PNGs, inspected with numeric/hash sidecars. Execution panels explicitly show N/A; the world figure contains historical OLD-to-B and prepared reference geometry only. Its frozen 3s execution panel title describes the planned window, not an observed rollout. The deformation panel labels both candidates invalid.

Saved-only validator and PNG checker pass, with zero new scientific calls. Raw logs, solver traces, rejected/accepted proposal checks and nonconverged candidates remain in ignored data under the authenticated result seal. The frozen renderer has minor footer/axis crowding in the no-execution layout; it was preserved under the no-post-outcome-code-change rule.

Final post-execution relevant regression: **236 passed in154.51s**. Saved-only validation, figure checker, source/code hashes, compileall, diff and staged-diff checks pass. Exact regression command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/s3-mpl .venv/bin/python -m pytest -q tests/test_s3_isaac_relative_factor_ablation01.py tests/test_successive_isaac_four_method01.py tests/test_successive_handoff_severity01.py tests/test_successive_isaac_replay01.py tests/test_b_to_entry_boundary_row_ablation04.py tests/test_spatial_entry_suffix_execution01.py tests/test_spatial_correspondence_selector_diag01.py tests/test_se2*.py tests/test_canonical_se2_graph.py
```

## Limitations and claim boundary

One historical development source/handoff, fixed controller/source, controlled
actual-Isaac logical Xform execution. No wheel dynamics or online latency result.
Reference optimization does not establish execution improvement; tracking does
not establish navigation task success. No population, real-world, obstacle
avoidance, universal graph/R-factor, VLA or LightNav improvement claim. Do not
proceed to another source, weak-R tuning or new environment after this experiment.
