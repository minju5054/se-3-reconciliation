# GP-SE2-01: environment-constrained OLD–FRESH handoff pilot

**Final operational status: GP_SE2_01_COMPLETED_WITH_LIMITATIONS. Research interpretation: NO_ADDITIONAL_BENEFIT_OBSERVED.** The fixed ten-case comparison completed: native/adapter/rigid local successes are 6/7/7; both GP methods found zero accepted candidates. All failures, 460 required plots and 15 actual Isaac captures are retained. A validator-only bookkeeping correction was independently revalidated without changing numerical results.

This is an offline counterfactual formulation pilot over preserved genuine online
LightNav handoffs. The question is whether switching from the actual pose and
motion state to FRESH can improve transition quality while preserving obstacle
clearance, the required local passage, and the original downstream goal. A small
FRESH deformation, zero boundary gap, or smoother stopping is insufficient.
Implementation completion and positive research evidence are separate outcomes.

## Implementation freeze and preserved source

Starting main: `ada33e145a412899ef5d33b05e9757fe4a780db7`, equal to fetched
`origin/main`. The source collector SHA is
`eed5f2c68f9bfc2b62d70a00a6d784647c32a971`. Primary input is
`data/robotless_online_handoffs_v1/primary_20260915T091900Z`; shape diagnostics are
`data/robotless_online_handoff_shape_analysis/primary_20260915T091900Z_v1`.
The source validator again passed all 881 events in 60 episodes and all 1,762
valid-event PNGs, with no errors. Rehashing the cleanup preservation inventory
matched all 36,684 retained data files. Source timing/history limitations remain.
Evidence is in `data/robotless_gp_se2_01/source_audit_20260918`.

The two unrelated Stage 0 configuration edits remain unmodified and uncommitted.
No new LightNav inference, observations, dataset, robot articulation, gain tuning,
external source modification, checkpoint modification, or model timing claim is
part of this pilot. Every derived run writes a new directory and refuses overwrite.
Code/config snapshots and hashes, selected raw inputs, frozen B/goal/gates,
candidates, per-start numerical diagnostics and all method outcomes are retained.

## Oracle environment and validation

The same configured Hospital 6.0 USD asset is loaded with native stage units,
full nested transforms, instance proxies and resolved references. The current
export contains 2,058 meshes, including 2,035 instance proxies. The evaluation
volume is a circular footprint of radius 0.20 m and ground-relative height band
[0.05, 0.65] m; these are experimental assumptions, not physical robot dimensions.
Clearance is centre-to-obstacle distance minus the radius. Required edge clearance
is 0.05 m; radius and margin are not applied twice.

Obstacle geometry uses height-clipped triangle projections and even-odd filled
lower-slab cross sections for solid interiors. Sibling material sections are
joined per actual asset; disconnected unresolved shells remain unknown. Floors
and overhead-only meshes are not indiscriminately projected as obstacles. Thin
surfaces remain; whole-wall AABBs do not replace door openings. Known evaluated
floor area is 3,245.396 m²; unresolved interior area of 5.471 m² is excluded.

The optimizer queries a 0.05 m distance grid with the conservative bilinear
Lipschitz error bound 0.03535534 m subtracted. The independent checker instead
uses direct projected geometry and swept circular footprints along each supplied
polyline. Its numerical uncertainty is 1e-7 m. Unknown space and borderline
clearance never count as a pass. GP evaluation samples at 0.005 s and refines to
0.00125 s when clearance is at most 0.10 m. Actual held-command rollout queries
include every 60 Hz boundary plus at most 0.005 s samples; the unicycle arc-to-chord
bound `max|v omega| dt²/8` is added to required clearance. This is a sampled
continuous-trajectory assessment, not a continuous-time safety proof.

Validated export:
`data/robotless_gp_se2_01/environment_technical_20260918_retry04`.
All 13 environment checks pass: free floor, wall/furniture overlap, real doorway,
unknown space, ground/overhead exclusion, transforms, independent distance/grid
checks, every gate and actual Isaac screenshots. Twenty-eight independent world
transform checks matched; 200 full-union distance checks differed by at most
3.10e-9 m. The largest observed error over 2,000 grid queries was 0.023728 m,
below the bound. All 293 non-anonymous USD layer hashes matched on actual reload.
Historical collection did not hash every layer: scene URL/configuration equality
is established, but historical byte identity of every remote asset is not proven.

Technical export attempts are preserved. Pre-primary fixes addressed JSON
serialization, an analytic horizontal GroundPlane, solid interiors missing from
surface-only projection, and overly connected unknown-shell sections that closed
a real doorway. They are implementation validation, not formulation outcomes.

## Cases, local goals and route proxy

Selection scans all 881 saved events without new optimizer or MPC outcomes.
Exact timing eligibility requires client-request-to-receipt matched-state RTF in
[0.8, 1.2], maximum enclosed loop stall at most 0.25 s, capture and actual
execution overlap, and recorded post-switch motion. Source integrity, valid B,
valid original FRESH endpoint, clearance-valid original future suffix and an
interpretable route relation are also required. Already colliding FRESH suffixes
remain in the diagnostic exclusion inventory.

Groups target three events each: A small mismatch/straight, B small mismatch/turn,
C large position or reliable direction mismatch, D shortcut/route sensitive.
The precise descriptive thresholds are versioned in `configs/gp_se2_01.yaml`.
Selection handles scarce groups in D, C, B, A order, then prefers an unused
episode, unused ordered raw pair, named priority, and lexical event order.
No event belongs to two selected groups. Missing coverage is reported rather
than filling the quota by relaxing conditions or replacing cases after outcomes.

The fixed goal is the last original FRESH pose, with 0.15 m and 15 degree
tolerances. The final 0.20 s must remain within both. Eight actual doorway
intervals and one independently verified broad corner-to-stairs passage form a
scene gate catalog. The latter is explicitly an open-corner passage proxy, not a
doorway: its safe-centre interval is 8.693 m wide, with 1 mm endpoint precision
accounted for, and full-interval swept clearance exceeds 0.05136 m. It is derived
from the building corner and actual stairs for episode_017_repeat_00/handoff_007;
the raw crossing clearance is 0.11552 m while the direct B-to-goal shortcut has
only 0.003889 m edge clearance. This is a clearance-sensitive case, not a claim
that the shortcut physically overlaps the footprint.

Remaining raw-suffix strict crossings establish ordered direction and fixed
crossing times. The solver constrains the gate plane at that time, the finite
safe-centre segment, and opposite signed sides at ±0.05 s (minimum progress
0.0001 m). Actual rollout crossings are checked independently. Reverse crossings,
grazing along the gate plane, and finite-segment endpoints do not pass. A reliable
clear shortcut with no remaining catalogued crossing needs no gate; a blocked
shortcut without a reliable gate is UNKNOWN and excluded. These are local
route-preservation proxies, not complete natural-language semantic ground truth.

## Reference preparation and methods

`F_native` is every original FRESH world row, unchanged. At B the official
weighted squared-pose distance `[10,10,1]`, wrapped yaw and next-row rule choose
a future suffix. The first row is assigned 0.1 s, the last 3.0 s, and intermediate
rows uniform row-order times. XY interpolation is linear, yaw shortest-angle;
rotation-only rows remain. A single row gives a constant future reference.
`F_common` contains exactly 30 samples at 0.1,...,3.0 s. No B connector is
inserted into either original or prepared reference. This is an evaluation
clock; intrinsic LightNav waypoint_dt remains null.

| Method | Reference and optimization |
|---|---|
| M0_NATIVE | Original full FRESH + official MPC |
| M0_ADAPTER | Common suffix and resampling only + official MPC |
| M1_RIGID | One left SE(2) transform of all common rows |
| M2_GP_NO_OBSTACLE | GP boundary/motion/workspace/goal/gates, no obstacle inequality |
| M3_GP_CONSTRAINED | Identical GP formulation plus obstacle inequality |

Rigid minimizes uniform FRESH preservation plus the finite-lookahead entry
residual against `B Exp(0.1 nu_minus)`. Goal, route, known workspace and obstacle
constraints apply without imposing an impossible hard initial boundary on a
three-parameter transform. Its undefined initial reference connector is assessed
through the actual MPC rollout.

## GP prior, interpolation and constraints

The prior follows the locally linear Lie-group construction of
[Dong et al. (2018)](https://dongjing3309.github.io/files/Dong18icra.pdf), with
[continuous-time GP planning background](https://www.roboticsproceedings.org/rss12/p01.pdf).
World agent poses use right-local perturbations `X <- X Exp(delta)` and body
velocity `nu = vee(X^-1 dX/dt)`. For `xi=Log(X_i^-1 X_j)` and interval h,

```
r = [xi - h nu_i; J_r(xi)^-1 nu_j - nu_i]
Q = [[h^3/3 Qc, h^2/2 Qc], [h^2/2 Qc, h Qc]]
J_GP = 0.5 sum ||Cholesky(Q)^-1 r||²
J = J_GP + mean ||Log(F_common^-1 X)||²_SigmaF
```

`Qc=diag(1,1,1)`, preservation standard deviations are `[0.1 m,0.1 m,10 deg]`,
and lambda is 1 uniformly over 30 future samples. These are design scales, not
learned uncertainty. The integrated-Wiener conditional local mean is evaluated
as cubic Hermite interpolation in local coordinates and their derivatives.
Body velocity is `J_r(xi) xi_dot`; acceleration includes the directional derivative
of J_r. The paper's printed Eq. 33 inverse is inconsistent with its Eq. 22;
the implemented forward-Jacobian relation is independently finite-difference
verified. It is not an exact global constant-body-acceleration or jerk guarantee.

Support states at 0,0.1,...,3.0 s fix `X(0)=B` and
`nu(0)=[v_minus,0,omega_minus]`. Support lateral velocity is structurally zero;
midpoint lateral velocity is an equality constraint, and dense intermediate
lateral velocity is independently checked. Limits match the resolved official
object-navigation MPC: v in [0,0.8] m/s, |omega|≤3 rad/s, |dv/dt|≤2 m/s²,
|domega/dt|≤5 rad/s². Support/midpoint checks include both sides of knot
acceleration discontinuities; final checks include denser samples and an
independent pose-derivative/body-velocity identity. Goal, ordered gates and known
workspace apply to both GP methods; only M3 adds obstacle clearance inequalities.

SciPy 1.18.1 SLSQP handles actual equality/inequality constraints. GP factors have
local sparse dependency, but SLSQP is a dense constrained backend, not the paper's
unconstrained sparse solver. Two deterministic initializations are shared by M2
and M3: FRESH support poses with finite-log initial velocities and a constant
boundary body-twist extrapolation. Both receive 200 iterations and 30 s each,
ftol 1e-7, equality/inequality acceptance tolerances 1e-5, seed 0 (no stochastic
search), single BLAS thread. Feasible initial/latest/intermediate iterates compete
by their own objective. A timeout may retain a checked feasible iterate but is
never called convergence. No feasible candidate means no MPC rollout for that
method; no RAW fallback or mathematical infeasibility claim follows.

Math tests cover Exp/Log, right retraction, J_r finite differences, covariance
SPD, zero-residual constant twists, endpoint/derivative identities, yaw wrapping,
chart boundaries, factor Jacobians and sparsity. A pre-primary SE(2) Exp/Log
small-angle series threshold correction from 1e-8 to 1e-4 avoids cancellation
near 4e-7 rad; independent matrix-exponential checks cover both sides. Synthetic
straight/curved solver qualification is retained separately and never treated as
dataset performance evidence.

## Common-state MPC execution and primary evaluation

The pinned external official module is unchanged at commit
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`, source SHA-256
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
Physical u_minus and recorded pre-first-FRESH-solve controller previous_control
are restored separately. Historical replay uses the historical solve input pose;
it does not incorrectly require a new solve at B to reproduce an earlier command.
Initial audits on all three named priority candidates reproduced commands exactly.

Every accepted candidate uses an independent official MPC instance and identical
B/u_minus/controller-memory policy. A fresh solve runs at t=0, then 0.1,...,2.9 s;
exact held-command unicycle integration runs at 60 Hz for 3.0 s. Solver wall time
is measured separately and does not advance simulation time. Candidate world rows
are inverse-transformed by the original observation pose and roundtrip-verified;
this is coordinate representation, not re-anchoring at B. GP velocities are not
fed forward. There is no teleport, snap, new observation, new VLA output, or
collision-triggered artificial stopping. Every controller-selected reference
stream and independently integrated state is saved.

Primary success requires actual rollout clearance, known workspace, motion
limits, route crossings, common goal and terminal dwell. GP plan validity is a
separate field. Motion acceleration for commands is a 10 Hz finite difference
including u_minus, not a physical actuator bound. Source and candidate hashes,
all held-command transitions and selection indices are independently checked.

Native-success→M3-failure and adapter-success→M3-failure regressions are reported
before secondary paired-success metrics. Time to goal, error, progress, path,
command changes/variation and acceleration retain separate linear/angular units;
clearance, deformation, optimizer and MPC compute time are also reported. No
cross-method objective comparison establishes superiority.

## Execution commands and result recording

Run commands from the repository root, using the exact new paths recorded in the
final result section. All numerical commands use `OPENBLAS_NUM_THREADS=1` and
`OMP_NUM_THREADS=1`; plots use `MPLCONFIGDIR=/tmp/gp_se2_01_mpl`.

```bash
.venv/bin/python scripts/run_gp_se2_01.py prepare --run "$GP_RUN" \
  --environment data/robotless_gp_se2_01/environment_technical_20260918_retry04
.venv/bin/python scripts/run_gp_se2_01.py optimize --run "$GP_RUN"
/home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/.venv/bin/python \
  scripts/lightnav/gp_se2_mpc_rollout.py --request "$GP_RUN/mpc_request.json" --output "$GP_MPC"
.venv/bin/python scripts/run_gp_se2_01.py evaluate --run "$GP_RUN" --mpc-output "$GP_MPC"
.venv/bin/python scripts/plot_gp_se2_01.py "$GP_RUN"
./scripts/launch_gp_se2_comparison_replay.sh --run "$GP_RUN" --output "$GP_GUI" --verify --no-hold
.venv/bin/python scripts/validate_gp_se2_01.py "$GP_RUN" \
  --gui-validation "$GP_GUI/runtime_validation.json" --write
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
bash -n scripts/isaac/run_gp_se2_environment_export.sh scripts/launch_gp_se2_comparison_replay.sh
git diff --check
```

Every case/method receives eight required 160 dpi plots, GP methods three more,
plus a common actual-rollout overlay and linked index. Failed attempts are
explicitly labelled and never given fabricated zero traces. Isaac representative
selection is frozen as first native-success→M3-failure if present, first D, and
first A, deduplicated; absent categories are recorded. The GUI label is
**OFFLINE COUNTERFACTUAL HANDOFF COMPARISON**. It replays saved actual comparison
rollouts in the original Hospital; it does not imply live online optimization.

At implementation freeze the primary outcome is pending. The final report will
record actual case coverage, every method outcome, regressions, paired metrics,
compute times, image paths, runtime checks and separate operational/research
statuses without rewriting this pre-outcome protocol.

## Frozen primary selection (before outcome review)

Primary run: `data/robotless_gp_se2_01/primary_20260918T054000Z`. Implementation commit: `25d65ffab091c8155a1ace4ddef89a2a5739e911`. Freeze time: 2026-09-18T05:39:04.621157+00:00. Config SHA-256: `15f3f1580f13d437cff0bf88fba59b8526b209e070692bfaab8367f2784863f6`; case manifest SHA-256: `30e884c600a01e452e97dc159bc1c4742a6137190822ca25eea815befe83605d`.

Planned 12; source events 881; eligible 730; selected 10 (A3/B3/C3/D1). The 10 selected events have distinct episode IDs and distinct ordered raw pairs. Eligible descriptive group counts are A381, B45, C98, D1; these groups can overlap before exclusive assignment.

| Case ID | Group / selection reason | Cross-track gap m | Window direction deg | Pose-yaw difference deg | Raw accumulated yaw deg |
|---|---|---:|---:|---:|---:|
| `episode_017_repeat_00/handoff_007` | D: corner shortcut clearance violation | 0.03597 | 5.26790 | 7.61785 | 43.23592 |
| `episode_013_repeat_01/handoff_024` | C: large gap or direction | 0.27911 | 36.54128 | 36.18598 | 64.63608 |
| `episode_014_repeat_01/handoff_026` | C: large gap or direction | 0.16257 | N/A (unreliable chord) | 17.34023 | 130.19143 |
| `episode_000_repeat_01/handoff_019` | C: large gap or direction | 0.32337 | 0.00211 | 0.00170 | 0.07711 |
| `episode_000_repeat_00/handoff_011` | B: small / turn | 0.01166 | 4.89190 | 4.48703 | 15.76876 |
| `episode_002_repeat_00/handoff_039` | B: small / turn | 0.00458 | 3.21142 | 2.69739 | 15.07913 |
| `episode_006_repeat_00/handoff_000` | B: small / turn | 0.00772 | 1.00797 | 2.77143 | 44.98583 |
| `episode_001_repeat_00/handoff_000` | A: small / straight | 0.00003 | 0.00122 | 0.00244 | 0.20971 |
| `episode_001_repeat_01/handoff_002` | A: small / straight | 0.00001 | 0.00390 | 0.00388 | 0.02802 |
| `episode_002_repeat_01/handoff_000` | A: small / straight | 0.00001 | 0.00140 | 0.00328 | 0.02802 |

All 151 excluded events and eligible nonselection reasons remain in `case_manifest.json` and `aggregate/eligibility.csv`. Overlapping exclusions include 111 environment failures, 43 timing failures and 108 unknown routes; these counts must not be summed. Raw suffix statuses are 779 clearance-valid, 50 physical-overlap, 13 margin-violation and 39 unknown. Since unknown status takes precedence, counting numeric overlap separately gives 63 overlapping suffixes; they are diagnostics, not safe-proposal primary cases.

Priority `episode_008_repeat_01/handoff_013` is excluded: B, goal and suffix are unknown workspace, the suffix also overlaps geometry (clearance −0.072765 m), and no valid route can be assigned. The other two named priorities are selected in C. Nine selected cases have a directly verified clear shortcut and no required gate. D has the fixed broad corner gate, directed raw-suffix crossing at 0.759461 s, suffix minimum clearance 0.111832 m and shortcut minimum 0.003889 m.

Physical u_minus differs from controller memory in three selected cases: `episode_000_repeat_01/handoff_019`, `episode_002_repeat_00/handoff_039`, and `episode_001_repeat_00/handoff_000`. This distinction is retained for every method and matters for the first applied-command acceleration check; it is not repaired by method-specific memory resets.

An independent source-only audit recomputed all 881 suffix/environment relations, authenticated 881 FRESH arrays, 60 frozen case-input files and 27 environment files with zero discrepancies. It did not inspect method outcomes. See `environment_selection_independent_audit.json` in the primary run.

## Primary results: failures and regressions first

**M3 changes six M0_NATIVE successes and seven M0_ADAPTER successes into failures. Every such M3 regression is `no_candidate`; no GP execution was substituted or fabricated.** Both GP methods produce zero accepted candidates across all ten cases. Thus the pilot observes no additional benefit from GP under the frozen formulation, initialization and compute budget. This does not prove mathematical infeasibility or disprove Lie-group GP methods in general.

| Method | Candidate available / 10 | Independently plan-valid / 10 | Actual local-transition success / 10 |
|---|---:|---:|---:|
| M0_NATIVE | 10 | 10 | 6 |
| M0_ADAPTER | 10 | 10 | 7 |
| M1_RIGID | 10 | 8 | 7 |
| M2_GP_NO_OBSTACLE | 0 | 0 | 0 |
| M3_GP_CONSTRAINED | 0 | 0 | 0 |

For native/adapter/rigid, plan validity checks reference geometry, endpoint and route; initial connector/motion are not invented. GP plan validity additionally requires sampled boundary and motion feasibility. All actual rollout success conditions are identical across methods. Candidate availability is not equivalent to independently valid planning or execution.

| Case | Group | M0_NATIVE | M0_ADAPTER | M1_RIGID | M2 | M3 |
|---|---|---|---|---|---|---|
| `episode_017_repeat_00/handoff_007` | D | clearance_regression, route_violation | PASS | PASS | no_candidate | no_candidate |
| `episode_013_repeat_01/handoff_024` | C | goal_failure | goal_failure | goal_failure | no_candidate | no_candidate |
| `episode_014_repeat_01/handoff_026` | C | PASS | goal_failure | goal_failure | no_candidate | no_candidate |
| `episode_000_repeat_01/handoff_019` | C | goal_failure, motion_violation | goal_failure, motion_violation | goal_failure, motion_violation | no_candidate | no_candidate |
| `episode_000_repeat_00/handoff_011` | B | PASS | PASS | PASS | no_candidate | no_candidate |
| `episode_002_repeat_00/handoff_039` | B | motion_violation | PASS | PASS | no_candidate | no_candidate |
| `episode_006_repeat_00/handoff_000` | B | PASS | PASS | PASS | no_candidate | no_candidate |
| `episode_001_repeat_00/handoff_000` | A | PASS | PASS | PASS | no_candidate | no_candidate |
| `episode_001_repeat_01/handoff_002` | A | PASS | PASS | PASS | no_candidate | no_candidate |
| `episode_002_repeat_01/handoff_000` | A | PASS | PASS | PASS | no_candidate | no_candidate |

Native-success→M3-failure event list:

- `episode_000_repeat_00/handoff_011` — no candidate
- `episode_001_repeat_00/handoff_000` — no candidate
- `episode_001_repeat_01/handoff_002` — no candidate
- `episode_002_repeat_01/handoff_000` — no candidate
- `episode_006_repeat_00/handoff_000` — no candidate
- `episode_014_repeat_01/handoff_026` — no candidate

Adapter-success→M3-failure event list:

- `episode_000_repeat_00/handoff_011` — no candidate
- `episode_001_repeat_00/handoff_000` — no candidate
- `episode_001_repeat_01/handoff_002` — no candidate
- `episode_002_repeat_00/handoff_039` — no candidate
- `episode_002_repeat_01/handoff_000` — no candidate
- `episode_006_repeat_00/handoff_000` — no candidate
- `episode_017_repeat_00/handoff_007` — no candidate

No physical footprint overlap was found in the 30 executed counterfactuals. This
is not a GP collision-success rate: no GP candidate was executed. D's native
rollout reaches the goal but violates the required edge clearance (minimum
0.043271 m) and crosses outside the usable gate segment. Adapter and rigid pass
that same frozen gate with minimum clearances 0.063069 and 0.058482 m. The raw
reference itself is clearance-valid, demonstrating the need to assess execution
separately from the proposed path.

Preparation is not uniformly beneficial. Adapter and rigid fail the original
goal yaw in `episode_014_repeat_01/handoff_026`, where native succeeds; adapter's
terminal yaw error is 38.828 degrees despite position error only 0.025552 m.
In `episode_013_repeat_01/handoff_024`, native and adapter terminal position
errors are 0.209820 and 0.204504 m. In `episode_000_repeat_01/handoff_019`, all
three executed methods fail goal and physical-command acceleration; native's
terminal position error is 0.389220 m and peak command-grid linear acceleration
is 3.807342 m/s². The preserved controller-memory/u_minus difference matters here.
The native-only motion failure at `episode_002_repeat_00/handoff_039` is a small
strict threshold exceedance: angular acceleration 5.000429 rad/s² versus the
5 rad/s² limit and frozen 1e-5 tolerance, not a large physical dynamics event.

Rigid's two plan-invalid candidates are recorded separately: original endpoint
position errors are 0.150002135 m for `013_repeat_01/024` and 0.150022955 m for
`000_repeat_01/019`. SLSQP's numerical inequality tolerance can admit these tiny
boundary exceedances; the independent endpoint checker does not relabel them
PASS. Tolerances were not adjusted after these results.

## Paired execution and preservation

Every comparison involving M3 has **zero jointly successful events**:
M0_ADAPTER/M3, M0_NATIVE/M3, M1/M3 and M2/M3. Their paired execution, GP FRESH
deformation and obstacle-ablation benefits are N/A, not zero. There is no basis
for claiming a transition improvement or environmental benefit from GP.

M0_NATIVE/M0_ADAPTER has five jointly successful events: `000_repeat_00/011`,
`001_repeat_00/000`, `001_repeat_01/002`, `002_repeat_01/000`, `006_repeat_00/000`.
The following descriptive medians are adapter minus native, with failures already
reported above; they do not imply a general preparation advantage.

| Paired metric | Median difference | Range |
|---|---:|---:|
| Time to goal | +1.260 s | +1.245 to +1.310 s |
| Terminal position error | +0.008237 m | −0.004951 to +0.016229 m |
| Terminal yaw error | +0.000281 rad | +0.0000304 to +0.006906 rad |
| Path length | −0.011139 m | −0.019532 to −0.004506 m |
| Linear command total variation | −0.152321 m/s | −0.261205 to −0.013660 m/s |
| Angular command total variation | −0.000533 rad/s | −0.573582 to +0.0000713 rad/s |
| Peak command-grid linear acceleration | +4.29e-8 m/s² | −0.025012 to +7.34e-8 m/s² |
| Peak command-grid angular acceleration | −0.003765 rad/s² | −0.722773 to −0.000407 rad/s² |
| Goal-distance improvement | −0.008237 m | −0.016229 to +0.004951 m |
| Goal reentries | 0 | 0 to 0 |

First-command differences, per-event clearances, progress, time/errors, compute
and deformation are in `aggregate/paired_metrics.csv`; all outcomes remain in
`aggregate/primary_outcomes.csv`. The adapter has zero deformation against
F_common by definition. Native-to-common deformation is not asserted as a timed
point correspondence. Descriptively across all ten rigid candidates, mean local
log-translation deformation per case has median 0.033551 m and range 0.0000196
to 0.150023 m. These are candidate descriptions, not paired GP benefits.

## Solver and runtime costs

| Method | Numerical terminations (two starts/case) | Iterations/start | Total optimizer + internal checking |
|---|---|---:|---:|
| M1_RIGID | 20 converged | 1–12 | 1.535577 s |
| M2_GP_NO_OBSTACLE | 20 timeout | 39–49 | 600.261555 s |
| M3_GP_CONSTRAINED | 19 timeout, 1 solver failure | 24–47 | 587.819245 s |

Rigid per-case wall time ranges 0.050296–0.216713 s. M2 is about 60.025 s/case;
M3 ranges 47.534125–60.084101 s/case. The nominal 30 s per-start budget is checked
between numerical calls; one M3 start reaches 30.0579 s before the check returns.
Post-solve checks and serialization add overhead. This is offline compute, not a
real-time deadline or simulated optimizer latency. Baselines have no optimizer;
their microsecond reference-loading overhead is not reported as optimization.

All 40 GP latest iterates violate dense lateral-velocity and linear-acceleration
conditions. Maximum absolute lateral velocities range 0.0005106–0.50066 m/s
against 1e-5 tolerance; linear-acceleration exceedance ranges
0.0014985–4.98345 m/s². Additional latest-iterate failures include linear speed
20/40, angular speed 3/40, angular acceleration 5/40 and goal 7/40. Of the four
gate-constrained GP starts, three miss the gate-plane equality and one misses
direction. Only 2/20 M3 latest iterates violate conservative obstacle constraints;
none of the 40 fail workspace or the pose/body-derivative identity. Consequently,
most GP failures cannot be attributed to obstacles. These are saved-iterate
numerical diagnostics, not proofs that the constrained problem has no solution.

The single solver failure is M3 start 0 for `episode_000_repeat_01/handoff_019`,
SLSQP status 4, “Inequality constraints incompatible”; the message is not accepted
as an infeasibility proof. M2/M3 configuration, support grids, initial vectors,
initial support poses and twists match numerically and in float64 bytes for every
case. `aggregate/solver_diagnostics_independent.json` authenticates all 60 solver
starts and preserves the earlier partial diagnostic without overwriting it.

The unchanged official MPC executed 900 new solves across 30 independent
rollouts, with **zero controller failures**. Total official solve time was
2.865857 s; median 0.003066 s and maximum 0.007717 s/solve. Totals by method:
native 1.014915 s, adapter 0.930660 s, rigid 0.920282 s. All 10 historical audits
reproduce recorded commands exactly (maximum error 0); all reconstructed 60 Hz
state transitions also have maximum error 0. Runtime is Python 3.11.16,
NumPy 2.4.6 and CasADi 3.7.2 in the existing isolated official-demo environment.

## Images and actual Isaac verification

The primary [local image index](../data/robotless_gp_se2_01/primary_20260918T054000Z/index.html)
contains 460 required method PNGs at 160 dpi and ten common rollout overlays.
Every selected case/method, including all 20 no-candidate GP outcomes, is present.
Original OLD/FRESH, frozen boundary/goal/gates, actual obstacle/workspace geometry,
footprint/margin, accepted candidate and actual execution use labelled distinct
lines. Rejected GP iterates are dotted and explicitly rejected; no execution or
zero-filled command trace is fabricated. PNG/source/config/metric hashes and
common metric axis limits are recorded in each plot provenance file.

Actual Isaac replay output is
`data/robotless_gp_se2_01/gui_primary_20260918T054000Z`. Runtime verification passed
for 15 actual 2474×1315 screenshots, all five methods for each of:

- Native-success/M3-failure: `episode_000_repeat_00/handoff_011`.
- Obstacle/route-sensitive: `episode_017_repeat_00/handoff_007`.
- Benign reference: `episode_001_repeat_00/handoff_000`.

Nine views display saved rollouts; six display explicit NO CANDIDATE / NO ROLLOUT.
The exact required GUI label is visible. Reset/play/pause/end/next callbacks and
USD pose readback are tested in actual Isaac, not claimed as OS mouse automation.
Method camera bounds are identical within each case. D's 8.693 m gate makes the
full-scene overview wider; the metric boundary zoom plots retain local detail.
Screenshots, sidecars, unchanged source hashes and visual inspection are recorded
in `runtime_validation.json` and `independent_visual_review.json`. GUI replay does
not generate states, inference or MPC solves. Isaac exited normally.

## Post-primary verification correction and preservation

The first artifact validator reported one mismatch after 42,142 checks:
`aggregate comparison mismatch: paired_metrics`. The cause was validator row
bookkeeping: its reconstructed rows omitted saved optimizer wall time and FRESH
deformation, while the primary aggregation already included those columns. All
other comparison fields and outcome sections matched exactly. The original error
report and frozen validator source are preserved.

A distinct verification run,
`data/robotless_gp_se2_01/verification_20260918T060500Z`, records the narrow
validator correction, old/new code hashes, source proof, and 1,097 primary
method/result hashes before and after. **The numerical primary was not rerun**:
no formulation, config, case, candidate, reference, MPC execution or outcome
changed. Only the independent validator now carries the already-saved
`optimizer_wall_s` and `deformation` into its comparison reconstruction. The
corrected validator is a disclosed post-primary code drift; the actual numerical
experiment remains authenticated against implementation commit 25d65ff and its
archived source snapshots. The new verification ID does not pretend to be a new
optimization experiment.

Final preservation audit passed all 15 checks: all 36,684 retained source/derived
files (2,504,027,893 bytes), all 63 checkpoint-tree files (9,709,456,575 bytes), the
original source validator result, both unrelated user configuration files, and
the external clean LightNav/MPC pin remain unchanged. This includes the 16 model
and configuration artifacts plus ancillary files; no historical Jackal data was
recreated. See `preservation_after.json`.

## Final validation, commands and interpretation

Final artifact validation is **valid=true, errors=[]**, with 42,512 checks,
50 method records, 460 required images, 943 local index links and actual GUI
validation. The only disclosed current-workspace drift from the archived
experiment is the corrected validator and its regression test. All 1,097 frozen
method/result hashes are unchanged. Final host pytest: **1,432 passed, 18 skipped
in 49.22 s**; skips are explicitly unavailable retired historical corpora.
Compileall, both launchers' `bash -n`, diff and staged-diff checks pass.

The final operational status is **GP_SE2_01_COMPLETED_WITH_LIMITATIONS**. The
independent validator's generic COMPLETED status establishes artifact/runtime
completion; the report adds the missing D coverage and other research limits.
Research interpretation is separately **NO_ADDITIONAL_BENEFIT_OBSERVED** under
this fixed formulation and budget. There are no jointly successful GP pairs,
so no GP smoothness, preservation or obstacle benefit is inferred. The result
motivates no unrequested new solver, weight search or dataset extension here.

The append-only authoritative final interpretation is
`aggregate/final_interpretation.json`. `aggregate/summary.json` retains its
pre-GUI/pending-interpretation flags as an audit snapshot; its method counts and
paired metrics are unchanged. The correction ID and final test logs live in the
separate `verification_20260918T060500Z` directory.

Exact actual paths used (all output directories were new on first execution):

```bash
GP_RUN="$PWD/data/robotless_gp_se2_01/primary_20260918T054000Z"
GP_MPC="$PWD/data/robotless_gp_se2_01/mpc_primary_20260918T054000Z"
GP_GUI="$PWD/data/robotless_gp_se2_01/gui_primary_20260918T054000Z"
GP_VERIFY="$PWD/data/robotless_gp_se2_01/verification_20260918T060500Z"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
export MPLCONFIGDIR=/tmp/gp_se2_01_mpl
.venv/bin/python scripts/run_gp_se2_01.py prepare --run "$GP_RUN" \
  --environment data/robotless_gp_se2_01/environment_technical_20260918_retry04
.venv/bin/python scripts/run_gp_se2_01.py optimize --run "$GP_RUN"
/home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/.venv/bin/python \
  scripts/lightnav/gp_se2_mpc_rollout.py --request "$GP_RUN/mpc_request.json" --output "$GP_MPC"
.venv/bin/python scripts/run_gp_se2_01.py evaluate --run "$GP_RUN" --mpc-output "$GP_MPC"
.venv/bin/python scripts/plot_gp_se2_01.py "$GP_RUN"
./scripts/launch_gp_se2_comparison_replay.sh --run "$GP_RUN" --output "$GP_GUI" --verify --no-hold
.venv/bin/python scripts/validate_gp_se2_01.py "$GP_RUN" \
  --gui-validation "$GP_GUI/runtime_validation.json" --write
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q -p no:cacheprovider
.venv/bin/python -m compileall -q src scripts tests
bash -n scripts/isaac/run_gp_se2_environment_export.sh scripts/launch_gp_se2_comparison_replay.sh
git diff --check
```

To recheck the existing completed run, use the validator command **without
`--write`**. A new numerical reproduction must choose new output IDs; overwrite
refusal is intentional. To open a persistent comparison GUI, use a new GUI output
ID and omit `--no-hold`. The primary ran with implementation commit
`25d65ffab091c8155a1ace4ddef89a2a5739e911`; only final validation used the disclosed
corrected validator (SHA-256
`a4248695eb8628fa56cbe0cc35e52283706c3294b938b45fa12c3b613184901b`).

Starting SHA is `ada33e145a412899ef5d33b05e9757fe4a780db7`. Final result-report
commit and normal-push outcome are recorded after commit in
`verification_20260918T060500Z/git_completion.json` to avoid a self-referential
commit hash. No generated geometry, arrays, images, recordings, checkpoints,
external source or environments are committed. Both user configuration edits
remain unstaged and unchanged.
