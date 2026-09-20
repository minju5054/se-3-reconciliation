# GP-SE2-DIAG-03: saved GP interpolation feasibility audit

Question: where do the fixed hard-case GP curves violate the original motion
conditions, and is that discrepancy interpolation arithmetic or finite enforcement?
This audit does not optimize, change acceptance, or execute a rejected curve.

Starting revision: `da5b93151aec62ce88d4fa6bda58ca8ae5b7dd86`.
Source: `data/robotless_gp_se2_02/primary_20260919T024000Z/`, numerical execution
revision `bd273b22ed9ddcb35a3f2eadcf550895adee46b9`. Its final `validation.json`
is the authoritative historical artifact validation, rather than intermediate
reporting-only validator failures.

## Frozen scope and numerical protocol

The ledger contains initial and latest vectors for M2/M3 × I0_FRESH/I1_DECEL of
`episode_013_repeat_01/handoff_024` (eight records), plus the saved benign M3/I1
latest vector from `episode_001_repeat_01/handoff_002`. The four hard finals are
the analysis targets; initials and the benign reference are context, not new
independent performance samples. Null candidate fields do not discard saved
latest iterates. Missing vectors remain explicit and are never regenerated.

Original 150-variable right-local chart, 31 support states, 3 s horizon, 0.1 s
support spacing, B/physical initial twist, F_common, original goal, route and
validated environment are loaded through the original frozen-case loader.
Original core, inputs, results and images are hashed, not overwritten.
Unrelated Stage-0 configuration edits are separately preserved.

Original objective/collocation/dense reports are compared to saved records at
absolute 1e-8 / relative 1e-10 tolerance, with literal equality also reported.
Support reconstruction uses absolute 1e-10 and periodic yaw comparisons.
Physical tolerances remain the original 1e-5: equality in m/s, motion inequalities
in their respective units, and squared goal-distance margin in m². The separate
independent plan checker uses its original motion_numeric_tolerance, coincidentally
also 1e-5, and actual endpoint distance rather than a squared-distance residual.

All 30 intervals are queried using original collocation, dense and full-offset
grids and supplemental spacings 0.010/0.005/0.001 s. The last grid includes the
original 0.371 offset. Knot-side acceleration is retained separately. Sampled
extrema are not continuous extrema. Only observed safe/violating transition
brackets are refined to <=0.0001 s; safe endpoints alone prove nothing inside.

Independent cubic coefficients are compared against production Hermite values.
The algebraic expression shares frozen Exp/Log/right-Jacobian primitives; a
separate temporal finite difference of pose provides body velocity, and finite
differences of independently reconstructed velocity provide body acceleration.
Fixed interval fractions are .125/.25/.5/.75/.875, augmented by observed witnesses.
Epsilons are 1e-4, 3e-5, 1e-5 s, guarded from crossing knots or local Log cuts.
Frozen diagnostic absolute tolerances are coefficient pose/twist 1e-9,
coefficient acceleration 1e-8, FD twist 2e-6 and FD acceleration 2e-5 in component
units. Both finest valid epsilons must pass; all error trends remain recorded.
These are diagnostic agreement thresholds, never physical acceptance changes.

G0 uses original 0/.5/1 motion points and midpoint lateral equality. G1 also
queries quarter points. G2 adds post-hoc observed witnesses. They only detect
violations of the same saved vector; no new constraints are solved. Extra equality
counts do not imply extra independent rank or a feasible refined optimization.

New VLA inference, GP/rigid optimization, MPC solve, rollout and Isaac runtime
counts are all zero. The runner forbids optimizer and rollout entry points.
No positive scientific outcome is assumed by tests.

## Completed saved-record result

The audit code/protocol was committed as
`d43cf9e5e37bee4a4f859686fd9b95349cdd74c0` before numerical queries. Primary output:
`data/robotless_gp_se2_diag_03/audit_20260920T010000Z/`.
All nine planned records are available, representing six unique vectors. M2/I0
and M3/I0 final vectors are identical; both pairs of initials are shared across
M2/M3. They remain nine provenance records, not nine independent trajectories.

All unpacked supports match stored chart-reconstructed supports literally. Every
available saved objective, collocation report, dense report and historical full
feasibility flag reproduces literally, not only within the declared tolerance.
Original initial full-check records also reproduce literally. Hard rejected
finals were historically excluded immediately by the dense checker, so their
original post-checks contain a false full-feasibility flag and a skip reason,
not a full offset-grid array. This audit runs the unchanged full checker on those
final vectors and confirms rejection without inventing a historical full check.
Per-factor GP/FRESH costs for rejected iterates were not saved independently;
they are recomputed from the unchanged evaluator, with their summed objective
compared to the original record.

| Record | Objective | Collocation | Dense | Full |
|---|---:|---|---|---|
| Hard M2/I0 initial | 2937.344516854675 | FAIL | FAIL | FAIL |
| Hard M2/I0 final | 16.343333506702 | PASS | FAIL | FAIL |
| Hard M2/I1 initial | 128.361602499565 | FAIL | FAIL | FAIL |
| Hard M2/I1 final | 16.343333525860 | PASS | FAIL | FAIL |
| Hard M3/I0 initial | 2937.344516854675 | FAIL | FAIL | FAIL |
| Hard M3/I0 final | 16.343333506702 | PASS | FAIL | FAIL |
| Hard M3/I1 initial | 128.361602499565 | FAIL | FAIL | FAIL |
| Hard M3/I1 final | 16.343333504194 | PASS | FAIL | FAIL |
| Benign M3/I1 final | 0.194335067089 | PASS | PASS | PASS |

All five recorded final solver terminations are `CONVERGED`; this historical
solver status is never recomputed. None of the four hard starts produced an
accepted candidate. All four hard finals pass original goal, route, workspace
and obstacle checks, and fail lateral velocity, forward speed and linear body
acceleration. Their objective reductions do not imply valid motion or execution.
I0 initial fails multiple motion families. I1 initial satisfies the checked
motion conditions but misses the original goal: its benign feasibility does not
transfer to this hard case. The benign final remains fully valid and has sampled
maximum lateral speed 2.13281e-8 m/s, well below the original 1e-5 m/s tolerance.

## What the original constraints inspect

For these no-gate inputs, exact evaluator ordering is:

| Row family | Count | Query location / units |
|---|---:|---|
| lateral equality | 30 | midpoint of each interval; m/s |
| lower/upper forward speed | 90 each | u=0,.5,1; m/s |
| positive/negative angular speed margins | 90 each | u=0,.5,1; rad/s |
| positive/negative linear acceleration margins | 90 each | u=0,.5,1; m/s² |
| positive/negative angular acceleration margins | 90 each | u=0,.5,1; rad/s² |
| terminal squared position margin | 1 | t=3; m² |
| terminal wrapped yaw margins | 2 | t=3; rad |
| workspace margin | 90 | u=0,.5,1; m |
| obstacle margin, M3 only | 90 | u=0,.5,1; m |

Motion rows are family-major, then interval-major and local-fraction order.
Total equalities/inequalities are 30/813 for M2 and 30/903 for M3. The generated
`constraint_time_map.csv` verifies every original numerical row and sign.
Support lateral velocity is structurally zero; it is not 31 extra equality rows.

The original dense report adds 0.01 s queries and both knot-side limits. The
independent plan checker uses its original 0.005 s grid, with the original
proximity-based refinement if triggered, and both knot accelerations. The full
check additionally uses 0.001 s base and 0.371-offset queries. Interval analysis
preserves both knot sides in all reported grid families. Exact float timestamps
can differ in their final bit; literal and rounded unique-time counts are both
reported rather than treating these as extra independent locations.

## Violations inside intervals

Indices below are zero-based; interval 4 is [0.4,0.5] s. All listed extrema are
observed finite-grid extrema, not exact continuous maxima.

| Hard final | max abs(v_y) at .479 s, u=.79 (m/s) | min v_x at .478 s, u=.78 (m/s) | min a_x at .029 s, u=.29 (m/s²) |
|---|---:|---:|---:|
| M2/I0 | 0.000130154486 | -0.003038027080 | -2.000078365984 |
| M2/I1 | 0.000130195712 | -0.003041773152 | -2.000078365697 |
| M3/I0 | 0.000130154486 | -0.003038027080 | -2.000078365984 |
| M3/I1 | 0.000130140040 | -0.003036628040 | -2.000078366262 |

These are off-collocation locations: the nearest motion enforcement point is
0.022 s away for the speed minimum and 0.021 s away for the acceleration minimum.
The lateral maximum is 0.029 s from the enforced midpoint, or 0.021 s from the
structurally zero next support. The distinction is recorded explicitly.
Tolerance-adjusted excess is about 1.20e-4 m/s for lateral velocity, 0.00303 m/s
for negative forward speed, and 6.84e-5 m/s² for linear acceleration. Angular
speed and acceleration have no tolerance-exceeding violations in the hard finals.

| Family | First observed violating sample | Refined observed entry bracket |
|---|---:|---|
| linear acceleration, interval 0 | .003 s | [.002371, .002449625] s, all four finals |
| lateral velocity, interval 1 | .102 s | [.101764125, .101842750] s, all four finals |
| forward speed, interval 4 | .456 s | [.455685500, .455764125] s, except M2/I1 |
| forward speed, M2/I1 | .456 s | [.455606875, .455685500] s |

The refined bracket widths are 78.625 microseconds. They locate the first
observed transition, not a proof that no smaller unobserved violation occurs
earlier. Every interval, every observed violating run, its exit bracket and all
six physical quantities are retained in the tables. The finer grid improves on
the old .480/.030 s sampled extrema without changing the curve or acceptance.

All hard final support/midpoint motion checks pass within tolerance. Both knot
acceleration limits are retained and neither violates the original tolerance.
Accordingly `one_sided_knot_violation_observed=false` is scoped to **hard finals**.
I0 initial context does contain knot-side violations; that flag is not a claim
about all nine records. Acceleration may jump across a knot without either side
violating its bound. No finite difference mixes these sides.

## Independent calculation consistency

Every record passes the coefficient comparison and fixed multi-step temporal
derivative protocol. For the hard finals, coefficient acceleration differences
are at most about 2.40e-14 in component units. The independent finite-difference
errors below are maximum componentwise absolute errors across the four finals:

| epsilon (s) | vx (m/s) | vy (m/s) | omega (rad/s) | ax (m/s²) | ay (m/s²) | alpha (rad/s²) |
|---|---:|---:|---:|---:|---:|---:|
| 1e-4 | 2.063e-8 | 6.523e-9 | 2.228e-8 | 2.411e-9 | 2.738e-8 | 2.771e-12 |
| 3e-5 | 1.907e-9 | 5.963e-10 | 2.009e-9 | 2.172e-10 | 2.465e-9 | 7.355e-12 |
| 1e-5 | 3.783e-10 | 2.911e-10 | 2.355e-10 | 2.774e-11 | 1.911e-9 | 2.045e-11 |

The errors fall with epsilon before roundoff in some components. I0 initial is
much less regular: the coarsest twist FD error can exceed the finest-step
diagnostic tolerance, but both predeclared required finer epsilons pass, with the
expected truncation trend. No threshold was increased after observing results.
The body acceleration discrepancy is much smaller than the observed physical
bound excess. Algebraic and time-derivative evidence therefore supports finite
enforcement gaps, not an interpolation/derivative inconsistency at these probes.
This is not an independent reimplementation of every SE(2) primitive or a global
proof that the numerical core has no other error.

## Detection-only extra points

Every hard final has 14 intervals with lateral violations (28 separated sampled
violating runs), one interval/run with negative speed, and three intervals/runs
with linear acceleration violations.

| Grid | Lateral equality evaluations | Motion evaluations | Observed detection |
|---|---:|---:|---|
| G0 original | 30 | 90 | all three failing families missed |
| G1 quarters | 90 (+60) | 150 (+60) | all observed intervals and all 28/1/3 sampled violating runs detected |
| G2 witness | 48 (+18) | 108 (+18) | all observed violating intervals detected; only 14/28 lateral runs directly hit |

G1 would add 480 motion inequality rows; G2 would add 144. Variables remain 150.
The independent rank of 90 or 48 hypothetical lateral equality rows was not
computed. G2 uses one maximum witness per interval/quantity, deduplicated against
existing points; it is post-hoc and deliberately does not hit every secondary
lateral pocket. Benign G0/G1/G2 all pass, with no G2 witnesses. These are fixed
vector detection checks, not optimization, convergence or feasibility recovery.

## Interpretation and next single change

Confirmed diagnosis A: all four hard finals satisfy the original collocation
checks but violate physical motion conditions inside intervals; pose/body-twist/
acceleration calculations agree with independent coefficient and time-derivative
checks. No unit, source, chart or original-result discrepancy was found. Current
data do not establish nonexistence of a feasible GP path, inability of SLSQP to
solve it, or an unsuitable physical tolerance.

Next single experiment: retain the original formulation, initialization, support
grid and physical thresholds, and compare a **bounded quarter-point constraint
refinement** against the original enforcement grid on these frozen starts.
Question: can the additional lateral/motion constraints produce a full-check-valid
candidate within a fixed solve budget? G1's detection success motivates that one
change but does not predict solver success; equality rank/conditioning must be
reported. No such solve was run here. Controller redesign is not a prerequisite,
and GP execution improvement remains a separate future question.

## Reproduction commands and evidence

```bash
.venv/bin/python scripts/run_gp_se2_diag03.py prepare --run data/robotless_gp_se2_diag_03/audit_20260920T010000Z
.venv/bin/python scripts/run_gp_se2_diag03.py audit --run data/robotless_gp_se2_diag_03/audit_20260920T010000Z
.venv/bin/python scripts/run_gp_se2_diag03.py validate --run data/robotless_gp_se2_diag_03/audit_20260920T010000Z
.venv/bin/python scripts/present_gp_se2_diag03.py --primary data/robotless_gp_se2_diag_03/audit_20260920T010000Z --output data/robotless_gp_se2_diag_03/presentation_20260920T011000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

Existing output is exclusive and cannot be overwritten; use a new run ID when
reproducing. Validator re-loads the original vectors and recomputes acceptance,
interval/derivative outputs, tables and plot numbers without a solver. Primary
authoritative validation passes with zero errors. All 1,275 relevant source,
core, environment and historical-result hashes, plus both unrelated edits, are
preserved. No unrelated corpus is re-evaluated or bulk-copied.

Preparation took 0.5111 s, environment loading 0.0441 s and nine record analyses
4.2149 s. The audit stage including output/plots/package took 19.3434 s;
independent saved-record/artifact validation took 12.7189 s. These offline audit
costs are not optimization performance or online feasibility evidence.

The primary index and seven PNG/sidecars are under the primary run. A separate
presentation-only correction fixes crowded labels in the worst-interval plot
and clarifies that zero positive excess does not mean zero signed residual.
Primary numbers and PNGs are preserved; the preferred presentation index is
`data/robotless_gp_se2_diag_03/presentation_20260920T011000Z/index.html`, with its
small `review_bundle.zip`. The other six figures are copied byte-for-byte.
All images label rejected iterates as not executed. No GUI or video was produced.
The preferred ZIP is 11,443,730 bytes, SHA-256
`4f36945b1c3cefa23f581a2a39d94a9c062f4ba96a210f7e3d01c27cd292f56a`.
Presentation validation confirms all seven numeric payloads, six unchanged images
and all ZIP members, without any additional GP queries. Original external MPC
source SHA-256 `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`
is unchanged and its pinned external checkout remains clean.

Final repository verification: 2,225 tests passed, 19 existing historical-data
or fixture skips, in 119.71 s. No new audit test was skipped. Compileall and
`git diff --check` pass; no shell launcher changed. Primary saved-record validator
passes 23,334 checks with no errors, and presentation validation passes 138
checks. These checks establish artifact consistency, not research performance.
Code/tests/docs only are committed; generated arrays, images and ZIP stay local.
Final report SHA and normal push confirmation are in the ignored run's
`git_completion.json` and the user-facing completion report.

Operational status: `GP_SE2_DIAG_03_COMPLETED_WITH_LIMITATIONS`.
Diagnosis flags: original_results_reproduced=true;
interpolation_derivative_consistent=true;
collocation_pass_interior_fail_observed=true;
one_sided_knot_violation_observed=false (hard finals);
supplemental_grid_detects_missed_violations=true;
implementation_inconsistency_found=false; insufficient_saved_evidence=false.
Limitations are finite sampling, shared primitive algebraic checks, historical
full-query arrays absent after dense rejection, and no new optimization or
execution evidence. All six categories of prohibited new runtime remain zero.
