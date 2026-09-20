# GP-SE2-DIAG-04: quarter-point motion constraint refinement

Question: with the same GP representation, objective, saved initializations and
physical acceptance, does adding quarter-point motion enforcement recover a
full-valid candidate for the fixed hard handoff? This experiment performs new
GP solves. It does not execute paths or change the controller/selector.

Starting revision: `9f6031bf46f4b885af91433ab023a1552e222599`.
Historical numerical source: GP-SE2-02
`data/robotless_gp_se2_02/primary_20260919T024000Z/`, execution revision
`bd273b22ed9ddcb35a3f2eadcf550895adee46b9`. The preceding diagnosis is
`data/robotless_gp_se2_diag_03/audit_20260920T010000Z/`.
Their final authoritative validations, original inputs/configuration, numerical
core, environment, historical artifacts and unrelated Stage-0 edits are hashed.
The original external MPC file is hashed and never executed or edited.

## Frozen experiment

The hard event is `episode_013_repeat_01/handoff_024`. The benign control is
`episode_001_repeat_01/handoff_002`. The ten sequential starts are:

1. Hard M2 / I0_FRESH: G0 then G1.
2. Hard M2 / I1_DECEL: G0 then G1.
3. Hard M3 / I0_FRESH: G0 then G1.
4. Hard M3 / I1_DECEL: G0 then G1.
5. Benign M3 / I1_DECEL: G0 then G1.

Each pair uses byte-identical original GP-SE2-02 initialization files. Historical
latest vectors are verification inputs only. There are no extra starts or
outcome-dependent retries. These are four comparisons of one hard event, not
four independent hard handoffs.

The original 31 support states, 150-variable right-local chart, 3 s horizon,
0.1 s support spacing, B, physical initial twist, F_common, goal, objective,
weights and environmental queries remain unchanged. World poses are
T_world_agent; body twist is vee(X^-1 dX/dt). Translation is in metres and yaw
in radians. The downstream time convention is not an intrinsic LightNav
waypoint timestamp. No re-anchoring or new reference preparation occurs.

G0 delegates objective and constraints to the original GPProblem. G1 appends
60 lateral equalities (v_y=0 at u=.25,.75 of each interval) and 480 original
motion inequalities (eight signed margins per added sample). Acceleration is
the derivative of body-twist components, not world acceleration. No new a_y
condition, environment row, goal row, terminal stop, slack or soft penalty is
introduced. Original arrays are exact prefixes.

| Grid | Variables | Equalities | M2 inequalities | M3 inequalities |
|---|---:|---:|---:|---:|
| G0 | 150 | 30 | 813 | 903 |
| G1 | 150 | 90 | 1293 | 1383 |

Both use installed SLSQP with supplied objective gradient and both constraint
Jacobians: 200 iterations, ftol=1e-7, 30 s prepared budget, float64 CPU and
single-thread BLAS. The new adapter preserves callback/dense-first retention;
both grids disable historical module-global inner profiling hooks. Inclusive
primal evaluation and derivative callback costs are measured. Unmeasured
inner GP/environment costs remain N/A. Compilation, input/environment loading,
candidate checks, rank diagnostics and offline cost are separate from solve.
Package versions and source hashes are frozen in the generated protocol.

## Derivative gate and rank protocol

The unchanged DIAG-02 provider supplies all original derivatives. A separate
JAX extension uses the original AD interpolation primitives to append only
quarter rows, including current right-chart chain rule and both body
acceleration terms. No finite-difference fallback is used. Original relative
Log/goal cut guards and analytic environment branch handling remain active.

Before any primary solve, freeze five seed records, five historical latest
records, and five fixed seed perturbations. Method provenance is retained for
duplicates. Random seed is 20260919; perturbation scales are 1e-4 in each chart
column and are never used as optimization initializations. Three normalized
150-dimensional directions use steps 2e-4, 2e-5, 2e-6. Both finer steps must
pass. Reused DIAG-02 absolute/relative tolerances are:

| Check | Absolute | Relative |
|---|---:|---:|
| Primal values | 1e-8 | 1e-10 |
| Objective derivative | 2e-4 | 2e-5 |
| Constraint derivatives | 2e-5 | 2e-5 |

Original branch-aware base verification is repeated at these records; quarter
families are checked separately, including signs/order/shapes and worst errors.
All base primal/derivative prefixes require literal equality. Diagnostic
tolerances never change physical acceptance. A failed gate blocks optimization.

After all solves, evaluate G0/G1 equality Jacobians at initial/latest/selected
vectors only. Report raw and column-scaled SVD separately, using repeated
[1 m,1 m,1 rad,.8 m/s,3 rad/s] scales. Rank cutoffs are max(shape)*eps*s_max
and relative 1e-12,1e-10,1e-8,1e-6; near-zero row norm threshold is 1e-12.
These scales never enter SLSQP. Rank deficiency does not prove physical
infeasibility and does not authorize removing equalities.

## Candidate and artifact policy

Retain initial/latest and actual solver-grid-feasible callbacks. Recompute
their grid residuals and original dense report after the solve. Among eligible
candidates select minimum original objective, then lexical source label.
Independently run unchanged check_full_candidate on the original base problem,
including original environment/goal, offset grid and both knot accelerations.
Full failure does not trigger reselection. Initial/latest/selected, convergence,
availability, full validity and unchanged-seed return remain separate fields.

Full 3 s curves are also queried with the frozen DIAG-03 interval diagnostics,
including .001 s and .371 offset samples. Extremum figures use each new result's
maximum tolerance excess and earliest-interval tie-break, not historical
witnesses. These are sampled observations, not a continuous-time certificate.
All paths are unexecuted plans. Failed results stay in every table and index.

The saved-record validator recomputes grid/dense/full reports, selection,
callback values, interval traces, SVDs, aggregate tables, plot numeric sidecars
and hashes without optimization. Generated data and images are excluded from
Git. Existing run/phase outputs refuse overwrite and retry.

Interface references: [SciPy minimize](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html),
[SLSQP](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html),
[JAX jacfwd](https://docs.jax.dev/en/latest/_autosummary/jax.jacfwd.html).
Installed versions are used without upgrades.

## Execution and results

Pending the frozen actual-point derivative gate and ten primary solves.
No experimental success is claimed by implementation unit tests.
