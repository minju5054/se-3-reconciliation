# GP-SE2-DIAG-08: fixed planning endpoint reserve versus official MPC execution

Question: does one fixed 4 cm planning endpoint reserve recover the original
3 s execution goal-position and final-dwell criteria on the fixed hard event?
This is a development-event formulation intervention, not margin optimization
or a general navigation claim. DIAG-07's four hard G3 references failed original
goal position/dwell despite passing actual clearance, route, command motion and
yaw; their plan endpoints were approximately .13314 m from the original goal,
and final execution-to-plan displacement was approximately .03846 m.

## Frozen formulation and evidence

Starting main: `1fa1e1e83074218b44456e4a00c64947a2591760`, equal to fetched
origin/main. Run: `data/robotless_gp_se2_diag_08/primary_20260920T151000Z/`.
G3 numerical authority: DIAG-06 `primary_20260920T083200Z/validation.json`.
G3 execution authority: DIAG-07
`primary_20260920T103000Z/verification/validation.json`, plus delivery validation.
The earlier DIAG-07 root validator's redirected-log hash error is preserved;
it is not substituted for the authoritative reporting completion.

G4 is G3 plus exactly one appended inequality:

`0.11**2 - ||p_T - original_goal_xy||**2 >= 0`.

The original goal tolerance field remains .15 m. Original goal yaw, lateral
acceptance, objective/prior/interpolation/weights, chart, support grid, physical
limits, footprint, environment/route and controller/selector are unchanged.
The 4 cm reserve is frozen once, rounded from the observed DIAG-07 tracking
scale, not tuned after results. No future witnesses are extracted: the same
three-row DIAG-06 witness file is copied byte-for-byte and hash-checked.
Dimensions: 150 variables, 30 equalities, 1297 M2 / 1387 M3 inequalities.

The G4 primal delegates to G3 and appends an independently computed NumPy
squared endpoint distance. The existing CPU float64 JAX derivative of the
original goal-position row is reused verbatim: changing the constant radius
has zero derivative. The original row is identified from the source row map
(index 720 for these 30 intervals), not by nearest geometry. This retains the
current nonzero right-chart chain rule and all existing branch guards. No new
AD graph, FD fallback, scaling or numerical-core change is introduced.

The derivative gate uses five original seeds, five saved G3 latest vectors,
and the five unchanged DIAG-04 seed perturbations and three directions inherited
through DIAG-06. FD steps are 2e-4, 2e-5, 2e-6; both finer steps must pass.
Primal absolute/relative thresholds are 1e-8/1e-10, constraint derivatives
2e-5/2e-5. G3 validation is also reused on those points. Objective, equality,
original inequality prefix and all Jacobian prefixes must match literally.
A failed gate blocks every primary solve.

## Predeclared solve, acceptance and execution policy

Exactly five G4 solves, sequential, once:

1. `episode_013_repeat_01/handoff_024`, M2 / I0_FRESH.
2. Same hard event, M2 / I1_DECEL.
3. Same hard event, M3 / I0_FRESH.
4. Same hard event, M3 / I1_DECEL.
5. `episode_001_repeat_01/handoff_002`, M3 / I1_DECEL.

Original saved initialization bytes must match G3 and GP-SE2-02. Historical
latest/selected vectors are derivative probes only. SLSQP, supplied derivatives,
200 iterations, ftol=1e-7, prepared budget 30 s/start and single-thread BLAS
remain fixed. Construction/compilation, solve, post-check and execution costs
are separate. No retry, additional initialization or second margin is allowed.

The unchanged DIAG-04 retention harness inspects initial/latest/actual
solver-grid-feasible callbacks and selects minimum original objective, then
lexical source label, among grid+dense-valid vectors. The original full checker
receives the unchanged GPProblem; a later full failure never triggers another
selection. Every inspected vector also receives original DIAG-03 interval
checks, both knot sides and the DIAG-06 sampled motion-run diagnostics.

**The new reserve has two separate fields.** Solver-grid feasibility retains
the original inequality allowance (1e-5 m² for a squared-distance row).
`planning_endpoint_reserve_pass` is the direct nominal check
`norm(endpoint-original_goal_xy) <= .11`, with no extra numerical tolerance.
Thus a tiny nominal excess can be solver-grid-valid but execution-ineligible.
Both the raw excess and squared margin are retained. This conservative policy
is frozen before any new solve and will not be loosened after results.

The diagnostic execution reference is always the **saved latest iterate**,
exactly `unpacked_poses[1:]`: 30 pose rows, with no interpolation, velocity
feed-forward, suffix reselection, re-anchoring or optimization-based reselection.
The retained candidate is reported separately. Execution admission requires
the nominal reserve, all original nonlateral checks and all supplemental
sampled motion inequalities. Hard lateral-only invalid plans may execute only
as explicitly labeled diagnostic references; they remain plan-invalid and
not deployment candidates. Fully valid references are recorded as valid
candidates, but no actual deployment occurs. A second plan failure blocks
that reference and stays in the complete ledger. Benign requires full validity.

Exact reference byte hash plus frozen state/config identity determines aliases;
no approximate deduplication. Each eligible unique reference gets one independent
official tracker, unchanged original capture inverse/roundtrip, original B,
physical u_minus and recorded controller memory. Original official selector,
H=5, dt=.1, 10 Hz solves, 60 Hz exact held-command integration and 3 s horizon
remain: 30 solves, 180 ticks, 181 states. Two types of MPC calls are counted
separately: primary solves and one historical-input audit per eligible event.
A/B historical commands are not used to integrate new executions. No GP body
velocity is fed forward. G3/DIAG-07 rollouts are read-only historical baselines.

The new execution wrapper composes DIAG-07's unchanged transport. Its restricted
label fields are normalized in a temporary copy when reusing the common saved
state/command/selector audit; the G4 admission itself is independently checked.
Every actual reference/state/command/prediction/timing record is checked without
normalization. No tracker override, monkey patch or external edit occurs.

The execution evaluator still requires original .15 m / 15° goal, last .20 s
dwell, clearance, known workspace, route, command motion and controller validity.
`e_plan = p_GP-goal`, `e_track = p_exec-p_GP`, `e_exec = p_exec-goal` must close as
vectors. Norms, dot product, angle and reserves are measured; scalar norms are
not added as the actual goal error. The .11+.04=.15 triangle bound is conditional
on measured tracking remaining <=.04, not assumed. Optimization changes the
whole path, so endpoint reserve is not claimed as the unique old failure cause.

## Commands

From repository root; original environments and dependencies are unchanged:

```bash
.venv/bin/python scripts/run_gp_se2_diag08.py prepare --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest tests/test_gp_se2_diag08.py -q
.venv/bin/python scripts/run_gp_se2_diag08.py derivatives --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
# Commit implementation/protocol before primary.
.venv/bin/python scripts/run_gp_se2_diag08.py freeze --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
.venv/bin/python scripts/run_gp_se2_diag08.py solve --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
.venv/bin/python scripts/run_gp_se2_diag08.py plans --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
.venv/bin/python scripts/run_gp_se2_diag08.py audit --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
.venv/bin/python scripts/run_gp_se2_diag08.py execute --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
.venv/bin/python scripts/run_gp_se2_diag08.py evaluate --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
.venv/bin/python scripts/plot_gp_se2_diag08.py --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
.venv/bin/python scripts/validate_gp_se2_diag08.py --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
.venv/bin/python scripts/plot_gp_se2_diag08.py --package --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

All redirected stage logs are kept outside hashed live output inventories until
the producing process has closed. Static plots and numeric/source sidecars cover
all five conditions, including ineligible/N/A execution. No GUI, RGB, VLA,
new online episode or historical rerun is permitted. Unit fixtures are synthetic
implementation checks only. The existing full pytest suite includes one real
historical MPC audit; it must be counted separately from primary execution and
its record preserved. A full-suite runtime audit is not an additional condition.

Pre-execution verification: all 15 actual points pass with literal G3 primal
and Jacobian prefixes. Actual M2/M3 dimensions are 150/30/1297 and 150/30/1387.
Maximum endpoint AD/NumPy primal error is 6.439293542825908e-15; maximum endpoint
directional error over the two finer steps is 2.17455645157294e-9. The gate took
10.267701 s. Source inventory preserves 31,457 files and both unrelated user
config changes. No primary solve occurred before the implementation commit.

Pre-freeze implementation validation: 32 new tests passed; combined DIAG-04/05/06
and DIAG-08 relevant set passed 84 tests in 28.75 s. Synthetic fixture solves do
not form actual-event evidence. No actual MPC or primary GP solve ran in this set.
