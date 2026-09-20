# GP-SE2-DIAG-05: quarter motion inequality-only ablation

Question: does retaining the original midpoint lateral equality and adding only
quarter motion inequalities remove DIAG-04's numerical regression, and recover
an independently full-valid hard-handoff GP candidate? If convergence returns,
does between-point lateral velocity remain the only full-check failure?

Starting revision: `070f2ca5cff44fcd3027e86fd198e8ebb1addd57`.
Historical baseline: `data/robotless_gp_se2_diag_04/primary_20260920T015400Z/`.
Its execution revision is `e3776fa30a161ff361ba5f1d321a8caa788e729d`;
its final report revision is the starting revision above. G0/G1 are saved
historical results, not new solves. Original source and the preceding saved
interpolation audit remain GP-SE2-02 and GP-SE2-DIAG-03 respectively.

## Frozen single change

DIAG-03 found collocation-pass/interior-fail paths with consistent interpolation
and derivatives. DIAG-04 added quarter lateral equalities and motion inequalities
together; full recovery was absent and SLSQP numerical failures appeared.
This ablation separates those additions. It changes no trajectory representation,
objective, initialization, physical tolerance, environment query, or controller.

| Formulation | Equality rows | M2 inequality rows | M3 inequality rows | Evidence |
|---|---:|---:|---:|---|
| G0 original | 30 | 813 | 903 | Historical DIAG-04 |
| G1 quarter full | 90 | 1293 | 1383 | Historical DIAG-04 |
| G2 quarter inequality only | 30 | 1293 | 1383 | Five new starts |

All use 150 variables, 31 support states, 3 s horizon, 0.1 s support spacing,
fixed B and initial body twist. Poses are T_world_agent, with the original
right-local chart; body twist is vee(X^-1 dX/dt). Units are metres, radians,
seconds, m/s, rad/s, m/s² and rad/s². Body acceleration is the time derivative
of twist components. No re-anchoring, support change, intrinsic LightNav time
interpretation, terminal-stop constraint, slack, or soft lateral penalty occurs.

`InequalityOnlyView` delegates objective/equality to GPProblem literally and
appends only the original DIAG-04 `quarter_motion_values` inequality outputs.
At u=.25,.75 of each interval, eight signed margins enforce forward speed,
angular speed, linear body acceleration and angular acceleration. There is no
quarter lateral equality, a_y bound, new goal or environmental row.

The new derivative wrapper directly returns the original objective gradient
and equality Jacobian. It reuses the frozen quarter AD implementation for the
inequality suffix. Its internal graph also calculates quarter lateral outputs;
these unused values and Jacobian rows are never exposed to SLSQP. No FD fallback
or derivative guard removal is allowed. Original numerical modules are unchanged.

## Schedule and verification gate

Exactly one new G2 solve per row, sequentially:

1. `episode_013_repeat_01/handoff_024`, M2, I0_FRESH.
2. Same hard event, M2, I1_DECEL.
3. Same hard event, M3, I0_FRESH.
4. Same hard event, M3, I1_DECEL.
5. `episode_001_repeat_01/handoff_002`, M3, I1_DECEL (benign control).

Initialization files must be byte-identical to both DIAG-04 pairs and the
original GP-SE2-02 saved seed. Historical final vectors are verification inputs
only. The four hard comparisons are one handoff, not four independent events.
No retry, additional start, G0/G1 reoptimization, VLA, MPC, rollout or GUI is used.

The exact saved DIAG-04 fifteen-point manifest and three directions are reused:
five seeds, five historical latest vectors and five fixed perturbations. Required
literal parity covers objective, equality, equality Jacobian, objective gradient,
original inequality prefix and its Jacobian. Appended values are compared with
NumPy interpolation. Family-wise directional checks use steps 2e-4, 2e-5, 2e-6;
both finest steps must pass. Frozen absolute/relative tolerances remain 1e-8/1e-10
for primal values, 2e-4/2e-5 for objective derivatives, 2e-5/2e-5 for constraint
derivatives. Branch-aware original environment verification is retained.
Failure blocks all primary solves; thresholds cannot be relaxed after inspection.

Prepared SLSQP settings: 200 iterations, ftol=1e-7, 30 s per start, supplied
objective gradient and both Jacobians, CPU float64 and single-thread BLAS.
DIAG-04's `run_refined` is imported unchanged. Graph construction, first-call
compilation, prepared solve and post-check costs are recorded separately.
Historical timing is context rather than a newly paired runtime measurement.

## Candidate, rank and analysis policy

The unchanged solver helper retains initial/latest and actual grid-feasible
callbacks. It selects the lowest objective among grid- and original-dense-valid
candidates, breaking ties by source label. Independent full acceptance does not
silently substitute a different candidate. `check_full_candidate` receives the
original base GPProblem. A new grid-pass path remains rejected if original
lateral, speed, acceleration, goal, route or environmental checks fail.

Every retained vector receives original full checks and DIAG-03 interval
analysis; duplicated vectors can share calculations while source labels remain.
Latest and selected are separate. A null selected vector is never filled with RAW.
The latest outcome classes distinguish grid failure, lateral-only full rejection,
other full rejection and full validity. Lateral-only requires all other full-grid
flags, original dense inequalities and pose/body derivative identity to pass.

At initial/latest/selected, G2 and G0 equality Jacobians must be literally equal
at the SAME vector (30×150). Raw and column-scaled SVD use DIAG-04's unchanged
protocol, with no scaling sent to the solver. G0/G2 final vectors from different
optimizations need not have identical spectra. G1 has 90 equality rows.

Quarter margin diagnostics use `abs(margin) <= original inequality tolerance`
for active-within-tolerance counts and `margin <= max(1e-6,10*inequality_tolerance)`
for near-active counts, with violating and feasible-near-active counts separated.
These are descriptive saved-margin counts, not observed SLSQP internal active sets.

Benign numerical objective improvement must exceed
`max(10*ftol,1e-8*max(1,abs(J_seed)))`; raw differences are retained. Seed retention,
solver convergence, numerical objective improvement and full feasibility are
separate. None is navigation evidence. Supplemental sampled extrema are not a
continuous-time feasibility certificate. No next refinement is implemented here.

## Reproduction commands

The runner refuses existing outputs and any second solve-phase invocation.
Use the run path recorded in the completed-results section:

```bash
.venv/bin/python scripts/run_gp_se2_diag05.py prepare --run "$run"
.venv/bin/python scripts/run_gp_se2_diag05.py derivatives --run "$run"
.venv/bin/python scripts/run_gp_se2_diag05.py solve --run "$run"
.venv/bin/python scripts/run_gp_se2_diag05.py analyze --run "$run"
.venv/bin/python scripts/run_gp_se2_diag05.py validate --run "$run"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

The saved-record validator recomputes checks, interval diagnostics, equality
parity/rank, retention, tables and plot sidecars, without new optimization. All
fifteen historical/new results appear in the static index. Every PNG has numeric
and source/config hash metadata; plans are labelled **PLAN ONLY / NOT EXECUTED**.
Raw data, environment exports, external MPC, historical solver arrays and
unrelated local edits are preserved; only new code/tests/docs are committed.
