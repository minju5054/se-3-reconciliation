# GP-SE2-DIAG-06: frozen extremum-witness motion inequalities

Question: can one frozen refinement close the known between-point speed and
acceleration gaps without adding lateral equalities or changing acceptance?
A secondary question is whether only lateral velocity then remains invalid.
This is a bounded development-event diagnostic, not an adaptive algorithm.

Starting revision: `f86adffc4d54ae09a3581f557a9fc3eb77c07ed4`.
Historical G0/G1: DIAG-04 `primary_20260920T015400Z`; historical G2:
DIAG-05 `primary_20260920T064500Z`, execution revision
`b84d81598591eb3fc9ba017a0c251cc59bc528df`. Original inputs and seed files
resolve to GP-SE2-02 `primary_20260919T024000Z`. Authoritative validations,
numerical core, environment provenance, historical outputs and unrelated
Stage-0 config edits are hash-preserved. No historical numerical file changes.

## Frozen protocol

DIAG-03 established collocation/interior gaps with consistent interpolation.
DIAG-04 added quarter lateral and motion rows and encountered numerical failures.
DIAG-05 removed only quarter lateral rows: convergence recovered but hard
speed/acceleration and lateral failures remained. G3 adds only observed
motion-inequality extrema from the four saved hard G2 latest vectors.

Exactly five new starts, sequentially, once:

1. Hard `episode_013_repeat_01/handoff_024`, M2 / I0_FRESH.
2. Same hard event, M2 / I1_DECEL.
3. Same hard event, M3 / I0_FRESH.
4. Same hard event, M3 / I1_DECEL.
5. Benign `episode_001_repeat_01/handoff_002`, M3 / I1_DECEL.

Original initialization .npy bytes must match GP-SE2-02 and DIAG-05. No G2
final, witness or perturbation is an optimizer seed. G0/G1/G2 are saved baselines,
not rerun. Four hard starts are one event, not independent handoff evidence.

The original 31 supports / 150 right-local variables / 3 s / 0.1 s, initial B
and physical body twist, F_common, goal, objective, GP interpolation, weights,
environment and all physical/numerical tolerances stay fixed. World poses are
T_world_agent in metres/radians. Body twist is vee(X^-1 dX/dt); body acceleration
is the derivative of twist components, not world acceleration. Downstream
trajectory times are not intrinsic LightNav timestamps. No re-anchoring occurs.

Witness extraction uses the exact DIAG-05 finest trace: 1 ms plus 0.371 offset,
supports/midpoints and both knot sides. It checks literal sample and value
agreement with the saved trace. Per signed motion family, each connected
sequence of margins < -original inequality tolerance contributes one minimum-
margin sample; ties use earliest time then canonical index. Lateral velocity
and feasible near-active rows never generate witnesses. The common union key
is (family, interval, canonical sample index), without float rounding. Each
witness adds only its generating signed inequality, never all eight families.
The same frozen union applies to all five starts, including benign.

The source precheck observes 12 generating runs and three unique rows:

| Signed family | Interval | Canonical index | Time s | Local u |
|---|---:|---:|---:|---:|
| linear_speed_lower | 4 | 965 | 0.480 | 0.7999999999999996 |
| linear_acceleration_lower | 0 | 76 | 0.038 | 0.37999999999999995 |
| linear_acceleration_lower | 1 | 276 | 0.13737100000000002 | 0.37371000000000015 |

These are **observed extremum witnesses**, not exact continuous extrema.
The generated manifest, not a hard-coded expected count, is authoritative.
G3 dimensions are 150 variables, 30 equalities, and 1296 M2 / 1386 M3 inequalities.
No equality, environment, goal or a_y condition is added.

G3 delegates all G2 primal/derivative prefixes literally. A separate CPU
float64 JAX graph uses the frozen Lie-group interpolation primitives only for
the scalar witness suffix, retaining right-chart chain rule, full acceleration
terms and original branch guards. No FD fallback or solver scaling.

The pre-solve gate uses five original seeds, five saved G2 latest vectors and
five unchanged DIAG-04 fixed perturbations. Three original directions and FD
steps 2e-4, 2e-5, 2e-6 are retained; both finer steps must pass. Primal absolute/
relative tolerances: 1e-8/1e-10; constraint derivatives: 2e-5/2e-5; original
objective derivative: 2e-4/2e-5. These diagnostic thresholds do not change
physical acceptance. Failed validation blocks all primary optimization.

Installed SLSQP, supplied objective/equality/inequality derivatives, 200
iterations, ftol=1e-7, 30 s prepared budget and single-thread BLAS stay fixed.
Provider construction/compilation, solve and post-check costs are separate.
The unchanged DIAG-04 harness inspects initial/latest/actual grid-feasible
callbacks, selecting minimum original objective then lexical source label only
among grid+dense-valid candidates. Original check_full_candidate receives the
base GPProblem. Full rejection does not trigger another selection or fallback.

Post-solve queries cover the whole unchanged sampled grid and both knot
acceleration sides, not just witness locations. A repaired witness with a new
motion violation at another canonical sample is classified as relocation;
overlap with the old observed violating run is separately recorded. No second
witness round, restart, tolerance change or new execution is permitted.

## Commands and evidence

From repository root, for the exclusive run
`data/robotless_gp_se2_diag_06/primary_20260920T083200Z`:

```bash
.venv/bin/python scripts/run_gp_se2_diag06.py prepare --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
.venv/bin/python scripts/run_gp_se2_diag06.py derivatives --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
# Commit verified implementation/protocol before freeze and solve.
.venv/bin/python scripts/run_gp_se2_diag06.py freeze --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
.venv/bin/python scripts/run_gp_se2_diag06.py solve --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
.venv/bin/python scripts/run_gp_se2_diag06.py analyze --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
.venv/bin/python scripts/run_gp_se2_diag06.py validate --run data/robotless_gp_se2_diag_06/primary_20260920T083200Z
```

The source/union/protocol/code hashes are frozen before primary solves. Every
PNG has numeric/hash sidecars. Outputs include twenty historical/new outcome
rows, five paired results, per-vector interval analyses, witness margins,
remaining runs, timing, static index and compact review ZIP. No historical
arrays are bulk-copied; no VLA/MPC/rollout/GUI is executed.

Implementation status at freeze: tests and actual derivative gate precede the
five primary starts; numerical results and interpretation will be appended.

Pre-execution gate: all fifteen actual records passed with original/G2 literal
primal and derivative-prefix parity. The witness union contains three rows,
with 1,806 preserved-file hashes. Eighteen new implementation tests and 65
related DIAG-04/05 tests pass. No primary G3 solve had run at this freeze.
