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

## Completed result

Operational: **GP_SE2_DIAG_08_COMPLETED_WITH_LIMITATIONS**.
Interpretation: **ENDPOINT_RESERVE_NOT_PLAN_FEASIBLE**.
G4 numerical execution/freeze SHA: `5e2da93927196fb65cb5a260b2f0c40adbf7cc0d`.
The final report SHA and normal push are recorded in the ignored `completion.json`.

Exactly five new G4 SLSQP calls ran, once each. All terminated status 0 and
passed the G4 solver grid. However, all four hard latest plans fail the original
full checker on **lateral velocity and forward speed**. No full-valid hard
candidate is selected, and no hard G4 reference is execution-eligible. Thus hard
MPC execution and tracking/error recovery are **N/A**, not four executed failures.
The one full-valid benign reference was newly executed and succeeds.

This result does not prove mathematical infeasibility or a solver failure.
The endpoint was tightened to the 11 cm boundary within the original solver
allowance, but the frozen finite motion witnesses no longer preserve the
previous nonlateral sampled feasibility. The original full checker correctly
catches this second failure. Margin efficacy for hard execution remains untested.

### All five paired plans

G3 is saved DIAG-06; G4 is new. Objectives compare the same original objective,
not navigation quality. Every entry has status 0; hard G4 full/selected are FAIL.
Benign full/selected are PASS. The retained benign source is `callback_0122`,
identical to latest and different from its original seed (J_seed=3.9633655920).

| Start | G3 → G4 iterations | G3 → G4 J latest | G3 endpoint m | G4 direct endpoint m | G4 full failure | G4 MPC |
|---|---:|---:|---:|---:|---|---|
| Hard M2/I0 | 141 → 141 | 16.3435524298 → 16.3777371924 | .133143779637 | .11000000007647903 | lateral + speed | N/A |
| Hard M2/I1 | 144 → 142 | 16.3435524336 → 16.3777372108 | .133142577887 | .11000000000501670 | lateral + speed | N/A |
| Hard M3/I0 | 143 → 140 | 16.3435524341 → 16.3777371898 | .133145685727 | .11000000000955860 | lateral + speed | N/A |
| Hard M3/I1 | 140 → 143 | 16.3435524268 → 16.3777372186 | .133145424516 | .11000000000466847 | lateral + speed | N/A |
| Benign M3/I1 | 122 → 122 | .194335066666 → .194335066666 | .000039736700 | .00003973669995537 | none | PASS |

The original checker re-evaluates its endpoint through the historical sampling
interface, so its distance can differ from direct support distance by a few
femtometres. Both recorded values remain unchanged; this is not re-anchoring.

The four hard nominal .11 m excesses range 4.6685e-12–7.6479e-11 m. Squared
margins range -1.0271e-12–-1.6825e-11 m², far inside the unchanged 1e-5 m²
solver allowance. The predeclared strict nominal reserve predicate nevertheless
returns false; this is an additional recorded admission limitation. **Even if
that arithmetic-scale boundary issue were set aside, all four still fail the
unchanged forward-speed predicate.** No tolerance was increased after results.
The literal frozen per-start taxonomy is `MARGIN_PLAN_INFEASIBLE_OR_SOLVER_FAILURE`;
its broad name is not a claim that SLSQP failed. `ineligibility_layers.csv`
separates status 0, numerical-row pass, nominal reserve and material motion failure.

### Reappearing between-point speed violation

All hard G4 sampled minima occur in interval 4, t=.482 s, u=.82, after the
unchanged v_x witness at .480 s, u=.80. G3's sampled minimum was t=.479 s.
All three original witnesses remain within original tolerance. The maximum
linear acceleration remains approximately 2.000009493 m/s², inside the original
2+1e-5 allowance; max speed=.8 m/s, angular speed≈1.38035 rad/s, angular
acceleration≈5 rad/s² also pass. Original goal/yaw, workspace, obstacle, route,
boundary and derivative-identity flags pass. The original vy threshold remains
1e-5 m/s; no lateral relaxation or promotion occurs.

| Hard start | G3 min v_x m/s | G4 min v_x m/s | G4 excess beyond original allowance m/s | G4 max abs(v_y) m/s |
|---|---:|---:|---:|---:|
| M2/I0 | -5.1288934745e-6 | -1.6876063496e-5 | 6.8760634958e-6 | .000111345288058 |
| M2/I1 | -5.3319501396e-6 | -1.7733741164e-5 | 7.7337411644e-6 | .000111296274569 |
| M3/I0 | -4.9741474660e-6 | -1.7124773453e-5 | 7.1247734530e-6 | .000111331624248 |
| M3/I1 | -5.2092751495e-6 | -1.7397530108e-5 | 7.3975301079e-6 | .000111273991752 |

`presentation/hard_forward_speed_gap.png` shows all four curves and the frozen
witness versus original threshold. This is sampled interior failure, not a
continuous maximum proof. G4 mean FRESH translation deformation is approximately
.252764 m versus G3 .262763 m, while mean absolute yaw deformation increases
from .256639 to .26849 rad. A changed endpoint also changes the entire reference;
these secondary numbers neither certify feasibility nor establish execution gain.

### Execution, geometry and benign control

Only benign is eligible; exact deduplication gives one new unique reference.
One protocol historical-input audit passes, then one fresh tracker performs
30 primary solves / 180 integration steps / 181 states. Four hard ledger entries
are retained with no fabricated execution. Historical G3 is never rerun.

The new benign reference, states, commands, selected targets and saved predictions
are literally equal to their historical G3 counterparts (timings excluded).
This is measured reproduction from a new GP solve and new MPC instance, not a
copied rollout or improvement claim. Benign plan endpoint is .0000397367 m from
goal, plan reserve .1499602633 m, max |vy|=2.19723e-8 m/s, and full acceptance
passes. Objective remains .194335066666, improved from the same original seed
but not from G3.

Benign execution: final position error .0163533983543 m; yaw error
.00005634901679 rad (.00322856°); first goal entry/time-to-goal 2.595 s;
achieved sampled final dwell .405 s (required .20 s). Minimum footprint-edge
clearance 1.3269449699 m; workspace, route, command motion and controller pass.
No collision or controller failure. These cases have no required gate; route
PASS is not evidence of an executed gate crossing.

Benign first command is [.799999994626, .000035976876]; first physical-command
change [-5.37442e-9, .000150720787]. Command TV is .6086549580 m/s and
.000428216116 rad/s; max control-grid accelerations 1.9999987193 m/s² and
.001507207873 rad/s². Actual path length is 1.1573315863 m. First selected target
inside the original goal region is 2.1 s; first final-reference row is 2.5 s,
then maintained. These match G3. Hard G4 selector/prediction/command histories
are N/A, with the historical G3 histories retained as context only.

For historical hard M2/I0, the terminal vectors [world x, y] in metres are:

- e_plan = [-.129590962946, .030552387451]
- e_track = [-.032432247465, .020674675836]
- e_exec = [-.162023210411, .051227063287]

Their norms are .133143780, .038461577 and .169928611 m, with dot product
.004834586886 m² and angle .335986065 rad. Norms are not summed as the error.
For newly executed benign, e_plan=[-1.507918727e-5,-3.676443165e-5],
e_track=[-1.218368010e-5,.016390140061], e_exec=[-2.726286737e-5,.016353375629].
Tracking norm is .0163901445893 m, dot=-6.023904640e-7 m², angle=2.751617710 rad,
execution reserve=.133646601646 m. Vector closure is literally zero for every
available old/new execution. Hard G4 e_plan is saved; e_track/e_exec/angle are
N/A with reasons in `endpoint_geometry_partial.csv`. No unmeasured G4 tracking
bound or execution outcome is inferred from the triangle inequality.

### Actual compute and runtime accounting

| G4 start | Construction s | Compile/warmup s | Prepared solve s | Candidate checks s | Cold per-start sum s |
|---|---:|---:|---:|---:|---:|
| Hard M2/I0 | .1590 | 1.9442 | 4.2914 | 1.6547 | 8.0555 |
| Hard M2/I1 | .0562 | 1.7526 | 4.1661 | 1.5025 | 7.4828 |
| Hard M3/I0 | .0592 | 1.7774 | 4.7131 | 1.3319 | 7.8870 |
| Hard M3/I1 | .0598 | 1.7237 | 4.4699 | 1.6860 | 7.9447 |
| Benign M3/I1 | .0575 | 1.7622 | 2.6577 | 8.2052 | 12.6880 |

Prepared solve sum 20.298129 s; cold per-start sums 44.058008 s; entire five-start
stage 51.998327 s including serialization/orchestration, with environment load
.037593 s. Compilation/warmup sum 8.960050 s and post-candidate validation
14.380438 s are nested in cold totals, not added twice. Each original G3 provider
is constructed/warmed per start; the new endpoint row needs zero extra graphs.
Primal cache misses total 1,914; original objective/equality/inequality call
counts per start are 373/374/374, 379/380/380, 376/377/377, 377/378/378,
409/410/410. Each derivative callback is called 688 times across five starts.
Inner environment/interpolation counts remain unavailable in the reused harness.

Source prep 8.409720 s; actual derivative gate 10.267701 s; interval/plan analysis
11.502602 s; execution evaluation 6.176682 s; fifty-figure rendering 6.201540 s;
supplemental report 6.665882 s. Benign rollout wall .129508 s, including .122814 s
of official solve computation; protocol audit worker .070558 s. Original
saved-record validator 49.516080 s and additive authoritative verification
11.222977 s are separate validation costs. Full tests take 177.34 s. Setup,
validation and tests are not included in the 30 s prepared solve budget, and no
wall time is added to the 3 s simulated trajectory. Unrecorded process startup
and manual review are not presented as an exact end-to-end total.

Actual task totals: **5 new GP solves**, **1 new rollout**, **30 primary MPC
solves + 1 protocol historical audit + 1 existing full-suite historical audit =
32 actual MPC calls**. No GP retry, G3 rerun, VLA inference, RGB, GUI or online
episode. Synthetic implementation fixtures are not actual event samples.

### Preserved technical failures and authoritative verification

The first isolated MPC audit/execute worker imports failed before tracker or
MPC creation: importing the new combined endpoint module pulled SciPy into the
pinned official MPC environment, where it is not installed. Both tracebacks
are retained in `technical_failure/`; actual solve/rollout count at failure was
zero. No dependency/environment was changed. An additive SciPy/JAX-free bridge
and worker reuse the same DIAG-07 execution functions; 11 tests verify identical
admission and transport behavior, including import isolation in the official
venv. The frozen G4 module, five numerical results and failed worker are untouched.

Execution completion has a separate run ID:
`data/robotless_gp_se2_diag_08/runtime_20260920T153000Z/`.
`runtime_freeze.json` records exact additive code hashes before its first MPC
call and links the parent GP freeze SHA. Those additions were not yet committed
at MPC runtime; they are included in the final report commit. This provenance
is explicit rather than claiming they ran verbatim at the original SHA. Primary
`rollouts/` and `historical_audits/` are documented path links to that new runtime,
not copied historical results. Only the previously unstarted benign audit and
rollout ran. Correction commands:

```bash
PYTHONDONTWRITEBYTECODE=1 /home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/.venv/bin/python scripts/lightnav/gp_se2_diag08_mpc_completion.py --run data/robotless_gp_se2_diag_08/runtime_20260920T153000Z --stage audit
PYTHONDONTWRITEBYTECODE=1 /home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/.venv/bin/python scripts/lightnav/gp_se2_diag08_mpc_completion.py --run data/robotless_gp_se2_diag_08/runtime_20260920T153000Z --stage execute
.venv/bin/python scripts/complete_gp_se2_diag08_report.py --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
.venv/bin/python scripts/finalize_gp_se2_diag08.py --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
.venv/bin/python scripts/finalize_gp_se2_diag08.py --package --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z
```

The frozen validator recomputed 79 retained source labels, every original full/
dense/interval check, retention, state integration, commands, selection, metrics,
plots and tables. Its only four errors were descriptive `plan_failure_reason`
list comparisons: `+=` mutated a shared list in the validator, whereas the
producer correctly created a separate supplemental-reason list. The original
30,016-check report and source remain preserved, `valid=false`. This reporting
error did not change eligibility, plan validity, execution or numerical results.

**Final authority: `verification/validation.json`, valid=true.** Its restricted
completion accepts only those four exact errors, explicitly reproduces the
aliasing, checks nonmutating parity, rechecks execution literally and verifies
all source/runtime/plot hashes. No scientific threshold changes or new solves.
The original failed report is not silently relabeled authoritative.

Full pytest: **2,393 passed / 19 existing skips**, 177.34 s. Two subsequently
added report-only regression tests pass separately; no second full-suite MPC
call is implied. Compileall and diff checks pass; no shell launcher changed.
The test-only real MPC audit is copied from pytest temporary output before
retention, with its SHA and task call ledger. Generated data/PNG/ZIP, external
source, environments and the two unrelated Stage-0 config edits stay uncommitted.

### Review and remaining uncertainty

All 50 primary figures plus one motion-gap figure retain original units and
numeric/hash sidecars. Hard G4 curves are plan-only/not-executed; N/A executions
have no fabricated lines. Goal figures distinguish .15 m execution and .11 m
planning circles from footprint/safety geometry. The readable entry is
`presentation/index.html`, full comparison `index.html`, mechanism figure
`presentation/hard_forward_speed_gap.png`, compact `review_bundle.zip`.
The ZIP excludes raw RGB, environment export and checkpoints. Authority and
both technical corrections are included. No GUI or video was run.

Confirmed: tightening the endpoint changes a previously repaired nonlateral
interior motion pocket; a fixed witness is not invariant to the new optimum.
The existing interpolation/Jacobian was not changed, and all five solves
converge. Unknown: whether a reserve-qualified, nonlaterally valid hard G4
reference would recover actual execution goal/dwell. No such execution occurred.
The larger reserve is neither validated nor disproved as an execution remedy.
Lateral rejection still applies and is not identified as an execution cause.

**Next single experiment proposed, not implemented:** keep the 11 cm endpoint,
all original tolerances and this controller interface fixed, and test one
predeclared forward-speed witness refinement at the newly observed .482 s
pocket. Question: can the nonlateral full-check gap close without another
relocation, permitting the originally intended execution comparison? This is
not a recommendation to tune weights, relax lateral tolerance, change SLSQP,
or claim 4 cm is generally optimal.

Visual review checked all ten plot types, benign execution and the four-panel
speed-gap figure. Five additional tracking-distance panels wrap a long legend
that was clipped at the edge of the original layout; only presentation changes,
with exact source-sidecar equality verified. Their index is
`presentation/tracking_panels/index.html` and original images remain untouched.
The packet therefore contains 51 substantive figures plus five readable copies.
Command: `.venv/bin/python scripts/present_gp_se2_diag08.py --run data/robotless_gp_se2_diag_08/primary_20260920T151000Z`.
