# STATE_SHIFT_TRANSPORT_SCALE_01

## Frozen protocol

Starting fetched HEAD and origin/main: `0db03d81ee33605ac96fc2d0886364b8499874e9`.
**Timing-controlled offline causal reference comparison.** Diagnostic experiment,
with exactly four fixed sources from RELATIVE_FACTOR_MULTISOURCE_01:
S1 OSA03_R00; S2 episode_001_repeat_01/handoff_013;
S3 episode_008_repeat_01/handoff_023; S4 episode_013_repeat_00/handoff_020.
No acquisition, source search, source replacement, alpha tuning or repetitions.

Run: `data/state_shift_transport_scale_01/primary_20260930T080000Z`.
Directory name is a fixed identifier; actual UTC/host and simulation clocks are
recorded separately. Arrays, solver traces and rollouts stay under ignored data.
Small numeric summaries and exactly four final PNGs are tracked.

### Fixed spatial problem

A and B are fixed. The **observation-to-application state-shift / transport factor**
moves editable FRESH nodes toward a transported target:

```
G = B A^-1; G_alpha = Exp(alpha Log(G)); Ftilde(alpha) = G_alpha F
r_S = Log(Ftilde(alpha)^-1 X)
r_R = Log((F_j^-1 F_(j+1))^-1 (X_j^-1 X_(j+1)))
r_A = Log(F_j^-1 X_j)
E(alpha) = E_S(alpha) + E_R + E_A
s = cumulative original-FRESH XY arc / total original arc
w_S = (1-s)^2; w_A = s^2
normalization = diag(.10 m, .10 m, 10 degrees)
```

Use existing weighted node means / relative-edge mean and unit lambdas.
Historical L keys retain their code spelling and mean S above. A-local raw
`F=A F_local` is immutable; derived local arrays use `A^-1 X`. Never re-anchor
raw rows at B. World XY metres, yaw radians CCW, +Z up; local x forward/y left.
At alpha=1, `B^-1 Ftilde = A^-1 F`. Alpha=.5 intentionally relaxes this identity
using complete SE(2) Log/Exp, without independently scaling pose x/y/yaw.

Compare only Native (alpha=0), Half Local-SE2 (.5), Full Local-SE2 (1).
At alpha=0, X=F gives zero objective up to floating-point roundoff. Authenticate
and reuse the four Native references/rollouts/metrics and four Full
references/rollouts/metrics; no new Native or Full solve. Reuse the four Full
planning solutions. If reuse fails authentication, stop instead of rerunning.
The new subclass overrides the transported target only. All historical files,
LocalSE2 default arithmetic, solver, controller and validators remain unchanged.

Half starts from original FRESH once/source. Same right-local `X Exp(delta)` LM,
central FD 1e-6, max80, damping .001, reject x10/accept x.3, maximum1e12,
gradient/step1e-9, cost1e-12. No warm starts, retries or compensating factors.
No waypoint time/dt, GP, twist, velocity, controller, obstacle, correspondence,
projection-join, adaptive alpha or alternate selector. Spatial reference yaw
changes do not directly specify the robot's angular command.

### Authentication, schedule and safety

Authenticate tracked historical result summary, complete saved result hash ledger,
original source/code hashes, saved B/state/geometry, and unchanged old multisource,
R00 and R01 validators. Copy common state, schedules, scenario, metric protocol,
source audit/manifest and recorded OLD-to-B byte-for-byte; reference/rollout/metric
reuse points to exact old files. The named original-world file needed by the
existing runner is copied byte-for-byte, never reconstructed.

Restore B pose/tick/simulation time, physical u_minus provenance, already-applied
u_B_plus, official previous_control, generation/version/chunk and source dt exactly.
Use source-specific frozen submit/release pairs from historical schedule.json,
without generating a new schedule. Reuse the existing official worker and runtime
function directly. Simulation waits at the submit state; wall-clock completion
cannot advance simulation or choose application ticks. Official memory poll order,
nearest-row H5 selection and command computation remain unchanged. No future state.
Require exact attempted/accepted/application sequences, initial provenance, 54
primary and 180 full integration intervals. The source's final result may remain
withheld at the excluded cap; count it without inventing an application.

Same Hospital/cart full-polyline checker, circular radius .20 m, required edge
clearance .05 m, workspace and numerical reserve. LM accepts only objective decrease
and complete-reference feasibility. No B connector, crop, repair, smoothing or new
margin. Unsafe final references are recorded and skipped. Execution uses the exact
abort-only guard; never apply the rejected command or fabricate continuation.
A failed schedule gate bars reference-causal interpretation and is reported with
its actual cause, including scientific safety censoring.

### Evaluation and predeclared interpretation

All 18 primary metrics use ORIGINAL FRESH: initial position/yaw, max .5 s position,
initial growth, position/yaw AUC .3/.9, sustained attachment, fractional row, arc
fraction, remaining arc, swept clearance, endpoint, linear/angular TV, max |omega|,
termination. Same forward-only continuous projection and shortest yaw. Attachment
requires <=.10 m and <=15 degrees for the complete following .30 s sampled dwell.
Null remains null. Half own-reference position/yaw .3/.9 AUC and .5 s max error are
separate secondary metrics. Planning includes all node/edge/rigid-fit/clearance/
chord/segment/intersection diagnostics and separate factor costs.

Report `Log(A^-1 B)` and its translation norm as the body-motion measure in the
cross-source table; separately report world `Log(B A^-1)` and full/half transform
translations, which depend on world origin when yaw is nonzero. Also report full
and half body-motion translation norms and yaw. B projection onto ORIGINAL FRESH
is separately identified from each method's B projection. No continuous curve or
optimal alpha inferred from these three discrete conditions.

Selector CSV/JSON reads finished records only: every pose, nearest row, H5 indices,
command, application tick, withheld result. Compare Half-Native and Half-Full first
differing tick; report attachment co-occurrence without attributing causality.

Hypotheses overlap, so freeze this conservative classification precedence (numeric
sign tolerance1e-9 in each unit; no fitted thresholds and no combined winner score):

1. TECHNICAL_BLOCKED for source/state/schedule/controller/validator/history failure.
2. SOURCE_DEPENDENT_TRANSPORT (H3) if Half-Full .9 s position AUC improves in at
   least one source and worsens in another across S1-S4.
3. TRANSPORT_MAGNITUDE_INSUFFICIENT (H2) otherwise if any S2/S3/S4 fails to lower
   .9 s AUC, Native is lower than both in any S2/S3/S4, or safety is not preserved.
4. FULL_TRANSPORT_OVERCOMPENSATES (H1) otherwise: safety preserved, all problematic
   S2/S3/S4 AUCs improve, and Native is not strictly better than both there.

Also show every H1-compatible safe AUC improvement, all attachment availability,
changes and trade-offs regardless of category. This prevents an H2/H3 label from
hiding partial overcompensation evidence. Attachment is not imputed when null.
The .9 s AUC defines reproducible direction counts only; all requested metrics and
nine interpretation answers remain necessary. No optimum/final method claim.

### Freeze and output discipline

Prepare/authenticate with zero new science; run synthetic tests plus old regression
suite, compileall, diff/staged review; commit and normal push before four new Half
planning solves and four eligible Half rollouts. No Native/Full/Taper rerun.
Saved-only validation and reporting follow, with PNG inspection and numeric
sidecars. Update this report/log, review, commit and normal push the small results.

Exactly four final PNGs: world_execution_overview.png,
transport_scale_primary_metrics.png, transport_scale_deltas.png,
reference_transport_overview.png. Equal world axes, whole cart + safety region,
recorded OLD-to-B, original rows, distinct execution styles/markers, attachment,
minimum clearance, endpoints and numerical overlap annotations. No selector PNG.

## Repository-confirmed facts

Pending the pushed scientific freeze and single bounded execution.

## Source/state-shift table

Pending saved results; exact states/schedules/hashes are in the freeze ledger.

## Native vs Half vs Full results

Pending execution; no synthetic fixture is experimental evidence.

## Selector diagnostics

Pending saved results; diagnostic only, no control changes.

## Cross-source analysis

Pending execution and exact primary/full schedule gates.

## Research interpretation

Pending execution. This diagnostic does not select a final transport fraction.

## Limitations

Four fixed development sources, no population claim. Offline timing-controlled
scheduler, not real asynchronous deployment timing. Original FRESH has intrinsic
tracking difficulty. No controller-aware factor, online repeated chunks or real
robot. Geometry/progress and relative edges do not prove semantic intent.

## Not demonstrated

Optimal alpha, final method superiority, general graph superiority, online latency
improvement, real-world benefit, complete navigation/bypass, population effects,
or a correspondence solution.

## PNG paths

All under `/home/gpuadmin/Workspace/se-3-reconciliation/results/state_shift_transport_scale_01/figures/`.
The four exact full paths will be printed with the completed results.

## Reproduction commands

Commands run from `/home/gpuadmin/Workspace/se-3-reconciliation` using the existing
repository environment. Official MPC uses its existing external environment without
modification. All new scientific work occurs only in the single `execute` command
following the pushed freeze.

```bash
git fetch origin main
git status --short --branch
git remote -v
git log -1 --format=fuller
git rev-parse origin/main
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_state_shift_transport_scale01.py --run data/state_shift_transport_scale_01/primary_20260930T080000Z --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/transport-scale-mpl .venv/bin/python -m pytest -q tests/test_state_shift_transport_scale01.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/transport-scale-mpl .venv/bin/python -m pytest -q tests/test_state_shift_transport_scale01.py tests/test_relative_factor_multisource01.py tests/test_osa03_relative_factor_replication01.py tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py tests/test_handoff_delay_attribution.py tests/test_genuine_source_scan.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_state_shift_transport_scale01.py --run data/state_shift_transport_scale_01/primary_20260930T080000Z --mode freeze
git diff --cached --check
git commit -m "Freeze four-source half-transport diagnostic with authenticated historical reuse"
git push origin main
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_state_shift_transport_scale01.py --run data/state_shift_transport_scale_01/primary_20260930T080000Z --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_state_shift_transport_scale01.py --run data/state_shift_transport_scale_01/primary_20260930T080000Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/transport-scale-mpl .venv/bin/python scripts/report_state_shift_transport_scale01.py --run data/state_shift_transport_scale_01/primary_20260930T080000Z
```

Preparation also ran direct saved-only historical ledger/validator authentication
before implementation. No numerical solver or controller was called by that audit.

### Pre-execution verification

New focused tests: **25 passed (29.30 s)**. Relevant regression including historical
Local-SE2/common-B/multisource/R00/R01, frame, safety, controller and timing tests:
**492 passed, 1 skipped (91.36 s)**. The skip is absent ignored historical
EXP-01B/EXP-02B corpus. Compileall and diff checks pass. Preparation and historical
saved validation made zero new scientific calls. No scientific outcome exists yet.
