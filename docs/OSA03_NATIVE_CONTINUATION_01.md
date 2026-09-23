# OSA03_NATIVE_CONTINUATION_01

## Frozen question and source

From the exact genuine OSA03 REPEAT_00 switch boundary and saved controller
phase, what does unchanged Native tracking of original FRESH execute without a
new VLA update or reconciliation? This is one **offline Native-only continuation**,
not a new online navigation episode or a method comparison.

Starting/fetched main: `565e40a586e5f8081339c44d8f2949c5abffce9a`.
Authoritative source: `data/obstacle_source_acquisition_03/primary_20260923T085200Z/`.
Representative remains REPEAT_00; source classification remains
`QUALIFIED_GENUINE_OBSTRUCTED_OLD_HANDOFF_SOURCE`.
Both unrelated Stage 0 config edits are preserved and excluded from commits.

New exclusive output: `data/osa03_native_continuation_01/primary_20260923T103000Z/`.
Raw OLD SHA256: `6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae`.
Raw FRESH SHA256: `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521`.
These are loaded/checked against the sealed source, not substituted constants.
Whole OSA03 files, source geometry exports and unrelated configuration hashes
are recorded for before/after preservation. No source artifact is overwritten.

World is Isaac Z-up, metres, yaw CCW radians; local x forward, y left.
Original FRESH world rows remain `T_world_observation * T_observation_row`.
No resampling, cropping, B re-anchor, prepared reference, REF-02 selector or Oracle.
LightNav row order has no intrinsic timestamps. **3 s is an execution observation
cap, not a timestamp or duration attached to the LightNav rows.**

## Saved-only controller phase gate

B is state/tick 92 at source simulation time `1.5666667483747005` s:
`[19.203630553052257, 24.14333536015282, -1.5689760264727979]`.

| State at the switch | [v m/s, omega rad/s] |
|---|---|
| Physical u_minus | [0.8, -0.0000015584691584098122] |
| First FRESH command u_B_plus | [0.8, 0.49999844893907003] |
| Controller previous_control at B | [0.8, 0.49999844893907003] |

Historical solve 000005 was submitted at tick90, then applied at B/tick92.
Next original grid submit was tick96 (solve000006), applied at tick97.
Saved postroll ends at tick98. Six exact held-command integration intervals
cover `.10000000521540642` s. Next **new** submit is tick102 on the original
absolute grid, not at B or at saved-prefix termination.

Prefix validation replays the two **saved accepted results** at their recorded
application ticks through existing CommandActivation and independently integrates
all six steps. Command identity/value/hold phase, selected rows and pose parity
are checked. This is not an independent numerical re-solve of the two historical
MPC outputs. There is **no solve at B** and no solve of REPEAT_01.
Unresolved source phase stops the experiment; prefix mismatch invalidates it.
No state adjustment or tolerance tuning to force parity is permitted.

A zero-solve installation/restoration preflight imports the pinned official
tracker with `MPCController.solve` disabled in that test process. Exact source
installation and both saved-result restorations passed; world error is recorded
in `restoration_preflight.json`. It neither edits external source nor evaluates
a new controller command. Research unit fixtures are not experiment evidence.

## Execution protocol frozen before scientific solves

`configs/osa03_native_continuation_01.yaml` defines one rollout, no retries.
The original online MPC worker asynchronous submit/poll loop is reused by a
small research-side wrapper adding only saved-result restoration. Native
nearest/+1, Q/R, horizon5, .1s control convention, speed/acceleration limits,
coordinate conversion and solver failure handling remain official.
Official MPC SHA256: `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
Pinned LightNav checkout: `c6f40e3220edbf7011e4f17eaf2c865416737d4d`.

The tracker restores accepted command/memory, generation, selected reference and
prediction. The official solver reinitializes its numerical guesses from each
current pose/reference; no saved optimizer iterate is a carried warm start.
The new process can have cold-start computation cost. This is recorded rather
than hidden by an extra warmup solve. Future result arrivals use the actual new
worker/host timing; they are not claimed to reproduce unobserved original host
latencies. Saved-prefix phase/trajectory is the mandatory historical parity gate.

The existing NoCatchupPacer and source-resolved float32 `1/60` step are used.
180 intervals are nominal3s, exactly `3.0000001564621925` s on that source clock.
Control submits retain absolute ticks divisible by6. Results apply when seen;
commands hold while pending. Original .5s hold timeout applies. Controller error
or no result for2s ends characterization. Stop earlier on the original abort-only
guard; every new command checks its next nominal .1s and held commands check the
next integration step. The guard never steers, clips, modifies FRESH or fills a
terminated path. Cart/Hospital, .20m footprint and .05m margin stay fixed.

One raw reference stays installed through the cap, even after attachment or at
its endpoint. No VLA/RGB/Isaac/model update, GP, rigid, graph, splice or
reconciliation call is allowed. CPU single-thread BLAS is inherited from the
original Worker launcher. Optimizer/controller timing is separate from simulated
time and no online improvement is claimed.

## Frozen evaluation and presentation

Existing forward-only continuous polyline projection and shortest-angle yaw
interpolation evaluate original FRESH, without controlling reference selection.
A .10m /15deg tube must hold over a complete following .30s of saved samples.
Transient/censored/no-entry are retained, missing times remain null.
No continuous-time attachment proof or completed obstacle bypass is claimed.

Report full cap and first18/54 intervals (.30/.90s): mean/max position distance,
position and excess AUC, yaw and excess AUC; initial and first.5s separation growth;
first tube entry/sustained join; original-row/arc/remaining progress; final-pose
entry, yaw/distance and command decay. Native selected rows/actual solve pose,
memory, reference and result are saved for every new solve.

Safety reports raw-FRESH and execution clearance, curve-adjusted swept result,
finite dense minimum/time/pose/cart-relative state and safety abort. Commands
report switch delta separately from later TV. Nominal10Hz differences and actual
application-interval differences are separate diagnostics, not continuous physical
acceleration. Non-negligible turn1e-6rad/s and near-zero command1e-3 thresholds
are descriptive only.

Six frozen figures: world OLD/B/FRESH/continuation/cart/attachment/clearance;
position/yaw/lateral; v/omega; progress; clearance; native row selections.
Every PNG has plotted numeric data and source hashes, plus CSV/static index/ZIP.
A saved-only validator rechecks source/phase/integration/guard/selection/memory,
metrics and figure/CSV arithmetic without model/controller solves.

## Commands and execution status

```bash
.venv/bin/python scripts/run_osa03_native_continuation.py --mode prepare --run data/osa03_native_continuation_01/primary_20260923T103000Z
# focused tests, diff review, then freeze; commit and normal push before execute
.venv/bin/python scripts/run_osa03_native_continuation.py --mode freeze --run data/osa03_native_continuation_01/primary_20260923T103000Z
.venv/bin/python scripts/run_osa03_native_continuation.py --mode execute --run data/osa03_native_continuation_01/primary_20260923T103000Z
.venv/bin/python scripts/run_osa03_native_continuation.py --mode validate --run data/osa03_native_continuation_01/primary_20260923T103000Z
.venv/bin/python scripts/report_osa03_native_continuation.py --run data/osa03_native_continuation_01/primary_20260923T103000Z
.venv/bin/python scripts/report_osa03_native_continuation.py --validate-only --run data/osa03_native_continuation_01/primary_20260923T103000Z
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Pre-primary status: source phase resolved; saved prefix parity PASS (zero pose
error); scientific continuation not yet executed. The result section will be
appended after the single pushed-freeze execution.

## Repository-confirmed facts — completed single continuation

Execution freeze was committed and pushed as
`5d76492653057a8b2e707ac82c54d5a73e53d39b` before any new solve. One continuation
ran. All258 preservation hashes remain unchanged. OSA03's saved-only authoritative
source/report revalidation passes. No source search or REPEAT_01 rollout occurred.

**Classification: `NATIVE_CONTINUATION_SAFE_SUSTAINED_ATTACHMENT`.**
The original6-step prefix has literal command identity/value parity, exact step
count and maximum pose error0. Native then reached the nominal3s cap
(actual source clock3.0000001564621925s,180 intervals/181 states), with no guard
abort, controller failure, busy submit, hold timeout or additional FRESH update.

### Measured separation and attachment

| Window | Mean position [m] | Max position [m] | Position AUC [m·s] | Position excess AUC [m·s] | Yaw AUC [rad·s] | Yaw excess AUC [rad·s] |
|---|---:|---:|---:|---:|---:|---:|
| First.30s (18 source intervals) | .159129 | .200567 | .047739 | .017739 | .121097 | .043104 |
| First.90s (54 source intervals) | .186913 | .214078 | .168222 | .078222 | .191402 | .043104 |
| Full nominal3s | .130269 | .214078 | .390808 | .096913 | .330157 | .043104 |

Initial B-to-original-FRESH distance is.106648569m; wrapped yaw error30.000556deg.
Maximum separation is.214078141m at.466666691s, also the first.5s maximum;
initial separation growth is+.107429572m. Maximum yaw error is the initial
.523608488rad. The first.30/.90s windows have no tube entry, not a fabricated
failure timestamp.

First tube entry and first sustained attachment both start at1.466666743s;
the following.30s are fully observed. The trajectory stays inside the tube
through the cap (observed span1.533333413s), with post-attachment mean distance
.096040282m and maximum.099373881m. At attachment:
`[19.77464387710378,23.172213858026282,-.918547735820427]`;
original fractional-row progress8.722074565, remaining original arc.041848269m.
Thus the observed join happens **near the end of this short FRESH**, not early
in its middle. Sampled attachment does not establish continuous physical tracking.

### Safety and endpoint behavior

| Quantity | Measured value |
|---|---:|
| Raw original FRESH minimum edge clearance | .229486890m |
| Initial B edge clearance | 1.095851731m |
| Execution finite-sample minimum edge clearance | .133849099m |
| Execution swept lower bound including curve allowance | .133806153m |
| Frozen required edge clearance | .050000000m |
| First observed minimum time | 2.783333478s |
| Cart-relative longitudinal / left-lateral at minimum | −.473974558m / +.609758607m |

Minimum-clearance pose is
`[19.815860016409328,23.11564533847062,-1.0449625559391]`.
Native approaches the cart more closely than the raw FRESH, by about.095638m,
but remains clearance-valid and within known workspace. All180 upcoming-command
guard checks pass. The dense minimum is an observed extremum, not an exact
continuous-time maximum/minimum claim; the separate swept check includes the
explicit4.294551e-5m curved-path allowance.

Forward-only projection reaches final row9 at1.566666748s and remains there.
Unrestricted closest-point progress also never moves backward, so the primary
monotonicity rule is not hiding a retreat in this result. Final original-FRESH
arc progress1.353245520m, remaining arc0; the controller subsequently stays near
that finite endpoint. Endpoint tolerance first entry1.516666746s, minimum endpoint
distance.094022136m at1.583333416s, terminal distance.096102204m and yaw error
.071441478deg. Final pose:
`[19.815860016409328,23.11564533847062,-1.0461364022515864]`.

Commands decay to essentially zero forward speed after about1.68s (final.30s
maximum3.21e-11m/s; literal zero first occurs2.783333478s). Final command is
`[0,-.003468612344]`; angular speed is small but exceeds the frozen1e-3 diagnostic
near-zero threshold, so “both commands zero” is **false**. Maximum execution
projection beyond the endpoint tangent plane is.027754178m. This is local
endpoint settling, not completion of the cart bypass or hallway goal. In the
world figure, thin directional arrows show yaw; their tips are not extra
executed states.

### Commands, selection and compute

Switch delta is `[0,+.5000000074082285]` in[m/s,rad/s], separate from later
variation. Linear/angular TV including switch:.8m/s and3.622483660rad/s;
post-B-only TV:.8m/s and3.122483653rad/s. Maximum v=.8m/s; maximum |omega|=
1.546038252rad/s at.283333348s. Turning is already active atB; it is not delayed
until the first new solve in this experiment.

The original nominal10Hz command-grid motion diagnostic **passes**:
maximum |dv|/.1=2.000000061m/s² and |domega|/.1=5.000000074rad/s², within the
unchanged numerical tolerance. Physical execution is not a continuous-acceleration
simulation. Using actual command-application intervals instead gives maxima
1.999999956m/s² and5.999999753rad/s². The latter is the **saved historical**
tick97 result, applied.083333338s afterB. Its nominal angular value is5.000000055.
The genuine switch itself has an actual-spacing diagnostic4.285714126rad/s²
(last OLD command application toB=.116666673s).

Actual intervals span.066666670–.133333340s around nominal.1s. The6rad/s² value
exceeds5 if that bound is interpreted on actual application spacing; it is not
hidden by reporting only the passing nominal test. It is not a continuous physical
acceleration measurement and does not change the frozen geometric classification.

The saved first/second solves use nearest rows1/2 and selected rows[2,3,4,5,6] /
[3,4,5,6,7]. New Native nearest indices progress2→3→4→5→6→7→8→9. At input time
1.266666733s, nearest8 yields[9,9,9,9,9]; from1.466666743s nearest9 also repeats
that final row. Every actual selected reference and its pose/memory/result are
saved; there is no selector comparison or attribution of the error to the selector.

29 new official MPC submissions/results,0 failures/busy/timeout; two historical
results replayed,0 historical re-solves. Total new official solve wall time
.149018908s (median3.488977ms, mean5.138583ms; first/cold48.523071ms;
range3.154068–48.523071ms). Worker-start/continuation/shutdown wall time3.008848s.
Maximum paced integration wall step.016756082s. Future host receipt timing belongs
to this offline run, not to an unrecorded continuation of the original episode.
LightNav inference/RGB/Isaac calls0; GP/rigid/graph/splice/reconciliation calls0;
real model/MPC calls from tests and validators0.

### Validation and review artifacts

The first saved validator attempt found a **reporting/schema issue**:
`guard.command` was captured before `command_id`, `relative_time_s`, `origin`
were attached for journaling. The checker compared its post-journal re-evaluation
against that pre-journal snapshot. `validation_attempt_01.json` preserves failure.
No command, trajectory, guard decision, threshold, evaluator or execution code
was changed and no second rollout ran.

The authoritative `scripts/validate_osa03_native_continuation.py` compares all
physical/timing/identity and guard fields, permitting only those three journal
fields to be absent. The report imports this validator. `reporting_revisions.json`
records the presentation-only hash change; original frozen runner/worker/metric
core hashes still match. This supersedes the initial runner `--mode validate`
command, which remains preserved at its execution revision. The exact final
saved-only validation commands are:

```bash
.venv/bin/python scripts/validate_osa03_native_continuation.py --run data/osa03_native_continuation_01/primary_20260923T103000Z
.venv/bin/python scripts/report_osa03_native_continuation.py --run data/osa03_native_continuation_01/primary_20260923T103000Z
.venv/bin/python scripts/report_osa03_native_continuation.py --validate-only --run data/osa03_native_continuation_01/primary_20260923T103000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_se2.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_online_switch.py tests/test_online_history.py tests/test_online_ipc.py tests/test_online_mpc_adapter.py tests/test_online_handoff_analysis.py tests/test_obstacle_source_pacing.py tests/test_obstacle_source_acquisition.py tests/test_obstacle_source_online.py tests/test_obstacle_source02_report.py tests/test_obstacle_source03.py tests/test_join_online02.py tests/test_join_online03.py tests/test_handoff_execution_loss.py tests/test_gp_se2_join01.py tests/test_gp_se2_environment.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Saved source/phase/command-memory/integration/selection/guard/metric validation
PASS; six PNG numeric/hash sidecars and window CSV parity PASS. All six images
visually inspected; world axes equal metres, figures keep legends clear of curves.
Final relevant regression suite371 PASS; compileall and diff check PASS.

Local run contains source_manifest/controller_phase/protocol/freeze,
stage0_prefix_reconstruction/prefix_parity/restoration,
rollout/metrics/validation/report_validation,6 PNG+JSON sidecars,
review/windows.csv and review/trace.csv. Primary
[world figure](../data/osa03_native_continuation_01/primary_20260923T103000Z/review/world.png),
[static index](../data/osa03_native_continuation_01/primary_20260923T103000Z/index.html),
[review ZIP](../data/osa03_native_continuation_01/primary_20260923T103000Z/review_bundle.zip).
Large/generated files remain local and ignored; Git contains code/config/tests/docs.

## Research interpretation

Native is geometrically safe and eventually attaches in this one fixed source.
It nevertheless exhibits initial separation growth, about1.47s before attachment,
and reduced execution clearance. These are measured transition costs, not proof
that reconciliation can improve them. Attachment near an exhausted endpoint also
limits how much post-join intent following this reference can demonstrate.

| Observed Native behavior | Bounded future formulation question |
|---|---|
| Distance.107→.214m; attachment1.467s near endpoint | Can an explicit B→FRESH attachment objective reduce early separation without sacrificing future progress? |
| Historical switch adds.5rad/s; saved next result has6rad/s² actual-spacing diagnostic | How should physical incoming motion, controller memory and asynchronous application phase enter feasibility? |
| Reference clearance.229m becomes execution.134m, still valid | Should executable-transition clearance supplement reference-only clearance? |
| Native ultimately remains safe and attached | Any future method must preserve this baseline safety/intent behavior rather than merely reduce one metric. |
| Native eventually repeats final row in all five slots | How much endpoint settling reflects the finite reference/interface? No selector effect was isolated here. |

Largest remaining uncertainty: whether a future **same-B, same-phase** method
can reduce early attachment cost while preserving safety and forward intent;
this experiment did not compare or implement such a method.

## Not demonstrated

Graph/SE(2) necessity or benefit; superiority over rigid/splice; recoverable cost;
complete obstacle bypass; online closed-loop improvement; real-robot feasibility;
population prevalence. A safe raw FRESH and safe local Native continuation are
not a general obstacle-avoidance result. No next-stage factor was implemented.
