# GP-SE2-REF-04: saved prediction–execution stress audit

This audit asks where required clearance and the original route first fail for
`episode_017_repeat_00/handoff_007`. It compares the saved REF-03
`B_DENSE_ROW_STEP` and `C_DENSE_SOURCE_PROGRESS` records; `A_NATIVE` is historical
context. It does not change any outcome, trajectory, reference selector, MPC
calculation, physical acceptance or gate.

Source: `data/robotless_gp_se2_ref_03/primary_20260919T141000Z/`.
Starting revision: `fc51724ececc29ad00e1d62b832ce097821d16b7`.
The new runner resolves the case through its manifest and checks the authoritative
validation, transitive original files, source code, external MPC, configuration,
environment and copied inputs before auditing. Unrelated user configuration
changes are separately hashed and preserved.

## Frozen audit protocol

The audit uses three distinct layers:

1. The actual five selected world targets, controller-local targets, source-row
   identities and sequential yaw. Point checks, target-to-target connectors and
   current-pose-to-first-target connectors remain separate. These connectors are
   spatial diagnostics; their row parameter is not physical time.
2. The six saved MPC prediction poses: current state followed by five forward
   Euler states at 0.1 s spacing. The first interval is [issue, issue + 0.1 s];
   the remaining 0.4 s is an unexecuted prediction tail. Node clearance and direct
   swept clearance of the saved discrete prediction polyline are both reported.
3. The saved applied commands and execution states. The unchanged exact held
   unicycle integrator reconstructs each stored interval. It does not compute
   a new command or closed-loop rollout.

The original `evaluate_rollout` supplies full acceptance. Its dense sample grid,
60 Hz boundaries, geometry uncertainty and global curve allowance are retained.
Additional first-violation localization subdivides the earliest failing swept
segment, checking both halves rather than assuming safe endpoints imply a safe
interior. The target bracket width is at most 1 ms; unresolved brackets remain
explicit. Refinement never replaces the original acceptance result.

The original finite directed gate is a safe footprint-center interval. No second
footprint or clearance subtraction is applied. A prediction with no crossing in
its 0.5 s horizon does not thereby fail the full route. Already passed gates and
inherited invalid start states are identified separately.

First-interval comparisons retain four endpoints: saved prediction, Euler using
the applied command, exact held-command reconstruction and the next recorded
control state. Full solved control sequences, pre-clipping first controls and
original local predictions were not saved. Therefore solver numerical residuals
and command clipping cannot be uniquely separated. Beyond the first interval,
prediction/actual differences include replanning; beyond 3 s actual counterparts
are unavailable.

`PROTOCOL` in `gp_se2_ref04_audit.py` freezes numerical comparison tolerances,
missing-data handling and representative plots before the output run. Figures
use t=0, C's earliest prospective warning, first actual violation, minimum
clearance and invalid gate-crossing intervals, with duplicates removed and all
reasons retained. The entire 0–3 s execution and all 60 prediction records remain
in the audit and heatmaps.

No new VLA inference, GP/rigid optimization, MPC solve or closed-loop rollout is
part of this implementation. No GUI is required. Numerical findings and exact
output commands are recorded below after the saved-record audit.

## Completed saved-record result

Operational status: **GP_SE2_REF_04_COMPLETED_WITH_LIMITATIONS**.
Diagnosis: **FAILURE_LAYER_LOCALIZED**. Completion means that the failure layers
were localized; C remains a failed execution under the original predicates.

Audit source commit: `0638cffb9b4cdb5d58521693bde969fd8435aae1`.
Primary output:
`data/robotless_gp_se2_ref_04/audit_20260919T162300Z/`.
The final report commit is recorded in that run's `git_completion.json` after
commit; it does not rewrite the frozen audit source or numerical results.

### Source, frame and data availability

The authoritative REF-03 validation is valid, with SHA-256
`d5ae1392b92d546a17ce10e2796f4cb34892941641b14722f296340d44a43595`.
All 44,929 transitive source, environment, numerical-code and historical-result
files were verified and preserved. B/C reference `.npy` and lineage files are
byte-identical; installed world XY roundtrip error is 3.56e-15 m, periodic yaw
error 1.12e-16 rad. Selected source-row progress is checked against each actual
selected index and the saved lineage. No nearest rule or stride is recomputed.

Original B is `[-25.206685088232042, 8.785083286170527, 1.016382271364498]`
in fixed Isaac world metres/radians. The original capture pose is
`[-25.41706629538567, 8.369176499573253, 1.1897104663489868]`.
The preserved installation is `inverse(T_world_capture) * T_world_candidate`,
with local +x forward, +y left, yaw counterclockwise; it is not B re-anchoring.
Both original physical command and controller memory happen to equal
`[0.8, -0.29769455142094675]` here; they are nevertheless retained as separate
fields. Original observation/readiness/execution timestamps and intrinsic
LightNav waypoint_dt=null remain in `input_context.json`/`source.json`.

Each method supplies 30 successful saved control records, 30 actual 5×3 target
arrays, 30 saved 6×3 world predictions, 181 states and 180 integration commands.
Prediction node zero equals the actual solve input. Each applied command matches
its six held integration ticks; solve inputs, schedule and controller-memory
updates are independently checked. These are **60 prediction records for one
handoff**, not 60 independent events. Each method has ten future prediction
nodes beyond t=3 s, explicitly N/A for actual comparison. Full future controls,
pre-clipping first controls and original controller-local prediction states were
not saved. Actual controller-local reference targets were saved and verified.

No new VLA inference, GP/rigid solve, MPC solve, closed-loop rollout or GUI runtime
was performed. Historical MPC timing fields in the reproduced outcomes describe
REF-03 computation, not new calls by this audit.

### Original full outcome recheck

Every original evaluator field, including dense poses/times, environment, route,
execution metrics and failure reasons, reproduces literally. The full execution
checker retains 0.20 m footprint radius, 0.05 m required edge clearance, 1e-7 m
geometry uncertainty, the original curve allowance, goal, gate and tolerances.

| Quantity | B dense row step | C source progress |
|---|---:|---:|
| Original full success | Pass | Fail: clearance and route |
| Minimum swept footprint-edge clearance, m | 0.063068847164 | 0.036086665935 |
| Effective clearance threshold, m | 0.050000605763 | 0.050001401392 |
| Physical overlap | No | No |
| Known workspace | Pass | Pass |
| Directed finite gate | Pass | Fail |
| Final position error, m | 0.020281681 | 0.061338515 |
| Final absolute yaw error, deg | 2.506982 | 0.044489 |
| Required final 0.20 s goal dwell | Pass | Pass |
| Original goal time, s | 2.525 | 1.085 |
| Motion limits / controller failures | Pass / 0 | Pass / 0 |

The original goal is `[-24.528587338892518, 9.528111797464593,
0.4351012987731506]`. Positive 0.0361 m clearance means **required-clearance
violation, not physical overlap**. C's faster goal entry and successful goal
position/yaw/dwell do not override earlier clearance and route failure. The
historical full-success predicate already rejects C; goal-only reporting would
hide that failure. A_NATIVE remains context only and was not rerun or promoted
into the B/C audit sample.

### L1: safe reference geometry does not imply safe prediction

The shared dense path has minimum direct swept clearance 0.111806571 m.
All 150 selected target points and all 30 target-to-target polylines pass in each
method. At t=0, both methods use the same actual pose, dense path and nearest
row. B selects derived rows `[1,2,3,4,5]`, source progress
`[3.206897,3.413793,3.620690,3.827586,4.034483]`. C selects
`[5,10,15,20,25]`, source progress
`[4.034483,5.068966,6.103448,7.137931,8.172414]`.

| t=0 geometric diagnostic, m | B | C |
|---|---:|---:|
| Selected-target polyline minimum clearance | 0.111806571 | 0.112737738 |
| Current-to-first-target connector minimum clearance | 0.124806074 | 0.101949543 |
| Saved prediction polyline minimum clearance | 0.061077063 | 0.036392441 |

C's selected-target polyline crosses the finite gate in its valid center interval
(63.109 mm inside the endpoint); B's short t=0 target polyline has not reached it.
Thus this event does **not** support the claim that an initially unsafe straight
connector between selected targets causes the failure. C has two unsafe entry
connectors at t=0.3 and 0.4, after its current state already violates clearance;
these are consequences/continuations, not an advance explanation of first failure.

Target selection and actual command differ at t=0. B applies approximately
`[0.8, +0.202305282]`, C `[0.8, -0.520556759]` m/s, rad/s. C's farther progress
sequence leads the unchanged MPC calculation to an inside-corner prediction and
execution despite the safe reference polyline. Subsequent B/C prediction inputs
are different closed-loop states; there is no invented matched-state prediction
comparison. The existing MPC source contains pose/control costs and motion
bounds, with no obstacle/gate constraint. That source fact explains why no such
constraint prevents the observed predicted shortcut, while the saved records
establish where this particular event fails.

### L2/L3: issue time, forecast time and actual failure

| Event | Issue time, s | Forecast/actual time or bracket, s | Interpretation |
|---|---:|---:|---|
| First target and command difference | 0 | 0 | Same initial input state |
| First prospective C prediction clearance warning | 0 | [0.271875, 0.27265625] | Prediction tail, not the applied first interval |
| First predicted invalid gate crossing | 0 | 0.397564306 | Tail, 15.120 mm outside finite interval |
| First prospective C predicted-first-interval violation | 0.2 | [0.26328125, 0.2640625] | The next applied 0.1 s interval |
| First original sampled actual clearance violation | N/A | 0.265 | Original dense sample |
| Earliest original failing swept segment | N/A | [0.260, 0.265] | Original dense segment |
| Refined actual first-violation bracket | N/A | [0.261875, 0.262500] | Exact saved-command queries; 0.625 ms width |
| C minimum-clearance location on checked polyline | N/A | 0.371938557 | In original segment [0.370, 0.375] |
| C invalid actual gate crossing | N/A | 0.397130489 | Refined plane bracket [0.396875, 0.397500] |

C's minimum-clearance center is `[-25.031071528680155, 9.025003068603297]` m.
The refined first-violation bracket runs from center
`[-25.086243930, 8.956390078]` to `[-25.085938206, 8.956785721]` m,
evaluated from the same saved held command.
B's minimum is at approximately t=0.503529566 s,
`[-25.077536891217488, 9.005821193017912]` m. These times are the linear parameters
of closest points on the originally checked polylines, not certified exact
continuous-arc extrema. The original success predicates are not recomputed with
a tighter threshold or replaced by the refined location diagnostics.

B has no unsafe saved prediction. C has five unsafe predictions issued at
0.0–0.4 s: three prospective warnings at 0.0, 0.1 and 0.2 from valid starts, and
two `INHERITED_INVALID_START` records at 0.3/0.4. The first warning was available
in the saved prediction about 0.262 s before actual first violation, but no online
monitor acted on it. Its forecast crossing time differs from actual onset by
about 10 ms; subsequent replanning is included in that difference. At t=0 the
first applied interval is safe; by t=0.2 the predicted first interval itself fails.
No predicted-unsafe tail segment with an available actual counterpart becomes
safe in this record; the audit retains that comparison rather than assuming all
tails execute. Twenty future nodes across B/C extend beyond saved execution and
remain unavailable.

### Gate geometry

The gate normal is `[0.7071067812,0.7071067812]`; half-width is 4.3465 m and the
original crossing tolerance is 1e-6 m. It is a broad corner-to-stairs passage
proxy whose interval already represents safe robot-center positions, not a
literal doorway or required crossing deadline. The checker uses finite geometry,
positive crossing direction and order, not `gate.time_s` as an arrival deadline.

| Actual positive crossing | B | C |
|---|---:|---:|
| Crossing time, s | 0.719965940 | 0.397130489 |
| Center world x, m | -25.041853339 | -25.018005779 |
| Center world y, m | 9.064194917 | 9.040347358 |
| Signed distance inside finite endpoint, m | +0.018567129 | -0.015158413 |
| Footprint-edge clearance at crossing, m | 0.070567129 | 0.036841587 |
| Classification | VALID_CROSSING | INVALID_INTERVAL_CROSSING |

Neither crossing is reverse or grazing. C passes the infinite gate line in the
right direction but outside the finite safe center interval, so reaching the
original final goal is not route success. Later predictions entirely beyond the
gate retain actual prior crossing history; they are not required to recross it.

### Model discrepancy and causal limits

Across all 30 first intervals, maximum prediction-versus-applied-Euler XY
residual is 0 for B and 9.38e-10 m for C under this audit's explicit time arithmetic.
Maximum saved-prediction-versus-exact endpoint difference is 0.8092 mm for B and
2.0821 mm for C. Original 60 Hz reconstruction matches saved states within
3.56e-15 m and 4.45e-16 rad; the independently reconstructed full 0.1 s endpoint
also matches the next saved control state.

No first-interval clearance classification flips between saved prediction,
applied-command Euler polyline and exact reconstructed execution in either
method. For C at issue 0.2 s, first-prediction minimum clearance is 0.043288364 m
and actual prefix minimum is 0.042122802 m: both already fail by several mm.
C's full minimum is 13.9147 mm below its effective requirement. The original
curve allowance is 0.000001301 m for C (B 0.000000506 m); prediction polylines have
zero curve allowance. These distinct thresholds remain explicit. The observation
supports the conclusion that Euler/exact mismatch does not explain away this
failure, not a general bound on future model error or a unique decomposition of
solver error and clipping.

**Measured facts:** safe stored reference/targets, unsafe predicted motion before
actual failure, first-interval warning before application, actual clearance and
finite-gate failures, successful final goal/dwell, and no physical overlap.
**Strongly supported mechanism:** source-progress target selection changes the
MPC's chosen motion, allowing an inside-corner route that the reference polyline
itself does not take. **Not identified:** the full optimized future control
sequence, pre-clipping controls, a unique internal objective-term cause, physical
robot dynamics, online monitoring, or whether adding any particular GP factor
would prevent the tracker failure.

The relevant conditions remain separate: L1 plan/reference validity, L2 tracker
predicted first-interval validity, and L3 full applied execution/route validity.
This audit adds no safety filter, obstacle constraint, fallback or selector tune.
A GP obstacle factor cannot by itself be claimed to guarantee the existing
tracker's execution, and this result does not resolve GP between-point motion
feasibility.

## Verification, figures and reproducibility

Primary artifact validation recomputes the audit from the saved source records,
compares all original outcomes and per-solve records, recomputes CSV/plot numbers,
checks immutable hashes and verifies ZIP bytes. It passed with no errors; the
authoritative report is `audit_20260919T162300Z/validation.json`. New solver/rollout
counts are zero in both generation and validation. Full repository tests passed:
**2,160 passed, 19 skipped** (114.60 s, including the additive presentation tests). Skips are documented unavailable historical
artifacts or an intentionally non-exact fixture, not excluded stress outcomes.
The first sandboxed suite encountered two existing Unix-socket permission errors;
the same full suite passed with local-socket access. Compileall and whitespace
checks pass. No shell launcher was changed.

Recorded costs: source preparation/hash preservation 6.921 s; environment load
0.042 s; numerical saved-record audit 3.302 s. Independent final validation took
10.728 s including its own source hashing and recomputation (environment 0.045 s,
audit 3.304 s). These nested timings are not added as independent percentages;
plotting and Git activity are not represented as MPC computation or simulation.
The existing research `.venv` supplied Python 3.12.3, NumPy 2.5.2, SciPy 1.18.1,
Shapely 2.1.2, Matplotlib 3.11.1 and pytest 9.1.1; the supplemental
`verification/software.json` records the audit runtime. No package, system Python,
official MPC environment or external source was installed or changed.

All seven primary PNGs and exact numeric/hash sidecars are in `plots/`, with a
static `index.html`. The primary review ZIP is 3,195,802 bytes, SHA-256
`a07754a924bbb25c668062ca8c31da6d8cecad318a0bdddbc7a2d8d87a5e12fa`.
It contains the audit, tables, source/protocol explanation and figures, with no
raw RGB, full environment, checkpoint or external code. Actual visual review
identified two legends obscuring some geometry and an unclear heatmap tick range;
a separate additive presentation below preserves every primary byte and numeric
result while improving only those displays. This remains a static diagnostic,
not Isaac GUI evidence.

Commands, from the repository root:

```bash
.venv/bin/python scripts/run_gp_se2_ref04.py prepare --run data/robotless_gp_se2_ref_04/audit_20260919T162300Z
.venv/bin/python scripts/run_gp_se2_ref04.py audit --run data/robotless_gp_se2_ref_04/audit_20260919T162300Z
MPLCONFIGDIR=/tmp/gp_se2_ref04_mpl .venv/bin/python scripts/plot_gp_se2_ref04.py --run data/robotless_gp_se2_ref_04/audit_20260919T162300Z
MPLCONFIGDIR=/tmp/gp_se2_ref04_mpl .venv/bin/python scripts/run_gp_se2_ref04.py validate --run data/robotless_gp_se2_ref_04/audit_20260919T162300Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

Creation commands refuse existing output. To regenerate, choose a new REF-04 run
ID; to revalidate the existing run use `--output` with a new report path. Do not
rerun the historical MPC worker, corpus or environment export.

## Next single formulation experiment

Return to the existing GP-SE2-02 hard-case records with a **saved GP interpolation
feasibility audit**: take the already retained hard-position-and-direction
candidates and localize their support/collocation versus between-point lateral
velocity and acceleration violations under the unchanged original full checker.
Keep plan availability, full plan validity and the prediction/execution conditions
identified here separate. This directly addresses the unresolved GP formulation
limitation without making a redesigned or “perfect” controller a prerequisite,
without claiming that static plan clearance guarantees execution, and without
performing that follow-up inside REF-04.

## Additive presentation and final review

Preferred [static figure index](../data/robotless_gp_se2_ref_04/presentation_20260919T163000Z/index.html)
and [review ZIP](../data/robotless_gp_se2_ref_04/presentation_20260919T163000Z/review_bundle.zip):
`data/robotless_gp_se2_ref_04/presentation_20260919T163000Z/`.
The three changed figures only move legends outside data and expose negative/zero
heatmap ticks. All seven numeric sidecars match the primary exactly; the other
four PNGs and sidecars are byte-identical. All actual corrected images were
visually reviewed. The original audit, protocol, validation, frozen plotter and
primary PNGs remain unchanged. Presentation validation passes, including ZIP
member bytes. No numerical audit, optimizer or rollout was repeated to improve a
result. `FINDINGS.md` gives a compact reading guide inside the bundle.

Presentation ZIP: 3,190,627 bytes; SHA-256
`aecc5dd26823142a4b087856528ddf29ba063085a03063d8251320ecf96632fe`.
Rendering, packaging and presentation validation entrypoint took 3.819 s.
GUI runtime: **NOT_RUN_STATIC_DIAGNOSTIC**.

```bash
MPLCONFIGDIR=/tmp/gp_se2_ref04_mpl .venv/bin/python scripts/present_gp_se2_ref04.py \
  --primary data/robotless_gp_se2_ref_04/audit_20260919T162300Z \
  --output data/robotless_gp_se2_ref_04/presentation_20260919T163000Z
```

This command also refuses overwrite. Its additional unit tests exercise numeric
identity, external legends, negative margin ticks, authoritative source checking,
unchanged images, ZIP content and overwrite refusal. It does not change the
scientific result or the next single formulation experiment above.
