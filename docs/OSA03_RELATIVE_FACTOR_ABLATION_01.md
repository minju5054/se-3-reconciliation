# OSA03_RELATIVE_FACTOR_ABLATION_01

## Frozen protocol

**Timing-controlled offline causal reference comparison.** One sealed OSA03
REPEAT_00 source; one new NO_RELATIVE planning solve and one rollout per eligible
reference. This is not an online latency benchmark or real asynchronous deployment
timing. No source selection, new LightNav, RGB or Isaac calls, tuning or retries.

Fetched `origin/main` and starting HEAD:
`53af3488809063a8fddf16a9bb9f7f35bd19aa94`. Read the current repository instructions,
README, work log, formulation/common-B/Native/trackability reports and implementations.
The two unrelated Stage 0 config edits are preserved and excluded from commits.
The historical common-B saved-only validator passed before implementation edits.

Run: `data/osa03_relative_factor_ablation_01/primary_20260929T050000Z/`.
The directory identifier is fixed; actual UTC/host clocks are recorded separately.
Raw inputs stay in their sealed directories. All new arrays/rollouts are derived,
ignored data. Small source/config/code hash ledgers and summaries are tracked under
`results/osa03_relative_factor_ablation_01/`.

### Formulation and frames

A and B are fixed. The state-shift factor moves editable early FRESH nodes X_j
toward the transported target Ftilde_j = B A^{-1} F_j, equivalently encouraging
B^{-1} X_j ≈ A^{-1} F_j.

The historical L factor is described here as the **observation-to-application
state-shift / transport factor**, denoted S in mathematical discussion. Historical
code keys L are retained. `r_S,j = Log(Ftilde_j^{-1} X_j)`; relative and anchor
residuals remain exactly the existing implementation:

```
r_R,j = Log((F_j^-1 F_(j+1))^-1 (X_j^-1 X_(j+1)))
r_A,j = Log(F_j^-1 X_j)
s_j = cumulative original-FRESH XY arc / total original-FRESH arc
w_S = (1-s)^2; w_A = s^2
normalization = diag(0.10 m, 0.10 m, 10 degrees)
```

S/A use normalized weighted block means; R uses the edge mean. Initialization is
exact original FRESH. No waypoint timestamps/dt, twist, GP, correspondence,
controller/command factor, obstacle cost or replacement smoothness term.
World XY is metres, yaw radians CCW, Z up; body x forward/y left.
`F_j = A F_local,j` remains observation-anchored. A-local derived representations
use `A^-1 X`; raw FRESH is never anchored at B.

| Fixed order | Reference | Planning calls |
|---|---|---:|
| M0_NATIVE | Original FRESH F | 0 |
| M1_TAPER | Existing `Exp((1-s) Log(B A^-1)) F` | 0 |
| FULL_LOCAL_SE2 | Exact authenticated frozen optimized world array | 0 |
| NO_RELATIVE | Optimize E_S + E_A; remove R from residual vector/cost | 1 |

The default LocalSE2 API still optimizes E_S+E_R+E_A. Only the explicit
`include_relative=False` option omits R. Raw R residuals remain available for
**diagnostic relative-edge distortion, not optimized cost**. No weight or solver
setting changes. Frozen Full is not reconstructed or re-optimized.

Unchanged LM: 80 iterations, central finite difference 1e-6, right-local SE(2)
updates, initial damping .001, rejection ×10, acceptance ×.3, maximum damping
1e12, gradient/step tolerances 1e-9, cost tolerance 1e-12. Same complete-polyline
candidate acceptance callback; only improving feasible steps may be accepted.
An optimizer exception records the trace and stops scientific execution; no fallback.

### Authenticated inputs and logical schedule

Source: `data/obstacle_source_acquisition_03/primary_20260923T085200Z/`, REPEAT_00.
Raw FRESH SHA256 `8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521`.
Frozen Full world SHA256 `bd459b0343fac1e78e4ac80831f4f99af9b86dd4876a3b0df1b45f41c21ce120`.
Official MPC SHA256 `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
The existing clean pinned external checkout and environment are used read-only.

The exact schedule is loaded from the previous common-B M0 rollout, authenticated
against its sealed result ledger and the tracked result summary. It is not generated
from nominal frequency or reconstructed from prose. The entire 180-interval schedule
is reused, including the primary 54 intervals (~.9 s). Full hash values, each pair,
and source paths are in the tracked freeze summary/config.

- B state/tick92, simulation time1.5666667483747005 s; original source dt
  `float32(1/60)=0.01666666753590107` s.
- Shared already-applied u_B_plus and advanced controller memory are restored
  exactly from saved solve000005; physical u_minus is recorded separately.
- There is no solve at B and no replay of future saved solve000006.
- Primary attempted/accepted submits: `[96,102,108,114,120,126,132,138,144]`.
- Their predetermined applications: `[99,103,109,115,121,127,133,139,145]`.
- Initial application92 belongs to the common saved first command.

Each independent tracker is initialized through the existing official observation-A
transform before its clock starts, with the saved generation/command/memory/reference/
prediction fields. Official MPC selection, H=5, objective, limits, initial guesses,
solve and poll behavior remain unchanged. No warmup solves.

At each authenticated submit tick, the research wrapper submits that tick's current
state and pauses the simulation while the unchanged worker finishes and polls.
The result waits in a release queue until its predetermined application tick;
previous commands are integrated/held in the meantime. No future state is used.
The official poll updates controller memory; every frozen release is strictly before
the next submit, so no new solve can observe an alternative memory ordering.
Generation/chunk/version rejection is preserved. Actual worker receipt and logical
release/application are separate records. Slow wall time cannot advance simulation.
A wall wait timeout (30 s) is a technical termination, never an invented command.

Before interpreting differences, the gate requires the identical attempted submits,
accepted submits, successful applications, all 54 integration intervals, and exact
initial state/command/memory provenance. Failure yields `SCHEDULE_CONTROL_FAILURE`;
no reference-causal interpretation is then permitted. Unsafe references are excluded
with explicit null metrics; the gate records whether all four were observed.

### Safety and evaluation

All complete references use the existing direct Hospital+cart geometry checker,
.20 m circular footprint, .05 m required footprint-edge clearance and unchanged
workspace/numerical uncertainty conventions. No B-to-first-row connector is added.
Independent full-union distance checks precede rollouts. Unsafe references are
recorded and skipped without repair. The existing abort-only execution guard
checks each new command's .1 s hold and each held command's next integration step.
An unsafe proposal is not applied; preserve the valid prefix and censor the rollout.

Primary evaluation is always against ORIGINAL FRESH with unchanged forward-only
continuous projection, shortest-angle yaw, and <=.10 m/15 degree tube over the
complete following .30 s sampled dwell. Missing attachment stays null. Report
initial errors, .5 s maximum/growth, .3/.9 s position/yaw AUC, attachment time/
original progress/remaining arc, clearance, endpoint, command TV/max omega and
termination. Secondary own-reference AUC/errors are separate, never substituted.
No scalar score or post-result threshold for superiority.

Planning diagnostics include node corrections, arc/segments, node-local lateral
correction and B-frame lateral extent (definitions explicit), yaw/relative edges,
closed-form rigid fit, full safety, B-to-row0 and continuous projection geometry,
first chord of at least .02 m relative to B heading, and Shapely self-intersection.
These are spatial pose deformations. The official MPC decides actual omega(t).

Nine figures with numeric/hash sidecars: references/world execution (equal axes,
cart+center exclusion boundary, A/B, attachment and minimum-clearance markers),
original-FRESH position/yaw/progress, clearance, commands, schedule, node/edge
distortion. Distinct markers/styles and numeric separation describe overlapping curves.

### Freeze and validation discipline

Preparation and zero-numerical-solve external restoration preflight passed. The
preflight tests the three already available references; the new reference uses
exactly the same generic installation/restoration path after its sole planning solve.
Synthetic tests cover the fourth method and are not experimental evidence.

Pre-execution regression: **384 passed, 1 skipped** (absent ignored historical
EXP-01B/EXP-02B corpus). This includes 19 new tests, historical Local-SE2/common-B,
SE(2), controller/generation, timing, guard, attachment and geometry tests. The
pre-edit Full synthetic residual/cost/solution golden is bit-exact. No new scientific
solve was used to test default preservation. Source authentication includes the
sealed R00 geometry and official source hashes; no R01 scientific evaluation.

Commit and normal push of implementation/config/tests/protocol/freeze ledger must
precede the one execution. The runner verifies live remote HEAD and committed file
bytes, and creates exclusive one-shot execution/optimization/method markers.
Validators reconstruct saved equations, retractions, safety, controller selection,
memory, integration, schedule and both metric targets without scientific solves.
Results, figures, interpretation and call counts are appended after execution.

## Repository-confirmed facts

Scientific freeze **`0b0f4297b79e4ed52bddda20a7090c511a8858eb`** was normally pushed
before execution. Classification: **`TIMING_CONTROLLED_OFFLINE_COMPARISON_VALID`**.
One optimizer call, zero Full reconstruction solves, four rollouts, 30 MPC solves
per method (120 total); LightNav/RGB/Isaac/new source calls all zero. No retries.
All four references passed. All four executions ended at `OBSERVATION_CAP` after
180 integrations, 3.0000001564621925 s, with no abort, stale result, busy result,
controller error or hold timeout. Saved-only validation passed.

### Schedule and common-state gate

All four passed the complete primary 54-step gate. Attempted submits equal accepted
submits `[96,102,108,114,120,126,132,138,144]`; corresponding successful applications
are `[99,103,109,115,121,127,133,139,145]`. Each starts from the identical saved B,
physical u_minus, already-applied u_B_plus, generation 3 and controller memory.
Every recorded wall wait has identical before/after simulation time and pose.

The complete application sequence is identical for all four:
`[92,99,103,109,115,121,127,133,139,145,151,157,163,169,175,181,187,193,199,205,211,217,223,229,235,241,247,253,259,265,271]`.
All 30 attempted/accepted submits are the exact saved M0 sequence 96 through 270.
Native and Taper have **zero maximum pose-component and command-component difference**
from their previously frozen common-B rollouts. Full's old one-tick-later rollout
is not substituted for the new controlled result.

```
A = [19.203242163778583, 24.356668352984542, -1.5689752526514715]
B = [19.203630553052257, 24.14333536015282, -1.5689760264727979]
u_minus = [0.8, -1.5584691584098122e-06]
u_B_plus = u_mem_B = [0.8, 0.49999844893907003]
B tick = 92; B simulation time = 1.5666667483747005 s
```

### Primary execution against ORIGINAL FRESH

AUC units are m·s for position and rad·s for yaw. Initial yaw is reported in degrees.
All fields below derive from the original-FRESH evaluator, independent of each
method's controller reference. Initial separation growth is maximum position error
in the first .5 s minus the initial error.

| Method | Initial position m | Initial yaw deg | Max position .5 s m | Initial growth m |
| --- | --- | --- | --- | --- |
| Native | 0.106648569 | 30.000556457 | 0.215262377 | 0.108613807 |
| Taper | 0.106648569 | 30.000556457 | 0.210303942 | 0.103655372 |
| Full | 0.106648569 | 30.000556457 | 0.216171444 | 0.109522874 |
| No-relative | 0.106648569 | 30.000556457 | 0.210021130 | 0.103372561 |


| Method | Position AUC .3 s | Position AUC .9 s | Yaw AUC .3 s | Yaw AUC .9 s |
| --- | --- | --- | --- | --- |
| Native | 0.047920362 | 0.169065633 | 0.122763625 | 0.193368049 |
| Taper | 0.047473914 | 0.166853435 | 0.122779344 | 0.187911281 |
| Full | 0.047920433 | 0.169963954 | 0.122774526 | 0.193334828 |
| No-relative | 0.047473903 | 0.166121029 | 0.122777407 | 0.191946562 |

Attachment uses the complete following .30 s sampled dwell. Row progress is fractional row index (final row 9); arc fraction is normalized original XY arc. All four have observed sustained attachment; none attaches in the primary .9 s. No attachment value is null or censored in this run.

| Method | Attachment s | Original row /9 | Original arc fraction | Remaining original arc m |
| --- | --- | --- | --- | --- |
| Native | 1.466666743 | 8.717846370 | 0.968605162 | 0.042484924 |
| Taper | 1.500000078 | 8.738964973 | 0.970954999 | 0.039305018 |
| Full | 1.483333411 | 8.781296840 | 0.975665206 | 0.032930950 |
| No-relative | 1.433333408 | 8.474642824 | 0.941544244 | 0.079104989 |

TV includes the common physical u_minus→u_B_plus switch. All terminations are `OBSERVATION_CAP`.

| Method | Swept clearance lower bound m | Endpoint error m | Linear TV m/s | Angular TV rad/s | Max abs omega rad/s |
| --- | --- | --- | --- | --- | --- |
| Native | 0.133561094 | 0.096439021 | 0.800000000 | 3.629555771 | 1.548333658 |
| Taper | 0.134208269 | 0.095981706 | 1.199999972 | 3.513599927 | 1.499998444 |
| Full | 0.133709423 | 0.096335462 | 0.799999999 | 3.539108927 | 1.499998446 |
| No-relative | 0.138784907 | 0.091298626 | 1.199998407 | 3.507769359 | 1.499998438 |

All final original-FRESH projected arcs are 1.3532455201158953 m (fraction 1,
row 9, remaining arc 0). None has backward unrestricted projection. Minimum execution
clearance occurs at the final sample for all four. Final speeds are near zero;
all four still fail the pre-existing near-zero-both-commands diagnostic because
|omega| is about .0035–.0037 rad/s. No method is censored by a safety abort.
Nominal 10 Hz command-grid diagnostics pass for all four. The inherited compressed
application interval 99→103 produces approximately 7.5 rad/s² angular command-jump
diagnostics; these are not continuous physical accelerations.

No-relative minus Full (signed, no absolute-value conversion):

| Metric | Difference |
| --- | --- |
| max_position_error_05_m | -0.006150314 |
| position_auc_03_m_s | -0.000446530 |
| position_auc_09_m_s | -0.003842925 |
| yaw_auc_03_rad_s | 2.88006587e-06 |
| yaw_auc_09_rad_s | -0.001388266 |
| sustained_attachment_s | -0.050000003 |
| remaining_arc_at_attachment_m | 0.046174039 |
| execution_clearance_lower_bound_m | 0.005075483 |
| endpoint_error_m | -0.005036836 |
| linear_command_TV | 0.399998408 |
| angular_command_TV | -0.031339568 |

### Secondary tracking against each method's own reference

These values use each method's own reference and remain secondary.

| Method | Own pos AUC .3 | Own pos AUC .9 | Own yaw AUC .3 | Own yaw AUC .9 | Own max pos .5 m |
| --- | --- | --- | --- | --- | --- |
| Native | 0.047920362 | 0.169065633 | 0.122763625 | 0.193368049 | 0.215262377 |
| Taper | 0.018224663 | 0.097326213 | 0.102281575 | 0.167395814 | 0.135741566 |
| Full | 0.016987214 | 0.095301244 | 0.101992576 | 0.170848314 | 0.130096010 |
| No-relative | 0.015885930 | 0.087873797 | 0.100640286 | 0.169780498 | 0.118106356 |

### Planning geometry

Node displacement is world XY distance to the corresponding original FRESH node.
Lateral correction is the translation correction's y component in that original
node's body frame; B-lateral extent uses B's fixed heading. Relative-edge columns
use the existing SE(2) Log residual. Rigid fit is the existing closed-form minimum
XY squared-error left transform; yaw is assessed under that same transform.

| Method | First shift m | Endpoint shift m | XY arc m | Segment min m | Segment max m | Self-intersection |
| --- | --- | --- | --- | --- | --- | --- |
| Taper | 0.213333346 | 0.000000000 | 1.173311648 | 0.129839730 | 0.130724154 | false |
| Full | 0.211144018 | 0.002186831 | 1.178800300 | 0.116958805 | 0.145860166 | false |
| No-relative | 0.213333346 | 0.000000000 | 1.176168914 | 0.112274799 | 0.147912381 | false |


| Method | Max lateral correction m | Max B-lateral extent m | Max yaw correction deg | Min reference clearance m |
| --- | --- | --- | --- | --- |
| Taper | 0.094785722 | 0.676245107 | 0.000044337 | 0.229486890 |
| Full | 0.102800304 | 0.676301930 | 0.341394174 | 0.227809208 |
| No-relative | 0.105021842 | 0.676245107 | 0.000044337 | 0.229486890 |


| Method | Edge translation RMS m | Edge translation max m | Edge yaw RMS deg | Edge yaw max deg | Rigid XY RMS m | Rigid yaw RMS deg |
| --- | --- | --- | --- | --- | --- | --- |
| Taper | 0.023703805 | 0.023768270 | 4.92631353e-06 | 4.93971245e-06 | 0.057411363 | 5.213437227 |
| Full | 0.026158510 | 0.040260838 | 0.073043299 | 0.099286219 | 0.066834803 | 5.950349118 |
| No-relative | 0.028378514 | 0.046835270 | 5.88734914e-06 | 9.68022214e-06 | 0.070601511 | 6.469142327 |


| Node | Taper displacement m | Full displacement m | No-relative displacement m |
| --- | --- | --- | --- |
| 0 | 0.213333346 | 0.211144018 | 0.213333346 |
| 1 | 0.189567298 | 0.205470712 | 0.210039074 |
| 2 | 0.165842403 | 0.190337280 | 0.197196920 |
| 3 | 0.142099432 | 0.163595912 | 0.170562279 |
| 4 | 0.118331201 | 0.126830671 | 0.129827873 |
| 5 | 0.094624345 | 0.086125715 | 0.082992638 |
| 6 | 0.070989871 | 0.049517694 | 0.042564477 |
| 7 | 0.047345409 | 0.022905791 | 0.016082122 |
| 8 | 0.023737354 | 0.007841660 | 0.003299264 |
| 9 | 0.000000000 | 0.002186831 | 0.000000000 |

B-to-reference geometry distinguishes first-row displacement from the continuous
nearest projection used for evaluation. Reliable chord threshold is .02 m.

| Method | B→row 0 distance m | B→row 0 yaw deg | Projection distance m | Projection yaw deg | First chord vs B deg |
| --- | --- | --- | --- | --- | --- |
| Native | 0.213324006 | 18.000818357 | 0.106648569 | 30.000556457 | 29.992462379 |
| Taper | 0.000010965 | 18.000774020 | 0.000010965 | 18.000774020 | 35.207124175 |
| Full | 0.002180501 | 18.051435831 | 0.001085789 | 18.207767451 | 31.099832354 |
| No-relative | 0.000010965 | 18.000774020 | 0.000010965 | 18.000774020 | 30.630335986 |

No-relative versus Full maximum reference-node XY difference is .006967254552074201 m;
maximum sampled execution XY difference is .019981176831461167 m. Full versus Taper
has maximum reference difference .024495079760069642 m and execution difference
.019981254987017097 m. The plots therefore overlap strongly and use distinct styles
and markers; these curves are not exactly identical.

### Factor costs and solver

L in the saved keys means the state-shift/transport factor S. These objectives have
different terms and their totals are not an execution-performance score.

| Condition | E_S | E_R optimized | E_A | Optimized total | R diagnostic only |
| --- | --- | --- | --- | --- | --- |
| Full | 0.348589943 | 0.068480118 | 0.348193565 | 0.765263626 | N/A |
| No-relative | 0.345923097 | N/A | 0.345422957 | 0.691346054 | 0.080534007 |

For No-relative, R=.08053400706931935 is **diagnostic relative-edge distortion,
not optimized cost**. It was absent from every optimization vector/cost.
The initial E_S+E_A was 4.551116373117762. The solver ended by `step_tolerance` at
iteration 3 with two accepted steps, zero unsafe rejections and no restart;
recorded wall time .044227928010514006 s. Full is byte-identical to its original
frozen file; no reconstruction or numerical Full solve occurred.

### Figures and saved artifacts

[Static review](../data/osa03_relative_factor_ablation_01/primary_20260929T050000Z/index.html)
contains all nine visually inspected figures and their validated numeric/hash
sidecars. [Review ZIP](../data/osa03_relative_factor_ablation_01/primary_20260929T050000Z/review_bundle.zip).
The .25 m expanded cart outline is a robot-center exclusion boundary; the .20 m
footprint circle is not tested against a second copy of that radius.

Small tracked artifacts: [summary](../results/osa03_relative_factor_ablation_01/result_summary.json),
[primary CSV](../results/osa03_relative_factor_ablation_01/primary.csv),
[signed gaps vs Native](../results/osa03_relative_factor_ablation_01/signed_gaps.csv),
[signed gaps vs Full](../results/osa03_relative_factor_ablation_01/signed_gaps_vs_full.csv),
[secondary CSV](../results/osa03_relative_factor_ablation_01/own_reference_secondary.csv),
[freeze ledger](../results/osa03_relative_factor_ablation_01/freeze_summary.json).
Tracked CSV copies use LF line endings; their parsed values match the saved report
CSVs. Large state/command/result/solver traces and arrays remain under ignored data/.

## Research interpretation

Removing E_R gave a **small earlier-attachment and early-position-error benefit**
under this source and frozen schedule. No-relative attached .0500000026 s (three
integration ticks) before Full and .0333333351 s before Native. Its .9 s position
AUC was lower than Full by .0038429248 m·s, with .0050754834 m more swept clearance
and .0050368361 m less final endpoint error. It retained all original downstream
progress and attached with more original arc remaining.

The planning change is modest in XY: at most 6.97 mm relative to Full. Removing R
increases translation-edge distortion RMS from .026158510 to .028378514 m and maximum
from .040260838 to .046835270 m; it also removes almost all of Full's .341394 degree
node yaw correction. The unchanged official selector chooses different early rows
(Full nearest row 1 versus No-relative row 0 at submit 102). These recorded changes
are compatible with controller/selector sensitivity to small reference differences;
this experiment does not isolate a continuous geometry-to-command mechanism.

The result is mixed in commands: No-relative raises linear TV by .3999984081 m/s,
while angular TV decreases by .0313395682 rad/s. Its first .3 s yaw AUC is higher by
.0000028801 rad·s, whereas .9 s yaw AUC is lower by .0013882661 rad·s. No scalar
score is used to erase these trade-offs. Maximum |omega| is essentially the same
for Full and No-relative. Removing a spatial factor does not directly command a
faster robot rotation; the official MPC computes omega(t).

No-relative also has lower measured tracking error against its own reference.
Its own-reference .9 s position AUC .087873797 is much lower than
its original-FRESH .166121029: early attachment to the modified path and recovery
toward original FRESH are distinct questions. Intrinsic original-FRESH/interface
tracking difficulty remains visible in every rollout.

Full and simple Taper are numerically distinguishable here, with at most 19.98 mm
execution separation. Taper has lower .9 s position and yaw AUC, while Full attaches
one tick earlier and has less linear command variation. There is no uniform Full
advantage. This ablation supplies bounded evidence that the current E_R restricts
some useful early deformation in this source/schedule, while also reducing edge
translation distortion and linear command variation. Its general necessity is
not established, and this result is not a new final-method recommendation.

## Limitations

- One source, OSA03 REPEAT_00; no replication or population claim.
- Offline timing-controlled scheduler, not real asynchronous deployment timing.
  Host solves finish while the simulation is paused; measured wall times are not
  online latency results.
- The common already-applied initial command is fixed; its switch cannot improve.
- No controller-aware factor, command factor or alternate selector was introduced.
- Original FRESH itself has intrinsic tracking difficulty, already measured in the
  observation-start control and retained here.
- Attachment is a sampled .30 s dwell, not a continuous-time proof. Safety uses
  the existing kinematic model and frozen direct geometry/uncertainty conventions.
- Small discrete-tick and selector-sensitive effects have no population uncertainty
  estimate. No material-effect threshold was predeclared.
- Relative edges, endpoint and projected progress are intent proxies; the comparison
  does not prove semantic intent preservation.

## Not demonstrated

- General graph superiority.
- Real-world improvement.
- Complete obstacle bypass.
- Online repeated-chunk improvement.
- Necessity of E_R across sources.

## Commands and final verification

Pre- and post-execution relevant suites: **384 passed, 1 skipped** each (19 new
checks; the skip is the absent historical EXP-01B/EXP-02B corpus). Initial development
run had 13 passed/2 mock-fixture setup failures; the corrected focused run passed 15,
then four more tests were added before the 384-test freeze. These fixtures never
called real MPC or LightNav. Compileall and diff checks passed. Saved-only validators
and figure checks made zero new scientific solves. All frozen code/config bytes
remain unchanged after scientific output. No scientific protocol deviation.

Environment tooling note: the sandbox launcher failed before the initial fetch
and reads (`mountinfo path is not absolute`); approved command execution was used.
The same image-viewer launcher failure was bypassed by reading existing PNG bytes
for visual inspection. Neither affected the experiment or changed environments.

Exact scientific and verification commands from repository root:

```bash
git fetch origin main
.venv/bin/python scripts/run_osa03_relative_factor_ablation01.py --mode prepare --run data/osa03_relative_factor_ablation_01/primary_20260929T050000Z
.venv/bin/python scripts/run_osa03_relative_factor_ablation01.py --mode preflight --run data/osa03_relative_factor_ablation_01/primary_20260929T050000Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_osa03_relative_factor_ablation01.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
.venv/bin/python scripts/run_osa03_relative_factor_ablation01.py --mode freeze --run data/osa03_relative_factor_ablation_01/primary_20260929T050000Z
git diff --cached --check
git commit -m "Freeze OSA03 relative-factor ablation with controlled logical applications"
git push origin main
.venv/bin/python scripts/run_osa03_relative_factor_ablation01.py --mode execute --run data/osa03_relative_factor_ablation_01/primary_20260929T050000Z
.venv/bin/python scripts/validate_osa03_relative_factor_ablation01.py --run data/osa03_relative_factor_ablation_01/primary_20260929T050000Z
.venv/bin/python scripts/report_osa03_relative_factor_ablation01.py --run data/osa03_relative_factor_ablation_01/primary_20260929T050000Z
.venv/bin/python scripts/report_osa03_relative_factor_ablation01.py --run data/osa03_relative_factor_ablation_01/primary_20260929T050000Z --validate-only
# The same complete regression command above was run once more after execution.
.venv/bin/python scripts/validate_osa03_relative_factor_ablation01.py --run data/osa03_relative_factor_ablation_01/primary_20260929T050000Z --check-only
.venv/bin/python -m compileall -q src scripts tests
git diff --check
git diff --cached --check
git commit -m "Report timing-controlled OSA03 relative-factor ablation trade-offs"
git push origin main
```

## Explicit answers

1. **Did removing E_R materially change planning geometry?** It changed geometry
   measurably but modestly: maximum Full-versus-No-relative node displacement 6.97 mm,
   edge translation RMS .026159→.028379 m, maximum node yaw correction
   .341394→.000044337 degrees. No wholesale shape change or self-intersection;
   a materiality cutoff was not predeclared.
2. **Did removing E_R reduce early separation/AUC against ORIGINAL FRESH?** Yes for
   position: .5 s maximum fell by .006150314 m and .9 s AUC by .003842925 m·s versus Full.
   Initial error is identical. Yaw is mixed: .3 s AUC slightly higher, .9 s lower.
3. **Did removing E_R produce earlier sustained attachment?** Yes: 1.433333408 s
   versus Full 1.483333411 s, three integration ticks earlier, with the full dwell.
4. **Did removing E_R hurt reference or execution safety?** No observed safety harm.
   Reference clearance .229486890 m and execution lower bound .138784907 m both
   exceed .05 m and exceed Full's .227809208/.133709423 m. No abort or repair.
5. **Did removing E_R damage downstream recovery / original-FRESH intent proxies?**
   No observed endpoint/progress damage: exact original reference endpoint, final
   original arc fraction 1, and lower execution endpoint error. Relative-edge
   translation distortion increased, and semantic intent remains unproven.
6. **Is Full Local-SE2 still distinguishable from simple Taper in this source?**
   Numerically yes, with maximum execution separation 19.98 mm. Taper has lower
   .9 s position/yaw AUC; Full attaches one tick earlier and has lower linear TV.
   Neither has a uniform advantage.
7. **Based on this experiment only, is there evidence that E_R is helping,
   hurting, or not identifiable?** Mixed: E_R helps translation-edge regularization
   and reduces linear command variation; it hurts the measured early-position and
   attachment outcomes in this source/schedule. General necessity is not identifiable
   from this one-source experiment.
