# GP-SE2-REF-02: fixed-geometry source-progress lookahead

This diagnostic changes only the post-nearest reference-selection stride on the
same stored dense FRESH path. **MPC computation is identical; the reference
selector differs.** It is not a repeat of REF-01's 2×2 input intervention, and
not a GP/rigid optimization or new navigation episode.

## Source and fixed scope

Starting and fetched origin/main are
`9a615cee443848e79fff8805e9a038bbbcb2c341`. No REF-02 implementation existed at
startup. Both unrelated Stage 0 user configuration edits remain untouched.
Primary input is `data/robotless_gp_se2_ref_01/primary_20260919T062000Z/`, whose
final authoritative validator passes 709,970 checks; its earlier reporting
preflights are not substituted for that final result. Related source resolves to
`data/robotless_gp_se2_02/primary_20260919T024000Z/`. Original config and validated
environment resolve through that provenance to GP-SE2-01. Raw/derived arrays,
lineage, results, figures, numerical core and external MPC remain immutable.
The startup preservation audit checks 41,033 paths plus both user config hashes.

REF-01 observed large-turn Native success and adapter yaw/dwell failure. Suffix
alone succeeded; resampling alone failed. Matched-state probes showed changed
lookahead, but that experiment did not hold the stored dense geometry fixed
while changing target-selection spacing. REF-02 supplies that paired intervention.

New output: `data/robotless_gp_se2_ref_02/primary_20260919T081000Z/`.
The order is fixed before any probe/solve:

| Case | Event | A_NATIVE | B_DENSE_ROW_STEP | C_DENSE_SOURCE_PROGRESS |
|---|---|---|---|---|
| PRIMARY_LARGE_TURN | episode_014_repeat_01/handoff_026 | REF-01 R00 | REF-01 R11 | same R11 |
| BENIGN_CONTROL | episode_001_repeat_01/handoff_002 | REF-01 R00 | REF-01 R11 | same R11 |

Each case runs A, B, C once. Native has 10 rows in these inputs, but the selector
is not hard-coded to that length. Both dense inputs are float64 30×3 with exact
shape/dtype/value/file-hash identity. Lineage is copied byte-for-byte, not
reconstructed by projecting coordinates. Dense source progress ranges are 1–9
(large turn) and 2–9 (benign). The fixed dense file SHA256 values are:

- Large turn: `112782148b776a7d9ed09d60e26c6ecb30d66e9d06c783315806bdf11b0ce67f`.
- Benign: `4648cfd98f6fa555824d36b1f96581bcf4fb6f75cc967649a343a2fe95ac55d8`.

No suffix is recalculated, no row is resampled, no new point is interpolated,
and C never substitutes Native poses. Original B, physical u_minus, recorded
controller previous_control, capture transform, original goal/route, footprint,
clearance and all acceptance configuration remain unchanged.

## Selector and floating-point policy

Let C be the installed dense world rows, s the stored original fractional row
coordinates and R the current world pose. Both B and C find j using the same
original weighted squared XY/wrapped-yaw cost, actual Q_WEIGHTS=(10,10,1), scalar
atan2(sin,cos) wrapping and NumPy first-argmin tie handling. The path array and
nearest state are identical in matched-state comparisons.

A/B use official indices `min(j+h,M-1)` for h=1,…,H. For C:

```text
q_h = min(s_j + h, s_last)
i_h = first i such that s_i >= q_h
reference = C[i_h].copy()
```

H=5, stride=1.0 original-row unit. Every q_h starts from the same unrounded s_j;
there is no accumulated ceil overshoot, progress memory, rounding, native-path
nearest lookup or external suffix recut. Implementation is CPU float64 strict
`searchsorted(s,q_h,side='left')`, with **zero search tolerance**. Each returned
row's q, selected index/s, nonnegative overshoot and endpoint clamp/repeat are
saved. After selection the original scalar sequential yaw unwrap begins at R's
yaw; XY and periodic yaw remain those of selected existing installed rows.

Ordinary progress must be finite and strictly increasing. Repeated poses with
different progress remain distinct. Repeated-single-source special input needs
explicit constant-reference/source-row metadata plus identical pose rows; it
repeats the last row. Other invalid lineage fails explicitly. Identity tests on
unresampled `s_i=i` reproduce the official selector, including yaw branches and
endpoint repetition. Synthetic fixtures are correctness tests only.

Progress is an original row-order label, not physical time, distance, learned
correspondence or an optimal speed schedule. Discrete ceil quantization and the
different Native/dense nearest sets prevent a general sampling-invariance claim.

## Runtime wrapper and actual-reference evidence

The official checkout remains read-only at
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`; mpc.py SHA256 is
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
No external source is copied or edited. Each diagnostic instance binds a fresh
function with the original `MpcTracker.submit` code object and a private globals
dictionary whose only changed binding is its selector callback. Official global
bindings are untouched; asynchronous instances cannot select through another
instance's callback. A/B call the official selector; C calls the new selector.

Original `_solve`, coordinate conversion, generation/future management, `poll`,
path installation and controller mathematics are used directly. An instance-local
recorder copies the actual `MPCController.solve` local arguments then delegates
to the original bound method. The existing counterfactual integration loop's code
object is reused with a private instrumented synchronous-solve entry point.
Tests and runtime identity flags verify this limited scope, failure behavior,
command memory and instance isolation. This is not a claim that the entire
controller is unchanged: reference selection is explicitly modified.

Each solve saves the actual selector reference submitted to `_solve`, actual
controller-local reference/pose/memory/limit and returned reference/prediction.
These are compared through the original coordinate transform. A/B retain the
original consecutive-row audit. C uses an independent linear lower-bound scan,
not its production searchsorted, and checks actual XY, periodic yaw and literal
sequential unwrap. No audit is disabled or forced to accept the other method.

## Probes, rollout, evaluation and predeclared interpretation

Before rollouts, 180 selector-only probes use the exact same 30 historical Native
input poses per case as REF-01. The original capture inverse and official world
projection produce the same installed arrays as actual rollout, without creating
a tracker or solving MPC. These arrays/transforms are saved in probe installation
metadata. All B/C nearest indices/progress/cost terms must agree exactly at the
same pose. Only subsequent target indices may differ. A/C target differences are
measured without requiring them to agree. Plot snapshots are fixed at 0,1,2 s.

Six independent tracker instances perform 180 primary MPC solves, zero additional
historical solves, at t=0,0.1,…,2.9 s. Each rollout integrates held commands exactly
at 60 Hz for 3 s, giving 181 states/180 ticks/30 solves. It preserves the original
capture-frame inverse/installation roundtrip and separate physical command versus
controller memory. No VLA update, pose snap, artificial stop, future RGB reuse or
wall-time insertion into simulation occurs. Original solve failure/hold policy
and post-collision recording scope are preserved.

Original `evaluate_rollout` supplies the same 0.15 m / 15° goal limits, final
0.20 s dwell, footprint/clearance/workspace/route and command-grid motion checks.
Achieved final sampled dwell is a separate diagnostic. Missing goal times remain
N/A. C's final selected target never redefines the original FRESH goal.

A/B compare their first prescribed primary run with stored REF-01 R00/R11 using
unchanged REF-01 tolerances: selected poses 1e-12, commands/states/endpoint metrics
1e-6 absolute (rtol=0), exact indices/settings/initial inputs/success flags.
Bitwise equality is separate from numerical agreement. No tolerance is enlarged
based on outcomes. A/B reproduction plus fixed B/C geometry, matched nearest and
validated actual reference input are prerequisites for confirming C's effect.

Full fixed-event recovery requires B failure and C passing **all** original
conditions. Lower yaw error with failed success is partial recovery, and new
safety/motion failures are a regression or tradeoff. Benign success preservation,
command TV, time to goal and clearance are separate results. No outcome is
hard-coded into tests; an unsuccessful C can still produce valid artifacts.
The frozen command-divergence diagnostic uses 1e-6 native command units.

Pre-execution validation: 151 relevant tests passed; compileall and diff checks
passed. A/B wrapper parity is tested with hash-pinned official runtime code and a
fake numerical controller, without extra experimental MPC solves. Actual parity
is assessed by the six prescribed rollouts. Static index/PNG/ZIP evidence is
primary; optional GUI is not run for this diagnostic.

## Actual results and baseline reproduction

Execution source was frozen at
`1801da1603439182bb95ed38d706fe776ab88e92` before any primary selector probe or
MPC solve. All six prescribed first runs completed in order, with no retry,
controller failure or additional historical solve. No scientific implementation,
configuration, threshold or selector rule changed after that freeze.

All four A/B reproductions are **bitwise equal** to the corresponding REF-01
records for selected indices, references (including literal unwrapped yaw),
controller memory, commands, states, predictions and compared outcome metrics.
This is stronger than the predeclared numerical agreement check. It does not
claim wall-time equality. Both copied and installed B/C arrays are exactly equal;
the original lineage bytes and all original goals remain unchanged.

| Case | Method | Original success | Position error (m) | Absolute yaw error (deg) | Goal time (s) | Dwell pass | Achieved final sampled dwell (s) |
|---|---|---|---:|---:|---:|---|---:|
| Large turn | A Native | PASS | 0.008370718 | 0.332938812 | 1.67 | PASS | 1.33 |
| Large turn | B dense row step | FAIL | 0.025552086 | 38.827543097 | N/A | FAIL | 0.00 |
| Large turn | C dense source progress | PASS | 0.006143463 | 0.255007664 | 1.58 | PASS | 1.42 |
| Benign | A Native | PASS | 0.000060895 | 0.000012344 | 1.28 | PASS | 1.72 |
| Benign | B dense row step | PASS | 0.016289849 | 0.001754224 | 2.59 | PASS | 0.41 |
| Benign | C dense source progress | PASS | 0.000060879 | 0.000011845 | 1.28 | PASS | 1.72 |

B's large-turn failures remain `GOAL_YAW_FAILURE` and `GOAL_DWELL_FAILURE`.
All six rollouts pass the unchanged footprint collision, required clearance,
known workspace, command-grid motion and applicable route checks. Minimum edge
clearance is 0.302121251 m for all three large-turn methods and 1.326944970 m for
all three benign methods. Neither case has a required gate. The minimum often
occurs in the shared start segment, so equal minima do not imply identical paths.
Small numerical excursions around acceleration bounds remain subject to the
**original** acceptance tolerance; no new tolerance was introduced.

Thus the large-turn B→C intervention is `FULL_FIXED_EVENT_RECOVERY` under the
original fixed 3 s conditions. Benign success is preserved. There is no new
safety, motion, goal or controller-failure regression in these two cases.
This is local execution evidence for a reference-selector intervention, not a
GP result or a population navigation success rate.

## Mechanism: fixed input → selected targets → commands → outcome

All 60 paired B/C same-state probes preserve the installed path, nearest index,
nearest original progress and every cost component exactly. The post-nearest
targets differ in 24/30 large-turn states and 15/30 benign states. At the
large-turn initial pose both methods have j=0 and s_j=1:

| Method | Derived target indices | Original progress of selected rows |
|---|---|---|
| B | 1, 2, 3, 4, 5 | 1.275862, 1.551724, 1.827586, 2.103448, 2.379310 |
| C | 4, 8, 11, 15, 19 | 2.103448, 3.206897, 4.034483, 5.137931, 6.241379 |

C's requested q values are exactly 2,3,4,5,6. The gaps between requested and
selected progress are the stored discrete-ceil overshoot, not new interpolated
poses. At this same pose, B's sequential target yaw runs from 79.85185° to
59.75841°; C's runs from 64.51873° to −1.73588°. Their selected XY arc lengths
are 0.016220 m and 0.032159 m respectively. A longer row-order lookahead here
exposes substantially more of the required rotation, even though the movement
in XY is small. These labels are not waypoint timestamps.

| Case | Evidence stream | A first final-goal inclusion | B | C |
|---|---|---:|---:|---:|
| Large turn | Same historical input states | 0.9 s | 1.8 s | 0.9 s |
| Large turn | New independent closed loop | 0.9 s | N/A | 0.9 s |
| Benign | Same historical input states | 0.5 s | 1.3 s | 0.5 s |
| Benign | New independent closed loop | 0.5 s | 2.5 s | 0.5 s |

After its first inclusion the final goal remains in each available sequence's
selected horizon. The matched-state and closed-loop numbers are intentionally
separate: B's later state divergence prevents the large-turn final row from
entering its actual horizon, despite its entry in the historical-state probe.
At matched t=1 s in that case, both dense methods have j=12, s_j=4.310345.
B selects progress 4.586207→5.689655; C selects 5.413793→9 and includes the goal.
At matched t=2 s, C repeats the final row five times, while B still includes
earlier rows before repeating the endpoint twice.

Actual submitted world targets and actual controller-local arguments agree with
these independently audited selections. Targets differ at t=0; the first
large-turn B/C command difference above the frozen 1e-6 threshold occurs at
t=0.3 s. The first three commands are nearly equal under the common acceleration
bounds. At 0.3 s B applies [0.729815898, −1.136243292] and C applies
[0.783597317, −1.513881775] in [m/s, rad/s]. At 1 s their angular commands are
−0.650448342 and −1.540487314 rad/s. At the last solve (2.9 s), B still commands
−0.425055046 rad/s and ends with 38.827543° yaw error; C commands
−0.014903028 rad/s and ends with 0.255008° error. B's failure is in this fixed
horizon, not evidence that rotation could never finish.

For benign, the first angular command difference is at t=0 (about 2.16e-5 rad/s)
and the first linear difference is at 0.1 s: B commands 0.602501217 m/s, C 0.8 m/s.
Earlier goal inclusion and faster progress accompany a 1.31 s earlier goal entry
and 1.31 s longer achieved sampled final dwell. This has a measurable tradeoff:
linear command total variation increases from 0.539192505 to 0.799988233 m/s,
and maximum linear acceleration from 1.974987832 to 1.999999977 m/s², while
remaining valid. Angular command TV decreases from 0.001358974 to
0.001127201 rad/s. Clearance is unchanged. A and C both reach the benign goal
at 1.28 s; their linear commands differ by less than 1e-6 m/s.

C is not an exact Native reconstruction. Maximum positive ceil overshoot is
0.241379310 original-row units in the large turn and 0.206896552 in benign.
Matched A/C targets differ at 19/30 and 13/30 states respectively. Their maximum
target XY discrepancies are 0.017577628 m and 0.181089790 m; maximum periodic
yaw discrepancies are 20.494261° and 0.007650°. Native/dense nearest sets also
differ. C's large-turn goal time is 0.09 s earlier than A's, but that single local
result does not establish general superiority or sampling invariance.

The intervention identifies target stride as sufficient to recover this stored
large-turn dense input under the frozen MPC calculation. The logs localize the
chain through selected rotation/goal horizon and actual commands. They do not
show that every REF-01 geometric change is irrelevant in other cases, or that
this mechanism explains GP between-point motion violations.

## Compute and software

Actual runtime: existing isolated Python 3.11.16, NumPy 2.4.6, CasADi 3.7.2;
single-thread BLAS, unchanged pinned official source/settings. No environment or
system Python was modified. There are 180 selector-only probe calls and another
180 actual rollout selector calls, 180 actual MPC solves, zero historical audit
solves, zero GP/rigid solves and zero VLA updates.

| Case | Method | 30 MPC solves, reported total (s) | Rollout wall (s) | Independent evaluation (s) |
|---|---|---:|---:|---:|
| Large turn | A | 0.169663 | 0.190975 | 0.052736 |
| Large turn | B | 0.115066 | 0.147911 | 0.038172 |
| Large turn | C | 0.130271 | 0.166080 | 0.037402 |
| Benign | A | 0.108734 | 0.128494 | 0.038320 |
| Benign | B | 0.083238 | 0.115589 | 0.039334 |
| Benign | C | 0.098264 | 0.131708 | 0.038151 |

Preparation was 6.548555 s, including 6.468879 s for preservation hashes.
The worker subprocess took 1.319402 s; its internal 1.247870 s includes
0.013012 s runtime loading, 0.147308 s probes and 1.043952 s rollout batch.
The 0.705236 s sum of reported MPC solve times is **inside** that batch, not an
additional cost. Evaluation environment load took 0.043133 s; evaluation and
aggregate construction took 0.388508 s (including the table's per-method checks).
Nonoverlapping preparation + worker process + environment load + evaluation is
8.299599 s. Plotting, packaging, tests, independent artifact validation and human
report preparation are outside that computational subtotal. These one-pass times
are descriptive; C was not uniformly cheaper than B, and no online latency or
real-robot claim follows from this offline calculation.

## Evidence, validation and reproduction commands

Primary numerical record, all failures, 22 PNGs, numeric/hash sidecars, index and
review archive are in `data/robotless_gp_se2_ref_02/primary_20260919T081000Z/`.
`aggregate/outcomes.csv`, `paired_comparison.csv`, `selection_mechanism.csv` and
`command_divergence.json` retain raw values and N/A. `execution_freeze.json`
records the execution SHA plus every frozen code/input hash. Final preservation
and authoritative independent validation are separate from the earlier preflight.

Visual inspection found overlapping table headings in the two frozen
`fixed_input_world.png` figures. Original primary PNGs, index and ZIP remain
preserved. An additive presentation-only output,
`data/robotless_gp_se2_ref_02/presentation_20260919T_ref02/`, fixes that layout,
copies the other 20 PNGs unchanged and records source hashes. It performs **zero**
new probes, solves or numerical evaluations; this is not a replacement primary
run. Both numerical sidecars and all scientific results remain the same.
No Isaac GUI runtime was executed: `NOT_RUN_STATIC_DIAGNOSTIC_PRIMARY`.

From the repository root (the primary run already exists; these commands refuse
to overwrite/retry it):

```bash
.venv/bin/python scripts/run_gp_se2_ref02.py prepare --run data/robotless_gp_se2_ref_02/primary_20260919T081000Z
.venv/bin/python scripts/run_gp_se2_ref02.py freeze --run data/robotless_gp_se2_ref_02/primary_20260919T081000Z
.venv/bin/python scripts/run_gp_se2_ref02.py execute --run data/robotless_gp_se2_ref_02/primary_20260919T081000Z
.venv/bin/python scripts/run_gp_se2_ref02.py evaluate --run data/robotless_gp_se2_ref_02/primary_20260919T081000Z
.venv/bin/python scripts/plot_gp_se2_ref02.py --run data/robotless_gp_se2_ref_02/primary_20260919T081000Z
.venv/bin/python scripts/package_gp_se2_ref02_review.py --run data/robotless_gp_se2_ref_02/primary_20260919T081000Z
MPLCONFIGDIR=/tmp/ref02_presentation_mpl .venv/bin/python scripts/present_gp_se2_ref02.py --primary data/robotless_gp_se2_ref_02/primary_20260919T081000Z --output data/robotless_gp_se2_ref_02/presentation_20260919T_ref02
.venv/bin/python scripts/validate_gp_se2_ref02_presentation.py --presentation data/robotless_gp_se2_ref_02/presentation_20260919T_ref02
.venv/bin/python scripts/validate_gp_se2_ref02.py --run data/robotless_gp_se2_ref_02/primary_20260919T081000Z --preflight
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
.venv/bin/python scripts/run_gp_se2_ref02.py finalize --run data/robotless_gp_se2_ref_02/primary_20260919T081000Z
.venv/bin/python scripts/validate_gp_se2_ref02.py --run data/robotless_gp_se2_ref_02/primary_20260919T081000Z
```

The execute phase invokes the existing isolated official MPC Python recorded in
`execution_started.json`, with the exact worker command and request hash. The
validator only recalculates records; it never runs MPC/GP. Initial independent
preflight passed 1,061,371 checks with zero errors in 8.009 s, deferring only the
final preservation and artifact manifest. Final verification is recorded below.
Presentation-only independent validation passed 1,017 checks in 0.309 s; a
separate read-only 340-check audit also verified exact ZIP bytes, primary
preservation and unchanged numbers. The presentation archive is 6,500,087 bytes,
SHA256 `274e74f4cc1e2f4cccc75fa72bc3d3c7e44b9ff3ed0e9eeafef43aa09fa9bde3`.

Final authoritative `validation.json` passes **1,102,800 checks**, zero errors,
zero deferred checks, in 8.058177 s. It performs no MPC/optimizer solves. The
final original-source audit preserves all 41,033 paths, both unrelated user
config hashes, the external checkout SHA and clean external status. Original
scientific code and frozen execution code remain unchanged.

Required full pytest: **1,972 passed, 19 skipped** in 93.89 s; log
`pytest_authoritative.log`. Eighteen skips concern missing ignored historical
corpora; one is the existing S3 exact-reproduction exclusion. No REF-02 test is
skipped. Compileall and `git diff --check` pass; no shell launcher changed.
Earlier 1,965/1,970-test runs remain recorded; reporting tests added afterward
are included in the final full run. No successful scientific outcome is asserted
by the tests. No validator correction or scientific rerun was required.

Final machine-readable outcome (also in `aggregate/summary.json`):

```text
operational_status = GP_SE2_REF_02_COMPLETED
baseline_reproduced = true
fixed_dense_reference_preserved = true
matched_state_nearest_preserved = true
source_progress_selection_validated = true
large_turn_success_recovered = true
benign_success_preserved = true
new_safety_or_motion_regression = false
mechanism_evidence_level = FIXED_GEOMETRY_MATCHED_NEAREST_SELECTION_COMMAND_OUTCOME
```

Starting and execution SHAs are above. The final report commit SHA and normal
current-branch push result are recorded in the ignored primary
`git_completion.json`, excluded from the artifact manifest to avoid a
self-referential commit hash. Commits contain only code/config/tests/docs;
generated arrays, figures, archives and external environment are not committed.

## Remaining uncertainty and next single experiment

Only two prescribed development-corpus events were evaluated. Source progress
has row-order semantics, not physical timing; coarse ceil selection, different
Native/dense nearest choices and low-speed/rotation-only paths remain relevant
limits. The benign result also exposes a speed-versus-command-variation tradeoff.
The next single experiment is a predeclared B/C transfer comparison on additional
existing handoffs, including obstacle-near and rotation-dominant events, with this
same stride/nearest/MPC/acceptance frozen and all regressions retained. Do not
tune the rule on those results. Applying original provenance to GP-deformed paths
remains a separate question; this task does not resolve GP continuous-time
feasibility, general navigation or online execution.
