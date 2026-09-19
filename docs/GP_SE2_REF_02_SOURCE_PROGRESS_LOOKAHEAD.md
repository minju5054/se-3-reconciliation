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
is assessed by the six prescribed rollouts. Full results and final verification
will be appended after execution. Static index/PNG/ZIP evidence is primary;
optional GUI is not planned for this diagnostic.
