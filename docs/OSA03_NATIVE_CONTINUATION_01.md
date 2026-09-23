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
