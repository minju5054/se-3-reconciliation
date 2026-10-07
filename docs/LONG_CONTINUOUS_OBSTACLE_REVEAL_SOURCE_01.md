# LONG_CONTINUOUS_OBSTACLE_REVEAL_SOURCE_01

## Frozen protocol before science

Starting HEAD and fetched origin/main: `c0a862acb45449db6269aada8bd8ff910136ddd0`.
Run: `data/long_continuous_obstacle_reveal_source_01/primary_20261007`.
This is one new source-acquisition episode: one full Isaac launch, no retries or
source search, at most ten terminal predictions C0–C9. No reconciliation is
implemented or evaluated. EXPLORATORY_02 and all inherited authority remain
immutable; saved-only validators and Git-object comparisons authenticate them.

### Native controller speed configuration

The effective EXPLORATORY_02 linear bound is **0.8 m/s**, authenticated from its
saved worker provenance and pinned official `_solve` argument. TRACK_V_MAX=1.5
is not the effective bound. This experiment sets **0.4 m/s**, exactly 0.5 times
the prior bound. External MPC source SHA256:
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
External LightNav Git SHA: `c6f40e3220edbf7011e4f17eaf2c865416737d4d`.

The isolated worker loader sets module `OBJNAV_V_MAX` before the unchanged worker
constructs its tracker. The pinned tracker reads this global at `_solve`, passes
it to `MPCController.solve`, and that method sets the existing CasADi `v_max`
parameter before solving. The preexisting bound equation and official command
extraction are unchanged. No added output scaling or clipping is performed.
There is no external source modification, equation change or environment install.

The objective, H=5, nearest+1 selector, Q=[10,10,1], R=[.1,.1], control dt=.1 s,
W_MAX=3 rad/s, A_MAX_V=2 m/s², A_MAX_W=5 rad/s², IPOPT, generation/stale handling
and previous-command semantics remain unchanged. Each submit's controller memory
must equal the last accepted official result; every actual non-timeout command
must exactly equal its official solve result. The inherited explicit timeout-zero
behavior is retained. Official solved and applied speeds must obey the new bound.
The speed gate fails before science as SLOW_EXECUTION_CONFIGURATION_BLOCKED if
authentication or this isolated configuration is impossible. No alternate
slowdown mechanism is permitted. No scientific solve is used for preflight.

### Source, model input and safety

POSE11 initial world pose [m,m,rad]:
`[19.20312073159454, 24.423334915767065, -1.5689742328041627]`.
The Hospital scene, cart asset/transform, camera/BRIGHT, model/checkpoint, history,
generation parameters and instruction are identical to EXPLORATORY_02:

> Go to the far end of the hallway. If the path is blocked, pass the supply cart on your right without touching it. Continue straight after passing it and stop at the end of the hallway.

Only RGB/history and the instruction are delivered through the unchanged official
protocol. No radius, map, obstacle coordinates, clearance or collision information
is provided to LightNav. History sampler reconstruction is distinct from direct
server telemetry. Exact serialized input/response and image hashes are retained.

The existing exploratory geometry adapter is reused: radius .20 m, required
extra clearance .00 m, legacy .05 m diagnostic. Uncertainty, workspace, numerical
reserve, curved-command bound and fail-closed unknown/borderline handling are
unchanged. Every complete raw reference is checked before installation, without
inventing a connector. Guard lookahead and every actually integrated interval
receive both policy checks. A rejected raw chunk is preserved but never installed
or applied; no subsequent prediction is requested. No unsafe command is applied,
no repair is performed, and valid execution prefix data remain intact.

### Genuine continuous schedule and stop rule

The original collector, model worker, reveal hook, geometry worker, MPC adapter
and command activation implementation are reused byte-for-byte. A separate
request policy extends the historical four-request bound to ten, preserving its
first-frame, order and one-inflight rules. Only the runtime attempt count and
active cap change: nine handoff attempts, 8.0 s active cap, .10 s final postroll.
Minimum useful length six and preferred length eight are classification thresholds,
not runtime stopping rules. No C10 is requested.

Integration remains 60 Hz, MPC 10 Hz/every six ticks, RGB 4 Hz/every 15 ticks,
with `minimum_wall_step_no_catchup_v1`. Physics is not paused for inference and
no catch-up schedule is substituted. C0 uses the fourth actual stationary RGB
following three buffers, with cart OFF. The cart reveals once before the first
scheduled capture strictly after C0 actual application. That exact capture is C1.
It stays ON/static thereafter. Each next terminal uses the first eligible capture
strictly after the preceding actual application, with one terminal request in
flight. OLD continues physically executing throughout FRESH inference.

Stop at the earliest of C9 applied plus .10 s postroll, rejected next reference,
model STOP, controller failure, execution guard abort, active cap or technical
failure. The unchanged collector has a stale descriptive terminal-reason string
mentioning "1s postroll"; the executed/configured postroll is .10 s and validation
checks its actual six integration intervals. This does not change the runtime.

### Coordinates, clocks and handoff readiness

Raw local rows are immutable cumulative spatial poses, +x forward/+y left,
CCW yaw radians. World is Isaac XY metres, +Z up. Each derived world reference is
`T_world_chunk = T_world_Ak * T_Ak_chunk`, using its own actual observation A_k.
There is no ready/install/B re-anchoring and intrinsic waypoint dt remains null.
B_k exists only at actual first command application; P_k is the preceding saved
state. Host request/receipt timestamps and simulation observation/ready/install/
application timestamps are kept separate. No cross-clock latency is inferred.

The ignored episode holds raw RGB, requests, responses and local NPYs separately
from derived world arrays and the `source_bundle` indices/segments. Every actual
handoff retains OLD/FRESH local/world hashes, A_old/A_fresh, all clocks, B/P,
physical u_minus, controller memory, first command/solve, generation/version,
request/episode IDs, RGB/history and model/freeze provenance, and actual OLD-to-B
execution. Generated-only chunks create no B and no applied-handoff bundle entry.
No correspondence or reconciliation optimization is computed.

### Descriptors and predeclared classification

Every available adjacent generated pair C0→C1 through C8→C9 has raw hash equality,
symmetric maximum vertex-to-continuous-XY-polyline separation, endpoint forward/
lateral/wrapped-yaw change, wrapped net-yaw change, maximum absolute lateral/yaw
excursion change, observation translation/yaw, cart pixels and clearance deltas,
and world-polyline separation. Meaningful change is the frozen OR of separation
>=.02 m, absolute endpoint lateral change >=.02 m, or absolute wrapped endpoint/
net-yaw change >=5 degrees. Missing/STOP-empty geometry stays N/A.

First reaction requires applied C0/C1, actual reveal with C0 OFF/C1 ON, at least
20 visible cart pixels, nonSTOP C1, passing exploratory reference and the existing
meaningful first-response geometry criterion. Historical moving-B/future-progress
criteria remain descriptive. All applied references and actual intervals must be
overlap-free, with one reset only, exact lineage, source authentication and timing
checks. Existing request-local RTF [.8,1.2] and maximum loop stall .25 s remain.
Timing is checked on every observed terminal, including a final rejection/STOP.

Priority:

1. Unavailable clean native speed override: SLOW_EXECUTION_CONFIGURATION_BLOCKED,
   before science.
2. Infrastructure, timing or provenance failure: TECHNICAL_EXECUTION_BLOCKED.
3. Continuous record without qualified C0→C1: FIRST_REACTION_NOT_QUALIFIED.
4. All qualification gates and 10 consecutive applied: LONG_CONTINUOUS_SOURCE_MAX_REACHED.
5. All gates and 8–9 applied: LONG_CONTINUOUS_SOURCE_TARGET_REACHED.
6. All gates and 6–7 applied: LONG_CONTINUOUS_SOURCE_QUALIFIED.
7. Qualified first reaction but only 2–5 applied: PARTIAL_CONTINUOUS_SOURCE.

A later rejected/STOP chunk can coexist with a qualified preceding prefix. Its
exact cause and unavailable B remain explicit. No favorable post-hoc category.

### Reporting and validation

Four primary PNGs: continuous world episode (solid execution colored by active
command, dashed raw references, A/B, cart/boundaries); all own-local chunks;
simulation event timeline with inference intervals and active identity; exact
request RGB grid with metadata in separate axes. With six or more applied, add a
fifth descriptive handoff figure. Rejected/unexecuted references are labeled.
Timeline shading uses runtime-seen simulation events; exact send/receipt host
clocks remain in numeric sidecars. It does not fabricate cross-clock timestamps.

Saved-only validation reuses the historical complete integration/history/controller
validator and independently checks per-inference OLD identity, speed/native-memory
parity, complete reference/actual interval safety, reveal, first-frame ordering,
no skips, raw hashes, lineage, classifications and bundle counts. All validation
and reporting invoke zero LightNav, MPC, Isaac or optimizer calls.

### Commands and freeze discipline

```bash
git fetch origin main
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/long-source01-mpl .venv/bin/python scripts/run_long_continuous_obstacle_reveal_source01.py --mode prepare --run data/long_continuous_obstacle_reveal_source_01/primary_20261007
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/long-source01-mpl .venv/bin/python -m pytest -q tests/test_long_continuous_obstacle_reveal_source01.py tests/test_continuous_obstacle_reveal_exploratory02.py tests/test_continuous_obstacle_reveal_episode01.py tests/test_continuous_obstacle_reveal_episode01b.py tests/test_obstacle_source*.py tests/test_join_online*.py tests/test_online*.py tests/test_robotless_online*.py tests/test_data02*.py tests/test_se2*.py tests/test_join_source02*.py tests/test_gp_se2_environment*.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/long-source01-mpl .venv/bin/python scripts/run_long_continuous_obstacle_reveal_source01.py --mode freeze --run data/long_continuous_obstacle_reveal_source_01/primary_20261007
# Review and stage only this namespace plus documentation; review staged diff.
git diff --cached --check
# Commit, normal push, verify HEAD == origin/main; then:
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/long-source01-mpl .venv/bin/python scripts/run_long_continuous_obstacle_reveal_source01.py --mode verify --run data/long_continuous_obstacle_reveal_source_01/primary_20261007
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/long-source01-mpl .venv/bin/python scripts/run_long_continuous_obstacle_reveal_source01.py --mode start --run data/long_continuous_obstacle_reveal_source_01/primary_20261007
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/long-source01-mpl .venv/bin/python scripts/run_long_continuous_obstacle_reveal_source01.py --mode launch --run data/long_continuous_obstacle_reveal_source_01/primary_20261007
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_long_continuous_obstacle_reveal_source01.py --mode stop --run data/long_continuous_obstacle_reveal_source_01/primary_20261007
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/long-source01-mpl .venv/bin/python scripts/validate_long_continuous_obstacle_reveal_source01.py --run data/long_continuous_obstacle_reveal_source_01/primary_20261007 --seal
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/long-source01-mpl .venv/bin/python scripts/report_long_continuous_obstacle_reveal_source01.py --run data/long_continuous_obstacle_reveal_source_01/primary_20261007
# Reexecute validator without --seal and reporter with --validate, inspect all PNGs.
```

No navigation safety, complete avoidance, reliable instruction-side compliance,
task success, reconciliation benefit, generalization or real-world claim follows.
The speed limit is an acquisition setting intended to increase spatial observation
density. Subsequent method comparisons are outside this task.

Pre-science focused suite: **27 passed** (initial 25, then two additional memory/
optional-figure checks). Relevant regression: **615 passed, 1 skipped**, the absent
immutable generated DATA02 v1 corpus. Compileall and whitespace checks pass.
The socket-based worker tests ran with local Unix-socket permissions. No model,
MPC scientific solve, or Isaac call occurred during preparation/testing. Tests
include a pinned `_solve` AST executed against a stub controller, not a real solve.

## Repository-confirmed results

Pushed scientific freeze: **`56c2c88c58559f9b16005b076dbfe03e4d3e190b`**.
Local HEAD equaled origin/main before server startup and the single Isaac launch.
One real episode generated **three nonSTOP chunks C0–C2**. Exactly **two consecutive
chunks C0–C1** were installed and physically applied. **C2 was rejected before
installation** for physical footprint overlap, clearance **−0.03675014821263675 m**.
No B2, C2 command, C3–C9 request, retry, repair, or continued source acquisition.
The runtime termination is SCENE_INVALID / RAW_UNSAFE at sim **2.0333334393799305 s**.
The desired six-chunk minimum was not acquired.

The frozen top-level classification is **TECHNICAL_EXECUTION_BLOCKED**. This is a
saved scheduler-audit failure described below, distinct from the raw-overlap
condition that actually stopped collection. Do not relabel it PARTIAL_CONTINUOUS_SOURCE.
The partial records are reproducible and all 120 executed intervals are overlap-free;
this does not make the source qualified.

### Native speed intervention observed

The prior/new effective limits are exactly **.8/.4 m/s**, ratio **.5**. The observed
maximum applied absolute linear speed is **.4 m/s**. All ten submitted controller
memories match the previous accepted official commands, all ten solved results
obey the limit, and actual commands exactly match those results. The official
source/checkpoint hashes, selector, objective, horizon, weights, angular and
acceleration limits remain unchanged. The loader configured the bound before
tracker construction. No output command scaling/clipping was added.

The official controller already has numerical output extraction with its passed
v_max; it is unchanged. This is separate from the prohibited added post-solve
slowdown. Full source/config/provenance is in
[speed_intervention.json](../results/long_continuous_obstacle_reveal_source_01/speed_intervention.json).

### Generated chunks, raw geometry and reference safety

Every generated chunk has ten untimed raw poses. C0 was observed with cart OFF;
C1/C2 with cart ON, static at the authenticated transform. C0's hypothetical
cart-ON reference remains obstructed, which is distinct from actual execution.
C1 and C2 have positive local lateral coordinates (left); no reliable instruction-
side compliance claim is made. Each raw output uses its own observation anchor.

| Quantity | C0 | C1 | C2 |
|---|---|---|---|
| Generated / applied | yes / yes | yes / yes | yes / no |
| N | 10 | 10 | 10 |
| Raw XY arc [m] | 1.351820588145103 | 1.353245520115897 | 1.3432522077612163 |
| Endpoint local [forward m,left m,yaw rad] | `[1.501985788345337,3.947936875192681e-06,8.423162398685236e-06]` | `[1.1721760034561157,0.6762442588806152,0.5215919613838196]` | `[1.4554990530014038,0.3231041729450226,0.0200838390737772]` |
| Wrapped net yaw [rad] | 1.0226143785985187e-05 | 0.20741918683052063 | -0.2397757526487112 |
| Maximum absolute lateral [m] | 3.947936875192681e-06 | 0.6762442588806152 | 0.3231041729450226 |
| Maximum absolute yaw [deg] | 0.0009005603795339317 | 30.000670193364716 | 15.152028475506464 |
| Cart visible pixels | 0 | 4073 | 5872 |
| Whole-reference footprint clearance, actual scene [m] | 1.1175140107938666 | 0.23473751791079572 | -0.03675014821263675 |
| Exploratory overlap-free | PASS | PASS | FAIL |
| Legacy extra .05 m | PASS | PASS | FAIL |

Raw local NPY SHA256:

```text
C0 6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae
C1 8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521
C2 df8bcdd097ac93e61ae161ce5f70e4d98a6b472f86c62db4fc9acfd6d95a5c3a
```

C3–C9: not requested, all source geometry/poses/timestamps N/A.

### Observation and actual application poses

World [x m,y m,yaw rad], full saved precision:

```text
A0 = [19.20312073159454, 24.423334915767065, -1.5689742328041627]
A1 = [19.203230030691454, 24.36333501187208, -1.5689749934176171]
A2 = [19.210145291929376, 24.16372504986463, -1.379483950911216]
B0 = [19.20312073159454, 24.423334915767065, -1.5689742328041627]  (stationary bootstrap, not a moving handoff)
B1 = [19.20343640999803, 24.25000186053582, -1.5689755606609315]
P1 = [19.203424271491343, 24.256668516499424, -1.5689755391010196]
B2 = null (never installed or applied)
```

| Simulation-clock event [s] | C0 | C1 | C2 |
|---|---|---|---|
| Observation | 0.7833333741873503 | 1.2833334002643824 | 1.7833334263414145 |
| Ready seen | 1.0166667196899652 | 1.5333334133028984 | 2.0333334393799305 |
| Install | 1.0166667196899652 | 1.5333334133028984 | N/A |
| Actual application | 1.1166667249053717 | 1.5666667483747005 | N/A |

All following host events are **2026-10-07, Korea time UTC+09:00**. Exact monotonic nanoseconds remain in result_summary.json. No host/simulation subtraction.

| Host event (KST) | C0 | C1 | C2 |
|---|---|---|---|
| Observation | 2026-10-07T12:46:11.432109+09:00 | 2026-10-07T12:46:12.081438+09:00 | 2026-10-07T12:46:12.743824+09:00 |
| Request sent | 2026-10-07T12:46:11.520350+09:00 | 2026-10-07T12:46:12.182196+09:00 | 2026-10-07T12:46:12.834770+09:00 |
| Response received | 2026-10-07T12:46:11.734480+09:00 | 2026-10-07T12:46:12.405591+09:00 | 2026-10-07T12:46:13.063013+09:00 |
| Ready seen | 2026-10-07T12:46:11.739055+09:00 | 2026-10-07T12:46:12.417741+09:00 | 2026-10-07T12:46:13.071207+09:00 |
| Install | 2026-10-07T12:46:11.739251+09:00 | 2026-10-07T12:46:12.417977+09:00 | N/A |
| Actual application | 2026-10-07T12:46:11.916903+09:00 | 2026-10-07T12:46:12.529657+09:00 | N/A |
| Client RTT [s] | 0.214140105 | 0.223404715 | 0.228253646 |


C0 inference remained stationary with no active chunk. C0 stayed physically active
through C1 inference (saved inflight states 77–89); C1 stayed active through C2
inference (107–119). C1 observation was the first scheduled capture after C0
application and the exact reveal frame, state75. C2 observation was the first
scheduled capture after B1, state105. There was one reveal and no later reset.

At B1, physical `u_minus = [0.4, -1.2935946487914903e-06]` [m/s,rad/s]. The first
C1 solve used this same previous command. Its returned/applied command is
`[0.4, 0.49999871164531273]`. Official poll had already stored that new command in
controller memory at the B1 cut; this is the unchanged native memory convention.
The bundle retains both pre-solve previous command and B-time accepted memory.
Version1, official generation3, solve `EPISODE_00_solve_000005`, input state90,
application state92. No model row timestamp or correspondence is introduced.

### Every adjacent generated pair

Both available pairs are **EVOLVING**, using the predeclared OR thresholds.
C1→C2 is a prediction-evolution diagnostic: C2 was never executed.

| Metric | C0→C1 | C1→C2 |
|---|---|---|
| Raw hash equal | false | false |
| World hash equal | false | false |
| Symmetric local vertex-to-polyline separation [m] | 0.7506085364710099 | 0.4527470272660435 |
| Endpoint forward/lateral/yaw delta [m,m,rad] | `[-0.3298097848892212,0.67624031094374,0.5215835382214209]` | `[0.2833230495452881,-0.35314008593559265,-0.5015081223100424]` |
| Wrapped net-yaw delta [rad] | 0.20740896068673464 | -0.44719493947923183 |
| Max absolute lateral excursion delta [m] | 0.67624031094374 | -0.35314008593559265 |
| Max absolute yaw excursion delta [deg] | 29.99976963298518 | -14.848641717858252 |
| Observation translation [m] | 0.060000003447498305 | 0.19972971178721619 |
| Observation yaw change [rad] | -7.606134544424492e-07 | 0.18949104250640136 |
| Cart visible pixels | `[0,4073]` | `[4073,5872]` |
| Cart pixel delta | 4073 | 1799 |
| Reference clearance, actual scene [m] | `[1.1175140107938666,0.23473751791079572]` | `[0.23473751791079572,-0.03675014821263675]` |
| Reference clearance delta [m] | -0.8827764928830708 | -0.27148766612343245 |
| Cart-only hypothetical edge clearance delta [m] | 0.3599530115252219 | -0.27148766612343245 |
| World-polyline separation [m] | 0.720669836215717 | 0.4036823666490811 |


C0→C1 actual-scene clearance delta compares OFF to ON; it is not a same-obstacle
comparison. The separate cart-only delta evaluates the cart hypothetically for
C0. The frozen first-response geometry gates all pass: interior world separation
.720669836215717 m, reliable tangent difference29.97231713519825 degrees and
reliable yaw difference29.95388277020058 degrees. First-reaction validity does
not repair the failed final scheduler gate or satisfy the six-chunk length gate.

### Actual safety and call accounting

Minimum **actually executed interval** swept clearance lower bound:
**1.022053073830261 m**. Minimum accepted command-guard lookahead lower bound:
**1.0160703201529018 m**. All120 actual intervals and all120 accepted lookaheads
also pass the legacy .05 m diagnostic. No execution guard abort, command timeout,
controller error, physical footprint overlap, model STOP or second reveal was
observed. C2 reference overlap was not physically integrated.

| Call/event | Count |
|---|---|
| Full Isaac launches | 1 |
| Initialized scientific episodes | 1 |
| Finalized episodes | 1 |
| Startup warmups | 1 |
| Buffer-only requests | 5 |
| Terminal predictions | 3 |
| RGB captures | 8 |
| MPC submissions | 10 |
| MPC solved results | 10 |
| MPC successful results | 10 |
| Physical new command applications | 10 |
| Integration intervals | 120 |
| Cart reveals | 1 |
| Initialization resets | 1 |
| Later resets | 0 |
| Retries | 0 |
| New source searches | 0 |


Graph=0; canonical=0; B_ENTRY=0; Hermite=0; V2=0; GP=0; rigid=0;
correspondence optimization=0. Saved-only validation/reporting model=0, MPC=0,
Isaac=0, optimizer=0. The owned server was identity-checked and stopped.

### Frozen scheduler-gate failure and diagnostic boundary

All request-local RTFs pass: .9901681435452682, .990997336881889,
.9907181720919116. Maximum loop stall .22649273497518152 s passes .25 s.
No burst/catch-up steps occurred; minimum completed wall step .016755727003328502 s
exceeds the resolved physics dt .01666666753590107 s. All MPC submission ticks
are on the six-tick grid. However, `capture_ticks_valid=false` makes the frozen
scheduler gate false and therefore retains TECHNICAL_EXECUTION_BLOCKED.

Saved scheduler records contain120 completed integration loops and one final
abort-only loop at state120. Its `sim_steps=0`, `capture_happened=false` and
`mpc_submitted=false`; it receives C2 SCENE_INVALID. The unchanged collector
checks model rejection before entering the capture branch. The frozen inherited
checker expects `range(0,len(all_loop_records),15)`, which includes120. Actual
captures are `[0,15,30,45,60,75,90,105]`: every scheduled capture of the completed
loops, with no capture after the rejection. This terminal-boundary audit issue
is explicitly reported; the frozen validator and classification were not changed.
The ignored valid prefix is available for inspection but has no fully qualified
long-source claim. Even a later independent resolution of this audit issue would
leave only two applied chunks, below the required six.

[termination_diagnostic.json](../results/long_continuous_obstacle_reveal_source_01/termination_diagnostic.json)
and the separately added saved-only diagnostic script bind the exact terminal
record and runtime/checker hashes. This post-freeze diagnostic adds no scientific
call, changes no gate, and does not reclassify the episode.

The passive shutdown observer recorded one PermissionError while the child was
exiting. Its predeclared handler preserved the error and continued; child return
code0, full episode completion and actual bundled nvJitLink/cusparse mappings are
recorded. This did not trigger another launch. No external source/venv/system
change, scientific retry, result-driven tuning or runtime protocol deviation.

### Bundle, saved-only validation and figures

Bundle: `data/long_continuous_obstacle_reveal_source_01/primary_20261007/source_bundle/`.
Manifest integrity/derived reproducibility passes: **101 source files, 9 derived
files**. `qualification.json`: `valid_long_continuous_source=false`,
`validated_partial_data=true`. The complete source/controller/history/geometry
validator executes successfully; overall scheduler qualification does not pass.

There is exactly **one recoverable actual applied-to-applied handoff C0→C1**, at
`handoffs/C0_to_C1/ready_record.json`, plus its `context.json` and
`actual_old_to_B.npy`. It includes all raw/world paths and hashes, own A anchors,
clock records, physical/controller states and model/freeze provenance. There are
**zero post-reveal applied-to-applied handoffs** and no C1→C2 applied bundle entry.
This is preserved partial data, not a qualified later multi-handoff benchmark.

Frozen saved-only validation was rerun and matched its sealed output. Bundle
hash/count checks, four-PNG input/numeric/hash checks and terminal diagnostic
revalidation pass. All four PNGs were visually inspected: equal axes, actual
segments colored C0/C1, C2 rejection visible, no B2/activation, all three exact
RGBs with separated metadata. The optional fifth figure is absent because fewer
than six chunks applied. No final PNG was added beyond the four required ones.

1. [Continuous world episode](../results/long_continuous_obstacle_reveal_source_01/figures/continuous_world_episode.png)
2. [All own-local chunks](../results/long_continuous_obstacle_reveal_source_01/figures/observation_local_evolution.png)
3. [Timeline and active chunk](../results/long_continuous_obstacle_reveal_source_01/figures/episode_timeline.png)
4. [Exact terminal RGBs](../results/long_continuous_obstacle_reveal_source_01/figures/request_rgb_sequence.png)

## Research interpretation and claim boundary

This single slower episode **did not acquire a useful long source**. C1→C2
observation displacement was .19972971178721619 m, versus 0.38612945283240657 m in
the previous .8 m/s episode (descriptive comparison of different online records).
The intended speed bound was effective, but a subsequent native reference still
physically overlapped and ended the episode. There is no evidence here that the
slower setting reliably yields a longer source. Changing observation poses also
changes model inputs; this is not a controlled causal model-output comparison.

The data establish two actually applied native chunks and one preserved rejected
prediction. They do not establish a six-chunk source, navigation safety, complete
obstacle avoidance, reliable instruction-side compliance, task success, graph or
reconciliation benefit, generalization, or real-world performance. No later graph
or method comparison was implemented. Work stops after result commit/push.

Additional exact saved-only commands:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/diagnose_long_source01_terminal_audit.py --run data/long_continuous_obstacle_reveal_source_01/primary_20261007
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/long-source01-mpl .venv/bin/python scripts/validate_long_continuous_obstacle_reveal_source01.py --run data/long_continuous_obstacle_reveal_source_01/primary_20261007
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/long-source01-mpl .venv/bin/python scripts/report_long_continuous_obstacle_reveal_source01.py --run data/long_continuous_obstacle_reveal_source_01/primary_20261007 --validate
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/diagnose_long_source01_terminal_audit.py --run data/long_continuous_obstacle_reveal_source_01/primary_20261007 --validate
.venv/bin/python -m compileall -q src scripts tests
git diff --check
git diff --cached --check
```
