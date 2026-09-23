# OBSTACLE-SOURCE-ACQUISITION-03 — exact POSE11 OLD, sudden reveal

Question: does a genuine straight finite OLD generated at POSE11 while cart is
absent become obstructed after reveal, while the first new FRESH is raw-safe,
turning/progressing, and applied at a timing-qualified moving B? Source acquisition
only. No GP/rigid/graph/splice/correspondence/Oracle/lookahead comparison.

Starting fetched HEAD and origin/main both
`84b8d80e295e9f33e6b6737a867c590cf1404cae`. Two unrelated Stage0 edits are preserved.
OSA01/02 results remain immutable: OSA01 paired POSE11/12 passed; POSE11 selected;
OSA02 pacing passed and both new safe FRESH/moving-B records failed solely OLD
obstruction because their finite OLD was observation-anchored .40 m farther back.
No PhaseA rerun, POSE12 fallback, source search or technical pacing trial here.

## Frozen protocol before scientific calls

Run: `data/obstacle_source_acquisition_03/primary_20260923T085200Z/`.
Config: `configs/obstacle_source_acquisition_03.yaml`.
OSA02 source: `data/obstacle_source_acquisition_02/primary_20260923T090000Z/`.
The exact initial pose is loaded from its authoritative `selected_pose11.json`:
`[19.20312073159454, 24.423334915767065, -1.5689742328041627]`.
World Isaac Z-up, x/y metres, yaw radians CCW; local +x forward, +y left.
No translation of OLD/FRESH arrays. Every chunk is anchored at its own actual
observation pose. N is arbitrary; rows have no intrinsic timestamps.

Cart is OFF before and throughout target OLD observation/inference. The logical
agent is initialized stationary at this exact pose, with zero physical motion
until OLD's first command application. Existing native bootstrap semantics:
three newly captured chronological live RGB buffer requests, then a fourth live
frame with genuine terminal OLD prediction (`chunk_000`). Legacy `bootstrap.json`
records that target OLD's actual application; the preceding stationary bootstrap
is not moving OLD execution. No historical JPEG/model-response injection.

Exact instruction for both chunks/repetitions:

> Go to the far end of the hallway. If the path is blocked, pass the supply cart on your right without touching it. Continue straight after passing it and stop at the end of the hallway.

Same Hospital, natural cart mesh and exact world transform, camera, BRIGHT profile,
checkpoint, VLN task, greedy generation, official MPC and abort-only guard as
OSA02. RIGHT wording versus historical local-LEFT response remains a caveat.
No side compliance is presumed. No evaluation threshold changes.

`t_old_active` is actual first target-OLD command application, not ready/install.
Reveal is the first normal scheduled 4 Hz capture **strictly after** that actual
application. Render visibility and oracle occupancy switch ON before its render.
That exact first post-reveal JPEG alone may request FRESH (`chunk_001`). Queued
pre-reveal frames stay buffer-only; no later FRESH replacement. OLD continues
normally while FRESH is pending. First physically applied FRESH command defines B.
Record physical incoming command separately from completed controller memory,
first FRESH solve input/output and application. Short postroll .10 s only,
maximum4 s active, same hold timeout/failure policy.

**Necessary request-eligibility difference:** OSA02's inherited
`minimum_active_before_prediction_sim_s=.5` would consume the prescribed first
post-active frame as buffer-only. OSA03 explicitly sets this to **0** before any
scientific output to implement the user's new request/reveal timing. This is
not a pacing change or a hidden wait. Resolved config differs from OSA02 PhaseB
only at this field; initial pose/reveal rule reside in episode/protocol records.
Stationary live bootstrap is not asserted identical to OSA01 moving H8 input.

Existing `minimum_wall_step_no_catchup_v1` scheduler and collector source are
byte-unchanged. 60 Hz exact held-command integration, MPC every6 ticks, capture
every15 ticks remain on the simulation clock. No catch-up, artificial inference
pause, timestamp rescaling or whole-episode RTF optimization. Request-local RTF
[.8,1.2] and max episode stall<=.25s remain required; whole RTF is supplemental.

Exactly independent **REPEAT_00, REPEAT_01**, both run regardless of first result.
At most4 new terminal predictions (one OLD+one FRESH per repetition), no retry or
third repetition. Separate official server startup warmup is counted. No new
technical qualification, PhaseA or offline counterfactual solves.

## Gates and deterministic outcome

Reuse OSA01/02 source geometry/mismatch/future helpers and original direct Hospital
plus cart mesh checker. Radius .20 m, required footprint-edge margin .05 m,
original geometry uncertainty/workspace/swept assumptions. No observer→first-row
connector, prefix trimming, finite OLD extrapolation or final-segment extension.

- OLD OFF: nonSTOP, positive progress, whole raw safe/workspace valid, roughly
  straight (lateral<.10m, yaw/reliable tangent<20deg).
- Gate B `B_OBSTACLE_RELEVANT`: the **same actual finite OLD** with cart must fail
  existing clearance, and its nominal minimum must be<.05m. B here names gate B,
  not application boundary B.
- FRESH: entire original raw safe, nonSTOP, arc>=.60m, positive hallway progress,
  endpoint reaches existing front-.50m interaction region; no mere truncation.
- Meaningful change: existing interior lateral>=.10m OR reliable tangent/pose-yaw
  >=20deg, reliable chord>=.02m. Endpoint projection gaps are excluded.
- Moving B: physical v_minus>.20m/s, obs→B arc>=.02m, B clearance-valid.
- Future at B: >=4 relevant original rows and >=.60m remaining arc; eligibility
  projection does not modify reference or implement correspondence optimization.
- Exact initial/OLD observation pose, stationary prior to OLD, cart OFF until
  OLD application, first normal strictly-post-active reveal, first post-reveal
  FRESH, actual OLD execution during request, controller-memory provenance,
  instance pixels>=20, unchanged timing and no-burst/cadence checks all pass.

Guard abort is retained separately and never repairs raw output. No collision is
intentionally integrated. Complete bypass is a descriptor, not a gate. First
qualifying repeat is representative (00 before01). Any qualifying repeat gives
`QUALIFIED_GENUINE_OBSTRUCTED_OLD_HANDOFF_SOURCE`; neither qualifies gives
`POSE11_OBSTRUCTED_OLD_SOURCE_NOT_QUALIFIED`. A genuine failure preventing valid
scientific output gives `TECHNICAL_EXECUTION_BLOCKED`. Missing fields are N/A.
No new positive interpretation/category after seeing results.

## Preservation, validation and commands

Preparation independently revalidates OSA02 saved records, hashes previous raw,
external/checkpoint/core sources and unrelated edits, and copies only small
immutable config/scenario/protocol descriptions. New live data goes exclusively
to this run. Freeze binds inherited OSA02 source hashes plus new wrapper/tests,
inputs, two repetitions and at-most4 terminal requests. Commit and normal push
must precede server/scientific calls.

Saved validator reuses original episode/wire/history/anchor/state checker,
recomputes geometry/guard/dynamic cart/memory/timing and verifies the exact new
OLD anchor and first scheduled reveal rule. Qualified bundles retain raw outputs,
RGB/history/wire references, world arrays, actual execution, command/memory,
all clock domains and environment/model/MPC provenance. No failed record becomes
a qualified candidate. Validators make zero new model/MPC/optimizer calls.

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_obstacle_source_acquisition03.py --mode prepare --run data/obstacle_source_acquisition_03/primary_20260923T085200Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q tests/test_obstacle_source03.py tests/test_obstacle_source02_report.py tests/test_obstacle_source_online.py tests/test_obstacle_source_pacing.py tests/test_join_*.py tests/test_gp_se2_join01*.py tests/test_online_*.py tests/test_robotless_online*.py tests/test_se2.py tests/test_obstacle_source_acquisition.py tests/test_handoff_delay_attribution.py
.venv/bin/python scripts/run_obstacle_source_acquisition03.py --mode freeze --run data/obstacle_source_acquisition_03/primary_20260923T085200Z
# Review diff, commit and normal push BEFORE start/collection.
.venv/bin/python scripts/run_obstacle_source_acquisition03.py --mode start --run data/obstacle_source_acquisition_03/primary_20260923T085200Z
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 /home/gpuadmin/isaacsim/python.sh scripts/isaac/obstacle_source03_online.py --run data/obstacle_source_acquisition_03/primary_20260923T085200Z
.venv/bin/python scripts/validate_obstacle_source_acquisition03.py --run data/obstacle_source_acquisition_03/primary_20260923T085200Z
.venv/bin/python scripts/run_obstacle_source_acquisition03.py --mode stop --run data/obstacle_source_acquisition_03/primary_20260923T085200Z
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

All scientific outcomes will be appended after the frozen two repetitions.
No reconciliation/complete traversal/general navigation claim is authorized.

Pre-scientific full relevant regression: 532 passed (6.88s), compileall/diff PASS,
zero real model/MPC calls. OSA02 saved revalidation and inherited source parity PASS.

## Completed frozen acquisition

**QUALIFIED_GENUINE_OBSTRUCTED_OLD_HANDOFF_SOURCE: 2/2 qualified; REPEAT_00 is the representative, REPEAT_01 the replication.** Both fixed repetitions ran once. No PhaseA rerun, pacing trial, retry, additional FRESH or next-stage comparison.

Scientific code/config/tests freeze was committed and pushed as `76352cf89f461df0c47dd2e7bb4011d665df905d` before server startup and acquisition. Result numbers are also tracked in [the small JSON summary](results/obstacle_source_acquisition_03.json). Original OSA01/02 conclusions are unchanged.

### Repository-confirmed facts

All distances below are metres, clearance is footprint-edge clearance (radius .20 m, unchanged required margin .05 m), angular values are degrees unless marked rad/s. OLD obstruction is a hypothetical check of the actual finite returned OLD with the newly revealed cart; it is **not an executed collision**. Both whole raw polylines were checked, with no extrapolation, deleted rows or observer-to-first-waypoint connector.

| Measured quantity | REPEAT_00 | REPEAT_01 |
|---|---:|---:|
| OLD raw arc [m] | 1.351820588 | 1.351820588 |
| OLD OFF minimum edge [m] | 1.117514011 | 1.117514011 |
| Same finite OLD + cart minimum edge [m] | -0.125215494 | -0.125215494 |
| FRESH whole raw minimum edge [m] | 0.229486890 | 0.221686301 |
| FRESH raw arc [m] | 1.353245520 | 1.353245520 |
| FRESH maximum local +y/LEFT displacement [m] | 0.676244259 | 0.676244259 |
| FRESH maximum absolute yaw [deg] | 30.000670193 | 30.000670193 |
| FRESH hallway progress [m] | 1.172444916 | 1.172445022 |
| OLD→FRESH interior lateral mismatch [m] | 0.717343090 | 0.712353270 |
| Reliable tangent mismatch [deg] | 29.972302282 | 29.972293322 |
| Reliable pose-yaw mismatch [deg] | 29.954514014 | 29.955474216 |
| Observation→B actual travel [m] | 0.213333346 | 0.213333346 |
| B minimum edge [m] | 1.095851731 | 1.085852164 |
| Actual acquisition path clearance lower bound [m] | 0.949676819 | 0.939678459 |
| FRESH request-local RTF | 0.991909419 | 0.991224685 |
| Maximum loop stall [s] | 0.194015202 | 0.151906103 |
| FRESH client RTT [s, host] | 0.227037605 | 0.224975173 |
| Whole-episode RTF (supplemental) | 0.712710743 | 0.733695738 |
| OLD obstruction | PASS | PASS |
| FRESH STOP | false | false |
| Remaining original rows / arc [m] | 9 / 1.202489115 | 9 / 1.202489115 |
| Cart visible pixels | 3992 | 4220 |
| Safety abort / catch-up bursts | none / 0 | none / 0 |
| All frozen source gates | PASS | PASS |
| Complete bypass | false | false |

Both OLD local-array hashes are identical, and both FRESH local-array hashes are identical; neither equality was enforced. OLD versus FRESH hashes differ. The independent new live sessions have different RGB/observation poses. FRESH world clearance consequently differs despite identical local arrays. Each actual output has N=10; analysis remains generic N, and row indices have no intrinsic time.

```text
OLD raw npy SHA256: 6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae
FRESH raw npy SHA256: 8cd364cd4acb96d9af85e2e2b97a64f11ab54d0ab4e6d2177da26982af9af521
```

**Direction caveat:** the frozen instruction says RIGHT, while the returned local trajectories use +y/LEFT. Neither side compliance nor full instruction completion is claimed. Direction was not retuned or introduced as a new rejection criterion. FRESH endpoints remain .311381/.301381 m before the cart front plane, within the frozen .50 m interaction region; this is a safe turning local future, not a completed bypass.

### Exact boundaries and controller state

Coordinates are Isaac world `[x m, y m, yaw rad]`. Each FRESH remains anchored at its actual observation pose, never at B. Command tuples are `[v m/s, omega rad/s]`.

```text
REPEAT_00
OLD observation = [19.20312073159454, 24.423334915767065, -1.5689742328041627]
FRESH observation = [19.203242163778583, 24.356668352984542, -1.5689752526514715]
B = [19.203630553052257, 24.14333536015282, -1.5689760264727979]
physical u_minus = [0.8, -1.5584691584098122e-06]
controller previous_control at B = [0.8, 0.49999844893907003]
```

```text
REPEAT_01
OLD observation = [19.20312073159454, 24.423334915767065, -1.5689742328041627]
FRESH observation = [19.203260368199164, 24.346668368545373, -1.5689754090345875]
B = [19.20364874270168, 24.133335375688848, -1.5689760469373495]
physical u_minus = [0.8, -1.2790285101476235e-06]
controller previous_control at B = [0.8, 0.4999987283797167]
```

At first FRESH application the worker has already accepted the first FRESH solve result, so its memory is approximately `[.8, .5]` while the incoming physical OLD command is `[.8, 0]`. This is the actual asynchronous state, not a reset, correction or assertion that the two are equal. Solve inputs/outputs, accepted-result events and applied identities remain in each episode and the bundle provenance. A future common-B comparison must account for this phase distinction; no such comparison ran here.

### Timing, live history and execution

Three newly captured stationary live frames were buffered before the fourth capture generated genuine OLD. There was no preceding moving chunk. OLD observation was exactly POSE11 in both sessions, with cart OFF throughout inference and zero motion until OLD application. Reveal was the first normal scheduled capture strictly after actual OLD application; the exact resulting JPEG generated the first and only FRESH. FRESH history count was 6 in each session; the full episode captured 7 frames. No historical RGB or model response was replayed.

| Simulation-clock event [s] | REPEAT_00 | REPEAT_01 |
|---|---:|---:|
| OLD observation | 0.783333374 | 0.783333374 |
| OLD first actual application | 1.116666725 | 1.066666722 |
| Reveal / first FRESH observation | 1.283333400 | 1.283333400 |
| Reveal minus OLD application | 0.166666675 | 0.216666678 |
| FRESH ready seen | 1.533333413 | 1.533333413 |
| FRESH install | 1.533333413 | 1.533333413 |
| First FRESH application / B | 1.566666748 | 1.566666748 |
| FRESH observation→B duration | 0.283333348 | 0.283333348 |

| Host-monotonic event [s] | REPEAT_00 | REPEAT_01 |
|---|---:|---:|
| FRESH observation | 80643.001066609 | 80645.605106347 |
| Request sent | 80643.097832803 | 80645.698869603 |
| Full response receipt | 80643.324870408 | 80645.923844776 |
| Ready seen | 80643.331957551 | 80645.933387692 |
| Install | 80643.332427079 | 80645.933752279 |
| First command application | 80643.452123090 | 80646.047255055 |

Host clocks are not subtracted from simulation clocks. Observation→application is .451056481/.442148708 s on the host clock and .283333348 s on the simulation clock. The request-local RTF criterion passes independently (.991909/.991225); whole-episode RTF is .712711/.733696 because blocking rendering is not repaid by catch-up. No claim of whole-episode real-time throughput. Exact held-command reconstruction has zero stored-pose error in both 99-state / 98-integration records. Capture every15 simulation ticks, MPC every6, 60 Hz integration and the unchanged `minimum_wall_step_no_catchup_v1` scheduler pass.

FRESH was actually applied and only the frozen .10 s postroll was recorded. Both normal terminations are `ATTEMPT_LIMIT`; neither guard aborted. The stopped source episodes do not demonstrate that continuing native tracking would clear the cart.

### Source and compute preservation

Pinned official LightNav source `c6f40e3220edbf7011e4f17eaf2c865416737d4d`, checkpoint revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, task VLN, actual greedy settings temperature=0/top_p=1/top_k=0/traj_top1=0. Official MPC SHA256 `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`. Historical Hospital/cart/camera/BRIGHT provenance is referenced in `source.json`, `acquisition_scene.json`, `workers.json`, and each bundle. 2,123 preserved input/core/external/historical/unrelated-edit hashes revalidated unchanged.

Exactly **4 scientific terminal predictions** (OLD+FRESH per repetition), **10 buffer-only requests**, **1 separately counted synthetic server startup warmup**, and **14 official MPC submissions / 14 saved solves**. MPC summed solve wall time .106523391 s; scientific terminal RTT sum .883671581 s; collector wall 14.792618380 s excludes Isaac startup/server loading. New PhaseA, technical qualification, GP, rigid, graph, splice, reconciliation and test/validator real model/MPC calls are all **0**. The owned server was stopped.

### Validation, figures and sealed source

Independent saved-record recomputation passes every frozen gate, source/input hashes, exact poses/anchors, session/history/wire integrity, reveal/application order, finite OLD obstruction, whole raw FRESH geometry, actual command integration, B/memory, timing, guard and remaining future. The separate report validation also passes bundle raw-byte parity and CSV/JSON/plot/ZIP parity, without new inference or controller calls.

- [Static review](../data/obstacle_source_acquisition_03/primary_20260923T085200Z/index.html)
- [Representative geometry](../data/obstacle_source_acquisition_03/primary_20260923T085200Z/review/REPEAT_00_world.png)
- [Live OFF/ON RGB](../data/obstacle_source_acquisition_03/primary_20260923T085200Z/review/REPEAT_00_RGB.png)
- [Timing](../data/obstacle_source_acquisition_03/primary_20260923T085200Z/review/REPEAT_00_timing.png)
- [Both repetitions/gates](../data/obstacle_source_acquisition_03/primary_20260923T085200Z/review/qualification_matrix.png)
- [Review ZIP](../data/obstacle_source_acquisition_03/primary_20260923T085200Z/review_bundle.zip)
- [Sealed source manifest](../data/obstacle_source_acquisition_03/primary_20260923T085200Z/source_bundle/manifest.json)
- [Source validation](../data/obstacle_source_acquisition_03/primary_20260923T085200Z/validation.json) / [report validation](../data/obstacle_source_acquisition_03/primary_20260923T085200Z/validation_final.json)

Each repetition has raw OLD/FRESH npy and responses, observation JPEGs, immutable history/request references, observation-anchored world arrays, boundary execution/state/timing, and environment/model provenance with hashes. Seven figures have numeric/hash sidecars. PNG labels/legends were visually inspected. Large/raw/generated artifacts stay local and ignored.

Additional saved-only commands (no scientific rerun):

```bash
MPLCONFIGDIR=/tmp/osa03_mpl OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/report_obstacle_source_acquisition03.py --run data/obstacle_source_acquisition_03/primary_20260923T085200Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/report_obstacle_source_acquisition03.py --run data/obstacle_source_acquisition_03/primary_20260923T085200Z --validate --output data/obstacle_source_acquisition_03/primary_20260923T085200Z/validation_final.json
# After initial output creation, validate read-only by omitting --output.
```

### Research interpretation

The question is answered positively for this frozen scenario in both attempts: the actual straight finite OLD becomes obstructed by sudden cart reveal, the first new raw FRESH is safe/turning/progressing, and it becomes active at a timing-qualified moving B while leaving enough future. This is a **genuine sudden-obstacle local-avoidance moving-B handoff**. REPEAT_00 is selected by the predeclared first-qualified rule, not by visual appearance or best margin.

### Not demonstrated / remaining uncertainty

No complete bypass, end-to-end navigation, broad LightNav obstacle competence, real-robot feasibility, reconciliation benefit or optimizer superiority was established. The principal remaining uncertainty is whether a transition starting at this actual B can execute into the safe original FRESH while preserving safety/motion and reducing handoff cost. That requires a separate common-B experiment; none was implemented or run.

Final relevant regression: **532 passed (7.02 s)**; zero real model/MPC test calls.
Final `compileall -q src scripts tests` and `git diff --check` PASS. No shell
launcher changed. Unrelated Stage0 edits are preserved and excluded from staging.
