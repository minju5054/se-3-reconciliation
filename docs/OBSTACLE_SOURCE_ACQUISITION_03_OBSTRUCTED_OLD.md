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
