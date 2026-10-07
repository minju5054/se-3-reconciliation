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
