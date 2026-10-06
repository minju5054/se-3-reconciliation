# CONTINUOUS_OBSTACLE_REVEAL_EPISODE_01

## Frozen protocol, before acquisition

Starting HEAD and fetched origin/main: `6b08712295185d1fb4fa067bc467ba05bdf98719`.
One genuine continuous native episode, `EPISODE_00`, without retry. No graph,
canonical, V2, B_ENTRY, transport, interpolation, smoothing, correspondence,
reference repair, new instruction or source search. Synthetic fixtures are tests,
not experimental evidence. Historical episodes are authenticated, never replayed
as live model inputs and never rerun.

Authority: OSA03 `data/obstacle_source_acquisition_03/primary_20260923T085200Z`,
`REPEAT_00`; historical freeze `76352cf89f461df0c47dd2e7bb4011d665df905d`.
Preparation independently reruns its saved-only validator and hashes 2,350
preserved files. Exact authority paths/hashes are in the ignored `source.json`;
tracked `results/continuous_obstacle_reveal_episode_01/protocol_inputs.json`
binds that manifest and every prepared input. POSE11 is loaded, not reconstructed:
`[19.20312073159454, 24.423334915767065, -1.5689742328041627]`.
Initial physical command is `[0,0]`. Exact Hospital layers, cart triangles and
transform, BRIGHT lighting, camera, checkpoint/config, official MPC and source
provenance are inherited from OSA03 and checked before acquisition.

Instruction, verbatim:

> Go to the far end of the hallway. If the path is blocked, pass the supply cart on your right without touching it. Continue straight after passing it and stop at the end of the hallway.

Historical instruction says RIGHT while local outputs turned +y/LEFT. This
caveat is retained; there is no instruction retuning or new directional gate.
Official LightNav source SHA: `c6f40e3220edbf7011e4f17eaf2c865416737d4d`;
checkpoint revision: `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`;
official MPC SHA256: `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.

### Continuous native execution

The byte-unchanged `robotless_online_handoffs.collect_episode` supplies session,
history, strictly increasing versions, native official MPC, async result handling,
actual first-command application, held commands and exact unicycle integration.
The OSA03 `SuddenReveal`, scene setup, visibility raster and direct guard are
reused. The new wrapper only enforces request eligibility and aborts before
installing an unsafe complete raw reference. It never changes a command or path.
The historical runtime configuration changes only `maximum_handoff_attempts`
from 1 to 3. Existing 4 s active cap, .10 s postroll, 240 s host cap and request/
controller timeout rules remain unchanged.

- C0: first three new scheduled live RGBs are buffer-only; fourth predicts C0,
  stationary at exact POSE11, with renderer and oracle cart OFF.
- Reveal: immediately before the first normal capture strictly after C0's first
  actual command application. Renderer and oracle become ON together and remain
  ON/static. That exact captured JPEG is the only eligible C1 input.
- C2/C3: first normal capture strictly after preceding actual application, with
  no terminal request in flight. One history session, no reset. If that exact
  eligible frame would be buffered or replaced by a queued frame, abort with a
  technical record. Never choose a later observation to rescue the episode.
- At most four terminal predictions, no C4. Stop at C3 application +.10 s or
  STOP/nonfinite/raw unsafe/guard abort/controller or request failure/frozen cap.
  Missing chunks remain N/A; no fabricated continuation.

Cadence remains 60 Hz integration, every6 ticks MPC, every15 ticks capture,
`minimum_wall_step_no_catchup_v1`. No inference pause, catch-up or time rescaling.
As in OSA03, capture defines request eligibility; JPEG encoding and worker
transmission are asynchronous. Exact wire-send host time and runtime send-seen
simulation time are separately recorded; the latter is labeled in the figure.
Ready, install and actual application remain separate events. B is actual first
application, never ready/install. Physical incoming command and controller memory
are reported separately. The controller may already have updated its memory
before a command is physically applied.

World: XY metres, +Z up, yaw CCW radians. Local: +x forward, +y left.
Every world chunk is `T_world_observation * raw_local`, using its own A_k. Raw
arrays stay immutable and separate from derived world arrays; no B re-anchor,
no inserted B row, no waypoint timestamp and intrinsic waypoint dt remains null.

### Safety and qualification

Complete returned raw polylines use the unchanged direct checker with .20 m
circular footprint, .05 m edge clearance and existing workspace/numerical reserve.
No observation/B-to-first-row connector is added. C0 is checked with cart OFF;
C1–C3 with cart ON. Raw unsafe stops before installation, recorded as
`RAW_UNSAFE`; the historical collector receives its supported `SCENE_INVALID`
status. Original worker response and raw arrays remain unchanged. Actual execution
keeps the abort-only preview guard: an unsafe proposed command cannot create B
or enter the executed prefix. Cart-only, Hospital-only and combined checks remain
separate diagnostics.

The C0→C1 gate reuses OSA03 `geometry`, `qualify`, `future_at_pose`, timing audit
and actual boundary/memory extraction. C0 must be finite/nonSTOP, positive,
roughly straight and safe OFF; that same finite raw C0 must violate revealed
cart clearance. No extrapolation. C1 must be finite/nonSTOP, whole-safe ON,
positive hallway progress, arc ≥.60 m, meaningful interior lateral ≥.10 m OR
reliable tangent/pose-yaw difference ≥20° (reliable chord .02 m), and sufficient
future at B (≥4 rows, ≥.60 m arc). Cart pixels ≥20, physical v_minus >.20 m/s,
observation-to-B travel ≥.02 m, safe B, available controller memory, request-local
RTF [.8,1.2], maximum episode loop stall ≤.25 s and valid cadence remain required.
This supports only `OBSTACLE_REVEAL_ASSOCIATED_FIRST_FRESH_CHANGE`.

C2/C3 require new eligible observation, finite/nonSTOP whole-safe raw reference,
safe actual application, valid continuous lineage/history/timing, static ON cart,
and no controller failure. Four request-local RTF windows and the whole-episode
stall/cadence checks are reported. No large later shape change is required.

Later local evolution uses the symmetric maximum of vertex-to-continuous-XY-
polyline nearest distances in both directions; it supports arbitrary unequal N
without row matching. Meaningful if that separation ≥.02 m, endpoint lateral
difference ≥.02 m, or wrapped endpoint/net-yaw difference ≥5°. Hash inequality
alone is insufficient. Also report world separation, A delta, endpoint delta,
max-lateral delta, first reliable local chord tangent delta, cart-clearance and
pixel differences. Any meaningful available pair means
`POST_REVEAL_LOCAL_INTENT_EVOLVING`; otherwise available pairs are STABLE. Missing
pairs stay N/A. Stable intent is not an acquisition failure.

Frozen classification precedence:

1. First gate and all three safe fresh applications, source/timing/guard valid:
   `CONTINUOUS_OBSTACLE_REVEAL_EPISODE_QUALIFIED`.
2. First gate passes but later sequence incomplete/invalid:
   `FIRST_REACTION_ONLY_INSUFFICIENT_SUCCESSIVE_CHUNKS`, with exact cause.
3. Technical failure prevents first-reaction evaluation:
   `TECHNICAL_EXECUTION_BLOCKED`.
4. Otherwise: `CONTINUOUS_CHUNKS_NO_QUALIFIED_FIRST_REACTION`.

Raw unsafe/STOP/guard/cap are scientific truncations, not retries. A technical
request failure after a qualified first reaction retains category 2 and its cause.
Authentication or validator failure blocks a valid source claim.

### Artifacts and verification

Run: `data/continuous_obstacle_reveal_episode_01/primary_20261006`.
Raw model/wire/RGB/visibility/controller/state streams remain ignored. Derived
`source_bundle` indexes raw hashes and contains actual execution segments and
canonical handoff records with A, B, P, u_minus, controller memory, timestamps,
request identity, raw paths and actual OLD-to-B arrays. No future stage is run.

Exactly four PNGs are saved in `results/continuous_obstacle_reveal_episode_01/figures/`:
world episode, observation-local evolution, separate simulation/host timeline,
and exact request RGBs. Numeric sidecars and image hashes are independently checked.
Plotting does not enhance/alter the saved model input JPEGs.

Commands (repository root):

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_episode01.py --mode prepare --run data/continuous_obstacle_reveal_episode_01/primary_20261006
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python -m pytest -q tests/test_continuous_obstacle_reveal_episode01.py tests/test_obstacle_source*.py tests/test_join_online*.py tests/test_online*.py tests/test_robotless_online*.py tests/test_data02*.py tests/test_se2*.py tests/test_join_source02*.py tests/test_gp_se2_environment*.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
.venv/bin/python scripts/run_continuous_obstacle_reveal_episode01.py --mode freeze --run data/continuous_obstacle_reveal_episode_01/primary_20261006
# Review, focused commit, normal push; verify origin/main freeze before start.
.venv/bin/python scripts/run_continuous_obstacle_reveal_episode01.py --mode verify --run data/continuous_obstacle_reveal_episode_01/primary_20261006
.venv/bin/python scripts/run_continuous_obstacle_reveal_episode01.py --mode start --run data/continuous_obstacle_reveal_episode_01/primary_20261006
/home/gpuadmin/isaacsim/python.sh scripts/isaac/continuous_obstacle_reveal_episode01.py --run data/continuous_obstacle_reveal_episode_01/primary_20261006
.venv/bin/python scripts/run_continuous_obstacle_reveal_episode01.py --mode stop --run data/continuous_obstacle_reveal_episode_01/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python scripts/validate_continuous_obstacle_reveal_episode01.py --run data/continuous_obstacle_reveal_episode_01/primary_20261006 --seal
MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python scripts/report_continuous_obstacle_reveal_episode01.py --run data/continuous_obstacle_reveal_episode_01/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python scripts/validate_continuous_obstacle_reveal_episode01.py --run data/continuous_obstacle_reveal_episode_01/primary_20261006
MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python scripts/report_continuous_obstacle_reveal_episode01.py --run data/continuous_obstacle_reveal_episode_01/primary_20261006 --validate
```

## Claim boundary

This is a single saved source episode using logical SE(2) execution in Isaac,
not physical robot dynamics. Even if qualified, it does not establish semantic
causal obstacle recognition, full bypass, navigation-task success, reconciliation
improvement, graph superiority, generalization or real-world performance.
Future multi-chunk reconciliation requires a separate task and scientific freeze.
