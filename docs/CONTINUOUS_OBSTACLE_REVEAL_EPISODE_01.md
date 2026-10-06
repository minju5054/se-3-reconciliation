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

## Repository-confirmed outcome

**TECHNICAL_EXECUTION_BLOCKED. No continuous source episode was acquired.**

Scientific freeze `2334b8e528b77395e24bde956f31cc0ee5cb474b` was committed,
normally pushed, and confirmed by `git ls-remote origin refs/heads/main` before
server startup or the acquisition attempt. The freeze binds 530 code/config/test
files, nine prepared artifacts, all declared checkpoint files, the official MPC
file and 2,350 preserved historical/input hashes. Exact hashes are in the tracked
freeze/input manifests and the ignored authenticated source chain.

The official server started, passed its environment/checkpoint/argv audit and
completed one startup model warmup. The one authorized Isaac launch then failed
inside `SimulationApp`, before `setup_scene`, worker construction, episode
initialization or the first capture. Its error at `2026-10-06T08:56:16Z` was:

```text
Import error: .../omni.isaac.ml_archive/pip_prebundle/torch/lib/../../nvidia/cusparse/lib/libcusparse.so.12:
undefined symbol: __nvJitLinkCreate_12_8, version libnvJitLink.so.12
```

This is a library-loading failure; no package installation or external environment
change was attempted. The launch command returned exit code 0 despite that error.
Absence of `episodes/`, `workers.json`, `acquisition_scene.json`, any activation
and any episode-completion record establishes that no acquisition occurred.
The prelaunch `execution_start.json` field `scientific_episodes=1` records the
intended attempt budget, **not an acquired episode**. It remains unmodified.
The owned LightNav server was stopped and its exit observed at
`2026-10-06T08:57:07.545028Z`. Its inherited shutdown-reason text says collection
complete; this does not imply successful acquisition. No retry was made.

### C0–C3 availability

| Chunk | Generated | Actually applied | Observation / ready / application times | A | B | Cart pixels | Raw local SHA256 | Raw/world safety |
|---|---|---|---|---|---|---|---|---|
| C0 | No | No | N/A | N/A | N/A | N/A | N/A | N/A |
| C1 | No | No | N/A | N/A | N/A | N/A | N/A | N/A |
| C2 | No | No | N/A | N/A | N/A | N/A | N/A | N/A |
| C3 | No | No | N/A | N/A | N/A | N/A | N/A | N/A |

POSE11 is an authenticated **intended initial pose**, not a measured A0 in this
attempt. The intended scene/cart/camera/BRIGHT/instruction sources were bound;
their live installation was not reached. C0→C1 reaction gates, C1→C2 and C2→C3
evolution descriptors, and actual swept clearance are all N/A. Neither EVOLVING
nor STABLE is assigned. STOP, raw-unsafe, guard-abort and controller-timeout
stages were not reached; no such event was observed.

### Call accounting

| Item | Count |
|---|---:|
| Authorized acquisition launch / SimulationApp startup attempt | 1 |
| Initialized scientific episodes / completed scientific episodes | 0 / 0 |
| Terminal LightNav predictions / buffer-only requests | 0 / 0 |
| Startup model warmup | 1 |
| Live RGB captures | 0 |
| MPC submissions / solved results / physical applications | 0 / 0 / 0 |
| Integration intervals / cart reveals | 0 / 0 |
| Episode resets, including after initialization | 0 |
| Graph / canonical / B_ENTRY / reconciliation | 0 / 0 / 0 / 0 |
| Retries / new source searches | 0 / 0 |
| Saved-only validation/report scientific calls | 0 |

### Validation and output status

- Required regression suite: **538 passed, 1 skipped**. Skip: unavailable
  immutable generated DATA02 v1 corpus. Final focused suite: **30 passed**;
  these are included in the 538 distinct tests, not additional evidence.
- Initial sandbox regression had two Unix-socket permission failures; the same
  full suite passed outside the sandbox. No test was weakened or disabled.
- Compileall and diff checks pass. Existing OSA03 saved validation passes.
  Checkpoint, official MPC, frozen implementation and input hashes still match.
- The pre-science Isaac CLI-import check passed without constructing
  SimulationApp. It did not test this later CUDA extension-loading stage.
- The frozen saved-episode validator was executed once and **failed**, because
  the episode directory does not exist. It did not validate an episode or create
  `validation.json`. Its original failure log is preserved. No frozen validator,
  runtime, gate, configuration or method was changed after the freeze.
- A separate saved-only **technical evidence audit** passes: source/log hashes,
  zero-call accounting, missing episode records, explicit null fields, four PNG
  hashes and their numeric sidecar. This is not a scientific episode validation.

`results/continuous_obstacle_reveal_episode_01/result_summary.json` records these
facts. `chunks.csv` contains four unavailable rows. The source-bundle path is
`data/continuous_obstacle_reveal_episode_01/primary_20261006/source_bundle/`;
its manifest explicitly says `valid_scientific_source=false`, with no chunks,
handoffs or execution segments. It must not be used as a source for later graphs.

Four PNGs were generated and visually inspected. These are **technical-status
figures**, since no scientific geometry or RGB exists:

1. [Continuous world episode — unavailable](../results/continuous_obstacle_reveal_episode_01/figures/continuous_world_episode.png)
2. [Observation-local evolution — unavailable](../results/continuous_obstacle_reveal_episode_01/figures/observation_local_evolution.png)
3. [Host startup lifecycle; simulation timeline unavailable](../results/continuous_obstacle_reveal_episode_01/figures/episode_timeline.png)
4. [C0–C3 request RGB — unavailable](../results/continuous_obstacle_reveal_episode_01/figures/request_rgb_sequence.png)

### Executed commands and deviations

The pre-science commands above were run through freeze, verification and server
start. The actual single launch command was:

```bash
/home/gpuadmin/isaacsim/python.sh scripts/isaac/continuous_obstacle_reveal_episode01.py --run data/continuous_obstacle_reveal_episode_01/primary_20261006 > data/continuous_obstacle_reveal_episode_01/primary_20261006/logs/isaac.log 2>&1
.venv/bin/python scripts/run_continuous_obstacle_reveal_episode01.py --mode stop --run data/continuous_obstacle_reveal_episode_01/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python scripts/validate_continuous_obstacle_reveal_episode01.py --run data/continuous_obstacle_reveal_episode_01/primary_20261006 --seal > data/continuous_obstacle_reveal_episode_01/primary_20261006/logs/saved_validator.log 2>&1
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python data/continuous_obstacle_reveal_episode_01/primary_20261006/reporting/blocked_status.py
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python data/continuous_obstacle_reveal_episode_01/primary_20261006/reporting/validate_blocked_status.py
```

The planned ordinary saved-episode report could not run without an episode.
The two archived reporting-only commands above instead document the technical
failure and explicit N/A panels. Their paths/hashes are bound in tracked
sidecars. They make no scientific calls and do not modify frozen implementation,
configuration or historical data. This reporting substitution, the unavailable
episode, and the frozen validator's missing-directory failure are the protocol
deviations. There was no retry, retuning, environment repair or future-stage work.

## Research interpretation

There is no new evidence about continuous obstacle response or post-reveal local
intent evolution. The intended C0→C1→C2→C3 source remains unacquired. Only the
implementation, pre-science authentication/tests and failed startup attempt are
established. No bypass, semantic recognition, graph/reconciliation improvement,
navigation success or generalization claim follows.

## 2026-10-06 startup diagnosis addendum

Starting HEAD and fetched origin/main for this diagnosis:
`a8409a0013d4fab91b9384ac6511bdaa9b3512f3`.
The repeated attachment contains the same experiment and `retry = 0` constraint.
No second acquisition was launched. The scientific freeze remains
`2334b8e528b77395e24bde956f31cc0ee5cb474b`; the failed result and its raw logs
remain preserved.

### Confirmed cause and correction to the earlier account

The assistant omitted the environment-cleaning prefix documented in
[the OSA03 launch command](OBSTACLE_SOURCE_ACQUISITION_03_OBSTRUCTED_OLD.md).
The inherited `LD_LIBRARY_PATH` placed `/usr/local/cuda/lib64` before Isaac's
bundled libraries. This selected CUDA 12.6 `libnvJitLink.so.12.6.20`, which lacks
the `12_8` symbols required by Isaac's bundled `libcusparse.so.12`. This launch
omission is an additional protocol deviation. There is no evidence here that a
PyTorch package needs installation.

Read-only loader relocation checks on the same `libcusparse.so.12` establish:

| Process environment | Selected nvJitLink | Undefined symbols |
|---|---|---:|
| Inherited `LD_LIBRARY_PATH` | System CUDA 12.6 | 8 |
| `LD_LIBRARY_PATH` removed | Isaac bundled nvJitLink | 0 |

Both `ldd -r` commands returned exit code 0; their text output, rather than the
exit code alone, establishes the difference. The exact commands, library paths,
SHA256 hashes, output hashes and historical command are recorded in
[startup_loader_diagnosis.json](../results/continuous_obstacle_reveal_episode_01/startup_loader_diagnosis.json).
The two diagnostic commands were:

```bash
ldd -r /home/gpuadmin/isaacsim/extsDeprecated/omni.isaac.ml_archive/pip_prebundle/nvidia/cusparse/lib/libcusparse.so.12
env -u LD_LIBRARY_PATH ldd -r /home/gpuadmin/isaacsim/extsDeprecated/omni.isaac.ml_archive/pip_prebundle/nvidia/cusparse/lib/libcusparse.so.12
```

Removing the variable applies only to the diagnostic child process. No external
file, system Python, package or virtual environment was changed. This check
resolves the observed relocation failure; it does **not** validate SimulationApp
startup, live rendering, timing qualification or acquisition.

### Execution boundary and proposed recovery

Additional LightNav predictions, warmups, RGB captures, SimulationApp starts,
MPC solves and research optimizer calls during this diagnosis are all **0**.
The classification remains **TECHNICAL_EXECUTION_BLOCKED**, with no valid new
source episode. The four existing PNGs continue to show unavailable scientific
data; they have not been replaced with inferred geometry or images.

A recovery attempt would preserve this failed run, prepare a separately named
run with separately preserved manifests, authenticate the same OSA03 inputs,
and commit/push a new freeze before starting. Its launch would use the exact
historical OSA03 environment cleanup, as recorded in the diagnosis JSON's
`proposed_launch_template`; no installation or scientific-method change is
proposed. No such recovery has been executed. The attachment's section 5 says
`retry = 0`, and section 19 says `Do not retry`; an explicit user exception is
required before an additional acquisition attempt.

### Diagnosis validation

Focused tests: **30 passed**. Frozen implementation/input/external hashes,
compileall and diff checks pass. A saved-only audit verifies the recorded
library/log/source hashes, the 8-versus-0 symbol counts, and the unchanged prior
technical result, CSV and four PNGs. Its archived path/hash is in the diagnosis
JSON. This is not a scientific episode validation.

Reexecuting the prior technical evidence script reached its exclusive-create
guard because `chunks.csv` already exists. No output was overwritten. The new
audit executes that script's exact read-only assertions, then checks the existing
outputs; it does not execute its output-writing block. An initial ad hoc loader
hash check captured the trailing comma after each symbol name; the final parser
checks symbol names without this punctuation. The raw loader logs are unchanged.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python -m pytest -q tests/test_continuous_obstacle_reveal_episode01.py
.venv/bin/python scripts/run_continuous_obstacle_reveal_episode01.py --mode verify --run data/continuous_obstacle_reveal_episode_01/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/continuous-reveal-mpl .venv/bin/python data/continuous_obstacle_reveal_episode_01/primary_20261006/reporting/loader_diagnosis/validate_saved.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```
