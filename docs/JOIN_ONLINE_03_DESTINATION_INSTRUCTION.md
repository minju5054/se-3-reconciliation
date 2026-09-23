# JOIN-ONLINE-03 — Destination-first instruction rerun

## Frozen question and single intervention

User instruction, used verbatim for every prediction in every new episode:

> Go to the far end of the hallway. Pass around the supply cart without touching it, and stop only when you reach the end of the hallway.

Does this destination-first wording change the shortening/STOP behavior observed
under "Avoid the supply cart and continue to the end of the hallway."?
This is a new native online source experiment, not reconciliation or a new
avoidance/controller policy. Historical JOIN-ONLINE-02 is immutable.

Starting HEAD and freshly fetched origin/main:
`68ebd7ec32428ede040dcd715f0925a6de2fd9de`.
Baseline: `data/robotless_join_online_02/primary_20260923T001500Z/`.
New run: `data/robotless_join_online_03/primary_20260923T031300Z/`.
Actual UTC, monotonic, and simulation times are recorded separately from run ID.

## Preserved setup

Reuse the baseline's exact resolved config; change only its instruction and
experiment label. The inherited `join_online02` configuration key is retained
for the unchanged collector. New wrapper never changes legacy defaults.

- Exact selected 5 m start: [19.19599771672821,27.640298097840173,-1.5685636348381817].
- Exact Hospital layers, cart mesh/transform, BRIGHT light, camera, footprint,
  required edge clearance .05 m, direct environment and curve allowances.
- Cart absent throughout OFF; present before the first capture throughout ON.
- Official LightNav c6f40e3220edbf7011e4f17eaf2c865416737d4d; checkpoint revision
  7221d418bfff55cfcbadd09f7a26aaab81e1f8a6; VLN. Actual argv/config/hashes verified.
- temperature0, top_p1, top_k0, traj_top1=0, unchanged history sampler.
- Unchanged official MPC source SHA256
  2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1.
- Native nearest/+1 selector, gains, limits, objective, 10 Hz solves,
  60 Hz exact held-command SE(2) execution, 4 Hz live RGB.
- Four bootstrap frames (first three buffer-only); later predictions require
  .5 simulation seconds after actual activation of the preceding reference.
- OLD continues while FRESH is pending. World poses remain observation anchored;
  readiness, installation, first command application/B, physical previous
  command and controller memory remain separately recorded.
- Original oracle abort guard only: no steering, reference editing or unsafe
  execution. Unsafe raw prediction alone does not trigger an abort.

Frozen order, once each: OFF_REPEAT_00, ON_REPEAT_00, OFF_REPEAT_01, ON_REPEAT_01.
Retain the original 20-activation/attempt and 25 s bounds and natural STOP
termination. All four planned episodes run regardless of earlier scientific
outcome. No prompt/placement/threshold tuning or outcome retry.

The exact former `scenario.json`, `start_selection.json`, and `side_passages.json`
bytes are copied; no start search is repeated. Previous raw RGB/responses are
not reused as new model inputs. Every new episode captures new current RGB and
has an independent session. Therefore this is an instruction intervention on
the same declared scenario, **not an identical-history paused inference pair**.

## Frozen evaluation and interpretation

Reuse full raw-polyline footprint clearance, without suffix trimming or an
observation-to-first-row connector; generic N. Keep the previous meaningful
lateral/tangent criteria, SAFE_BYPASS_ONSET and full SAFE_BYPASS conditions.
Physical cart traversal, safe raw future, STOP and hallway goal completion
remain distinct. There is no oracle hallway-end target, so model STOP alone
does not prove arrival at the instruction's destination.

Report every response, actual execution, boundary/timing/clearance, original
classifications, and actual model/MPC calls. Compare both ON repetitions with
historical ON and new OFF controls. Preserve the original request-local RTF
[.8,1.2] and .25 s stall checks; do not repair pacing after seeing outcomes.
If paths shorten or STOP again, this does not establish a hidden neural cause.
If bypass appears, it is evidence on this scenario only. No GP/rigid solve.

The read-only STOP/pointing auditor can inspect saved new outcomes afterward;
OPOS/cart overlap remains a grounding hypothesis, not proof of goal identity.
No model justification is fabricated or elicited as a substitute for evidence.

## Implementation and commands

Only a new config, preparation/report/validation wrapper and tests are added.
Unchanged `join_online02_collect.py` performs real preflight and collection;
unchanged online history, transform, solver and environment functions execute.
The new validator uses the original full stream validator and independently
reconstructs every guard, mask, config comparison and plotted series.
The original plot generator is reused literally; `review/index.html` is its
legacy-labelled helper output. The primary correctly labelled new index is
`index.html`, with `instruction_review_bundle.zip`.

```bash
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode init
# Same clean Isaac environment as JOIN-ONLINE-02, no system changes:
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 /home/gpuadmin/isaacsim/python.sh scripts/isaac/join_online02_collect.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode preflight
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode freeze
# Review, commit, normal push before model startup and actual collection.
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode start
# Repeat the exact clean Isaac command above with --mode collect.
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode stop
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode analyze
MPLCONFIGDIR=/tmp/join_online03_mpl .venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode report
.venv/bin/python scripts/run_join_online03.py --run data/robotless_join_online_03/primary_20260923T031300Z --mode validate
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_join_online03.py tests/test_join_online02.py tests/test_join_online02_stop_audit.py tests/test_join_source04.py tests/test_online_history.py tests/test_se2.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Pre-model technical rendering/import and unit tests are separate from
scientific calls. No MPC solve/model inference is used in preflight or tests.

## Completed result

**BYPASS_ONSET_BUT_NO_COMPLETE_BYPASS.** All four frozen episodes ran once.
The new ON responses turn laterally and continue rather than producing the
previous native STOP. Both ON episodes nevertheless terminate at the unchanged
oracle guard before the next command would violate required clearance. This
is a behavior change with incomplete avoidance, not successful cart traversal
or an optimization-ready full bypass source.

Execution revision, committed and pushed before model startup:
`1bd1be8f1c50f27a638e1e638b3b6870c01eb0d2`.
No source, numerical controller, history policy, geometry, threshold or
instruction was changed after collection started. No episode was retried.

| New episode | Native/collector termination | Responses / applied chunks | First safe onset | Full raw bypass | Minimum actual edge clearance (m) | Final longitudinal position relative to cart (m) |
|---|---|---:|---|---|---:|---:|
| OFF_REPEAT_00 | MODEL_STOP | 15 / 14 | N/A | N/A | 0.519634 | +3.984297 |
| ON_REPEAT_00 | SAFETY_ABORT_BEFORE_UNSAFE_COMMAND | 7 / 6 | C1 | None | 0.053295 | −0.405908 |
| OFF_REPEAT_01 | MODEL_STOP | 14 / 13 | N/A | N/A | 0.560607 | +3.685600 |
| ON_REPEAT_01 | SAFETY_ABORT_BEFORE_UNSAFE_COMMAND | 7 / 7 | C1 | None | 0.053732 | −0.324205 |

Actual clearance uses the unchanged swept checker with the exact-command curve
bound. OFF clearance concerns its actual cart-absent environment. OFF paths
crossing a hypothetical cart are not actual collisions. OFF STOP is not proof
of reaching the hallway end: the experiment has no independently specified
hallway-end goal region. The ON episodes end before the cart rear plane.

### Raw FRESH and actual execution are different records

All new ON raw arrays have ten spatial rows; the implementation remains generic
in N. C0–C4 are individually safe raw polylines. Neither their safety nor the
frozen SAFE_BYPASS_ONSET label demonstrates passage past the cart. C5/C6 are
unsafe. No observation-to-first-row connector or trimmed suffix is used.

| ON repeat | Chunk | Raw arc (m) | Max absolute hallway lateral offset (m) | Final yaw relative to hallway (rad) | Raw minimum edge clearance (m) | Physical overlap in raw polyline | First unsafe row / segment, zero-based |
|---|---|---:|---:|---:|---:|---|---|
| 00 | C0 | 1.357453 | 0.132912 | −0.258270 | 1.278421 | false | N/A |
| 00 | C1 | 1.352627 | 0.294823 | −0.294496 | 1.010515 | false | N/A |
| 00 | C2 | 1.347980 | 0.286984 | −0.012182 | 0.750321 | false | N/A |
| 00 | C3 | 1.355374 | 0.446913 | −0.249315 | 0.677214 | false | N/A |
| 00 | C4 | 1.357550 | 0.633367 | −0.316461 | 0.420564 | false | N/A |
| 00 | C5 | 1.355843 | 0.599066 | +0.266654 | −0.073963 | true | 7 / 6 |
| 00 | C6 | 1.354077 | 0.593277 | +0.284177 | −0.140236 | true | 2 / 1 |
| 01 | C0 | 1.357453 | 0.132912 | −0.258270 | 1.278421 | false | N/A |
| 01 | C1 | 1.352627 | 0.294930 | −0.294527 | 1.009538 | false | N/A |
| 01 | C2 | 1.355665 | 0.457766 | −0.145064 | 0.587303 | false | N/A |
| 01 | C3 | 1.354497 | 0.591220 | −0.018197 | 0.478653 | false | N/A |
| 01 | C4 | 1.354497 | 0.707577 | +0.022593 | 0.458578 | false | N/A |
| 01 | C5 | 1.355843 | 0.715282 | +0.323092 | +0.005718 | false | 8 / 7 |
| 01 | C6 | 1.354077 | 0.672824 | +0.313965 | −0.089948 | true | 2 / 1 |

The lateral column is offset from the fixed hallway centre, not deformation
within a single chunk. C5/C6 turn back toward the cart while the agent is on its
side. Repeat01 C5 has positive clearance but less than .05 m: required-clearance
failure, not physical overlap. Its C6 and repeat00 C5/C6 do overlap the cart
under the frozen footprint. These are predicted-path findings, not executed
collisions.

| Repeat | Guard simulation time (s) | Unapplied held command (v m/s, omega rad/s) | Next-step edge lower bound (m) | Command source | Applied? |
|---|---:|---|---:|---|---|
| ON00 | 7.150000 | (0.8, 0.241386) | 0.042435 | C5 | false |
| ON01 | 7.266667 | (0.8, 0.247143) | 0.046244 | C6 | false |

The nominal .05 m edge margin and .20 m footprint are unchanged. The next
1/60 s interval has curve allowances 0.000006705 / 0.000006865 m respectively.
It violates required clearance but does not yet predict physical overlap in
that one step. The actual executions remain above .05 m, without collision.
ON00 received and installed C6 but never applied its first command; its final
guard decision checks the held C5 command. ON01 applied C6 before its abort.
No post-abort motion is fabricated.

### Boundary and timing evidence

First onset C1 in both repetitions is an early partial side response. Its
observation is still about 4.64 m from the cart centre, so this onset is not
evidence of a complete bypass local horizon. OLD C0 remains active during the
request; physical v-minus at the new command boundary is .8 m/s.

| C1 record | ON00 | ON01 |
|---|---:|---:|
| observation simulation time (s) | 1.783333 | 1.783333 |
| request-to-receipt host RTT (s) | 0.272659 | 0.267028 |
| ready-seen / install simulation time (s) | 2.050000 | 2.050000 |
| first command application simulation time (s) | 2.166667 | 2.183333 |
| observation-to-B traveled distance (m) | 0.306667 | 0.320000 |
| B edge clearance (m) | 3.916119 | 3.899499 |

Host clock differences are not subtracted from simulation timestamps. Exact
UTC/monotonic request and receipt records and all state/command memory values
are in each chunk record. The early moving-B onset bundle retained by the
existing selector is **not** a qualified complete bypass source: its
`moving_handoff_available` field only certifies that an actual moving boundary
exists. Neither episode provides a full safe cart-passing raw future.

All fourteen ON request-local RTF values exceed the retained 1.2 upper bound
(approximately 1.346–1.426), and maximum loop stalls are .255182 / .257825 s,
above .25 s. Whole-episode RTF .9793 / .9920 does not repair those failures.
The behavior evidence has acquisition pacing limitations. No timing-qualified
source or online-latency improvement is claimed.

### Historical comparison and limits

Historical ON00/ON01 ended at native C6 STOP, with actual minimum edge clearance
about .355 m. Their paths shortened from approximately 1.356 to 1.056 to .511 m
before STOP, without a lateral bypass. New ON paths retain approximately
1.35 m arc through C6, turn farther sideways, and encounter the guard.

New OFF controls also turn slightly: OFF00 C0 is byte-identical to both new ON
C0 arrays. The initial turn alone therefore is not evidence of visually
conditioned avoidance. At C1, returned lateral change is approximately −.287 m
in both ON repetitions versus −.086 / −.074 m in the OFF repetitions. ON side
offset reaches .633 / .715 m before termination; OFF C5 offsets are .202 / .172 m.
The later visual conditions and trajectories differ, so this is a scenario
comparison with changing live histories, not a fixed-image wording contrast.

All ON responses have non-STOP action codes. C5 and C6 codes match between the
two ON repetitions, while the corresponding world paths differ with their
observation poses. All ON APOS are unclamped. APOS/OPOS are observable outputs;
they do not reveal the model's internal obstacle reasoning or prove grounding
on the intended hallway end.

Supported conclusion: the requested wording changes the observed online
behavior and produces partial lateral response, but does not achieve safe
bypass. It cannot establish that the previous failure was solely bad prompting.
The central remaining uncertainty is why a partial side response turns back
before sufficient footprint clearance; intended destination grounding and
action geometry remain unseparated. No next experiment is implemented here.

### Compute, validation and review

- 43 actual scientific terminal predictions; 136 buffer-only requests;
  one separately counted technical server warmup. Terminal client RTT sum
  15.402936 s; collection wall time including Isaac startup 68.147044 s.
- 409 accepted official MPC submissions, 408 saved solve results. Final
  `ON_REPEAT_01_solve_000061` has no saved completion and no applied command.
  Saved solve wall times sum to 1.679291 s, a lower bound; complete solver wall
  time is unavailable. These counts are not collapsed into a claim that every
  accepted submission has a recorded completed solve.
- New GP, rigid and reconciliation calls: zero. Test model/MPC calls: zero.
  Technical preflight verifies the same cart mesh, camera/state, OFF zero pixels,
  ON 372 pixels and pinned official MPC; no technical preflight MPC solve.
- `validation.json`: PASS. Saved-only recomputation verifies all four streams,
  raw/wire/anchor provenance, exact state integration (maximum reconstruction
  error 0), applied command lineage, all guard queries, independent sessions,
  config/source hashes, complete result analysis and figure numeric sidecars.
  Scientific and timing failures remain failures.
- 293 relevant tests pass. First sandbox attempt passed 291 and blocked two
  local Unix-socket tests with EPERM; rerunning that nine-test IPC module with
  local socket permission passed. No scientific rerun. Compileall and diff
  whitespace checks pass.

Authoritative local artifacts, relative to the new run root:

- `aggregate/analysis.json`, `aggregate/chunks.csv`,
  `aggregate/instruction_comparison.json`, `aggregate/call_counts.json`.
- `episodes/ON_REPEAT_00/guard_abort.json` and
  `episodes/ON_REPEAT_01/guard_abort.json`: proposed-but-unapplied commands.
- `validation.json`: authoritative saved-record validation.
- `index.html`: primary new experiment review;
  `instruction_review_bundle.zip`: all figures and numerical/source sidecars.
- `review/ON_REPEAT_00_world.png`, `review/ON_REPEAT_01_world.png`:
  dashed raw futures versus solid actual execution. Brown dotted contour is
  the cart mesh inflated by footprint radius plus required margin (.25 m).
- `review/ON_REPEAT_01_chunks_01.png`: actual C6 RGB and unsafe raw FRESH.
  No diagnostic annotation is passed back into model RGB.

World overlays and terminal RGB/geometry panels were visually inspected for
legible legends, axes and distinct prediction/execution lines. Presentation is
static saved evidence; no extra GUI or diagnostic rollout was run.

Final regression commands (no model or real MPC solves):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_join_online03.py tests/test_join_online02.py tests/test_join_online02_stop_audit.py tests/test_join_source04.py tests/test_online_history.py tests/test_se2.py tests/test_online_mpc_adapter.py tests/test_online_ipc.py tests/test_online_switch.py tests/test_online_handoff_analysis.py tests/test_robotless_online.py tests/test_robotless_online_validator.py tests/test_robotless_online_replay.py tests/test_join_source02_geometry.py
# Same local IPC module, with Unix sockets permitted after sandbox EPERM:
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_online_ipc.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```
