# Robotless successive LightNav chunks in Isaac Sim

Decision: **ROBOTLESS_SUCCESSIVE_CHUNKS_VALIDATED**.

On 2026-09-14, one Isaac capture process produced two RGB observations at
different logical poses. One official LightNav session returned OLD and FRESH,
each finite `(10,3)`. Both paths were transformed using their own observation
poses and inspected together in the actual Isaac viewport.

## Research question

Can two observations at different logical SE(2) poses in the same robotless
Isaac scene produce successive OLD/FRESH predictions in one LightNav session,
with each chunk correctly represented in the same world frame using its own
observation pose?

OLD means C0; FRESH means C1. This task validates generation, session continuity,
coordinate frames and timestamp semantics. It does not execute a trajectory,
reconcile chunks, choose a switch boundary or evaluate navigation success.

## Architecture and reuse

The workflow uses the previously validated
[single-chunk interface](ROBOTLESS_ISAAC_LIGHTNAV_SINGLE_CHUNK.md).
Scene loading, logical pose assignment, the child camera and viewport readback
are shared in `scripts/isaac/robotless_runtime.py`. The original single-chunk CLI
continues to use the same behavior through that helper. Pure schema, immutable
file writers, checkpoint audits, response parsing and SE(2) composition are reused.

1. One Isaac process loads the Hospital, sets R0, captures RGB0, directly assigns
   R1, captures RGB1, and exits. The simulation timeline remains stopped.
2. An unmodified official server starts in the external LightNav environment.
   One synchronous client connection sends login, reset, seq=0, then seq=1.
3. The server exits. Isaac reloads the saved observations and displays both
   observation poses, the scripted displacement, both raw-derived world paths
   and their headings simultaneously.

There is no robot mesh or articulation, wheel dynamics, controller/MPC,
trajectory following, asynchronous execution, correction, rigid alignment,
correspondence or graph optimization. Both captures precede inference; agent
motion cannot occur during inference in this file-handoff workflow.

## Pose and camera definitions

Agent +X is forward, +Y left, +Z up; positive yaw is CCW about +Z. World axes
are right-handed and +Z up. Translation is in metres, yaw in radians.

The configured values are:

```text
R0 = [19.0, 26.7, pi/2]
Delta_local = [0.30, 0, 0]
R1 = R0 * Delta_local = [19.0, 27.0, pi/2]
```

Thus the requested 0.30 m is R0-local forward, which is world +Y at this yaw.
The logical agent is assigned this pose directly between observations; it does
not follow the OLD trajectory. Capture records the actual poses and the local
relative transform, plus timestamps bracketing the assignment.

Both observations use the same child camera and sensor settings: 480×270 RGB,
requested 112.2° HFOV, agent-relative translation `[0.09,0,0.65]` m, pinhole lens.
USD camera +X right, +Y up and −Z optical forward map to agent −Y, +Z and +X.
Actual intrinsics, local camera extrinsics and world camera matrices are saved
for both observations. The local extrinsic must agree within the declared
numerical tolerance; world extrinsics change with the agent pose.

Configuration:
[`configs/robotless_lightnav_successive_chunks.yaml`](../configs/robotless_lightnav_successive_chunks.yaml).
The run saves an immutable configuration snapshot and its SHA-256.

## Session and timing semantics

Exactly one WebSocket context owns this sequence:

```text
connect -> login -> reset -> next(seq=0, RGB0) -> next(seq=1, RGB1) -> disconnect
```

There is one login, one reset and two predictions. No reconnect, intervening
reset, retry or instruction change is allowed. The complete request/response
wire messages and connection identifier are retained. An invalid result is
preserved before validation; it is never replaced by another inference.

Each observation records a UTC timestamp, host monotonic nanoseconds and Isaac
simulation time. Capture timestamps bracket render/readback; direct pose
assignment has its own host event bracket. Simulation time remains zero, so
host timestamps and explicit event order distinguish the captures.

Each request is stamped immediately before sending; its response is stamped
immediately after receive. Round-trip milliseconds are the difference between
those host monotonic timestamps divided by 1,000,000. UTC is provenance, not the
clock subtracted for RTT. RTT is recorded only: it is not agent-motion time,
switch latency, a waypoint schedule or evidence of a latency-induced gap.
Execution time is explicitly null. Capture timestamps are not sent as a new
protocol field; upstream history processing receives frames in seq order.

## Independent observation anchors

Both raw arrays contain arbitrary nonempty finite `N x 3` cumulative local
poses `[forward_m,left_m,yaw_ccw_rad]`. OLD and FRESH may have different N.
Rows are not increments to integrate.

```text
T_world_OLD_i   = T_world_R0 * T_R0_OLD_i
T_world_FRESH_j = T_world_R1 * T_R1_FRESH_j
```

World yaw is wrapped to `[-pi,pi)`. Neither the OLD endpoint nor R0 anchors
FRESH. No alignment or correction is applied. Raw wire responses and decoded
arrays remain immutable; each derived array records its raw inputs, config and
own observation pose.

## Reproduction commands

Run from `/home/gpuadmin/Workspace/se-3-reconciliation` with a fresh directory.
The server helper uses the external checkout's Python executable, records exact
argv/PID/checkpoint hashes and waits for the official readiness indication.
It sends no image requests. Its stop command verifies the task-owned process
identity before sending SIGTERM.

```bash
scripts/isaac/run_robotless_successive_chunks.sh capture RUN \
  --config configs/robotless_lightnav_successive_chunks.yaml --no-hold
.venv/bin/python scripts/lightnav/robotless_successive_server.py start RUN
scripts/lightnav/run_robotless_successive_inference.sh RUN --timeout-s 120
.venv/bin/python scripts/lightnav/robotless_successive_server.py stop RUN --timeout-s 60
scripts/isaac/run_robotless_successive_chunks.sh visualize RUN --no-hold

# Reopen immutable saved paths without another inference or evidence write:
scripts/isaac/run_robotless_successive_chunks.sh visualize RUN --view-only

# After inspecting and recording the actual viewport image:
.venv/bin/python scripts/validate_robotless_successive_chunks.py RUN --write
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

GPU, display, ordinary Isaac/model caches and Unix-socket tests require host
access. Isaac and LightNav environments remain separate. No package is installed
or external source modified by these commands.

## Artifacts

All generated files live under ignored `data/robotless_successive_chunks/`.

| Purpose | Run-relative files |
| --- | --- |
| Captured RGB | `raw/observation_000.jpg`, `raw/observation_001.jpg` |
| Raw decoded chunks | `raw/chunk_000.npy`, `raw/chunk_001.npy` |
| Untouched responses | `raw/response_000.json`, `raw/response_001.json` |
| Own-observation world paths | `derived/chunk_000_world.npy`, `derived/chunk_001_world.npy` |
| Frozen capture metadata/config | `metadata.json`, `config_snapshot.yaml`, `capture_validation.json` |
| Session/provenance/timestamps | `inference_metadata.json`, `server_process.json`, raw protocol log |
| Simultaneous viewport | `evidence/old_fresh_world_trajectories.png` |
| Runtime and explicit visual checks | `visualization_validation.json`, `visual_review.json`, `validation.json` |

Capture metadata is immutable. The separate inference record links its hash and
adds seq IDs, request/response times, RTTs and raw/derived hashes; together these
records contain the complete run metadata. The validator requires actual runtime
artifacts and an explicit screenshot review. Mock tests cannot establish PASS.

## Actual runtime result

Starting HEAD and fetched `origin/main` were both
`1572ea407bc26b6cc5ff94f967ed12316de729e3` on `main`. The first two host fetch
requests and a read-only GPU probe did not execute because automatic approval
review was at capacity. A later fetch succeeded before runtime and confirmed
that origin/main was unchanged; host GPU execution then succeeded. These access
attempts are retained in the run logs. No rejection was bypassed.

The actual run directory is
[`data/robotless_successive_chunks/20260914T073030Z/`](../data/robotless_successive_chunks/20260914T073030Z/).
Substitute that path for `RUN` in the commands above. The instruction was fixed
before capture: “At the end of the hallway, turn left into the cross corridor.”

| Check | Actual result |
| --- | --- |
| Isaac | Installed 6.0.1; Hospital loaded once for both captures |
| Scene inventory | 1,936 prims, 126 collision prims; zero robot-named paths, articulations, rigid bodies or physics scenes |
| R0 `[x_m,y_m,yaw_rad]` | `[19.0,26.7,1.5707963267948963]` |
| R1 `[x_m,y_m,yaw_rad]` | `[19.0,27.0,1.5707963267948963]` |
| Actual local displacement | `[0.3000000000000007,5.430501542237711e-16,0.0]` |
| Actual world XY displacement | `[0.0,0.3000000000000007]` m |
| RGB | Two distinct actual 480×270 corridor images; inspected directly |
| Camera | Same intrinsics and local extrinsics; HFOV 112.1999983°, VFOV 79.8646136°; `fx=161.2733123, fy=161.2733097, cx=240, cy=135` |
| Timeline | Stopped, 0.0 s for both captures and visualization; direct assignment only |
| Connection identifier | `e7399440-1437-4834-a3d4-31d0ff871488` |
| Actual wire sequence | One connection: login → reset → next(seq=0) → next(seq=1) → disconnect |
| Protocol counts | login=1, reset=1, next=2, reconnect=0, retry=0; constant instruction |
| Server response history | `actions.step=1` for OLD, `actions.step=2` for FRESH |
| Raw OLD / FRESH | Both finite `(10,3)`, both `stop=false`; exact nested response values preserved |
| Transform | OLD uses only R0; FRESH uses only R1; cumulative rows, no correction |
| Viewport | Both paths, both poses, displacement and all 10 headings per chunk visible together |

Observation and prediction timestamps below use UTC. The complete corresponding
monotonic nanosecond events are retained in metadata. The pose assignment was
bracketed by `07:36:52.758446Z` and `07:36:52.758774Z`, between captures.

| Chunk | Observation UTC | Request send UTC | Response receive UTC | Recorded RTT ms |
| --- | --- | --- | --- | --- |
| OLD, seq=0 | `2026-09-14T07:36:52.753173Z` | `2026-09-14T07:38:26.576888Z` | `2026-09-14T07:38:26.777008Z` | 200.128384 |
| FRESH, seq=1 | `2026-09-14T07:36:52.886816Z` | `2026-09-14T07:38:26.777476Z` | `2026-09-14T07:38:26.970064Z` | 192.592501 |

These are two recorded round trips, not a latency experiment or model-only
timings. Both RGB captures completed before the client connected.

First waypoint examples, in metres and radians:

```text
OLD local = [0.0005890281172469258, -0.000056214201322291046, 0.3125922977924347]
OLD world = [19.000056214201322, 26.700589028117246, 1.8833886245873313]

FRESH local = [0.150419220328331, 0.00005595880429609679, 0.0006009606295265257]
FRESH world = [18.999944041195704, 27.15041922032833, 1.571397287424423]
```

The server used the clean external source
`c6f40e3220edbf7011e4f17eaf2c865416737d4d` and existing checkpoint
`LightOriginsHQ/LightNav-0`, revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`.
Model SHA-256 is `ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18`.
The task-owned server PID was 3616889; exact argv, process start identity,
checkpoint manifest and readiness events are saved. Official READY occurred at
`2026-09-14T07:37:59.763Z`. Its built-in synthetic startup warm-up is separate
from the two actual observation requests and is not experimental evidence.
The server log contains one client connection and two actual prediction calls.
After inference, the stop helper verified ownership, sent SIGTERM and confirmed
exit before Isaac visualization. All checkpoint hashes, sizes and mtimes, and
the external source SHA/cleanliness, remained unchanged.

## Visualization and validation

The actual
[1280×720 viewport PNG](../data/robotless_successive_chunks/20260914T073030Z/evidence/old_fresh_world_trajectories.png)
was inspected: OLD is blue, FRESH magenta, R0 yellow, R1 orange, and the scripted
displacement green. Both paths and heading sets are simultaneously visible in
the Hospital. The blue heading fan near R0 reflects largely turning early OLD
rows; FRESH extends forward from R1. No geometry scaling or scene hiding was
used. Observation-to-first-waypoint display connectors are not extra model
rows. A separate UI legend and the visualization metadata identify the colors.

The explicit `visual_review.json` is bound to screenshot SHA-256
`fd93fe548fb068603c32ff754f3d2dd612d95d65d65f171c71e3f72ea7f0e08d`.
The offline validator verifies captured JPEG bytes against requests, complete
protocol ordering, raw response values, own-observation transforms, clocks,
config/provenance hashes and the actual reviewed screenshot. It returned the
validated decision with no missing artifacts or failures.

- Pure Python suite: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest`
  completed with **766 passed in 20.83 s**, including 136 new cases. Cases cover
  local SE(2) displacement, rotated anchors, arbitrary and unequal N, preservation,
  invalid/nonfinite inputs, timing conventions, the mock one-session protocol,
  server endpoint/provenance and tampered/missing runtime evidence. Mocks are tests only.
- `.venv/bin/python -m compileall src scripts tests`, shell syntax checks and
  `git diff --check` passed.
- Actual Isaac capture, actual two-request inference and simultaneous evidence
  visualization all exited successfully. A later successive `--view-only
  --no-hold` replay passed without another inference or evidence write.
- The prior single-chunk run also reopened successfully with `--view-only
  --no-hold`, exercising the shared runtime extraction. Its historical evidence
  manifest remains unchanged.

Runtime logs retain Isaac DLSS resolution/readback and plugin-release warnings,
upstream CUDA deprecations and seed/chunked-prefill warnings, and the official
server's NCCL teardown warning after deliberate SIGTERM. The observed commands
completed; these logs are retained rather than treating warning-free execution
as an acceptance condition.

## Changed files and Git scope

| Area | Files |
| --- | --- |
| Configuration | `configs/robotless_lightnav_successive_chunks.yaml` |
| Shared/capture/viewer | `scripts/isaac/robotless_runtime.py`, `scripts/isaac/robotless_successive_chunks.py`, `scripts/isaac/run_robotless_successive_chunks.sh`, `scripts/isaac/robotless_lightnav_single_chunk.py` |
| Isolated client/server | `scripts/lightnav/robotless_successive_inference.py`, `scripts/lightnav/run_robotless_successive_inference.sh`, `scripts/lightnav/robotless_successive_server.py`, `scripts/lightnav/robotless_single_frame_inference.py` |
| Pure helpers/validator | `src/reconciliation/robotless_successive_chunks.py`, `scripts/validate_robotless_successive_chunks.py` |
| Tests | `tests/test_robotless_successive_chunks.py`, `tests/test_robotless_successive_client.py`, `tests/test_robotless_successive_server.py`, `tests/test_robotless_successive_validation.py` |
| Documentation | This report, `README.md`, append-only `docs/WORK_LOG.md` |

The two preexisting modified Stage0 YAML files were preserved and excluded.
Generated data, upstream source, checkpoint files, caches and environments are
not committed. Exact executed research source snapshots and phase hashes are
retained in the ignored run. Final commit identity is recorded in the run's
`git_completion.json`; the focused report commit can also be found with
`git log -1 -- docs/ROBOTLESS_ISAAC_LIGHTNAV_SUCCESSIVE_CHUNKS.md`.
The final staged whitespace check removed extra trailing blank lines from
the shared runtime helper after the runtime tests; executed source snapshots
remain unchanged and retain the exact phase hashes.

## Limitations

The result establishes that two pose-distinct robotless observations can produce
successive chunks in one LightNav session and be expressed correctly in one
world frame with their respective observation anchors. It is a single observed
interface run, not an output-quality or statistical reproducibility study.

Even if the two paths show a gap, this task does not establish a need for
asynchronous reconciliation, latency causation, correspondence, graph benefit,
navigation success or improved closed-loop performance. Synthetic tests are
tests only. The remote Hospital asset tree is not recursively content-pinned;
its resolved URL and version family are recorded. The 0.65-m logical camera
height is a declared difference from the official demo's body-relative height.
No controller, asynchronous OLD execution, switch boundary B, correspondence,
rigid reconciliation or graph optimization was implemented in this task.
