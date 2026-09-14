# Official LightNav MuJoCo TurtleBot reproduction

Decision: **OFFICIAL_LIGHTNAV_MUJOCO_DEMO_REPRODUCED**.

On 2026-09-14 this machine ran the authors' unmodified MuJoCo TurtleBot, bundled
ProcTHOR scene, LightNav WebSocket client/server, and CasADi/IPOPT MPC. The single
predeclared autonomous run consumed 26 finite waypoint chunks, moved the robot,
and ended on model STOP after 6.849 seconds. This establishes an executable
reference pipeline, not navigation success or reconciliation effectiveness.

## Source and repository boundary

- Research starting HEAD: `66f890eb81247c2cbaa9760ad33d54add35cf275`, branch `main`.
- Fetched `origin/main`: `6b5781752ffa89d5e823700a68e07b1492dcca2a`, the externally
  reviewed commit. The existing local PDF/presentation commit was inspected and retained.
- Preserved unrelated edits in `configs/stage0_jackal_controller_validation.yaml`
  and `configs/stage0_lightnav_single_chunk.yaml`; their SHA-256 values are in
  the generated `source_provenance.json` and remained unchanged.
- Fresh independent checkout: `/home/gpuadmin/Workspace/external/LightNav-0-official-demo`.
- Remote: `https://github.com/lightorigins/LightNav-0.git`.
- `official_lightnav_git_sha`: `c6f40e3220edbf7011e4f17eaf2c865416737d4d`
  (`README: update insight-bench url`), detached HEAD.
- `git status --short` was empty after checkout, installation, tests, and runtime.
  No upstream source, assets, lockfiles, or scientific behavior were changed.
- The previous LightNav source/environment was neither used nor modified. Only
  its existing checkpoint was validated and read, as explicitly allowed below.
- Historical Isaac/Jackal, DATA-02, and EXP-02 results and generated data remain
  intact. No research instrumentation, controller, robot, camera, or scene was added.

Inspected the pinned source's `README.md`, `docs/GETTING_STARTED.md`,
`docs/DEPLOYMENT.md`, `docs/CONFIGURATION.md`, `docs/JETSON_THOR.md`, root
`pyproject.toml`, `scripts/smoke_gpu.sh`, and official CLI/server implementation.
For the demo, inspected `README.md`, `pyproject.toml`, `run.sh`, `__main__.py`,
`model.py`, `simulation.py`, `server.py`, `mpc.py`, `vln_client.py`,
`robots/base.py`, `robots/turtlebot.py`, `web/index.html`, asset manifest and
scene references, and all nine files under `mujoco_demo/tests/`.

## Machine and isolated environments

| Item | Observed value |
| --- | --- |
| OS | Ubuntu 24.04.4 LTS, Linux 7.0.0-31-generic, x86_64 |
| System Python | `/usr/bin/python3`, Python 3.12.3; unchanged |
| GPU | NVIDIA GeForce RTX 5060 Ti, 16,311 MiB reported VRAM, capability 12.0 |
| NVIDIA driver | 595.84; `nvidia-smi` CUDA capability 13.2 |
| uv | 0.12.9 |
| Demo Python | 3.11.16, external checkout `mujoco_demo/.venv` |
| Demo libraries | MuJoCo 3.11.0, CasADi 3.7.2, aiohttp 3.14.3, NumPy 2.4.6, Pillow 12.3.0 |
| Demo testing | pytest 9.1.1, pytest-asyncio 1.4.0 |
| Renderer | Default GLFW, `DISPLAY=:1`; `MUJOCO_GL` unset |
| Inference Python | 3.11.16, separate external checkout `.venv` |
| Inference package | LightNav 0.1.0, editable from the fresh pinned checkout |
| Inference libraries | torch 2.10.0+cu128, torchvision 0.25.0, vLLM 0.19.1, Transformers 5.8.0, CUTLASS DSL 4.5.2 |
| Torch CUDA runtime | 12.8; `CUDA_VISIBLE_DEVICES=0` |

The initial plain `uv run pytest` failed before collection: inherited
`PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages` loaded ROS's
`launch_testing` plugin, which failed with `ModuleNotFoundError: yaml` in the
isolated Python 3.11 environment. The fix was process isolation:
`env -u PYTHONPATH -u LD_LIBRARY_PATH`. This removes inherited ROS Python and
ROS/system-CUDA library search paths only for those child processes. No ROS,
system Python, driver, CUDA installation, or global package was changed.

The pinned docs' cu129 workaround concerns sm_103 B300/B30Z, and the Jetson
instructions concern aarch64/SM 11.0. Neither was needed here. Default bf16,
CUDA graph execution (`enforce_eager=False`), default cache allocation, and
official attention selection all worked. No quantization or runtime GPU
workaround was applied. The vLLM internal patches named in server logs are
part of the released LightNav source, not changes made for this reproduction.

## Observed TurtleBot implementation

These facts come from the code, not an assumption about a physical TurtleBot.
The CLI defaults to TurtleBot and `Simulation()` constructs `TurtleBotBackend`.
The implementation is in
[`robots/turtlebot.py`](https://github.com/lightorigins/LightNav-0/blob/c6f40e3220edbf7011e4f17eaf2c865416737d4d/mujoco_demo/vln_mujoco/robots/turtlebot.py).

| Property | Official source value |
| --- | --- |
| Geometry | Project-defined cylinders/plates, mast, lidar, camera box, two wheels and spherical caster; simplified differential-drive TurtleBot-like shape |
| Wheel radius / track | 0.033 m / 0.160 m; wheel centers at y=±0.080 m |
| Initial world robot pose | x=6.5 m, y=13.8 m, z=0.033 m, yaw=0 rad |
| First-person camera | `robot_rgb`, fixed at body-relative (0.090, 0, 0.165) m |
| Camera orientation | `xyaxes="0 -1 0 0 0 1"`; optical forward along body +X, image right along body −Y, image up along +Z |
| First-person FOV | Source vertical FOV 79.865°; 480×270 RGB; derived horizontal FOV ≈112.200° |
| Initial world camera position | (6.590, 13.8, 0.198) m, derived by adding body-relative offset at yaw=0 |
| Third-person camera | Body-relative (−0.65, 0, 0.50) m, `xyaxes="0 -1 0 0.38 0 0.925"`, vertical FOV 65° |
| Simulation step | 0.005 s; target camera interval 0.05 s; stale manual command timeout 0.35 s |

`TurtleBotBackend.step()` directly integrates base x/y with midpoint heading,
updates the yaw quaternion, updates wheel angle/velocity, advances `data.time`,
and calls `mj_forward`. It does not use `mj_step` to move the body through
wheel-contact dynamics. The model has zero actuators. These are kinematic
movement semantics despite the presence of masses, contact geometry and friction
properties in the MJCF. This is not the official high-fidelity TurtleBot3 mesh
or a validation of physical TurtleBot dynamics.

## Bundled scene and integrity

- Scene ID: `procthor-val-2`; scene name `val_2`.
- Dataset: MolmoSpaces ProcTHOR 10K validation; ceiling variant.
- XML: `mujoco_demo/vln_mujoco/assets/scenes/procthor-10k-val/val_2_ceiling.xml`.
- Manifest: `mujoco_demo/vln_mujoco/assets/manifest.json`.
- MolmoSpaces revision: `c89e1f5481af56fd25ef4efb76bdced9b726ec6a`.
- **2,236 files, 65,457,963 bytes** (about 62.43 MiB).
- Manifest SHA-256: `101052ef34ac643358656859f8d8c9fe39199c7ed01499416ca1a377ed645b17`.
- Every manifest file matched its recorded size and SHA-256. The XML's 2,235
  file references (2,231 unique) resolve inside the bundled asset root.
- Compilation, both rendered cameras, and official asset tests passed.
  No full ProcTHOR/MolmoSpaces download was needed.

`model.py` freezes environment joints and resolves asset paths in memory as
part of the official loader. Those are observed upstream behaviors; on-disk
scene files were not changed. No Isaac Sim or ROS is required by the default
demo's imports or dependencies.

## Phase A: simulator-only acceptance

`./run.sh --robot turtlebot` started and served `http://127.0.0.1:8088`.
`/api/health` returned HTTP success with `ok=true`, robot `TurtleBot`, scene
`procthor-val-2`, and ready camera. Both `/api/camera.jpg` and
`/api/third-person.jpg` returned valid 480×270 JPEGs and were visually inspected.

Through the unchanged official `/ws` control API, a temporary ignored client
acquired manual control, sent `{"type":"twist","linear":0.2,"angular":0}`
ten times at requested 0.1-second intervals, then sent `stop` and `reset`.

| Event | World pose (x m, y m, z m, yaw rad) |
| --- | --- |
| Before drive | (6.5, 13.8, 0.033, 0) |
| After drive | (6.7020000000000675, 13.8, 0.033, 0) |
| After reset | (6.5, 13.8, 0.033, 0), exact equality |

Manual motion is consistent with positive forward velocity at yaw zero.
Simulator-only acceptance passed before any LightNav environment installation
or server load. The Phase A process was stopped before the server-only test.

## Phase B: checkpoint, server and sanity inference

Checkpoint identifier: `LightOriginsHQ/LightNav-0`. Reused read-only:
`/home/gpuadmin/Workspace/external/LightNav-0/checkpoints/LightNav-0`.

All 20 local checkpoint files match published revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`. Live Hugging Face main resolved to
`826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`. All model, configuration, tokenizer,
and decoder files also match that current revision exactly; the only changed
file is ancillary `wechat_group.png`. The local directory is therefore the
older full snapshot with identical current inference assets, not a claim of
full-directory equality to current main. No checkpoint download was needed.

Verification used official API blob metadata, SHA-256 for LFS files, and Git
blob SHA-1 for regular files. SHA-256 was also recorded for every local file.
All file sizes and mtimes remained unchanged after inference.

| Important file | SHA-256 |
| --- | --- |
| Model safetensors | `ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18` |
| `config.json` | `0e047b5e9ae7981cf0066e74d8b655a2f49d7d7965c25fca386f9eea80b63e58` |
| `eval_config.json` | `7f475edbef4a13d2db99fa12eff5e9dedf48916969ec777f75e6e85e2a492b57` |
| `processor_config.json` | `ac89ec99ea773dc28f5b217edec5947120071ad5b9edcd48d3843ecc63b9877e` |
| Decoder `manifest.json` | `00d77ae028edf59e0ce68b49122dd0a60da527f48508f5ee335a580993a12864` |
| `tokenizer_config.json` | `8e8e879bfe8a52ececc118c9b80a48240ab35ff01a8be88d9a071df7ee7c4803` |

The shipped RVQ decoder resolves through `eval_config.json` without an override:
horizon 10, three 256-entry levels, float32 codebooks of shape (256,30).
All decoder dependencies exist and match published hashes.

The fresh official `lightnav-serve` built/loaded the model, reporting 8.7 GiB
for model loading. Engine initialization took 25.68 s; the separate official
synthetic readiness warm-up succeeded in 519 ms. READY and listening on
`0.0.0.0:8050` occurred at **2026-09-14 06:07:58.631 UTC**.
Synthetic warm-up is a runtime check, not experimental evidence.

GPU memory observations are snapshots, not measured peaks:

| Observation | GPU memory |
| --- | --- |
| Initial desktop, before demo/model | 1,075 MiB used |
| Before model load, after stopping Phase A demo | Server log: 14.3 GiB free |
| Model ready, simulator stopped | 13,278 MiB used, 2,550 MiB free |
| Both official processes loaded, after autonomous STOP | 14,865 MiB used, 963 MiB free |
| Server process after integration | 12,138 MiB allocated, as reported by `nvidia-smi` |

The standalone torch probe's 2,842 MiB observation included the Phase A
renderer and probe CUDA context, and is not the immediate pre-load baseline.
The file named `integrated/gpu_during_run.csv` was sampled after the short
autonomous run had already emitted STOP; it measures both loaded processes.

The official `lightnav-ws-client` succeeded on one saved simulator RGB frame.
A separate one-frame check using the documented minimal login/reset/next
protocol saved the complete raw response for validation: **(10,3), all finite,
stop=false, server latency 195.091 ms, measured round trip 202.185 ms**.
JSON numbers carry no dtype field. The official CLI reported 193.3 ms on its
own request. Both used the same fixed instruction; no prompt search occurred.
The raw response and original JPEG remain separate from derived summaries.

## Phase C: integrated autonomous acceptance

Both official components ran directly: LightNav on 8050 and the TurtleBot
demo on 8088 with `--vln-server ws://127.0.0.1:8050`. No proxy, execution
adapter, custom controller, or source patch intervened. The temporary test
client only sent ordinary web UI controls and saved public status messages.

Instruction, declared before the run:

> move forward, then go to the trashcan on the right

This is the exact default in the pinned `web/index.html`. The scene includes
GarbageCan objects; the green trashcan is visible in the mid-run images.
The initial view partly obscures that target, consistent with the instruction's
forward-motion preface. The endpoint was model STOP, clear completion, or
60 seconds, whichever occurred first. Completion was not an acceptance gate.

| Acceptance observation | Result |
| --- | --- |
| Start / end UTC | 06:09:26.257370 / 06:09:33.106030 on 2026-09-14 |
| Duration | 6.848662 s, measured by monotonic clock |
| Connection | Official server logs `client_id=vln_mujoco`; public status connected/RUNNING |
| Consumed results | 26 distinct accepted result updates; all finite (10,3) |
| STOP fields | 25 false, then one true |
| Client round-trip latency | 208.297–295.363 ms |
| MPC | 59 distinct non-null solve-duration values, 2.690–6.756 ms; empty error |
| Observed MPC commands | All finite; linear 0–0.8 m/s, angular −1.585494–1.055956 rad/s |
| Initial world pose | (6.5, 13.8, 0.033, 0) |
| Final world pose | (9.3378515006, 11.1757109215, 0.033, −0.9599450898) |
| Net planar displacement | 3.865267689 m; not path length |
| Sequence/schema/capture errors | None observed |
| MPC fatal error / process crash | None observed |
| End | Model STOP; zero final velocity/command, control released |

Sequence evidence is deliberately precise. The public `vln.sequence` counts
sent requests, not returned results. Each of the first 25 consumed result
updates has a distinct latency, finite waypoints, and contemporaneous request
counter 2 through 26. On STOP, the counter resets to zero. The unmodified
client checks exact response/request sequence equality before accepting each
result (`vln_client.py:228–247`); no mismatch was reported. We count consumed
latency/result updates, not outgoing request increments. One additional
in-flight inference completed after STOP; it is not counted as consumed.
The public API does not expose the individual raw returned sequence IDs.

Capture alignment was exercised through the official implementation:
`simulation.py` renders both views and captures pose under the simulation lock,
then attaches a wall-clock `time.time_ns()` stamp. `server.py:351–384` saves the
pose for each offered frame and matches the result's locally retained stamp
before calling `MpcTracker.set_body_path()`. No
`missing robot pose for VLN capture timestamp` error occurred.

Returned waypoints use body x-forward/y-left in metres, yaw CCW in radians.
The official MPC transforms them with the capture-time world pose, then
expresses the world trajectory in the current robot-local frame for tracking.
No external coordinate conversion was introduced. `mpc.py` supplies the
10 Hz, horizon-5, dt=0.1 s CasADi/IPOPT unicycle controller with default costs,
velocity and acceleration bounds. The waypoint dt is the controller's timing
convention, not an intrinsic model-output timestamp.

Finite `solve_ms`, finite nonzero commands, empty errors, and autonomous
motion establish accepted MPC execution. The public API exposes neither
literal IPOPT return-status strings nor matched capture-pose values; these
were not invented or instrumented. The official test suite's legacy control
helper tests alone would not establish runtime MPC behavior.

External status records carry UTC and monotonic observation times. The
official camera stamp is distinct from these, and `simulation.sim_time` is
the separate accumulated 0.005-second simulation clock. No clock equality or
new observation/readiness/execution timing model is assumed.

## Commands used

Commands below use the actual absolute locations; logs are retained in the
evidence root listed next. Installation uses the official project/extras.
No dependency changes were made to the research `.venv`.

```bash
cd /home/gpuadmin/Workspace/se-3-reconciliation
git status --short --branch
git branch --show-current
git remote -v
git fetch origin
git rev-parse HEAD origin/main
git log --oneline -15

git clone https://github.com/lightorigins/LightNav-0.git \
  /home/gpuadmin/Workspace/external/LightNav-0-official-demo
cd /home/gpuadmin/Workspace/external/LightNav-0-official-demo
git checkout --detach c6f40e3220edbf7011e4f17eaf2c865416737d4d
git status --short --branch
git rev-parse HEAD
git log -1 --oneline
git remote -v

cd mujoco_demo
uv sync --extra dev
uv run pytest  # initial inherited-ROS plugin import failure, before collection
env -u PYTHONPATH -u LD_LIBRARY_PATH uv run pytest
env -u PYTHONPATH -u LD_LIBRARY_PATH ./run.sh --robot turtlebot
```

With the last command running, the ignored `simulator/acceptance_client.py`
was executed using `mujoco_demo/.venv/bin/python` with the same environment
isolation. It uses `/api/health`, both JPEG endpoints, and `/ws` messages
`acquire_control`, `twist`, `stop`, `reset`. Then the Phase A process received
SIGTERM. The separate model environment and server were started as follows:

```bash
cd /home/gpuadmin/Workspace/external/LightNav-0-official-demo
env -u PYTHONPATH -u LD_LIBRARY_PATH python3.11 -m venv .venv
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  uv pip install --python .venv/bin/python -e '.[vllm,video]'

env -u PYTHONPATH -u LD_LIBRARY_PATH PORT=8050 CUDA_VISIBLE_DEVICES=0 \
  .venv/bin/lightnav-serve --task vln \
  --model_path /home/gpuadmin/Workspace/external/LightNav-0/checkpoints/LightNav-0 \
  --backend vllm_local
```

After READY, in another shell:

```bash
cd /home/gpuadmin/Workspace/external/LightNav-0-official-demo
env -u PYTHONPATH -u LD_LIBRARY_PATH .venv/bin/lightnav-ws-client \
  --server ws://127.0.0.1:8050 \
  --frames /home/gpuadmin/Workspace/se-3-reconciliation/data/reference_reproduction/lightnav_official_mujoco/20260914T060141Z/lightnav/sanity_frames \
  --instruction 'move forward, then go to the trashcan on the right'

cd mujoco_demo
env -u PYTHONPATH -u LD_LIBRARY_PATH ./run.sh \
  --robot turtlebot --vln-server ws://127.0.0.1:8050
```

The additional raw sanity check follows the minimal client in
`docs/DEPLOYMENT.md`: login, reset, one `next` containing seq=0, the original
JPEG as base64, and the same instruction. Exact requests/responses and their
timestamps are preserved in `lightnav/logs/sanity_protocol.jsonl`.

With both components running:

```bash
cd /home/gpuadmin/Workspace/se-3-reconciliation
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  /home/gpuadmin/Workspace/external/LightNav-0-official-demo/mujoco_demo/.venv/bin/python \
  data/reference_reproduction/lightnav_official_mujoco/20260914T060141Z/integrated/acceptance_client.py
```

This ignored temporary client sends `set_vln` with `enabled=true` and the
fixed instruction, keeps the owning socket open, observes the declared stop
condition, saves final health before sending `enabled=false`, and disconnects.
Both official processes were subsequently observed alive with listening
sockets, then deliberately stopped with SIGTERM after evidence collection.

## Tests, warnings, and integrity

| Check | Result |
| --- | --- |
| Initial isolated official `uv run pytest` | 27 passed, 0 failed, 16.96 s |
| Final isolated official `uv run pytest` | 27 passed, 0 failed, 16.67 s |
| `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest` in research repo | 512 passed, 0 failed, 20.27 s |
| `.venv/bin/python -m compileall src scripts tests` | Passed |
| `git diff --check` | Passed before commit |
| Final external `git status --short` and `git diff --exit-code` | Clean / passed |

No source fix was needed. Retained failures/warnings:

- Initial ROS plugin import failure described above; zero tests collected.
- Restricted sandbox initially denied `.git/FETCH_HEAD`, GPU/port access,
  and uv cache writes. Authorized host execution resolved runtime access;
  a `/tmp` uv cache was used for read-only package inventory. These were
  execution-environment restrictions, not upstream demo failures.
- Upstream Transformers image-processor/`use_fast` and CUDA Python module
  deprecations; vLLM global-seed warning; upstream disabling-chunked-prefill
  warning; insufficient SMs for `max_autotune_gemm`; explicit KV-cache memory
  informational notice. No corresponding runtime failure was observed.
- After deliberate SIGTERM, the server emitted a PyTorch NCCL warning that
  `destroy_process_group()` was not called before exit. This occurred during
  teardown, after the successful integrated run; it was not a runtime crash.

No extra tests or SE(2)/timing implementation were added to the research repo.
Its only changes for this task are this report, a small README note, and an
append-only work-log entry. External source/assets, checkpoints, raw/generated
evidence and virtual environments are not staged or committed.

## Evidence

Ignored local run root:
`data/reference_reproduction/lightnav_official_mujoco/20260914T060141Z/`.
The existing `data/` Git ignore rule covers every artifact below.

| Artifact | Relative path within run root |
| --- | --- |
| Environment / source / decision | `environment.json`, `source_provenance.json`, `validation.json` |
| Asset validation | `simulator/asset_validation.json` |
| Simulator health | `simulator/health_initial.json`, `simulator/health_after_manual_motion.json`, `simulator/health_after_reset.json` |
| Simulator images | `simulator/camera.jpg`, `simulator/third_person.jpg` |
| Simulator logs and temporary client | `simulator/logs/`, `simulator/acceptance_client.py`, `simulator/acceptance.json` |
| Checkpoint validation / raw remote metadata | `lightnav/checkpoint_validation.json`, `lightnav/hf_model_main.raw.json`, `lightnav/hf_model_historical.raw.json` |
| Server metadata / raw sanity response | `lightnav/server_metadata.json`, `lightnav/sanity_response.raw.json`, `lightnav/sanity_validation.json` |
| Model/install/sanity logs | `lightnav/logs/` |
| Integration predeclaration and run summary | `integrated/predeclared_acceptance.json`, `integrated/run_metadata.json`, `integrated/validation.json` |
| Integrated health | `integrated/health_initial.json`, `integrated/health_mid.json`, `integrated/health_final.json`, `integrated/health_after_operator_stop.json` |
| Integrated images | `integrated/camera_mid.jpg`, `integrated/third_person_mid.jpg` |
| Integrated raw public status and demo log | `integrated/logs/websocket.jsonl`, `integrated/logs/demo.log` |
| Live-process proof | `processes_after_run.json` |
| Artifact hashes | `evidence_manifest.json` |
| Tests / inventories | `research_pytest.log`, `research_compileall.log`, each environment's `package_freeze.txt`, official test logs in `simulator/logs/` |

Health/JPEG/log files are original observations. JSON validation summaries
identify their input paths and processing conventions; the evidence manifest
records hashes. No raw VLA response was overwritten or transformed.

## Claim boundary and next task

This validates scene loading, the official kinematic TurtleBot, RGB rendering,
the official LightNav protocol and waypoint chunks, capture-time pose alignment,
official MPC execution, and autonomous movement on this machine.

It does not establish navigation success rate, obstacle avoidance quality,
collision safety, real-world TurtleBot dynamics, Jackal equivalence,
reconciliation effectiveness, OLD/FRESH discontinuities, or graph-optimization
performance. Model STOP and a visible target are not a navigation benchmark.
Historical research is preserved without reclassification.

Next task: add passive OLD/FRESH transition instrumentation without changing
the official TurtleBot, scene, LightNav client, or MPC behavior.
That next task is not implemented here.
