# Robotless Isaac Sim and LightNav: one static chunk

Decision: **ROBOTLESS_SINGLE_CHUNK_VALIDATED**.

On 2026-09-14, actual Isaac RGB capture, one official LightNav prediction and
Isaac viewport rendering completed successfully. The raw output is finite
`(10, 3)`. Both forward/left fixtures and the actual world trajectory were
visually inspected. This establishes the requested static interface only.

## Research question and scope

Can an Isaac Sim logical SE(2) agent and child RGB camera, without a robot model,
provide one image to LightNav, preserve its local `N x 3` output, transform it
using the observation pose, and display that world trajectory in Isaac?

This is an interface and coordinate-frame validation. Instruction satisfaction,
navigation quality, obstacle avoidance, controller execution and moving-agent
behavior are outside its acceptance criteria. Existing Jackal, MuJoCo, EXP and
DATA records retain their historical meanings.

## Architecture

Three sequential processes exchange immutable files:

1. Isaac's bundled Python loads the Hospital scene, constructs a logical Xform
   and child camera, and captures one static RGB observation.
2. A thin client in the existing external LightNav Python environment sends the
   original JPEG bytes to the unmodified official server. It performs login,
   reset and exactly one prediction request.
3. Isaac reloads the frozen scene/configuration and observation pose, verifies
   the saved transform, and displays the trajectory and coordinate fixtures.

Isaac and the model server can release the GPU between phases. No LightNav or
Torch package is installed into Isaac or the research environment. There is no
robot asset, controller, dynamics execution, agent motion, asynchronous
navigation, OLD/FRESH transition, correspondence factor or graph optimization
in this implementation.

## Frames and transform

The world and logical agent are right-handed, +Z up. Agent +X is forward, +Y is
left; positive yaw is CCW about +Z. Translation uses metres and yaw uses radians.
LightNav rows are cumulative local poses, not increments to integrate.

For observation pose `R = [x_R, y_R, theta_R]` and local row `[f, l, theta]`:

```text
T_world_waypoint = T_world_agent(t_observation) * T_agent_waypoint
x_world = x_R + cos(theta_R)*f - sin(theta_R)*l
y_world = y_R + sin(theta_R)*f + cos(theta_R)*l
yaw_world = wrap(theta_R + theta), in [-pi, pi)
```

The implementation reuses `reconciliation.se2.local_trajectory_to_world` through
a robotless schema wrapper. It accepts any nonempty finite `N x 3` array,
preserves the raw array, and uses only the saved observation pose. Model rows
have no intrinsic timestamps; no waypoint execution schedule is assigned.

The USD camera has +X image-right, +Y image-up and -Z optical-forward. Its
column-vector rotation into the agent frame is explicitly configured:

```text
R_agent_camera = [[ 0, 0,-1],
                  [-1, 0, 0],
                  [ 0, 1, 0]]
```

Thus image-right is agent -Y, image-up is agent +Z, and optical-forward is agent
+X. Metadata records the actual USD transforms, intrinsics, axes and applied
FOV, rather than treating optical coordinates as local trajectory coordinates.

## Frozen configuration and provenance

The versioned configuration is
[`configs/robotless_lightnav_single_chunk.yaml`](../configs/robotless_lightnav_single_chunk.yaml).
Each capture saves and hashes its own configuration snapshot.

- Scene: `/Isaac/Environments/Hospital/hospital.usd`, Isaac 6.0 asset family,
  one metre per stage unit, +Z up. Reuses only the established scene choice.
- Static agent pose: `[19.0, 26.7, pi/2]`, ground-level logical origin.
- RGB: 480 by 270; requested horizontal FOV 112.2 degrees; pinhole camera.
- Camera translation in agent coordinates: `[0.09, 0, 0.65]` m.
  The 0.65-m height is a declared logical-camera choice. The official demo's
  body-relative 0.165-m height is not claimed to be reproduced.
- Camera axes, resolution and approximate FOV follow the inspected official
  reference semantics documented in the
  [previous reproduction](OFFICIAL_LIGHTNAV_MUJOCO_DEMO_REPRODUCTION.md).
- External source: `/home/gpuadmin/Workspace/external/LightNav-0-official-demo`,
  SHA `c6f40e3220edbf7011e4f17eaf2c865416737d4d`.
- Checkpoint: `LightOriginsHQ/LightNav-0`, existing local snapshot revision
  `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6` at
  `/home/gpuadmin/Workspace/external/LightNav-0/checkpoints/LightNav-0`.
- Model file SHA-256:
  `ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18`.
- Decoder manifest SHA-256:
  `00d77ae028edf59e0ce68b49122dd0a60da527f48508f5ee335a580993a12864`.

The wire protocol and cumulative-pose semantics were inspected directly in the
pinned external `docs/PROTOCOL.md`, `docs/DEPLOYMENT.md`, and server source.
The model's internal preprocessing remains upstream behavior. The client does
not resize or re-encode the captured JPEG.

## Evidence and reproducibility

Generated artifacts are ignored under `data/robotless_single_chunk/`. Raw JPEG,
wire response and decoded local array are separate from derived world arrays.
Exclusive creation prevents raw output replacement. Capture metadata includes
the observation's UTC, host monotonic and frozen simulation clock conventions,
RGB SHA-256, instruction, camera/scene/frame definitions, research source hashes
and LightNav provenance. Inference and visualization have separate records.

Forward/left fixtures at yaw zero and +90 degrees are deterministic coordinate
tests and visual demonstrations only. They are never model outputs or research
evidence. The actual LightNav path uses the same observation-anchored transform.

## Commands and observed result

Starting research HEAD and freshly fetched `origin/main` were both
`7c34b05a9f8852bb8ea9878926d9f85e65141f3d` on `main`. The two existing edits in
`configs/stage0_jackal_controller_validation.yaml` and
`configs/stage0_lightnav_single_chunk.yaml` were preserved and excluded.

The generated run is
`data/robotless_single_chunk/20260914T065832Z/`. The instruction was fixed before
capture/inference: “At the end of the hallway, turn left into the cross corridor.”

| Check | Actual observation |
| --- | --- |
| Isaac version | 6.0.1, installed `/home/gpuadmin/isaacsim` |
| Capture UTC | `2026-09-14T07:04:42.401007Z` |
| Observation pose | `[19.0, 26.7, 1.5707963267948963]` |
| Simulation time | 0.0 s throughout; timeline stopped |
| Stage inventory | 1,936 prims, 126 collision prims; zero robot-named paths, articulations, rigid bodies or physics scenes |
| RGB | 480×270 RGB JPEG; visible Hospital corridor, doors, floor and lights |
| Actual camera HFOV / VFOV | 112.1999983° / 79.8646136° |
| Actual camera aperture / focal length | horizontal 20.9549993 mm / focal 7.0405878 mm |
| Actual K | `fx=161.2733123, fy=161.2733097, cx=240, cy=135` |
| Camera basis | optical forward→agent +X; image right→−Y; image up→+Z; proper rotation and translation checks pass |
| Server | pinned official `lightnav-serve`, `vllm_local`, default bf16, localhost:8050 |
| Request | login, reset, exactly one `next(seq=0)` with the original JPEG and fixed instruction |
| Response | finite `(10,3)`, `seq=0`, `stop=false`; exact response and nested values preserved |
| World transform | exact observation-pose composition verified against saved derived array |
| Isaac visualization | all 10 headings, origin/forward/left, cyan trajectory; three inspected 1280×720 viewport PNGs |
| Upstream integrity | clean pinned source before/after; checkpoint hashes, sizes and mtimes unchanged |

One corresponding waypoint pair, in metres/radians:

```text
raw local: [0.0005890281172469258, -0.000056214201322291046, 0.3125922977924347]
world:     [19.000056214201322, 26.700589028117246, 1.8833886245873313]
```

The final local row is approximately `[-0.232920736, 0.421594232, 2.09684038]`.
The early rows primarily change yaw and almost coincide spatially; their heading
markers form the visible fan near the origin. Geometry is not scaled. The
observation-origin-to-first-waypoint segment is a display connector, not an
extra saved model waypoint. No trajectory is executed.

The following are the runtime commands used, from the repository root. The
host GPU/display and ordinary runtime caches require host execution; the first
sandboxed Git fetch was read-only-denied and then succeeded through authorized
host execution. No system packages or environments were modified.

```bash
git status --short --branch
git remote -v
git fetch origin
git rev-parse HEAD origin/main

scripts/isaac/run_robotless_lightnav_single_chunk.sh capture \
  data/robotless_single_chunk/20260914T065832Z \
  --config configs/robotless_lightnav_single_chunk.yaml
```

After Isaac exited, the official server ran from its external checkout with
`PYTHONPATH` and `LD_LIBRARY_PATH` removed for that child process:

```bash
cd /home/gpuadmin/Workspace/external/LightNav-0-official-demo
env -u PYTHONPATH -u LD_LIBRARY_PATH CUDA_VISIBLE_DEVICES=0 PYTHONUNBUFFERED=1 \
  .venv/bin/lightnav-serve --task vln \
  --model_path /home/gpuadmin/Workspace/external/LightNav-0/checkpoints/LightNav-0 \
  --backend vllm_local --host 127.0.0.1 --port 8050 \
  --ready_file /home/gpuadmin/Workspace/se-3-reconciliation/data/robotless_single_chunk/20260914T065832Z/server.ready
```

A host Python `subprocess.Popen(..., start_new_session=True)` wrapper retained
PID 3591173, exact argv/environment, source cleanliness and pre-run checkpoint
manifest in `server_launch.json`. After the process was observed alive with the
ready marker and the official READY log, it wrote `server_process.json` with
`ready_confirmed=true` and `ready_observed_utc`. The client binds that file by
hash; the wire protocol itself does not attest which model a server loaded.
The official server's built-in synthetic readiness warm-up is separate from
the single real observation request and is not experimental evidence.

```bash
cd /home/gpuadmin/Workspace/se-3-reconciliation
scripts/lightnav/run_robotless_single_frame_inference.sh \
  data/robotless_single_chunk/20260914T065832Z --timeout-s 120
```

After that command completed, the wrapper sent SIGTERM only to its own server
PID and confirmed GPU compute processes had exited. Then:

```bash
scripts/isaac/run_robotless_lightnav_single_chunk.sh visualize \
  data/robotless_single_chunk/20260914T065832Z --no-hold

# Reopen saved data without inference, evidence replacement or motion:
scripts/isaac/run_robotless_lightnav_single_chunk.sh visualize \
  data/robotless_single_chunk/20260914T065832Z --view-only

.venv/bin/python scripts/validate_robotless_single_chunk.py \
  data/robotless_single_chunk/20260914T065832Z

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

Use a fresh run directory for another capture/inference. Capture and inference
refuse existing reserved artifacts. `--view-only` reopens the saved trajectory
without writing evidence; omit it only for the first evidence render. The
offline validator's optional `--write` creates `validation.json` exclusively.
An explicit `visual_review.json`, bound to the three screenshot hashes and all
six view checks, is required before the validator can return success. Missing
runtime evidence cannot be replaced by unit-test results.

For a fresh run, this equivalent launch/provenance helper supplies the required
`server_process.json`; run it from the research root after capture and before
the client, using the external Python environment. It reuses the client module's
read-only hashing helpers and does not import or modify LightNav source. Replace
`RUN` with the fresh captured directory. After the client completes, stop the
PID recorded here before launching Isaac visualization.

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  /home/gpuadmin/Workspace/external/LightNav-0-official-demo/.venv/bin/python - RUN <<'PY'
from pathlib import Path
from datetime import datetime, timezone
import json, os, runpy, subprocess, sys, time

r = Path(sys.argv[1]).resolve()
c = runpy.run_path('scripts/lightnav/robotless_single_frame_inference.py')
cfg = c['load_config'](r / 'config_snapshot.yaml')
checkout = c['resolve_path'](cfg['paths']['lightnav_checkout'])
checkpoint = c['resolve_path'](cfg['paths']['checkpoint_path'])
source = c['git_state'](checkout)
assert source['clean'] and source['git_sha'] == cfg['lightnav']['expected_git_sha']
manifest = c['checkpoint_manifest'](checkpoint)
c['verify_expected_checkpoint_hashes'](manifest, cfg['lightnav']['checkpoint_sha256'])
files = [{**f, 'size': f['bytes']} for f in manifest['files']]
for name in ('server_process.json', 'server_launch.json', 'server.ready', 'logs/server.log'):
    assert not (r / name).exists(), name
(r / 'logs').mkdir(exist_ok=True)
argv = [str(checkout / '.venv/bin/lightnav-serve'), '--task', 'vln',
        '--model_path', str(checkpoint), '--backend', 'vllm_local',
        '--host', '127.0.0.1', '--port', '8050', '--ready_file', str(r / 'server.ready')]
env = dict(os.environ, CUDA_VISIBLE_DEVICES='0', PYTHONUNBUFFERED='1')
with (r / 'logs/server.log').open('xb') as log:
    p = subprocess.Popen(argv, cwd=checkout, env=env, stdout=log,
                         stderr=subprocess.STDOUT, start_new_session=True)
state = dict(process_id=p.pid, argv=argv, command_cwd=str(checkout),
             start_utc=datetime.now(timezone.utc).isoformat(), stdout_log='logs/server.log',
             lightnav_git_sha=source['git_sha'], lightnav_checkout=str(checkout),
             checkpoint_identifier=cfg['lightnav']['checkpoint_identifier'],
             checkpoint_path=str(checkpoint), source_status_clean=True, checkpoint_files=files)
c['save_json_exclusive'](r / 'server_launch.json', state)
deadline = time.monotonic() + 180
while not (r / 'server.ready').exists():
    if p.poll() is not None or time.monotonic() > deadline:
        if p.poll() is None:
            p.terminate()
        raise RuntimeError('Server did not become ready; inspect logs/server.log')
    time.sleep(0.25)
assert p.poll() is None and '[lightnav-ws] READY' in (r / 'logs/server.log').read_text()
state.update(ready_confirmed=True, ready_observed_utc=datetime.now(timezone.utc).isoformat())
c['save_json_exclusive'](r / 'server_process.json', state)
print('Ready server PID:', p.pid)
PY
```

## Artifact map and implementation provenance

Verification completed: **630 repository tests passed** in 20.07 s with pytest
plugin autoload disabled and host socket access. This includes 118 new core,
client and artifact-validator cases, plus the 512 existing tests. The first
sandboxed full run had 628 passes and two existing IPC tests blocked by
`PermissionError` on Unix sockets; the host run resolved both without code
changes. `compileall`, both launcher syntax checks and `git diff --check` passed.
The final `--view-only --no-hold` Isaac smoke also passed. Pure tests use synthetic
inputs and mock protocol exchanges; the actual Isaac/server runs above are
separate runtime checks. No second real inference was used for testing.

| Artifact | Path within the run |
| --- | --- |
| Frozen configuration and observation | `config_snapshot.yaml`, `metadata.json`, `capture_validation.json` |
| Raw RGB | `raw/observation.jpg` |
| Exact client/server wires | `raw/lightnav_request.json`, `raw/lightnav_response.json`, `raw/lightnav_protocol.jsonl` |
| Original decoded local values | `raw/lightnav_waypoints.npy` |
| Derived world values | `derived/world_waypoints.npy` |
| Inference inputs, transforms, clocks and provenance | `inference_metadata.json` |
| Actual server argv/PID/checkpoint provenance | `server_launch.json`, `server_process.json`, `server_shutdown_requested.json` |
| Synthetic fixture views | `evidence/synthetic_frame_yaw_000.png`, `evidence/synthetic_frame_yaw_090.png` |
| Actual LightNav viewport | `evidence/lightnav_world_trajectory.png` |
| Runtime/visual audit and final decision | `visualization_validation.json`, `visual_review.json`, `validation.json` |
| Logs and environment | `logs/` |
| Exact source used for capture | `source_snapshot_capture/` |
| Exact source used for inference/initial visualization | `source_snapshot_inference_visualization/` |

The runtime ran from the starting Git SHA plus the new implementation under
review. Each phase records relevant source hashes; ignored source snapshots
preserve the exact research code used. Subsequent viewer-only and input-guard
changes do not rewrite the original capture, inference or visualization data.
No upstream source, model, dataset, RGB, screenshot, cache or virtual environment
is committed. The final implementation is the focused commit containing this
document; `git log -1 -- docs/ROBOTLESS_ISAAC_LIGHTNAV_SINGLE_CHUNK.md` identifies it.

## Limitations and retained warnings

This one observation validates the static interface, coordinate mapping and
visualization. It does not measure navigation success, scene performance,
instruction quality, collision safety, latency, or moving-agent behavior.
The logical camera height differs from the official demo. Render warm-up
updates do not create a model history: exactly one saved image enters the new
LightNav session. The model's own preprocessing and internal scheduler remain
unmodified upstream behavior.

Isaac logged a DLSS minimum-input-resolution warning, renderer readback
performance warnings, an initial missing render-variable warning, and a
shutdown plugin-release warning. The saved actual RGB and viewport PNGs were
nonblank and inspected. Official LightNav logged its existing processor/fast
and CUDA deprecations, seed/chunked-prefill notices and an NCCL teardown warning
after deliberate SIGTERM. No crash or failed inference was observed. Timing
fields present in the untouched server response are retained without a latency
analysis.

The Hospital asset URL and version family are recorded; the remote asset tree
is not vendored or content-pinned recursively. Reproduction depends on those
assets and the external installation remaining available. Source/checkpoint
identity is established from the locally launched process, command and hashes,
not a remote attestation feature of the WebSocket protocol.
