# DATA-02 Online-Successive OLD/FRESH v1

## Scope and research question

DATA-02 asks whether the actual online LightNav + wheel-driven Isaac Jackal system produces
enough technically valid and geometrically diverse same-episode successive transition contexts
for later SE(2) reconciliation formulation research. It is a data-collection and diagnostic stage,
not an algorithm improvement or reconciliation evaluation. It implements no graph, residual,
correspondence, selector, smoothing, blending, collision factor, obstacle avoidance, controller
tuning, physics tuning, or LightNav modification.

The Stage 0-G, G2, and G3 qualification failures remain unchanged. G3 established that moving
visual history changes some outputs, but did not pass the frozen directional qualification. This
research decision proceeds because the next uncertainty requires the target data itself: genuine
successive chunks while the physical robot keeps executing OLD during natural FRESH latency.

## Frozen system and provenance

- Isaac Sim 6.0.1, official Hospital
  `/Isaac/Environments/Hospital/hospital.usd`, and official Jackal
  `/Isaac/Robots/Clearpath/Jackal/jackal.usd`.
- LightNav checkout `a645828d81a8439651172197ca80a75dc1377977`, checkpoint
  `LightOriginsHQ/LightNav-0` revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`.
  Important checkpoint hashes are checked before Isaac collection.
- Stage 0-G2/G3 camera contract: 480 x 270 HWC uint8 RGB, 112-degree HFOV, level
  robot-forward mount `[0.30, 0.0, 0.48]` m; model preprocessing remains upstream stretch to
  448 x 256 bilinear.
- One persistent LightNav process is built and warmed once per collection invocation. It uses the
  already coexistence-tested 0.65 GPU-memory utilization setting and 1 GiB KV cache. It resets
  once at each episode boundary and never between chunks in an episode.
- The execution stack is the existing `TrajectoryFollower`, frozen Stage 0-D `pi_strong`
  correction, and Isaac `DifferentialController`. All `pi_strong` artifact and runtime-code hashes
  are checked. Stage 0-F's exact limitation is retained: its LightNav workload was covered by the
  current calibrated platform, while `execution_platform_validated` remained false.

`configs/data02_online_successive_v1.yaml` is the scientific protocol. The primary run records
its config hash and the exact collector commit. Generated runs live under the Git-ignored
`data/data02_online_successive_v1/<run_id>/`.

## Episode bank and successive protocol

Twelve Hospital templates were declared before primary inference: approximately two each for
straight, left, right, doorway, detour, and compound instructions, across several Hospital
regions. Each records its initial pose, physical region, visible affordance, expected qualitative
route, intended clearance, and lossless RGB preview. A contact sheet and all 12 x 7 initial-pose
feasibility checks are stored in `template_bank/manifest.json`. The seven variants are V0,
yaw +/-0.05 rad, lateral +/-0.09 m, and longitudinal +/-0.15 m. They are fixed before primary
outputs are seen.

At an episode start, Isaac/Jackal and the LightNav history are reset, the robot settles, and 64
validated frames are captured at 4 Hz. LightNav produces initial `chunk_00`, which becomes OLD.
Exactly 0.50 simulation seconds after each active chunk begins, the current lossless RGB frame is
delivered and one asynchronous FRESH request starts. Jackal continues executing OLD through the
frozen wheel stack. Frames captured while prediction owns the single-flight IPC connection are
queued and delivered in timestamp order immediately after the response, so the persistent agent
receives the entire episode stream before the next request.

At the next controller boundary, raw observation-anchored FRESH is activated unchanged. Its
controller reference is not re-anchored at B, B is not prepended, and no row is deleted or
smoothed. A new follower is evaluated at the actual switch boundary. `FRESH_i` then becomes
`OLD_(i+1)` in the same episode state. The low-level PI state remains part of the continuous
physical execution and is reset only at the next episode boundary.

Each primary episode attempts at most six transitions. The complete frozen grid is 12 templates x
7 variants = 84 episodes and at most 504 transition opportunities. STOP, OLD exhaustion,
collision, instability/out-of-envelope, timeout, and technical failures terminate an episode and
remain recorded; they are never replaced with hand-picked episodes.

## Time and coordinate semantics

LightNav returns arbitrary `N x 3 [forward_m, lateral_m_left_positive, yaw_rad_ccw_positive]`
waypoints. The current checkpoint normally returns N=10, but storage and pure helpers accept any
non-empty N. The rows have **no intrinsic timestamp**. `waypoint_dt` is always null. Geometry is
therefore spatial and must not be described as time-aligned waypoint tracking.

Local output is transformed exactly once using the robot world pose at the final FRESH input
frame. World coordinates are Isaac x/y metres with yaw CCW about +Z. Every chunk stores raw,
local, and observation-anchored world arrays separately and records the transform anchor.

Per transition, the clock domains remain explicit:

- `t_obs_sim_s`: final RGB input simulation time;
- `t_request_host_monotonic_ns`: IPC request start;
- `t_model_ready_host_monotonic_ns`: response availability observed by Isaac;
- `t_ready_sim_s`: corresponding simulation sample;
- `t_switch_sim_s`: actual controller activation boundary;
- host request/response, upstream model-reported latency, simulation observation-to-ready,
  ready-to-switch delay, and effective `tau = t_switch - t_obs` are separate;
- RTF is simulation observation-to-ready duration divided by host request/response duration and
  must lie in the frozen `[0.90, 1.10]` gate for timing eligibility.

P is the measured pose one controller interval before switching. B is measured at the switch.
The model-ready pose is also stored, so motion during inference (`observation -> model-ready`)
is not conflated with motion over effective latency (`observation -> B`). Neither P nor B is
substituted into the raw FRESH path.

## Artifacts and telemetry

Each episode owns one lossless `rgb/` stream and one `rgb_frames.csv`; RGB is not duplicated under
transitions. The index stores frame number, simulation timestamp, robot pose, server progress, file
path, and SHA-256. `chunks/chunk_XX/` stores the immutable raw LightNav NPY, raw decoder text,
derived local/world paths, observation anchor/time/frame, request metadata, and hashes.
Run-level `protocol.json` freezes architecture, timing, coordinates, statuses, templates, and
readiness gates. `collection_manifest.json` hashes every episode index/control stream plus all chunk
and transition metadata; the strict validator reconstructs observation frames, transforms, clock
ordering, P/B controller sampling, model-ready pose, and the successive chunk chain.

The episode `telemetry.csv` and each transition's bounded telemetry contain simulation time,
measured SE(2), finite-difference body velocity, follower desired v/omega, calibrated executed
v/omega, four canonical wheel targets and four measured wheel velocities, follower nearest/target
indices, goal state, active chunk ID, inference-in-flight state, collision state, and RGB index.
Nearest-reference metrics, if added downstream, are spatial only.

Every transition has exactly one status:

- `ELIGIBLE_MOVING`
- `MODEL_STOP`
- `OLD_EXHAUSTED`
- `CHUNK_EXHAUSTED_BEFORE_TRIGGER`
- `TIMING_INVALID`
- `NONFINITE_MODEL_OUTPUT`
- `EXECUTION_COLLISION`
- `EXECUTION_OUT_OF_ENVELOPE`
- `TECHNICAL_INVALID`

Its `transition.json` identifies canonical chunk requests; raw arrays are also copied into its
strict self-contained context with file hashes. It records observation, P, B, all timing values,
raw and world ordered-pair identities, untimed OLD/FRESH geometry, B-to-FRESH[0], P-to-B, and the
actual command jump. The first FRESH command is the real first desired command from the frozen
follower evaluated at B, not an interpretation of row 0.

## Characterization, split, and readiness

FRESH geometry bins and BENIGN/INTERMEDIATE/CHALLENGING command-jump bins use the thresholds in
the frozen task/config. They are descriptive coverage labels, never reconciliation gates. Exact
raw chunk and ordered OLD/FRESH-pair hashes are reported separately from world identities.

After strict validation, the splitter constructs bipartite connected components between episode
IDs and exact ordered raw-pair IDs. Components, never individual transitions, enter approximately
75/25 development/held-out sets. Thus neither an episode nor an exact raw pair leaks. Split
feasibility also requires meaningful template coverage and at least 60 held-out eligible samples.

The final status is `DATA02_READY_FOR_FORMULATION_RESEARCH` only when all predeclared A-H gates
pass: 300 eligible, 75 unique pairs, largest pair <=20%, 25 unique OLD and FRESH, required geometry
and difficulty counts, a valid isolated split, and strict artifact validity. Otherwise the exact
status is `DATA02_COLLECTED_BUT_DIVERSITY_INSUFFICIENT`, `DATA02_TECHNICAL_INVALID`, or
`DATA02_EPISODE_TEMPLATE_INSUFFICIENT`. Gates are not changed after results.

## Commands

Technical smoke (one episode, up to three transitions):

```bash
./scripts/isaac/run_data02_online_successive.sh \
  --phase smoke --run-id data02-online-successive-smoke-r6
```

Primary collection from the clean frozen collector commit:

```bash
./scripts/isaac/run_data02_online_successive.sh \
  --phase primary --run-id data02-online-successive-primary-v1
```

Append-safe resume skips every already-attempted episode directory and writes only never-attempted
episodes:

```bash
./scripts/isaac/run_data02_online_successive.sh \
  --phase primary --run-id data02-online-successive-primary-v1 --resume
```

Validation, summary/split, and plots:

```bash
.venv/bin/python scripts/validate_data02_online_successive.py \
  data/data02_online_successive_v1/data02-online-successive-primary-v1
.venv/bin/python scripts/summarize_data02_online_successive.py \
  data/data02_online_successive_v1/data02-online-successive-primary-v1
.venv/bin/python scripts/plot_data02_online_successive.py \
  data/data02_online_successive_v1/data02-online-successive-primary-v1
```

Saved-only GUI replay performs no model inference:

```bash
./scripts/isaac/run_data02_online_successive.sh \
  --replay-run data/data02_online_successive_v1/data02-online-successive-primary-v1 \
  --episode episode_000012 --transition 3 --gui
```

GUI legend: blue is OLD, magenta is raw observation-anchored FRESH, green is recorded Jackal
actual history, yellow is FRESH observation, orange is P, red is B/switch, and the official Jackal
mesh replays the saved physical motion. A GUI panel and terminal output show the current saved
phase, sample, simulation time, and Jackal SE(2). A low chase camera and replay-only light keep the
Jackal visible inside Hospital; neither affects collected pixels or physics. This is qualitative
inspection only and never reruns LightNav.

## Frozen-collector technical smoke

Ignored run `data02-online-successive-smoke-r6` exercised one H00/V0 episode and three successive
opportunities with one warmed model build, one episode reset, four predictions (bootstrap plus
three FRESH), and no OOM. All three contexts were immutable and strictly reconstructable. Two were
`ELIGIBLE_MOVING`; one was retained as `TIMING_INVALID` because its natural RTF fell below the
frozen 0.90 lower bound. For the eligible contexts, host latency was 0.556--0.827 s, effective
latency was 0.600--0.850 s, and Jackal moved 0.176--0.258 m before model readiness (0.200--0.293 m
by B). These are technical-smoke observations, not primary dataset evidence and not a basis for
changing templates, timing gates, trigger cadence, or controller settings.

Saved-only GUI smoke on `episode_000000`, transition 1 visibly showed the illuminated official
Jackal moving through Hospital, OLD/FRESH paths, actual history, observation/P/B markers, and a
live phase panel. Its viewport capture is under that ignored run's `gui_replays/` directory.

## Claim limitation

DATA-02 can establish dataset integrity, timing, actual motion, duplicate structure, and
descriptive coverage. It cannot establish that an instruction was semantically satisfied, prove
general navigation quality, identify causal tire/contact effects, or show that any future
reconciliation formulation works. Collisions and poor trajectories are upstream/execution
observations retained in the dataset, not evidence to tune this frozen collector.
