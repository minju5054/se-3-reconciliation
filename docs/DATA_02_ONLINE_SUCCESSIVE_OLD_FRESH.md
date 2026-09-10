# DATA-02 Online-Successive OLD/FRESH v1

> This document freezes the original v1 protocol and result. It has not been reinterpreted or
> relabeled. The independent v2 cohort, read-only v1 timing diagnosis, and final reference-only
> v1+v2 assessment are documented in
> [DATA-02 v2 Extension and Final v1+v2 Assessment](DATA_02_V2_EXTENSION_AND_FINAL_SPLIT.md).
> The combined decision is `DATA02_COMBINED_DIVERSITY_INSUFFICIENT`, so EXP-02D remains
> unauthorized.

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

## Frozen primary result

The immutable primary run is
`data/data02_online_successive_v1/data02-online-successive-primary-v1/`. It was collected from a
separate clean worktree at collector commit
`254ab6f07ce69847b8612bc40082a28adf5f45bd`; `collector_git_status` is empty. The config and
protocol SHA-256 values are respectively
`b92f4ddc1ba7e80d5544573ffedd8b8175e6b92ae79c8d79637cbb3e17c5a313` and
`883099b35d0cec8724b08367057cd6b008b7d3eb3a96190a8ba36e14199e1868`.

All 12 predeclared templates and their seven variants were attempted:

| ID | Class | Physical region | Exact instruction |
|---|---|---|---|
| H00_STRAIGHT_SOUTH | straight | central_south_hall | Continue straight down the hospital hallway. |
| H01_STRAIGHT_NORTH | straight | central_mid_hall | Continue straight down the hospital hallway. |
| H02_LEFT_SOUTH_JUNCTION | left | south_cross_corridor | At the end of the hallway, turn left into the cross corridor. |
| H03_LEFT_NORTH_JUNCTION | left | north_cross_corridor | At the junction, turn left into the hospital hallway. |
| H04_RIGHT_NORTH_JUNCTION | right | north_cross_corridor | At the junction, turn right into the hospital hallway. |
| H05_RIGHT_SOUTH_JUNCTION | right | south_cross_corridor | At the junction, turn right into the cross corridor. |
| H06_DOORWAY_WEST | doorway | south_patient_room | Go forward through the open patient-room doorway. |
| H07_DOORWAY_CONTINUE | doorway | south_patient_room | Enter through the open patient-room doorway and continue toward the bed. |
| H08_DETOUR_LEFT | detour | warning_sign_south | Go around the warning sign on the left and continue forward. |
| H09_DETOUR_RIGHT | detour | warning_sign_south | Go around the warning sign on the right and continue forward. |
| H10_COMPOUND_SOUTH | compound | south_cross_corridor | Continue to the junction, turn left, then proceed down the cross corridor. |
| H11_COMPOUND_NORTH | compound | north_cross_corridor | Continue to the junction, turn right, then proceed down the cross corridor. |

The variant grid was V0, yaw offsets +0.05/-0.05 rad, lateral offsets +0.09/-0.09 m, and
longitudinal offsets +0.15/-0.15 m. The fixed plan was therefore 84 episodes and at most 504
transition opportunities. All 84 episodes completed collection. One model process served the
entire run without restart: one model build, 84 episode resets, and 564 predictions (84 initial
chunks plus 480 FRESH requests). Explicit episode termination produced 480 actual attempts.

The exact transition-status distribution is:

| Status | Count |
|---|---:|
| ELIGIBLE_MOVING | 248 |
| MODEL_STOP | 3 |
| OLD_EXHAUSTED | 6 |
| CHUNK_EXHAUSTED_BEFORE_TRIGGER | 0 |
| TIMING_INVALID | 219 |
| NONFINITE_MODEL_OUTPUT | 0 |
| EXECUTION_COLLISION | 4 |
| EXECUTION_OUT_OF_ENVELOPE | 0 |
| TECHNICAL_INVALID | 0 |

The 219 timing-invalid contexts and all terminal outcomes remain in the raw dataset; only the 248
eligible contexts enter timing-dependent characterization. Natural inference/real-time-factor
variation was not corrected by changing the frozen gate.

For eligible transitions, distributions below are min / median / p90 / p95 / max:

| Quantity | Distribution |
|---|---|
| Host latency [s] | 0.470697 / 0.569578 / 0.837373 / 0.844859 / 0.851770 |
| Model-reported latency [s] | 0.380114 / 0.440547 / 0.483625 / 0.490124 / 0.512790 |
| Effective latency `tau` [s] | 0.500000 / 0.600000 / 0.850000 / 0.850000 / 0.950000 |
| RTF [dimensionless] | 0.900087 / 0.932721 / 1.077281 / 1.081253 / 1.088596 |
| Translation during inference [m] | 0.076784 / 0.188265 / 0.269348 / 0.285378 / 0.330803 |
| Absolute yaw during inference [rad] | 0.000002 / 0.000348 / 0.047175 / 0.132404 / 0.342143 |

Raw identity counts were 45 unique OLD chunks, 59 unique FRESH chunks, and 96 unique ordered raw
pairs. The largest exact raw pair occurred 93 times, or 37.5% of eligible transitions. World
identity counts were 238 OLD, 241 FRESH, and 241 ordered pairs. World uniqueness is kept separate
because the same raw output anchored at different observations is not new model-output diversity.

FRESH spatial geometry counts were 169 `STRAIGHT_LIKE`, 39 `POSITIVE_TURNING`, 34
`NEGATIVE_TURNING`, and 6 `OTHER`. Representative continuous distributions are:

| Untimed FRESH descriptor | Min / median / p90 / p95 / max |
|---|---|
| Path length [m] | 0.069281 / 1.355959 / 1.356219 / 1.356219 / 1.364536 |
| Endpoint lateral [m] | -0.513864 / 0.000024 / 0.202798 / 0.498392 / 0.770318 |
| Signed net yaw [rad] | -1.015159 / 0.000607 / 0.586948 / 0.905325 / 1.252415 |
| Cumulative absolute pose yaw [rad] | 0.000025 / 0.003597 / 0.841081 / 0.913693 / 1.252415 |
| Maximum absolute pose-yaw increment [rad] | 0.000007 / 0.000892 / 0.146627 / 0.166033 / 0.309027 |
| Maximum lateral excursion [m] | 0.000007 / 0.000317 / 0.424120 / 0.498392 / 0.770318 |

These are spatial descriptors of untimed waypoint rows, not time-aligned motion metrics. The
difficulty distribution was 80 `BENIGN`, 123 `INTERMEDIATE`, and 45 `CHALLENGING`. Absolute
desired-command jumps were:

| Quantity | Min / median / p90 / p95 / max |
|---|---|
| `abs(delta_v_des)` [m/s] | 0.000000 / 0.059809 / 0.228359 / 0.362339 / 0.473801 |
| `abs(delta_omega_des)` [rad/s] | 0.000026 / 0.011407 / 0.491180 / 1.363820 / 1.698676 |

The isolated splitter found 13 episode/raw-pair connected components. It assigned 213 eligible
transitions to development and 35 to held-out (14.11%, versus the 25% target), with zero episode
leakage and zero exact ordered-raw-pair leakage. Development contains all 12 scenarios; held-out
contains H02, H03, H05, H06, H08, H09, and H10. Difficulty classes occur in both splits, but
held-out has no `OTHER` geometry, only seven scenario families, and fewer than the required 60
eligible contexts. Split feasibility therefore fails rather than relaxing isolation.

The predeclared A-H readiness checks were: A sample size fail (248 < 300), B raw-pair diversity
pass (96 >= 75), C duplicate domination fail (37.5% > 20%), D chunk diversity pass (45 OLD and 59
FRESH), E geometry coverage pass, F difficulty coverage pass, G isolated-split feasibility fail,
and H strict artifact validity pass. The exact final decision is therefore:

`DATA02_COLLECTED_BUT_DIVERSITY_INSUFFICIENT`

This result does not authorize EXP-02D formulation development. It also does not justify changing
the observed outputs, thresholds, controller, timing gate, or scenario weights after inspection.

Strict validation independently reconstructed all 84 frame streams, raw-to-world transforms,
observation anchors, timing/order relations, P/B samples, successive chunk chains, and hashes for
all 480 transitions with zero errors. The generated run occupies about 857 MiB and remains ignored
by Git. All ten required figures are under the run's `plots/` directory:
`eligible_transition_counts.png`, `latency_distribution.png`,
`robot_motion_during_inference.png`, `command_jump_distribution.png`,
`fresh_geometry_distribution.png`, `raw_pair_frequency.png`,
`scenario_geometry_matrix.png`, `benign_challenging_matrix.png`,
`development_heldout_summary.png`, and `representative_transition_overview.png`.

The deterministic representative set contains 30 transition IDs in
`summary/representatives.json`. A real saved-only GUI smoke used representative
`episode_000014_transition_04`: the official Jackal visibly traversed the recorded history in
Hospital; OLD/FRESH/actual curves and observation/P/B markers were visible; the phase changed from
`ACTIVE_OLD` through `FRESH_IN_FLIGHT` to `FINISHED`; and a non-black capture was saved to
`gui_replays/episode_000014_transition_04.png`. No inference or physics re-execution occurred in
that replay.

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
  --episode episode_000014 --transition 4 --gui
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
