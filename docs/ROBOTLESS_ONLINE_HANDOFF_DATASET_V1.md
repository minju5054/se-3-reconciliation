# Genuine online robotless LightNav handoff dataset v1

This collector acquires new Hospital RGB while a logical SE(2) agent executes
commands from the read-only official LightNav `MpcTracker`. A persistent official
LightNav server receives successive observations in one uninterrupted session per
episode. Each FRESH reference becomes OLD only when its first computed command is
actually applied. The saved execution stream, rather than a spatial surrogate,
defines the handoff boundary.

## Fixed execution and coordinate semantics

The external LightNav checkout is pinned to
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`, checkpoint
`LightOriginsHQ/LightNav-0` revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`. Its source and checkpoint files are
read-only. Isaac, the model server/client and the CPU MPC worker use their existing
separate environments. There are no robot meshes, articulations, wheels or contact
responses in the execution stack.

Agent state is world `[x,y,yaw]`, metres and radians, Z up, CCW yaw. Local x is
forward, local y left. An exact constant-command unicycle exponential integrates
`[v,omega]` at nominal 60 Hz. The resolved PhysX callback interval is float32
`1/60`; that value is recorded separately from the requested interval. Rotation
without translation consumes real simulation ticks. No state is snapped to a
waypoint, projection or newly installed reference.

The official MPC is imported from the external `mujoco_demo/vln_mujoco/mpc.py`
without copying or changing it. Its source SHA-256 is
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
After reference installation, its first solve waits for the next fixed control
tick; no off-grid solve is submitted.
Its fixed control rate is 10 Hz, horizon 5, dt 0.1 s, effective maximum v 0.8 m/s,
maximum |omega| 3 rad/s, acceleration limits 2 m/s² and 5 rad/s²,
Q `(10,10,1)` and R `(0.1,0.1)`. The module's `TRACK_V_MAX=1.5` constant is not
used by its actual solve. The controller uses forward Euler internally; the actual
60 Hz executor uses the exact unicycle exponential. IPOPT numerical tolerances
can produce command limits a few `1e-8` above their nominal values.

Reference selection finds the nearest pose using weighted XY and wrapped yaw,
then uses the next five rows, repeating the endpoint as necessary. Yaws are
sequentially unwrapped. No XY arc interpolation skips rotation-only poses.
Selection indices, reference poses, solve input states, generations and solve
start/end clocks are recorded. OLD commands remain applied while the new solve
is pending. A command is held for at most 0.5 simulation seconds without a new
accepted solve; timeout commands are zero and explicitly labelled. A stale solve
can never activate a reference. The upstream controller preserves its previous
command across path installation, so its acceleration limits can suppress command
jumps independently of path geometry.

Display annotations are cleared before every model RGB render and restored only after
the pixels have been copied. This prevents renderer-wide DebugDraw paths entering
model observations. The live GUI uses the last completed JPEG with its saved
state ID and timestamp.

Raw N×3 trajectories are cumulative local poses, anchored to their own actual
RGB capture-time agent pose with `T_W_waypoint = T_W_agent_capture T_agent_waypoint`.
They are neither incremental velocities nor a camera-optical-frame path. The
intrinsic model waypoint dt remains null. Camera configuration and actual USD
matrices record the 480×270 view, 112.2° horizontal FOV, agent-relative translation
`[0.09,0,0.65]` m and explicit USD camera-axis rotation. The USD matrix transpose
is recorded for its row-vector storage convention.

## Scheduling, clocks and history

Isaac `World.step(render=False)` advances PhysX once for each applied command.
The USD timeline is synchronized to that completed physics clock. Automatic
additional physics is suppressed during camera rendering and USD timeline commit, using the same
`/app/player/playSimulations` pattern as Isaac's own `SimulationContext.render`.
The timeline remains playing. Before/after capture checks require the actual
world time and logical pose to remain identical during RGB readback.

The camera samples every 15 integration ticks (nominal 4 Hz). The loop uses
absolute monotonic deadlines for target RTF 1. JPEG encoding and stream writes
run on bounded writer queues. No model inference, solver computation or plotting
blocks the execution loop. All captured JPEGs are saved once at quality 95, and
the exact bytes are transmitted. Queue overflow terminates with an explicit
technical status; unsent tail frames remain saved and indexed.

Four genuinely live frames bootstrap the session. The first three are buffer-only
`next` requests; the fourth predicts C0. Initial stationary frames are marked.
After each actual activation, the next prediction requires at least 0.50 simulation
seconds. During inference, later captures queue on the client. After receipt they
are uploaded chronologically, older frames buffer-only and the newest available
eligible frame as a prediction. A queued frame retains its original pose/time
anchor. One connection has one login, one explicit reset, unique seq numbers for
all `next` requests and at most one outstanding wire request. The official server
also initializes a newly created session internally; this is distinct from the
single explicit wire reset. No reset/reconnect occurs between predictions.

The checkpoint's nominal `num_history_frames=64` is **not a 64-frame ring**. VLN
uses full-episode SlowFast storage at configured video fps 4, 256×448 stretch
preprocessing, temporal patch size 2 and `do_sample_frames=False`. Source-pinned
SlowFast reconstruction records current, fast, mid, long and absolute anchor tiers.
Delivered history count, captured frame count, unique selected frame count and
selected slots including temporal padding are distinct. `history_full` means
actual delivered count at least nominal 64, not complete coverage of every tier.
Internal indices are reconstructed through the official sampler, not presented
as directly observed server telemetry. Server `actions.step` independently checks
the number of delivered frames. Model temporal positions use delivered-frame
indices divided by configured fps; irregular actual capture cadence is recorded
separately and can differ from that model convention.

Host UTC is provenance only. Client RTT uses host monotonic send and full-response
receipt. Observation, ready-seen, installation and command activation have separate
simulation timestamps. Remote server clocks are never subtracted from client
clocks. Exact client-inflight execution uses only saved state samples whose host
monotonic times lie between actual request and full response receipt. The
request-to-ready-seen range and receipt-to-loop-notice lag remain separate.

## Canonical handoff and immutable acquisition

Execution state i is the pre-integration state for command i. Command i produces
state i+1. B is the state at the first applied FRESH-reference command; P is the
preceding saved state. A reference identity change counts even when numeric
commands happen to match. Initial C0 activation is a bootstrap record and is not
counted as a normal handoff. Arrived but unactivated responses, STOP and invalid
outputs remain preserved with explicit reasons.

The collector labels an activated event moving when its fully enclosed client-
inflight state interval has translation path length above `1e-6` m or summed
absolute wrapped yaw change above `1e-6` rad. Otherwise it is stationary. The
separate incoming translational-direction diagnostic uses a `1e-9` m numerical
chord threshold and reports its exact pre-switch sample window. These thresholds
classify recorded kinematics; they are not a navigation or graph-needed score.

Each episode records `execution.csv`, `commands.csv`, `rgb_index.csv`, capture and
loop journals, exact wire messages, prediction metadata, raw local and derived
world arrays, history snapshots, controller records and handoff contexts.
`completion.json` hashes acquisition files. Resume verifies all completed hashes
and only executes previously unstarted episodes. Started incomplete directories
cannot be overwritten or silently retried.

The fixed primary schedule is 30 previous Hospital starting conditions × two
independent repetitions, in condition order then repeat 00/01. Only R0, instruction
and scenario labels are reused; every RGB and prediction is new. Each episode ends
at 40 handoff attempts or 60 simulation seconds after C0 activation, whichever
comes first, with earlier STOP/technical termination preserved. New predictions
cease in the final nominal second to allow post-roll; a pending response may still
arrive after an episode limit and is retained without activation. All 60 conditions
are attempted irrespective of accumulated counts or geometry. This is a
formulation-development corpus, not an independent held-out benchmark.

## Derived metrics, plots and verification

Every valid event receives 160 dpi world and boundary-zoom PNGs. Equal-aspect
world XY is unchanged: predicted OLD blue, raw FRESH magenta, actual past dark
gray, client-inflight execution green and actual FRESH execution orange. The
post-switch interval ends at the next actual activation boundary. Observation
poses, R_ready, P, B, projection Q and waypoint yaw arrows are distinct. Zoom
changes axis limits only. Hash-bound `plot_inputs.json` records source arrays,
stream ranges, exact displayed coordinates, units and plot settings.

Metrics preserve observation-to-ready/switch ages, client RTT, server-reported
stage times, RTF, actual inference translation/yaw, state update counts, command
changes and actual reference lifetimes. Incoming translational direction comes
from an explicit pre-switch execution window; zero translation makes it unavailable,
even during rotation. OLD tangent is an auxiliary diagnostic. Projection segment,
alpha, Q, progress, local tangent, 0.10 m window tangent and pose-yaw difference
are geometric measurements without a graph-needed threshold.

Every episode also receives executed-world, handoff overview, event timeline and
contact-sheet images. The HTML index links all event images and preserves invalid
attempts with diagnostic images or explicit reasons. Dataset aggregation separates
episode-weighted summaries from ordered raw-pair duplicate-aware summaries. Events
in a single successive episode are not treated as independent trials.

The validator reconstructs every unicycle step, capture anchor, raw/world transform,
history selection, exact wire JPEG payload, first FRESH command/B/P, OLD interval,
FRESH lifetime and plot input. Causal online validity is separate from real-time
pacing and nominal-history coverage. Live collection GUI is labelled
`LIVE ONLINE COLLECTION`. Separate saved playback is labelled
`RECORDED ONLINE EPISODE REPLAY` and uses recorded state samples without new
inference or fabricated integration.

## Commands

```bash
# Run a separate technical smoke before freezing the primary collector.
.venv/bin/python scripts/prepare_robotless_online_handoffs.py RUN_SMOKE --smoke
.venv/bin/python scripts/lightnav/robotless_online_server.py start RUN_SMOKE --timeout-s 300
bash scripts/launch_robotless_online_handoffs.sh RUN_SMOKE --gui
.venv/bin/python scripts/plot_robotless_online_handoffs.py RUN_SMOKE
.venv/bin/python scripts/validate_robotless_online_handoffs.py RUN_SMOKE --write

# After successful smoke, source review, tests and collector commit:
.venv/bin/python scripts/prepare_robotless_online_handoffs.py RUN_PRIMARY
bash scripts/launch_robotless_online_handoffs.sh RUN_PRIMARY
.venv/bin/python scripts/plot_robotless_online_handoffs.py RUN_PRIMARY
.venv/bin/python scripts/validate_robotless_online_handoffs.py RUN_PRIMARY --write
bash scripts/launch_robotless_online_replay.sh --run RUN_PRIMARY --episode EPISODE --verify --no-hold
.venv/bin/python scripts/lightnav/robotless_online_server.py stop RUN_SMOKE
```

The server remains warmed across all episodes. A later resume uses the identical
frozen run and skips only hash-verified completed episodes. Server launch/ready
provenance is linked when smoke and primary share the persistent process.

## Scope

Collision validity is unknown: there is no qualified wall-intersection gate or
physical collision response. This collection cannot establish physical robot
performance, navigation success/failure rate, collision safety, graph necessity,
rigid-versus-graph superiority, or natural deployment frequencies. No reconciliation
graph, correspondence factor, rigid correction, blending or optimization is added.

## Primary acquisition on 2026-09-15

Run: `data/robotless_online_handoffs_v1/primary_20260915T091900Z`.
Starting research SHA was `e56c0fe320309c2c4ffd8488fb3abdac42da8bfe`;
the collector was committed and pushed before acquisition as
`eed5f2c68f9bfc2b62d70a00a6d784647c32a971`. The final results commit is recorded in
the run's ignored `git_completion.json`. The frozen schedule SHA-256 is
`d5c3238100b13dfb25350b90b7ed13a70dd5d55e5545edf7cfe8dcc317a19081`, and the
resolved configuration SHA-256 is
`adc70b2f8d68b6228cda69b0a44e4de0c1753252601d6df50ef17e3fbf95e049`.
Acquisition source snapshots, protocol, configuration and server linkage are
preserved in the run. No collector, model, tracker, instruction or schedule change
was made during the primary acquisition.

All 60 planned episodes were attempted and sealed: 12 each for straight,
left-turn, right-turn, doorway and route-choice categories. The process exited 0
after `COLLECTION_SCHEDULE_FINISHED`. There were 59 normal model STOP terminals
and one 40-attempt-limit terminal. No episode was replaced or omitted. The target
of 500 events did not stop acquisition.

| Acquisition quantity | Actual count |
|---|---:|
| Planned / attempted / completed episodes | 60 / 60 / 60 |
| Initial C0 predictions, excluded from normal handoffs | 60 |
| Handoff prediction attempts | 940 |
| Activated valid handoffs | 881 |
| Moving / stationary handoffs | 880 / 1 |
| Non-activated MODEL_STOP attempts, all retained | 59 |
| Other model, protocol, controller or technical-invalid attempts | 0 |
| Total prediction requests, including C0 and STOP | 1,000 |
| Live camera frames saved | 4,639 |
| Frames delivered / saved but unsent at episode termination | 4,537 / 102 |
| Recorded states / applied command integrations | 69,125 / 69,065 |
| Valid events with at least 64 delivered history frames | 246 / 881 |

Saved unsent tail frames have explicit transmission status; they were neither
dropped from the archive nor appended to a later episode. Valid FRESH predictions
used 8–205 delivered frames (median 36); terminal session lengths were 28–213.
Thus the nominal 64-frame setting does not describe every inference's actual
history length. All histories include genuinely captured chronological frames,
and their server `actions.step`, exact transmitted JPEGs, capture poses and
source-reconstructed SlowFast selection are independently checked.
The reconstructed unique selected-frame and processor-slot counts range from
4 to 82 across all predictions, and 8 to 82 across valid FRESH predictions.
Every observed raw output, including STOP responses, has shape 10×3; support for
other N is covered by software tests rather than claimed as runtime diversity.

The persistent warmed bf16 official model server and Isaac ran together on the
RTX 5060 Ti 16 GB. A recorded primary snapshot reports 15,796 MiB GPU usage out
of 16,311 MiB, including 13,062 MiB for the model and 1,959 MiB for Isaac.
The requested `gpu_memory_utilization=0.55` is not a measured memory cap: the
official server explicitly reserves 2,457,600,000 bytes of KV cache and warns
that this overrides fraction-based cache profiling. No OOM occurred, and no
history reduction, quantization change, model reload or offline substitution was
used. The primary ran headless; the separately validated live GUI smoke used the
same final collector. The task-owned server was stopped after all 60 episodes.

## Actual overlap, activation and timing

Every valid event has 10–43 fully enclosed state updates during its exact client
request-to-full-response interval (median 22), 1–3 live captures, at least one
new official MPC solve command applied, and actual post-switch execution.
Using the collector's `1e-6` motion thresholds, 804 events have translational
motion and 76 have translation at or below the threshold but rotational motion.
The stationary event is `episode_000_repeat_01/handoff_020`: it still has 27
inflight updates, two live captures, five newly solved command applications and
1.3333 simulation seconds of post-switch execution. It was retained as an actual
reference handoff. A finer `1e-9` translation diagnostic counts 833 events; that
different numerical threshold does not replace the collector's status labels.
These are measured execution samples; no RTT-times-speed construction supplies
the paths. Every one of the 69,065 applied commands reconstructs its next saved
state with maximum pose error 0. All 10,740 accepted control submissions satisfy
the fixed 10 Hz grid. The validator checks the first FRESH command's generation,
solve and pre-integration B, preceding P and OLD command, and each subsequent
FRESH-to-OLD identity transition. It also checks that late old-generation results
are rejected and that post-switch ranges stop at the next activation.

The following are descriptive event distributions, not independent-trial
confidence estimates. Host RTT, simulation ages and matched-sample RTF are kept
in their own clock domains.

| Metric | Minimum | Median | Maximum |
|---|---:|---:|---:|
| Client RTT, host seconds | 0.2324 | 0.3801 | 0.8026 |
| Observation to ready-seen, simulation seconds | 0.2667 | 0.4667 | 0.9167 |
| Observation to actual switch, simulation seconds | 0.3667 | 0.5667 | 1.0833 |
| Ready-seen to actual switch, simulation seconds | 0.0167 | 0.1167 | 0.2167 |
| Receipt to loop notice, host seconds | 0.0043 | 0.0326 | 0.1653 |
| Exact client-inflight matched-sample RTF | 0.6092 | 1.0010 | 1.6248 |
| Inflight translation path length, metres | 0 | 0.2000 | 0.5733 |
| Inflight absolute accumulated yaw, radians | 0.000000709 | 0.000275 | 1.0796 |
| Actual execution under each FRESH, simulation seconds | 0.8333 | 1.1333 | 1.6833 |

The frozen pacing criterion is matched-sample RTF in `[0.8,1.2]` and no inflight
loop stall above 0.25 host seconds. It passes for 838/881 events (95.12%):
18 have lower RTF and 25 higher RTF. The largest measured inflight loop stall
is 0.2091 s. Whole-episode RTF is 0.9795–1.0000, which does not erase the local
pacing failures. Some missed deadlines occur even in intervals meeting the
aggregate pacing criterion; every event retains those counts.

Raw context `timing_flags` use the separately labelled request-seen to ready-seen
state interval and pass in 554/881 events. Here the retained label "request-seen"
starts at the first saved state at or after actual wire transmission; it does not
mean the later client-notice journal timestamp. The derived exact client-inflight
criterion passes in 838/881; 334 individual flags differ. Neither is overwritten
or treated as the other. Causal overlap and valid activation are established for
all 881 events, while the 43 local pacing limitations remain explicit.

## Diversity and interpretation

Among valid events, byte-identical raw-local array hashes identify 145 unique
OLD paths, 151 unique FRESH paths and 357 unique ordered OLD/FRESH pairs. The
most common pair occurs 178/881 times (20.20%). Every occurrence is retained with
its own observation anchors, history, execution and timing. Raw identity does not
make world-anchored paths or actual handoff states identical.

`aggregate/statistics.json` separates event-descriptive distributions, equally
weighted episode medians and equally weighted ordered-pair medians.
`aggregate/diversity.json` records pair membership and dominant frequency.
Repeated decoder outputs and within-episode dependence limit effective diversity;
this corpus does not estimate natural deployment frequencies. History-full
coverage is 246/881 (27.92%). Collision validity remains unknown throughout.

| Descriptive geometry median | All valid events | Equal-weight episode medians | Equal-weight raw-pair medians |
|---|---:|---:|---:|
| B-to-FRESH-polyline distance, m | 0.000432 | 0.000673 | 0.013855 |
| Absolute incoming-execution/local-FRESH direction difference, degrees | 0.0266 | 0.0257 | 2.6661 |

Direction is available for 779/881 events, all 60 episode groups and 304/357 raw
pair groups; unavailable translation directions are not replaced by yaw or zero.
These weighting differences describe the retained corpus and establish no
graph-needed threshold or physical navigation outcome.

## Images, GUI evidence and reproduction

Every valid event has both required 160 dpi PNGs: **1,762 expected and 1,762
generated**. All 59 STOP attempts also have two diagnostic PNGs, for 1,880 event
PNGs overall. Each of the 60 episodes has an executed-world overview with numbered
handoffs, an observation/request/ready/switch timeline and contact sheets. The
overall summary includes status, history, timing, motion and geometric distributions.
There are 60 overviews, 60 timelines and 99 contact sheets. All 3,041 image-index
links resolve. A scoped visual review directly inspected 19 images, including the
five preselected category pairs and stationary/STOP diagnostics; its exact list
and hashes are in `aggregate/visual_review.json`. Numerical validation covers all
events, whereas direct human-readable image inspection is explicitly sampled.

The root `index.html` links every event and episode image. Exact file mappings
are in `aggregate/plot_index.csv`; coordinates, input hashes, stream ranges and
rendered-image hashes are stored with each event's `plots/plot_inputs.json` and
`plots/plot_outputs.json`. Three disjoint episode shards produced the same frozen
plot functions' outputs, followed by one canonical whole-run aggregation pass.
The archived helper, its hash, commands, processing source hashes and execution
logs are in `logs/postprocessing/`; all heavy plotting occurred after collection.

Representative selection was fixed to the first valid event in schedule order
for each scenario category, without using residual magnitude:

| Category | Episode / event |
|---|---|
| Straight | `episode_000_repeat_00/handoff_000` |
| Left turn | `episode_006_repeat_00/handoff_000` |
| Right turn | `episode_012_repeat_00/handoff_000` |
| Doorway | `episode_018_repeat_00/handoff_000` |
| Route choice | `episode_024_repeat_00/handoff_000` |

For each row, the two images are under
`episodes/<episode>/handoffs/<event>/plots/trajectory_world.png` and
`trajectory_boundary_zoom.png`. These early events need not themselves exhibit a
turn; the category labels describe their frozen starting condition/instruction.
Stationary and STOP diagnostics are reviewed separately without replacing those
representatives.

Live GUI evidence comes from the final technical smoke:
`data/robotless_online_handoffs_v1/smoke_20260915_10/logs/live_gui_episode_000_repeat_00_1789463771733210541.png`.
It shows `LIVE ONLINE COLLECTION`, clean latest captured RGB, the blue active
reference and green actual execution after three genuine handoffs. The final
smoke has three valid moving handoffs, 313 states, 312 exact integrations, 21
captures and 6/6 event PNGs; its three local pacing flags fail and remain recorded.
The screenshot is an end-of-execution display check, not a screenshot taken
during inference. Its original asynchronous sidecar has a null screenshot field;
the completed PNG was independently hashed and visually checked, with the audit
preserved under the primary run's `logs/live_gui_smoke10_evidence.json`.

Primary recorded replay was actually launched for the first frozen episode,
`episode_000_repeat_00`, which has 21 valid handoffs and one retained STOP attempt.
Evidence is in `data/robotless_online_replay/primary_20260915T091900Z_replay01/`:
`replay_start.png`, `replay_handoff.png`, `replay_post_switch.png`,
`replay_overview.png`, `runtime_validation.json` and `visual_review.json`.
All four views were inspected with visible saved RGB and readable trajectory
geometry. The replay labels itself `RECORDED ONLINE EPISODE REPLAY`, uses saved
samples only, verifies unchanged source hashes, and exits after `--no-hold`.
The GUI is not left open and performs no new inference or execution.
The replay displays the raw context's request-seen-to-ready-seen timing flag.
For the pictured `handoff_000`, that flag is false while the derived exact
client-inflight flag is true (RTF 1.0148). This is the documented interval
distinction, not a changed event or a contradictory activation result.

Exact primary and replay commands, run from the repository root:

```bash
ONLINE_RUN=data/robotless_online_handoffs_v1/primary_20260915T091900Z
ONLINE_SERVER_RUN=data/robotless_online_handoffs_v1/smoke_20260915_01

# Server was started once for the retained technical smoke and kept warmed.
.venv/bin/python scripts/lightnav/robotless_online_server.py start "$ONLINE_SERVER_RUN" --timeout-s 300
.venv/bin/python scripts/prepare_robotless_online_handoffs.py "$ONLINE_RUN"
bash scripts/launch_robotless_online_handoffs.sh "$ONLINE_RUN" > "$ONLINE_RUN/collection.log" 2>&1

# Canonical equivalent of the archived disjoint plotting shards plus aggregation.
MPLCONFIGDIR=/tmp/online-handoff-mpl .venv/bin/python scripts/plot_robotless_online_handoffs.py "$ONLINE_RUN"
.venv/bin/python scripts/validate_robotless_online_handoffs.py "$ONLINE_RUN" --write

# Stop the identity-checked task-owned server after acquisition, before replay.
.venv/bin/python scripts/lightnav/robotless_online_server.py stop "$ONLINE_SERVER_RUN"
bash scripts/launch_robotless_online_replay.sh --run "$ONLINE_RUN" \
  --episode episode_000_repeat_00 --verify --no-hold \
  --output data/robotless_online_replay/primary_20260915T091900Z_replay01
```

These are the recorded run identities. Preparing a new acquisition or replay
requires a new output directory; existing raw runs are never overwritten.
The already collected run can be revalidated, and the plotter checks identical
existing derivatives. The precise commands used by the plotting shards are also
preserved in `logs/postprocessing/orchestration.json`.

## Final validation and status

The final complete validator exited 0 with `valid=true`, `causal_valid=true`,
`schedule_complete=true` and `errors=[]`. It validates all 60 episodes, 881 valid
handoffs and 1,762 required PNGs, including exact raw/world transforms, chronology,
command/state reconstruction, activation lineage and displayed coordinates.
The run and all 60 episode directories contain `validation.json`.

The root validation SHA-256 is
`a525bf52748573d13ceaaa71d31e763feacbe8fef362c27e98d07f4e4b10c1e2`.
The independently archived `logs/final_stream_audit.json` includes per-event
evidence, hash-linked completion manifests and exact audit recipes. Its SHA-256 is
`affd40756287c79bcb92ecbc72433138f5dbdd17e5ffbe78f9d1a9e78025f476`.

Full host pytest passed **1,286 tests in 44.44 s**. `compileall`, Bash syntax
checks for both launchers and `git diff --check` passed. Synthetic fixtures remain
software verification only. The final external audit rehashed all 16 checkpoint
files against the startup manifest and matched all 10 configured expected hashes;
the pinned external checkout and all 14 frozen collector source files are unchanged.
The two unrelated user-edited stage0 configurations were preserved exactly.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
.venv/bin/python -m compileall -q src scripts tests
bash -n scripts/launch_robotless_online_handoffs.sh scripts/launch_robotless_online_replay.sh
git diff --check
```

Final status: **`ROBOTLESS_ONLINE_HANDOFF_DATASET_COLLECTED_WITH_LIMITATIONS`**.
There is no remaining runtime or collection blocker. All required online causal
and image-coverage conditions are met, with 43 locally pacing-limited events and
635 valid events below nominal history-full length. Those events remain in the
dataset. Collision validity is unknown; repeated raw outputs and the prior
screening bank limit independence and generalization. Live GUI evidence is a
separate final technical smoke, while the full primary is headless. No physical
robot, navigation-success, collision-safety or reconciliation-method claim follows.
Generated raw data, images, external source, checkpoint and environments remain
outside Git; only implementation, tests, configuration and documentation are committed.

## Follow-up: straight motion and challenging geometry (2026-09-16)

The first event previously linked as an example is nearly straight. It was chosen
by episode/event order, not as a representative of the full difficulty range.
Analysis of all 881 valid FRESH events distinguishes raw chunk shape, actual
execution segments and time/distance spent executing individual commands.
These descriptive thresholds are new analysis settings, not original labels or
an assessment of navigation success, physical difficulty or graph necessity.

For the primary raw straight-ahead criterion, the saved waypoint XY path must
have arc length at least 0.05 m and chord/arc at least 0.995. Accumulated wrapped
pose-yaw change from capture-local yaw 0 through all waypoints must be at most
5°, and every XY segment longer than 0.1 mm must point within 5° of capture-local
forward +x. XY uses only the saved rows; no origin segment is added. A straight
line facing a new heading is therefore distinguishable from continuing straight
ahead. Model waypoint time remains unknown.

An actual FRESH execution segment extends from its activation B to the next
activation or termination, using its existing recorded metrics. It is classified
straight when it travels at least 0.05 m, chord/arc is at least 0.995 and accumulated
absolute yaw change is at most the selected angle. Raw and actual paths are not
substituted for each other.

| Angle tolerance | Raw straight-ahead chunks / 881 | Actual straight execution segments / 881 |
|---|---:|---:|
| 1° | 439 (49.83%) | 535 (60.73%) |
| 5° | 447 (50.74%) | 591 (67.08%) |
| 10° | 457 (51.87%) | 637 (72.30%) |
| 15° | 476 (54.03%) | 676 (76.73%) |

At 5°, equal weighting of the 60 episode-specific raw straight fractions gives
48.30%. Removing raw FRESH byte-identical repetitions leaves only 12/151 (7.95%)
distinct paths classified straight. The 447 straight occurrences thus contain
substantial repetition; unique-path weighting describes diversity, not occurrence
frequency. Twenty-six raw chunks have less than 5 cm of within-path XY motion
and are not called straight translation.

For execution-time composition, bootstrap commands with no active reference are
excluded; initial C0 and all subsequent active references are included. A tick
is straight translation when `|v| >= 0.05 m/s` and `|omega| <= 5 degrees/s`.
Turning translation exceeds that angular-speed threshold while translating;
low-translation rotation exceeds it with `|v| < 0.05 m/s`. Remaining ticks are
low motion. Time comes from adjacent saved simulation timestamps. Distance is
the sum of actual state-to-state XY displacements, not command-times-RTT.

| Active execution classification | Simulation time share | Actual XY distance share |
|---|---:|---:|
| Straight translation | 65.67% | 76.27% |
| Turning while translating | 21.29% | 23.69% |
| Rotation with low translation | 5.58% | 0.02% |
| Low motion | 7.47% | 0.02% |

Large rotations and larger handoff gaps exist. Raw accumulated yaw exceeds 30°
in 161/881 chunks (18.27%), 60° in 66 (7.49%) and 90° in 12 (1.36%). Actual
post-switch execution yaw travel exceeds those thresholds in 103, 33 and 8 events.
B-to-FRESH-polyline distance exceeds 10 cm in 138/881 events (15.66%) and 20 cm
in 17 (1.93%). These are descriptive measurements, not difficulty cutoffs.

Direction requires special care: the largest local-angle value, 164.37°, uses
an incoming chord of only 7.57 nm. All ten local-angle cases above 60° have a
selected raw segment no longer than 7.6 mm. They remain in the dataset but are
poor examples of large, well-supported direction changes. In the explicitly
separate subset with incoming chord at least 5 cm and window chord at least
2 cm, 12/689 events exceed 30° window direction difference and four exceed 60°.
No source metric or event was removed or relabelled.

Two visually inspected, timing-valid examples with substantial incoming motion:

| Episode / event | B-to-FRESH distance | Incoming/window direction difference | B/FRESH pose-yaw difference |
|---|---:|---:|---:|
| `episode_008_repeat_01/handoff_013` | 26.26 cm | 49.99° | 56.29° |
| `episode_013_repeat_01/handoff_024` | 27.91 cm | 36.54° | 36.19° |

Both have incoming execution chords about 18.6 cm, a 10 cm FRESH tangent window,
interior projection and a raw ordered pair occurring once. Existing world and
zoom PNGs are under the corresponding primary episode's `handoffs/<event>/plots/`.
For a large actual rotation, `episode_014_repeat_01/handoff_026` has raw accumulated
yaw 130.19° and actual post-switch yaw travel about 121.92° over 1.45 sim seconds.

The unchanged primary is the source. New derived outputs are separately stored at
`data/robotless_online_handoff_shape_analysis/primary_20260915T091900Z_v1/`:
`summary.json`, `events.json`, `provenance.json` and `geometry_tails.json`.
The provenance binds all inputs, processing source hash, frame conventions and
thresholds. `geometry_tails.json` is a supplementary audit of existing event
metrics with strict thresholds and raw-pair median grouping explicitly recorded
in that file. Reproduce the trajectory-mix outputs into a new output directory:

```bash
.venv/bin/python scripts/analyze_robotless_online_trajectory_mix.py \
  data/robotless_online_handoffs_v1/primary_20260915T091900Z \
  data/robotless_online_handoff_shape_analysis/NEW_OUTPUT_DIRECTORY
```
