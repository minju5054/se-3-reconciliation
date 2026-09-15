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
Reference installation waits for the next fixed control tick; no off-grid solve is submitted.
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

Actual run counts, smoke findings and final validation are recorded below after
acquisition; implementation and synthetic tests alone are not runtime evidence.
