# JOIN-SOURCE-02: genuine obstacle-responsive LightNav source discovery

Question: with the same navigation goal, can the pinned official LightNav produce
a safe bypass after a newly visible obstacle, and can a frozen scenario reproduce
that source in three new online episodes? This task ends at source qualification.
No GP/rigid/reconciliation optimization, correspondence choice, duration sweep,
new tracker, model modification or next-stage execution is permitted.

Starting and freshly fetched `origin/main`:
`fb181dbf4553aa91b4c864f2c61c6fea71ba2cc4`. The two unrelated Stage-0 config edits
are preserved. JOIN-01 and ATTACH-01 results remain unchanged. Local output:
`data/robotless_join_source_02/source_20260921T112111Z/`.

## Facts, hypotheses and official contract

JOIN-01 preserved actual obstacle-containing triggering JPEGs and byte-identical
wire inputs. All three FRESH paths were unsafe; semantic masks failed, with only
BACKGROUND/UNLABELLED. The zero mask was not evidence of an obstacle-free input.
Six-to-seven delivered history frames and one pacing failure limit that source.
The straight instruction, primitive appearance, short history and decoder are
hypotheses, not established causes. Saved pointing changed in some attempts even
with identical action tokens; that neither proves recognition nor decoder fault.

The clean read-only external checkout remains
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`; checkpoint LightOriginsHQ/LightNav-0
revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`. Exact hashes, source contract
and server argv are saved locally. Task stays `vln -> vlnce -> vlnce_traj`;
the official Prompt Guide supports a target-object instruction in this task.
The internal prompt, RVQ decoder, weights and external MPC are unchanged.

The original camera sends 480x270 RGB JPEG quality95, HFOV112.2deg, .09m forward
and .65m high. Official preprocessing stretches to256x448, normalizes RGB and
uses full-episode SlowFast storage. Nominal64 is neither a minimum input count
nor a ring-buffer cap. Supplied frame indices/fps determine model temporal
positions, not physical LightNav waypoint times. There are no intrinsic output
waypoint timestamps. All actual N-by-3 arrays remain supported and immutable.

Temperature0/top_p1/top_k0/traj_top1=0 are explicitly frozen; the launcher’s
other inherited serving overrides are recorded/cleared before startup. A server
synthetic startup warmup is counted separately, never source evidence.
Top-level `visible` denotes target visibility; APOS stop is distinct from
top-level stop. Returned pointing coordinates are on the sent client image;
missing fields stay N/A. No undocumented internal reasoning is inferred.

## Technical preflight and source-only scene design

Two locations only were inspected in the actual Hospital renderer:

| ID | Initial pose x,y,yaw | Instruction | Verified target |
|---|---|---|---|
| location_01 |19.2,28,-pi/2|Go to the tall blue shelving unit and stop.|`SM_Shelf_06f7`,991 visible pixels|
| location_02 |19.2,24,-pi/2|Go to the double doors at the end of the hallway and stop.|`Geo_M_DobleDoor6`,200 visible pixels|

The target paths, world bounds and actual RGB/mask records precede instruction
freeze. The doors are distant; a pixel gate is not proof of model recognition.
No instruction supplies an avoidance side or an extra obstacle policy.

A separate internal reference of the existing `SM_SupplyCart_02a7` is the runtime
prop; its original asset remains untouched. Scale1, original mesh, short local-Y extent aligned forward for development, rigid world
relocation with recorded USD matrices. Instance-ID segmentation is attached to
the **same render product as RGB**, independently of semantic labels. Actual
off/on/off counts are0/14682/0, same pose/camera/time, matching only the runtime
root. This validates the instrument, not learned perception.

The exact8024-triangle cart is projected using the original [.05,.65]m slab and
solid/unknown-interior rules. Its obstacle area is .270833411m²; a bounding box
would add .351985296m². No box replaces the mesh. Radius .20m and required edge
clearance .05m are applied once in the original independent swept checker.
Render visibility and oracle occupancy switch together. Original Hospital
layers and validated export remain read-only; unknown space is never free.

## Frozen bounded development protocol

`configs/join_source_02.yaml` defines two location IDs, two geometry-only placement
fractions .35/.65 and H=16/32/64, at most12 conditions/36 terminal predictions.
Order is location, placement, H. First complete qualifying A/B/A' stops screening.
All failures, unavailable histories and placements remain in the ledger.

One normal official-LightNav/MPC approach bank per location captures chronological
live RGB at4Hz. Stop at80 captured frames, within2m of the fixed target, a source
STOP, sustained absence of approach movement, or oracle abort. Insufficient H is
HISTORY_UNAVAILABLE; no cloned/duplicated frame padding or continuing after goal
to manufacture history. Across H truncate the bank at its latest activated OLD observation (no future
frames), then use actual suffixes with that same terminal pose; changing H also resets session positions/SlowFast anchors,
so it is not a pure history-length causal comparison.

Placement uses the recorded active OLD future and current pose only, before
obstacle-on output. Its feasible arc begins after .8m/s*.60s design latency plus
prop forward half-extent+.25m; it ends before OLD endpoint minus the same
half-extent+.25m. The two fractions choose within that interval. A placement-only straight observation-to-first-row connector is explicitly
recorded if the observation precedes the first model row. It is not counted as
returned model geometry in qualification. No extrapolation beyond the final row
or outcome-driven placement tuning. Empty interval is PLACEMENT_UNAVAILABLE.
The latency estimate includes receipt/install/application allowance and is not
a worst-case guarantee. At least one lateral passage probe through actual mesh
oracle must remain valid. No unsafe source step is forced to obtain moving B.

Pair rendering is paused, same pose/camera/simulation time, actual Isaac RGB.
A and A' use the same unannotated off JPEG bytes in independent sessions;
B uses an actual on JPEG. Each branch restores identical preceding history
bytes/order via official buffer-only requests, then one terminal prediction.
Each has independent login/reset/seq/cache, no branch contamination or retry.
This is DEVELOPMENT / COUNTERFACTUAL INPUT SCREENING, not online source evidence.

Raw whole-row polylines receive independent swept checks: off/sham valid in off
geometry but blocked by the new prop; on safe through and beyond the prop’s rear
plane plus .25m, while maintaining target-directed progress. Meaningful response
requires >=.20m **interior** lateral separation or reliable >=20deg tangent/yaw
change in the frozen obstacle influence region (prop extent plus .50m).
Endpoint projection gaps do not count as lateral detours. Projection segment,
alpha and reliability are saved. Observation-to-first-row connector is separate
from model-returned geometry. No suffix deletion makes a path pass.

STOP, changed unsafe, no material response, safe partial detour and complete
safe bypass are distinct. Pointing changes and raw hashes are auxiliary.
The sham quantifies same-input variation descriptively, not population causality.

## Confirmation gate and safety

Only the first complete SAFE_BYPASS_CANDIDATE can trigger a separate confirmation
freeze commit/push. Then exactly three new online sessions, same scenario and
instruction, newly captured RGB and actual execution. No replay RGB substitutes
for live observations; no reset between OLD and first post-reveal FRESH.
Actual reveal placement follows the frozen OLD-based rule. Record the first
FRESH command application as B, distinct from observation/receipt/installation.
After that application retain .10s source post-roll, not a traversal benchmark.

An external CPU geometry worker reuses the original direct checker. Each next
held-command interval is checked with a circular-arc sagitta allowance for
obstacle and workspace. Before an unsafe integration, abort with a saved rejected
endpoint and **unapplied** attempted command. No avoidance, artificial stop,
pose teleport in execution records, FRESH modification or successful relabeling.
The historical collector remains unchanged; its original terminal bookkeeping
is retained alongside explicit acquisition termination/abort records.

Qualification requires actual v_minus>.20m/s, observation-to-B path>=.02m,
valid B/prefix, actual inference overlap, request-local RTF[.8,1.2], stall<=.25s,
genuine changed safe bypass and exact JPEG parity. Physical command and controller
memory stay separate. World poses always use the observation transform; never B.
All attempted episodes persist, with each failed predicate disclosed. Source
qualification does not certify B-to-FRESH feasibility or actual obstacle traversal.

## Reproduction and stop boundaries

Technical preflight uses the installed Isaac launcher, with ROS/Python/CUDA loader
environment variables cleared as in the existing collector. Model/server and MPC
stay in their existing separate environments; geometry stays in research .venv.
No environment upgrade or new large asset is required.

Pre-development code/config/protocol must be tested, reviewed, committed and
pushed before real history acquisition/screening. Exact commands and observed
results are appended after execution. If no complete qualifying development
scenario exists, confirmation and source bundle are explicitly unavailable.
No fallback, relaxed thresholds, extra condition or next experiment is allowed.
