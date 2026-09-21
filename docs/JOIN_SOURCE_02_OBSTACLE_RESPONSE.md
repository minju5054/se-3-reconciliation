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

## Observed development result — no qualifying scenario

Execution freeze **3e082228f666ba2a0aad758a7de37693c54cb420** was committed and
pushed before the server/history calls. The run ended as
**NO_QUALIFYING_DEVELOPMENT_SCENARIO**. The exact ordered ledger contains all12
conditions, including unavailable conditions. Only two complete A/B/A' conditions
were executable; both are CHANGED_BUT_UNSAFE. No condition was replaced, no
threshold changed and no second collection bank was attempted.

|Location / placement|H16|H32|H64|
|---|---|---|---|
|location_01 / placement_01|CHANGED_BUT_UNSAFE,3 predictions|HISTORY_UNAVAILABLE|HISTORY_UNAVAILABLE|
|location_01 / placement_02|CHANGED_BUT_UNSAFE,3 predictions|HISTORY_UNAVAILABLE|HISTORY_UNAVAILABLE|
|location_02 / placement_01|PLACEMENT_UNAVAILABLE|PLACEMENT_UNAVAILABLE|HISTORY_UNAVAILABLE|
|location_02 / placement_02|PLACEMENT_UNAVAILABLE|PLACEMENT_UNAVAILABLE|HISTORY_UNAVAILABLE|

The two location01 conditions share the same15 preceding genuine history JPEGs
and final observation pose `[19.2046215225,23.6233363112,-1.56856363484]`.
Each branch has16 delivered frames and a separate official session. A' reuses
the exact off JPEG. All six actual terminal wire hashes match the stored triggering
JPEGs. Paused render simulation time is8.050000420s; this is not an application
boundary B, and no source prediction was executed from this paused pose.

|Quantity|placement_01|placement_02|
|---|---:|---:|
|Prop center world X,Y [m]|19.207161,22.640311|19.207224,22.618764|
|Observation-to-center design arc [m]|0.983029|1.004575|
|Obstacle pixels, off/on|0 /15779|0 /15194|
|A/A' clearance in obstacle-off world [m]|0.935417|0.935417|
|A/A' clearance in obstacle-on world [m]|-0.200000|-0.200000|
|B clearance in obstacle-on world [m]|-0.200000|-0.200000|
|Maximum interior lateral difference [m]|0.006775|0.006775|
|Maximum reliable tangent difference [deg]|16.411117|16.411117|
|Maximum reliable interpolated yaw difference [deg]|31.046266|31.046266|
|Off/sham raw array identical|yes|yes|
|Off/on raw array different|yes|yes|
|On endpoint beyond rear qualification plane [m]|-0.487315|-0.508862|
|Target-directed distance reduction [m]|0.762572|0.762572|
|Whole raw on path safe / bypass|no / no|no / no|

The two geometry-only placements are only2.15cm apart because the frozen latency
reserve and finite OLD horizon leave a narrow feasible interval. They are not
strongly diverse obstacle scenarios or independent evidence of broad behavior.
Both side-passage probes pass: minimum edge margins approximately.073760m and
.068399m. Those probes demonstrate local free space, not a generated bypass or
complete executable route. Actual cart mesh projection, static layer hashes,
render/occupancy activation and the same-camera instance masks pass.

A/A' returns an approximately straight1.356219m polyline (first-to-last row).
B returns a shorter.848790m polyline, with last yaw about-0.596717rad relative
to observation. Its center path still reaches/overlaps the cart. Meaningful
response passes the frozen yaw criterion, **not** the20cm lateral criterion.
A shorter output or changed yaw is not a safe detour. Physical overlap is a
geometric property of the predicted reference, not an observed executed collision.
No unsafe rows were trimmed, and no observation-to-first-row connector was counted
as a model-returned segment.

### Pointing and same-input sham

|Condition / branch|APOS pixel|APOS clamped|OPOS|target visible|top-level STOP|
|---|---|---|---|---|---|
|Both A and A'|235,165|false|not_visible|false|false|
|placement_01 B|245,265|true|not_visible|false|false|
|placement_02 B|235,265|true|not_visible|false|false|

Both on outputs have the same action array although their APOS differs by10px.
Both off/sham arrays and action tokens are identical. Off/on action tokens also
change; this is not an instance of a changed pointing token with unchanged action
output. Raw text and fields remain stored separately. None of these observations
identifies a decoder/perception failure. Target visibility is false in all six
responses; initial renderer target verification does not establish recognition or
continued visibility at the later screening pose. All positions are480x270 client
image coordinates; diagnostic overlays were never model inputs.

### Authentic-history coverage and acquisition abort

Location01 acquired33 live frames and ended within the declared2m target-approach
boundary. The latest activated OLD observation leaves28 usable causal prefix
frames; frames after that observation are preserved but excluded. H32/H64 are
therefore unavailable. The endpoint was not extended to accumulate stationary
frames. Its original collector status EPISODE_LIMIT is preserved alongside the
more specific TARGET_APPROACH_BOUNDARY reason.

Location02 acquired59 live frames,56 before/at its latest active OLD observation.
The oracle abort stopped the next unsafe held-command interval after871 completed
steps. Last recorded edge clearance is.050146128m; the rejected endpoint is
.047051410m (curve-allowance lower bound.047050160m). No physical overlap occurred.
One trailing attempted command is explicitly **unapplied**; it is not appended to
the executed trajectory. The historical collector's TECHNICAL_INVALID status is
preserved and distinguished from this expected safety abort. Available H16/H32
could not be used because the remaining OLD horizon failed the declared placement
reserve; H64 was also unavailable. The task does not keep advancing unsafe OLD to
manufacture history or a moving boundary.

## Confirmation and source-bundle outcome

|Episode|Attempted|Qualified|OLD/FRESH, moving B, timing|Reason|
|---|---|---|---|---|
|episode_01|no|N/A|N/A|development gate failed|
|episode_02|no|N/A|N/A|development gate failed|
|episode_03|no|N/A|N/A|development gate failed|

There are **zero acquired qualifying online sources**, with **0/3 planned
confirmations attempted**. This is not three failed online episodes or a measured
0% online success rate. No confirmation configuration was selected/frozen, and no
optimization-ready source bundle exists. B, physical u_minus, observation-to-B
motion and online reveal/request/receipt/application timeline are N/A for these
unrun confirmations. The history bank's ordinary handoffs are not substituted for
obstacle-reveal confirmation. GP/rigid/reconciliation calls remain0.

## Timing, evidence and limits

Development terminal RTTs (seconds), measured entirely in host monotonic time:

|Condition|A off|B on|A' sham|
|---|---:|---:|---:|
|placement_01 H16|0.316349|0.310251|0.311140|
|placement_02 H16|0.308469|0.309535|0.307988|

The two paused obstacle-on captures occurred at11:47:26.166774 and
11:47:27.606436 UTC; terminal requests occur later at11:48:09.510150 and
11:48:15.453689 UTC after independent history restoration. That replay delay is
not inference latency or moving online staleness. Full monotonic request/receipt
records are in aggregate/development_timing.json; no application timestamp is
invented for development. Frozen simulation time and host time are never subtracted.

Authoritative local evidence root:
`data/robotless_join_source_02/source_20260921T112111Z/`.
The actual RGB, masks, world trajectories, every unavailable condition, separate
pointing overlays and compact results are in `review_v2/index.html` and
`review_bundle_final.zip`, with numerical/source hash sidecars. Original unannotated
RGB and raw responses remain separate from derived geometry/figures. There is no
new GUI framework, online obstacle traversal claim or reconciliation comparison.

The main uncertainty remains whether a genuine obstacle-sensitive scene/history
can produce a **safe bypass future and moving online application B together**.
This bounded run shows changed-but-unsafe output in two closely spaced H16
conditions; it does not distinguish instruction, target recognition, history,
appearance, local horizon or model/decoder mechanisms. No follow-up tuning or
optimization is implemented here.

### Actual calls and compute

|Scope|Prediction requests|Buffer-only|Official MPC|Wall measure|
|---|---:|---:|---|---|
|Live history, location01|8|24|70 submitted|8.507344s host episode window|
|Live history, location02|14|45|135 submitted|14.945203s host episode window|
|Paired development|6|90|0|12.554s runner elapsed|
|Server startup warmup (not evidence)|1|0|0|0.272s model warmup|
|Online confirmation|0|0|0|N/A|
|Two repository historical-reproduction tests|0|0|2 separate test calls|not primary evidence|

History MPC totals are205 submitted attempts and204 recorded completions;
location02 final solve_000134 has no saved completion. Do not replace the unknown
with success or zero time. Sum of observed official solve times is.802302s.
Location01 final solve_000069 submit/result appears in location02's controller
journal; counts use the records' own episode/solve identity. Sixteen stale results
were rejected by the unchanged policy; no recorded controller error is hidden.
A late location01 chunk_007 was received but never activated.

There were92 actual captures,91 deliveries,1352 completed integration steps and
one separately recorded unapplied command. History model RTT sum7.146927s and
server latency sum7.041935s are overlapping measures, not additive costs.
Server startup-to-ready17.011692s; startup-to-exit148.203611s. Server logs cross-check
29 model prediction items:1 warmup+22 history+6 terminal;159 buffer-only requests
are not additional predictions. GP/rigid/reconciliation optimizations0.

### Saved-record validation and presentation correction

Final full tests: **2508 passed,19 skipped,171.95s**; the earlier pre-development
suite was2500 passed/19 skipped. Historical-MPC tests are explicitly separate
from source calls. Compileall and diff checks pass. The new saved-record validator
checks the complete ledger, source hashes, independent whole-path geometry,
paired byte/history/transform parity and figure numbers without inference or
MPC. Scientific rejection does not fail artifact validation.

The authoritative supplementary geometry audit is
`technical_preflight/post_execution_geometry_audit_v2.json` (58/58 checks pass).
Its v1 is preserved: the audit initially used ideal1/60 while the historical
collector explicitly records float32-resolved dt0.01666666753590107s. V2 uses
that recorded dt, reproducing all1352 executed updates literally with zero error;
no integration, acceptance threshold or source result was changed. Location02's
abort obstacle is the original Hospital `SM_MopSet_01b2`, not the runtime cart.

The first review is preserved. A separate reporting-only layout wrapper creates
**`review_v2/index.html` and `review_bundle_v2.zip`**, moving the world legend outside
the data axes to reveal the unchanged off/sham endpoints. The final distribution `review_bundle_final.zip` adds protocol, count and
authoritative validation records to that same presentation; no model/source/geometry
rerun occurred. Every PNG carries a
numeric/hash sidecar, and original frozen reporting code remains unchanged.

### Exact commands

From the repository root, the actual sequence was:

```bash
RUN=data/robotless_join_source_02/source_20260921T112111Z
.venv/bin/python scripts/run_join_source02.py --mode prepare --run "$RUN"
git commit -m 'Freeze JOIN-SOURCE-02 bounded obstacle-response source screening'
git push origin main
.venv/bin/python scripts/run_join_source02.py --mode verify --run "$RUN"
env -u ACTION_TOKENIZER_BUNDLE -u MAX_BATCH_SIZE -u MAX_WAIT_MS \
  -u VLN_VIT_CACHE_ENTRIES VLN_EVAL_TEMPERATURE=0 VLN_EVAL_TOP_P=1 \
  VLN_EVAL_TOP_K=0 VLN_EVAL_TRAJ_TOP1=0 \
  .venv/bin/python scripts/lightnav/robotless_online_server.py start "$RUN"
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
  -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 \
  /home/gpuadmin/isaacsim/python.sh scripts/isaac/join_source02_collect.py --run "$RUN"
.venv/bin/python scripts/run_join_source02.py --mode screen --run "$RUN"
.venv/bin/python scripts/lightnav/robotless_online_server.py stop "$RUN"
.venv/bin/python scripts/report_join_source02.py --run "$RUN" \
  --output review_v1 --bundle review_bundle.zip
.venv/bin/python scripts/report_join_source02_layout.py --run "$RUN" \
  --output review_v2 --bundle review_bundle_v2.zip
.venv/bin/python scripts/validate_join_source02.py --run "$RUN" \
  --review review_v2 --output validation_v2.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

All creation commands refuse existing outputs; this is a command record, not an
instruction to overwrite/rerun the frozen source. Logs and source/software hashes
are under the run root. Raw data, external source and generated images/ZIPs are
excluded from Git. The normal result commit adds only code/tests/docs; the two
unrelated Stage-0 config edits remain untouched.

The authoritative saved-record validation is **validation_v2.json: valid=true**
(1507 checks, no failures). The earlier validation_attempt_01.json records a
post-primary checker schema mismatch (string condition IDs versus dictionaries).
The preserved validation.json reports two checker-only bbox failures: it assumed
exclusive maxima while the frozen capture code stores inclusive pixel maxima.
Only the new saved-record validator was corrected to the producer's actual
contract; source data, frozen collection code, thresholds and outcomes are unchanged.
The final eight focused validator tests pass, in addition to the full suite above.
The review ZIP is 1,841,152 bytes and contains no full dataset/environment/checkpoint.
