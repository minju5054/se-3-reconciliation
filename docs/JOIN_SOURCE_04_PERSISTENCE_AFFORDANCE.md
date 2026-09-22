# JOIN-SOURCE-04: obstacle persistence and observable affordance/action geometry

Question: does a cart visible in several immediately preceding frames improve
raw LightNav clearance, and does observable APOS support a free-side direction
that the decoded path does not realize? This is **COUNTERFACTUAL MOVING-POSE
HISTORY**, not a genuine executed obstacle-reveal episode. No MPC, GP, rigid,
reconciliation, online confirmation or traversal is part of this experiment.

Starting/fetched main: `24a020971a7b30989812d5690a91b361388ae6a4`.
Scientific run: `data/robotless_join_source_04/persistence_20260922T050237Z/`.
Original bank/failed pre-request run: `persistence_20260922T044703Z/`.
Both unrelated Stage0 config edits and SOURCE02/03 raw/results remain preserved.

## Frozen scenario and inputs

Exact SOURCE03 `core_bright_H16` lineage at Hospital location01; final SE(2)
observation pose `[19.20462152247834,23.62333631121559,-1.5685636348381817]`.
Instruction: **Go to the tall blue shelving unit and stop.** Target
`/World/Hospital/SM_Shelf_06f7`. Cart source
`/World/Hospital/SM_SupplyCart_02a7`, runtime `/World/JOINSource02Obstacle`,
center `[19.207161167236976,22.64031056011843]` m. All 8024 authored mesh
triangles, wrapper transform and original height-slab projection must match
SOURCE03 literally. Actual mesh plus original Hospital direct geometry is the
oracle; no bounding-box collision replacement. Footprint radius .20 m, required
edge clearance .05 m, height slab [.05,.65] m, original uncertainty unchanged.

All 16 camera poses are re-rendered twice in Isaac, OFF then ON. Original source
frames12..26 plus SOURCE03's terminal observation are preserved as the same16
slot identities, in chronological pose order. SOURCE03's terminal rerender
clock is retained separately from its source-pose lineage; no new motion is
implied. Each K uses the same16 IDs, poses, original capture-time labels and
model session positions0..15. Newly captured UTC/monotonic timestamps identify
the new renders. Official sampler video_fps4 uses delivered indices, not these
original timestamps. LightNav rows have no intrinsic timestamps.

BRIGHT is exactly SOURCE03's added neutral downward RectLight
`/World/JOINSource03Fill`, position[19.2,24,2.6] m, size4x10 m, intensity1000,
exposure0, color[1,1,1], normalize=false; five authored lights unchanged.
Same 480x270 pinhole camera,112.2deg horizontal FOV, +.09 m forward/.65 m up,
RayTracedLighting/AA0, JPEG95. No post-processing. Composed scene-attribute
hashes match each OFF/ON pair, excluding only runtime-cart visibility. Camera,
simulation time, RGB/instance/depth render product remain stable. Authored USD
is never saved. This differs from SOURCE03's DARK preceding history plus
BRIGHT terminal: all16 current frames are BRIGHT, not repeated SOURCE03 samples.

Before any prediction: all32 RGB/mask/depth records are saved, source transforms
verified at1e-12, OFF cart pixels0, included ON cart pixels>=20, constant cart
transform and non-overlapping historical footprint checked. Unavailable K is
preserved without replacement. Current oracle environment status is also saved.
No duplicated final JPEG, stationary padding, future frame or reordered pose.

## Fixed scientific protocol

Independent official sessions, exactly this order:
`K0_OFF, K0_OFF_SHAM, K1, K1_SHAM, K2, K4, K8`.
H=16; last K frames ON, all others OFF. Shams use literally identical input
bytes in new sessions. Fifteen empty-instruction buffer-only requests then one
terminal prediction. At most7 scientific predictions/105 buffer restores, no
retry. Same source/checkpoint/task/generation as SOURCE03; no terminal MPC.

Official clean source `c6f40e3220edbf7011e4f17eaf2c865416737d4d`, checkpoint
revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, task VLN, temperature0,
top_p1, top_k0, traj_top1=0. Startup source/checkpoint/server argv and wire
requests are separately preserved. Official MPC hash
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1` must match
all earlier lineages; no tracker or optimization is constructed here.

Raw actions remain generic Nx3; world conversion is observation-anchored
SE(2), x-forward/y-left/yaw-CCW in metres/radians. Whole returned raw polyline
and every node/segment use the unchanged swept-footprint checker. No suffix
trimming and no observation-to-first-row connector counted as model output.
First clearance-crossing localization is sampled at<=1 mm spatial spacing;
this does not certify continuous dynamics or exact crossing position.

Frozen decision rules: entire returned path must satisfy clearance; recovery
also requires non-STOP and positive target-directed progress. Material minimum
clearance gain>=.02 m versusK1; or first unsafe segment moves>=1 later without
worsening earlier segment minima. Geometric equivalence uses1e-6 m XY and1e-6
rad wrapped yaw. Existing meaningful response criteria remain interior lateral
.20 m or reliable tangent/yaw20deg, chord>=.02 m. Same-input sham output
inequivalence makes persistence attribution inconclusive. Count of improving
conditions and monotonicity are reported separately; no monotonicity assumption.
An already-safe all-BRIGHT K1 cannot establish persistence recovery.

## APOS interpretation and frozen geometry diagnostics

Official README describes APOS as feasible local direction/free-space waypoint;
it remains a coarse output cell, not a demonstrated metric planner target.
Client-image coordinates follow the48x27 pointing grid. `apos_clamped=true`
means at-edge-or-beyond, not an exact intended edge point. Such a point is always
`CENSORED_BY_CLAMPING`; ray output is `BOUNDARY_RAY_PROXY` and cannot establish
free-side agreement. OPOS/visible concern target grounding, not cart recognition.
Raw response/text/action and pointing fields are preserved; absent values N/A.

Reported-pixel ray uses actual saved pinhole K and T_world_camera (+Xright,
+Yup,-Zforward). First rendered hit uses evaluator-only Isaac
`distance_to_camera` plus instance raster at floor(pixel). Reconstruct that
raster hit at pixel center(x+.5,y+.5), recording the subpixel difference from
the reported APOS cell. This is a pixel-sampled first-surface diagnostic, not
an exact analytic collision ray. A separate forward intersection with nominal
world z=0 is saved only when geometrically valid. Depth/masks never enter the
model. The raster-center clarification was made and draft protocol preserved
before predictions, after checking saved authored-floor depths.

A reliable free side requires: unclamped, first rendered hit floor, ground
oracle clearance valid, lateral beyond actual cart extent+.25 m, bearing>=20deg.
Trajectory broad side uses largest signed lateral if |lateral|>=.20 m, otherwise
first chord>=.02 m with |bearing|>=20deg. Ambiguous cases remain ambiguous.
Opposite reliable sides yield observable disagreement; matching side but unsafe
path yields geometric mismatch/insufficient magnitude, never proof of RVQ error.

## Reproduction commands and completion

Pre-prediction implementation/config/tests and frame/input hashes are committed
and pushed before the first scientific prediction. Exact runtime commands,
execution SHA, all outcomes, calls, checks and figures are appended after that
bounded run. Saved-only validator rechecks raw/world/wire, masks, source hashes,
all K inputs, classifications and figure numbers without inference. Synthetic
fixtures are implementation tests only.

## Pre-request technical correction

First pushed freeze `5f06f07d337af0196f7144b5906be40c13dc2acb` exposed an
import-only worker error in the external LightNav venv: importing the research
evaluator pulled in Shapely, which that environment does not contain. All seven
process attempts ended before a session/request; scientific predictions and
buffer calls were0. The server performed one separate synthetic startup warmup
and was stopped. All logs/ledger/code freeze remain in the original run.

The worker now reads the allowed IDs/scope from the already-frozen protocol
without importing the evaluator. External interpreter import-only smoke passed
without calls; a regression test guards this dependency boundary. No dependency
installation, model/environment modification, threshold/input/K change or
scientific-output retry. A new run preserves byte-identical32-frame bank files
as immutable hardlinks and separately hashes all failed-run artifacts. This
correction is committed/pushed before the first actual scientific prediction.

## Observed result: all seven declared scientific predictions completed

Execution freeze: `7682acd9c2767f09bcfdc110ebf20bfb0c438a0d`, pushed before
predictions. The seven sessions completed once, with no response-driven retry.
The server was stopped immediately after the bounded sequence. All K were
premodel-valid. Across all16ON frames cart pixels671..15779, original target
pixels1681..12014, cart edge separation from historical robot footprints
.575890..3.565414 m. Included K8 frame pixels1936..15779. OFF masks contain0
runtime-cart pixels; the original cart elsewhere in the authored Hospital is a
different instance. Frame-bank rendering took187.221508 s and performed0 model,
MPC or GP calls.

The primary result is **PERSISTENCE_IMPROVES_BUT_REMAINS_UNSAFE**. More ON history
changes actions and improves minimum clearance versusK1 by3.19–6.23 cm, but
none produces a safe raw future or meaningful lateral bypass. Improvement is
not monotonic: K2 has the highest clearance; K4/K8 coincide. The first unsafe
segment remains2 and first unsafe row3 (zero-based) in every condition. The
unsafe ON paths were only checked geometrically; no collision was executed.

All outputs contain10 rows in this runtime; code and tests remain generic N.
All response `stop=false`, `visible=false`, OPOS=`not_visible`/None. Top-level
visible concerns the target. Rendered target pixels do not establish model
target grounding. The original full-polyline checker and thresholds are unchanged.

| Condition | Minimum edge clearance, actual input scene (m) | Same path with cart ON (m) | Raw arc (m) | Max absolute lateral (m) | APOS pixel | Clamped | Terminal RTT (s) |
|---|---:|---:|---:|---:|---|---|---:|
| K0_OFF | 0.935417382 | -0.200000000 | 1.356219375 | .000392610 | [245,165] | false | .335816690 |
| K0_OFF_SHAM | 0.935417382 | -0.200000000 | 1.356219375 | .000392610 | [245,165] | false | .330046680 |
| K1 | -0.143249985 | -0.143249985 | .565196501 | .006376110 | [235,265] | true | .331968959 |
| K1_SHAM | -0.143249985 | -0.143249985 | .565196501 | .006376110 | [235,265] | true | .333282255 |
| K2 | -0.080954592 | -0.080954592 | .510887743 | .000214055 | [235,265] | true | .330860218 |
| K4 | -0.111383570 | -0.111383570 | .545469106 | .005875844 | [235,265] | true | .330979955 |
| K8 | -0.111383570 | -0.111383570 | .545469106 | .005875844 | [245,265] | true | .333418209 |

The K0 ON value is a hypothetical obstruction check, not failure of the obstacle-
absent input scene. Negative values indicate footprint overlap under the saved
mesh oracle; they are not measured physical collision dynamics or calibrated
penetration depth. The required positive edge margin remains.05 m.

| ON condition | Gain over K1 (m) | First required-clearance crossing, arc from first raw row (m) | Endpoint before cart front plane (m) | Final local yaw (rad) |
|---|---:|---|---:|---:|
| K1 | 0 | [.370963199,.371962687] | .031756050 | -.002972013 |
| K2 | .062295392 | [.376189398,.377188396] | .096213049 | -.000708737 |
| K4 | .031866415 | [.373480352,.374479689] | .062818533 | -.296959490 |
| K8 | .031866415 | [.373480352,.374479689] | .062818533 | -.296959490 |

All paths begin about.15 m ahead of the observation pose; the first returned
point is safe, with roughly.42 m edge clearance. Thus failure is not a cart-
overlapping initial pose/first row in this scenario. The footprint reaches the
cart even though the ON endpoint center remains before its front plane. No
observation connector or intrinsic row times are assumed.

The descriptive improvement is a shorter nearly straight returned extent,
not a recovered detour. K4/K8 change stored yaw late in a near-stationary part
of the spatial array (final yaw about-17.015deg), while XY lateral change stays
below6.4 mm. Untimed rows do not demonstrate braking speed, commanded stop or
wheel dynamics. Existing reliable interior-response thresholds are not met:
maximum interior separation versusK0 is .006519430/.000114266/.006000358 m
forK1/K2/K4; maximum reliable tangent differences .980344/.056077/1.117649deg.
Endpoint shortening is not misreported as lateral separation.

## APOS, image space and evaluator rays

The APOS conclusion is **AFFORDANCE_REMAINS_AMBIGUOUS_DUE_TO_CLAMPING**.
Every ON response is bottom-clamped; no condition supplies a reliable exact
free-side point. All belong to `APOS_OBSTACLE_DIRECTED_OR_AMBIGUOUS`. Neither
clear-side/action-unsafe nor opposite-direction disagreement is established.

Terminal cart mask is15779 pixels, bbox[145,110,336,269]. Reported ON APOS pixels
are outside that mask,28 px from its nearest pixel and within its horizontal
span. This only describes the censored reported cell; it does not prove that
the intended point avoids the obstacle. Actual `distance_to_camera` raster hit
is Hospital floor in all cases. Floor visible through/below a cart mesh is not
equivalent to space for a .20 m-radius footprint.

| Condition | Token | Raster first-hit distance (m) | Nominal ground forward/left from observation (m) | Ground bearing proxy (deg) | Ground proxy cart-edge clearance (m) |
|---|---|---:|---|---:|---:|
| K0 / sham | apos_793 | 3.494465590 | [3.584255044,-.108333332] | -1.731226 | 2.190855333 |
| K1 / sham | apos_1272 | 1.031971216 | [.896366549,.025000000] | 1.597587 | -.200000000 |
| K2 | apos_1272 | 1.031971216 | [.896366549,.025000000] | 1.597587 | -.200000000 |
| K4 | apos_1272 | 1.031971216 | [.896366549,.025000000] | 1.597587 | -.200000000 |
| K8 | apos_1273 | 1.032091141 | [.896366549,-.025000000] | -1.597587 | -.197490268 |

These ON bearings are **BOUNDARY_RAY_PROXY**, not intended metric directions.
They remain near the center rather than beyond a free lateral cart side, and
their footprint-ground proxies are themselves unsafe. Raster first-hit XYZ
lies at authored floor z≈.001002 m; nominal-ground intersection uses z=0 and
the reported coordinate, while depth uses the raster center. Exact origins,
directions, world XYZ, prim path and every offset are in each `evaluation.json`.
The additional full-world proxy figure displays even the long K0 ray; the
frozen local direction panel clips that ray to its common local plotting range.

Raw pointing and action token evidence:

| Condition | RVQ tokens (l0,l1,l2) | Raw .npy SHA256 |
|---|---|---|
| K0 / sham | 0,30,85 | `203a78fad06ec0ccb3b8bd2f7fd068c281da45ceac7de36cf621a45787c1a0d9` |
| K1 / sham | 64,253,221 | `e2d37c7a827dfc2afdf5645182996394e6e775433354bee2f5ce9505d1808f1a` |
| K2 | 64,217,220 | `5231d31b148e90f1ec7efb3f5fda23a26d76b396e60d3b84fe767409eac3eec1` |
| K4 / K8 | 64,67,163 | `6240dead38ea73854850b0d4a770395102f2ab520800745505eca968a3efd0e7` |

Both sham arrays/text match their corresponding original. K1/K2/K4 have the
same reported APOS but different actions; K4/K8 have different APOS cells but
identical actions. A coarse censored pointing trace does not specify a unique
trajectory. This is not evidence that the RVQ decoder is broken, nor proof
that obstacle recognition succeeded or failed internally.

## Bounded cause evidence

| Candidate | Evidence for / still possible | Evidence against / limitation | Status |
|---|---|---|---|
| Accidental MPC modification | Historical MPC motion shaped original poses | Pinned provenance matches; no new terminal MPC | STRONGLY_DISFAVORED |
| Low illumination | SOURCE03 relighting improved clearance | Current whole history BRIGHT still unsafe; not a SOURCE03 repeat | CONTRIBUTING_FACTOR_SUPPORTED |
| Insufficient history count | Richer context may matter elsewhere | Historical H16/H32 did not recover; current H fixed16 | DISFAVORED |
| Sudden single-frame reveal | K2/K4/K8 improve minimum clearance by>=.02m | Same early violation, no safe bypass, nonmonotonic | CONTRIBUTING_FACTOR_SUPPORTED |
| Obstacle distance/local horizon | Later chunks remain untested | Historical near/medium/far all unsafe; no current distance change | DISFAVORED |
| Target grounding | visible=false/OPOS not_visible persists | Target rendered12014pixels; pixels do not prove grounding | NOT_ISOLATED |
| Affordance spatial reasoning | Coarse output changes | Every ON APOS clamped, exact free-side interpretation unavailable | NOT_ISOLATED |
| Action geometric realization | Every raw ON footprint path unsafe | No reliable free-side APOS to localize mismatch; no hidden-state access | NOT_ISOLATED |
| Same-input variability | Only one sham per input | Both K0/K1 shams exactly match | DISFAVORED |

Persistence contributes to this fixed input's geometry but is insufficient to
explain or resolve the failure. A stronger claim that sudden reveal was the
primary cause is not supported. No safe or promising obstacle-bypass source
for later genuine online confirmation was obtained. Target grounding versus
spatial/trajectory realization remains the largest unresolved distinction.
This task stops here; no confirmation, movingB or SE(2) optimization follows.

## Computation, validation and evidence

- Scientific:7 terminal model predictions,105 buffer-only restores,7 independent
  sessions;0 MPC/GP/rigid/reconciliation/rollout calls.
- Technical:32 genuine Isaac renders (16pairs),187.221508 s capture phase;
  first failed run7 import-only processes/0 requests. Two separate synthetic
  startup warmups across the failed and corrected servers (287/277 ms).
- Corrected scientific-run server startup16.761073 s, lifetime106.828761 s;
  failed-run startup18.271576 s, lifetime82.632015 s. Terminal RTT sum2.326373 s;
  seven worker wall times sum32.142204 s. These nested times are not additive.
- Tests:0 real LightNav calls. The existing full-suite historical MPC test calls
  official solve once (.043629 s), reproducing command/reference exactly; its
  artifact is separate at `technical_preflight/historical_MPC_test_audit.json`.
- Full suite:2564 passed/19 skipped in169.38 s. Focused pre-freeze checks147
  passed; corrected worker tests58 passed; final focused regression148 passed.
  `validation_final.json` repeats saved-only checks after the full suite and passes.
  compileall/diff checks pass. No shell
  launcher changed, no external source/checkpoint/venv was modified.
- `validation.json` passes saved-record recomputation, source/core hashes,
  input/pose/wire equality, independent sessions, geometry, APOS and figure
  numbers. `validation_review.json` separately verifies failed-run preservation,
  actual call ledger, CSV numbers and ZIP member bytes. No inference in validators.
  Scientific unsafe outputs are valid artifacts, not validator failures.

Local review:
`data/robotless_join_source_04/persistence_20260922T050237Z/index_final.html`
and `review_bundle_final.zip`. Twelve static figures and numeric/hash sidecars:
K0/K1/K4/K8 full16-frame sheets, terminal cart-mask/APOS, APOS/bearing vsK,
world raw paths/actual mesh footprint, row/arc clearance, lateral/yaw/arc,
affordance/action geometry, full ground proxies and final evidence matrix.
All final images were inspected; input RGB was never annotated for the model.
The finalizer only adds presentation/call-accounting artifacts; frozen scientific
source, raw outputs, first review and classification remain unchanged.

## Exact commands

```bash
.venv/bin/python scripts/run_join_source04.py --mode init \
  --run data/robotless_join_source_04/persistence_20260922T044703Z
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
  -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 \
  /home/gpuadmin/isaacsim/python.sh scripts/isaac/join_source04_bank.py \
  --run data/robotless_join_source_04/persistence_20260922T044703Z
# Original prepare/freeze5f06f07 and pre-request import failure retained.
# Corrected run initialized; frame_bank copied using shutil.copytree(copy_function=os.link),
# technical_correction.json hashes the complete prior run; no re-render.
.venv/bin/python scripts/run_join_source04.py --mode prepare \
  --run data/robotless_join_source_04/persistence_20260922T050237Z
# Commit/push7682acd and verify before the seven scientific predictions.
.venv/bin/python scripts/run_join_source04.py --mode verify \
  --run data/robotless_join_source_04/persistence_20260922T050237Z
env -u ACTION_TOKENIZER_BUNDLE -u MAX_BATCH_SIZE -u MAX_WAIT_MS \
  -u VLN_VIT_CACHE_ENTRIES VLN_EVAL_TEMPERATURE=0 VLN_EVAL_TOP_P=1 \
  VLN_EVAL_TOP_K=0 VLN_EVAL_TRAJ_TOP1=0 OPENBLAS_NUM_THREADS=1 \
  OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 .venv/bin/python \
  scripts/lightnav/robotless_online_server.py start \
  data/robotless_join_source_04/persistence_20260922T050237Z
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  .venv/bin/python scripts/run_join_source04.py --mode execute \
  --run data/robotless_join_source_04/persistence_20260922T050237Z
.venv/bin/python scripts/lightnav/robotless_online_server.py stop \
  data/robotless_join_source_04/persistence_20260922T050237Z
.venv/bin/python scripts/report_join_source04.py \
  --run data/robotless_join_source_04/persistence_20260922T050237Z
MPLCONFIGDIR=/tmp/join_source04_mpl .venv/bin/python scripts/validate_join_source04.py \
  --run data/robotless_join_source_04/persistence_20260922T050237Z
MPLCONFIGDIR=/tmp/join_source04_mpl .venv/bin/python scripts/finalize_join_source04.py \
  --run data/robotless_join_source_04/persistence_20260922T050237Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  MKL_NUM_THREADS=1 MPLCONFIGDIR=/tmp/join_source04_mpl \
  .venv/bin/python -m pytest --basetemp /tmp/join_source04_pytest_20260922
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

All output creation is exclusive; use a new run ID for any future authorized
experiment. Do not rerun this completed scientific sequence to obtain success.
