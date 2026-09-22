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
