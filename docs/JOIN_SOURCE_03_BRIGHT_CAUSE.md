# JOIN-SOURCE-03: bright Hospital and bounded obstacle-response diagnosis

The question is which observable factor helps explain SOURCE02's changed but
unsafe cart response. This is upstream source diagnosis, not reconciliation,
navigation qualification, or a new controller. Starting/freshly fetched main:
`97ba41aab0ec846f9e6ddc0fc913dc3277cccb2e`.

Local run: `data/robotless_join_source_03/cause_20260921T153120Z/`.
Both unrelated Stage0 config edits and all historical raw/core/results are
preserved. MPC provenance matches the clean pinned official checkout across the
original dataset, JOIN01, SOURCE02 and REF01/02/03. Current actual module import
also matches; no tracker was constructed and no historical solve was needed.

## Pre-prediction protocol

`configs/join_source_03_bright_cause.yaml` freezes all conditions and thresholds.
The five authored lights remain unchanged. BRIGHT adds one neutral white
downward RectLight `/World/JOINSource03Fill`, world position `[19.2,24,2.6]`m,
size4x10m, intensity1000, exposure0, color[1,1,1], normalize=false. Hospital
geometry/materials/textures and camera remain unchanged; no JPEG enhancement.
Source USD is never saved/modified. Actual protected scene-attribute fingerprints
match DARK/BRIGHT. This is one profile, chosen before any new prediction.

Image-quality criteria: mean gain>=20 and median gain>=15 sRGB-luma code units,
whole-frame luma saturation>=250 fraction<=.05; cart>=20pixels, median>=45,
cart saturation<=.10; preserve target pixels if geometrically visible. Luma is
Rec.709 weighted stored sRGB code values, not calibrated physical illuminance.
Preflight mean12.271445→77.479331, median10.8516→65.9318, cart median12.2006→81.3702;
BRIGHT whole saturation.000679. Cart15779 pixels, target12014 pixels unchanged.

One initial technical snapshot failed because the timeline representation settled
during its first zero-delta render. Its `technical_preflight/inspect/` failure is
preserved. `inspect02` adds a setup render before checking exact stable state;
all subsequent captures retain the original strict synchronization check. No
model/MPC call or outcome-driven brightness change occurred.

Core: exact SOURCE02 location01 placement01 pose/cart/instruction,
`Go to the tall blue shelving unit and stop.` Independent A/OFF, B/ON, A'/SHAM
sessions for DARK then BRIGHT. The same15 original chronological DARK JPEGs
precede each newly rendered final RGB. Thus the clean intervention is **terminal
image lighting**, not an entirely bright-acquired history. Sham reuses identical
OFF JPEG bytes. No terminal MPC execution. Original observation anchoring,
arbitrary N rows and no intrinsic waypoint time remain unchanged.

Conditional phases, already frozen before predictions:

1. If BRIGHT does not recover a meaningful safe response: use actual saved
   location01 frame32 pose `[19.206962869,22.623339037,-1.568343243]`. The same
   shelf occupies29221 pixels with/without the cart; same target instruction.
   Cart center is .983029073m along that pose's heading, original mesh/orientation.
   H16 uses actual prior frames17..31. This changes pose/target-relative geometry,
   so it is a grounding association diagnostic, not pure target-recognition causality.
2. If target H16 remains unsafe/unavailable: H32 uses actual frames1..31 with
   **the identical H16 terminal off/on JPEGs**, cart, pose, instruction and lighting.
   Separately rendered technical H32 images remain unused. All33 original captures
   are genuine; frames28..32 excluded from SOURCE02's earlier terminal are now
   causal at the new terminal. H64 unavailable. SlowFast/session positions also
   change; this limitation is explicit. No extra acquisition or frame cloning.
3. If core BRIGHT remains unsafe: near/medium/far centers at .70/.90/1.10m along
   the historical no-obstacle path, fixed before new outputs. Cart mesh rear
   must remain inside that original output range; off path must be obstructed,
   observation safe, cart>=20pixels and a side passage valid. The existing off
   path has a separately labeled observation connector for placement only.
   Use core BRIGHT H16 pose/history/instruction and identical off/sham JPEGs.

At most7 conditions/21 terminal predictions, one attempt each. Optional receding
horizon is not run: later-chunk recovery remains untested. New MPC, GP/rigid,
reconciliation and closed-loop rollouts are all0. No online B or optimization-ready
moving handoff can be claimed from paused inputs.

Safety uses original Hospital+actual runtime cart mesh, radius.20m, edge clearance
.05m, height slab[.05,.65]m. No bounding-box replacement, suffix deletion or
connector masquerading as a model segment. Whole-polyline swept result and each
node/segment, first unsafe row/segment and <=1mm spatial localization are saved.
Positive clearance below5cm is not physical overlap. Rows are not timestamps.

Geometric equivalence: matching shape, XY<=1e-6m and wrapped yaw<=1e-6rad.
Material clearance gain:>=.02m. Meaningful off/on response reuses SOURCE02's
interior>=.20m OR reliable tangent/yaw>=20deg rule. Safe STOP is not bypass.
Safe partial detour and completed rear-plane bypass remain distinct. Bright
classification uses the predeclared L1–L4 rules; same-input sham variability is
reported separately, without statistical or hidden-neural-state claims.

All conditional inputs are rendered and checked before protocol/code freeze.
The pre-execution implementation is committed/pushed before the first diagnostic
prediction. Saved-only validation then replays no inference or controller.
Figures: same-pose RGB/luma, both paired world paths, row clearance, footprint
geometry, pointing, conditional target/history/distance and evidence matrix;
every figure has numeric/hash sidecar. No fabricated curves for unavailable
conditions. Observation anchoring and actual RGB are preserved unchanged.

## Commands

The exact run-specific execution and observed outcomes are appended after the
frozen comparison. Technical preflight uses the existing Isaac Python launcher
with the same cleared ROS/Python/CUDA loader variables as SOURCE02. Model uses
the pinned existing LightNav virtual environment and official launcher; no
dependency, driver, checkpoint or external source changes.
