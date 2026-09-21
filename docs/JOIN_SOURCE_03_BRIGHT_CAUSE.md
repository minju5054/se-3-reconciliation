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

## Observed result, 2026-09-22 KST

**LIGHTING_CONTRIBUTES_BUT_NOT_SUFFICIENT.** All seven frozen conditions completed
once: 21 terminal predictions, with no source/controller/reconciliation retry.
All seven obstacle-ON paths physically overlap the cart. The root cause remains
**UNRESOLVED_AFTER_BOUNDED_DIAGNOSIS**. This is completed bounded diagnosis with
the limitations below, not an optimization-ready online source or a navigation test.

Starting main: `97ba41aab0ec846f9e6ddc0fc913dc3277cccb2e`.
Execution/freeze revision, pushed before inference:
`1904bd2031ffa14cf86399ffdd1277a49bd4cc92`.
The result commit containing this report is a separate reporting revision; no
frozen experiment implementation or historical result was changed after inference.

### Pinned source and MPC audit

Official checkout `/home/gpuadmin/Workspace/external/LightNav-0-official-demo`
is clean at `c6f40e3220edbf7011e4f17eaf2c865416737d4d`.
Checkpoint `LightOriginsHQ/LightNav-0`, preserved revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, remains at
`/home/gpuadmin/Workspace/external/LightNav-0/checkpoints/LightNav-0`.
The actual startup manifest hashes every checkpoint file, including weights
`ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18`
and eval config `7f475edbef4a13d2db99fa12eff5e9dedf48916969ec777f75e6e85e2a492b57`.

Task stays `vln` → `vlnce` / `vlnce_traj`; no object-goal task switch.
Server uses official `lightnav-serve --task vln --backend vllm_local`, GPU memory
utilization .55, frozen generation environment temperature0/top_p1/top_k0/traj_top1=0.
Camera remains 480×270, JPEG quality95, HFOV112.2°, .09m forward/.65m high;
official 256×448 preprocessing and SlowFast are unchanged. The checkpoint's
nominal history64 is not a minimum or fixed delivered-frame count. Actual outputs
all have N=10; code and records retain arbitrary N and no waypoint timestamps.

The actual loaded file is
`mujoco_demo/vln_mujoco/mpc.py`, SHA256
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
`mpc_audit/result.json` reports **MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL** against
dataset/JOIN01/SOURCE02/REF01/REF02/REF03 records. Actual current import agrees.
HORIZON5, dt.1s, control10Hz, Q=[10,10,1], R=[.1,.1], v=.8m/s,
omega=3rad/s, acceleration2m/s²/5rad/s² are unchanged. The online import path is
`online_mpc_worker` → audited adapter → official `MpcTracker`. REF-01 changed
supplied reference preparation; REF-02/03 used explicit per-instance offline
selector wrappers with the same MPC computation. Those wrappers were not installed
as online defaults. SOURCE02 and SOURCE03 terminal paired inference precede any
terminal MPC execution, using restored fixed history/pose.

Thus accidental MPC modification is strongly disfavored here. MPC still affected
the original captured history indirectly through motion; this experiment does not
deny that effect. A separate existing full-suite historical replay calls official
MPC once and reproduces its saved command/reference exactly. It is a test, not
a SOURCE03 condition or a new controller experiment.

### Illumination and observation integrity

| Same ON camera/geometry statistic | DARK | BRIGHT |
|---|---:|---:|
| Whole mean luma, 0–255 | 12.271445 | 77.479331 |
| Whole median | 10.851600 | 65.931800 |
| Whole p10 / p90 | 2.0682 / 21.0562 | 31.2768 / 134.2768 |
| Near-black fraction | .479313 | .017515 |
| Saturated fraction | .000671 | .000679 |
| Cart median | 12.200600 | 81.370200 |
| Target median | 2.000000 | 33.863800 |
| Cart / target instance pixels | 15,779 / 12,014 | 15,779 / 12,014 |

All frozen brightness gates pass. Segmentation is from the same camera/render
state and identifies the cart and shelf separately. Hidden cart has zero pixels;
visible cart is detected. Protected geometry/material/camera hashes agree across
lighting. The 8,024-triangle cart mesh agrees with SOURCE02 literally. Model RGB
contains no overlays or mask; transmitted JPEG hashes match the saved bytes.
Original five light values and the new fill profile are in
`technical_preflight/inspect/light_inventory.json` and
`technical_preflight/render/lighting_mutation.json`.

Target is the actual blue shelf `/World/Hospital/SM_Shelf_06f7`. Cart is an
instance of `/World/Hospital/SM_SupplyCart_02a7`, retained under the existing
runtime prim `/World/JOINSource02Obstacle`. Instance names do not imply the
historical dataset was modified. Hospital + actual height-slab mesh is evaluated;
mask geometry is not supplied to LightNav.

### All completed conditions

Edge clearance is the original footprint metric in metres; nominal requirement
is +.05m. Negative values indicate physical overlap. This distance-based metric
saturates at −.20m when the footprint center lies inside occupancy; it is not an
exact signed mesh penetration depth. No unsafe rows or observation connector
were inserted into/deleted from the returned model polyline.

| Condition | OFF/sham minimum in OFF scene | ON minimum in ON scene | ON arc, first→last row | First unsafe row (zero-based) | ON label |
|---|---:|---:|---:|---:|---|
| Core DARK H16 | .935417 | −.200000 | .848790 | 3 | CHANGED_BUT_UNSAFE |
| Core BRIGHT H16 | .935417 | −.111384 | .545469 | 3 | NO_MATERIAL_RESPONSE |
| Target BRIGHT H16 | .220317 | −.080926 | .510888 | 3 | CHANGED_BUT_UNSAFE |
| Target BRIGHT H32 | .220317 | −.200000 | .848790 | 3 | CHANGED_BUT_UNSAFE |
| BRIGHT near .70m H16 | .935417 | −.200000 | 1.310908 | 1 | CHANGED_BUT_UNSAFE |
| BRIGHT medium .90m H16 | .935417 | −.192836 | .545469 | 2 | NO_MATERIAL_RESPONSE |
| BRIGHT far 1.10m H16 | .935417 | −.200000 | .848790 | 4 | CHANGED_BUT_UNSAFE |

In every row, OFF and sham raw arrays are literally equal; both are safe without
cart, and both overlap the cart when evaluated in the ON environment (minimum
−.20m). ON differs from OFF, but all ON full paths remain unsafe. All 21 responses
have `stop=false`, `visible=false`, OPOS `not_visible`. `NO_MATERIAL_RESPONSE`
means the original geometric-response threshold was not met; it does not mean
unchanged bytes. Complete OFF/ON/sham values, hashes and timings are in
`aggregate/outcomes.csv` and each `paired_diagnostics/<condition>/evaluation.json`.

Core BRIGHT improves minimum clearance by .088616m, exceeding the frozen .02m
comparison criterion: **BRIGHT_IMPROVES_CLEARANCE_BUT_UNSAFE**. However, it does so
with a shorter almost-straight output, not a recovered detour. Arc decreases
.848790→.545469m; maximum absolute lateral displacement remains below7mm.
The first returned row is safe in both (.423502/.424310m clearance). The first
unsafe segment is row2→3, and row3 (fourth waypoint) already overlaps. The sampled
required-clearance transition is localized to arc [.373407,.374403]m in DARK and
[.373480,.374480]m in BRIGHT, measured from the first returned row, not a timestamp.
This directly distinguishes an early unsafe prefix from merely ending too soon
to finish a later detour. No such path was executed.

The target diagnostic increases renderer target area to29,221 pixels (cart15,772),
but model target visibility remains false. Its internal label
`TARGET_VISIBLE_STILL_UNSAFE` refers to **renderer visibility**, not proven model
grounding. New pose/relative geometry is also different; this is not a clean
neural grounding intervention. Target grounding remains unresolved.

H16→H32 keeps the same final JPEGs, pose, cart and instruction. It changes output
and worsens minimum clearance −.080926→−.20m; no safety recovery. Classification:
**HISTORY_NOT_EXPLANATORY_UNDER_TEST**. The suffix histories are real chronological
frames, but session/SlowFast positions also differ. H64 and a wholly bright
temporal history remain untested; nominal64 is not proof that H16 is invalid.

Cart distance changes the response. Near produces a substantive rightward curve
(last local row [1.287630,−.552365]m, yaw−.890956rad), yet enters collision before
that curve clears the cart. Medium remains nearly straight; far reproduces core
DARK's local array. These three fixed distances therefore yield
**DISTANCE_CHANGES_REACTION_BUT_STILL_UNSAFE**. All intersect the historical OFF
range and have checked lateral space; far was not moved beyond the path to make
safety trivial. Distance and finite local horizon are not a sufficient explanation
within this range. Successive-chunk recovery is untested: optional receding-horizon
execution was disabled before outcomes, not retried after failure.

### Pointing and action-token observations

These are literal response fields, not hidden reasoning or perception labels.
Pixel coordinates refer to the official 480×270 client-image mapping.

| ON condition | APOS px | action RVQ tokens l0/l1/l2 |
|---|---|---|
| Core DARK H16 | [235,265] | 37 / 149 / 54 |
| Core BRIGHT H16 | [245,265] | 64 / 67 / 163 |
| Target H16 | [245,265] | 64 / 217 / 220 |
| Target H32 | [245,265] | 37 / 149 / 54 |
| Near | [245,265] | 248 / 218 / 166 |
| Medium | [245,265] | 64 / 67 / 163 |
| Far | [245,265] | 37 / 149 / 54 |

All ON APOS states are `point`, clamped=true; OPOS is null/not_visible,
clamped=false. APOS changes and emitted action-token changes establish observable
input/output sensitivity, not correct cart recognition. No conclusion that the
decoder failed follows from these records. Raw text, fields and arrays remain
under each branch's `chunks/terminal/response.json`.

### Cause interpretation

| Candidate cause | Evidence for / unresolved | Evidence against / limit | Status |
|---|---|---|---|
| Accidental MPC modification | No mismatch found; historical motion affects RGB | Six source chains/current import agree; no terminal MPC | STRONGLY_DISFAVORED |
| Low illumination | Same-pose BRIGHT changes path and raises minimum clearance by8.86cm | Every ON path still overlaps; dark history retained | CONTRIBUTING_FACTOR_SUPPORTED |
| Target grounding / visibility | `visible=false` despite actual shelf pixels | No successful model-grounding intervention; new pose confounds geometry | NOT_ISOLATED |
| Insufficient temporal history | H32 changes generated actions | Same-final H32 does not recover; H64/bright history untested | DISFAVORED within H16/H32 test |
| Obstacle distance / finite horizon | Distance changes path/turn response | All three within-range positions unsafe; unsafe prefix appears before endpoint | DISFAVORED as sole explanation in tested range |
| Same-input stochastic variability | One sham per condition only | Seven OFF/sham pairs identical | DISFAVORED within observed pairs |
| Spatial/free-space/action-generation limitation | Unsafe output persists after tested controls | Internal grounding/geometry/action stages not separately measured | NOT_ISOLATED |

The strongest supported account is **lighting changes the model's generated
geometry and clearance but does not recover safe avoidance; early unsafe path
segments persist**. It is not established that the model did not see the cart,
that the RVQ decoder is wrong, or that it cannot recover in a later chunk.
The largest remaining uncertainty is target grounding versus the upstream
spatial/action-generation stages; this bounded experiment cannot separate them.
No next-stage policy, avoidance repair or SE(2) optimization was implemented.

### Calls, validation, technical corrections and evidence

| Work | Actual count / time |
|---|---|
| Terminal scientific predictions | 21 (7×OFF/ON/sham) |
| Buffer-only history restores | 363; no prediction |
| Official synthetic server warmup | 1, separate from evidence |
| Server prediction log total | 22 =21+1 |
| Diagnostic MPC / GP / rigid / reconciliation / rollout | 0 / 0 / 0 / 0 / 0 |
| Real model tests / historical MPC test | 0 / 1 (MPC .041430s) |
| Terminal client RTT sum | 6.781750s |
| Diagnostic worker elapsed sum | 37.372727s |
| Server startup / lifetime | 17.261760 / 118.809556s |

Nested times overlap and must not be summed. Model server was terminated using its
recorded process identity. No online moving B, obstacle traversal or qualifying
future optimization source was obtained. `aggregate/actual_call_counts.json`
and original server/worker logs preserve accounting.

Full suite: **2536 passed,19 skipped in174.14s**; skips concern missing historical
fixtures and existing diagnostic scope. Final focused SOURCE03/presentation tests:
**23 passed**. The real historical MPC test is separately preserved in
`technical_preflight/historical_MPC_test_audit.json`; commands and references agree
exactly. `compileall` and `git diff --check` pass. No shell launcher was changed.

Authoritative saved-only `validation_v2.json` passes2019 checks, including1899
preserved historical/core/raw/user-file hashes, scientific geometry recomputation,
all21 input/wire/array transforms, conditional coverage and figure/source parity.
`validation_review.json` additionally validates three paired-comparison figures,
all210 row records and ZIP members. Neither validator runs inference or MPC.
Scientific failure is retained as valid experimental evidence.

The original reporter failed while drawing a clipped Hospital GeometryCollection;
its partial `review/` and failure log are preserved. The first validator attempt
could not find the unfinished figure manifest. A presentation-only adapter adds
GeometryCollection rendering and versioned output names (`review_v2`,
`validation_v2`), retaining the frozen report/validator source hashes and identical
scientific summary. No scientific rerun, threshold change or source overwrite
occurred. Additional paired figures are separate `review_comparisons/` artifacts.
All21 final PNGs were visually inspected, with numbers/hash sidecars.

Local evidence under the run root:

- `index.html`: complete review entry.
- `review_v2/dark_bright_same_pose_rgb.png`: actual unchanged-pose Isaac images.
- `review_comparisons/lighting_comparison.png`: first unsafe prefix and clearance.
- `review_comparisons/history_comparison.png`, `distance_comparison.png`.
- `review_v2/*pointing.png`: diagnostic overlays, never transmitted to model.
- `aggregate/row_geometry.csv`, `outcomes.csv`, `summary.json`.
- `review_bundle_final.zip`: 5,302,134 bytes,21 figures and compact numeric/source
  records; excludes checkpoint, full environment and original RGB corpus.

### Exact execution and reporting commands

These describe the recorded run; existing outputs refuse overwrite. A new run
would require a separate protocol, not replaying this run into its directory.

```bash
git status --short --branch
git remote -v
git fetch origin
git rev-parse HEAD
git rev-parse origin/main
git log --oneline -10
RUN=data/robotless_join_source_03/cause_20260921T153120Z
```

The following exact Isaac prefix was used with `--mode inspect`, then
`--mode inspect --attempt inspect02`, `--mode render` and `--mode inputs`:

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
  -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 \
  /home/gpuadmin/isaacsim/python.sh scripts/isaac/join_source03_render.py \
  --run "$RUN" --mode inputs
.venv/bin/python scripts/run_join_source03.py --mode prepare --run "$RUN"
# Review/test/commit/push 1904bd2 before inference.
.venv/bin/python scripts/run_join_source03.py --mode verify --run "$RUN"
env -u ACTION_TOKENIZER_BUNDLE -u MAX_BATCH_SIZE -u MAX_WAIT_MS \
  -u VLN_VIT_CACHE_ENTRIES VLN_EVAL_TEMPERATURE=0 VLN_EVAL_TOP_P=1 \
  VLN_EVAL_TOP_K=0 VLN_EVAL_TRAJ_TOP1=0 \
  .venv/bin/python scripts/lightnav/robotless_online_server.py start "$RUN"
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  .venv/bin/python scripts/run_join_source03.py --mode execute --run "$RUN"
.venv/bin/python scripts/lightnav/robotless_online_server.py stop "$RUN"
# Original report attempt preserved; final presentation-only correction:
.venv/bin/python scripts/join_source03_presentation.py --mode report --run "$RUN"
.venv/bin/python scripts/join_source03_presentation.py --mode validate --run "$RUN"
.venv/bin/python scripts/review_join_source03_comparisons.py --run "$RUN"
.venv/bin/python scripts/validate_join_source03_review.py --run "$RUN"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest \
  --basetemp /tmp/join_source03_pytest_20260922
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  tests/test_join_source03.py tests/test_join_source03_presentation.py
.venv/bin/python -m compileall src scripts tests
git diff --check
```
