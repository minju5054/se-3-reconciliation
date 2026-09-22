# JOIN-SOURCE-05: explicit avoidance instruction

Question: with SOURCE04's exact visual/history inputs, does explicitly asking
native LightNav to avoid the supply cart produce a safe lateral detour?
This is **COUNTERFACTUAL MOVING-POSE HISTORY**, not an online handoff. No moving
B, MPC, GP, rigid optimization, reconciliation, rerender or traversal is allowed.
SOURCE04 remains immutable historical evidence.

## Pre-prediction protocol

Starting/fetched main: `dd138d72adb80a05019d40b2f8f57d651de23d54`.
Two unrelated Stage0 config edits are preserved by hash and excluded from Git.
SOURCE04: `data/robotless_join_source_04/persistence_20260922T050237Z/`.
Its frame paths refer to the original bank rendered in `persistence_20260922T044703Z`;
the successful run preserves byte-identical hardlinks and a technical-correction
hash manifest. Both recorded lineages are verified, not rerendered.

Exact instruction strings:

- I0, historical only: `Go to the tall blue shelving unit and stop.`
- I1: `Avoid the supply cart and continue straight down the hallway.`
- I2: `Avoid the supply cart and continue to the end of the hallway.`

All sixteen delivered frame IDs, poses, timestamps, JPEG bytes, camera, scene,
lighting, cart transform and masks/depth are SOURCE04's. H=16, K={0,1,2,4,8};
only the last K frames have the cart present. First15 requests are buffer-only
with empty instruction; only request15 contains the new instruction. Every
condition uses a new official session. K1 sham repeats the entire byte-identical
input in another session. No I0 rerun.

Order: I1_K0, I1_K1, I1_K1_SHAM, I1_K2, I1_K4, I1_K8, then the same six for I2.
Maximum12 terminal scientific predictions and180 buffer restores, retry count0.
The official server's synthetic startup warmup is counted separately and is not
scientific evidence. All scientific responses, including STOP and failures,
are retained; no replacement condition. Saved-only validation invokes no model.

Official source `c6f40e3220edbf7011e4f17eaf2c865416737d4d`, checkpoint revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, task VLN (`vlnce_traj`), same vLLM
backend and GPU memory fraction. Explicit temperature0/top_p1/top_k0/traj_top1=0.
Before scientific sessions the actual official process argv and relevant inherited
environment are read from `/proc`, saved and gated against the declaration.
All checkpoint file hashes and scientific argv match SOURCE04; ready-file path
and process/session IDs are run-specific. SOURCE04's declared launch environment
is preserved, but its complete historical process environment was not archived.

## Geometry and interpretation frozen before inference

Local forward/left/yaw-CCW Nx3 is preserved literally and transformed only by
SOURCE04 terminal observation pose `[19.20462152247834,23.62333631121559,
-1.5685636348381817]`. No ready-pose or B anchor. Row count is generic; waypoint
intrinsic time is null. Historical camera/scene context is counterfactual, not
newly executed robot motion.

Use unchanged Hospital direct geometry plus all8024 cart mesh triangles projected
through the original height slab. Radius.20m, required footprint-edge margin.05m,
original geometry uncertainty. Inspect the whole returned raw polyline, nodes
and swept segments without suffix deletion or an observation-to-first-row connector.
K0 is checked both in its actual cart-OFF scene and hypothetically cart-ON.
Safety is not inferred from endpoint, yaw, centerline or shortened output.

Reused SOURCE04 response comparison samples both polylines in the fixed obstacle
influence region; endpoint gaps do not count as lateral separation. Meaningful
response is interior separation>=.20m or reliable tangent/yaw difference>=20deg,
with chord reliability>=.02m. Numerical equivalence1e-6m/rad; material clearance
gain>=.02m. Safe lateral detour additionally requires actual max lateral>=.20m,
positive hallway forward endpoint, non-STOP, and full raw path clearance. This
explicit lateral requirement excludes yaw-only and shortening responses.
Complete bypass additionally reaches the cart rear plane; safe partial detours
remain separately labelled and are not proof of actual traversal.

K0 tests text-induced steering. Same-side lateral K0/ON trajectories without
meaningful influence-region difference count as substantially the same text-only
turn. Classification priority is frozen in YAML: incomplete/sham variation,
visual safe recovery, text-only steering, material gain with no safe ON, no effect,
then changes without safety gain (safe non-detour is inconclusive).

APOS uses original SOURCE04 image/mask/raster-first-hit/ground-proxy diagnostics.
Clamped coordinates are censored. No mask/depth is transmitted. The old shelf
remains a geometry label only: I1/I2 no longer ask for it, so visible/OPOS cannot
support target-grounding improvement claims. The intervention changes the whole
terminal instruction, including goal semantics, not solely the word "avoid".

## Reproduction and execution

The runner reuses SOURCE04's unchanged generic prediction worker. SOURCE05 adds
manifest preparation, exact historical-request comparison, evaluation/aggregation
and reporting. It does not modify historical numerical/source code or model files.
All source/config/input files freeze before commit/push; execution refuses dirty
frozen code or an unpushed HEAD. Outputs use exclusive creation throughout.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q \
  tests/test_join_source05.py tests/test_join_source04.py tests/test_join_source03.py \
  tests/test_join_source02_paired.py
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/source05_mpl .venv/bin/python \
  scripts/run_join_source05.py --run "$RUN" --mode init
.venv/bin/python scripts/run_join_source05.py --run "$RUN" --mode freeze
# Review, commit and normal push before the commands below.
.venv/bin/python scripts/run_join_source05.py --run "$RUN" --mode start
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 .venv/bin/python \
  scripts/run_join_source05.py --run "$RUN" --mode execute
.venv/bin/python scripts/run_join_source05.py --run "$RUN" --mode stop
MPLCONFIGDIR=/tmp/source05_mpl .venv/bin/python scripts/report_join_source05.py --run "$RUN"
MPLCONFIGDIR=/tmp/source05_mpl .venv/bin/python scripts/validate_join_source05.py --run "$RUN"
.venv/bin/python -m compileall src scripts tests
git diff --check
```

No prediction results are claimed in this pre-execution protocol. Final saved
results, calls, limitations and Git identities are appended after the fixed run.

## Technical pre-image correction, before scientific outputs

Initial freeze `230e3934600737aad2099b5ac2488f1bf281bfab` was pushed. Run
`instruction_20260922T103000Z` opened12 independent sessions but sent **zero
images / zero buffer requests / zero scientific predictions**. All workers
failed on `future frame cannot enter request history`: SOURCE04 historical
capture monotonic values belong to an earlier host-clock domain and exceed
the current host monotonic reading. The original online same-boot guard is
correct for live acquisition, but is inapplicable to cross-boot saved replay.
The official server did one separately counted synthetic startup warmup.
Every failed log/result, original freeze and source hash remains preserved.

A SOURCE05-only `SavedReplayHistory` adapter retains the original history state
machine, sequence and official sampler. It authenticates each pre-frozen frame
identity/bytes/pose/source timestamps instead of comparing capture and request
monotonic values across host clocks. It never rebases or fabricates capture or
send timestamps. Snapshot fields explicitly label the two domains; capture-to-
request age is N/A. Actual send/receipt and RTT retain the current real clock.
The existing online worker, external source and SOURCE04 are not modified.
Unit parity checks match the original state machine's frame/order/sampler output
when its same-boot condition holds; a separate test exercises reversed clock
epochs. First15 empty buffers/terminal instruction and all model inputs remain
unchanged. This technical correction is committed/pushed before any scientific
prediction, with a new exclusive run; no response-driven retry or tuning.

## Completed result

**No lateral bypass was generated.** Explicit wording produced a different,
useful response type: safe short forward chunks and, in I2_K8, STOP. Neither
must be labelled successful obstacle traversal or a bypass source. The frozen
classification is **INCONCLUSIVE** (both instructions:
`INSTRUCTION_EFFECT_INCONCLUSIVE`), specifically the predeclared **safe
non-detour** branch. Coverage is complete and measurements are conclusive:
six of eight non-sham ON arrays pass raw geometry, including one STOP; zero
qualify as visual-conditioned lateral detours. The outcome does not fit
"all remain unsafe" or "no safety gain", and the taxonomy was not changed.

Scientific run: `data/robotless_join_source_05/instruction_20260922T125204Z/`.
Execution freeze, pushed before all12 predictions:
`51d6d39d33600bfa24fbdc78178b5de55ac885d8`.
Initial/failed pre-image freeze: `230e3934600737aad2099b5ac2488f1bf281bfab`.
Starting SHA: `dd138d72adb80a05019d40b2f8f57d651de23d54`.
Final report commit is recorded in local `git_completion.json` after push.

### Input and model identity

All32 historical JPEGs and their32 masks/32 depths match SOURCE04 hashes.
The32 paired bank images were not rerendered or enhanced. All12 conditions
preserve exact16 frame IDs, chronology, original capture values and poses.
**All192 scientific `next` payloads reproduce SOURCE04 literally after removing
only the instruction field; first15 instruction strings are empty and the
terminal one equals exactly I1/I2.** Independent session/login identities and
current request clocks intentionally differ. Historical I0 is not rerun.

`generation_actual.json` records live `/proc` argv/environment: task vln,
temperature0, top_p1, top_k0, traj_top1=0, CUDA_VISIBLE_DEVICES=0. Scientific
argv (apart from ready-file path), pinned source and all16 checkpoint files
match SOURCE04. The source revision/checkpoint specified above are unchanged.
Formal exactness covers saved scientific inputs and declared generation;
SOURCE04 did not archive every historical inherited environment variable, so
complete retrospective process-state equivalence cannot be certified.

### Every historical/new condition

Clearance is footprint-edge metres; required>=.05m, radius.20m. Negative values
are physical footprint overlap under the original checker, not exact signed
mesh penetration depth. K0's **actual scene is OFF and safe**; its ON column
is explicitly hypothetical. Max lateral is in **millimetres**, yaw in degrees.
Arc starts at first returned row, not the observation connector. All returned
arrays happened to contain10 rows; code and tests retain generic N.

| Condition | Safe actual scene | Safe cart ON | Min edge m (ON) | Max lateral mm | Raw arc m | Final yaw deg | STOP |
|---|---|---|---:|---:|---:|---:|---|
| I0_K0 | true | false | -.200000 | .393 | 1.356219 | .0384 | false |
| I0_K1 | false | false | -.143250 | 6.376 | .565197 | -.1703 | false |
| I0_K2 | false | false | -.080955 | .214 | .510888 | -.0406 | false |
| I0_K4 | false | false | -.111384 | 5.876 | .545469 | -17.0145 | false |
| I0_K8 | false | false | -.111384 | 5.876 | .545469 | -17.0145 | false |
| I1_K0 | true | false | -.200000 | .393 | 1.356219 | .0384 | false |
| I1_K1 | false | false | -.080955 | .214 | .510888 | -.0406 | false |
| I1_K1_SHAM | false | false | -.080955 | .214 | .510888 | -.0406 | false |
| I1_K2 | false | false | -.080955 | .214 | .510888 | -.0406 | false |
| I1_K4 | true | true | .275466 | .748 | .133174 | -.1909 | false |
| I1_K8 | true | true | .275466 | .748 | .133174 | -.1909 | false |
| I2_K0 | true | false | -.200000 | .393 | 1.356219 | .0384 | false |
| I2_K1 | true | true | .275466 | .748 | .133174 | -.1909 | false |
| I2_K1_SHAM | true | true | .275466 | .748 | .133174 | -.1909 | false |
| I2_K2 | true | true | .186459 | .678 | .258962 | -.4270 | false |
| I2_K4 | true | true | .469362 | 1.035 | .073596 | .0858 | false |
| I2_K8 | true | true | .575890 | .000 | .000000 | .0000 | true |

Both K1 shams match their corresponding original **raw array and raw text
literally**. All scientific conditions completed on their first image/prediction
attempt. There was no scientific-output retry, threshold change or next K.

### Mechanism supported by the paired records

K0 trajectories are literally identical across I0/I1/I2 (action codes0/30/85).
Mentioning the cart without a visible cart does not induce a lateral turn in
this control. Pointing/OPOS tokens do change, reinforcing that equal/different
pointing alone cannot stand in for trajectory evidence.

With cart visible, the instruction affects returned travel extent:

- I1_K1 gains.062295m minimum clearance over I0_K1 but remains unsafe;
  I1_K2 is literally I0_K2's trajectory, with no clearance change.
- I1_K4/K8 gain.386850m and terminate about.300m forward of observation,
  still.450645m before the cart front plane. They are short, nearly straight
  and safe, not bypasses. I2_K1 produces exactly this same array.
- I2_K2 gains.267413m; I2_K4 gains.580746m and ends only.098573m forward,
  .651929m before the cart front plane. I2_K8 gains.687273m by emitting the
  official explicit STOP response, decoded to ten unchanged zero poses (no motion).

No new ON path has max lateral>=.0011m, let alone the frozen.20m criterion.
No returned path reaches the cart rear plane in a safe lateral detour.
SOURCE04 influence-region lateral/yaw tests are below threshold or N/A when
the shortened/STOP path no longer supplies a reliable interior comparison.
The short outputs' changed endpoints are not counted as lateral evidence.
Thus visual cart presence matters beyond text alone for **shortening/STOP**,
but the strong visual-conditioned safe-detour test fails in every condition.
This is not `INSTRUCTION_ONLY_STEERING`: the OFF controls do not turn.

### APOS and raw token trace

| Condition | APOS px | Clamped/state | RVQ l0/l1/l2 |
|---|---|---|---|
| I1_K0 | [235,165] | false/point | 0/30/85 |
| I1_K1 / sham / K2 | [235,265] | true/point | 64/217/220 |
| I1_K4 | [235,265] | true/point | 4/253/124 |
| I1_K8 | [245,265] | true/point | 4/253/124 |
| I2_K0 | [245,165] | false/point | 0/30/85 |
| I2_K1 / sham | [245,265] | true/point | 4/253/124 |
| I2_K2 | [245,265] | true/point | 205/224/215 |
| I2_K4 | [235,265] | true/point | 43/93/65 |
| I2_K8 | N/A | false/stop | 6/122/174 |

All non-STOP ON APOS remain bottom-clamped and therefore censored. Their
reported pixels lie outside the cart mask but within its horizontal span;
saved raster first hits are floor. Boundary ground proxies are not exact
model metric targets or proof of free passage. Same coarse APOS can accompany
unsafe and safe-short action arrays. OPOS/visible are preserved but no shelf
grounding comparison is valid under the changed hallway goal semantics.

### What this establishes and leaves unresolved

The entire **instruction intervention** changed the model's geometry under
byte-identical visual history and recovered raw clearance in several conditions.
It did **not** produce a collision-free lateral bypass. Saying the old failure
was simply caused by not requesting avoidance is not supported as a sufficient
explanation for missing bypass behavior. Conversely, saying explicit wording
has no effect or never produces safe raw geometry is contradicted by this run.
Only two goal/wording variants and one frozen counterfactual scene were tested.

There is **no bypass-qualified source for later moving online confirmation**.
Five unique-condition non-STOP short safe chunks are preserved as observations,
plus one STOP. They are not relabelled unsafe, but they do not provide the
intended safe route past the cart. Internal recognition/spatial reasoning/action
causes remain unobserved. Whether subsequent chunks would progressively bypass
the cart is untested; no further calls, online B or reconciliation are added.

### Calls, compute, validation and review

| Scope | Actual count/time |
|---|---:|
| Scientific terminal predictions | 12 |
| Buffer-only requests | 180 |
| Completed scientific sessions | 12 |
| Earlier failed sessions / image requests | 12 / 0 |
| Official synthetic server warmups, both runs | 2 |
| New render / MPC / GP / rigid / rollout / reconciliation | 0 / 0 / 0 / 0 / 0 / 0 |
| Actual model/MPC calls in tests or validators | 0 |
| Terminal RTT sum | 3.961543s |
| Scientific schedule start to last session close | 62.158958s |
| Successful-run server startup / lifetime | 16.511266s / 132.615513s |
| Earlier server startup / lifetime | 18.522993s / 121.615387s |

RTT is nested in these wall times; they must not be summed. SOURCE05's local
worker wall timer excludes audited checkpoint/source bootstrap (sum5.084114s),
whereas SOURCE04 included it; those fields are not directly comparable.
`aggregate/timing_detail.json` retains original start/close clock keys and hashes.

Final relevant SOURCE02–05, history, acquisition and SE2 tests: 193 passed
(2.57s), including the safe-short/STOP regression; no real model/MPC calls. Compileall and diff whitespace checks pass. Authoritative
saved-only validator passes219 checks, rechecking full preserved source hashes,
all192 payloads, original geometry/APOS, arrays/transforms, summary/CSV and PNG
numeric sidecars. SOURCE04 authoritative revalidation passes4686 checks.
`validation_completion.json` additionally checks failed-run preservation and
clarified presentation; no new inference. Scientific outcomes do not drive
artifact validation PASS/FAIL.

The original 12 figures remain unchanged. A presentation-only finalizer adds
one response table to distinguish actual OFF safety from hypothetical ON,
and actual output change from the frozen meaningful-detour threshold. Numerical
records/classification remain unchanged. All 13 PNGs have source/numeric sidecars;
raw RGB, checkpoint and environment are excluded from the review ZIP.

Review: `index_final.html`, `review_bundle_final.zip` under the scientific run.
Primary plots: `review/trajectory_K1.png`, `trajectory_K4.png`,
`clearance_vs_K.png`, `response_geometry_clarified.png`.
Finalizer command, after the original reporter and validator:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/source05_mpl .venv/bin/python \
  scripts/finalize_join_source05.py --run data/robotless_join_source_05/instruction_20260922T125204Z
```

Operational work is complete. Primary scientific classification: **INCONCLUSIVE**
— safe shortening/STOP observed; safe lateral bypass not recovered.
