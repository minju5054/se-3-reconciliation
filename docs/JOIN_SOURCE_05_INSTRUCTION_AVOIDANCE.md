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
