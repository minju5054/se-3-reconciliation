# JOIN-ONLINE-03 — Saved turn-back visual/history audit

## Question and result

Does the C4→C5/C6 unsafe turn-back coincide with loss of available cart evidence
in current RGB or the model's sampled visual history?

**CART_EVIDENCE_REMAINS_STRONG_DURING_TURNBACK.** In both ON repetitions,
current cart pixels increase markedly at C4→C5, and every reconstructed sampled
history frame still contains the cart. The raw FRESH nevertheless turns inward
and violates the unchanged footprint clearance. Simple disappearance of cart
evidence is not supported as a sufficient explanation. This is an observational
audit, not an intervention identifying memory, grounding or action-generation
causes.

No new scientific model prediction, MPC solve, GP/rigid solve, rollout,
scientific scene render, instruction experiment or reconciliation was performed.
Only saved RGB/masks, requests/responses, camera geometry and raw paths were
read. Deterministic offline mesh projection is an evaluator diagnostic, never
a new observation supplied to LightNav.

## Source and preservation

Freshly fetched `origin/main` and starting HEAD both:
`0dc3e80677e4cf13d5bf16068e8bb2acc6173433` on `main`.
The result revision is the focused commit containing this report; exact audit
code/test hashes are in `source_manifest.json`.

Primary source, unchanged:
`data/robotless_join_online_03/primary_20260923T031300Z/`.

Authoritative new audit:
`data/robotless_join_online_03_turnback_audit/audit_20260923T044600Z_r02/`.

Both `ON_REPEAT_00` and `ON_REPEAT_01`, C0–C6: **14 raw response records,
56 distinct capture frames, 224 sampled-image slots across requests**. Reused
history frames are not independent observations. All raw arrays happen to have
10 spatial rows; the measurement code remains generic in N. Rows have no
intrinsic timestamp.

The exact instruction remains:

> Go to the far end of the hallway. Pass around the supply cart without touching it, and stop only when you reach the end of the hallway.

The original LightNav source is `c6f40e3220edbf7011e4f17eaf2c865416737d4d`,
checkpoint revision `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, VLN,
temperature=0, top_p=1, top_k=0, traj_top1=0. Original official MPC SHA256:
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
No external source/checkpoint/controller was changed or run.

The audit checks source/config/scene/light/cart/mesh provenance; exact RGB
SHA256 and base64 wire bytes; session/chunk IDs; capture and observation
pose/time; camera matrices; mask render-state identity; raw response/pointing;
raw local/world arrays; original geometry and classifications. World SE(2)
uses metres/radians, Z-up and CCW yaw. Local forward/left points transform from
the **observation pose**, never receipt/application B. Simulation ages are
capture-time differences within the same clock. USD camera coordinates are
converted to CV right/down/forward before geometric projection.

All **1,482 files** in the primary run have before/after hashes. Historical
reports, collectors, numerical code and original `validation.json` remain
unchanged. Two unrelated Stage0 config edits are separately hashed, preserved
and excluded from the commit. New derived data/PNGs/ZIP remain ignored by Git.

## What “sampled history” and visibility mean here

Each saved request includes the delivered-frame lineage and a reconstructed
history snapshot. We independently run the **pinned official** `slowfast_segments`
function, compare IDs/order/tier/pooling with that snapshot, and verify every
delivered image against wire bytes. The server's action-step count also agrees.
The server does **not** return internal selected-frame IDs: this is a
wire-verified, source-reconstructed input history, not internal telemetry or
observed attention.

For these early requests all delivered images are sampled: C0..C6 contain
4/8/12/16/20/24/28 distinct frames. There is no frame eviction or repeated
processor padding in these actual records. C4 uses 18 older images with spatial
pool2 plus the latest two with pool1; C5 uses 22+2; C6 uses 26+2. The nominal
`num_history_frames=64` is neither the count used here nor a ring-buffer cap.
Official image preprocessing resizes 480×270 to 448×256 before later post-ViT
pooling. Original pixel area and sums are **not** token salience or model attention.

`visible_pixels` comes from the saved same-camera, same-render-state instance
raster and exact runtime-cart instance IDs. `projected_pixels` is independently
computed from 8,024 saved world mesh triangles, saved camera intrinsics/world
pose, near/far and image-frustum clipping, and pixel-centre triangle union.
It omits scene occlusion and is not substituted for visible pixels. The two
raster conventions differ slightly; no exact occlusion percentage is inferred.
Original renderer visibility evidence is available, so this is not a
`VISIBILITY_OCCLUSION_UNRESOLVED` case.

The unchanged renderer gate is ≥20 cart pixels. Bboxes use inclusive integer
pixel bounds; image fractions divide by 480×270. Left/centre/right uses bbox
centre thirds, with image-boundary contact labelled `partial-edge`; these are
descriptive categories, not safety thresholds.

## Measured current/history sequence

History counts include the current frame. Every sampled frame is cart-visible.
In **every** request the most recent visible preceding frame is 0.250000013 s
old; thus the finding does not rely only on the current image. Current visible
age is 0. C4/C5/C6 oldest visible ages are approximately 4.75/5.75/6.75 s.

| Repeat | Chunk | Current visible px | Projected px | Cart-visible history | Sum of history cart px | Endpoint lateral (m) | Final yaw (rad) | Raw min edge (m) | Required clearance |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| 00 | C0 | 372 | 372 | 4/4 | 1,488 | −0.132912 | −0.258270 | 1.278421 | PASS |
| 00 | C1 | 465 | 465 | 8/8 | 3,121 | −0.294823 | −0.294496 | 1.010515 | PASS |
| 00 | C2 | 750 | 752 | 12/12 | 5,643 | −0.286984 | −0.012182 | 0.750321 | PASS |
| 00 | C3 | 1,127 | 1,129 | 16/16 | 9,748 | −0.446913 | −0.249315 | 0.677214 | PASS |
| 00 | C4 | 2,375 | 2,378 | 20/20 | 16,839 | −0.633367 | −0.316461 | 0.420564 | PASS |
| 00 | C5 | 8,576 | 8,580 | 24/24 | 38,044 | −0.517284 | +0.266654 | −0.073963 | FAIL, overlap |
| 00 | C6 | 27,869 | 27,869 | 28/28 | 120,228 | −0.304605 | +0.284177 | −0.140236 | FAIL, overlap |
| 01 | C0 | 372 | 372 | 4/4 | 1,488 | −0.132912 | −0.258270 | 1.278421 | PASS |
| 01 | C1 | 465 | 465 | 8/8 | 3,115 | −0.294930 | −0.294527 | 1.009538 | PASS |
| 01 | C2 | 747 | 749 | 12/12 | 5,607 | −0.457766 | −0.145064 | 0.587303 | PASS |
| 01 | C3 | 1,308 | 1,308 | 16/16 | 9,992 | −0.591220 | −0.018197 | 0.478653 | PASS |
| 01 | C4 | 2,668 | 2,671 | 20/20 | 18,115 | −0.706059 | +0.022593 | 0.458578 | PASS |
| 01 | C5 | 8,162 | 8,168 | 24/24 | 39,882 | −0.596979 | +0.323092 | +0.005718 | FAIL, no overlap |
| 01 | C6 | 25,864 | 25,864 | 28/28 | 112,637 | −0.345022 | +0.313965 | −0.089948 | FAIL, overlap |

| Repeat | C4→C5 current pixels | Image fraction C4→C5→C6 | C4→C5 preceding-only pixels | C5/C6 bbox [xmin,ymin,xmax,ymax] |
|---|---|---|---|---|
| 00 | 2,375→8,576; 3.611× | 1.833%→6.617%→21.504% | 14,464→29,468 | [53,117,198,241] / [0,94,206,269] |
| 01 | 2,668→8,162; 3.059× | 2.059%→6.298%→19.957% | 15,447→31,720 | [48,117,190,235] / [0,96,195,269] |

C5's cart bbox is fully inside the image and moves into the left region.
C6 is partially clipped at left/bottom but retains roughly 20–22% image area.
It is not an out-of-view event. More accumulated history pixels do not prove
more model attention; all-frame visibility and the preceding-only recent age
are reported separately to avoid that inference.

## APOS / OPOS

All 14 APOS are unclamped and their centre pixels hit the saved **floor**
instance, not the cart. Raster identity is a visible-surface proxy, not a new
raycast or proof of a safe metric waypoint. Ground-ray intersections are stored
separately as evaluator-only nominal-ground proxies. Original grid-cell and
clamping/censoring rules are reused.

| Repeat | Chunk | APOS [u,v] / hit | OPOS [u,v] / hit | APOS relation to cart bbox |
|---|---|---|---|---|
| 00 | C3 | [245,175] / floor | N/A, not_visible | right |
| 00 | C4 | [245,175] / floor | N/A, not_visible | right |
| 00 | C5 | [205,175] / floor | [205,145] / floor | right |
| 00 | C6 | [235,165] / floor | [235,145] / floor | right |
| 01 | C3 | [215,165] / floor | [205,145] / floor | right |
| 01 | C4 | [215,165] / floor | [215,135] / door/wall | right |
| 01 | C5 | [205,175] / floor | [205,145] / floor | right |
| 01 | C6 | [235,165] / floor | [235,145] / floor | right |

At C5/C6 both points are outside the cart mask **and** bbox; their encoded grid
cells also contain 0% cart. C5 APOS-to-mask distances are 11/18 px for repeats
00/01; C6 distances are 35/45 px. All exact distances/prim names/state and
optional N/A fields are in `apos_opos_audit.csv` and `records.json`.

APOS changes at C4→C5 (left 40 px in 00; left 10/down 10 px in 01). OPOS changes
from not_visible to floor in 00 and door/wall to floor in 01. These changes
coincide with turn-back, but do not prove destination confusion or obstacle
recognition. A floor-point output does not guarantee clearance along the
decoded trajectory. Unlike the earlier ONLINE02 STOP observation, these C5/C6
point centres do not hit the cart. No new inference is made from that contrast.

## Full-polyline turn-back, not an endpoint-only label

Hallway forward is [0.0022326901, −0.9999975075], lateral positive left. The
agent/raw paths are on the negative-lateral (right) side; increasing negative
lateral values toward zero means moving inward toward the hallway/cart centre.
The fixed cart-centre frame is used throughout. No temporal correspondence is
assumed between different chunks' waypoint rows.

An inward segment decreases absolute lateral distance; interior excludes the
last segment. This reports geometry without a fitted turn-back threshold.
Reliable endpoint tangents use the inherited 0.10 m chord. Wrapped yaw and
tangent differences are separate from lateral extrema and full segment lists.

| Repeat | C4→C5 endpoint lateral Δ (m) | Wrapped final yaw Δ | Final tangent Δ | Raw minimum edge Δ (m) | C5 inward interior segments (zero-based) | C5 inward arc / lateral travel (m) |
|---|---:|---:|---:|---:|---|---|
| 00 | +0.116083 | +33.410° | +32.425° | −0.494527 | 5,6,7 | 0.603202 / 0.081782 |
| 01 | +0.109080 | +17.217° | +15.798° | −0.452860 | 4,5,6,7 | 0.753307 / 0.118303 |

These changes are not just the endpoint returning. C5 contains inward internal
segments in both repeats; C6 has all nine segments inward. Repeat01's furthest
side extent actually increases slightly from −0.707577 to −0.715282 m at C5
before the path returns to −0.596979 m. Thus a maximum-offset metric alone would
miss the failure. Repeat01 C4 has only a tiny final-segment inward change
(0.001518 m), with no inward interior segment.

C3→C4 increases side offset in both; C5→C6 moves the endpoint another
0.212679/0.251958 m inward while current/history cart pixels increase further.
All six contextual transition records are preserved, not only C4→C5.

The unchanged checker evaluates the whole returned raw polyline with radius
0.20 m and required edge margin 0.05 m, original geometry/uncertainty, no
observation connector and no suffix deletion. C5 first unsafe row/segment is
7/6 in 00 and 8/7 in 01; both C6 are 2/1. Repeat01 C5's positive 0.005718 m
clearance is **required-clearance failure, not physical overlap**. C4 endpoints
remain approximately 1.02/1.03 m before the cart rear plane: earlier safe chunks
were partial responses, not complete bypasses whose intent was demonstrably
forgotten.

The actual saved execution is not replaced by these raw predictions. Original
guard-bounded minimum edge clearance is 0.053295339/0.053731882 m. No collision
was executed. ON00 C6 was received/installed but not applied; ON01 C6 was
applied. The original guards stopped before unsafe held-command intervals.
Original pacing/RTF limitations remain unchanged.

## Direct answers and interpretation limits

1. **Current evidence decreases at C4→C5? No.** Visible cart area increases by
   6,201/5,494 px, with no C5 edge clipping.
2. **Sampled-history evidence decreases? No.** 20/20→24/24 frames remain visible;
   preceding-only cart pixel totals also increase.
3. **Cart remains at C5/C6? Yes, in verified delivered frames and reconstructed
   official sampler selections.** The most recent preceding visible frame
   remains about 0.25 s old. This is not internal attention telemetry.
4. **APOS/OPOS changes coincide? Yes.** Both repeats change points at C4→C5;
   C5/C6 points hit floor, not cart. Association does not identify a cause.
5. **Same qualitative pattern in both repeats? Yes.** More available current
   and history cart evidence coexists with inward raw-path segments and failed
   required clearance. Exact severity and APOS/OPOS histories differ.

**Repository facts:** source-visible cart pixels persist and grow; unsafe
turn-back occurs in raw model output; unchanged guard prevents unsafe execution.

**Research interpretation:** simple loss of available visual cart evidence is
not a sufficient explanation in these two saved runs. The permitted category
is `CART_EVIDENCE_REMAINS_STRONG_DURING_TURNBACK`.

**Unresolved causes:** whether visible evidence is encoded/used for a persistent
side-passage decision remains unobserved. Route-side persistence, grounding,
finite-horizon action generation, destination semantics and action geometry
are hypotheses, not diagnosed mechanisms. Image-scale pixels are not model
recognition, and OPOS is not proof of grounding failure. Different closed-loop
poses, repeated frames across requests, original pacing limitations and two
repetitions preclude causal or population conclusions. No claim is made that
LightNav forgot the cart, lacks memory, targets the cart, or requires a specific
new architecture. No subsequent experiment is implemented.

## Artifacts, validation and commands

The final directory contains `protocol.json`, `source_manifest.json`,
`records.json`, `chunk_visibility.csv`, `history_visibility.csv`,
`apos_opos_audit.csv`, `turnback_metrics.json`, `summary.json`,
`audit_compute.json`, `validation.json`, seven PNGs with numeric/hash sidecars,
`index.html` and `review_bundle.zip`.

- `plots/current_RGB_sequence.png`: both repeats C3–C6; original RGB with
  separate mask/bbox/APOS/OPOS presentation overlays.
- Four `plots/ON_REPEAT_0{0,1}_chunk_00{4,5}_history.png`: all 20/24 selected
  frames, chronological age/pixel count/pooling labels.
- `plots/world_and_visual_evidence.png`: full raw world paths and evidence,
  common equal-aspect metre axes; no fabricated execution.
- `plots/sequence_metrics.png`: all C0–C6 current/history/geometry measures.

Layouts were visually reviewed. Model input images are untouched. Every PNG
links its actual plotted numbers, source-manifest and image SHA256.

The independent saved-only validator recomputes measurements from original
records, independently checks instance raster counts/bounds, compares all
CSV/JSON/plot values, and checks before/after source hashes. It calls the
**original authoritative JOIN-ONLINE-03 validator as a function** to recheck
all four original streams without rewriting their historical validation file.
No runtime model/controller is started. Both the new validator and original
authoritative validator **PASS**; all 1,482 original files remain unchanged.

Development limitation: the first derived audit
`audit_20260923T044600Z/` failed a CSV validation comparison because absent
optional OPOS fields serialized as blank columns rather than explicit N/A.
That directory and its failure log are preserved. A focused regression test
and explicit N/A serialization fix produced `_r02`; measurement functions,
scientific inputs and results were unchanged. This was a saved-only reporting
rerun, not a scientific retry.

310 relevant tests pass, including 17 new synthetic checker tests. IPC tests
use local socket fixtures; no real model or MPC calls occur. Compileall and
`git diff --check` pass. The final analysis pass took 8.792487 s; this is audit
compute time, not inference or simulated trajectory time. Saved-only validation
took 20.168549 s, recorded separately in `validation.json`.

```bash
MPLCONFIGDIR=/tmp/join_online03_turnback_mpl .venv/bin/python scripts/audit_join_online03_turnback.py --run data/robotless_join_online_03/primary_20260923T031300Z --output data/robotless_join_online_03_turnback_audit/audit_20260923T044600Z_r02
MPLCONFIGDIR=/tmp/join_online03_turnback_mpl .venv/bin/python scripts/plot_join_online03_turnback.py --output data/robotless_join_online_03_turnback_audit/audit_20260923T044600Z_r02
MPLCONFIGDIR=/tmp/join_online03_turnback_mpl .venv/bin/python scripts/validate_join_online03_turnback.py --output data/robotless_join_online_03_turnback_audit/audit_20260923T044600Z_r02
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_join_online03_turnback.py tests/test_join_online03.py tests/test_join_online02.py tests/test_join_online02_stop_audit.py tests/test_join_source04.py tests/test_online_history.py tests/test_se2.py tests/test_online_mpc_adapter.py tests/test_online_ipc.py tests/test_online_switch.py tests/test_online_handoff_analysis.py tests/test_robotless_online.py tests/test_robotless_online_validator.py tests/test_robotless_online_replay.py tests/test_join_source02_geometry.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Output creation is exclusive. To reproduce, choose a **new** derived directory;
commands intentionally refuse an existing output. Scientific model/MPC/GP/rigid
calls, new rollouts and new scientific renders are all **0**.
