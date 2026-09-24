# BLIND_CORNER_SOURCE_ACQUISITION_02

## Protocol

Second and final bounded blind-left-corner source search for now. The question is whether a different, independently declared Hospital corner can hide a static cart inside a plausible finite turning OLD corridor while leaving a safe escape band and enough interaction length. Source acquisition only; no reconciliation, optimizer, controller comparison, Native continuation or observation-start control.

Starting fetched main/origin: `f30cc25557063a37a0c38700851e8024f9b01f13`. Unrelated edits to `configs/stage0_jackal_controller_validation.yaml` and `configs/stage0_lightnav_single_chunk.yaml` are preserved and excluded. Acquisition 01 raw and derived artifacts are hash-preserved. C02 remains **not qualified**, with finite OLD unobstructed, FRESH wall/trim margin failure and unestablished obstacle-region mismatch.

Output: `data/blind_corner_source_acquisition_02/preflight_20260925T014000Z/`. The directory label is an identifier; actual UTC and monotonic render timestamps are stored in every snapshot.

### Model-free bank

Exactly three new exterior room-block corners, C03/C04/C05. Existing Hospital layers/meshes are unchanged. No wall creation/removal, doorway traversal, custom maze, C02 retry or additional candidate is allowed. We inspected the existing exported Hospital occupancy map and wall mesh bounds to identify these three corners; the common construction below was written to the config before rendering them. No new model output informed placement.

Canonical corner coordinates are +x outgoing and +y opposite incoming. The inner room quadrant is (+x,+y). All three candidates approach east and turn left/north. The coordinate conversion is explicitly `world = corner + outgoing*x - incoming*y`.

- Approach: canonical [-.43,.75], heading incoming.
- Cart placement: canonical [.66,-.55]; long axis parallel to the initial corner shadow boundary, relative yaw `atan2(-(.75-.09), .43)` from outgoing. This favors wall occlusion while allowing a near finite-path conflict. Actual mesh/USD transforms, not approximate rectangles, determine qualification.
- Nominal family: straight approach followed by a left quarter circle with radii {.43,.59,.75} m, then straight outgoing. Circle center is [r-.43,r-.43], straight approach length .75+.43-r. These are **model-free candidate-selection probes**, not LightNav outputs, trajectories to execute, or synthetic scientific evidence.
- Finite spatial prefix: 75th percentile of the already saved genuine corpus chunks satisfying the unchanged turning gates (arc≥.50 m, yaw/tangent excursions≥30°, reliable segment≥.02 m). The saved-only census found 1000 raw chunks, 80 passing these turning descriptors, and a bound of **1.3254158228259225 m**. All input hashes and metrics are saved. This is a design scale, not a predicted LightNav horizon or timestamp.
- Sample each nominal finite prefix at .005 m, including the exact last arc. Require at least one Hospital-safe member whose finite footprint conflicts with cart clearance .05 m. Require entry into the inherited cart-longitudinal influence slab (mesh extents ±.50 m) at least .20 m before the prefix end. No extrapolation of model output.
- Visibility preflight: fixed spatial probes every .15 m from 0 to 1.50 m on the r=.59 curve. Actual Isaac RGB/instance masks at those held poses. The first visible diagnostic probe is a spatial estimate only, **not a scheduled online crossing**.
- Reaction reserve: nominal conflict arc minus the expected visibility-probe arc ≥.45 m, a geometric proxy based on previously observed .28 m inference travel plus .17 m design reserve. It is not a worst-case latency or executable-motion guarantee.
- Escape band: one declared outer polyline through canonical [-.43,.75],[-.85,.75],[-.85,-1.55],[2.4,-1.55]. Exact swept footprint-edge separation ≥.10 m against Hospital **and** cart, with known workspace; no AABB safety substitution. The last 1.05 m outgoing segment must also pass .10 m. The .45 m motion design reserve leaves .60 m spatial room. This is a connected free-space probe, not a planner or controller reference.
- Scientific radius .20 m and margin .05 m remain unchanged. Candidate-selection .10 m is deliberately more conservative. Cart/scene projected overlap area must be ≤1e-6 m². The inherited [.05,.65] m body height band remains unchanged.

The common analytic rules and three exact corner transforms are in `configs/blind_corner_source_acquisition_02.yaml`. Ranking uses only geometry: all gates first, then descending minimum combined bypass reserve, then lexical candidate ID. Run every geometry-qualified candidate once; select the first fully qualifying source afterward. If none passes preflight, stop with `BLIND_CORNER_02_GEOMETRY_UNAVAILABLE` and make **zero scientific calls**. No placement/instruction/threshold repair after outcomes.

### Intended scientific contract, conditional on geometry

Exact instruction for all candidates:

> Turn left at the corner and continue down the hallway. Avoid obstacles and keep going.

Static cart from episode initialization in renderer and oracle; authored/computed visibility and full world mesh remain constant. Reveal must arise from camera motion and a real wall intersection before the cart, with its representative ray inside the camera frustum. Initial OLD image has <20 cart pixels. The exact first scheduled 4 Hz <20→≥20 frame is the only allowed FRESH; no queue substitution.

One genuine OLD and at most one FRESH per candidate, no retry/extra prediction. OLD must turn left, be nonSTOP and Hospital-safe, while the added cart makes that same finite raw path margin-invalid. FRESH must be nonSTOP, whole-path safe, outgoing-progressing and at least .60 m, with meaningful interior lateral (.10 m), reliable tangent or pose-yaw (20°) change **inside** the frozen influence region. Empty region is unestablished, never replaced by endpoint mismatch. At B, actual v_minus>.20 m/s, |omega_minus|≥.10 rad/s, observation→B travel≥.02 m, safe B, and ≥4 remaining original rows/.60 m arc. Incoming physical control, worker memory and first applied FRESH command are distinct saved quantities.

Reuse unchanged 60 Hz exact integration / 10 Hz official MPC / 4 Hz capture, `minimum_wall_step_no_catchup_v1`, request-local RTF [.8,1.2], max stall≤.25 s and zero bursts. Abort-only guard does not steer. All chunks remain observation-anchored, never B-anchored; raw rows have no intrinsic timestamps. No downstream execution after sealing. Scientific code/config/tests and eligible order must be committed and normally pushed before any model calls.

Lighting/camera are inherited from OSA03/Acquisition 01: authored Hospital plus white rectangular fill intensity1000/exposure0/4×10 m at each corner XY,z2.6; 480×270, ~112.2° hFOV, camera height .65 m/forward offset .09 m. No image enhancement or overlays in RGB. Instance masks are evaluator-only from the same render product/state. Geometry probes do not advance physics or represent online movement.

## Commands

```bash
.venv/bin/python scripts/run_blind_corner_source02.py --run data/blind_corner_source_acquisition_02/preflight_20260925T014000Z --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_blind_corner_source02.py tests/test_blind_corner_source.py
# Existing Isaac runtime; sanitize inherited CUDA/ROS/Python library environment as Acquisition 01.
/home/gpuadmin/isaacsim/python.sh scripts/isaac/blind_corner_preflight02.py --run data/blind_corner_source_acquisition_02/preflight_20260925T014000Z
.venv/bin/python scripts/run_blind_corner_source02.py --run data/blind_corner_source_acquisition_02/preflight_20260925T014000Z --mode audit
```

## Pre-inference geometry result / frozen execution bank

All 33 real diagnostic poses were rendered once (11 per candidate). Cart presence/transform stayed fixed within each candidate; full initial/final triangles match exactly. All three initial images contain zero cart pixels. Cart-center rays are inside the camera and first hit the declared existing Hospital wall at ~.651710 m before cart. Actual scientific first visibility must still be re-established from live scheduled images.

| Candidate | Approach [x,y,yaw] | Desired cart [x,y], yaw | Initial / first visible pixels | Finite nominal r=.75: Hospital / cart edge m | Bypass Hospital / cart / combined m | Result |
|---|---|---|---|---|---|---|
| C03 | [3.71,11.9182260132,0] | [5.01,13.0082260132], 0.577439616 rad | 0 / 947 at .15 m probe | .070474 / .024459 | .211873 / .342161 / .211873 | PASS |
| C04 | [2.4092840576,4.9199963379,0] | [3.7092840576,6.0099963379], 0.577439616 rad | 0 / 677 at .15 m probe | .055345 / .024459 | −.105809 / .342161 / −.105809 | FAIL: bypass Hospital overlap |
| C05 | [16.5100085449,9.8199877930,0] | [17.8100085449,10.9099877930], 0.577439616 rad | 0 / 677 at .15 m probe | −.200000 / .024459 | −.200000 / .342161 / −.200000 | FAIL: initial/probe/nominal Hospital overlap; no valid reaction member |

Exact yaw and matrices in declaration/cart.json are authoritative; desired center differs slightly from the rotated mesh AABB center. Added-cart mesh overlap with original Hospital obstacles is zero for all three candidates. C03 qualifying nominal finite member enters influence at .985 m and first conflicts at segment259 / arc1.295 m; the conservative same-spatial-progress reaction proxy is 1.145 m from first visible probe. These are spatial centerline descriptors, not seconds or predictions. The alternative free-space band's .211873 m combined minimum exceeds the declared .10 m. AABB bounds only identify scene corners/describe props; actual clipped meshes determine all safety values.

**Eligible scientific order: [C03].** C04/C05 have zero scientific exposure. One OLD and at most one first-crossing FRESH for C03, followed by at most .10 s postroll. No extra candidate or retry. Geometry-pass is not a claim about the actual LightNav OLD/FRESH.

## Result — Repository-confirmed facts

Scientific freeze **`309a169ffd839a442bbdccf44119f75daccfd8ac`**, normally pushed before server startup and the only scientific acquisition. C03 ran once; C04/C05 received no model/MPC calls. No retry, changed placement, additional frame substitution or threshold adjustment. **Final: `BLIND_CORNER_02_LIGHTNAV_SOURCE_NOT_QUALIFIED`**. Representative: **none**. This concludes the second/final bounded blind-corner search for now.

The saved-only validator passes, including the original online episode validator. Artifact integrity PASS does not mean source qualification PASS. Five false Boolean flags describe **four distinct missing conditions** (finite cart obstruction and its attribution are two checks of the same missing condition):

1. OLD reliable tangent excursion **28.341658° < 30°**, despite yaw excursion30.516760° and arc1.238296 m.
2. Actual finite OLD is **not cart-obstructed**: cart-only edge clearance .411010 m; Hospital-only/combined .206581 m.
3. Cart-region mismatch remains **unestablished**. FRESH has samples inside the influence slab, but their OLD projections lie outside it or at the endpoint. There are zero eligible within-region correspondences; the reported primary mismatch values are null, not zero.
4. Incoming rotation is **|omega_minus|=.007337 rad/s < .10**, although speed .8 m/s, travel .213333 m, safe moving B and timing all pass.

| Actual C03 quantity | OLD | First-visible FRESH |
|---|---:|---:|
| Raw N / STOP | 10 / false | 10 / false |
| XY arc | 1.238296020 m | 1.239975930 m |
| Waypoint yaw excursion | 30.516760° | 40.100588° |
| Net reliable tangent change | +28.298993° | +36.745895° |
| Max absolute local lateral | .137959972 m | .306844771 m |
| Outgoing +world-Y returned-path progress | +.138158372 m | +.312567730 m |
| Hospital-only edge clearance | +.206581019 m | +.168307595 m |
| Cart-only edge clearance | +.411010093 m | +.267234610 m |
| Combined whole raw clearance | +.206581019 m | +.168307595 m |
| Limiting geometry | existing Hospital corner trim | same Hospital corner trim |
| First margin-invalid segment | N/A; none | N/A; none |

Limiting prim for both paths:
`/World/Hospital/Geo_M2_TrimTwoCorner5_60/Geo_M2_TrimTwoCorner/Geo_M2_TrimTwoCorner`.
Both whole raw paths are geometrically safe at the unchanged .05 m margin. No raw prefix/suffix was deleted; the observation-to-row0 connector is not counted as returned output. FRESH continues left with a smaller descriptive arc/net-turn proxy (OLD2.507126 m, FRESH1.933424 m), but this does not establish obstacle-induced avoidance. No intrinsic row timing is assigned.

Raw OLD SHA256:
`ee21dd5fc971bcb29273d6c3b425b4ead1427697d2f371c619e87dcaf5d347f6`.
Raw FRESH SHA256:
`aa07fc7cf0b91a595ffadf25883e594e42666a194b21246a2347a8dc87531926`.
World OLD/FRESH hashes are `2b9c0326c745344358829c3a5136d532ba72372100e8a4d90baf743e4d1ba802` / `e785927b5bb6b58133a3ce13f5fa574d2250721358cecf8870fce5360cd1b323`. The transform uses each chunk's own actual observation pose, never ready/install/B.

### Visibility, execution and controller state

- OLD observation: `[3.71,11.918226013183608,0]`, cart pixels0, in-frustum cart-center ray blocked by actual `Geo_M2_DoorWall2Corner2_656/.../Section1` before cart. Initial screen center proxy [95.062988,170.182367]. Wall hit distance .651710 m.
- Actual scheduled sequence pixels: **[0,0,0,0,0,104,3374]**. The first crossing is frame000004→frame000005, state60→75, **0→104**. The exact frame000005 JPEG/hash is the FRESH wire input; later frame000006 was not substituted. A small visible mask area does not prove semantic recognition.
- FRESH observation: `[3.7766663250404275,11.918422150641886,0.004743965331897471]`.
- B at state/tick92: `[3.9899946841871037,11.919869051366652,0.007998081047978278]`.
- Physical command immediately before FRESH observation: `[.6000000287735553,.01879258552061752]`. Actual yaw change in the preceding .5 s: .0047439653 rad. Physical OLD yaw stays close to straight over this early boundary, even though the tail of its predicted chunk turns left.
- **Physical u_minus at B:** `[.8,.007337087871439335]`.
- **Worker previous_control at B:** `[.8,.3237439049587109]`.
- **First applied FRESH command:** `[.8,.3237439049587109]`. Memory and incoming physical control are not equated.
- First FRESH solve: `C03_solve_000005`, input state90 at sim1.5333334133 s, pose `[3.963328843234108,11.919659032294254,.0077535114395081975]`, previous command `[.8,.007337087871439335]`. Official selector nearest0→rows[1,2,3,4,5]. Solve4.028230 ms; application at state92. Full input/result/selection records are saved, not reconstructed from rounded report numbers.
- Observation→B travel .213333346 m over .283333348 simulation seconds. B combined edge clearance .210230734 m. Remaining original FRESH: **10 rows / 1.239975930 m** by the inherited diagnostic weighted-pose nearest rule (index0).
- Actual short executed prefix minimum clearance **.204006267 m**, no abort. After B only the frozen .100000005 s source postroll was executed; this is not a Native continuation experiment or obstacle traversal.

The static cart's actual mesh AABB center is `[5.013241690773416,13.008571482376606]` (desired [5.01,13.008226013184], root yaw .577439616145). All 108 runtime static-state checks agree; full cart triangles unchanged; authored/computed visibility stays `inherited`; oracle/render presence constant. Full USD matrices and source/actual triangles are preserved. Seven live RGB/mask pairs have same camera/state identity. Preflight poses are separate diagnostic renders and are never delivered as fabricated online history.

### Timing

| Event | Simulation s | Host UTC on 2026-09-24 |
|---|---:|---|
| OLD observation | .783333374 | 16:44:36.626776 |
| OLD request / receipt | N/A | 16:44:36.715340 / 16:44:36.937621 |
| First OLD application | 1.116666725 | 16:44:37.113203 |
| Last hidden frame000004 | 1.033333387 | saved capture timestamp |
| First visible/FRESH observation frame000005 | 1.283333400 | 16:44:37.278501 |
| FRESH request / full receipt | N/A | 16:44:37.374931 / 16:44:37.601473 |
| Ready seen / install | 1.533333413 / 1.533333413 | 16:44:37.609375 / 16:44:37.609821 |
| First FRESH command application B | 1.566666748 | 16:44:37.725394 |
| Postroll end | 1.666666754 | saved execution timestamp |

OLD RTT .222300948 s; FRESH RTT .226550932 s. Request-local RTF **.991162402**, max loop stall **.190133362 s**, catch-up/no-sleep bursts **0**. All frozen timing/scheduler gates pass. Observation→B host elapsed (~.446893 s) and simulation elapsed (.283333348 s) are different quantities; no mixed-clock subtraction or full-RTT substitution for request-local RTF. OLD remained active during FRESH inference.

### Provenance, compute and outputs

Official LightNav `c6f40e3220edbf7011e4f17eaf2c865416737d4d`, checkpoint `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, VLN, temperature0/top_p1/top_k0/traj_top1=0. Live server argv/checkpoint files/environment match the preserved native launch. Official MPC SHA256 `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`, provenance PASS. External source/checkpoint/MPC and Hospital layers unchanged.

- Scientific terminal predictions **2**: one OLD, one FRESH. Buffer-only requests **5**. One separate server warmup (~294 ms); no diagnostic retry. Native engine initialization/profiling is separately logged, not scientific predictions.
- Official MPC submissions/results **7/7**, saved solve wall sum **.075395757 s**. Terminal RTT sum **.448851880 s**. Acquisition scene/worker/episode wall **12.115906 s** (Isaac process startup separate). Preflight process wall **26.198065 s** for33 diagnostic snapshots.
- GP/rigid/graph/splice/reconciliation/continuation/trackability calls **0**. Tests and saved-only validators made **0 model/MPC calls**.
- Relevant final regression **462 passed**. Compileall and `git diff --check` PASS. C02 negative result/hash preservation, whole finite mesh checks, static obstacle/first crossing, raw/anchor/command/scheduler invariants and arbitrary N remain covered.
- `validation.json`: independently recomputed geometry and actual saved episode, all gate failures explicit. `artifact_validation.json`, `artifact_validation02.json`, `supplemental_review_validation.json`: figure numeric/hash parity, CSV/JSON parity.
- `technical_preflight/`: all C03/C04/C05 actual RGB/masks, cart triangles/USD transforms, same-state and static-state proof.
- `candidates/C03/episodes/C03/`: immutable original RGB/history/wire, OLD/FRESH local/world arrays, responses, controller events, applied commands, execution, guard and visibility records.
- `aggregate/ledger.csv`, `aggregate/ledger.json`: all three candidates; unavailable scientific fields remain blank/null for C04/C05.
- `source_bundle/manifest.json`: **entries=[], representative=null**. No qualified source bundle.
- `review/index.html`: geometry bank, all preflight pairs, actual hidden/visible pair, raw OLD/FRESH/actual-prefix plot, separated clearances and timeline.
- `review/influence_index.html`: additional saved-only influence-slab/footprint panel, with a separate numeric/hash sidecar. This descriptive addition leaves frozen execution/evaluation and primary report products unchanged.
- `review_bundle.zip`: static review package; large data and images stay ignored by Git.

Exact post-freeze commands:

```bash
# Only after commit 309a169... and normal push:
.venv/bin/python scripts/run_blind_corner_source02.py --run data/blind_corner_source_acquisition_02/preflight_20260925T014000Z --mode start
/home/gpuadmin/isaacsim/python.sh scripts/isaac/blind_corner_online02.py --run data/blind_corner_source_acquisition_02/preflight_20260925T014000Z --candidate C03
.venv/bin/python scripts/run_blind_corner_source02.py --run data/blind_corner_source_acquisition_02/preflight_20260925T014000Z --mode stop
.venv/bin/python scripts/validate_blind_corner_source02.py --run data/blind_corner_source_acquisition_02/preflight_20260925T014000Z
.venv/bin/python scripts/report_blind_corner_source02.py --run data/blind_corner_source_acquisition_02/preflight_20260925T014000Z
.venv/bin/python scripts/extend_blind_corner_source02_review.py --run data/blind_corner_source_acquisition_02/preflight_20260925T014000Z
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Isaac launches use the unchanged sanitized environment documented above. The exact regression file list/output is in `logs/regression.log`; code/config/collection/evaluation stayed byte-identical to freeze after scientific results. The additional reporting script only reads saved records and cannot trigger a scientific retry.

## Research interpretation

The stronger geometry proxy succeeded in finding one finite nominal cart conflict with a free escape band. It did **not** predict the actual LightNav OLD: that raw path stayed farther outside the corner and below the cart influence region, and the early executed boundary had almost zero incoming yaw rate. Unlike Acquisition 01 C02, C03's raw FRESH is whole-path safe, but this improvement alone does not satisfy the requested obstructed-OLD/turning-boundary source definition. A global or endpoint OLD/FRESH difference cannot replace the unestablished obstacle-region comparison.

The single remaining uncertainty is whether native LightNav can jointly produce a cart-obstructed finite turning OLD and a whole-safe differing first-visible FRESH at an already rotating boundary in an existing Hospital blind corner. This bounded bank did not resolve it; no third adaptive blind-corner search or alternative regime is implemented.

## Not demonstrated

No qualified blind-corner source, complete obstacle bypass, causal proof of obstacle recognition/avoidance, reconciliation benefit, graph superiority, improved closed-loop navigation, population prevalence or real-robot feasibility. Safe raw FRESH and valid pacing do not establish those claims. Acquisition 01 and OSA03 remain unchanged historical evidence.
