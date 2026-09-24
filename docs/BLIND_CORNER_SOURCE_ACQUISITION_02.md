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

Exact yaw and matrices in declaration/cart.json are authoritative; desired center differs slightly from the rotated mesh AABB center. All cart/initial Hospital projected overlap areas are zero. C03 qualifying nominal finite member enters influence at .985 m and first conflicts at segment259 / arc1.295 m; the conservative same-spatial-progress reaction proxy is 1.145 m from first visible probe. These are spatial centerline descriptors, not seconds or predictions. The alternative free-space band's .211873 m combined minimum exceeds the declared .10 m. AABB bounds only identify scene corners/describe props; actual clipped meshes determine all safety values.

**Eligible scientific order: [C03].** C04/C05 have zero scientific exposure. One OLD and at most one first-crossing FRESH for C03, followed by at most .10 s postroll. No extra candidate or retry. Geometry-pass is not a claim about the actual LightNav OLD/FRESH.
