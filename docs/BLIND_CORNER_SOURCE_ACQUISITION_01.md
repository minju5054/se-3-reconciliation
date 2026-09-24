# BLIND_CORNER_SOURCE_ACQUISITION_01

## Pre-scientific protocol

Question: can a static cart hidden by a real Hospital left-corner wall become visible during genuine turning OLD execution and produce a safe differing FRESH at a timing-qualified moving boundary with nonzero incoming yaw rate? This task acquires source only. No reconciliation, optimizer, method comparison, Native continuation, or observation-start control is performed.

Starting main/origin: `9bdf775e5b4dd5de2eac00efc1453a100923fefa` (fetched at task start). The two unrelated stage0 config edits are preserved and excluded from commits. Historical OSA03 and external LightNav/MPC remain unchanged.

Technical preflight: `data/blind_corner_source_acquisition_01/preflight_20260924T000000Z/`. Run directory: `data/blind_corner_source_acquisition_01/primary_20260924T054500Z/`. Directory labels are identifiers; capture records contain actual UTC/monotonic times.

Only two actual Hospital exterior room-block corners were inspected. No doorway traversal, new wall or synthetic corridor was created. Camera viewpoints in preflight are diagnostic renders, not executed trajectories or synthetic source evidence. The cart is an internal reference of `/World/Hospital/SM_SupplyCart_02a7`; evaluation projects its actual triangles in the unchanged [.05,.65] m height band, not an AABB replacement. Radius .20 m and edge margin .05 m are unchanged.

| Candidate | Approach [x,y,yaw rad] | Desired cart center [x,y], root yaw | Initial / next cart pixels | Passage edge clearance | Geometry result |
|---|---|---|---|---|---|
| C01 | [-24.1,9.4,π] | [-26.2,7.7], π | 1079 / 4107 | 0.338482 m | Excluded: already visible; center ray not wall-occluded |
| C02 | [-24.15,13.7,-π/2] | [-22.5,12.15], π/2 | 12 / 5655 | 0.333121 m | Qualified |

C02 approaches south along the west wall of the room block then turns left/east into the south corridor. Its cart-center projection is inside the camera at [56.7263,164.3204] pixels. An exact exported-triangle ray hits `/World/Hospital/Geo_M3_SideWall8/Geo_M3_SideWall/Section0` at 0.820077 m before reaching the cart; this is wall occlusion, not simply outside the frustum. The first visible diagnostic pose has 0.385569 m environment edge clearance; this is a geometric reserve, not a worst-case inference-distance guarantee. Both cart meshes have zero projected overlap area with existing Hospital obstacles. The constant cart transform is recorded as the complete USD matrix; desired center and actual mesh AABB center are distinguished.

Original Hospital layers/materials stay unchanged. The inherited neutral ceiling fill (intensity 1000, exposure 0, white, 4×10 m) is translated over each corner: C01 [-25,9,2.6], C02 [-23.5,12,2.6]. Camera is the unchanged 480×270, ~112.2° horizontal FOV, logical height .65 m. RGB receives no overlays or postprocessing. Same-render-product instance masks remain evaluator-only.

### Frozen execution

Geometry-qualified order: **[C02]**. Run all eligible candidates exactly once; select first fully qualifying afterward. C01 gets no scientific call. No retries or additions.

Exact instruction:

> Turn left at the corner and continue down the hallway. Avoid obstacles and keep going.

Initialize cart rendering/oracle ON before reset and session; never change it during acquisition. Check USD transform, authored/computed visibility, oracle presence at captures and guards; verify full cart triangles unchanged at termination. Bootstrap: three live chronological buffer frames and a terminal OLD prediction at the fourth capture. No fabricated or replicated history. First normal 4 Hz capture crossing `<20 → >=20` is the only allowable FRESH frame. If request eligibility/queue ordering misses it, no later frame is substituted. There is at most one OLD and one FRESH; at most 64 total image requests and one server warmup.

Reuse OSA `minimum_wall_step_no_catchup_v1`: 60 Hz exact held-command SE(2), 10 Hz official MPC, 4 Hz RGB. No new pacing study. Maximum active observation 8 s; only .10 s source postroll after FRESH application. OLD continues during inference while the unchanged guard permits; guard only aborts before unsafe integration. No continuous safety claim beyond the inherited checker/curve allowance.

OLD: original EXP02D path metrics require arc ≥.50 m, yaw excursion ≥30°, reliable tangent excursion ≥30° with segments ≥.02 m. OLD must be hidden, nonSTOP, Hospital-safe without the added cart, and its actual finite whole raw path must violate .05 m with the cart. No extrapolation or observation connector.

FRESH: first-crossing genuine nonSTOP response, full raw path ≥.05 m, outgoing-corridor progress >0, raw arc ≥.60 m; existing interior mismatch ≥.10 m or reliable tangent/yaw ≥20° in cart influence region. No suffix deletion. At actual first-command application B: physical v_minus>.20 m/s, |omega_minus|≥.10 rad/s, observation→B travel≥.02 m, safe B, remaining original rows≥4 and arc≥.60 m. Physical incoming command, first applied FRESH command, and worker memory remain separate. Evaluate actual OLD overlap, request-local RTF [.8,1.2], stall≤.25 s and zero bursts. Preserve all source failures as outcomes, separately from artifact corruption.

Turning descriptors use reliable signed tangent change (20° descriptive straightening boundary) and arc/absolute net-turn radius proxy. They are not qualification rankings or intrinsic waypoint timing. LightNav rows have no timestamps. Every world path is anchored only at its own observation pose.

Official source: `c6f40e3220edbf7011e4f17eaf2c865416737d4d`; checkpoint `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`; task VLN; temperature=0, top_p=1, top_k=0, traj_top1=0. Official MPC hash `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`. Live process argv/environment are authenticated before scientific sessions.

### Commands

```bash
# Model-free actual Isaac renders (sanitized existing Isaac environment)
/home/gpuadmin/isaacsim/python.sh scripts/isaac/blind_corner_preflight.py --run data/blind_corner_source_acquisition_01/preflight_20260924T000000Z
.venv/bin/python scripts/run_blind_corner_source.py --run data/blind_corner_source_acquisition_01/primary_20260924T054500Z --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests/test_blind_corner_source.py
.venv/bin/python scripts/run_blind_corner_source.py --run data/blind_corner_source_acquisition_01/primary_20260924T054500Z --mode freeze
# Commit + normal push before start/collect.
.venv/bin/python scripts/run_blind_corner_source.py --run data/blind_corner_source_acquisition_01/primary_20260924T054500Z --mode start
/home/gpuadmin/isaacsim/python.sh scripts/isaac/blind_corner_online.py --run data/blind_corner_source_acquisition_01/primary_20260924T054500Z --candidate C02
.venv/bin/python scripts/run_blind_corner_source.py --run data/blind_corner_source_acquisition_01/primary_20260924T054500Z --mode stop
.venv/bin/python scripts/validate_blind_corner_source.py --run data/blind_corner_source_acquisition_01/primary_20260924T054500Z
.venv/bin/python scripts/report_blind_corner_source.py --run data/blind_corner_source_acquisition_01/primary_20260924T054500Z
```

Isaac commands unset inherited LD_LIBRARY_PATH/PYTHONPATH/CUDA/ROS/AMENT/CMAKE/COLCON variables as in OSA03; OPENBLAS/OMP/MKL threads=1. Preflight log preserves warnings. Raw/artifact arrays/RGB remain ignored. Immutable freeze hashes source code, all preflight inputs, source history, external contract and unrelated edits. Saved-only validator rechecks original episode integrity, raw/wire parity, transform anchoring, static occupancy, masks, first crossing, command/state reconstruction, direct safety and scheduler. Figure sidecars carry full numeric/source hashes and are verified without new inference or MPC.

## Result

Scientific freeze **`ee814f307904b878777fcfa56e5d7629e5837795`**, pushed before server startup. C02 was collected exactly once; no retry, source replacement, placement change, or threshold change. C01 remained excluded. Final classification: **`BLIND_CORNER_LIGHTNAV_SOURCE_NOT_QUALIFIED`**. The original saved-episode validator and new saved-only qualification validator both pass; source qualification itself fails three gates.

### Repository-confirmed facts

| Quantity | C01 | C02 scientific acquisition |
|---|---|---|
| Geometry / scientific exposure | Initial occlusion failed; no model/MPC | Geometry passed; one episode |
| Cart pixels at OLD observation | N/A | 12 (below frozen 20-pixel gate, not absolute zero) |
| Last hidden → first visible | N/A | frame_000004:12 → frame_000005:237 |
| OLD raw arc / yaw excursion / reliable tangent excursion | N/A | 1.283276 m / 52.190122° / 49.606466°; turning PASS |
| OLD whole Hospital+cart edge clearance | N/A | +0.160775 m; **finite OLD not obstructed** |
| OLD cart-only edge clearance | N/A | +0.740713 m |
| FRESH whole edge clearance | N/A | +0.015055 m; **.05 m margin FAIL** |
| FRESH cart-only edge clearance | N/A | +0.512887 m |
| FRESH STOP / raw N / arc / outgoing-corridor progress | N/A | false / 10 / 1.334217 m / +0.698079 m |
| Raw max local lateral OLD → FRESH | N/A | .490543 → .607958 m |
| Required obstacle-region mismatch | N/A | NOT ESTABLISHED: no eligible samples inside frozen influence region; values null, not zero |
| Physical u_minus at B [v,omega] | N/A | [0.8,0.126377241005012] m/s,rad/s |
| Controller memory at B | N/A | [0.8,0.6263772407238692] |
| First FRESH applied command | N/A | [0.8,0.6263772407238692] |
| Observation→B travel | N/A | .279999892 m |
| B edge clearance | N/A | .344559064 m |
| Remaining raw FRESH | N/A | 9 rows / 1.182883277 m |
| Request-local RTF / maximum loop stall / bursts | N/A | .991361139 / .200494433 s / 0; PASS |
| Minimum actual executed-prefix clearance / abort | N/A | .329437471 m / none |
| Qualified source | No scientific attempt | No |

C02 cart actual mesh AABB center is [-22.4956033921,12.1541992894] m (desired root placement center [-22.5,12.15], yaw π/2). Bounds: x[-22.7234950992,-22.2677116850], y[11.7331208076,12.5752777712], z[-.00571508789,.77636878745]. C01 actual center [-26.2041992894,7.7043966079]. Full exact USD wrapper/source transforms are saved in preflight `cart.json`, geometry qualification and `acquisition_scene.json`; the small desired/AABB-center difference comes from rotating the authored mesh, not runtime movement.

Actual FRESH observation:
`[-24.14613262341616,13.630129709963809,-1.4810420853953086]`.
Actual B (state/tick97):
`[-24.10899088959018,13.352648672499399,-1.4121391037170778]`.
Physical command immediately before observation: [.6000000287643693,.34464029123595447]. Recent .5 s actual yaw change at observation: .0897542414 rad. OLD was active throughout FRESH inference. `u_minus`, accepted worker memory and first-FRESH result are separately reconstructed; their angular difference is .5 rad/s, not silently zeroed.

| Event | Simulation seconds | Host-clock detail |
|---|---:|---|
| OLD observation | .783333374 | observation frame_000003; original stationary approach anchor |
| OLD request / receipt | N/A | 05:53:32.426143Z / 05:53:32.650837Z; RTT .224702777 s |
| First OLD application | 1.100000057 | actual command activation |
| Last hidden frame_000004 | 1.033333387 | captured before first OLD activation |
| First visible FRESH observation frame_000005 | 1.283333400 | 05:53:32.988809Z; state75 |
| FRESH request / full receipt | N/A | 05:53:33.084475Z / 05:53:33.320030Z; RTT .235566267 s |
| Ready seen / installed | 1.550000081 / 1.550000081 | 05:53:33.426178Z / 05:53:33.426622Z |
| First FRESH application B | 1.650000086 | 05:53:33.529264Z |
| Source termination | 1.750000091 | .100000005 s post-B; ATTEMPT_LIMIT |

Simulation observation→B duration .366666686 s and host observation→application .540454785 s are different clocks/quantities. Request-local RTF uses the unchanged saved first/last in-flight state window, not a mixed-clock division. The inherited metadata human string says “available 1s postroll”; that legacy text is stale: configured .10 s and actual six integration intervals are authoritative. No collector code was edited to conceal this textual limitation.

The cart remains present in renderer and geometry from before the episode reset. 113 runtime static-state checks agree; authored/computed visibility remain `inherited`; full final cart triangles exactly match initial triangles. All seven masks align to the exact RGB render state. The first 12→237 crossing is the exact transmitted FRESH JPEG; no later frame or queue substitution was used. Original inputs/outputs and independent source/checkpoint/MPC hashes are preserved.

Raw OLD file SHA256:
`e8ee2758e55b29b00b686548bf1bdd7eda93a920d8d3fb39a70164be70606028`.
Raw FRESH file SHA256:
`9ca6a1bf23e87af4de6917ce7713b5530d516a65e658652ee5f7b7d1e86dfd5d`.
World arrays use each chunk's own observation anchor, never B.

A supplemental saved-only direct-mesh check locates the FRESH margin violation at the existing corner wall/trim, `/World/Hospital/Geo_M3_TrimStrateSide3_356/Geo_M3_TrimStrateSide/Geo_M3_TrimStrateSide`. Closest raw-path point [-23.7421354043,12.7035054666]; nearest obstacle [-23.5644306766,12.8246231216]. First margin-violating returned segment is zero-based4. The predicted footprint has .015054543 m edge separation, below .05 m but still above physical overlap. This is **raw future margin failure**, not an executed collision; actual collection stops well before that region. Cart-only clearance is .512886513 m. No scientific gate or frozen evaluator was changed for this supplemental attribution.

FRESH continues left with a larger descriptive arc/net-turn radius proxy (OLD1.482192 m, FRESH2.006756 m); this is not a fitted curvature or a required yaw rate. The raw arrays differ, but the frozen obstacle-interaction mismatch cannot be evaluated because neither raw future reaches its influence window. This absence is retained as null/unestablished, not evidence of zero response or proof that the model understood the cart.

### Compute, artifacts and validation

Scientific terminal predictions: **2** (OLD1+FRESH1); buffer-only image requests: **5**; one server warmup (~282 ms); seven new RGB captures. Scientific terminal RTT sum .460269044 s. Official MPC submissions/results: **7/7**, saved solve wall sum .070488088 s. Acquisition scene/worker/episode wall12.099965 s (Isaac process startup separate). Geometry preflight and implementation tests made **0** model/MPC calls. GP/rigid/splice/graph/reconciliation/Native-continuation/trackability: **0**. No second candidate, retry or hidden prediction.

Authoritative artifacts under the run:

- `geometry_qualification.json`: all two candidates, including C01 rejection.
- `validation.json`: saved-only original episode and full source checks, with three failed scientific gates explicitly separated from integrity PASS.
- `candidates/C02/episodes/C02/`: immutable `rgb/`, `chunks/`, requests/wire/history, controller events, applied commands/execution, same-state masks, guard and first-crossing record.
- `candidates/C02/static_initial.json`, `static_final.json`: constant transform/presence and exact-mesh proof.
- `source_bundle/manifest.json`: **entries=[]; representative=null**. No qualified bundle exported.
- `review/index.html`, `review_bundle.zip`: scene/handoff, hidden/visible RGB, timing, plus predicted-footprint wall-clearance diagnosis; numeric/hash sidecars. Scene and handoff are combined in one equal-aspect world figure to avoid duplicating the same source.
- `artifact_validation.json`: frozen plot-data parity. `supplemental_clearance_attribution.json`: independent wall/cart separation; post-run descriptive script hash.

Additional exact saved-only command:

```bash
.venv/bin/python scripts/audit_blind_corner_clearance.py --run data/blind_corner_source_acquisition_01/primary_20260924T054500Z
```

Final relevant regression: **270 passed**; compileall and `git diff --check` PASS. Tests use synthetic fixtures only and make no real model/MPC calls.

The execution, gates and initial report code remain byte-identical to the pushed freeze after results. The supplemental wall/cart attribution adds no inference, controller, source selection or threshold change.

### Research interpretation

A genuine wall-occlusion reveal during left-turn OLD execution, nonzero incoming yaw rate, moving B and timing integrity are demonstrated in this one candidate. A **qualified obstacle-induced handoff is not obtained**: actual finite OLD is not obstructed, FRESH violates the existing wall margin, and the frozen obstacle-region mismatch is unestablished. No result-driven repositioning is performed. The single remaining uncertainty is whether a different independently declared blind-corner geometry can put the cart inside the actual finite turning OLD while yielding a whole-safe differing FRESH.

### Not demonstrated

No reconciliation benefit, complete obstacle bypass, graph superiority, improved closed-loop navigation, real-robot feasibility, or general LightNav obstacle capability. No Native continuation or follow-up optimization was run.
