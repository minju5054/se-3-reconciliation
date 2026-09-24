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

Pending the pushed scientific freeze. No positive source claim is made at preparation.
