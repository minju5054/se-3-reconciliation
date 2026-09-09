# Stage 0-G2 — Jackal domain-matched Isaac scene qualification

## Purpose and boundary

Stage 0-G2 asks whether a visually rich, realistic indoor domain elicits reproducible straight,
left, right, doorway, and left/right-detour geometry from the exact LightNav baseline while the
robot remains Jackal. It is an inference-only scene qualification, not an algorithm improvement,
navigation benchmark, controller test, planner, reconciliation experiment, or DATA-02 collector.
No controller, physics, LightNav weights, graph, residual, factor, gate, selector, smoothing, or
OLD/FRESH interface changed.

Primary run:

```text
data/stage0/lightnav_scene_qualification_g2/20260909T_stage0g2_primary_r3/
```

Final decision:

```text
STAGE0G2_SCENE_QUALIFICATION_FAILED
```

## Frozen variables

- official Clearpath Jackal: `/Isaac/Robots/Clearpath/Jackal/jackal.usd`
- LightNav checkout: `a645828d81a8439651172197ca80a75dc1377977`
- checkpoint revision: `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`
- task/prompt: `vln` → `vlnce` / `vlnce_traj`, `unified_traj`
- decoded output: float32 `10 x 3 [forward, left, CCW yaw]`
- camera: 480 x 270, 112° HFOV, `[0.30, 0, 0.48]` m, level robot-forward
- preprocessing: stretch to 448 x 256 with bilinear resampling
- history: 64 stationary HWC uint8 RGB frames at 4 Hz
- world anchor: robot pose at the final observation frame only
- variants: V0, yaw `+/-0.05` rad, and robot-frame lateral `+/-0.09` m

LightNav waypoint rows have no intrinsic timestamps. The results below are spatial decoded
waypoints; they are not time-aligned tracking data.

Frozen config SHA-256:

```text
49d339d16694d96dc5113271a2236e79ce413ae2411915e340885f8310fc9f01
```

## Asset audit and pre-inference selection

Isaac Sim 6.0.1 exposed Office, Hospital, Simple_Room, Grid, and several Warehouse families under
the official Isaac 6.0 asset root. The audit was completed before primary LightNav inference:

| Candidate | Availability/load | scale; lighting | structure/Jackal fit | Decision |
|---|---|---|---|---|
| Office `/Isaac/Environments/Office/office.usd` | available; loaded | 1 m/unit; authored | 4,653 prims; rich corridors but one detected collision prim | rejected: collision coverage too weak |
| Hospital `/Isaac/Environments/Hospital/hospital.usd` | available; loaded | 1 m/unit; authored | 1,910 prims, 126 collision prims; 3.5 m corridors, doors, signs, beds, carts | selected before inference |
| Simple_Room `/Isaac/Environments/Simple_Room/simple_room.usd` | available; loaded | 1 m/unit; authored | 139 prims, 60 collision prims; insufficient route scale | rejected: insufficient diversity |
| Simple_Warehouse `/Isaac/Environments/Simple_Warehouse/warehouse.usd` | available; loaded | 1 m/unit; authored | 3,417 prims, 781 collision prims; wide industrial domain | rejected: weaker indoor-VLN match |
| Grid | available/loadable | 1 m/unit; basic | abstract calibration environment; drivable | rejected: not realistic/domain matched |

The selected Hospital uses scale 1.0, Z-up, 1 m stage units, authored lighting, and authored
collision geometry. Selection did not inspect any G2 LightNav output. Exact resolved URLs and
candidate decisions are in `environment_asset_manifest.json` and `environment_selection.json`.

## Scenarios and feasibility

All six V0 images were inspected before freeze. The gate required visible free space, intended
route, referenced feature, natural appearance, usable camera, and no material pose displacement
after one second of physics settling. The initial doorway candidate showed a closed exterior
door and was rejected before freeze; the frozen candidate uses an open patient-room door.

| Scenario | Pose `[x,y,yaw]` | Instruction/affordance | width; minimum clearance |
|---|---|---|---|
| G2_Q0_STRAIGHT | `[19.0,23.5,-pi/2]` | straight down patient hallway | 3.50 m; 1.53 m |
| G2_Q1_LEFT_TURN | `[19.0,26.7,+pi/2]` | left at north cross corridor | 3.50 m; 1.53 m |
| G2_Q2_RIGHT_TURN | `[19.0,31.5,-pi/2]` | right at the same junction | 3.50 m; 1.53 m |
| G2_Q3_DOORWAY | `[18.8,26.9,pi]` | through open patient-room door | 1.18 m; 0.37 m |
| G2_Q4_DETOUR_LEFT | `[19.65,7.4,+pi/2]` | left of warning sign | 3.50 m; 1.20 m |
| G2_Q5_DETOUR_RIGHT | same observation as Q4 | right of warning sign | 3.50 m; 1.20 m |

Jackal's approximate footprint is 0.43 x 0.51 m. Clearance values are geometric feasibility
records, not planned paths or obstacle-avoidance evidence. Settling displacement is recorded per
case; it does not prove all future trajectories collision-free.

## Protocol and commands

The primary protocol ran each frozen case once: Q0/Q1/Q2 V0 smoke, then the remaining 27 cases.
No preferred-output retries or post-output tuning occurred.

```bash
# Create run, audit official assets, and render six pre-freeze V0 images
.venv/bin/python scripts/prepare_stage0g2_jackal_domain_scene_qualification.py --run-id RUN_ID
./scripts/isaac/run_stage0g2_jackal_domain_scene_qualification.sh RUN_DIR --mode asset-audit --headless
./scripts/isaac/run_stage0g2_jackal_domain_scene_qualification.sh RUN_DIR --mode preview --headless

# Only after visual inspection
.venv/bin/python scripts/prepare_stage0g2_jackal_domain_scene_qualification.py \
  --freeze RUN_DIR --visual-inspection-passed
./scripts/isaac/run_stage0g2_jackal_domain_scene_qualification.sh RUN_DIR --mode capture --all --headless
./scripts/lightnav/run_stage0g2_jackal_domain_scene_inference.sh RUN_DIR --smoke
./scripts/lightnav/run_stage0g2_jackal_domain_scene_inference.sh RUN_DIR --remaining
.venv/bin/python scripts/summarize_stage0g2_jackal_domain_scene_qualification.py \
  RUN_DIR --visual-inspection-passed
.venv/bin/python scripts/validate_stage0g2_jackal_domain_scene_qualification.py RUN_DIR
```

Persistent GUI:

```bash
./scripts/isaac/run_stage0g2_jackal_domain_scene_qualification.sh \
  data/stage0/lightnav_scene_qualification_g2/20260909T_stage0g2_primary_r3 \
  --mode gui --scenario G2_Q3_DOORWAY --variant V0
```

GUI legend: realistic Hospital is the environment; the official Jackal is the stationary
observer; cyan is the world-frame prediction and endpoints; yellow segments are decoded heading
markers. The GUI does not move Jackal or execute the prediction.

## Artifacts and schemas

The ignored run root contains the asset manifest/selection, pre-freeze previews and explicit
gate, frozen config/hash, input contract, 30 `qualification/<Q>/<V>/` cases, summary, decision,
comparison, validation, and overview figures. Each case contains:

- `input/history/frame_*.png`, `latest_rgb.png`, `frame_samples.csv`, capture metadata;
- immutable `raw/lightnav.npy` and raw decoder text;
- observation-pose-anchored `derived/trajectory_world.npy`;
- metadata, provenance hashes, spatial descriptors, and two figures.

Exclusive creation protects raw/case outputs from overwrite. Generated data are ignored by Git;
the frozen Stage 0-G run and its config/document were not changed.

## Observed result

| Scenario | valid | intended match | unique exact raw | result |
|---|---:|---:|---:|---|
| G2_Q0_STRAIGHT | 5/5 | 5/5 | 3 | PASS |
| G2_Q1_LEFT_TURN | 5/5 | 0/5 | 2 | FAIL |
| G2_Q2_RIGHT_TURN | 5/5 | 0/5 | 1 | FAIL |
| G2_Q3_DOORWAY | 5/5 | 5/5 task-local | 1 | task-local PASS |
| G2_Q4_DETOUR_LEFT | 5/5 | 0/5 | 2 | FAIL |
| G2_Q5_DETOUR_RIGHT | 5/5 | 0/5 | 2 | FAIL |

Across 30 predictions there were 5 unique exact raw arrays, dominant exact group fraction 0.667,
and 3 coarse geometry signatures. The diversity count and 80% dominance gates pass, but the left,
right, and detour reproducibility gates fail. Doorway forward progress passes, but its output is
not distinct from straight/unrelated outputs: `DISTINCT_INTENT_NOT_DEMONSTRATED`.

Compared descriptively with Stage 0-G, exact unique arrays increased 3 → 5, dominant fraction
decreased about 0.767 → 0.667, and signatures increased 2 → 3. Both stages still have Q1/Q2/Q4/Q5
at 0/5. This paired configuration comparison is descriptive and cannot establish that the scene
caused any difference.

## Claim limitation

The observed realistic images and five exact outputs show that domain appearance changed while
the same straight-like mode still dominated. They do not prove a model defect, prompt defect,
camera defect, semantic grounding failure, controller behavior, or real navigation performance.
Because left/right/detour reproducibility did not qualify, Stage 0-G2 does not authorize DATA-02
or successive OLD/FRESH collection.
