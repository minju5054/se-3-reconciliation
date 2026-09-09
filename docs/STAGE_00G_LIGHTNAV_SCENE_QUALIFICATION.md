# Stage 0-G — LightNav input-contract and scene qualification

## Purpose and research boundary

Stage 0-G asks whether the exact LightNav-0 baseline receives an upstream-justified Isaac
input and whether a small neighborhood of six human-legible navigation scenes reproducibly
elicits distinct trajectory geometries. It is an inference-only qualification:

```text
frozen scene + 64 stationary RGB frames + instruction -> one LightNav N x 3 trajectory
```

It does not collect successive OLD/FRESH chunks, move the Jackal along a prediction, tune a
controller or physics, change LightNav, or add/change a graph, factor, objective, selector,
gate, smoothing method, or benchmark. The 30 outputs are qualification evidence, not a
formulation dataset.

## Exact LightNav baseline

The audited external checkout was clean at
`a645828d81a8439651172197ca80a75dc1377977`. The local checkout was intentionally not pulled
or modified. The checkpoint was `LightOriginsHQ/LightNav-0`, cached revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`. Important file SHA-256 values were:

| File | SHA-256 |
|---|---|
| `eval_config.json` | `7f475edbef4a13d2db99fa12eff5e9dedf48916969ec777f75e6e85e2a492b57` |
| `config.json` | `0e047b5e9ae7981cf0066e74d8b655a2f49d7d7965c25fca386f9eea80b63e58` |
| `processor_config.json` | `ac89ec99ea773dc28f5b217edec5947120071ad5b9edcd48d3843ecc63b9877e` |
| `action_tokenizer/manifest.json` | `00d77ae028edf59e0ce68b49122dd0a60da527f48508f5ee335a580993a12864` |
| `tokenizer_config.json` | `8e8e879bfe8a52ececc118c9b80a48240ab35ff01a8be88d9a071df7ee7c4803` |
| `model.safetensors.index.json` | `d725f1a587bde70cdf16ae5e1e672ff13cada2740c11baf13fbcff49dd7e00b1` |

The generated input-contract report independently rejects a checkout/revision/hash mismatch.

## Input-contract audit

Upstream does not prescribe one universal physical camera calibration. The official robot
deployment describes a 640 x 360 Orbbec stream downsampled to 480 x 270 but not camera height
or HFOV. The wheeled MuJoCo demo uses 480 x 270, a level camera, vertical FOV 79.865 degrees
(about 112 degrees horizontal at 16:9), and a TurtleBot-specific optical height near 0.198 m.
Habitat evaluation uses 480 x 270, 120-degree HFOV, and 0.88 m height. Calibration metadata is
not passed to the checkpoint as an inference tensor.

The profile was selected before qualification inference:

| Property | Previous Isaac | LightNav reference | Stage 0-G | Reason |
|---|---|---|---|---|
| source resolution | 448 x 256 | 480 x 270 robot/MuJoCo; model 448 x 256 | 480 x 270 | match wheeled client before preprocessing |
| HFOV | 90 deg | about 112 deg MuJoCo; 120 deg Habitat | 112 deg | prefer official wheeled reference |
| camera height | 0.48 m Jackal mount | about 0.198 m TurtleBot; 0.88 m Habitat; deployed robot unspecified | 0.48 m | preserve collision-safe, already verified Jackal mount because upstream heights are embodiment-specific |
| orientation | level, forward | level, forward | level, robot +x | direct match |
| aspect mode | stretch | training/default stretch | stretch | preserve checkpoint preprocessing |
| RGB | HWC uint8 RGB | HWC uint8 RGB | HWC uint8 RGB | direct match |
| history | 64 stationary frames | 64-frame `vlnce` SlowFast | 64 stationary frames | checkpoint task config |
| cadence | 4 Hz | `video_fps=4` | 4 Hz | checkpoint timestamp semantics |
| task/prompt | `vln`/`vlnce_traj` | `vln` maps to `vlnce`, `unified_traj` | same | official mapping |

Isaac RGBA is explicitly sliced to contiguous HWC uint8 RGB. The model then applies bilinear
stretch to `[H=256, W=448]` and normalization to `[-1, 1]`; there is no extra crop or hidden
resampling. All 64 simulation timestamps are retained. For this 64-frame episode, SlowFast
can select current (age 0–1), fast (2–33), mid (34–63), and anchor frames; its age-90 long
tier is not reachable.

LightNav returns float32 `10 x 3 [forward_m, lateral_m_left_positive,
yaw_rad_ccw_positive]` cumulative poses relative to the final observation frame. These rows
have no intrinsic timestamp. No row index is called time. The only world conversion is:

```text
T_world_i = T_world_robot_at_observation * T_robot_i
```

There is no later-pose anchoring, action resampling, or additional SE(2) accumulation.

## Scene and preview protocol

One consistent primitive indoor-map style contains six bays with common floor, walls,
lighting, scale, and accent materials. Physical openings and collision geometry—not colored
rectangles alone—define the affordance:

| Scenario | Physical geometry | Frozen instruction | Preview inspection |
|---|---|---|---|
| Q0 straight | open straight corridor | `Go straight through the corridor.` | clear forward passage |
| Q1 left | forward wall and left-only T opening | `Turn left at the intersection.` | left opening visible |
| Q2 right | mirrored right-only T opening | `Turn right at the intersection.` | right opening visible |
| Q3 doorway | wall segments, raised lintel, open centered doorway and room | `Go through the open doorway.` | open doorway visible |
| Q4 detour left | real obstacle shifted right, leaving a larger left passage | `Go around the obstacle on the left and continue forward.` | left passage visible |
| Q5 detour right | mirrored obstacle, leaving a larger right passage | `Go around the obstacle on the right and continue forward.` | right passage visible |

All six V0 egocentric images were rendered and visually inspected before the primary config
was frozen. The first preview attempt was technically invalid because the camera did not
follow articulation teleports; it produced no inference and is preserved as invalid. The
camera transform was then explicitly synchronized to each Jackal pose, the six previews were
re-rendered and inspected, and only then was the primary r2 config frozen.

## Deterministic variants and frozen criteria

Each scenario uses the same predeclared nearby poses: V0 baseline, V1 yaw +0.05 rad, V2 yaw
-0.05 rad, V3 robot-left +0.09 m, and V4 robot-left -0.09 m. These variants do not change
task semantics or collide with scene geometry. The primary config SHA-256 is
`8be2c0c65b7188a3a1e38d2528b99dd38006a6842239ce4d303cf7648a4d5d57`.

The criteria were frozen before inference: straight requires endpoint lateral <= 0.15 m in
magnitude, net yaw <= 0.15 rad in magnitude, and maximum lateral excursion <= 0.20 m. Left
and right require endpoint lateral or yaw of the expected sign with magnitude at least 0.20,
plus a contradictory-sign guard. Detours require the corresponding lateral excursion of at
least 0.25 m and endpoint forward progress of at least 0.30 m. Doorway requires at least
0.30 m forward progress. A path shorter than 0.05 m is not meaningful; <=0.01 m is
non-moving. Each scenario requires 4/5 matches. Global diversity requires at least five raw
outputs, reproducible left, right, and doorway/detour evidence, and no dominant invalid/STOP
behavior. These are engineering qualification criteria, not graph or paper thresholds.

## Actual primary result

The run is
`data/stage0/lightnav_scene_qualification/20260909T_stage0g_primary_r2/`. Smoke inference was
Q0/Q1/Q2 V0, followed by exactly the remaining 27 cases. There were no retries for preferred
geometry and no research condition changed after freeze.

| Scenario | Valid | Intended match | STOP/non-moving | Unique raw | Path length range m | Endpoint lateral range m | Net yaw range rad | Result |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| Q0 straight | 5/5 | 5/5 | 0 | 1 | 0.683–0.683 | 0.0009–0.0009 | 0.0015–0.0015 | PASS |
| Q1 left | 5/5 | 0/5 | 0 | 2 | 1.504–1.508 | 0.0001–0.0414 | 0.0007–0.1848 | FAIL |
| Q2 right | 5/5 | 0/5 | 0 | 1 | 1.508–1.508 | 0.0001–0.0001 | 0.0007–0.0007 | FAIL |
| Q3 doorway | 5/5 | 5/5 | 0 | 1 | 1.508–1.508 | 0.0001–0.0001 | 0.0007–0.0007 | PASS |
| Q4 detour left | 5/5 | 0/5 | 0 | 1 | 1.508–1.508 | 0.0001–0.0001 | 0.0007–0.0007 | FAIL |
| Q5 detour right | 5/5 | 0/5 | 0 | 1 | 1.508–1.508 | 0.0001–0.0001 | 0.0007–0.0007 | FAIL |

All outputs are finite and moving, but only three exact raw file hashes and two coarse
geometry signatures occur. One output is shared by all Q0 variants; one slight-left output
appears for Q1 V0/V2; and a single almost-straight output appears for the other 23 cases.
Q1 V0/V2 show a positive yaw of 0.1848 rad but still fail the frozen 0.20 criterion. Q2 and
both detours show no reproducible intended-sign response. Q3 passes only its predeclared
forward-progress rule; its raw output is exactly shared with the dominant straight-like
group. Thus visual legibility and non-STOP validity pass, while left, right, detour, minimum
five-output diversity, and global diversity fail.

The strict validator reconstructed all 30 observation-pose transforms, checked raw hashes,
rejected missing/non-finite artifacts, verified the frozen config hash, opened all figures,
and returned PASS for artifact integrity. Artifact validity is not scene-qualification PASS.

## Artifacts, descriptors, and GUI

Each `qualification/Qx/Vx/` contains the 64 PNG history frames, final RGB, frame timestamp
CSV, capture metadata, immutable `raw/lightnav.npy`, raw decoder text,
`derived/trajectory_world.npy`, metadata, descriptors, provenance, and local/world figures.
Descriptors include pose count, path length, forward/lateral endpoints, left/right excursion,
wrapped net/cumulative/incremental yaw, tangent turning, curvature-sign changes, and exact
STOP/non-moving flags. Summary artifacts include duplicate groups, geometry signatures, and
a descriptive pairwise mixed-unit trajectory-distance matrix.

The GUI shows the stationary Jackal, active scene, cyan world trajectory, yellow heading
ticks, and first/endpoint points. LiDAR rays are hidden only in the diagnostic viewport. The
Jackal is deliberately not executed. Default GUI behavior holds until the user closes Isaac:

```bash
./scripts/isaac/run_stage0g_lightnav_scene_qualification.sh \
  data/stage0/lightnav_scene_qualification/20260909T_stage0g_primary_r2 \
  --mode gui --scenario Q1_LEFT_TURN --variant V0
```

The six-scene and all-variant overviews are:

- `overview/all_scenarios_camera_and_trajectory.png`
- `overview/all_variants_by_scenario.png`

## Decision and claim limitation

The exact decision is:

```text
STAGE0G_SCENE_QUALIFICATION_FAILED
```

The input contract is resolved and the scenes are visibly distinct, but the frozen primary
run does not reproducibly elicit left, right, and detour trajectory geometry and has only
three unique raw outputs. Therefore this setup is not authorized for DATA-02 or successive
OLD/FRESH collection. This failure does not prove that LightNav cannot navigate, that one
particular camera/scene redesign would fail, that controller execution is defective, or that
any reconciliation method succeeds or fails. A future, explicitly versioned Stage 0-G scene
qualification may redesign scientific conditions; this failed run must remain reported.

## Reproduction sequence

For a new run ID, preserve the order below. Do not overwrite this primary run:

```bash
.venv/bin/python scripts/prepare_stage0g_lightnav_scene_qualification.py --run-id NEW_RUN
./scripts/isaac/run_stage0g_lightnav_scene_qualification.sh \
  data/stage0/lightnav_scene_qualification/NEW_RUN --mode preview
.venv/bin/python scripts/prepare_stage0g_lightnav_scene_qualification.py --freeze \
  --run-directory data/stage0/lightnav_scene_qualification/NEW_RUN
./scripts/isaac/run_stage0g_lightnav_scene_qualification.sh \
  data/stage0/lightnav_scene_qualification/NEW_RUN --mode capture --all --headless
./scripts/lightnav/run_stage0g_lightnav_inference.sh \
  data/stage0/lightnav_scene_qualification/NEW_RUN --smoke
./scripts/lightnav/run_stage0g_lightnav_inference.sh \
  data/stage0/lightnav_scene_qualification/NEW_RUN --remaining
.venv/bin/python scripts/summarize_stage0g_lightnav_scene_qualification.py \
  data/stage0/lightnav_scene_qualification/NEW_RUN --visual-inspection-passed
.venv/bin/python scripts/validate_stage0g_lightnav_scene_qualification.py \
  data/stage0/lightnav_scene_qualification/NEW_RUN
```
