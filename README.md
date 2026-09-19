# SE(3) Reconciliation Research

This repository studies how to reconcile successive Navigation VLA action chunks across an
OLD-to-FRESH transition while preserving the intent of the new trajectory. LightNav-0 emits
arbitrary `N x 3 [x, y, yaw]` SE(2) waypoint references; the reconciliation method developed
here is separate from LightNav.

## Current research state

The [fixed-geometry lookahead diagnostic](docs/GP_SE2_REF_02_SOURCE_PROGRESS_LOOKAHEAD.md) holds the same 30 dense rows and nearest rule fixed while selecting targets in original-row progress. Across six new MPC rollouts, all Native/adapter baselines reproduce bitwise. On the fixed large-turn event, the new selector recovers all original success conditions (yaw error 38.828°→0.255°); benign success remains and goal time changes from 2.59 to 1.28 s, with higher linear command variation. The MPC calculation and acceptance remain unchanged; the reference selector differs. See the [local figures](data/robotless_gp_se2_ref_02/presentation_20260919T_ref02/index.html). This is a two-event offline diagnostic, not a GP or general navigation improvement claim.

The [FRESH preparation factor isolation](docs/GP_SE2_REF_01_PREPARATION_ISOLATION.md) ran eight new official-MPC rollouts on the fixed large-turn and benign events. Native and suffix-only succeed on the large turn; resampling-only and the current adapter fail the original 3 s yaw/dwell criteria (37.30° and 38.83° yaw error). All four benign variants succeed, although resampling delays goal entry. Native/Adapter reproduce the previous states and commands bitwise. Matched-state probes show that row resampling changes the fixed five-row MPC lookahead and delays final-goal inclusion. The [local image index](data/robotless_gp_se2_ref_01/primary_20260919T062000Z/index.html) contains all 22 diagnostic figures. No controller/GP change, new inference or navigation improvement is claimed.

The [hard-handoff transfer pilot](docs/GP_SE2_02_HARD_HANDOFF_TRANSFER.md) ran all 16 fixed GP starts, eight rigid starts and 16 new official-MPC counterfactual rollouts. On three hard events, M2 produces one full-valid candidate and M3 none; neither improves local success. The large-turn event succeeds with native RAW but fails yaw/dwell with the valid M2 reference. All six methods succeed on the benign control. Converged collocation solutions can fail between-point motion checks, and plan validity does not guarantee MPC goal/dwell success. The [local review index](data/robotless_gp_se2_02/primary_20260919T024000Z/review_bundle/index.html) includes all four cases, failures and 12 actual Isaac screenshots. This is an offline development-corpus diagnostic, with **no additional execution benefit observed**.

The [supplied-derivative comparison](docs/GP_SE2_DIAG_02_DERIVATIVES.md) preserves the same benign event, two initializations, original GP objective/constraints and SLSQP budget. All four supplied-Jacobian starts converge with independently full-feasible candidates in 1.74–2.13 s; the four new FD baselines time out at 30 s. Both FRESH starts recover feasibility, and both deceleration seeds improve objective from 3.9634 to 0.1943 (95.1%) with changed paths. This is numerical optimization evidence on one event, with declared nonsmooth-domain limitations and no new MPC execution or navigation claim. The [local comparison index](data/robotless_gp_se2_diag_02/diagnostic_20260918T163000Z/index.html) contains all eight outcomes and 18 figures.

The [GP feasibility diagnosis](docs/GP_SE2_DIAG_01_FEASIBILITY.md) recovers a fully checked GP candidate for M2 and M3 on the fixed benign event by changing only the second initialization to a same-curvature deceleration. Both return that feasible seed unchanged; the solver still times out without a better feasible candidate. All 40 historical latest iterates fail collocation, while all three full-size synthetic perturbed starts recover feasibility. The diagnosis is partially localized, and no new MPC execution or navigation improvement is claimed. The [compact local review ZIP](data/robotless_gp_se2_diag_01/diagnostic_20260918T071302Z/review_bundle.zip) contains the comparison graphs and numeric evidence.

The [GP-SE2-01 environment-constrained handoff pilot](docs/GP_SE2_01_ENVIRONMENT_CONSTRAINED_HANDOFF.md) completed its frozen 10-case offline comparison and actual Isaac replay. Local-transition success is native RAW 6/10, preparation adapter 7/10, rigid 7/10, and both GP variants 0/10: neither GP variant found a feasible candidate within the fixed budget. M3 therefore loses six native and seven adapter successes through candidate unavailability. The result is **no additional GP benefit observed under this protocol**, with only one obstacle-sensitive case, static oracle geometry and idealized kinematic execution. The [local comparison index](data/robotless_gp_se2_01/primary_20260918T054000Z/index.html) contains all 460 required method graphs plus ten overlays; the report discloses the separate validator-only bookkeeping correction.

The [genuine online robotless handoff dataset](docs/ROBOTLESS_ONLINE_HANDOFF_DATASET_V1.md) completed all 60 frozen Hospital episodes with live Isaac RGB, persistent official LightNav, the read-only official MPC and 60 Hz logical SE(2) integration. It preserves 881 actual handoffs (880 moving, one stationary), 59 STOP attempts, 4,639 live frames and all 1,762 required valid-event trajectory PNGs. Every valid event has measured inference/execution/capture overlap and post-switch execution. Local pacing passes in 838/881 events and nominal history-full coverage is 246/881; collision validity remains unknown. The [local image index](data/robotless_online_handoffs_v1/primary_20260915T091900Z/index.html) links every event and episode.

Persistent GUI evidence is available for challenging and benign [OLD-consistent OLD/FRESH handoffs](docs/ROBOTLESS_OLD_FRESH_PROBLEM_GUI.md).

Five frozen diagnostic cases now have genuine successive LightNav predictions from OLD-consistent spatial observation poses, with exact planning-OLD reproduction in every final session. Direction and yaw disagreement remains in some cases, and local versus windowed tangents differ in others; see the [OLD-consistent observation pilot](docs/ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT.md) for actual Isaac evidence and interpretation limits.

The frozen Hospital bank now compares straight and OLD-conditioned counterfactual continuation while preserving both raw predictions and separating local from windowed direction diagnostics. Some residuals decrease substantially while others remain or increase; see the [OLD-conditioned report](docs/ROBOTLESS_OLD_CONDITIONED_HANDOFF.md) for paired distributions, anchor limitations and actual Isaac evidence.

The frozen 30-episode Hospital screening bank produced 30 valid successive LightNav pairs with 16 distinct raw pairs; episode and duplicate-aware distributions include both small and larger handoff geometry differences. See the [screening report](docs/ROBOTLESS_LIGHTNAV_HANDOFF_SCREENING.md) for full distributions, deterministic Isaac representatives and interpretation limits.

Projection-based robotless handoff characterization now measures continuous closest position, cross-track distance, tangent-direction mismatch, pose-yaw mismatch, and arc-length progress on the fixed FRESH trajectory. The four prescribed conditions passed numerical and actual Isaac visualization checks; see the [projection report](docs/ROBOTLESS_PROJECTION_HANDOFF_GEOMETRY.md) for values and interpretation limits.

Controlled robotless staleness characterization now measures a moving boundary against one fixed, observation-anchored FRESH trajectory from the saved run. In the prescribed conditions, entry distance decreases then increases while polyline distance decreases; see the [characterization report](docs/ROBOTLESS_CONTROLLED_STALENESS_CHARACTERIZATION.md) for values and interpretation limits.

Robotless successive interface validation now succeeds: one LightNav session returns OLD/FRESH from two Isaac observations, and each chunk is displayed in the world frame using its own observation pose. See the [successive-chunk report](docs/ROBOTLESS_ISAAC_LIGHTNAV_SUCCESSIVE_CHUNKS.md) for actual runtime evidence and scope limits.

Robotless static interface validation now succeeds: Isaac captures one RGB image
from a logical SE(2) agent, isolated official LightNav returns one chunk, and
Isaac displays its observation-anchored world trajectory. See the
[robotless single-chunk report](docs/ROBOTLESS_ISAAC_LIGHTNAV_SINGLE_CHUNK.md)
for coordinate checks and evidence; this does not validate navigation or motion.

The unmodified official LightNav MuJoCo TurtleBot pipeline also ran successfully on this
machine as a separate reference reproduction. Its evidence remains in
`data/reference_reproduction/`; see the [reproduction report](docs/OFFICIAL_LIGHTNAV_MUJOCO_DEMO_REPRODUCTION.md)
for provenance, runtime evidence, and claim limits.

## Historical research findings

The following Jackal, Stage 0, EXP and DATA-02 findings are historical. On 2026-09-18,
the user explicitly approved deleting their previous local raw and derived runs, including
final runs unrelated to the current robotless research. Their tracked source, configurations,
tests and reports remain, but the old data paths in those reports are no longer present
locally. See the [2026-09-18 cleanup audit](docs/REPOSITORY_CLEANUP_AUDIT_20260918.md)
for the exact deletion and preservation boundaries.

- DATA-02 comprised the immutable 84-episode v1 cohort and an independently predeclared
  168-episode v2 extension, both collected by persistent LightNav and actual wheel-driven Jackal
  execution. The reference-only union contained 1,479 attempted transitions, 959
  `ELIGIBLE_MOVING` contexts, and 191 unique ordered raw pairs. One pair occupied
  593/959 contexts and created a 920-transition connected component. Duplicate domination and
  isolated-split feasibility therefore failed. The exact historical decision remains
  `DATA02_COMBINED_DIVERSITY_INSUFFICIENT`. EXP-02D used the complete immutable union only as a
  development corpus for mechanism/formulation analysis, never as an independent final test set.
- Stage 0-G3 compared the frozen Stage 0-G2 stationary histories with 30 paired scripted moving
  Jackal histories ending at the same observation poses. Moving history changed 16/30 raw outputs
  but did not qualify left/right/doorway-or-detour behavior. That historical failure remains
  unchanged; DATA-02 was the later explicit research decision to collect the target context rather
  than another indirect qualification stage.
- The execution-platform investigation prompted by feedback item 1 is complete through the
  Stage 0-D/E/F and EXP-02B-R reports. This does not claim that the platform is
  generally validated beyond the observed LightNav execution envelope.
- EXP-02C attributed the principal M4 failure to the incoming-direction transition
  factor. EXP-02D kept historical M4 intact and froze one isolated redesign: its M3 points
  the direction factor at the raw follower lookahead `F_q` rather than nearest entry `F_k`.
- The EXP-01B-derived DATA-01 bank is retired from primary formulation use. Its dedicated
  generated bank and pipeline were removed earlier; the independent EXP-01B local source
  evidence was also deleted in the 2026-09-18 cleanup. Its historical reports remain.
- DATA-02 was coverage-oriented and descriptive. Its reports do not estimate natural deployment
  frequencies, validate instruction satisfaction, or evaluate any reconciliation formulation.
- EXP-02D completed its development-only primary over all 959 reconstructable transitions. Its
  pair-balanced mean `J_cmd` is RAW `1.3936`, historical M4 `0.9179`, no-direction `1.2085`, and
  lookahead M3 `0.8724`; M3 is lower than RAW and no-direction overall, but not distinguishable
  from historical M4 under the frozen pair-cluster bootstrap. Optimized candidates were evaluated
  offline and were not physically executed. See the dedicated report for the important benign,
  intermediate, challenging, and failure-regime qualifications.

## Current system boundaries

The current online collector uses a logical SE(2) agent in the Isaac Hospital scene:

1. Isaac captures live RGB at nominal 4 Hz from the agent's recorded pose.
2. Persistent official LightNav returns untimed spatial `N x 3 [x, y, yaw]` references.
3. The read-only official `MpcTracker` computes body commands `[v, omega]` at 10 Hz.
4. The research executor integrates those commands kinematically at nominal 60 Hz using the
   exact constant-command unicycle update and records every actual state.

The first newly computed FRESH command's actual application defines the handoff boundary.
OLD commands continue while inference or the new MPC solve is pending. Isaac advances the
simulation clock; the logical agent's pose is assigned from that recorded integration.
There is no robot mesh, articulation, wheel controller or physical collision response in this
collector. Collision validity remains unknown. The historical `TrajectoryFollower` and Jackal
wheel-execution stack remain in source for their historical protocols.

Isaac Sim 6.0.1, the official LightNav model server and official MPC worker run in separate
existing environments. The current robotless collector does not require ROS 2. The machine's
ROS 2 Jazzy installation continues to use system Python; repository tests use the local Python
3.12 `.venv`. Do not install or modify LightNav inside this repository.

LightNav waypoint rows have no intrinsic timestamps. The MPC's 0.1 s horizon step is a
controller convention. Each cumulative local waypoint is transformed with the logical agent
pose recorded at that chunk's own RGB observation:

```text
T_world_waypoint = T_world_agent_at_observation * T_agent_waypoint
```

World and agent coordinates use metres, Z up, local x forward, local y left and CCW yaw in
radians. Observation, readiness, installation and command-activation timestamps, transforms
and source hashes remain explicit. Raw VLA outputs are never overwritten. Saved GUI replay
uses recorded samples and performs no new inference or execution.

## Evidence retained locally

Generated data are ignored by Git. The 2026-09-18 cleanup preserves all `data/robotless*`
roots and `data/reference_reproduction/`, including these current sources and derivatives:

| Evidence | Local path |
|---|---|
| Genuine online primary, raw streams and event plots | `data/robotless_online_handoffs_v1/primary_20260915T091900Z/` |
| Online technical runs and server provenance | `data/robotless_online_handoffs_v1/` |
| Primary recorded GUI replay | `data/robotless_online_replay/primary_20260915T091900Z_replay01/` |
| Online trajectory-shape analysis | `data/robotless_online_handoff_shape_analysis/primary_20260915T091900Z_v1/` |
| Frozen 30-condition screening bank | `data/robotless_handoff_screening/20260914T101519Z/` |
| OLD-conditioned comparison | `data/robotless_old_conditioned_handoff/20260915T021149Z/` |
| OLD-consistent observation pilot | `data/robotless_old_consistent_observation/20260915T043415Z/` |
| OLD/FRESH problem GUI evidence | `data/robotless_old_consistent_problem_gui/20260915T064814Z/` |
| Controlled staleness and projection characterization | `data/robotless_controlled_staleness/`, `data/robotless_projection_handoff/` |
| Single- and successive-chunk interface validation | `data/robotless_single_chunk/`, `data/robotless_successive_chunks/` |
| Separate official MuJoCo reference reproduction | `data/reference_reproduction/lightnav_official_mujoco/20260914T060141Z/` |

Previous local runs under `data/exp01*`, `data/exp02*`, `data/data02_*`, `data/stage0/`
and `data/controller_effect_check/` were deleted with user authorization. Their historical
reports and tracked reproducers remain, including code that current robotless tools import.
Those historical raw paths cannot be replayed or revalidated from this checkout's local data.

The [2026-09-18 cleanup audit](docs/REPOSITORY_CLEANUP_AUDIT_20260918.md) records the current
preservation boundary and supersedes the local-retention decisions in the
[2026-09-09 audit](docs/REPOSITORY_CLEANUP_AUDIT_20260909.md).

## Documentation map

Detailed protocols, commands, schemas, observed results, and claim limitations live in `docs/`.
Historical report commands that refer to deleted runs require their original inputs; retaining
source code does not mean those raw inputs remain locally available.

- Current robotless collection: [online dataset and runtime](docs/ROBOTLESS_ONLINE_HANDOFF_DATASET_V1.md),
  [OLD/FRESH GUI](docs/ROBOTLESS_OLD_FRESH_PROBLEM_GUI.md),
  [OLD-consistent observations](docs/ROBOTLESS_OLD_CONSISTENT_OBSERVATION_PILOT.md),
  [OLD-conditioned comparison](docs/ROBOTLESS_OLD_CONDITIONED_HANDOFF.md), and
  [Hospital screening](docs/ROBOTLESS_LIGHTNAV_HANDOFF_SCREENING.md).
- Current interface and geometry: [single chunk](docs/ROBOTLESS_ISAAC_LIGHTNAV_SINGLE_CHUNK.md),
  [successive chunks](docs/ROBOTLESS_ISAAC_LIGHTNAV_SUCCESSIVE_CHUNKS.md),
  [controlled staleness](docs/ROBOTLESS_CONTROLLED_STALENESS_CHARACTERIZATION.md), and
  [projection geometry](docs/ROBOTLESS_PROJECTION_HANDOFF_GEOMETRY.md).
- Separate reference: [official LightNav MuJoCo reproduction](docs/OFFICIAL_LIGHTNAV_MUJOCO_DEMO_REPRODUCTION.md).
- Historical platform: [Stage 0 Jackal](docs/STAGE_00_JACKAL_TRAJECTORY.md),
  [controller validation](docs/STAGE_00_CONTROLLER_VALIDATION.md),
  [single-chunk LightNav](docs/STAGE_00_LIGHTNAV_SINGLE_CHUNK.md),
  [execution calibration](docs/STAGE_00_EXECUTION_LAYER_CALIBRATION.md),
  [closed-loop validation](docs/STAGE_00_CLOSED_LOOP_EXECUTION_VALIDATION.md),
  [LightNav execution envelope](docs/STAGE_00_LIGHTNAV_EXECUTION_ENVELOPE.md), and
  [current controller effect check, 2026-09-13](docs/CURRENT_CONTROLLER_EFFECT_CHECK.md).
- Historical LightNav qualification: [Stage 0-G2 Jackal domain scene](docs/STAGE_00G2_JACKAL_DOMAIN_SCENE_QUALIFICATION.md)
  and [Stage 0-G3 moving egocentric history](docs/STAGE_00G3_MOVING_HISTORY_QUALIFICATION.md).
- Historical dataset collection: [DATA-02 online-successive OLD/FRESH v1](docs/DATA_02_ONLINE_SUCCESSIVE_OLD_FRESH.md),
  [v2 extension/final combined assessment](docs/DATA_02_V2_EXTENSION_AND_FINAL_SPLIT.md), and
  [saved high-motion GUI demo](docs/DATA_02_HIGH_MOTION_GUI_DEMO.md).
- Historical transition characterization: [EXP-01](docs/EXPERIMENT_01.md),
  [EXP-01A](docs/EXP_01A_LIGHTNAV_LATENCY.md),
  [EXP-01B](docs/EXP_01B_ONLINE_RAW_SWITCH.md),
  [EXP-01B extension](docs/EXP_01B_EXTENSION.md), and
  [redesigned controlled latency](docs/EXP_01B_REDESIGNED_CONTROLLED_LATENCY.md).
- Historical reconciliation: [EXP-02 pilot](docs/EXP_02_ORACLE_GRAPH.md),
  [EXP-02A](docs/EXP_02A_SPATIAL_ENTRY_RECONCILIATION.md),
  [EXP-02B](docs/EXP_02B_CONTROLLER_AWARE_RECONCILIATION.md),
  [EXP-02B GUI diagnosis](docs/EXP_02B_GUI_DIAGNOSIS.md),
  [EXP-02B-R](docs/EXP_02B_CALIBRATED_REEVALUATION.md), and
  [EXP-02C](docs/EXP_02C_FACTOR_ISOLATION.md), and
  [EXP-02D](docs/EXP_02D_LOOKAHEAD_DIRECTION.md), and
  [EXP-02D candidate physical execution and GUI](docs/EXP_02D_PHYSICAL_EXECUTION.md), and
  [matched historical/Exp02D physical execution videos](docs/EXP_02D_MATCHED_OBJECTIVE_VIDEOS.md), and
  [turning-case search for an exclusive Exp02D tracking pass](docs/EXP_02D_TURNING_EXCLUSIVE_SUCCESS.md).
- Chronology and commands actually run: [append-only work log](docs/WORK_LOG.md).

## Local environment

Keep the repositories separate:

```text
~/Workspace/
├── se-3-reconciliation/       # this repository, Python 3.12 .venv
└── external/
    ├── LightNav-0-official-demo/  # pinned upstream source and separate model/MPC environments
    └── LightNav-0/               # external checkpoint storage
```

Reproduce the research test environment without changing system Python, ROS 2, CUDA, Isaac
Sim, or LightNav:

```bash
cd ~/Workspace/se-3-reconciliation
uv venv --python /usr/bin/python3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

## Validation

Run the repository test suite with external pytest plugin autoload disabled:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

Synthetic fixtures are tests and mechanism demonstrations only. They are never experimental
evidence.

Validate the retained online primary and inspect its recorded GUI:

```bash
.venv/bin/python scripts/validate_robotless_online_handoffs.py \
  data/robotless_online_handoffs_v1/primary_20260915T091900Z
bash scripts/launch_robotless_online_replay.sh \
  --run data/robotless_online_handoffs_v1/primary_20260915T091900Z \
  --episode episode_000_repeat_00
```

The replay is labelled `RECORDED ONLINE EPISODE REPLAY` and displays saved states and RGB.
Blue is OLD, magenta is FRESH, green is actual execution during inference, orange is actual
post-switch execution, and yellow marks the current state and handoff boundary. Play/pause,
reset, handoff selection and speed controls operate only on the saved stream. The GUI remains
open until closed; `--verify --no-hold` runs its verification sequence and exits, writing to a
new replay output directory.

Historical Jackal, EXP and DATA-02 launch and validation commands remain in their linked
reports and scripts. Their previous local inputs were deleted in the 2026-09-18 cleanup;
those demonstrations are no longer available from the removed data paths.
