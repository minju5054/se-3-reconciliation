# SE(3) Reconciliation Research

This repository studies how to reconcile successive Navigation VLA action chunks across an
OLD-to-FRESH transition while preserving the intent of the new trajectory. LightNav-0 emits
arbitrary `N x 3 [x, y, yaw]` SE(2) waypoint references; the reconciliation method developed
here is separate from LightNav.

## Current research state

The [JOIN-ONLINE-03 destination-first instruction rerun](docs/JOIN_ONLINE_03_DESTINATION_INSTRUCTION.md)
completed four new native online OFF/ON episodes with the user's exact wording.
Both cart-ON repeats produced a larger lateral response and continued instead
of STOP, but later raw futures were unsafe. The unchanged oracle guard halted
before required clearance would fall below .05 m; actual minimum edge clearance
was .05330 / .05373 m. No collision was executed and no complete bypass source
was obtained. Request-local pacing limitations remain; this comparison uses
new live histories, not identical RGB. Official LightNav/MPC were unchanged,
and no reconciliation ran. See the
[saved review](data/robotless_join_online_03/primary_20260923T031300Z/index.html).

The [JOIN-ONLINE-02 far-approach audit](docs/JOIN_ONLINE_02_FAR_APPROACH.md)
ran four genuine native online OFF/ON episodes from the frozen 5 m start.
Both cart-ON repeats generated seven responses: forward chunks shortened from
1.356 m to 1.056 m to 0.511 m, then native STOP. Neither generated a lateral
bypass or triggered the oracle command-abort guard; actual minimum edge
clearance stayed about 0.355 m. OFF continued beyond the absent cart location.
All ON request-local RTF checks exceed the prior 1.2 upper bound, so these are
recorded behavior observations with pacing limitations, not timing-qualified
bypass sources. No reconciliation ran. See the
[saved static review](data/robotless_join_online_02/primary_20260923T001500Z/review/index.html)
and the report's interactive Isaac replay command.

A [saved STOP-cause audit](docs/JOIN_ONLINE_02_STOP_CAUSE_AUDIT.md) confirms
explicit native STOP action codes. In both ON stop frames the target-point
OPOS cell lies entirely on the cart, whereas prior OPOS centres do not.
This supports a target-grounding hypothesis but does not separate target
confusion from occlusion/blocked-route stopping. No new inference or execution.

The [JOIN-SOURCE-05 instruction diagnostic](docs/JOIN_SOURCE_05_INSTRUCTION_AVOIDANCE.md)
reused SOURCE04 RGB bytes for12 new predictions with two explicit cart-avoidance
instructions. K0 trajectories stayed identical to the historical straight path.
Six of eight non-sham cart-ON outputs passed raw clearance, including one STOP;
the five safe non-STOP outputs were short forward chunks ending before the cart. No lateral bypass
was generated (all new ON lateral magnitudes below1.1mm). Frozen classification
is **INCONCLUSIVE: safe shortening/STOP, no bypass recovery**. Both shams match.
No rendering, MPC, online handoff or reconciliation ran; see the
[saved review](data/robotless_join_source_05/instruction_20260922T125204Z/index_final.html).

The [saved execution loss audit](docs/SAVED_HANDOFF_EXECUTION_LOSS.md) measures
the 13 screened native handoffs after B. Position error initially grows before
convergence; the hard013/01/024 case averages34.4cm separation over the common
first0.9s. One sampled0.30s attachment is observed, four are dwell-censored by
the next chunk, one enters then exits, and seven never enter during observation.
All13 recorded post-switch paths pass the reused clearance checker. This is
evidence of local tracking/attachment cost, not population failure frequency,
collision prevention, or demonstrated optimization benefit.

The [source-only moving/mismatch scan](docs/GENUINE_SOURCE_MOVING_MISMATCH_SCAN.md)
finds 13 of 881 genuine handoffs with safe original FRESH/common geometry,
physical v−>0.20m/s, observation-to-B travel≥0.02m, and substantial lateral or
orientation mismatch. Eleven have interior FRESH projections; two have nearly
exhausted endpoint geometry. None also satisfies the prior retained-suffix
obstacle-near criterion (edge clearance≤0.20m). This is an input inventory, not
new inference, optimization, execution, or evidence of reconciliation benefit.
The saved Isaac viewer can also replay each case's original **post-switch FRESH
execution** through the next chunk activation; the corpus was not plan-only.
Reference mismatch and remaining tracking error are not automatically execution
failure or evidence that optimization would improve the result.

The [JOIN-SOURCE-04 persistence/affordance diagnosis](docs/JOIN_SOURCE_04_PERSISTENCE_AFFORDANCE.md) completed seven independent terminal predictions using the same 16 moving poses, all re-rendered BRIGHT, with the cart present in the final 0/1/2/4/8 frames. K2/K4/K8 improved minimum clearance versus K1 by 3.19–6.23 cm, but every ON raw path still overlapped the cart at the same early segment; no bypass source was obtained. Both shams matched exactly. All ON APOS were bottom-clamped, so a precise free-space direction or affordance/action mismatch remains unidentifiable. This is a counterfactual input-history diagnostic, with no online execution or reconciliation. See the [complete static review](data/robotless_join_source_04/persistence_20260922T050237Z/index_final.html).

The [JOIN-SOURCE-03 bright Hospital diagnosis](docs/JOIN_SOURCE_03_BRIGHT_CAUSE.md) completed seven frozen OFF/ON/SHAM conditions (21 terminal predictions). Actual scene lighting increased mean image luma from 12.27 to 77.48; the core ON path's minimum edge clearance improved from −0.2000 to −0.1114 m, but every ON path still overlapped the cart. H32 and the tested cart distances did not recover a safe detour. Official MPC provenance matches, and no diagnostic MPC execution occurred. **Lighting contributes but is insufficient; the internal failure cause remains unresolved.** The intervention changed the terminal RGB while retaining authentic dark history. No new online handoff or optimization-ready source was obtained. See the [actual RGB and paired static review](data/robotless_join_source_03/cause_20260921T153120Z/index.html).

The [JOIN-SOURCE-02 source screening](docs/JOIN_SOURCE_02_OBSTACLE_RESPONSE.md) ended with **NO_QUALIFYING_DEVELOPMENT_SCENARIO**. Actual Hospital cart visibility and mesh geometry were verified. Two complete off/on/sham H16 conditions changed the predicted yaw and path length, but both obstacle-on paths overlapped the cart; off/sham arrays were identical. The remaining ten declared conditions lacked sufficient authentic history or safe placement reserve. No online confirmation or reconciliation comparison ran, and no qualifying source bundle was produced. See the [saved RGB/trajectory review](data/robotless_join_source_02/source_20260921T112111Z/review_v2/index.html). This bounded source-discovery result does not establish a LightNav failure cause or general obstacle-avoidance performance.

The [fixed-correspondence duration experiment](docs/GP_SE2_ATTACH_01_FIXED_CORRESPONDENCE_DURATION.md) completed 14 GP starts and four official-MPC counterfactual rollouts on one source-selected obstacle-sensitive handoff. With identical prepared FRESH rows across all six external durations, T=1.4s supplies a full-valid retained plan and safe sustained execution. Attachment is .170s versus Native/Adapter .155s; linear command variation decreases, while attachment and goal arrival are later. This is a constructibility result with a trade-off, not faster attachment. T=1.0s is excluded solely by a nominal tube-boundary roundoff excess. The selected B is nearly stationary and the qualifying position gap is along the path, so large lateral mismatch remains untested. See the [complete static review](data/robotless_gp_se2_attach_01/primary_20260921T103000Z/review_v2/index.html). Correspondence and duration were not optimized; JOIN-01 remains preserved.

The new [obstacle-reveal attachment experiment](docs/GP_SE2_JOIN_01_OBSTACLE_REVEAL.md) implements an explicit original-FRESH join objective and evaluates its upstream source gate first. All three predeclared Hospital box placements produced genuine OLD/FRESH and application-time B records, but **none qualified**: FRESH stayed nearly straight and intersected the box while B remained safe. Therefore no GP/rigid comparison or common-B rollout ran. Actual RGB shows the box, although semantic visibility labels were unavailable; the first attempt also fails the pacing gate. See the [static source review](data/robotless_gp_se2_join_01/primary_20260921T083025Z/review/index.html). This is an upstream qualification failure with instrumentation limitations, not evidence for or against GP attachment performance.

The [fixed endpoint-reserve experiment](docs/GP_SE2_DIAG_08_ENDPOINT_MARGIN_EXECUTION.md) completed five new G4 solves with a predeclared 4 cm planning reserve (11 cm plan radius; original 15 cm execution criterion unchanged). All converge, but all four hard plans regain an interior negative-speed violation as well as lateral rejection, so no hard G4 rollout is admitted. One newly executed benign reference reproduces success. The result is **endpoint reserve not plan-feasible under this frozen sampled formulation**, not a mathematical infeasibility proof or measured hard execution failure. See the [static review](data/robotless_gp_se2_diag_08/primary_20260920T151000Z/presentation/index.html); import/report corrections and final authoritative validation are disclosed.

The [lateral-only plan/execution diagnostic](docs/GP_SE2_DIAG_07_LATERAL_PLAN_VS_EXECUTION.md) newly executed four unchanged rejected hard G3 references and one full-valid benign reference. Hard executions pass safety, motion and yaw but fail original 3 s position/dwell (about 0.16993 m vs 0.15 m); benign succeeds. Plan invalidity is preserved, and lateral velocity is not established as the failure cause. The [static execution review](data/robotless_gp_se2_diag_07/primary_20260920T103000Z/presentation/index.html) separates plan and actual execution.

The [frozen extremum-witness refinement](docs/GP_SE2_DIAG_06_EXTREMUM_WITNESS_INEQUALITY.md) completed exactly five new G3 solves with one common three-row motion-inequality union. All five converge. The four hard finals pass original speed/acceleration checks at unchanged tolerance, but full acceptance still fails on lateral velocity alone (0/4 hard candidates). Benign optimization remains full-valid. Original equality/core/thresholds are unchanged; no second refinement or execution occurred. See the [compact static review](data/robotless_gp_se2_diag_06/primary_20260920T083200Z/compact_review/index.html). This is finite sampled evidence on development events, not continuous feasibility or navigation improvement.

The [quarter inequality-only ablation](docs/GP_SE2_DIAG_05_QUARTER_INEQUALITY_ONLY.md) completed exactly five new G2 solves, using saved DIAG-04 G0/G1 baselines. Restoring the original 30 lateral equality rows recovers SLSQP convergence in all five starts, and benign optimization recovers a full-valid lower-cost candidate. Hard full recovery remains 0/4: between-point lateral velocity, small negative speed and linear acceleration violations remain despite reduced magnitudes. Equality values/Jacobians match G0 literally; physical acceptance and numerical core are unchanged. See the [static comparison index](data/robotless_gp_se2_diag_05/primary_20260920T064500Z/index.html). No MPC, rollout, GUI or navigation improvement is claimed.

The [quarter-point GP refinement](docs/GP_SE2_DIAG_04_QUARTER_CONSTRAINTS.md) completed ten new paired solves after verifying all 15 frozen derivative records. No hard full-valid candidate recovered: G0 converges but fails between-point checks; G1 encounters SLSQP failures. The benign G1 result retains the original valid seed without optimization improvement. Equality-rank diagnostics expose dependent/ill-conditioned added lateral rows; this does not prove infeasibility. See the [figures and review index](data/robotless_gp_se2_diag_04/presentation_20260920T020100Z/index.html). Original acceptance/core are unchanged; no MPC, rollout or GUI was run.

The [saved GP interpolation audit](docs/GP_SE2_DIAG_03_INTERPOLATION_FEASIBILITY.md) reproduces all nine prescribed GP-SE2-02 records. Four converged hard finals pass collocation but fail between-point lateral velocity, forward speed and body acceleration checks; independent coefficient and temporal-derivative checks agree. Quarter-point queries detect every observed violation pocket, but no refined optimization or execution was performed. The benign reference remains valid. See the [static figures and review index](data/robotless_gp_se2_diag_03/presentation_20260920T011000Z/index.html); finite sampling does not prove continuous feasibility.

The [saved stress prediction/execution audit](docs/GP_SE2_REF_04_STRESS_PREDICTION_AUDIT.md) localizes the REF-03 source-progress failure without any new solve or rollout. All selected target points and target-to-target polylines remain clearance-valid, but the t=0 MPC prediction already contains an unsafe future tail; the t=0.2 prediction includes an unsafe first applied interval. Actual required-clearance failure begins in [0.261875, 0.262500] s, followed by a finite-gate crossing 15.158 mm outside its valid interval. There is no physical overlap, and successful final goal/dwell does not restore full success. Euler/exact differences do not flip the observed first-interval clearance classifications. See the [saved audit figures](data/robotless_gp_se2_ref_04/presentation_20260919T163000Z/index.html); future control sequences are unavailable, and no GUI or GP improvement is claimed.

The [frozen selector transfer audit](docs/GP_SE2_REF_03_SELECTOR_TRANSFER.md) completed 81 offline MPC rollouts on 24 additional source-selected events and three separate controls. The known obstacle stress regresses from dense-row success to source-progress clearance/route failure (minimum clearance 0.0631→0.0361 m). Additional A/B/C success is 23/21/23: two dense-row failures recover, but another already-failing event gains a physical angular-acceleration violation associated with recorded controller memory. The result is **mixed transfer with regressions**, using frozen REF-02 selector rules and unchanged MPC computation, reference preparation and acceptance; no GP or general navigation claim. The [local static index](data/robotless_gp_se2_ref_03/primary_20260919T141000Z/index.html) contains all 27 cases and 193 figures; no GUI runtime was executed.

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

GP-SE2-DIAG-07 completed a fixed-reference offline execution diagnostic:
four unique lateral-only-invalid G3 hard references all passed actual motion,
clearance/workspace/route checks but failed the original 3 s goal position/dwell;
the full-valid benign control succeeded. No execution-success counterexample to
the strict plan lateral criterion was observed, and no lateral causal claim or
tolerance change follows. See
[plan vs execution report](docs/GP_SE2_DIAG_07_LATERAL_PLAN_VS_EXECUTION.md).
