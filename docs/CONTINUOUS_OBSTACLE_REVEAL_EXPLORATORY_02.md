# CONTINUOUS_OBSTACLE_REVEAL_EXPLORATORY_02

## Frozen protocol before science

Starting HEAD / fetched origin/main:
`0f21559cbac6cda0d58b28870b2d823b4de7e409`.
Run: `data/continuous_obstacle_reveal_exploratory_02/primary_20261006`.
One newly authorized episode, one full Isaac launch, no retries, no source search,
and at most four terminal predictions C0–C3. This is exploratory source
acquisition, not navigation safety validation or a reconciliation experiment.

### Sole scientific intervention

01B generated a C1 with 0.00799722992900126 m footprint-edge clearance and no
physical overlap. Its extra 0.05 m reserve gate rejected installation. This
experiment changes only the required **additional** clearance to **0.00 m**.
The circular robot radius remains **0.20 m**. Both complete native-reference
acceptance and the runtime command guard use this policy. The legacy 0.05 m
checks are retained as descriptive diagnostics.

The isolated `ExploratoryEnvironment` delegates both thresholds to the same
unchanged direct Hospital/cart checker. Numerical tolerance, obstacle projection
and uncertainty, workspace convention, swept-circle geometry and curved-command
sagitta bound are unchanged. Unknown workspace and borderline checks fail closed.
A negative footprint-edge clearance cannot pass the zero-reserve criterion.
No unsafe command is applied, no rejected reference is installed, and no repair,
replacement prediction, clipping or alternative method is permitted.

All historical source files and defaults remain byte-identical. The new Isaac
entry point binds only a different geometry-worker class into the historical
collector process. That worker uses the historical RPC and command guard with
the explicit margin adapter. The collector, request policy, reveal rule, command
computation and model worker are reused. The independent saved-only validator
and four-PNG reporter derive from the historical implementations in this separate
namespace to retain historical hash validation.

The geometry adapter computes both thresholds during collection; that bookkeeping
may consume host time, which remains measured. No simulation pause, catch-up or
logical schedule replacement is introduced.

### Authenticated source and unchanged inputs

01B: `data/continuous_obstacle_reveal_episode_01b/primary_20261006T100411Z`,
freeze `86c54fbd78b66282c4eb6707a8b9f38e5faba12e`.
Its complete saved validator is reexecuted without scientific calls; its tracked
results are compared with exact Git objects at the starting HEAD. A 140-file
manifest preserves the old run/results, in addition to 2,389 inherited authority
files. OSA03 REPEAT_00 is independently revalidated saved-only.

The POSE11, scene/cart transform, side passages, runtime YAML, episode schedule
and external authority files are copied byte-for-byte. The protocol delta is
limited to namespace, explicitly declared clearance policy and classification.
The instruction, camera/BRIGHT, official MPC and LightNav source/checkpoint,
generation parameters and history processing are unchanged.

Initial world pose `[x m, y m, yaw rad]`:
`[19.20312073159454, 24.423334915767065, -1.5689742328041627]`.
Initial physical command `[0,0]`. Instruction:

> Go to the far end of the hallway. If the path is blocked, pass the supply cart on your right without touching it. Continue straight after passing it and stop at the end of the hallway.

The margin is a downstream acceptance policy. LightNav receives the unchanged
RGB/history and instruction interface: `seq`, encoded `image`, and `instruction`.
It receives no robot radius, clearance threshold, cart coordinates or map. The
saved-only audit checks payload keys, instruction bytes/hash, JPEG bytes/hash,
full ordered history IDs/hashes, reconstructed SlowFast selections and raw
response/action hashes. Reconstructed sampler indices are not server telemetry.
No replay or determinism investigation is part of this task.

### Continuous execution and frames

- One session/login/reset at initialization; no subsequent reset or reconnect.
- Integration 60 Hz; MPC every six ticks; RGB every 15 ticks. The unchanged
  `minimum_wall_step_no_catchup_v1` scheduler never pauses for inference.
- C0 uses the fourth genuine stationary capture after three buffer-only frames.
  Cart is OFF during C0 observation and inference.
- Reveal exactly once before the first scheduled capture strictly after C0's
  first physical command application. That exact frame requests C1.
- C2 and C3 use the first eligible scheduled capture strictly after the preceding
  actual application, with one terminal request in flight. No frame substitution,
  C4, cart movement or second reveal.
- Each immutable raw chunk is anchored at its own measured observation A_k:
  `T_world_chunk = T_world_Ak * T_Ak_chunk`. Local +x forward/+y left; world XY
  metres, +Z up, CCW radians. B_k is actual first-command application, not receipt
  or installation. No B re-anchoring; intrinsic waypoint dt remains null.
- Same 4 s active cap, .10 s postroll after C3, timeouts and official MPC memory,
  generation/stale handling, nearest+1 selector and H=5.

The process-local launch is the exact successful OSA03/01B environment sanitation
(11 removed variables, four assignments). Read-only relocation checks require
Isaac's bundled nvJitLink with no missing symbols before launch. Actual mapped
library paths/hashes are observed via `/proc`. No system or external file changes.
The new passive observer catches OSError, including permission failures, and
records them without stopping or relaunching the scientific child. This fixes
01B's administrative observer failure and does not interact with simulation.

### Descriptors and predeclared outcomes

C0→C1 retains the frozen response geometry descriptors, including cart-OFF C0,
hypothetical cart-ON C0, continuous interior separation, tangent/yaw differences,
progress and cart pixels. An exploratory first reaction requires actual C0/C1
applications, reveal, nonSTOP C1, meaningful change and a passing exploratory
reference. Historical moving-B/future-progress qualification is descriptive only.

For C1→C2 and C2→C3, record raw hash equality, endpoint forward/lateral/wrapped
yaw delta, wrapped net-yaw delta, max absolute lateral/yaw excursion change,
symmetric vertex-to-continuous-polyline distance, observation translation/yaw,
A→B travel, physical and controller commands, pixels and both clearance flags.
Meaningful change uses the frozen OR thresholds: separation >=.02 m, absolute
endpoint lateral change >=.02 m, or endpoint/net yaw change >=5 degrees.

Pairs receive `C1_TO_C2_EVOLVING/STABLE` and `C2_TO_C3_EVOLVING/STABLE`.
Episode intent is `POST_REVEAL_INTENT_EVOLVING` if either observed pair changes;
`POST_REVEAL_INTENT_STABLE` requires **both** observed pairs below threshold.
Missing pairs are N/A and cannot establish stability.

Top-level categories, in order:

1. `CONTINUOUS_EXPLORATORY_C0_C3_ACQUIRED`: all four generated/applied, one reveal,
   no reset, exploratory reference/actual execution checks pass, timing/provenance
   and source-bundle validation pass. Legacy 5 cm failures are allowed and shown.
2. `PARTIAL_CONTINUOUS_EXPLORATORY_SEQUENCE`: C0 and C1 applied, but no C3
   application. Exact overlap/guard/STOP/controller/timeout/cap/failure cause stays
   in the result; missing continuation is not fabricated.
3. `FIRST_POST_REVEAL_REFERENCE_OVERLAPS`: C1 physically overlaps; never install it.
4. `TECHNICAL_EXECUTION_BLOCKED`: restoration, runtime infrastructure, timing or
   validator failure prevents qualified collection.

A behavior-only stop before C1 application, without physical overlap (for example
STOP or borderline clearance), is not covered by the requested first three
categories. Its classification remains null with the exact reason; it is not
relabeled an infrastructure failure. This coverage rule is frozen before science.

### Saved-only validation, outputs and limits

Every guarded interval retains exploratory and legacy results. The report also
rechecks the **actual one-step executed interval** separately from a new command's
nominal .1 s guard lookahead. Limiting Hospital/cart group diagnostics use the
same sweep; no future state enters controller computation.

The bundle preserves raw chunks/world installation, A/B/P, observation/request/
receipt/readiness/install/application clocks, exact requests/RGB/history,
controller previous_control and physical commands, generation/reference versions,
and actual OLD execution segments. A manifest authenticates both raw sources and
derived indices. A partial bundle stays explicitly partial.

Exactly four primary PNGs: world episode (equal axes, separate .20 m physical and
.25 m legacy boundaries), own-local geometry/yaw, timeline/active identity, and
exact RGB sequence with metadata outside images. Missing chunks are N/A. Numeric
plot inputs and large arrays stay under ignored data; compact JSON/CSV, hashes
and PNGs are tracked. Optional fifth figure is omitted as redundant.

No optimizer, graph, B_ENTRY, canonical method, Hermite/spline, transport,
correspondence, repair or follow-on reconciliation is implemented or called.
Even acquisition of C3 establishes only successive native behavior in this one
exploratory episode. It does not establish navigation safety, complete obstacle
bypass, right-side instruction reliability, real-world improvement, graph benefit
or general multi-chunk navigation success. Any overlap-free claim is limited to
the frozen 0.20 m footprint checker with **no additional 5 cm reserve**.

## Commands

```bash
git fetch origin main
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode prepare --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python -m pytest -q tests/test_continuous_obstacle_reveal_exploratory02.py tests/test_continuous_obstacle_reveal_episode01.py tests/test_continuous_obstacle_reveal_episode01b.py tests/test_obstacle_source*.py tests/test_join_online*.py tests/test_online*.py tests/test_robotless_online*.py tests/test_data02*.py tests/test_se2*.py tests/test_join_source02*.py tests/test_gp_se2_environment*.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode freeze --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
# Review/stage only this experiment; check staged diff; commit and normal push.
# Confirm HEAD == origin/main before server warmup or launch.
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode verify --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode start --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode launch --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_continuous_obstacle_reveal_exploratory02.py --mode stop --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/validate_continuous_obstacle_reveal_exploratory02.py --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006 --seal
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/report_continuous_obstacle_reveal_exploratory02.py --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
# Recheck without --seal and with reporter --validate; inspect all PNGs.
```

## Repository-confirmed results

Scientific freeze **`d9d29e73b8261702b7c3e5b3249d3b82c58a9226`** was committed and pushed before server warmup and the single Isaac launch. Both returned normally. The full launch exit code was 0; the owned official model server was identity-checked and stopped.

**PARTIAL_CONTINUOUS_EXPLORATORY_SEQUENCE**. All four genuine nonSTOP chunks were generated. C0, C1 and C2 were installed and physically applied. C3 had a physical footprint overlap and was rejected before installation: no B3, C3 command, post-C3 postroll or fabricated continuation exists. Runtime status is `SCENE_INVALID`, with preserved policy reason `RAW_UNSAFE`; the concrete cause is C3 `PHYSICAL_OVERLAP`, not a legacy-margin-only rejection. End simulation time: 2.533333465 s.

### Generated geometry and clearance

| Chunk | Generated / applied | N | XY arc m | Endpoint local [forward, left, yaw rad] | Max absolute local lateral m | Max absolute local yaw deg | Cart pixels | Footprint-edge clearance m | Exploratory pass | Legacy 5 cm pass |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| C0 | yes / yes | 10 | 1.351820588 | `[1.501985788345337, 3.947936875192681e-06, 8.423162398685236e-06]` | 0.000003948 | 0.000901 | 0 | 1.117514011 | True | True |
| C1 | yes / yes | 10 | 1.366089584 | `[1.389505386352539, -0.5808053612709045, -0.3078382909297943]` | 0.580805361 | 29.598703 | 4080 | 0.006270831 | True | False |
| C2 | yes / yes | 10 | 1.148547455 | `[1.0552818775177002, -0.39522966742515564, 0.07674673944711685]` | 0.398723036 | 30.746192 | 8168 | 0.047950498 | True | False |
| C3 | yes / NO | 10 | 1.244421020 | `[1.2275692224502563, 0.5463818311691284, 0.8079448342323303]` | 0.546381831 | 46.291829 | 20492 | -0.139375182 | False | False |

C0 clearance above uses its actual cart-OFF reference gate. The same finite C0 with the cart ON has **-0.125215494 m**, physical overlap. C1/C2 are positive-clearance references below the old 5 cm reserve; C3 is negative-clearance and never applied. These are three different facts, not interchangeable safety labels.

All four raw outputs contain ten untimed spatial rows. Their wrapped net yaw changes are C0: 0.000010226 rad, C1: -0.114034578 rad, C2: 0.391207851 rad, C3: 0.771915231 rad. No row was deleted, smoothed, transported, optimized or re-anchored.

### Observation/application boundaries and control

| Chunk | A = actual observation [world x, y, yaw] | B = first physical application |
| --- | --- | --- |
| C0 | `[19.20312073159454, 24.423334915767065, -1.5689742328041627]` | `[19.20312073159454, 24.423334915767065, -1.5689742328041627]` |
| C1 | `[19.20324823224938, 24.353335024838714, -1.5689753047983097]` | `[19.203636616578333, 24.140002031998613, -1.568976033476454]` |
| C2 | `[19.190818601626948, 23.967405680751945, -1.7606430287069978]` | `[19.110279716340877, 23.756455015085873, -2.0648916619277466]` |
| C3 | `[19.016978548357283, 23.610496889641936, -2.183117764632593]` | N/A |

| Chunk | Observation sim s | Ready seen sim s | Install sim s | First application sim s | A→B executed travel m |
| --- | --- | --- | --- | --- | --- |
| C0 | 0.783333374 | 1.016666720 | 1.016666720 | 1.100000057 | 0.000000000 |
| C1 | 1.283333400 | 1.533333413 | 1.533333413 | 1.566666748 | 0.213333346 |
| C2 | 1.783333426 | 2.033333439 | 2.033333439 | 2.066666774 | 0.226663422 |
| C3 | 2.283333452 | 2.533333465 | N/A | N/A | N/A |

Reveal was exactly at C1 observation, state/tick 75, simulation 1.283333400 s. The cart was ON and at the same transform for all subsequent captures. C2/C3 used states 105/135, the first eligible scheduled captures strictly after C1/C2 physical application. No reset after initialization.

| Boundary | P immediately before B | physical u_minus [m/s, rad/s] | first applied command / controller previous_control at B |
| --- | --- | --- | --- |
| B1 | `[19.203612345850008, 24.153335343937258, -1.568976009043844]` | `[0.8, -1.4659565148272683e-06]` | `[0.8, -0.5000014717940552]` |
| B2 | `[19.11652910466556, 23.768232986610393, -2.0523543133236717]` | `[0.8, -0.7522408770120824]` | `[0.8, -0.8831926717266878]` |
| B3 | N/A | N/A | N/A |

The previous_control value is the official controller memory confirmed by the accepted result at B; it can differ from the physically held incoming u_minus. Both provenance records and the before-solve memory remain in the bundle. No memory reset or reinterpretation was used.

### Raw hashes and history provenance

| Chunk | raw_local.npy SHA256 | Delivered frame IDs at prediction |
| --- | --- | --- |
| C0 | `6d53cfff6ca770055ca6563548fc80b6696e2336fd4343f35c2c4e01da1bd5ae` | frame_000000, frame_000001, frame_000002, frame_000003 |
| C1 | `bee96feb024de770e98f217d60c7262830a76fadd8abf2a29d499e64b94b25be` | frame_000000, frame_000001, frame_000002, frame_000003, frame_000004, frame_000005 |
| C2 | `55ee7cc557211e6c0b21c923ccce236734f49201c0b9278a7dc229c23e32b5b9` | frame_000000, frame_000001, frame_000002, frame_000003, frame_000004, frame_000005, frame_000006, frame_000007 |
| C3 | `42ad4ee92aad07e8f08dc3b7a5ad4084be79860297ab15fc3be2905043582202` | frame_000000, frame_000001, frame_000002, frame_000003, frame_000004, frame_000005, frame_000006, frame_000007, frame_000008, frame_000009 |

Each exact RGB hash, ordered history hash list, reconstructed SlowFast segment indices, instruction text/hash, serialized request hash and raw response hash is in `result_summary.json → chunks → history_provenance`; the complete payloads/images are in the episode. The triggering history counts are 4, 6, 8 and 10. The activation log may show a later session history count after an intervening buffer append; that log count is not substituted for the frozen prediction snapshot. No determinism replay or investigation was performed.

### First reaction and successive evolution

The exploratory first-reaction gate passes. C0 is approximately straight and cart-OFF overlap-free; placing the revealed cart obstructs that finite reference. C1 is genuine, nonSTOP, has positive footprint clearance and was actually applied. C0→C1 maximum interior separation is **0.580810367 m**, reliable tangent difference **25.948880°**, and reliable yaw difference **26.709126°**. Returned world-forward progress for C0/C1 is 1.351820476 / 1.240565034 m. These descriptors do not establish semantic recognition or complete bypass.

| Pair | Raw hash equal | Symmetric local separation m | Endpoint forward / lateral delta m | Endpoint yaw delta deg | Net yaw delta deg | Max absolute lateral delta m | Max absolute yaw delta deg | Observation translation m | Observation yaw delta deg | Classification |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| C1->C2 | False | 0.382287447 | -0.334223509 / 0.185575694 | 22.035099 | 28.948259 | -0.182082325 | 1.147489 | 0.386129453 | -10.981752 | C1_TO_C2_EVOLVING |
| C2->C3 | False | 0.957243513 | 0.172287345 / 0.941611499 | 41.894565 | 21.812926 | 0.147658795 | 15.545637 | 0.396994017 | -24.206019 | C2_TO_C3_EVOLVING |

Episode evolution classification: **POST_REVEAL_INTENT_EVOLVING**. Both later pairs are observable raw predictions and cross the frozen thresholds; C3 remains an unapplied prediction.

### Executed prefix versus reference geometry

The actual 150 executed integration intervals have minimum swept footprint-clearance lower bound **0.377713861 m**. All 150 pass the legacy 5 cm check as well. The more conservative pre-application guard lookahead minimum is **0.365956649 m**. No execution guard abort and no executed physical overlap were observed. The run stopped on the rejected C3 reference, while the actual robot was still clear. Per-interval checks, limiting Hospital/cart diagnostics and both clearance flags are in the JSON sidecar.

Request-local RTF values: 0.991405, 0.989706, 0.989838, 0.990889. Maximum recorded loop interval/stall: 0.219475798 s. Frozen timing qualification and scheduler checks pass; no catch-up burst or inference pause was observed.

### Counts, validation and source bundle

| Item | Count |
| --- | --- |
| Full Isaac launches / initialized episodes / completed bounded records | 1 / 1 / 1 |
| Terminal LightNav / buffer-only / startup warmup | 4 / 6 / 1 |
| RGB captures / reveals | 10 / 1 |
| MPC submissions / solved / successful / new physical applications | 15 / 15 / 15 / 15 |
| Integration intervals / saved states | 150 / 151 |
| Initialization resets / later resets | 1 / 0 |
| Graph / canonical / B_ENTRY / Hermite / V2 / GP / transport / reconciliation | all 0 |
| Retries / new source searches | 0 / 0 |
| Saved-only validation / report scientific calls | 0 |

Regression: **588 passed, 1 skipped**; new focused tests: **30** (included in the regression count). The skip is the unavailable immutable DATA02 generated v1 corpus. Compileall, unstaged/staged diff checks and saved-only scientific validation pass. Historical 01B and OSA03 source hashes remain unchanged.

Bundle: `data/continuous_obstacle_reveal_exploratory_02/primary_20261006/source_bundle/`. The independent bundle check authenticates **122 raw episode files and 11 derived files**. `validated_partial_data=true`, **`valid_continuous_C0_C3_source=false`**. The bundle includes C0→C1 and C1→C2 actual handoffs and all four generated chunks; no applied C2→C3 handoff is invented.

### Four PNGs and numeric sidecars

1. [Continuous world episode](../results/continuous_obstacle_reveal_exploratory_02/figures/continuous_world_episode.png): solid dark curve is actual execution; colored references include unapplied C3. Inner contour is the .20 m footprint boundary; outer dashed contour adds the old .05 m reserve. C0 was observed/executed initially with the cart OFF.
2. [Own-observation local evolution](../results/continuous_obstacle_reveal_exploratory_02/figures/observation_local_evolution.png): +x forward/+y left, equal XY axes; pose yaw is spatial and is not robot omega.
3. [Timeline and active identity](../results/continuous_obstacle_reveal_exploratory_02/figures/episode_timeline.png): C3 receipt/readiness exists, but no application marker or active-C3 segment.
4. [Exact request RGB sequence](../results/continuous_obstacle_reveal_exploratory_02/figures/request_rgb_sequence.png): metadata outside the original images; C3 explicitly marked rejected.

All four PNGs and numeric sidecars were visually inspected. `chunks.csv`, `result_summary.json`, `execution_audit.json`, `figure_manifest.json` and `report_validation.json` retain compact outputs. Full plot arrays remain in ignored data.

### Protocol deviations and administrative observations

- No scientific method/configuration changes, retries, extra launch, replacement prediction or post-result tuning. C3 rejection and missing postroll are the predeclared fail-closed outcome.
- The passive `/proc` observer encountered one PermissionError during shutdown. The frozen new handler recorded it and the child exit code 0 without interrupting acquisition. Actual Isaac-bundled nvJitLink mapping and its SHA256 were captured before shutdown.
- The initial RGB report placed lower-row metadata too close to the upper images. A separate **saved-only presentation correction** rerendered that one PNG with dedicated metadata axes and applied/rejected labels. The frozen reporter/scientific code were not edited. Original PNG/manifest bytes remain under `data/.../reporting/original_frozen_report/`; both hashes and correction-script hash are in the final manifest. This is a reporting-only deviation, with zero scientific calls.
- CSV newline normalization to LF is presentation-only. The original runtime `raw_reference_safe` / `C_ON_WHOLE_SAFE` field names are retained for compatibility; in this experiment they denote the explicitly recorded zero-extra-reserve checker, not a navigation-safety claim.

Additional reporting command:

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/explore02-mpl .venv/bin/python scripts/report_continuous_obstacle_reveal_exploratory02_layout.py --run data/continuous_obstacle_reveal_exploratory_02/primary_20261006
```

## Research interpretation

Removing only the extra reserve allowed this episode to apply C1 and C2 and to observe the subsequent C3 prediction. The measured local references continued to change: C1/C2 extend to the local right, while C3 extends to the local left at a different observation pose and heading. Both later pairs satisfy the frozen evolution thresholds. This is evidence of successive native geometry in one uninterrupted episode, not a causal proof of obstacle understanding or a stable general navigation policy.

The acquisition objective was partially achieved: four genuine raw outputs and two real post-C0 handoffs are available, but all four chunks were not executed. C3 physical overlap means the zero-reserve policy still correctly blocked a generated reference. The valid executed prefix and low-margin references must be evaluated separately.

## Claim boundary

Collision-free **executed prefix under the frozen 0.20 m footprint checker**, with the additional 5 cm execution reserve disabled for this exploratory acquisition. The executed prefix happened to retain more than 5 cm, while C1/C2 whole references did not; C3 was rejected for actual predicted footprint overlap. This is not validated navigation safety, complete obstacle bypass, task completion, reliable side compliance, graph necessity/benefit, real-world improvement or population evidence. No reconciliation stage follows in this task.
