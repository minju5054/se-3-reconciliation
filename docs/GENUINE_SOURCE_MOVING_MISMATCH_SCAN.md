# Genuine corpus: safe FRESH, moving boundary and mismatch audit

The user's requested **source-only** audit finds 13 matching events in the
881-event genuine online Hospital corpus: 7 episodes, 13 ordered raw pairs and 10 distinct FRESH raw-local arrays.
Eleven project into a nonzero FRESH segment interior; two have endpoint
projections and almost exhausted original row-order progress. Eight satisfy
both lateral and orientation mismatch, seven of these with interior projection.
All 13 OLD/FRESH raw-local hashes differ. No event was selected for optimization.
These are development-corpus sources, not unseen or independent test samples.

No new LightNav prediction, GP/rigid optimization, MPC solve, rollout, scene
export or GUI was performed. Source evidence is preserved, not re-executed.
Starting/fetched main: `3a2734ba10ec07e7fc1b3d9cd9056ff8cc838f4f`.

## Pre-scan definitions and safeguards

Protocol: `configs/genuine_source_moving_mismatch_scan.yaml`, written before
examining the new candidate counts. It reuses the ATTACH01 source scan, REF03
source hash inventory and GP01 validated direct Hospital geometry. Original
GP01 source eligibility and exact request-local timing are retained. Historical
optimizer/MPC outcome tables are not selection inputs. All 881 records remain
in the new ledger, including exclusion flags and original reasons.

- **Safe FRESH:** the entire returned original FRESH polyline AND unchanged
  prepared common reference are valid under direct swept-circle geometry,
  known workspace, radius .20m, required edge clearance .05m and original
  1e-7m geometry uncertainty. The unchanged native suffix is checked separately.
  No observation-to-first-row or B connector is inserted. Unsafe prefix deletion
  cannot promote an unsafe whole FRESH into the main result.
- **Safe recorded boundary/history:** original B is valid; observation-to-B
  recorded execution is checked with the original swept checker. Each held-command
  segment has conservative chord deviation bound `abs(v*omega)*dt^2/8`;
  the maximum is supplied to the unchanged checker. Stored commands reconstruct
  saved states within 1e-10; this is not a new closed-loop rollout.
- **Actually moving at application:** recorded physical `v_minus > .20m/s` AND
  observation-to-B accumulated travel >=.02m. Travel is the exact sum of
  `abs(v)*dt` over recorded held commands; sampled chord sum is also saved.
  Physical command and recorded MPC previous-control memory remain distinct.
  The old `VALID_HANDOFF_MOVING` label alone is insufficient.
- **Mismatch:** absolute lateral component >=.10m OR reliable incoming-to-FRESH
  window direction difference >=20deg OR projected pose-yaw difference >=20deg.
  The existing .10m projection window and .05m incoming/.02m FRESH chord
  reliability thresholds are reused. `e_perp` is a Euclidean nearest-polyline
  distance and includes endpoints: it is not automatically a lateral error.
  We decompose B−Q into tangent/normal components, with endpoint status retained.
  Pose-yaw and direction are separate, wrapped quantities. AND counts are also
  reported, not substituted for the frozen OR criterion.
- Original simple/nondegenerate FRESH, >=2 suffix rows and positive suffix arc
  remain required. This weak source condition does NOT guarantee a substantial
  future interval, timed feasibility, or a feasible 3s transition.
- Obstacle-sensitive suffix clearance <=.20m and no-required-gate status are
  **separate subsets**, not silently added to or removed from the requested
  main three-condition query. Complete raw-path clearance is also preserved.

All poses remain fixed Isaac world metres/radians, +Z up, yaw CCW. Each raw
local FRESH is transformed only by its saved observation pose, never B.
Observation/request/receipt/install/application clocks remain separate. There
are no intrinsic LightNav waypoint timestamps. The diagnostic geometric
projection is not a new attachment correspondence selector.

## Coverage and all matching records

Cumulative: 881 source records →680 base integrity/timing/future/geometry cases
→673 with whole FRESH/common safe →670 with recorded pre-B path safe
→548 physically moving →**13 with sufficient mismatch**. Independent flag
counts differ from this cumulative chain; both are saved.

| Event | v− m/s | obs→B travel m | abs lateral m | reliable direction deg | pose yaw deg | full/suffix edge clearance m | projection | raw arc after Q m |
|---|---:|---:|---:|---:|---:|---:|---|---:|
| episode_014_repeat_01/handoff_025 | 0.8000 | 0.7200 | 0.1591 | 32.73 | 36.39 | 0.1306 / 0.4341 | end_endpoint | 0.0043 |
| episode_001_repeat_01/handoff_013 | 0.7940 | 0.4921 | 0.1276 | 15.02 | 15.00 | 0.5806 / 0.9314 | interior | 1.0302 |
| episode_013_repeat_00/handoff_033 | 0.3085 | 0.5938 | 0.0058 | 20.11 | 81.64 | 0.2527 / 0.2527 | end_endpoint | 0.0000 |
| episode_016_repeat_01/handoff_007 | 0.8000 | 0.3733 | 0.0709 | 20.35 | 16.62 | 1.2263 / 1.2263 | interior | 0.8537 |
| episode_016_repeat_01/handoff_001 | 0.8000 | 0.3067 | 0.1079 | 22.91 | 23.68 | 2.4064 / 2.4064 | interior | 1.1069 |
| episode_008_repeat_01/handoff_023 | 0.2936 | 0.5528 | 0.1844 | 24.53 | 22.10 | 0.1313 / 0.2335 | interior | 0.7994 |
| episode_001_repeat_01/handoff_011 | 0.7842 | 0.5032 | 0.1306 | 15.10 | 15.16 | 0.4365 / 0.4365 | interior | 1.0190 |
| episode_013_repeat_00/handoff_020 | 0.3576 | 0.4247 | 0.1609 | 29.65 | 28.97 | 0.9184 / 1.0721 | interior | 0.9440 |
| episode_013_repeat_01/handoff_024 | 0.8000 | 0.6400 | 0.2788 | 36.54 | 36.19 | 1.9834 / 2.1584 | interior | 0.7848 |
| episode_013_repeat_01/handoff_021 | 0.5087 | 0.4620 | 0.1222 | 25.68 | 25.80 | 0.8826 / 0.9956 | interior | 0.7790 |
| episode_020_repeat_00/handoff_017 | 0.2986 | 0.3839 | 0.1001 | 29.65 | 31.37 | 0.2358 / 0.2358 | interior | 1.1231 |
| episode_013_repeat_00/handoff_026 | 0.4477 | 0.5667 | 0.1791 | 31.21 | 30.51 | 1.9045 / 2.0467 | interior | 0.6985 |
| episode_013_repeat_01/handoff_029 | 0.3112 | 0.5167 | 0.1337 | 14.99 | 15.01 | 1.1408 / 1.1408 | interior | 1.0064 |

Rows above use predeclared lexical SHA256(`MOVING-SOURCE-v1:`+case_id), then
case ID. It is an inventory order, not performance ranking or a next-experiment
selection. The six overview plots use this same order.

## Obstacle relevance and limitations

**Zero** of the 13 meet the prior ATTACH01 definition of obstacle-sensitive
**retained suffix** (minimum edge clearance <=.20m). Among otherwise valid,
safe obstacle-near sources, 46 exist and 32 are physically moving, but none
has the required true lateral/direction/yaw mismatch. The previously selected
ATTACH01 event `episode_021_repeat_01/handoff_003` has v−~1.80e-9m/s,
obs→B travel .008435m and lateral gap ~1.12e-5m. Its old e_perp~.1412m was
mostly along-path endpoint gap; it does not qualify here.

This does not mean every part of these 13 FRESH paths is far from obstacles:
`episode_014_repeat_01/handoff_025` and
`episode_008_repeat_01/handoff_023` have **entire raw-path** minimum clearance
.1306m and .1313m. Their retained suffix clearances are .4341m and .2335m.
That distinction is displayed rather than changing the .20m criterion after
inspection. B clearance is .3044m/.3808m respectively. It has not been shown
that obstacles actively constrain a future optimized connection.

The 014/025 endpoint case has only .004324m raw arc after the projection,
and the 013/00/033 endpoint case has none. They remain in the original
three-condition inventory with explicit caveats; they are not automatically
promoted as useful future-transition sources. The 11 interior cases retain
.6985–1.1231m raw XY arc after Q. Seven of these meet lateral AND orientation
mismatch. `episode_013_repeat_01/handoff_024` is the already studied hard GP
case, not a fresh held-out example; no historical solver result was consulted
by this scan.

**Conclusion:** genuine moving/mismatched, geometrically safe source inputs
exist. The stronger obstacle-near-future combination was not found under the
preserved .20m rule. Safe FRESH geometry is not proof of successful MPC tracking,
B→FRESH feasibility, instruction completion, or an optimizer benefit. No new
obstacle appearance or detour causal claim is made. This task stops at source
inventory; it does not select a correspondence, duration, or optimized candidate.

## Reproducibility and evidence

Final source-only run:
`data/genuine_source_moving_mismatch/scan_20260922T073107Z/`.
`ledger.json`/`ledger.csv` contain all881 records; `summary.json`, `source.json`,
`protocol.yaml`, `validation.json`, `index.html`, and seven PNGs with numeric/hash
sidecars provide the review. Raw data are referenced, not copied or modified.
The first scan directory `scan_20260922T073500Z` remains preserved (its suffix
is an identifier, not execution time). The second source-only recomputation
(`scan_20260922T072648Z`) adds explicit B clearance and cumulative/report counts;
the final recomputation moves plot legends outside axes after visual review.
Primary predicates and all13 candidate IDs are identical in all three saved-only
passes. No new scientific execution was retried.

```bash
MPLCONFIGDIR=/tmp/genuine_source_scan_mpl OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python scripts/scan_genuine_moving_sources.py \
  --output data/genuine_source_moving_mismatch/NEW_UNIQUE_RUN
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/scan_genuine_moving_sources.py \
  --validate --output data/genuine_source_moving_mismatch/scan_20260922T073107Z
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q \
  tests/test_genuine_source_scan.py tests/test_online_handoff_analysis.py \
  tests/test_gp_se2_environment.py tests/test_gp_se2_reference.py \
  tests/test_gp_se2_attach01.py tests/test_gp_se2_ref03_cohort.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

Saved-only validator independently reruns source geometry/motion reconstruction
and reproduces every ledger/summary record, verifies CSV/plot numbers and source
hashes. It does not call solvers. Output creation is exclusive; validation also
refuses overwriting an existing validation file. Existing numerical/controller
code, environments, raw recordings, historical outputs and both unrelated
Stage0 config edits remain unchanged.

Relevant tests: 134 passed in 4.26s; no real model/controller calls. Saved-only
validation passes on all 881 records and verifies 3,920 source hashes, code,
CSV and figure numbers. Compileall and git diff --check pass.

## Saved-source Isaac GUI, requested 2026-09-22

`scripts/isaac/genuine_source_scan_gui.py` displays all 13 qualifying sources
in their unchanged Hospital world coordinates. The default display is
`episode_008_repeat_01/handoff_023` for visual inspection only, not selection
for a new optimization experiment. Previous/Next and the dropdown expose all
13, including the two endpoint caveats. The original observation JPEG is shown
without edits. Source records, raw/world observation anchors and the exact
prepared-reference value hashes are verified before display.

Blue is original OLD, pink original FRESH, orange actual recorded execution
from FRESH observation to B, green observation pose, yellow actual B. White
footprint is .20m and the outer margin is .25m. Optional cyan overlays the
unchanged common reference without offset. The GUI also shows physical speed,
travel, lateral/direction/yaw differences, full/suffix/B clearances and remaining
raw arc. No optimized connecting path is fabricated. `Replay saved OLD to B`
selects recorded samples only, stops at B, and runs no integrator/controller.

The renderer uses a display-only neutral RectLight (intensity1000, exposure0,
white,8x8m,z2.6m above the selected view) and annotation z1.35m for visibility.
World XY/yaw and historical input JPEG remain unchanged; this lighting is not
a new scientific observation. Physics is stopped; no new model/GP/MPC/rollout.

Actual screenshot and source/numeric sidecar:
`data/genuine_source_moving_mismatch_gui/view_20260922T081500Z/source_inventory_gui.png`.
The directory is a unique identifier; actual capture UTC is in its JSON sidecar.
The viewer rendered successfully and was visually checked. It remains open
until closed by the user. No external source or original USD was modified.

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CUDA_HOME -u CUDA_PATH \
  -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u RMW_IMPLEMENTATION \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1 \
  /home/gpuadmin/isaacsim/python.sh scripts/isaac/genuine_source_scan_gui.py \
  --run data/genuine_source_moving_mismatch/scan_20260922T073107Z \
  --output data/genuine_source_moving_mismatch_gui/NEW_UNIQUE_VIEW_ID
```

The saved-only loader authenticates all13 cases and104 display input files.
Focused GUI/replay tests:26 passed, including generic N, original observation
anchors, common byte hashes, saved-sample time selection and no solver/physics
calls. All synthetic fixtures are implementation tests only.

## Actual post-switch execution in the GUI (2026-09-22)

The preceding inventory view ended at B. That display choice must not be read
as evidence that FRESH was never executed. The original genuine corpus has
actual first-FRESH command applications and subsequent logical-SE(2) execution
for all 881 activated handoffs. SOURCE03/04's paused prediction diagnostics are
separate experiments; they did not execute their obstacle-on futures.

The opt-in `--include-post-switch` viewer mode now authenticates the recorded
commands and displays the complete lifetime of each of the 13 FRESH references.
Blue/pink remain OLD/FRESH predictions; orange is saved observation-to-B OLD
execution; cyan is actual saved FRESH execution after B. Green marks observation,
yellow B, white the current sampled state. The final cyan state precedes the
next reference's first command; that next command is excluded. The optional
prepared common reference is violet and was not installed by this online source
collector. World XY/yaw and observation anchors are unchanged.

The actual Isaac GUI rendered three illustrative cases (not a new selection
cohort or performance ranking). All 13 remain available in the case dropdown.

| Source | B→FRESH nearest distance | B pose-yaw error | Recorded FRESH duration / steps | End→same FRESH distance | End pose-yaw error |
|---|---:|---:|---:|---:|---:|
| episode_013_repeat_01/handoff_024 | .279113 m | 36.18598 deg | 1.550000 s / 93 | .182382 m | 8.35201 deg |
| episode_008_repeat_01/handoff_023 | .184683 m | 22.09812 deg | 1.500000 s / 90 | .115583 m | 2.64926 deg |
| episode_001_repeat_01/handoff_013 | .127644 m | 14.99966 deg | 1.250000 s / 75 | .077264 m | 6.32837 deg |

Distances above are nearest original-polyline distances, not necessarily normal
components or forward-only sustained-attachment errors. They show measured
reference/state mismatch and remaining tracking error. They do not alone prove
full execution failure, obstacle collision, or an optimization benefit. Actual
state stays continuous under the recorded held-command integration; a reference
change does not teleport the agent. For 013/01/024 the first applied FRESH command
changes v by -.200000 m/s and omega by +.500000 rad/s. This is a measured command
change, not by itself an acceleration-limit failure.

Evidence:
`data/genuine_source_moving_mismatch_gui/post_switch_20260922T085623Z/`.
`source_inventory_gui.png` shows 013/01/024; the other PNG names contain full
case IDs. Each JSON sidecar preserves actual state samples, command identities,
original metric reconstruction and source hashes. `recorded_execution_summary.csv`
covers all 13; `validation.json` checks 111 display input files, 1,011 historical
FRESH-applied integration steps, five original scalar metrics per case to 1e-12, and all
three screenshot sidecars. New model/GP/MPC/rollout calls: zero. The screenshots
were visually inspected. The 20cm/+5cm rings are reference sizes, not a fresh
collision validation of these post-switch paths.

Reproduce with the same environment-clearing Isaac command above, plus:

```bash
--include-post-switch --case episode_013_repeat_01/handoff_024 \
--capture-case episode_008_repeat_01/handoff_023 \
--capture-case episode_001_repeat_01/handoff_013
```

Use a new `--output` directory. The dropdown selects any of the 13 cases;
`Replay saved OLD + FRESH`, `Show B`, and `Show saved end` operate on saved
samples only (scroll the panel to see playback controls). Relevant source,
recorded replay and timing tests: 91 passed; compileall and diff whitespace check
passed. Historical source/controller/selector code was not changed.

For a future reconciliation comparison, the necessary reference is unchanged
native LightNav FRESH plus the pinned official MPC (no reconciliation). A rigid
correction is a separate geometric baseline, and the existing suffix/resampling
adapter is a separate preparation ablation. The optimized method must use the
same source event, B, command memory, environment and execution settings. LightNav
is the shared upstream generator, not a competing model to retrain here. Baseline
comparison should establish sustained attachment, motion/safety and compute
effects; visual path separation alone does not establish a failure or superiority.
No such new comparison was run as part of this GUI request.
