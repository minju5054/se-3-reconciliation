# Stage 0-F — LightNav Trajectory Execution-Envelope Analysis

## 1. Why Stage 0-F exists

Stage 0-E ended `EXECUTION_PLATFORM_NOT_READY`. The frozen Stage 0-D calibrated candidate
passed straight, gentle left/right, S-curve, and straight-turn-straight controlled paths, but
failed the strong-left and strong-right paths on yaw RMSE (`0.17309` and `0.17028 rad` versus
the frozen `<0.10 rad` gate). It nevertheless passed the unchanged Stage 0-B composite and all
three frozen EXP-02B OLD-reference closed-loop cases.

That left a narrower, application-specific uncertainty: do frozen real LightNav OLD/FRESH
references demand geometry that the current calibrated Jackal cannot execute? Stage 0-F
characterizes that workload and executes it. It is not a reconciliation experiment, does not
change the Stage 0-E result, and does not tune a controller, follower, physics, contact, asset,
or LightNav.

The immutable primary evidence is:

```text
data/stage0/lightnav_execution_envelope/stage0f-20260908T043300Z/
```

## 2. Research question and scope boundary

The sole question is:

> What is the coverage relationship between the frozen real LightNav trajectory distribution
> and the frozen calibrated Jackal trajectory-execution capability?

The pipeline performs geometry characterization, descriptive comparison to the seven Stage 0-E
fixtures, same-reference closed-loop execution, a geometry-frozen nominal comparison, and GUI
inspection. It implements no new controller, controller tuning, reconciliation residual,
objective, graph weight, correspondence, selector, `k_old`/`k_fresh` policy, obstacle method,
or LightNav inference.

## 3. Frozen source cohort and provenance

The source is the existing redesigned EXP-01B cohort:

```text
data/exp01b_redesign/exp01b-controlled-primary-20260906T-frozen/
```

No new LightNav call was made. The strict EXP-01B validator was reused, every attempt's required
OLD/FRESH raw/world arrays, metadata, attempt result, and controller CSV were present, and each
world path reconstructed exactly from its raw action and recorded observation pose. Frozen root
hashes include metadata `294294a9...d5d35`, protocol `476e39fa...a1ecd`, validation
`f7e19085...695fc`, and source config `9fe8a0c8...89682`. The cohort records research commit
`4a446000...9a91b`, LightNav checkout `a645828d...9777`, and checkpoint revision
`7221d418...6aaab`.

Every unique reference has its own `source_provenance.json`. It records all sharing source
trials, raw-action/world/metadata/attempt/controller-CSV hashes, OLD/FRESH role, start pose,
geometry/latency metadata, source commit/config, Stage 0-D model hash, and Stage 0-E follower
config provenance. Derived reference copies are hash-verified; source files are never changed.

## 4. Inventory, duplicates, STOP, and failed attempts

| Item | Count |
|---|---:|
| All retained attempts | 37 |
| Timing-valid attempts | 36 |
| Eligible `VALID_MOVING` transitions | 30 |
| `MODEL_STOP_OUTPUT` / `STOP_OUTPUT` | 3 |
| `OLD_EXHAUSTED` | 3 |
| `TIMING_INVALID` | 1 |
| All-zero FRESH arrays across all attempts | 6 |
| All-zero FRESH arrays that are non-STOP failed attempts | 3 |
| Unique OLD raw-action hashes | 8 |
| Unique OLD world-reference hashes | 9 |
| Unique FRESH raw-action hashes | 8 |
| Unique FRESH world-reference hashes | 16 |

Only the 30 eligible moving transitions contribute geometry. The three model STOP outputs are
retained as `STOP_OUTPUT`, never treated as zero-curvature paths. The three all-zero
`OLD_EXHAUSTED` failed attempts remain separately visible and are not mislabeled as model STOP.
Trial counts and unique references are kept distinct. A raw local action can yield more than one
world hash because its observation anchor differs; execution therefore uses 9 unique OLD plus
16 unique FRESH world references, not 60 trial-level copies.

## 5. Untimed spatial descriptor convention

LightNav produces arbitrary `N x 3 [x,y,yaw]` waypoints with no intrinsic row timestamps.
Stage 0-F never assigns `F_j = t_obs + j*dt`, never divides spacing by a fabricated `dt`, and
does not call any descriptor velocity or acceleration. Execution timestamps belong to Isaac
telemetry only.

For each path, the stored spatial descriptors include pose count; path length; endpoint
displacement; wrapped net, cumulative-absolute, and maximum-increment pose yaw; initial-heading
lateral departure; cumulative signed/absolute tangent turn; curvature-sign changes; and the
fraction of near-zero segments. Two curvature proxies remain explicitly separate:

```text
pose-yaw proxy:       wrap(yaw[i+1] - yaw[i]) / segment_length[i]
tangent proxy:        wrap(alpha[i+1] - alpha[i]) /
                      mean(segment_length[i], segment_length[i+1])
alpha[i] = atan2(delta_y[i], delta_x[i])
```

Median, p90, p95, and maximum absolute finite values are recorded. Sub-millimetre XY edges
(`segment_length < 0.001 m`) are undefined for these ratios so numerical noise is not converted
into an artificial enormous curvature; their pose-yaw increments and near-zero fraction remain
reported. Turning radius is `1/max(|tangent curvature|)` only when a finite non-negligible
curvature exists, otherwise JSON `null`. Pose yaw and XY tangent are never substituted for one
another.

## 6. Stage 0-E fixture reference envelope

The existing seven fixture definitions and their existing calibrated PASS/FAIL results were
loaded from the hash-verified Stage 0-E run. Nothing was reclassified.

| Fixture | Result | Length m | cumulative pose yaw rad | max pose step rad | p95 tangent curvature 1/m | cumulative tangent turn rad | position / yaw RMSE |
|---|---|---:|---:|---:|---:|---:|---:|
| straight | PASS | 1.400 | 0.000 | 0.000 | 0.000 | 0.000 | 0.00054 / 0.00097 |
| gentle left | PASS | 1.440 | 0.810 | 0.018 | 0.563 | 0.792 | 0.00348 / 0.03451 |
| gentle right | PASS | 1.440 | 0.810 | 0.018 | 0.563 | 0.792 | 0.00463 / 0.03754 |
| strong left | **FAIL** | 0.840 | 1.470 | 0.042 | 1.750 | 1.428 | 0.02043 / 0.17309 |
| strong right | **FAIL** | 0.840 | 1.470 | 0.042 | 1.750 | 1.428 | 0.01982 / 0.17028 |
| S-curve | PASS | 1.500 | 1.400 | 0.028 | 0.933 | 1.372 | 0.00983 / 0.08005 |
| straight-turn-straight | PASS | 1.525 | 0.875 | 0.035 | 1.400 | 0.875 | 0.00958 / 0.07548 |

These seven fixtures are synthetic engineering validation inputs, not experimental evidence.

## 7. Fixture-only descriptive comparison

Ten descriptor dimensions and their min/max scales are frozen in
`configs/stage0_lightnav_execution_envelope.yaml`. The normalizer uses only the seven Stage 0-E
fixtures. The outside radius (`1.463306`) is the maximum fixture leave-one-out nearest-neighbor
distance; the ambiguity margin (`0.146331`) is ten percent of that radius. LightNav values never
fit these numbers. Severity is the pre-result lexicographic order p95 tangent curvature,
cumulative absolute tangent turn, then maximum pose-yaw increment; there is no tuned weighted
score and no ML classifier.

All 25 LightNav world references received `OUTSIDE_STAGE0E_TESTED_GEOMETRY` (OLD 9, FRESH 16;
PASS-like 0, strong-turn-like 0, ambiguous 0). This is an important negative result: the seven
fixture points do not define a sufficiently dense descriptor neighborhood for a geometric
coverage claim. In particular, LightNav chunks are often shorter, and some contain discrete
local XY direction changes much sharper than a smooth fixture arc. Therefore the descriptive
label is not treated as an execution prediction; actual closed-loop execution below is the
deciding observation.

## 8. OLD geometry distribution

Each statistic weights one unique OLD world reference. Full G0/G1/G2 and L0/L1 descriptive
breakdowns are stored in `old_geometry_summary.json`; latency grouping is not a causal claim.

| Descriptor | min | median | p95 | max |
|---|---:|---:|---:|---:|
| path length m | 0.5109 | 1.3539 | 1.3703 | 1.3703 |
| cumulative absolute pose yaw rad | 0.0049 | 0.1856 | 0.8219 | 1.1468 |
| max absolute pose-yaw increment rad | 0.0008 | 0.0770 | 0.2084 | 0.2699 |
| p95 tangent-curvature proxy 1/m | 0.0037 | 0.6591 | 20.2346 | 28.1690 |
| cumulative absolute tangent turn rad | 0.0009 | 0.1507 | 2.2907 | 3.6379 |
| near-zero segment fraction | 0.0000 | 0.0000 | 0.4667 | 0.5556 |

## 9. FRESH full-reference geometry distribution

FRESH remains a separate observation-anchored source; it is never pooled with OLD.

| Descriptor | min | median | p95 | max |
|---|---:|---:|---:|---:|
| path length m | 0.5109 | 1.3541 | 1.3703 | 1.3703 |
| cumulative absolute pose yaw rad | 0.0049 | 0.1944 | 0.5376 | 1.1468 |
| max absolute pose-yaw increment rad | 0.0008 | 0.0815 | 0.1546 | 0.2699 |
| p95 tangent-curvature proxy 1/m | 0.0037 | 0.6591 | 13.2920 | 28.1690 |
| cumulative absolute tangent turn rad | 0.0009 | 0.1599 | 1.1118 | 3.6379 |
| near-zero segment fraction | 0.0000 | 0.0000 | 0.5556 | 0.5556 |

## 10. FRESH suffix diagnostic

Every `FRESH[k:]` with at least two poses was described, producing 144 suffix rows. This is
shape-only diagnostic data: no `k` is selected, no switch boundary is connected, and no suffix
is executed as a reconciliation claim.

| Descriptor | finite count | min | median | p95 | max |
|---|---:|---:|---:|---:|---:|
| path length m | 144 | 0.00056 | 0.45215 | 1.35421 | 1.37028 |
| cumulative absolute pose yaw rad | 144 | 0.00076 | 0.16058 | 0.38368 | 1.14678 |
| max absolute pose-yaw increment rad | 144 | 0.00076 | 0.07695 | 0.24781 | 0.26995 |
| p95 tangent-curvature proxy 1/m | 109 | 0.00036 | 0.68127 | 31.5209 | 42.8336 |
| cumulative absolute tangent turn rad | 144 | 0.00000 | 0.11108 | 2.49754 | 3.63789 |
| near-zero segment fraction | 144 | 0.00000 | 0.00000 | 1.00000 | 1.00000 |

Undefined short-segment ratios remain `null`; the finite-only curvature distribution does not
turn a sub-millimetre two-pose suffix into a huge number. Full FRESH passing does not prove that
an actual future `FRESH[k:]` splice will pass.

## 11. Actual calibrated execution protocol and gates

All 25 unique moving world references were executed three times using the official Jackal,
unchanged flat ground/physics, no added obstacles, the Stage 0-B `TrajectoryFollower`, the
frozen Stage 0-D `pi_strong` candidate, and `DifferentialController`. No historical controller
CSV command was replayed. The follower recomputed desired `[v,omega]` from measured pose at each
control step.

OLD starts from `robot_pose_at_old_observation`; FRESH starts from
`robot_pose_at_fresh_observation`, not ready or switch pose. No canonical transform is applied
to execution. Each repetition resets world pose, wheel state, follower progress, and low-level
integral state and then settles for one simulation second.

The unchanged Stage 0-E condition gate requires at least 2/3 goals, position RMSE `<0.15 m`,
final position `<0.20 m`, yaw RMSE `<0.10 rad`, final yaw `<0.10 rad`, finite/stable telemetry,
saturation `<=0.20`, monotonic progress, and goal or at least 0.90 non-pathological progress.

## 12. Actual calibrated execution coverage

All 75 calibrated trials completed and all unique conditions passed:

| Source | Unique references | PASS | FAIL | Goal repetitions |
|---|---:|---:|---:|---:|
| OLD | 9 | 9 | 0 | 27/27 |
| FRESH at observation pose | 16 | 16 | 0 | 48/48 |

G0/G1/G2 coverage was respectively OLD `3/3, 4/4, 2/2` and FRESH `6/6, 7/7, 3/3` unique
reference memberships. The largest yaw RMSEs were:

| Reference | Kind | position RMSE m | yaw RMSE rad | Result |
|---|---|---:|---:|---|
| `old_aac92c8f6993b6f9` | OLD | 0.03917 | 0.08606 | PASS |
| `fresh_b42af924228e8aaf` | FRESH | 0.04022 | 0.08121 | PASS |
| `fresh_0e5e355b652fa39f` | FRESH | 0.02916 | 0.06108 | PASS |
| `fresh_f70f3a0281dccc33` | FRESH | 0.02972 | 0.05976 | PASS |
| `fresh_6c28915be4db6f26` | FRESH | 0.02997 | 0.05884 | PASS |

The largest position RMSE was `0.04391 m` (`old_c3f59e110332e69f`). The hardest severity
reference `fresh_b42af924228e8aaf` reached its goal with final position/yaw error
`0.07296 m / 0.04183 rad`, desired-omega versus measured-omega RMSE `0.22423 rad/s`, aggregate
wheel RMSE `0.53145 rad/s`, saturation `0.06667`, and two sign-protection events. These are
observed correlations within one deterministic simulator; they do not identify a unique
contact or plant cause.

## 13. Command-level metric semantics

Every trial stores nearest-reference spatial trajectory errors, follower demand, low-level
execution, body motion, wheel motion, and calibration state. The four levels are:

```text
LightNav reference [x,y,yaw]
  -> TrajectoryFollower desired [v,omega]
  -> calibrated controller executed [v,omega]
  -> DifferentialController four wheel targets
  -> measured four wheel velocities and measured body motion/pose
```

Desired-versus-measured body RMSE answers follower demand tracking. Executed-versus-measured
RMSE characterizes the compensated low-level plant mapping. They are not interchangeable:
`executed omega` is intentionally much larger than desired omega under the frozen effective-
width plus PI correction. Wheel target-versus-measured RMSE is reported both aggregate and per
corner. Simulation trajectory error is nearest spatial reference error, not time-aligned
waypoint error.

## 14. Geometry-frozen nominal comparison

Before observing execution results, the pipeline froze lowest/median/highest severity OLD and
FRESH, nearest strong-turn-failure fixture, and highest sign-change representatives. Duplicate
roles were merged, leaving eight references. Each then ran nominal and calibrated three times.

| Reference / role | nominal pass | calibrated pass | nominal -> calibrated position RMSE m | nominal -> calibrated yaw RMSE rad |
|---|---|---|---:|---:|
| `old_0b944...` / OLD low | PASS | PASS | 0.04290 -> 0.04291 | 0.00084 -> 0.00109 |
| `old_b9c72...` / OLD median | FAIL | PASS | 0.09803 -> 0.02987 | 1.48171 -> 0.05488 |
| `old_aac92...` / OLD high | FAIL | PASS | 0.11573 -> 0.03917 | 0.78829 -> 0.08606 |
| `fresh_2276...` / FRESH low | PASS | PASS | 0.04285 -> 0.04285 | 0.00057 -> 0.00028 |
| `fresh_a19d...` / FRESH median | FAIL | PASS | 0.11464 -> 0.02980 | 1.54814 -> 0.05813 |
| `fresh_b42a...` / FRESH high | FAIL | PASS | 0.11450 -> 0.04022 | 0.88711 -> 0.08121 |
| `fresh_6c28...` / nearest strong fixture | FAIL | PASS | 0.04099 -> 0.02997 | 0.24057 -> 0.05884 |
| `fresh_0120...` / highest sign change | PASS | PASS | 0.04213 -> 0.04213 | 0.00131 -> 0.00122 |

Nominal passed 3/8 and calibrated passed 8/8. This supports use of the frozen calibration on
this workload; it is not a newly tuned controller comparison and the few near-zero-demand
cases need not improve every scalar metric.

## 15. Relationship to the Stage 0-E strong-turn failures

The two strong-turn fixture failures remain valid and are not overridden. Geometry alone does
not place the LightNav set cleanly inside the five PASS-fixture neighborhoods: all LightNav
references are outside the sparse seven-point fixture radius, and the hardest LightNav p95
tangent proxy (`28.169 1/m`) exceeds the smooth strong fixture (`1.750 1/m`) because the former
contains a sharp discrete local direction change.

Actual execution provides the narrower result: even the hardest frozen LightNav reference
passed at `0.08121 rad` yaw RMSE, while the smooth strong fixture remained above `0.17 rad`.
Thus one scalar spatial-curvature statistic is not a sufficient execution-demand equivalence;
path length, cumulative shape, pose-yaw targets, follower lookahead, and resulting command
history all differ. Stage 0-F supports workload-specific coverage but does not prove that the
synthetic failures are irrelevant to future LightNav outputs.

## 16. GUI validation and legend

Run a prepared and summarized result with:

```bash
./scripts/isaac/run_jackal_lightnav_execution_envelope_gui.sh \
  data/stage0/lightnav_execution_envelope/stage0f-20260908T043300Z \
  --selection low --real-time-factor 1.0 --no-hold

./scripts/isaac/run_jackal_lightnav_execution_envelope_gui.sh \
  data/stage0/lightnav_execution_envelope/stage0f-20260908T043300Z \
  --selection median --real-time-factor 1.0 --no-hold

./scripts/isaac/run_jackal_lightnav_execution_envelope_gui.sh \
  data/stage0/lightnav_execution_envelope/stage0f-20260908T043300Z \
  --selection high --real-time-factor 1.0 --no-hold

./scripts/isaac/run_jackal_lightnav_execution_envelope_gui.sh \
  data/stage0/lightnav_execution_envelope/stage0f-20260908T043300Z \
  --selection strong_like --real-time-factor 1.0 --no-hold
```

`BLUE` is the LightNav reference and heading ticks, `GREEN` is calibrated actual,
`ORANGE/RED` is nominal actual when that reference belongs to the frozen nominal subset, and
`MAGENTA` marks a FRESH observation/start pose. Nominal executes first, an explicit
`RESET_BEFORE_CALIBRATED` hold occurs, then calibrated executes while the nominal ghost remains.
Presentation slowdown changes wall time only.

Live terminal JSON reports source trials, OLD/FRESH, unique hash, severity rank, mode, phase,
actual pose, nearest/target index, goal state, desired/executed/measured body command, spatial
position/yaw error, four wheel targets/measurements, PI correction, integral, saturation, and
sign protection. The geometry header reports length, cumulative yaw, max/p95 tangent proxy,
nearest Stage 0-E fixture, and its existing PASS/FAIL status.

Actual non-headless GUI runs completed for low (`fresh_2276...`, rank 1), global median
(`old_5ebc...`, rank 13), high (`fresh_b42a...`, rank 25), and nearest-strong
(`fresh_6c28...`, rank 7). Jackal motion, live telemetry, reference/actual layers, FRESH marker,
reset phase, and final captures were observed. High severity visibly preserved the failing
nominal loop and passing calibrated path. No calibrated execution failed, so `first_fail` has
no valid target and was not fabricated.

## 17. Artifacts, telemetry, and plots

The run is immutable and ignored by Git. Preparation stores the config snapshot, root
provenance, source inventory, seven-fixture table, full OLD/FRESH descriptors, 144 suffix rows,
fixture comparison, and pre-execution representative selection. Each execution repetition
stores:

```text
raw/actual_trajectory.npy
raw/telemetry.csv
derived/metrics.json
derived/raw_provenance.json
metadata.json
```

Telemetry contains simulation/control indices, phase, follower indices/goal, desired and
executed body commands, finite-difference measured body motion, four wheel targets and measured
velocities, actual world pose, spatial errors, PI/integral/saturation/sign state. Summaries are
`old_execution_summary.json`, `fresh_execution_summary.json`,
`representative_mode_comparison.json`, and
`final_lightnav_execution_envelope_decision.json`.

Ten unit-labelled plots cover fixture/LightNav curvature, cumulative pose yaw, max pose-yaw
step, length versus curvature, fixture distance, severity versus yaw RMSE, desired omega versus
yaw RMSE, G0/G1/G2 pass/fail, low/median/high XY, and hardest-versus-strong descriptors. Paths
from different source frames are never directly overlaid; the six representative plot panels
use separate first-pose canonicalized display frames and say so explicitly.

## 18. Failure cases and final label

There were no calibrated LightNav failures to analyze or visualize. This is not converted into
a statement that failure is impossible. Nominal failures are retained as baseline evidence and
are not used to tune parameters.

The final Stage 0-F label is:

```text
OLD_EXECUTION_ENVELOPE_COVERED
FRESH_EXECUTION_ENVELOPE_COVERED_AT_OBSERVATION_STATE
LIGHTNAV_ENVELOPE_COVERED_BY_CURRENT_CALIBRATED_PLATFORM
```

The decision JSON deliberately keeps `execution_platform_validated: false`,
`stage0e_strong_turn_failure_overridden: false`, `fresh_suffix_execution_evaluated: false`,
and `reconciliation_solved: false`.

## 19. Claims, limitations, and application-specific freeze implication

What can be claimed: under the exact deterministic Isaac configuration, frozen follower and
calibration, recorded observation starts, and frozen redesigned EXP-01B cohort, all observed
unique moving OLD and full FRESH references pass the unchanged Stage 0-E absolute condition
gates. Three deterministic repetitions and strict hashes make this result reproducible.

What cannot be claimed: general `EXECUTION_PLATFORM_VALIDATED`, real-robot capability,
stochastic robustness, obstacle/collision feasibility, general future LightNav coverage,
causal latency effects, optimal control, a unique skid/contact mechanism, suffix-splice
feasibility, or any reconciliation improvement. The source is one outcome-retained frozen
cohort with three scenario classes; repeated deterministic runs are not independent environment
samples.

The evidence is sufficient to justify an explicitly bounded **application-specific observed
execution envelope** for this frozen LightNav workload if the project records that narrower
freeze decision. It is not sufficient to reverse Stage 0-E's global `EXECUTION_PLATFORM_NOT_READY`
decision. New LightNav distributions or a broader application claim must be revalidated.

## 20. Next uncertainty

The next research uncertainty is no longer whether these saved full OLD/FRESH shapes can be
tracked from their own observation starts. It is whether the project will formally accept the
bounded application-specific platform despite the preserved strong-fixture falsification, and
then whether OLD execution state, latency, candidate `FRESH[k:]` entry, and switch-boundary
reconciliation can be studied without conflating them. Stage 0-F does not answer that splice or
optimization question and makes no selector choice.
