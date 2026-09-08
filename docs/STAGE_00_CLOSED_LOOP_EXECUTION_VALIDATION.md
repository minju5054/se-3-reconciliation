# Stage 0-E — Closed-Loop Trajectory Execution Validation

## 1. Motivation and research boundary

Stage 0-D ended `EXECUTION_LAYER_NOT_YET_VALIDATED`. Its frozen `pi_strong` diagnostic
candidate improved the unchanged Stage 0-B composite, but failed held-out arbitrary body-rate
and historical EXP-02B command-replay gates. Historical body-command replay is not the same
question as executing a trajectory: a reconciliation backend returns an arbitrary `N x 3`
`[x,y,yaw]` reference, while `TrajectoryFollower` recomputes desired `[v,omega]` online from
the actual measured pose.

Stage 0-E therefore asks whether nominal or the frozen Stage 0-D candidate is sufficient as a
trajectory-level execution platform for a later, separate reconciliation experiment. It does
not improve a reconciliation algorithm, tune a controller, change contact/physics, or provide
reconciliation research evidence. The primary run is
`data/stage0/closed_loop_execution_validation/stage0e-20260908T021433Z/`.

## 2. Same-reference closed-loop design

The only independent variable is the execution layer:

```text
nominal:
trajectory reference -> TrajectoryFollower desired [v,omega]
                     -> DifferentialController -> wheel targets -> Jackal

calibrated:
trajectory reference -> TrajectoryFollower desired [v,omega]
                     -> frozen Stage 0-D JackalExecutionController
                        executed [v,omega]
                     -> DifferentialController -> wheel targets -> Jackal
```

Both modes use the same reference, original initial pose, Stage 0-B follower settings,
`1/60 s` physics, `0.1 s` control, scene, ground, Jackal asset, 1 s settling, and gates. Before
every repetition, the world, world pose, all wheel velocities, wheel command, follower progress,
and low-level integral state are independently reset. The follower consumes measured world pose
at every control boundary. No historical `controller_commands.csv` is used.

The Stage 0-D candidate is loaded from its completed artifact and verified by SHA-256. It is
`pi_strong`, `kp=3`, `ki=2`, feedforward scale `7.3526300675`, calibrated controller-only
separation `2.7615743019 m`, with the frozen saturation, anti-windup, and sign-protection
settings. These values were not copied into or tuned by the Stage 0-E config; the candidate
remains previously unvalidated.

## 3. Reference suites and timestamps

Group A contains seven deterministic trajectory-level held-out engineering fixtures: straight,
gentle left/right, strong left/right, S-curve, and straight-turn-straight. They contain 36–56
poses and 0.84–1.525 m of path, and are not exact repetitions of the Stage 0-D body-command
calibration profiles. They are synthetic platform-validation fixtures, not research evidence.

Group B regenerates the unchanged Stage 0-B composite reference verbatim from its existing
motion profile. Group C directly loads the immutable EXP-02B `derived/old_world.npy` for
`case_high_delta_omega`, `case_high_delta_v`, and `case_benign_delayed`. It is explicitly a
**reference-matched closed-loop evaluation**. Each mode generates a different online desired
command history as needed from its actual pose; it does not replay the source command history.

LightNav OLD/FRESH waypoint rows have no intrinsic timestamps. Position and yaw RMSE therefore
project each actual pose to its nearest reference-polyline segment and interpolate yaw spatially
on that segment. They are not time-aligned waypoint tracking errors. Final error compares the
actual endpoint with the reference endpoint. Simulation/control timestamps describe execution
samples only.

## 4. Frozen gates

Every condition has three repetitions per mode. A condition passes only if all of the following
pre-result checks pass: at least 2/3 goals; maximum repeat position RMSE `<0.15 m`; maximum
final position error `<0.20 m`; yaw RMSE `<0.10 rad`; final yaw error `<0.10 rad`; finite and
stable values; saturation fraction `<=0.20`; monotonic follower progress; and either goal reach
or at least 0.90 progress without pathological stuck behavior.

The controlled aggregate additionally requires 6/7 passing conditions, at least one passing
left curve, one right curve, and one direction-change path. EXP-02B requires 2/3 frozen cases.
A mode is platform-ready only when controlled, Stage 0-B composite, and EXP-02B groups all pass.
If both modes pass, nominal is selected for simplicity. Thresholds and path definitions were
snapshotted before the simulator run and were not changed afterward.

## 5. Controlled held-out results

All values below are the mean of three identical/deterministic repetitions; `G` is goal
successes out of three. `P/Y` are nearest-spatial-reference position/yaw RMSE and `FP/FY` are
endpoint errors.

| Path | Nominal G | Nominal P / Y | Nominal FP / FY | Pass | Calibrated G | Calibrated P / Y | Calibrated FP / FY | Pass |
|---|---:|---:|---:|---|---:|---:|---:|---|
| straight | 3 | 0.00105 m / 0.00077 rad | 0.07069 m / 0.00059 rad | PASS | 3 | 0.00054 / 0.00097 | 0.07073 / 0.00139 | PASS |
| gentle left | 0 | 0.11669 / 0.76470 | 0.28101 / 2.35273 | FAIL | 3 | 0.00348 / 0.03451 | 0.07491 / 0.06114 | PASS |
| gentle right | 0 | 0.10217 / 1.47558 | 0.15449 / 0.99214 | FAIL | 3 | 0.00463 / 0.03754 | 0.07134 / 0.05784 | PASS |
| strong left | 0 | 0.09423 / 1.44191 | 0.14467 / 1.10767 | FAIL | 3 | 0.02043 / 0.17309 | 0.05364 / 0.06456 | **FAIL yaw RMS** |
| strong right | 3 | 0.04411 / 0.23591 | 0.01421 / 0.07685 | FAIL | 3 | 0.01982 / 0.17028 | 0.03856 / 0.05800 | **FAIL yaw RMS** |
| S-curve | 0 | 0.09653 / 1.41391 | 0.17106 / 1.61091 | FAIL | 3 | 0.00983 / 0.08005 | 0.05465 / 0.00984 | PASS |
| straight-turn-straight | 0 | 0.09002 / 1.43166 | 0.16301 / 1.15660 | FAIL | 3 | 0.00958 / 0.07548 | 0.07467 / 0.06708 | PASS |

Nominal passes 1/7. Calibrated passes 5/7, including both gentle directions and both
direction-change paths, but misses the required 6/7 because both strong-turn yaw RMS values
exceed 0.10 rad. Thus neither controlled aggregate passes. Across controlled trials,
calibrated versus nominal mean position RMSE is `0.00976 vs 0.07783 m` and yaw RMSE is
`0.08170 vs 0.96635 rad`. Desired-v versus measured-v RMSE worsens slightly
(`0.08555 vs 0.07425 m/s`), while desired-omega RMSE improves
(`0.13113 vs 0.67593 rad/s`) and wheel RMSE improves (`0.48686 vs 0.97303 rad/s`). Calibrated
mean saturation is `0.00446`; strong-right is `0.03125`, below the gate.

## 6. Stage 0-B composite

| Mode | goals | position RMS | final position | yaw RMS | final yaw | desired v / omega RMSE | wheel RMSE | Result |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| nominal | 0/3 | 0.04926 m | 0.05334 m | 0.22934 rad | 0.14085 rad | 0.07726 m/s / 0.63217 rad/s | 0.80956 rad/s | FAIL |
| calibrated | 3/3 | 0.00250 m | 0.07863 m | 0.01900 rad | 0.00398 rad | 0.12016 m/s / 0.03138 rad/s | 0.36245 rad/s | PASS |

The calibrated composite passes all absolute trajectory gates, while nominal fails goal and yaw
gates. Calibrated final position is worse than nominal by `0.02528 m` but remains within the
frozen absolute criterion. No Stage 0-D non-degradation rule is imported into this distinct
trajectory-level gate.

## 7. EXP-02B OLD-reference closed loop

| Case | Mode | goals | position RMS | final position | yaw RMS | final yaw | desired v / omega RMSE | wheel RMSE | sat. | Result |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| high delta omega | nominal | 3/3 | 0.06697 m | 0.05559 m | 0.38108 rad | 0.07913 rad | 0.10313 / 0.74357 | 1.11290 rad/s | 0 | FAIL |
| high delta omega | calibrated | 3/3 | 0.03997 | 0.07262 | 0.07980 | 0.05510 | 0.11402 / 0.20583 | 0.57746 | 0.0667 | PASS |
| high delta v | nominal | 3/3 | 0.06697 | 0.05559 | 0.38108 | 0.07913 | 0.10313 / 0.74357 | 1.11290 | 0 | FAIL |
| high delta v | calibrated | 3/3 | 0.03997 | 0.07262 | 0.07980 | 0.05510 | 0.11402 / 0.20583 | 0.57746 | 0.0667 | PASS |
| benign delayed | nominal | 3/3 | 0.03951 | 0.01033 | 0.25546 | 0.07518 | 0.05233 / 0.51098 | 0.96252 | 0 | FAIL |
| benign delayed | calibrated | 3/3 | 0.02981 | 0.05671 | 0.05294 | 0.01446 | 0.06729 / 0.15262 | 0.54338 | 0 | PASS |

Calibrated passes 3/3 frozen source cases; nominal passes 0/3 because yaw RMS is above the
gate despite reaching each goal. The high-delta cases record two sign-protection events per
trial but stay below the 20% saturation gate. Repository provenance shows high-delta-omega and
high-delta-v have the same frozen OLD array/hash, so their Stage 0-E OLD-reference metrics are
necessarily identical; they remain distinct source transition cases but provide only one unique
OLD geometry here.

Desired and executed body commands remain distinct. For high-delta-omega calibrated,
desired-omega versus measured RMSE is `0.20583 rad/s`, while executed-omega versus measured is
`1.60261 rad/s`. The frozen candidate intentionally sends a large feedforward/PI-compensated
low-level omega to obtain motion closer to follower desired omega; the latter metric records the
plant mapping and must not be relabelled as desired tracking quality.

## 8. Historical replay versus new closed loop

Stage 0-D historical replay asks: **same saved body commands, different execution layer**. Its
representative calibrated result was OLD spatial RMS `0.09296 m`, desired-omega RMSE
`0.62880 rad/s`, wheel RMSE `1.05665 rad/s`, and a reproduced-boundary difference of
`0.13212 m / 1.60263 rad` because changed low-level execution changed the replay.

Stage 0-E asks: **same OLD trajectory reference, each mode recomputes commands from its own
actual pose**. The representative calibrated result instead reaches the reference goal with
position/yaw RMS `0.03997 m / 0.07980 rad`, desired-omega RMSE `0.20583 rad/s`, and wheel RMSE
`0.57746 rad/s`. Feedback can correct deviations and the run continues to the path goal, so
the protocols answer different questions and their raw values are not interchangeable. This
new result explains why poor arbitrary-rate/historical replay does not automatically imply poor
trajectory tracking; it does not retroactively change or invalidate the frozen EXP-02B result.

## 9. Telemetry, artifacts, and plots

Each trial stores immutable `raw/actual_trajectory.npy`, `raw/telemetry.csv`,
`derived/metrics.json`, `derived/raw_provenance.json`, and `metadata.json`. References live once
per condition with SHA-256. Telemetry includes simulation/control indices; phase; follower
nearest/target/goal; desired, executed, and finite-difference measured body motion; four target
and measured wheel velocities; actual world pose; nearest spatial errors; PI correction;
integral; saturation; and sign protection. Metadata records original input paths/hashes,
coordinate convention, follower settings, initial/reset semantics, and observation/execution
timing.

The summary directory contains `controlled_paths_summary.json`, `composite_summary.json`,
`exp02b_reference_summary.json`, `mode_comparison.json`, and
`final_execution_platform_decision.json`. Fifteen plots cover every controlled XY overlay, yaw
error distributions, position/yaw RMSE, goal success, representative desired/executed/measured
omega, composite, high-delta-omega, and all OLD references. Generated data is ignored by Git;
existing run/trial/GUI/plot paths are rejected rather than overwritten.

## 10. GUI validation and legend

```bash
./scripts/isaac/run_jackal_closed_loop_execution_validation_gui.sh \
  data/stage0/closed_loop_execution_validation/stage0e-20260908T021433Z \
  --suite controlled --scenario s_curve --real-time-factor 1.0 --no-hold

./scripts/isaac/run_jackal_closed_loop_execution_validation_gui.sh \
  data/stage0/closed_loop_execution_validation/stage0e-20260908T021433Z \
  --suite composite --real-time-factor 1.0 --no-hold

./scripts/isaac/run_jackal_closed_loop_execution_validation_gui.sh \
  data/stage0/closed_loop_execution_validation/stage0e-20260908T021433Z \
  --suite exp02b --case case_high_delta_omega --real-time-factor 1.0 --no-hold
```

`BLUE` is the trajectory reference/planned OLD, `ORANGE/RED` nominal actual, `GREEN`
calibrated actual, and EXP-02B-only `GREY` is raw FRESH context excluded from metrics. The
Jackal executes nominal, resets completely, then executes calibrated while the nominal ghost
remains. A final overlay is held by default unless `--no-hold` is used. Terminal live output
reports suite/scenario/mode/phase, pose, follower indices, all three body-command levels, spatial
errors, four-wheel target/measured values, feedforward scale, PI correction, integral,
saturation, and sign events. Slowdown changes presentation wall time only.

Actual non-headless runs completed S-curve, composite, and representative EXP-02B suites.
Jackal motion, reset, all path colors, nominal ghost persistence, grey context exclusion, and
live telemetry were observed. The three final viewport captures are retained under the primary
run `gui_metadata/` and were visually inspected.

## 11. Repeatability, decision, and claim limits

Position/yaw/final-error standard deviations were zero or floating-point epsilon across every
three-repeat condition; success rates were consistently 0/3 or 3/3. This demonstrates reset
repeatability for this deterministic simulator configuration, not robustness to stochastic
physics or real hardware.

Final decision: **`EXECUTION_PLATFORM_NOT_READY`**. Nominal fails all three group checks.
Calibrated passes the composite and EXP-02B OLD-reference groups but fails controlled held-out
paths at 5/7, below the frozen 6/7 criterion. Per the termination rule, Stage 0-E does not tune
gains or invent another controller, and neither mode is frozen for future reconciliation work.

What can be claimed: under the same-reference protocol, the tested calibrated candidate has
materially better simulation trajectory tracking than nominal; it is repeatable; and its
remaining falsification is localized to strong-turn yaw RMS under the frozen held-out set. Its
behavior also differs from historical command replay, but cross-protocol metric changes are not
an improvement claim. What cannot be claimed: real-robot behavior,
general skid-steer control, a unique tire/contact cause, LightNav quality, obstacle avoidance,
or any reconciliation-method improvement. The next uncertainty is whether the strong-turn
failure is acceptable under a separately justified application envelope or requires a future,
separately scoped execution-controller investigation; this task makes neither decision.
