# Stage 0-D — Jackal Execution-Layer Calibration and Validation

## 1. Why this stage exists

EXP-02B GUI diagnosis separated LightNav waypoints, `TrajectoryFollower` body commands,
`DifferentialController` wheel targets, and measured Jackal motion. In the representative
`case_high_delta_omega / k=3 / raw_k` OLD replay, the nearest-OLD-polyline RMS was
`0.06460 m`, body command-versus-measured RMSE was `0.12133 m/s` and `0.63989 rad/s`, and
wheel target-versus-measured RMSE was `0.84874 rad/s`. The replay reproduced the frozen
boundary exactly, so replay/reset mixing was not the principal explanation for that OLD gap.

Stage 0-D asks whether this execution mismatch can be characterized and corrected before any
further reconciliation work. It is an engineering gate for the execution platform, not an
optimization experiment and not evidence for a reconciliation method.

## 2. Scope boundary

The measured chain is:

```text
desired body [v, omega]
  -> optional execution correction [v_exec, omega_exec]
  -> official Isaac DifferentialController
  -> four wheel targets
  -> measured four wheel velocities
  -> measured body [v, omega] and world [x, y, yaw]
```

The official Isaac Sim 6.0.1 Clearpath Jackal asset, its physical wheel radius and separation,
and all contact/drive physics remain unchanged. The runtime-discovered physical values were
`r=0.0979999974 m` and `b=0.375589997 m`; `runtime_physics_overrides` is empty. The optional
calibrated effective separation is controller metadata only and never overwrites physical
geometry or USD state.

No LightNav output, FRESH trajectory, EXP-02A/02B factor, weight, graph result, candidate,
selector, k interface, or historical result was changed. Even a passing execution controller
would only become a fixed platform for later reconciliation research.

## 3. Four-phase protocol

The primary run is
`data/stage0/execution_calibration/stage0d-20260907T102700Z/`. Its configuration and acceptance
criteria were snapshotted before measurement; artifacts are immutable and ignored by Git.

### Phase 1 — nominal characterization

Primitive body commands bypass `TrajectoryFollower` and enter the official
`DifferentialController` directly. The frozen grid is the Cartesian product of
`v={0.00,0.15,0.30} m/s` and `omega={-0.60,-0.30,-0.15,0,+0.15,+0.30,+0.60} rad/s`.
The stationary condition is run once and every other condition three times: 61 actual Isaac
trials. Each trial uses 1.0 s settling, 0.5 s initial stop, 4.0 s active command, and 0.5 s
final stop, sampled at physics `dt=1/60 s` with control `dt=0.1 s`.

### Phase 2 — minimal correction selection

Option A fits a through-origin model
`omega_meas = c * omega_des` to condition means and converts it to the mathematically equivalent
yaw feedforward scale `1/c`, while retaining physical and effective separations separately.
Leave-one-condition-out prediction, worst condition error, sign correctness, scale limit, and
observed condition-gain range are evaluated before accepting the model.

Option A failed, so and only so were three predeclared PI candidates evaluated in order on the
same calibration grid. The minimal controller is

```text
omega_exec = clip((b_eff / b_physical) * omega_des
                  + kp * (omega_des - omega_meas)
                  + ki * integral_error,
                  -8, +8 rad/s)
```

It uses 0.1 s updates, deterministic state reset between trials, reset on zero or desired-sign
reversal, conditional-integration anti-windup, finite-value rejection, desired-sign protection,
and a 30 rad/s wheel-target guard. It does not modify `v_des`.

No candidate passed the calibration criteria. `pi_strong` was retained only as the fixed,
lowest-calibration-angular-RMSE diagnostic candidate; it is **not** a validated controller.
More complex/adaptive/MPC approaches were deliberately not introduced.

### Phase 3 — held-out and composite validation

The held-out grid `v={0.10,0.22,0.35}` and
`omega={-0.45,-0.20,+0.20,+0.45}` is disjoint from fitting. Each of its 12 conditions is run
three times in nominal and calibrated modes from the same initial pose. The held-out results
do not participate in fitting, candidate selection, or retuning.

The unchanged Stage 0-B composite reference and `TrajectoryFollower` are then executed once in
each mode. This checks whether a primitive-level correction survives closed-loop path following.

### Phase 4 — frozen EXP-02B OLD replay

The immutable saved OLD body commands for all three EXP-02B cases are replayed in nominal and
calibrated modes. The source commands are identical across modes. This is an execution replay,
not a new reconciliation result; `k=3 / raw_k` identifies the diagnostic context but does not
alter the OLD commands.

## 4. Calibration/validation separation and frozen gates

The following criteria were present in the configuration before the primary results:

| Held-out criterion | Gate |
|---|---:|
| sign correctness for `abs(omega_des)>=0.15` | 100% |
| per-condition steady mean absolute omega error | `<=max(0.06, 0.20*abs(omega_des)) rad/s` |
| aggregate body omega RMSE | `<=0.12 rad/s` |
| aggregate body v RMSE | `<=0.06 m/s` |
| condition repeatability std of steady omega | `<=0.05 rad/s` |
| sustained measured-omega sign alternations | `<=4` |
| active saturation fraction per condition | `<=20%` |

The Stage 0-B composite absolute gates remain final position `<0.20 m`, position RMS
`<0.15 m`, final yaw `<0.10 rad`, and yaw RMS `<0.10 rad`. A predeclared non-degradation check
also compares calibrated with nominal. The representative EXP-02B gate requires at least 30%
angular-RMSE reduction and spatial RMS no larger than the frozen tolerance
`max(1.25 * nominal, nominal + 0.02 m)`, which avoids an unrealistically tight allowance when
the nominal RMS is small.

The final `EXECUTION_LAYER_CALIBRATED_AND_VALIDATED` decision requires all primary gates,
stable/sufficiently unsaturated behavior, no held-out tuning, and the representative replay
improvement. Results did not cause any threshold change.

## 5. Telemetry and artifact contract

Each trial stores `raw/telemetry.csv`, `raw/actual_trajectory.npy`, derived metrics, raw hashes,
and metadata. CSV rows contain:

- sample/control index, simulation time, and `INITIAL_STOP/ACTIVE/FINAL_STOP` phase;
- desired and executed `v,omega`;
- ideal left/right and canonical front-left/front-right/rear-left/rear-right wheel targets;
- directly measured four wheel-joint velocities;
- actual world `x,y,yaw`, finite-difference measured body `v,omega`, yaw error, and saturation.

World coordinates are `[x_m,y_m,yaw_CCW_about_+Z_rad]`. Body motion is the finite difference
over each physics interval and is compared with the command applied over that ending interval.
This timing convention is explicit; no LightNav waypoint row is assigned an intrinsic timestamp.
The top-level files include the config snapshot, metadata, calibration model and provenance,
frozen criteria, four phase summaries, final decision, and eight unit-labelled PNG plots.
Existing paths are rejected rather than overwritten, and every fitting input raw file is
identified by SHA-256.

## 6. Baseline characterization results

The nominal mapping is deterministic under these repeated resets but strongly condition
dependent. Representative steady-state and full-active-window results are:

| `v` | `omega` | steady omega gain | body omega RMSE | body v RMSE | wheel RMSE |
|---:|---:|---:|---:|---:|---:|
| 0.00 | -0.30 | 0.2003 | 0.2440 rad/s | 0.0187 m/s | 0.1790 rad/s |
| 0.00 | +0.30 | 0.3644 | 0.2048 rad/s | 0.0073 m/s | 0.2395 rad/s |
| 0.15 | -0.30 | 0.0603 | 0.2822 rad/s | 0.0109 m/s | 0.0384 rad/s |
| 0.15 | +0.30 | 0.0521 | 0.2847 rad/s | 0.0159 m/s | 0.0594 rad/s |
| 0.30 | -0.60 | 0.0772 | 0.5551 rad/s | 0.0284 m/s | 0.0332 rad/s |
| 0.30 | +0.60 | 0.0717 | 0.5586 rad/s | 0.0349 m/s | 0.4218 rad/s |
| 0.30 | 0.00 | n/a | 0.00035 rad/s | 0.0235 m/s | 0.0372 rad/s |

Across nonzero-yaw conditions, gain ranged from `0.01950` to `0.36440`. Mean gain by
`abs(omega)` was `0.02390 / 0.13199 / 0.14402` for `0.15 / 0.30 / 0.60 rad/s`; mean gain by
`v` was `0.20107 / 0.04650 / 0.05234` for `0 / 0.15 / 0.30 m/s`. The largest paired left/right
absolute yaw-response difference was `0.04922 rad/s` at `v=0, abs(omega)=0.30`.
Repetition standard deviations were effectively numerical zero, so the mismatch is repeatable
for these deterministic simulator resets.

The effective-width diagnostic was undefined wherever measured yaw stayed below the frozen
floor. Where defined, representative medians ranged from `0.898 m` (`v=0,+0.30`) to
`8.216 m` (`v=0.15,+0.60`), with values near `4.88–5.21 m` for several moving high-rate arcs.
Its dependence on speed, magnitude, and direction is incompatible with treating one constant
effective width as a faithful plant model.

Wheel and body evidence must be kept separate. Some moving arcs had small wheel RMSE while
retaining very low body-yaw gain—for example `v=0.15, omega=-0.30` had wheel RMSE
`0.0384 rad/s` but yaw gain `0.0603`. Thus wheel-target tracking error alone does not explain
all body-yaw loss. Other conditions, especially rotations, also had material wheel error. This
supports an empirical body/contact/skid-steer response contribution, but without slip/contact
force instrumentation it does not identify a unique mechanical cause.

## 7. Calibration model and rejected alternatives

The fitted nominal gain was `c=0.1360058`, yielding scale `7.3526301` and controller-only
effective separation `2.7615743 m`. Leave-one-condition-out omega RMSE (`0.04948 rad/s`), sign,
and scale checks passed. Worst condition mean absolute prediction error was `0.12748 rad/s`
against `0.12`, and condition-gain range was `0.34490` against `0.20`; Option A was therefore
insufficient.

| PI candidate (`kp,ki`) | calibration omega RMSE | calibration v RMSE | all gates |
|---|---:|---:|---:|
| gentle `(1.0,0.5)` | 0.37337 rad/s | 0.07993 m/s | FAIL |
| moderate `(2.0,1.0)` | 0.25906 rad/s | 0.08366 m/s | FAIL |
| strong `(3.0,2.0)` | 0.19020 rad/s | 0.08225 m/s | FAIL |

All candidates preserved signs and avoided active saturation and unstable sign alternation,
but each failed aggregate and per-condition accuracy. The strong candidate was frozen for the
subsequent falsification tests because it had the lowest calibration omega RMSE—not because it
passed. No held-out observation changed this selection.

## 8. Held-out results

| Mode | body omega RMSE | body v RMSE | sign | repeatability | oscillation | saturation | overall |
|---|---:|---:|---:|---:|---:|---:|---:|
| nominal | 0.32423 rad/s | 0.02664 m/s | PASS | PASS | PASS | PASS | FAIL |
| selected calibrated | 0.18354 rad/s | 0.10130 m/s | PASS | PASS | PASS | PASS | **FAIL** |

The selected candidate reduced aggregate omega RMSE by 43.4%, but still missed `0.12 rad/s`,
increased linear RMSE beyond `0.06 m/s`, and failed the per-condition omega-error rule. It
passed all four held-out conditions at `v=0.22`, but failed all four at `v=0.10`; at `v=0.35`,
the `abs(omega)=0.20` conditions passed while both `abs(omega)=0.45` conditions failed. Steady
gain ranged from about `1.03` to `1.57`. The largest held-out calibrated left/right response
difference was `0.12250 rad/s` at `v=0.35 m/s, abs(omega)=0.45 rad/s`. Performance therefore
remains direction-dependent in some conditions and clearly depends on `v` and `abs(omega)`.

## 9. Stage 0-B composite result

| Metric | nominal | calibrated | absolute gate |
|---|---:|---:|---:|
| goal reached | false | true | PASS |
| position RMS | 0.05319 m | 0.02201 m | PASS |
| final position error | 0.05334 m | 0.07863 m | PASS |
| yaw RMS | 0.22918 rad | 0.02308 rad | PASS |
| final yaw error | 0.14085 rad | 0.00398 rad | PASS |
| desired-omega vs measured RMSE | 0.63217 rad/s | 0.03138 rad/s | descriptive |
| desired-v vs measured RMSE | 0.07726 m/s | 0.12016 m/s | descriptive |
| wheel RMSE | 0.80919 rad/s | 0.36225 rad/s | descriptive |

The calibrated composite passes every absolute Stage 0-B engineering gate and substantially
improves yaw/path RMS. Its final position error nevertheless increases by `0.02528 m`, so the
predeclared strict non-degradation comparison fails. This mixed outcome is not used to override
the failed held-out grid.

## 10. Frozen EXP-02B OLD replay result

| Case | Mode | OLD spatial RMS | body omega RMSE | body v RMSE | wheel RMSE | B error `[m,rad]` |
|---|---|---:|---:|---:|---:|---:|
| high delta omega | nominal | 0.06460 m | 0.63989 rad/s | 0.12133 m/s | 0.84874 rad/s | 0, 0 |
| high delta omega | calibrated | 0.09296 m | 0.62880 rad/s | 0.21764 m/s | 1.05665 rad/s | 0.13212, 1.60263 |
| high delta v | nominal | 0.07424 m | 0.58315 rad/s | 0.09989 m/s | 0.72874 rad/s | 0, 0 |
| high delta v | calibrated | 0.11086 m | 0.64854 rad/s | 0.21882 m/s | 1.11977 rad/s | 0.14099, 0.94042 |
| benign delayed | nominal | 0.05616 m | 0.00945 rad/s | 0.05871 m/s | 0.19564 rad/s | 0, 0 |
| benign delayed | calibrated | 0.05617 m | 0.00945 rad/s | 0.05873 m/s | 0.30238 rad/s | 0.00004, 0.00004 |

In the representative high-angular case, omega RMSE falls only 1.73%, far below the frozen
30% criterion, while OLD spatial RMS rises past its `0.08460 m` guard and wheel/linear error
increase. The calibrated replay does not reproduce B because the intentionally altered
execution command changes the physical replay; this is not an exact-reset display artifact.
The nominal branch still reproduces B exactly and confirms the historical replay protocol.

## 11. GUI qualitative validation

Use the completed primary run so the GUI reads its frozen selected controller:

```bash
./scripts/isaac/run_jackal_execution_calibration_gui.sh \
  data/stage0/execution_calibration/stage0d-20260907T102700Z \
  --suite primitive --scenario all --real-time-factor 0.5 --no-hold

./scripts/isaac/run_jackal_execution_calibration_gui.sh \
  data/stage0/execution_calibration/stage0d-20260907T102700Z \
  --suite composite --real-time-factor 0.5 --no-hold

./scripts/isaac/run_jackal_execution_calibration_gui.sh \
  data/stage0/execution_calibration/stage0d-20260907T102700Z \
  --suite exp02b --case case_high_delta_omega --k 3 --method raw_k \
  --real-time-factor 0.5 --no-hold
```

The legend is blue reference/planned OLD, orange-red nominal actual, green calibrated actual,
grey raw FRESH, yellow saved B, orange nominal reproduced B, and green calibrated reproduced B.
Primitive mode covers straight, left/right rotations, and left/right arcs. Composite overlays
the unchanged Stage 0-B reference and both actual histories. EXP-02B overlays planned OLD,
nominal/calibrated OLD histories, raw FRESH, and three logically separate boundary markers.

Terminal live telemetry reports suite/scenario/phase/mode; desired, executed, and measured
`v,omega`; yaw error; four target/measured wheel velocities; calibration parameters and state;
and saturation. Diagnostic slowdown and final hold affect GUI wall-clock presentation only and
are never used for quantitative timing. Actual GUI runs confirmed Jackal motion, both turn
directions, all three path colors, EXP-02B overlays/markers, and live body/wheel telemetry.
Composite and EXP-02B viewport captures are retained under the primary run's `gui_metadata/`.

## 12. Reproduction commands

Create a new immutable primary run, then independently regenerate its summaries/plots:

```bash
./scripts/isaac/run_jackal_execution_calibration.sh --run-id <new_run_id>
.venv/bin/python scripts/summarize_execution_calibration.py \
  data/stage0/execution_calibration/<new_run_id>
```

The headless runner performs characterization, fitting/candidate selection, held-out paired
validation, composite execution, and all three frozen replays in that order. It refuses an
existing run ID and validates each trial's raw provenance immediately after its exclusive
write. The summarizer evaluates only the frozen criteria and refuses to overwrite an existing
final summary or plot set.

## 13. Claims, limitations, and final decision

Repository facts: the historical Stage 0-B/EXP-02B code and results remain nominal; the new
controller is opt-in and only feeds corrected body omega into the unchanged official
`DifferentialController`. Observed facts: nominal yaw response is repeatable but nonlinear and
condition-dependent; wheel mismatch exists, yet low body-yaw gain remains in conditions with
small wheel error; a single effective width is inadequate; the tested PI improves some
aggregate and composite metrics but fails primary held-out and replay gates.

Research interpretation: a body/contact/skid-steer contribution is plausible, and wheel
tracking alone is insufficient as an explanation. It remains impossible here to assign a
causal fraction to controller, tire contact, joint actuation, or their interaction. There is no
contact-force/slip intervention and physics was intentionally not tuned. The empirical
correction must not be described as having solved skid steer.

Final decision: **`EXECUTION_LAYER_NOT_YET_VALIDATED`**. The controller must not be frozen as
the reconciliation research platform, and the selected parameters must not be promoted as
production calibration. The bounded experiment stops here instead of inventing a more complex
controller. These engineering results do not improve, invalidate, or otherwise constitute
evidence for any reconciliation optimization method.
