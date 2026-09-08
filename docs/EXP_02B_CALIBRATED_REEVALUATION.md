# EXP-02B-R: Frozen Candidate Re-evaluation under Calibrated Execution

## Purpose and claim boundary

EXP-02B-R asks one narrow question: when the exact 27 historical EXP-02B
`raw_k`, `rigid`, and `graph` candidate byte arrays are executed from the same
frozen switch boundary through the already selected Stage 0-D `pi_strong`
execution layer, does the EXP-02B formulation conclusion change?

This is an execution re-evaluation, not a new reconciliation algorithm. It does
not regenerate candidates, change EXP-02A residuals or weights, tune a controller,
add a gate or selector, or modify LightNav. Historical EXP-02B remains the result
under the nominal execution layer; EXP-02B-R is a separate calibrated post-switch
result.

The command layers are kept distinct:

1. `u_des = [v_des, omega_des]`: `TrajectoryFollower` output.
2. `u_exec = [v_exec, omega_exec]`: frozen calibrated-controller output sent to
   `DifferentialController`.
3. `u_meas = [v_meas, omega_meas]`: Jackal body motion measured from world poses
   over one physical control interval.
4. Wheel target and four measured wheel rates: articulation-level telemetry.

An executed-command jump is not an end-to-end production transition metric here:
OLD uses nominal execution while the post-switch controller is calibrated and its
PI integral is explicitly reset.

## Frozen inputs and provenance

The actual source run is
`data/exp02b/exp02b-controller-aware-20260906T150400Z`. Strict validation reconstructs
all 45 historical branches before selecting only the 27 required branches:

- cases: `case_high_delta_v`, `case_high_delta_omega`, `case_benign_delayed`
- `k`: 0, 3, 6
- methods: `raw_k` (M1), `rigid` (M3), `graph` (M4)

The full 64-character hash of every candidate is frozen in
`configs/exp02b_calibrated_reeval.yaml`; all 27 candidate files are loaded in place,
not copied or regenerated, and checked before preparation, execution, and summary.
The top historical hashes include:

- `source_selection.json`: `a879618500d978dacbb10fac2b3fafea38a323527029e47f586a6fb0948d7239`
- `protocol.json`: `add45eb55a3aa260834e88607e838bdf1cfbfc640d10c0bb3324e45ccbecc7e1`
- `execution_summary.json`: `78a9314df2ef2eed43dcc12253987375a5c5abab315156aacb476ffcbc29fac5`
- `summary.json`: `641e6162b5dd4cd3052067fd14224c8886e3585a5e202f2540fd5558b819588b`

The execution controller is loaded from
`data/stage0/execution_calibration/stage0d-20260907T102700Z`, model SHA-256
`40821584e14a3f444fdd19ff27acc03e752ec3d0c5f2e8635b824a1419a81464`.
Its frozen values are `pi_strong`, `kp=3`, `ki=2`, feedforward scale
`7.352630067488997`, effective separation `2.7615743018750605 m`, maximum executed
omega `8 rad/s`, wheel guard `30 rad/s`, sign protection, and conditional
anti-windup. No value was refit for EXP-02B-R.

Stage 0-F provenance is also checked before execution. Its bounded result supports
this observed LightNav workload only; it does not erase the Stage 0-E strong-turn
failures or globally validate the platform.

## Controlled boundary protocol

Every branch uses the historical EXP-02B comparability semantics:

1. replay the source OLD command rows with nominal execution;
2. check `B_reproduced` and the last OLD desired command against the saved source;
3. exact-reset the robot to `B_saved`;
4. restore the final nominal OLD wheel target;
5. reset the calibrated PI integral to zero;
6. use the final pre-reset OLD physical interval as the first calibrated feedback
   measurement;
7. execute the frozen candidate for 2.0 s at 0.1 s control / 1/60 s physics timing.

All 27 replay gates passed. For each of the three source histories, pre-reset
translation error, wrapped-yaw error, and both desired-command errors were exactly
zero in the saved summary. Isaac's pose round-trip value used by the follower is
also recorded separately as `historical_follower_effective_boundary_se2`; this
preserves the historical first-command floating-point state rather than silently
substituting a higher-precision source row.

The final OLD interval and first post-reset interval are differentiated separately.
No body-motion sample spans the `B_reproduced -> B_saved` teleport.

## Actual run and artifacts

The completed Isaac Sim run is:

```text
data/exp02b_calibrated_reeval/exp02b-r-20260908T054233Z/
```

It contains 27/27 strict-valid branch directories, seven plots, the historical
metric snapshot, source/hash provenance, and four summary files. Each branch stores:

- `actual_trajectory.npy`, `old_replay_actual.npy`
- `telemetry.csv` with desired/executed/measured body, four target/measured wheels,
  pose, follower progress, PI state, saturation, and spatial reference fields
- `desired_commands.csv`, `executed_commands.csv`, `measured_body.csv`
- `controller_metrics.json`, `tracking_metrics.json`, `metadata.json`
- `raw_provenance.json` with hashes of the raw branch products

Outputs are under the ignored `data/` tree and use exclusive creation. The frozen
historical run is never opened for writing.

Reproduce with a new run id:

```bash
.venv/bin/python scripts/prepare_exp02b_calibrated_reeval.py \
  --run-id <new_run_id>
./scripts/isaac/run_exp02b_calibrated_reeval.sh \
  data/exp02b_calibrated_reeval/<new_run_id>
.venv/bin/python scripts/summarize_exp02b_calibrated_reeval.py \
  data/exp02b_calibrated_reeval/<new_run_id>
.venv/bin/python scripts/summarize_exp02b_calibrated_reeval.py \
  data/exp02b_calibrated_reeval/<new_run_id> --validate-only
```

## Required desired-command invariant

All 27 branches passed at tolerance `1e-12`. The maximum historical-versus-reevaluated
absolute differences were:

| quantity | maximum difference |
| --- | ---: |
| first desired `v` | `0.0 m/s` |
| first desired `omega` | `0.0 rad/s` |

Thus downstream calibration did not and cannot change the original first follower
command at the same candidate, boundary, and follower state. Any original M4
first-command worsening is a candidate/formulation property, not a low-level Jackal
execution artifact.

## Historical nominal versus calibrated physical execution

For comparability, the following are means across all 27 branches sampled at the
same 0.1 s control times:

| lower-is-better metric | historical nominal | calibrated | result |
| --- | ---: | ---: | --- |
| desired-v vs measured-v RMSE | `0.116737 m/s` | `0.119536 m/s` | worse by `0.002800` |
| desired-omega vs measured-omega RMSE | `0.596104 rad/s` | `0.217624 rad/s` | better by `0.378479` |
| nearest-candidate position RMS | `0.268973 m` | `0.281053 m` | worse by `0.012079` |
| nearest-candidate yaw RMS | `0.133262 rad` | `0.178637 rad` | worse by `0.045374` |

The nearest-candidate measures are spatial polyline/pose comparisons. LightNav
waypoint rows have no intrinsic timestamp, so these are not time-aligned trajectory
tracking errors.

The immediate measured transition was essentially unchanged on average:
`|delta v_meas| = 0.00394544 -> 0.00394546 m/s` and
`|delta omega_meas| = 0.02861954 -> 0.02861958 rad/s`. The calibrated branch
wheel-target versus measured-wheel RMSE averaged `0.689193 rad/s` (range
`0.276843–1.558392`). Historical EXP-02B did not store equivalent wheel telemetry,
so EXP-02B-R cannot claim a historical wheel-RMSE improvement. Calibrated saturation
fraction averaged `0.111111` across branches.

The physical result is therefore mixed: angular desired tracking improved strongly,
but linear tracking and the short-horizon spatial metrics did not. The 2 s branch is
a controlled transition window; not reaching the candidate endpoint is not a
navigation failure.

## M4 graph versus M1 raw_k: all nine pairs

Values are `graph / raw_k`; arrows apply lower-is-better independently to the first
measured body-motion change. M4 was better on 12 of 18 immediate components and
worse on 6, so it was not consistently better.

| case | k | measured `|delta v|` m/s | measured `|delta omega|` rad/s |
| --- | ---: | ---: | ---: |
| high-delta-v | 0 | `0.001012 / 0.001080` better | `0.074417 / 0.074424` better |
| high-delta-v | 3 | `0.004310 / 0.009067` better | `0.074495 / 0.074233` worse |
| high-delta-v | 6 | `0.004318 / 0.009068` better | `0.074495 / 0.074233` worse |
| high-delta-omega | 0 | `0.000148 / 0.001071` better | `0.009491 / 0.009468` worse |
| high-delta-omega | 3 | `0.003770 / 0.003988` better | `0.009548 / 0.009551` better |
| high-delta-omega | 6 | `0.003853 / 0.004053` better | `0.009550 / 0.009551` better |
| benign delayed | 0 | `0.003669 / 0.009654` better | `0.001953 / 0.001933` worse |
| benign delayed | 3 | `0.003669 / 0.003668` worse | `0.001953 / 0.001953` better |
| benign delayed | 6 | `0.003669 / 0.003668` worse | `0.001952 / 0.001952` worse |

Across all nine reported physical metrics per pair (immediate/max3 body changes,
desired tracking, position/yaw RMS, and wheel RMSE), graph versus raw_k counted
37 better and 44 worse outcomes. No post-result materiality threshold was invented.

## M4 graph versus M3 rigid: all nine pairs

Values are `graph / rigid`. M4 was better on 10 of 18 immediate components, worse
on 6, and tied on 2, again not consistently better.

| case | k | measured `|delta v|` m/s | measured `|delta omega|` rad/s |
| --- | ---: | ---: | ---: |
| high-delta-v | 0 | `0.001012 / 0.001021` better | `0.074417 / 0.074416` worse |
| high-delta-v | 3 | `0.004310 / 0.004018` worse | `0.074495 / 0.074482` worse |
| high-delta-v | 6 | `0.004318 / 0.009068` better | `0.074495 / 0.074233` worse |
| high-delta-omega | 0 | `0.000148 / 0.000172` better | `0.009491 / 0.009491` better |
| high-delta-omega | 3 | `0.003770 / 0.003853` better | `0.009548 / 0.009550` better |
| high-delta-omega | 6 | `0.003853 / 0.003659` worse | `0.009550 / 0.009546` worse |
| benign delayed | 0 | `0.003669 / 0.003668` worse | `0.001953 / 0.001953` worse |
| benign delayed | 3 | `0.003669 / 0.003669` tie | `0.001953 / 0.001953` better |
| benign delayed | 6 | `0.003669 / 0.003669` tie | `0.001952 / 0.001952` better |

Across all nine metrics per pair, graph versus rigid counted 43 better, 36 worse,
and 2 ties. The directions vary by case, k, and metric.

## Benign Case C, k=0

This counterexample remains important:

| method/layer | first `|delta v|` m/s | first `|delta omega|` rad/s |
| --- | ---: | ---: |
| raw_k desired | `0.004422` | `0.001972` |
| graph desired | `0.240968` | `0.003594` |
| raw_k executed | `0.004422` | `0.017681` |
| graph executed | `0.240968` | `0.000563` |
| raw_k measured | `0.009654` | `0.001933` |
| graph measured | `0.003669` | `0.001953` |

The graph's first desired linear-command worsening is byte-for-byte the historical
result. Calibration/inertia mitigated its immediate measured linear consequence,
but did not remove the candidate-level modification: graph displaced the selected
entry by `0.381925 m` while raw_k left it unchanged. Graph suffix edge deformation
remained numerical noise (translation max `1.15e-14 m`, yaw max `2.84e-14 rad`).
This is evidence that the current objective still modifies an already benign raw
transition without consistently better physical outcomes.

## GUI diagnosis

The GUI is deliberately small and uses the existing DebugDraw trajectory helpers:

- blue: exact frozen candidate
- green: calibrated post-switch actual history
- orange: historical nominal post-switch actual history
- grey: raw full FRESH context
- yellow marker: `B_saved`
- cyan marker: candidate first pose `X_k`
- magenta marker: raw `F_k`

Terminal phases are `SETTLING`, `OLD_REPLAY`, `EXACT_RESET`,
`CALIBRATED_CONTROLLER_RESET`, `POST_SWITCH_EXECUTION`, and `FINISHED`. Live rows
show desired, executed, measured, both transition deltas, follower indices, four
target/measured wheel rates, PI correction/integral, saturation, and sign protection.
The exact-reset inspection hold pauses the physics timeline; viewport refreshes do
not create an unrecorded motion interval.

Representative commands (omit `--no-hold` for interactive final inspection):

```bash
RUN=data/exp02b_calibrated_reeval/exp02b-r-20260908T054233Z
./scripts/isaac/run_exp02b_calibrated_reeval_gui.sh "$RUN" \
  --case case_high_delta_omega --k 3 --method raw_k --real-time-factor 1.0 --no-hold
./scripts/isaac/run_exp02b_calibrated_reeval_gui.sh "$RUN" \
  --case case_high_delta_omega --k 3 --method rigid --real-time-factor 1.0 --no-hold
./scripts/isaac/run_exp02b_calibrated_reeval_gui.sh "$RUN" \
  --case case_high_delta_omega --k 3 --method graph --real-time-factor 1.0 --no-hold
./scripts/isaac/run_exp02b_calibrated_reeval_gui.sh "$RUN" \
  --case case_benign_delayed --k 0 --method raw_k --real-time-factor 1.0 --no-hold
./scripts/isaac/run_exp02b_calibrated_reeval_gui.sh "$RUN" \
  --case case_benign_delayed --k 0 --method graph --real-time-factor 1.0 --no-hold
```

All five actual non-headless runs completed and their corrected captures are under
the run's `gui_metadata/`. Their first measured intervals exactly match the headless
branch results. Superseded captures from the pre-fix hold diagnostic are retained,
not overwritten, under `gui_metadata_pre_fix_do_not_use/`.

## Conclusion

Final label: `EXP02B_FORMULATION_CONCLUSION_UNCHANGED`.

Calibration did **not** rescue M4. It substantially improved aggregate angular
desired-command tracking in this bounded run, but M4's frozen first desired commands
were unchanged, M4 was not consistently better than raw_k or rigid at measured-motion
level, and the Case C benign modification persisted. The evidence does not prove a
new causal residual or identify a unique controller/contact mechanism. It does show
that removing much of the angular execution mismatch is insufficient to make the
current reconciliation objective consistently execution-smooth.
