# EXP-02B — Controller-Aware k-Conditioned SE(2) Reconciliation Evaluation

## Why this experiment exists

The redesigned EXP-01B showed that geometric staleness and controller discontinuity are
distinct: latency reliably increased OLD travel and pose gap, but did not uniformly increase
the immediate controller changes `|delta_v|` or `|delta_omega|`. EXP-02B therefore evaluates
the existing EXP-02A k-conditioned geometric reconciliation through the same Stage 0-B
Jackal `TrajectoryFollower` and Isaac `DifferentialController` used to establish the
execution-level problem.

This is a three-case development/stress evaluation, not a representative benchmark. The
spatial entry index k remains manual/oracle input and is not a temporal delay. No LightNav
inference was rerun, no selector was implemented, and the EXP-02A graph objective and weights
were not changed.

## Frozen sources and selection

Source root:
`data/exp01b_redesign/exp01b-controlled-primary-20260906T-frozen/`.
Selection was completed and recorded before producing any method output. Only timing-valid,
moving-FRESH trials were eligible; ties use the full path.

| Case | Deterministic rule | Frozen source | Latency condition | B `[x,y,yaw]` | raw `delta_v` / `delta_omega` | tangent disagreement | FRESH yaw progression | lateral departure |
|---|---|---|---|---|---:|---:|---:|---:|
| A | maximum `delta_v_abs_mps` | `primary/G1_turn/L0_natural/attempt_003` | natural; predict 442.44 ms, effective 0.450 sim s | `[0.229755, 7.895332, -0.305596]` | 0.416563 / 0.511720 | 0.509614 rad | 1.146781 rad | 0.906417 m |
| B | maximum `delta_omega_abs_rps` | `primary/G1_turn/L1_added_050/attempt_004` | +0.50 s; predict 446.71 ms, effective 0.967 sim s | `[0.296511, 7.870988, -0.193296]` | 0.332694 / 1.405038 | 0.228708 rad | 0.327284 rad | 0.384764 m |
| C | minimum normalized combined discontinuity among moving L1 | `primary/G2_route_change/L1_added_050/attempt_002` | +0.50 s; predict 441.98 ms, effective 0.950 sim s | `[0.469626, -7.799783, 0.404015]` | 0.004422 / 0.001972 | 0.000433 rad | 0.149996 rad | 0.033676 m |

The normalization medians for Case C were 0.0132599 m/s and 0.0164386 rad/s with epsilon
`1e-12`; its frozen score was 0.453482. The config stores SHA-256 for each source OLD/FRESH
raw array, OLD/FRESH world path, source controller CSV, attempt JSON, and metadata JSON.
Validation recomputes all hashes. In particular, FRESH world hashes are
`b42af924...f5ac3` (A), `ea42defe...ba71` (B), and `3c501284...084` (C); attempt hashes are
`b3b32f47...afd71`, `36cc7ef9...e84`, and `1eae8ce9...17b4b`, respectively. Exact complete
hashes are in `configs/exp02b_controller_aware.yaml` and the generated
`source_selection.json`.

## Frozen protocol

Every case uses k in `{0, 3, 6}` and exactly five methods:

- M0: unmodified upstream `FRESH[0:]`, independent of k.
- M1: same-k backend baseline `FRESH[k:]`.
- M2: EXP-02A diagnostic pose anchor plus complete FRESH-motion chain.
- M3: analytic rigid SE(2) transform.
- M4: current EXP-02A incoming-motion-aware graph, unchanged.

For M3, with previous measured pose P, committed pose B, and selected raw entry Fk,
`u_old=(B_xy-P_xy)/||B_xy-P_xy||`, the target translation is
`B_xy + ||Fk_xy-B_xy|| u_old`, and target yaw satisfies
`yaw(B^-1 Xk)=yaw(P^-1 B)`. The single left correction `Xk Fk^-1` is applied to the entire
suffix, so its internal relative motion is exactly preserved. Undefined near-zero incoming
directions are rejected using the same threshold as EXP-02A.

The execution uses Isaac Sim 6.0.1, the official experimental `DifferentialController`, the
Stage 0-B follower configuration, physics dt 1/60 s, controller dt 0.1 s, and a 2.0 s branch.
The source OLD commands are replayed first. Three qualification replays of the turning case
reproduced B and the last OLD command exactly at stored precision. The gate was then frozen at
5 mm translation, 0.01 rad yaw, and `1e-12` command error. After passing that gate, each branch
is reset exactly to saved B with the last OLD wheel target restored. This reset is disclosed as
the fairest deterministic post-switch comparison available; it is not a new controller.

### Separate GUI execution diagnosis

The frozen 45-branch quantitative protocol, M0–M4 definitions, candidates, and summary metrics
remain unchanged. A separate diagnostic-only runner makes the replay/reset boundary and
execution layers observable without writing into the frozen run:

```bash
./scripts/isaac/run_exp02b_gui_diagnosis.sh \
  --case case_high_delta_omega \
  --k 3 \
  --method raw_k
```

It simultaneously draws planned OLD, OLD replay actual, raw full FRESH, selected FRESH[k:],
the chosen frozen candidate, and post-reset actual. `B_saved`, `B_reproduced`, the exact-reset
pose, FRESH observation, `F_k`, and `X_k` are separate logical markers. Diagnostic-only paused
holds expose `PRE_RESET_INSPECTION` and `EXACT_BOUNDARY_RESET`; terminal JSON exposes every
phase plus live body/wheel telemetry. OLD versus reference is measured only as nearest-polyline
spatial deviation because LightNav waypoint rows have no intrinsic timestamps.

The representative GUI smoke reproduced B exactly and observed OLD reference-distance RMS
`0.06460 m`, body command-versus-measured RMSE `0.12133 m/s` and `0.63989 rad/s`, and aggregate
wheel target-versus-measured RMSE `0.84874 rad/s`. These are descriptive execution observations,
not proof of a skid-steer/contact root cause. The complete protocol, colors, telemetry schema,
output contract, observations, and interpretation limits are in
[EXP-02B GUI diagnosis](EXP_02B_GUI_DIAGNOSIS.md).

## Offline reconciliation results

For M1–M4, `entry` is `Log(Fk^-1 Xk)` translation; M0 is intentionally evaluated against its
own FRESH[0:] reference because it is not k-conditioned. `edge` is complete suffix
relative-motion RMS; `endpoint`, yaw progression difference, and lateral difference use the
same corresponding raw reference. M0/M1 rows are grouped only where these self-referenced
values are identical. All yaw-progression differences rounded to 0, and every method retained
the raw turn sign.

|Case|k|Method|entry m|edge RMS m/rad|endpoint m|yaw-prog diff rad|lateral diff m|
|-|-:|-|-:|-|-:|-:|-:|
|A|0|M0/M1|0.0000|8.83e-18/0|0.0000|0.0000|0.0000|
|A|0|M2|0.0567|3.81e-14/4.28e-14|0.0692|0.0000|-0.0690|
|A|0|M3|0.0418|1.24e-15/0|0.0558|0.0000|-0.0485|
|A|0|M4|0.0417|4.22e-12/5.87e-12|0.0485|0.0000|-0.0435|
|A|3|M0/M1|0.0000|8.44e-18/0|0.0000|0.0000|0.0000|
|A|3|M2|0.4998|3.48e-14/3.64e-14|0.5087|0.0000|-0.4696|
|A|3|M3|0.3184|9.28e-16/0|0.5456|0.0000|-0.4543|
|A|3|M4|0.2542|5.78e-12/1.19e-11|0.3605|0.0000|-0.3085|
|A|6|M0/M1|0.0000|5.67e-18/0|0.0000|0.0000|0.0000|
|A|6|M2|0.9529|3.56e-15/3.98e-15|0.9677|0.0000|-0.8007|
|A|6|M3|0.7649|1.31e-15/0|0.9202|0.0000|-0.7945|
|A|6|M4|0.4012|9.68e-16/2.56e-16|0.4719|0.0000|-0.4083|
|B|0|M0/M1|0.0000|7.56e-18/0|0.0000|0.0000|0.0000|
|B|0|M2|0.0204|2.03e-13/2.67e-13|0.2585|0.0000|0.2458|
|B|0|M3|0.0408|1.15e-15/0|0.2961|0.0000|0.2846|
|B|0|M4|0.0407|6.73e-15/1.04e-14|0.1462|0.0000|0.1456|
|B|3|M0/M1|0.0000|6.45e-18/0|0.0000|0.0000|0.0000|
|B|3|M2|0.4329|3.51e-14/3.18e-14|0.4968|0.0000|0.0624|
|B|3|M3|0.1035|1.22e-15/0|0.1157|0.0000|0.0879|
|B|3|M4|0.0871|3.42e-15/6.26e-15|0.0299|0.0000|0.0113|
|B|6|M0/M1|0.0000|4.14e-18/0|0.0000|0.0000|0.0000|
|B|6|M2|0.8829|4.55e-15/1.38e-15|0.9026|0.0000|-0.1415|
|B|6|M3|0.2081|7.21e-16/0|0.1299|0.0000|-0.1287|
|B|6|M4|0.1169|1.92e-15/0|0.0764|0.0000|-0.0762|
|C|0|M0/M1|0.0000|7.55e-18/0|0.0000|0.0000|0.0000|
|C|0|M2|0.1985|6.31e-14/2.66e-15|0.1985|0.0000|-0.0005|
|C|0|M3|0.3970|9.83e-16/0|0.3970|0.0000|-0.0005|
|C|0|M4|0.3819|8.99e-15/1.66e-14|0.3819|0.0000|-0.0003|
|C|3|M0/M1|0.0000|8.88e-18/0|0.0000|0.0000|0.0000|
|C|3|M2|0.2538|1.49e-14/8.11e-16|0.2539|0.0000|0.0015|
|C|3|M3|0.0002|7.25e-16/0|0.0015|0.0000|0.0015|
|C|3|M4|0.0002|2.02e-12/3.20e-12|0.0007|0.0000|0.0007|
|C|6|M0/M1|0.0000|3.65e-18/0|0.0000|0.0000|0.0000|
|C|6|M2|0.7048|3.42e-15/0|0.7044|0.0000|-0.0045|
|C|6|M3|0.0009|1.28e-15/0|0.0045|0.0000|-0.0045|
|C|6|M4|0.0006|1.20e-11/2.88e-12|0.0021|0.0000|-0.0021|

Inter-k entry-separation retention for pairs `(0,3)/(0,6)/(3,6)` was:

| Case | M1 raw-k | M2 pose anchor | M3 rigid | M4 graph |
|---|---|---|---|---|
| A | 1/1/1 | 0/0/0 | 0.999/1.000/0.963 | 0.963/0.915/0.905 |
| B | 1/1/1 | 0/0/0 | 0.911/0.956/1.000 | 0.908/0.949/0.994 |
| C | 1/1/1 | 0/0/0 | 0.122/0.561/1.000 | 0.156/0.577/1.000 |

Thus pose anchoring completely collapsed k semantics. M3/M4 mostly retained separation for
the stress cases, but early-versus-middle benign entries collapsed substantially. All M3/M4
suffixes were effectively rigid internally; the graph's maximum edge residual was numerical
noise (`1.2e-11`).

## Controller execution results

All 45 branches passed the frozen pre-switch comparability gate. `max3` and `mean3` show
absolute consecutive command changes over the switch plus the next two control intervals.
Signed changes, the exact first three OLD/FRESH commands, and discrete slew are retained in
each branch `controller_metrics.json`.

|Case|k|Method|`delta_v`|`delta_omega`|max3 v/omega|mean3 v/omega|
|-|-:|-|-:|-:|-|-|
|A|0|M0/M1|0.4166|0.5117|0.4166/0.5117|0.1522/0.2083|
|A|0|M2|0.3535|0.7684|0.3535/0.7684|0.1316/0.3297|
|A|0|M3|0.4140|1.0416|0.4140/1.0416|0.1532/0.4272|
|A|0|M4|0.4137|1.0267|0.4137/1.0267|0.1528/0.4297|
|A|3|M0|0.4166|0.5117|0.4166/0.5117|0.1522/0.2083|
|A|3|M1|0.0000|0.6120|0.5265/0.6120|0.3510/0.2510|
|A|3|M2|0.3568|0.6739|0.3568/0.6739|0.1321/0.3091|
|A|3|M3|0.5369|2.3833|0.5369/2.3833|0.1815/0.8309|
|A|3|M4|0.5488|2.0426|0.5488/2.0426|0.1837/0.7103|
|A|6|M0|0.4166|0.5117|0.4166/0.5117|0.1522/0.2083|
|A|6|M1|0.0000|0.8240|0.0000/0.8240|0.0000/0.3140|
|A|6|M2|0.3137|0.8611|0.3137/0.8611|0.1171/0.3464|
|A|6|M3|0.0000|2.4927|0.0000/2.4927|0.0000/0.8309|
|A|6|M4|0.5491|1.9189|0.5491/1.9189|0.1833/0.6690|
|B|0|M0/M1|0.3327|1.4050|0.3327/1.4050|0.1259/0.5413|
|B|0|M2|0.3617|0.9847|0.3617/0.9847|0.1353/0.3563|
|B|0|M3|0.3837|1.0184|0.3837/1.0184|0.1433/0.3634|
|B|0|M4|0.3827|1.2747|0.3827/1.2747|0.1436/0.4725|
|B|3|M0|0.3327|1.4050|0.3327/1.4050|0.1259/0.5413|
|B|3|M1|0.5402|1.4280|0.5402/1.4280|0.1848/0.5689|
|B|3|M2|0.3602|0.9850|0.3602/0.9850|0.1348/0.3564|
|B|3|M3|0.5347|2.3378|0.5347/2.3378|0.1808/0.8097|
|B|3|M4|0.5314|2.2092|0.5314/2.2092|0.3543/0.7737|
|B|6|M0|0.3327|1.4050|0.3327/1.4050|0.1259/0.5413|
|B|6|M1|0.5429|1.5236|0.5429/1.5236|0.1847/0.5891|
|B|6|M2|0.3607|0.8111|0.3607/0.8111|0.1345/0.2901|
|B|6|M3|0.5268|2.4863|0.5268/2.4863|0.3512/0.8288|
|B|6|M4|0.5347|2.4170|0.5347/2.4170|0.1808/0.8288|
|C|0|M0/M1|0.0044|0.0020|0.1438/0.0059|0.0657/0.0033|
|C|0|M2|0.0521|0.0033|0.0521/0.0055|0.0430/0.0035|
|C|0|M3|0.2410|0.0037|0.2410/0.0037|0.0962/0.0018|
|C|0|M4|0.2410|0.0036|0.2410/0.0036|0.1021/0.0018|
|C|3|M0|0.0044|0.0020|0.1438/0.0059|0.0657/0.0033|
|C|3|M1|0.2410|0.0058|0.2410/0.0058|0.0803/0.0027|
|C|3|M2|0.0521|0.0096|0.0521/0.0096|0.0430/0.0058|
|C|3|M3|0.2410|0.0072|0.2410/0.0072|0.0803/0.0029|
|C|3|M4|0.2410|0.0072|0.2410/0.0072|0.0803/0.0030|
|C|6|M0|0.0044|0.0020|0.1438/0.0059|0.0657/0.0033|
|C|6|M1|0.2409|0.0173|0.2409/0.0173|0.0803/0.0085|
|C|6|M2|0.0515|0.0876|0.0515/0.0876|0.0428/0.0371|
|C|6|M3|0.2409|0.0231|0.2409/0.0231|0.0803/0.0091|
|C|6|M4|0.2409|0.0040|0.2409/0.0040|0.0803/0.0035|

## Interpretation

The current graph did **not** show consistent execution-level benefit.

- High-linear Case A: at k=0, graph reduced `delta_v` by only 0.69% (0.4166 to 0.4137)
  while doubling `delta_omega`; at k=3 and k=6 it introduced large linear jumps and worsened
  angular jumps. This is not a meaningful linear-discontinuity improvement.
- High-angular turning Case B: graph reduced `delta_omega` by 9.27% only at k=0 (1.4050 to
  1.2747), while worsening `delta_v` by 15.0%. At k=3 and k=6 it worsened angular jumps by
  54.7% and 58.6% relative to the same-k raw suffix.
- Benign Case C: at k=0, graph changed an already benign `delta_v=0.0044` into 0.2410 m/s.
  It therefore failed the leave-benign-transitions-alone check. Its k=6 angular reduction
  (0.0173 to 0.0040 rad/s) did not offset the unchanged 0.241 m/s linear jump.
- All modified suffixes preserved turn sign and yaw progression, and M4's internal edge
  deformation was numerical. Nevertheless, endpoint/lateral displacement was sometimes large;
  Case A k=3 graph moved the endpoint 0.361 m and lateral departure by -0.309 m.
- The analytic rigid baseline and graph were geometrically similar because both were nearly
  rigid. Neither consistently improved controller metrics. The graph sometimes reduced the
  rigid method's endpoint displacement or angular penalty, but no result demonstrates that the
  nonlinear graph is necessary under the present factors.
- M0 must not be confused with M1. M0 is the upstream naive LightNav reference. M1 is the clean
  backend comparison for a supplied k. Some oracle-k branches differ from M0, but these selected
  cases show no consistent reconciled advantage, and manual k prevents an end-to-end claim.

This outcome matches decision category C/E: the current geometric objective is not aligned
reliably with execution-level smoothness, and it can modify a controller-benign transition.
EXP-02B therefore weakens the case for the current graph formulation as a controller-facing
method, while validating the evaluation machinery and preserving the EXP-02A geometric pilot.

## Artifacts and reproducibility

Final immutable run:
`data/exp02b/exp02b-controller-aware-20260906T150400Z/`.
It contains 45 candidate arrays, geometric metrics, actual trajectories, command traces,
comparability records, strict JSON summaries, validation, and nine PNG figures. The strict
validator reports source hashes matched, 45/45 execution branches valid, and nine plots.

```bash
.venv/bin/python scripts/run_exp02b_controller_aware.py --run-id <new_run_id>
./scripts/isaac/run_exp02b_branch_execution.sh data/exp02b/<new_run_id>
.venv/bin/python scripts/summarize_exp02b.py data/exp02b/<new_run_id>
```

## What can and cannot be claimed

Implementation validation: deterministic selection, immutable provenance, analytic rigid math,
unchanged graph reproduction, controller metric reuse, fair branch reset, and full output
validation passed. Negative research evidence: on these selected real LightNav transitions,
the current graph did not consistently reduce same-k controller discontinuity and altered the
benign case unnecessarily. Positive but limited evidence: turn sign, yaw progression, and
internal suffix relative motion were retained.

It is not supported that the selector works, that any k is best, that the graph outperforms
LightNav end to end, that graph optimization is necessary, or that navigation, obstacle
avoidance, generalization, real-robot behavior, or average occurrence rates improve.

The smallest next experiment should change one formulation aspect only: add a controller-aware
transition residual or an explicit reconciliation gate, then evaluate it against the frozen
M1/M3/M4 branches on these same cases before any larger held-out study. The current results
especially motivate a gate that leaves low-command-discontinuity transitions untouched. The
choice and units of such a residual remain the primary uncertainty.
