# Redesigned EXP-01B — Controlled Latency × Verified Geometry

## Status and relationship to earlier evidence

The earlier expanded characterization remains an immutable pilot. It showed repeatable
translational spatial mismatch, but its initial-pose perturbations did not produce meaningful
turn diversity. It also treated the difference between one actual control-interval displacement
and one LightNav waypoint spacing as `translation_motion_jump`. Because LightNav waypoints have
no intrinsic time base, that quantity is not a velocity or acceleration jump. It is retained here
only as `local_spatial_step_magnitude_mismatch_m`.

This redesigned experiment is a separate, immutable run. It measures the canonical
`[v, omega]` commands produced by the already validated Stage 0-B `TrajectoryFollower` directly
before and after a raw OLD→FRESH reference replacement. It contains no reconciliation,
correspondence, spatial-entry selection, graph, blending, or smoothing.

## Questions and hypotheses

The primary question is whether effective observation-to-action latency and actual OLD/FRESH
geometry affect execution-level discontinuity at a naive asynchronous switch.

- H1: increasing effective latency while OLD continues should increase `|delta_v|`,
  `|delta_omega|`, and/or the geometric boundary mismatch.
- H2: stronger OLD/FRESH directional or curvature disagreement should make rotational command
  discontinuity more visible.

Both hypotheses are falsifiable. A larger latency or more curved FRESH output is not assumed to
produce a larger controller jump.

## Timing and raw-switch semantics

For each episode, LightNav is reset once, a stationary 64-frame history is primed, OLD is
predicted and executed, and live observations continue at 4 Hz. At the fixed 0.50 simulation-s
OLD-execution delay, the FRESH observation pose and image are recorded atomically and the stateful
LightNav server receives an asynchronous request.

The saved events distinguish:

```text
fresh_observation -> fresh_request -> fresh_model_ready
                  -> optional added delay -> fresh_usable/raw_switch
                  -> first_fresh_controller_command
```

`L0_natural` uses no added delay. `L1_added_050` holds the immutable FRESH result for another
0.50 simulation s after model completion while physics and nonzero OLD commands continue.
No `sleep` freezes simulation. Effective latency is measured from observation to FRESH usability;
model inference itself is not modified.

FRESH world poses are always anchored at the FRESH observation pose:

```text
T_world_F_i = T_world_robot_at_fresh_observation * T_robot_F_i
```

At usability the controller receives this raw observation-anchored trajectory directly. There is
no ready-pose re-anchoring, guessed stale-row removal, interpolation, smoothing, graph, or `k`.

## Primary and secondary metrics

The final OLD and first FRESH controller commands define:

```text
delta_v_signed_mps     = v_FRESH - v_OLD
delta_omega_signed_rps = omega_FRESH - omega_OLD
delta_v_abs_mps        = abs(delta_v_signed_mps)
delta_omega_abs_rps    = abs(delta_omega_signed_rps)
```

The last three OLD and first three FRESH commands are also stored. Max/mean absolute consecutive
command change over `[OLD[-1], FRESH[0:3]]` is reported. A command difference divided by the
0.10 s control period is explicitly labelled discrete controller-command slew, not physical
acceleration.

Secondary geometry includes translation/yaw pose gap at B versus FRESH row 0, actual incoming
tangent, first meaningful FRESH tangent, tangent disagreement, FRESH yaw progression, lateral
departure, and observation-to-switch robot motion. Near-zero translations yield an undefined
tangent rather than an invented direction. `local_spatial_step_magnitude_mismatch_m` compares an
actual control-interval displacement with untimed spatial waypoint spacing and must not be read as
velocity.

## Geometry qualification and frozen conditions

Qualification selected geometry only, before primary command-discontinuity results were viewed.
All diagnostic artifacts were retained.

| Candidate | Scene / instruction | Geometry descriptors | Decision |
|---|---|---|---|
| initial G0 | straight corridor / `Go straight down the corridor.` | tangent 0.00273 rad; FRESH yaw progression 0.00123 rad; endpoint lateral -0.00622 m | accepted as straight; timing-invalid diagnostic was adequate only for geometry |
| initial G1 | left intersection / `Turn left at the blue wall.` | tangent 0.00031 rad; yaw progression 0.00061 rad; lateral -0.00052 m | rejected: straight output |
| initial G2 | mirrored intersection / `Go around the red obstacle on the right and continue forward.` | tangent 0.00021 rad; yaw progression 0.00061 rad; lateral 0.00025 m | rejected: straight output |
| revised G1 | left intersection, initial yaw -0.40 / `Turn left at the blue wall and continue along the left corridor.` | tangent 0.00145 rad; yaw progression 0.32728 rad; lateral 0.07977 m | accepted as a curved/turning future despite an initially parallel first tangent |
| revised G2 | mirrored intersection, initial yaw +0.40 / `Go around the red obstacle on the right and continue forward.` | tangent 0.00038 rad; yaw progression 0.00061 rad; lateral 0.00046 m | rejected |
| G2 candidate 3 | mirrored intersection / `Turn right at the red wall and continue along the right corridor.` | tangent 0.00014 rad; yaw progression 0.00061 rad; lateral 0.00013 m | rejected |
| G2 candidate 4 | mirrored intersection / `Move immediately to the open right side, then continue forward.` | same near-straight descriptors | rejected |
| G2 candidate 5 | mirrored intersection / `Enter the doorway on your left.` | tangent 0.68042 rad; yaw progression 0.82458 rad; lateral 1.06728 m | accepted as route-change |

The frozen `configs/exp01b_controlled_latency.yaml` defines G0 `[0,0,0]`, G1
`[0,8,-0.40]`, and G2 `[0,-8,+0.40]`, their static boxes, the exact instructions above, and
the two latency conditions. The primary design is 3 geometries × 2 latencies × 5 valid moving
FRESH transitions, with a fixed cap of 10 attempts per cell.

An important primary-run observation is that the selected G2 route-change was not repeatable at
the qualification magnitude. G2 primary FRESH outputs still had nonzero future yaw progression
(median 0.150 rad), but their initial tangent disagreement was only 0.00036–0.00146 rad and
endpoint lateral departure was 0.0336–0.0798 m. G1 retained curved futures (median yaw progression
0.327 rad), but 8/10 G1 entries also began nearly parallel; two G1 samples had larger tangent
disagreement, 0.229 and 0.510 rad. This variability is preserved, not relabelled as a uniformly
verified initial turn.

## Frozen primary protocol and runtime

The primary artifact is:

```text
data/exp01b_redesign/exp01b-controlled-primary-20260906T-frozen/
```

It used Isaac Sim 6.0.1, Jackal, 1/60 s physics, 0.10 s control, headless RGB rendering,
RTF gate `[0.90,1.10]`, and one persistent LightNav process. The server used LightNav SHA
`a645828d81a8439651172197ca80a75dc1377977`, package 0.1.0, checkpoint revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`, vLLM 0.19.1, prefix caching enabled,
GPU-memory utilization 0.65, and a 1 GiB KV cache. It built once, warmed once on the saved Stage
0-C 64-frame history, reset for each of 37 episodes, and made 74 OLD/FRESH predictions. Concurrent
RTX 5060 Ti use peaked at the recorded 13,589 MiB snapshot without OOM.

The finite collection produced all 30 planned valid moving transitions in 37 attempts:

| Cell | Attempts | Moving | STOP | RTF invalid | OLD exhausted | Other |
|---|---:|---:|---:|---:|---:|---:|
| G0 × L0 | 7 | 5 | 2 | 0 | 0 | 0 |
| G0 × L1 | 9 | 5 | 1 | 0 | 3 | 0 |
| G1 × L0 | 6 | 5 | 0 | 1 | 0 | 0 |
| G1 × L1 | 5 | 5 | 0 | 0 | 0 | 0 |
| G2 × L0 | 5 | 5 | 0 | 0 | 0 | 0 |
| G2 × L1 | 5 | 5 | 0 | 0 | 0 | 0 |
| **Total** | **37** | **30** | **3** | **1** | **3** | **0** |

The three real FRESH STOP outputs are retained and summarized separately. They do not count toward
the moving target. Their immediate command changes were large because raw switching commands stop/
rotate behavior (`|delta_v|` 0.229–0.348 m/s; `|delta_omega|` 1.495–1.497 rad/s), but they mix
latency with a new STOP decision and are not the primary moving-FRESH evidence.

## Latency manipulation

Across the 15 moving trials in each latency level:

| Quantity | Natural mean / median / range | +0.5 s mean / median / range |
|---|---|---|
| model wall latency [s] | 0.4452 / 0.4434 / 0.4302–0.4565 | 0.4425 / 0.4439 / 0.4247–0.4522 |
| effective simulation latency [s] | 0.4589 / 0.4500 / 0.4500–0.5000 | 0.9700 / 0.9667 / 0.9333–1.0000 |
| robot travel observation→switch [m] | 0.1651 / 0.1663 / 0.0997–0.1835 | 0.3331 / 0.3491 / 0.1705–0.3645 |
| RTF | 1.0090 / 1.0137 / 0.9585–1.0162 | 0.9873 / 0.9816 / 0.9706–1.0115 |

The manipulation succeeded: effective latency increased by about 0.51 simulation s and mean
robot travel increased by 0.1680 m. Effective latency and travel had descriptive Pearson
`r=0.926` over the 30 moving samples.

## Controller discontinuity results

Raw samples, p25/p75, standard deviations, and short-window values are in `summary.json`. The key
cell means/medians are:

| Cell | `|delta_v|` mean / median [m/s] | `|delta_omega|` mean / median [rad/s] | post-switch max `|delta_v|` mean | post-switch max `|delta_omega|` mean |
|---|---:|---:|---:|---:|
| G0 × L0 | 0.0195 / 0.0264 | 0.0154 / 0.0142 | 0.1582 | 0.0154 |
| G0 × L1 | 0.1158 / 0.1274 | 0.0229 / 0.0318 | 0.1158 | 0.0229 |
| G1 × L0 | 0.0944 / 0.0133 | 0.1216 / 0.0265 | 0.1994 | 0.1216 |
| G1 × L1 | 0.0739 / 0.0090 | 0.3035 / 0.0336 | 0.1821 | 0.3035 |
| G2 × L0 | 0.00747 / 0.00781 | 0.00177 / 0.00118 | 0.1446 | 0.00181 |
| G2 × L1 | 0.0690 / 0.00489 | 0.00224 / 0.00197 | 0.1527 | 0.00361 |

Latency increased `delta_v` clearly in G0. G1 `delta_v` decreased by mean and median. G2 had two
0.166 m/s delayed outliers, raising the mean while its median decreased. Median `delta_omega`
increased modestly in all three geometries, but the absolute changes were small in G0/G2. G1's
means were dominated by one high-geometry sample per latency cell (`0.512` and `1.405 rad/s`);
the cell medians were only `0.0265` and `0.0336 rad/s`.

Across all moving samples, effective latency correlation was `r=0.241` with `|delta_v|` and
`r=0.117` with `|delta_omega|`. Thus the controlled delay robustly changed robot travel and pose
gap, but did not produce a uniform controller-command increase across geometries.

## Geometry and geometric boundary results

G0 was genuinely straight: median tangent disagreement 0.00259 rad and essentially zero yaw
progression/lateral departure. G1 futures were curved, but their median entry tangent remained
parallel (0.00135 rad); two samples created the large angular command events. G2 futures had mild
yaw progression but did not reproduce the qualified lateral route change.

The measured heading-disagreement/`|delta_omega|` association was `r=0.673`, driven substantially
by the two G1 high-disagreement samples. G1 overall median `|delta_omega|` (0.0265 rad/s) exceeded
G0 (0.0183), but the evidence is heterogeneous rather than a stable turn-class effect. G2's
median was only 0.00174 rad/s.

Translation pose gap increased consistently with delay: mean L0→L1 was 0.0164→0.1736 m (G0),
0.0307→0.1674 m (G1), and 0.0156→0.2046 m (G2). Yaw pose gaps stayed near zero in typical
samples, with one large sample in each G1 latency cell. Translation pose gap and `|delta_v|`
had only `r=0.087`, so coordinate mismatch and immediate controller discontinuity are distinct
effects rather than interchangeable metrics.

`local_spatial_step_magnitude_mismatch_m` had overall mean/median 0.1161/0.1144 m. It is a
spatial-scale descriptor only, not velocity, acceleration, or execution discontinuity.

## Plots and deterministic representatives

The representative rule was frozen in code: nearest median of normalized `|delta_v| +
|delta_omega|`, with attempt index and path tie-breaks. Distribution plots show all moving
samples. Every command trace places the first FRESH command at exactly `t=0`.

Relative to the primary artifact root:

- `plots/robot_travel_by_latency_geometry.png`
- `plots/delta_v_by_cell.png`
- `plots/delta_omega_by_cell.png`
- `plots/effective_latency_vs_delta_v.png`
- `plots/effective_latency_vs_delta_omega.png`
- `plots/tangent_disagreement_vs_delta_omega.png`
- `plots/representative_controller_trace_<cell>.png` (six files)
- `plots/representative_transition_<cell>.png` (six files)

The transition plots separate OLD world path, raw observation-anchored FRESH, actual
observation→switch motion, observation pose, B, and FRESH row 0. `supplemental_analysis.json`
contains non-overwriting within-geometry L1-minus-L0 contrasts and primary geometry descriptors.

## Interpretation and evidence classification

### Positive evidence

- The +0.5 s manipulation reliably increased effective latency, OLD travel, and translation pose
  gap in all three geometries.
- Immediate `delta_v` increased strongly in G0; two delayed G2 samples also showed a large value.
- The two G1 samples with genuinely high entry-tangent disagreement produced the two largest
  moving-sample `delta_omega` values. This supports the controller-level relevance of angular
  mismatch when it is actually present.

### Negative evidence

- Added latency did not consistently increase `delta_v` within every geometry, and it increased
  typical `delta_omega` only slightly.
- G2 did not reproduce the strong lateral route-change found during qualification.
- Most curved G1/G2 futures began with a nearly straight/parallel entry, and the lookahead
  controller usually absorbed the raw reference change with a small immediate angular jump.

### Trade-off and experimental artifacts

- Pose gap and command discontinuity behaved differently: delay caused a large, stable pose-gap
  increase while immediate command jumps were sparse and output-dependent.
- LightNav produced repeated discrete outputs plus three real STOP responses. One high-geometry
  G1 sample per latency cell strongly affected means. These are retained model/protocol outcomes,
  not removed as outliers.
- Future curvature does not necessarily imply an immediate angular command jump because the
  controller uses the entry geometry and lookahead target at the switch.

### Implementation limitations and insufficient evidence

- G2 geometry qualification was not reproducible in the primary cohort, so a controlled
  route-change-class causal comparison was not achieved.
- Five samples per cell and repeated model outputs are insufficient for population inference.
- This run does not measure physical acceleration, navigation success, collision behavior,
  corrected execution, general LightNav/VLA behavior, or real robots.

## What can and cannot be claimed

In this tested LightNav–Isaac–Jackal stack, a controlled +0.5 s wait while OLD remained active
increased robot travel and observation-anchored translation pose mismatch. Immediate controller
discontinuity was heterogeneous: clear for G0 translation, rare but large for genuinely divergent
G1 entries, and usually small for G2. This is mixed evidence for latency-sensitive execution-level
reconciliation and limited positive evidence that yaw belongs in an SE(2) formulation when the
entry geometry truly disagrees.

It does not establish that all Navigation VLAs are discontinuous, that LightNav is defective,
that graph reconciliation solves anything, that EXP-02A improves execution, or that NavDP/real
robots behave similarly.

## Next decision

Before using a graph result as execution evidence, the smallest next experiment should make the
geometry manipulation reproducible across repeated model requests—for example, a fixed visual
intersection/route cue whose *primary* FRESH entry tangent is verified per trial—then compare raw
versus reconciled controller commands under the same L0/L1 timing. The current run already
justifies retaining latency and SE(2) pose/yaw in the backend interface, but it does not justify a
universal claim that the present controller always needs intervention.

## Reproduction

Run a new qualification, smoke cell, or full primary cohort (all outputs are exclusive):

```bash
cd ~/Workspace/se-3-reconciliation
./scripts/isaac/run_exp01b_controlled_latency.sh qualification
./scripts/isaac/run_exp01b_controlled_latency.sh smoke G0_straight__L0_natural
./scripts/isaac/run_exp01b_controlled_latency.sh primary
```

Validate an existing primary artifact without Isaac or LightNav:

```bash
.venv/bin/python scripts/summarize_exp01b_controlled_latency.py \
  data/exp01b_redesign/<run_id> --validate-only
```
