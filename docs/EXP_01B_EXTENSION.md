# EXP-01B Extension — Expanded Online Raw-Switch Characterization

## Research question

Across a larger, deliberately controlled cohort of timing-valid online LightNav transitions,
how frequently and under what OLD/FRESH execution geometries do translation and yaw pose/motion
inconsistencies appear when OLD continues during asynchronous FRESH inference?

This is characterization only. It implements no reconciliation, graph, correspondence,
spatial-entry selection, smoothing, or navigation-performance evaluation.

## Why the original N=3 is insufficient

The original immutable EXP-01B run, `data/exp01b/exp01b-20260903T155402Z/`, has three valid
transitions from one centered corridor protocol. It proves only that a measurable inconsistency
can occur. It cannot estimate a general occurrence rate, typical magnitude, or geometry
dependence. Its top-level `summary.json` and `metadata.json` SHA-256 values remained respectively
`4908f078...bf90` and `b3a27447...ec8c` before and after this extension.

## Original EXP-01B relationship

The extension reuses the same environment-separated architecture and semantics:

- one persistent, preloaded and explicitly warmed stateful LightNav process;
- one reset per episode, never between OLD and FRESH;
- Isaac physics and Stage 0-B feedback control continue during asynchronous FRESH inference;
- FRESH is transformed to world coordinates with the robot pose at FRESH observation, never
  the ready pose;
- FRESH row 0 is used directly, because the public LightNav action rows have no intrinsic
  waypoint time base; and
- raw actions, derived paths, actual execution, event/timing records, and hashes remain separate.

The old run and documentation remain the N=3 problem-existence pilot. This document and
`data/exp01b_extension/...` are a separate cohort namespace.

## Frozen condition design

The versioned `configs/exp01b_extension.yaml` was fixed before the primary collection. The
corridor is 3.0 m wide, so the ±0.15 m lateral and ±0.10 rad heading perturbations are
conservative relative to the walls.

| Condition | Initial `[x,y,yaw]` | FRESH observation delay | Intended controlled contrast |
|---|---:|---:|---|
| A nominal | `[0,0,0]` | 0.50 sim s | original centered protocol |
| B left recovery | `[0,+0.15,+0.10]` | 0.50 sim s | positive lateral/yaw perturbation |
| C right recovery | `[0,-0.15,-0.10]` | 0.50 sim s | negative lateral/yaw perturbation |
| D later observation | `[0,0,0]` | 0.75 sim s | more OLD progress before observation |

All conditions use `Go straight down the corridor.`, the existing controller gains and limits,
1/60 s physics, 0.10 s control, 4 Hz RGB, the same camera/checkpoint/backend, target valid N=6,
and maximum 10 attempts. No condition was changed after seeing output.

## Timing-validity protocol

The primary gate remains RTF `[0.90,1.10]`; ready must follow observation, the robot must move
more than 0.0001 m during inference, OLD progress may not regress, OLD may not exhaust, and the
response must remain within the configured wait. Classification is one of `VALID`,
`MODEL_STOP_OUTPUT`, `TIMING_INVALID`, `OLD_EXHAUSTED`, `NEW_TIMEOUT`, or
`OTHER_PROTOCOL_FAILURE`. A timing-valid FRESH STOP is retained and counted as valid, but is
also reported separately. The finite attempt cap was not extended when the target was missed.

## Data collection

The primary immutable generated run is:

```text
data/exp01b_extension/exp01b-extension-20260905T002500Z/
```

LightNav checkout was `a645828d81a8439651172197ca80a75dc1377977`, package 0.1.0;
checkpoint revision was `7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`; backend was
`vllm_local` 0.19.1 with prefix caching enabled, 0.65 GPU-memory utilization configuration,
and a 1 GiB explicit KV cache. The server built the model once (11,126 ms), warmed once from
the validated Stage 0-C history (660 ms host), and stayed alive for all 40 attempts. Isaac Sim
6.0.1 ran headless with camera rendering. No OOM or request timeout occurred.

The predeclared target was 24 valid transitions. The maximum 40 attempts yielded only 16:

| Condition | Attempts | Timing-valid | Valid moving FRESH | Valid FRESH STOP | OLD exhausted | RTF invalid | Other |
|---|---:|---:|---:|---:|---:|---:|---:|
| A | 10 | 5 | 3 | 2 | 4 | 1 | 0 |
| B | 10 | 4 | 3 | 1 | 5 | 1 | 0 |
| C | 10 | 3 | 3 | 0 | 6 | 1 | 0 |
| D | 10 | 4 | 2 | 2 | 6 | 0 | 0 |
| **Total** | **40** | **16** | **11** | **5** | **21** | **3** | **0** |

Every OLD-exhausted attempt had an all-zero OLD action array. These were preserved as actual
model outputs, but they cannot satisfy the required condition that OLD physically continues
during FRESH inference. Consequently, this run is validly collected and fully reconstructable,
but it did **not** meet the planned N=24 evidence target.

## STOP handling

Five of the 16 valid transitions returned all-zero FRESH. Their raw arrays were not replaced or
filtered. All five exceeded the 0.05 m translation-gap threshold and none exceeded the
translation-motion-jump threshold. They reflect OLD continuation plus a new STOP decision, so
they are not isolated evidence of pure geometric latency mismatch. Results below therefore
report both all valid transitions and the 11 moving-FRESH transitions.

## Metrics and geometry descriptors

Primary pose/motion metrics retain EXP-01B definitions and use actual execution at ready, not a
planned OLD pose. The incoming segment is the actual pose one control interval before ready to
the ready pose. FRESH first motion is world `F[0]→F[1]`. The fixed exploratory descriptors are:

- actual incoming magnitude and world tangent;
- FRESH first-segment magnitude and world tangent;
- wrapped absolute tangent disagreement; and
- absolute first-motion magnitude mismatch (plus a ratio when incoming motion is nonzero).

Zero-length vectors have an explicit undefined tangent (`null`), not an invented angle. These
descriptors are descriptive and are not causal features or training labels. The summary stores
mean, median, population standard deviation, min, max, p25, p75, and p90 (for N≥3), along with
every raw value.

## Aggregate results

All values below are timing-valid only. Ranges are min–max.

| Quantity | N | Mean | Median | Range |
|---|---:|---:|---:|---:|
| host request→response [s], all | 16 | 0.5020 | 0.4409 | 0.4315–0.7028 |
| simulation inference [s], all | 16 | 0.5156 | 0.4500 | 0.4500–0.7500 |
| RTF, all | 16 | 1.0272 | 1.0223 | 1.0076–1.0767 |
| robot translation observation→ready [m], all | 16 | 0.1859 | 0.1694 | 0.1645–0.2429 |
| absolute robot yaw observation→ready [rad], all | 16 | 0.000061 | 0.000046 | 0.000012–0.000146 |
| translation gap [m], all | 16 | 0.08065 | 0.02395 | 0.00965–0.24120 |
| translation-motion jump [m], all | 16 | 0.08349 | 0.10145 | 0.02318–0.11319 |
| yaw gap [rad], all | 16 | 0.001925 | 0.000550 | 0.000012–0.004164 |
| yaw-motion jump [rad], all | 16 | 0.002986 | 0.000107 | 0.000002–0.006785 |
| translation gap [m], moving FRESH | 11 | 0.02908 | 0.01499 | 0.00965–0.08808 |
| translation-motion jump [m], moving FRESH | 11 | 0.10640 | 0.10699 | 0.09946–0.11319 |
| yaw gap [rad], moving FRESH | 11 | 0.002778 | 0.004053 | 0.000107–0.004164 |
| yaw-motion jump [rad], moving FRESH | 11 | 0.004340 | 0.006742 | 0.000075–0.006785 |

For all 16 valid transitions, translation gap exceeded 0.05 m in 7/16 (43.75%), translation
motion jump in 11/16 (68.75%), and the descriptive combined translation indicator in 16/16.
For moving FRESH only, the respective counts were 2/11, 11/11, and 11/11. No yaw gap or
yaw-motion jump exceeded 0.05 rad in either subset.

## Per-condition results

These small condition Ns are descriptive, not frequency estimates.

| Condition | Translation gap mean / median / range [m] | Translation-motion jump mean / median / range [m] | Gap >.05 | Jump >.05 |
|---|---|---|---:|---:|
| A, N=5 | .07447 / .01499 / .00965–.16903 | .07708 / .09959 / .03761–.10699 | 2/5 | 3/5 |
| B, N=4 | .05704 / .02171 / .01496–.16978 | .09221 / .10796 / .03974–.11319 | 1/4 | 3/4 |
| C, N=3 | .01552 / .01489 / .01155–.02011 | .10362 / .10130 / .09946–.11008 | 0/3 | 3/3 |
| D, N=4 | .16083 / .15702 / .08808–.24120 | .06769 / .06822 / .02318–.11114 | 4/4 | 2/4 |

Moving-FRESH-only translation gap exceeded threshold only in D (2/2); motion jump exceeded it
in every moving sample: A 3/3, B 3/3, C 3/3, D 2/2. All per-condition yaw threshold counts
were zero. D had longer request/response latency (mean 0.6831 s) and robot motion (mean 0.2383
m) than A/B/C (host means 0.4383/0.4460/0.4414 s; motion means
0.1663/0.1710/0.1686 m). This is an observed association, not a causal trigger-delay claim.

## Geometry descriptors

Across 11 moving FRESH samples, actual incoming segments were 0.0281–0.0428 m and first FRESH
segments were 0.1392–0.1511 m. Their magnitude mismatch was 0.0995–0.1132 m, numerically equal
to the translation-motion jump under these definitions. Initial tangent disagreement was only
0.00017–0.00261 rad. Thus the ±offset/yaw initial conditions changed global starting geometry
but did not make LightNav produce a meaningful left/right recovery disagreement; OLD and FRESH
remained nearly parallel. This is negative evidence against calling B/C steering-recovery
samples and limits geometry-dependent interpretation.

For moving FRESH only, descriptive Pearson coefficients were 0.997 between robot motion during
inference and translation gap, 0.402 between robot motion and translation-motion jump, -0.648
between tangent disagreement and translation-motion jump, and 0.995 between tangent disagreement
and yaw-motion jump. N=11, repeated/near-identical model outputs, the two D samples, and the very
small tangent range make these unstable exploratory summaries; they do not establish causality.

## Representative trajectories and plots

The representative rule is automatic: among valid non-STOP samples in each condition, choose
the translation-motion jump nearest that condition's median, then break ties by attempt index
and artifact path. The selected attempts are A/000, B/009, C/009, and D/003.

- `plots/translation_metrics_by_condition.png`
- `plots/yaw_metrics_by_condition.png`
- `plots/inference_motion_vs_discontinuity.png`
- `plots/heading_disagreement_vs_jump.png`
- `plots/representative_inference_motion_condition_A_nominal.png`
- `plots/representative_inference_motion_condition_B_left_recovery.png`
- `plots/representative_inference_motion_condition_C_right_recovery.png`
- `plots/representative_inference_motion_condition_D_later_observation.png`

All paths are relative to the primary run directory above. Individual samples remain visible in
the distribution plots; STOP uses a separate marker/color. The representative overlays separate
planned OLD, actual observation-to-ready motion, raw FRESH, observation pose, and ready pose.

## Positive evidence

- Translational inconsistency was repeatable beyond the original N=3: every one of 11 valid
  moving-FRESH transitions exceeded the local translation-motion-jump threshold.
- The moving-sample motion jump appeared in A, B, C, and D under the frozen conditions.
- D's two moving samples showed both pose gap (0.0881 m) and motion jump (0.1111 m), while
  nominal/offset moving samples mainly showed motion jump rather than pose gap.

## Negative evidence

- No valid transition exceeded either yaw threshold; yaw was not meaningful in this cohort.
- B/C did not realize the intended nontrivial turning/tangent disagreement.
- Only 2/11 moving samples exceeded the translation pose-gap threshold.

## Heterogeneity

STOP changes the metric pattern: it produces large pose gap but small motion jump. Moving FRESH
produces a consistently large first-motion mismatch but usually a small pose gap. A robot moving
during inference therefore does not by itself determine which discontinuity component dominates.

## Limitations

- The cohort missed its target (16 valid rather than 24) because 21 attempts produced zero OLD
  motion and three missed the unchanged RTF gate. It is insufficient for the planned expanded
  frequency claim.
- Condition-level N is only 3–5, with just 11 moving-FRESH samples. Percentages are descriptive.
- One corridor, instruction, checkpoint, simulator, GPU/backend, and narrow perturbation scale
  were used. Prefix caching remained enabled as in the validated stack.
- Left/right initial offsets did not yield meaningful recovery steering. A future protocol must
  prevalidate geometric diversity without selecting conditions by desired discontinuity outcome.
- No collision, navigation success, corrected execution, real robot, or general VLA behavior was
  evaluated.

## What we can claim

Under this measured warmed LightNav + Isaac setup, OLD continued moving during 16 timing-valid
FRESH inferences, and translation pose and/or local-motion inconsistency recurred in four
controlled initial/protocol conditions. In the 11 moving-FRESH cases, the dominant observed
component was first-motion magnitude jump, not yaw.

## What we cannot claim

This run cannot provide a representative occurrence rate or establish broad geometry dependence,
because it missed N=24 and did not realize turn-diverse OLD/FRESH tangents. It does not show that
LightNav is defective, that all VLAs exhibit the issue, that navigation performance degrades, or
that EXP-02A solves it.

## Relevance to EXP-02A

The immutable valid moving attempts are technically consumable by EXP-02A/later offline
evaluation through raw hashes and separated OLD/FRESH/actual artifacts. They add repeated
translation-motion-jump cases and later-observation pose-gap cases. They are not yet a sufficiently
diverse evaluation cohort for claims about a k-conditioned formulation. The next data step should
address moving-OLD yield and deliberately verify turn/straight tangent diversity in a new,
predeclared extension—not retrospectively change this run.
