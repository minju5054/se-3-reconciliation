# EXP-02A — Z-conditioned Spatial-Entry Transition Reconciliation

## Research question

Given fixed OLD execution, a FRESH world trajectory, and pre-treatment context Z containing a
chosen spatial entry index `k`, can a minimal SE(2) graph reduce motion discontinuity from the
committed boundary into `FRESH[k:]` without erasing that selected entry or deforming the suffix?

This is an offline formulation experiment. It is not an OLD/FRESH pose-correspondence or
temporal-delay experiment. Every k branch starts from the same committed pose B; selecting a
larger k never executes OLD for more time.

## Relationship to the previous EXP-02

The previous EXP-02 is preserved as a graph-mathematics pilot. It assumed collaborator input
`Z_ij` was the SE(2) measurement `O_i^-1 X_j`. That assumption is obsolete and no serialized
oracle annotation is silently reinterpreted.

Collaborator-level Z now means a structured `SpatialEntryContext`:

```yaml
entry_index: 3
evidence:
  candidate_feature: 0.18
evidence_status:
  candidate_feature: candidate
source: oracle_exp02a
metadata: {}
```

Evidence accepts nested JSON-compatible optional fields, rejects non-finite numbers, is copied
into every result, and round-trips exactly. It is absent from residual construction and graph
weights. EXP-02A validates a backend *conditioned on* Z; it does not validate the scientific
meaning of an evidence feature or a WHICH-k selector.

## Input and fixed-data semantics

`TransitionReconciliationInput` contains:

- `old_poses_world`: fixed arbitrary `M x 3` planned OLD poses;
- `fresh_poses_world`: unchanged arbitrary `N x 3` FRESH poses;
- `committed_pose_world`: fixed current/ready pose B;
- `entry_context`: `SpatialEntryContext(entry_index=k, evidence, provenance)`;
- `actual_pose_before_committed`: optional measured pose immediately before B;
- metadata.

All poses are `[x, y, yaw]` in world SE(2), meters/meters/radians, +y left, and yaw positive
counter-clockwise. Measured execution is preferred for the incoming-motion calculation and is
never silently replaced by planned OLD. The planned relation `O_(m-1)^-1 B` is also retained
and reported. OLD, measured execution, full FRESH, and selected suffix remain separate.

Only `FRESH[k:]` is copied into the graph as variables. `FRESH[:k]` is never optimized or
executed. EXP-02A rejects `k=N-1` explicitly because its transition-tangent and suffix-motion
evaluation require at least one following pose; `k=0` and arbitrary other valid lengths work.

## Candidate formulations

Every optimized variant retains the complete FRESH relative-motion chain:

```text
D_j^F = F_j^-1 F_(j+1)
r_j^F = Log[(D_j^F)^-1 (X_j^-1 X_(j+1))],  j = k ... N-2
```

The predeclared residual scales are 0.10 m, 0.10 rad, and 0.10 for the dimensionless transition
vector. All active factor weights are 1.0. These are mechanism-pilot settings, not universal
hyperparameters.

### Raw

`FRESH[k:]` is evaluated without optimization.

### Pose anchor

The historical diagnostic baseline uses:

```text
r_pose = Log(B^-1 X_k)
```

plus the FRESH-motion chain. It tests whether minimizing pose gap destroys spatial k meaning.

### Entry preservation

This baseline uses:

```text
r_entry = Log(F_k^-1 X_k)
```

plus the FRESH-motion chain. Its raw initialization already has zero cost, so it preserves k
exactly but does not improve transition motion.

### Incoming-motion-aware transition

Let P be the measured pose immediately before B (or the planned OLD tail only when P is absent),
and let `u_old` be the unit world-plane direction P→B. Let:

```text
d_k = ||F_k.xy - B.xy||
q(X_k) = (X_k.xy - B.xy) / d_k
```

The direction/entry-radius residual is the two-component dimensionless vector:

```text
r_dir = q(X_k) - u_old
```

It asks B→X_k to have the incoming direction and the *selected raw entry radius* `d_k`. It does
not set transition length equal to one OLD execution sample. Unlike `atan2` or normalization by
the current candidate length, it remains finite when a numerical update crosses B and it
distinguishes parallel from antiparallel motion. The fixed raw `d_k` and incoming translation
must exceed `1e-6 m`; otherwise direction is explicitly undefined and the candidate is rejected.

The yaw-motion residual is:

```text
r_yaw = wrap(yaw(B^-1 X_k) - yaw(P^-1 B))
```

This variant combines `r_dir`, `r_yaw`, `r_entry`, and the complete FRESH-motion chain. It does
not use an OLD/FRESH correspondence or any Z evidence value.

## Metrics

Transition pose metrics report B→entry translation and yaw descriptively; distance alone is not
treated as bad because k is spatial. Motion metrics report incoming and transition directions,
their wrapped difference, transition-to-first-FRESH tangent difference, incoming/transition
translation-magnitude difference, and yaw-motion jump. Entry displacement is
`Log(F_k^-1 X_k)`. Edge deformation uses the SE(2) FRESH residual above. Per-pose
`Log(F_j^-1 X_j)` provides the downstream correction profile.

For every k pair, inter-k metrics record raw/optimized entry separation, first-transition
heading difference, mean separation over the first three suffix poses, and retention ratios.
A direction is reported as undefined—not fabricated—when pose anchoring collapses an entry to B.

## Actual setup

Validated immutable run: `data/exp02a/exp02a-20260904T162149Z/` (ignored by Git).

Synthetic data uses OLD poses `x=[-0.60,-0.40,-0.20] m`, B=`[0,0,0]`, and a ten-pose curved
FRESH path starting at `[0.18,0.12,0.10]` with repeated local motion
`[0.17,0,0.055]`. The controlled entries are early/middle/late k=`0/3/6`. Evidence fields
`old_fresh_initial_tangent_divergence_rad` and `shortcut_pressure` are labelled synthetic
candidates only.

The real backend pilot references timing-valid, non-stop EXP-01B
`exp01b-20260903T155402Z/trial_001`. OLD/FRESH shapes are `(10,3)`. Raw action hashes remain:

- OLD: `8edd60bf794d067dba1820f557e39388a8c3af352db7443774108f550219cf06`
- FRESH: `072c29e991bea91fad89d00ad458c00697960dc684bbdfc68cf105e3ace362fe`

The real k values `0/3/6` were fixed in configuration before graph execution: first returned
entry, middle moving entry before terminal slowdown, and later terminal-approach entry. They
are manual backend probes, not LightNav labels and not tuned by output appearance.

## Synthetic results

The table shows direction jump / translation-motion jump / yaw-motion jump, followed by entry
displacement and FRESH translation/yaw edge-distortion RMS.

| k | variant | transition motion [rad / m / rad] | entry displacement [m] | deformation RMS [m / rad] |
|---:|---|---|---:|---|
| 0 | raw or entry preservation | 0.588003 / 0.016333 / 0.100000 | ~0 | ~0 / 0 |
| 0 | pose anchor | undefined / 0.200000 / ~0 | 0.216423 | ~0 / ~0 |
| 0 | incoming aware | 0.024987 / 0.014775 / 0.050030 | 0.119786 | `3.34e-15 / 5.37e-15` |
| 3 | raw or entry preservation | 0.282898 / 0.511666 / 0.265000 | ~0 | ~0 / 0 |
| 3 | pose anchor | undefined / 0.200000 / ~0 | 0.713753 | ~0 / ~0 |
| 3 | incoming aware | 0.094924 / 0.505322 / 0.132598 | 0.133230 | `4.90e-16 / 0` |
| 6 | raw or entry preservation | 0.298377 / 1.020948 / 0.430000 | ~0 | ~0 / 0 |
| 6 | pose anchor | undefined / 0.200000 / ~0 | 1.230405 | ~0 / ~0 |
| 6 | incoming aware | 0.179070 / 1.007925 / 0.215190 | 0.145667 | `2.15e-15 / 2.56e-16` |

All incoming-aware optimizations converged in 5–6 iterations and decreased cost. They improved
direction and approximately halved yaw jump at every k, with modest translation-jump changes.
They did so through an almost exact rigid SE(2) transformation of each selected suffix, so the
complete relative-motion chain propagated correction to every downstream pose without an edge
kink. Entry motion was substantial (0.120–0.146 m), exposing the trade-off rather than proving
the weights are ideal.

Synthetic entry-separation retention for incoming-aware output was `0.964–0.998`; first-three-
pose separation retention was `0.965–1.002`. Pose anchoring reduced entry-separation retention
to `1.3e-13–4.2e-13`, demonstrating collapse. Entry preservation retained exactly 1.0.

## Real LightNav pilot results

Raw EXP-01B k=0 used an actual incoming motion of `0.038407 m`; the raw FRESH entry was only
`0.003854 m` from B but lay almost antiparallel to incoming motion. Larger k values are spatially
farther, as intended.

| k | variant | transition direction jump [rad] | translation-motion jump [m] | yaw-motion jump [rad] | entry displacement [m] | deformation RMS [m / rad] |
|---:|---|---:|---:|---:|---:|---|
| 0 | raw / entry | 3.121188 | 0.034553 | 0.004168 | ~0 | ~0 / 0 |
| 0 | pose anchor | undefined | 0.038407 | `3.85e-6` | 0.003854 | `3.94e-11 / 4.02e-11` |
| 0 | incoming aware | `3.03e-7` | 0.034553 | 0.002084 | 0.007708 | `3.67e-14 / 8.44e-14` |
| 3 | raw / entry | 0.009687 | 0.419639 | 0.006455 | ~0 | ~0 / 0 |
| 3 | pose anchor | undefined | 0.038407 | `3.85e-6` | 0.458047 | `2.66e-14 / 4.05e-16` |
| 3 | incoming aware | 0.001680 | 0.419636 | 0.003228 | 0.003668 | `2.40e-15 / 1.92e-14` |
| 6 | raw / entry | 0.011247 | 0.519463 | 0.001869 | ~0 | ~0 / 0 |
| 6 | pose anchor | undefined | 0.038407 | `3.85e-6` | 0.557871 | `2.49e-15 / 0` |
| 6 | incoming aware | 0.002669 | 0.519457 | 0.000935 | 0.004785 | `1.35e-16 / 5.14e-15` |

Incoming-aware optimizations converged in three iterations. Cost changed
`399.960103→0.006811` (k=0), `0.013551→0.003711` (k=3), and
`0.012999→0.003177` (k=6). The apparently large k=0 initial cost correctly detects the
antiparallel transition that a sine-only direction metric would miss.

Downstream translation-correction magnitudes were:

- k=0: `0.007708 ... 0.007798 m` across all ten suffix poses;
- k=3: `0.003668 ... 0.003999 m` across all seven suffix poses;
- k=6: `0.004785 ... 0.004788 m` across all four suffix poses.

Yaw correction was essentially constant within each suffix (`0.002084`, `0.003228`, and
`0.000935 rad`). Relative-motion distortion remained numerical, so correction reached every
non-entry pose with no geometric cutoff. This does not mean the transition is dynamically
executable; no online execution was performed.

Real incoming-aware entry-separation retention was `0.9833–0.99995`; first-three-pose retention
was `0.9793–0.99988`. Pose anchoring collapsed entry separation to about
`3.5e-13–1.2e-10` of raw. Entry preservation retained exactly 1.0.

## Ablation interpretation

- **Raw:** preserves k and FRESH but exposes the original transition.
- **Pose anchor + FRESH motion:** makes every first pose B and preserves each suffix rigidly;
  it removes pose gap/yaw but destroys entry separation and makes B→entry direction undefined.
- **Entry preservation + FRESH motion:** exactly returns raw input. It proves preservation but
  supplies no transition correction.
- **Incoming motion + entry + FRESH motion:** OLD incoming direction matters: it corrects the
  real k=0 antiparallel case and reduces direction/yaw jumps for all tested k while retaining
  96–100% of inter-k separation. It does not materially reduce the large translation-magnitude
  jumps for real k=3/6 because the formulation deliberately does not equate a spatial entry
  distance with one OLD sample length.

Thus EXP-02A gives positive mechanism evidence for direction/yaw continuity and k preservation,
negative evidence that this minimal objective solves translation-magnitude discontinuity, and
clear negative evidence for pose anchoring as the official spatial-entry formulation.

## Limitations and claim boundary

Synthetic results validate deterministic implementation, not research evidence. The LightNav
result is one manually probed straight-corridor pair from a three-valid-transition
problem-existence pilot. It cannot estimate occurrence rate, generality, navigation quality,
collision avoidance, selector correctness, feature validity, NavDP behavior, online execution,
or real-robot performance. No evidence feature predicts k here. No best k or universal weights
are selected.

The largest formulation uncertainty is how to improve translation-magnitude continuity without
collapsing a spatially meaningful distant entry or inventing a time semantics. A second
uncertainty is whether a non-rigid correction should ever be preferred to the observed rigid
suffix propagation.

What can be claimed is limited to: for manually specified k, the tested incoming-motion-aware
SE(2) formulation reduced direction/yaw inconsistency while preserving suffix relative motion
and inter-k spatial distinction in the tested synthetic and one LightNav pilot pair. It did not
solve the large spatial transition-length mismatch.

## Next decision

Do not freeze this formulation as a complete transition solution yet. Next, scope the
translation-magnitude trade-off while retaining this k-preservation test. In parallel, the
documented `EXP-01B-extension` should collect a larger, geometrically diverse, timing-valid
cohort spanning initial states, straight/turn geometry, and disagreement magnitudes. Only then
should development/evaluation pairs be separated and a selector or evidence-to-k method be
evaluated.

## Reproduction

```bash
cd ~/Workspace/se-3-reconciliation
.venv/bin/python scripts/run_exp02a_spatial_entry.py
```

Validate an immutable output:

```bash
.venv/bin/python scripts/summarize_exp02a.py data/exp02a/<run_id>
```
