# DATA-01 Frozen Real LightNav OLD/FRESH Transition Bank

> **Status: `RETIRED_FROM_PRIMARY_FORMULATION_USE` (2026-09-09).** DATA-01 is a legacy
> EXP-01B-derived regression artifact. It must not be used as the primary formulation dataset
> or as the mandatory source for EXP-02D. Its source cohort was designed for controlled
> latency/G0-G2 characterization and lacked sufficient output-trajectory diversity. The
> generated local bank and EXP-01B-specific build tooling were removed by a forward commit;
> historical EXP-01B/EXP-02B/EXP-02C artifacts and the original DATA-01 commit remain intact.

## Why this dataset exists

DATA-01 fixes the real inputs for the next formulation stage before that formulation is
written. It is data curation, transition-context freezing, visualization, and a deterministic
development/held-out partition. It is not an optimizer experiment and contains no EXP-02D
residual, graph solve, factor-weight change, selector, controller tuning, new LightNav
inference, or Isaac physics execution.

The generated bank is ignored at:

```text
data/frozen_transition_bank/lightnav_exp01b_v1/
```

Versioned config, generator, validator, plotter, tests, and this document are committed. A
bank version is immutable: build fails if its output directory already exists, plotting fails
if plots were already generated, and validation never rewrites artifacts.

## Source cohort and strict inventory

The sole source of truth is:

```text
data/exp01b_redesign/exp01b-controlled-primary-20260906T-frozen/
```

The generator first calls the existing EXP-01B and Stage 0-F validators. It checks the five
frozen root hashes, source config hash, all per-attempt reconstruction rules, and 481 required
per-attempt artifact SHA-256/size/mtime records. The observed inventory is:

| Source classification | Count | Bank treatment |
| --- | ---: | --- |
| `VALID_MOVING` | 30 | included |
| `MODEL_STOP_OUTPUT` | 3 | excluded and counted |
| `OLD_EXHAUSTED` | 3 | excluded and counted |
| `TIMING_INVALID` | 1 | excluded and counted |
| total attempts | 37 | inventoried |
| timing-valid attempts | 36 | inventoried |

No count is manufactured to meet a target. Eligibility is the conjunction of timing-valid,
`VALID_MOVING`, and a non-STOP FRESH array. The frozen source recorded research commit
`4a44600088a6b260f32cc7f612d6cf125da9a91b`, LightNav commit
`a645828d81a8439651172197ca80a75dc1377977`, and checkpoint revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`.

## Why a pair is not a unique trajectory

The unit is one frozen transition context:

```text
T_i = (OLD_i, FRESH_i, P_i, B_i, observation context, timing context, provenance)
```

Identical arrays can occur under different boundaries, observation poses, latency, or source
trials. All 30 eligible contexts are retained. They contain 8 unique OLD raw arrays, 8 FRESH
raw arrays, 15 ordered raw-pair identities, 9 OLD world arrays, 16 FRESH world arrays, and 16
ordered world-pair identities. Duplicate identities are metadata for analysis and leakage
control, never a deduplication instruction.

`pair_<12 hex>` is SHA-256 over canonical JSON containing the source-relative trial path,
four array hashes, P, B, FRESH observation pose, observation time, and usable-switch time.
`display_index` is a separate stable source-path order for human browsing.

## Pair schema

Each `pairs/<pair_id>/` contains:

```text
raw/old_lightnav.npy
raw/fresh_lightnav.npy
derived/old_world.npy
derived/fresh_world.npy
context.json
source_provenance.json
descriptors.json
plots/world_xy.png
plots/boundary_canonical_xy.png
plots/yaw_vs_path_length.png
```

The four NPY files are byte-for-byte copies; their file SHA-256 values must equal their source
files. Raw LightNav arrays remain float32 `N x 3` absolute poses in their respective
observation-time robot frames. They are never transformed, normalized, re-anchored,
resampled, interpolated, or overwritten. World arrays remain the already-derived EXP-01B
Isaac world `[x_m, y_m, yaw_rad]` references and are also copied without alteration.

`context.json` records shapes, dtypes, observation/model-ready/usable poses and both simulation
and host times, geometry/latency cells, exact source fields, historical last OLD and first
FRESH desired commands, frames, axes, units, and split. `source_provenance.json` maps every
copy to its source-relative file and hash and records all relevant source artifact hashes plus
EXP-01B/LightNav/config identity. `descriptors.json` holds spatial geometry, transition
descriptors, frozen controller-switch values, ranks, and their provenance.

## P and B timing semantics

P and B are not inferred from the waypoint rows:

- `P` is `results/attempt.json.actual_pose_before_ready`. It exactly matches the frozen
  timeline's final control sample immediately before the usable boundary.
- `B` is `results/attempt.json.robot_pose_at_new_ready`. It must exactly equal
  `metadata.json.robot_pose_at_fresh_usable` and the `raw_switch` event pose.

The bank looks up and stores the unique matching timeline simulation/host timestamp for each
pose. It also preserves the separate FRESH observation, model-ready, and usable events. There
is no replay or reset in DATA-01.

## Descriptors and historical raw-switch metrics

OLD and FRESH use the already-tested Stage 0-F `geometry_descriptor`: pose count, spatial path
length, endpoint displacement, wrapped and cumulative pose-yaw progression,
pose-yaw-per-metre, XY-tangent curvature, tangent turn, sign changes, lateral departure, and
near-zero segment statistics. Pose yaw and XY tangent remain distinct.

The transition descriptor is specifically the frozen raw `k_fresh=0` context. It records
`||B-F0||`, wrapped B-to-F0 yaw, P→B and B→F0 world directions when their translations exceed
1 mm, their absolute angular disagreement, yaw-increment disagreement, and OLD/FRESH
endpoints relative to B. It does not predict an optimal k.

The desired-command discontinuities, short-window maxima/means, last OLD command, and first
FRESH command are copied semantically from each frozen
`results/controller_switch_metrics.json`; the validator checks them against that source. They
are not recomputed graph metrics. Deterministic one-based ranks and Q1–Q4 labels cover
`|delta v|`, `|delta omega|`, direction disagreement, yaw disagreement, effective simulation
latency, and observation-to-switch robot travel.

Observed raw distributions across 30 pairs are:

| Descriptor | Min | Median | Max |
| --- | ---: | ---: | ---: |
| raw `|delta v|` [m/s] | 0.002782 | 0.013260 | 0.416563 |
| raw `|delta omega|` [rad/s] | 0.001184 | 0.016439 | 1.405038 |
| P→B vs B→F0 direction disagreement [rad] | 0.754941 | 3.140092 | 3.141545 |
| yaw-increment disagreement [rad] | 0.000337 | 0.000812 | 0.226989 |
| effective simulation latency [s] | 0.450000 | 0.716667 | 1.000000 |

The near-pi direction statistic is literal: by switch time, many F0 positions lie behind B,
so B→F0 points opposite the immediately preceding P→B motion. It is an untimed spatial raw
boundary observation, not a controller failure label or an optimal-index conclusion.

## Visualization convention

`world_xy.png` uses the actual world frame:

- blue: OLD world trajectory;
- orange: FRESH world trajectory;
- red star: frozen raw-switch boundary B;
- black circle and arrow: previous pose P and P→B incoming execution direction;
- open blue square/orange diamond: OLD/FRESH observation pose;
- blue/orange x: trajectory endpoint.

Axes are equal and titles include pair ID, G/L cell, effective latency, raw `|delta v|`, and
raw `|delta omega|`. Different world frames are never combined on one axis; contact sheets
give every pair an independent subplot.

`boundary_canonical_xy.png` applies the same `T_B^-1` SE(2) transform to OLD, FRESH, P, and B,
mapping B to `[0,0,0]`. This happens only in memory while plotting. No canonical array is
stored as raw or derived data. Every title states “visualization-only B-centered frame.”

`yaw_vs_path_length.png` plots display-unwrapped yaw against cumulative spatial XY length.
The horizontal axis explicitly says “not time.” The source yaw array is unchanged.

The two all-pair contact sheets, development/held-out sheets, three geometry sheets, and two
latency sheets are under `overview/`. There are 90 per-pair plots and 9 overview plots. Visual
inspection covered the minimum and median combined raw discontinuity, maximum delta-v,
maximum delta-omega, maximum direction disagreement, a held-out example, and both all-pair
overviews; colors, P/B markers, arrow, equal aspect, canonical B location, spatial yaw axis,
titles, and clipping were correct.

## Development and held-out split

Leakage groups are the exact ordered `(OLD raw SHA-256, FRESH raw SHA-256)` identity. The 15
groups are assigned whole. With 15 groups, the generator exhaustively examines every
non-trivial assignment, minimizing in order: missing joint geometry/latency coverage,
distance from the one-third held-out pair target, normalized stratum-count error, and a
fixed-seed SHA-256 tie-break. No rigid, graph, EXP-02C, or future result is an input.

The frozen result is exactly 20 development and 10 held-out pairs with zero raw-pair-group
overlap. Both partitions contain every G/L stratum:

| Stratum | Development | Held-out |
| --- | ---: | ---: |
| G0 / L0 | 3 | 2 |
| G0 / L1 | 3 | 2 |
| G1 / L0 | 3 | 2 |
| G1 / L1 | 3 | 2 |
| G2 / L0 | 4 | 1 |
| G2 / L1 | 4 | 1 |

`split_manifest.json` is authoritative. If a future held-out result is inspected and then
used to change the formulation or weight, that partition must no longer be described as
truly held-out.

## Visualization representatives

`representatives.json` is explicitly not an evaluation subset. It uses only frozen raw
metrics and deterministic pair-ID tie-breaks:

| Role | Pair |
| --- | --- |
| minimum combined raw discontinuity | `pair_4a4d7f199b92` |
| median combined raw discontinuity | `pair_1d5f64ce0384` |
| highest absolute delta-v | `pair_a7703fd6b06d` |
| highest absolute delta-omega | `pair_6f8a42dd8306` |
| largest direction disagreement | `pair_4bd02d56c173` |
| largest yaw disagreement | `pair_6f8a42dd8306` |

One pair may fill multiple roles.

## Reproduction and validation

From the repository root:

```bash
.venv/bin/python scripts/build_frozen_transition_bank.py \
  --config configs/lightnav_transition_bank_v1.yaml
.venv/bin/python scripts/plot_frozen_transition_bank.py \
  data/frozen_transition_bank/lightnav_exp01b_v1
.venv/bin/python scripts/validate_frozen_transition_bank.py \
  data/frozen_transition_bank/lightnav_exp01b_v1
```

The validator reconstructs IDs, four copy/source hashes, source artifact manifest,
descriptors, source controller values, duplicate groups, ranks, deterministic split, plot
set/signatures/hashes, pair counts, partition union/disjointness, and no-leakage condition.
Use `--allow-missing-plots` only between build and plot phases.

## What this bank supports—and what it does not

It supports repeatable formulation comparisons on identical frozen real LightNav inputs,
source-auditable transition diagnostics, and an outcome-independent development/held-out
protocol. It establishes neither that a new graph is better nor that a universal hard/benign
threshold or optimal k exists. It supplies no obstacle-safety evidence, physical-execution
evidence, reconciliation improvement, or guarantee about future held-out performance.

## Current research-use rule

Do not rebuild or consume DATA-01 as the primary formulation source. DATA-02 replaces that
role with newly collected, genuine same-episode successive OLD/FRESH outputs from predeclared
diverse scenarios. The measurements above remain a historical record and may support an
explicitly labelled regression comparison only; they do not satisfy DATA-02 diversity or
held-out requirements.
