# Projection-based robotless handoff geometry

Decision: **ROBOTLESS_PROJECTION_HANDOFF_GEOMETRY_VALIDATED**.

For the fixed saved FRESH trajectory, the four prescribed boundaries have
consistent continuous projections, distances, tangent-direction residuals,
pose-yaw residuals and arc-length progress. The actual Isaac overview, four
detail views and four offline plots were inspected. No new inference occurred.

The first three conditions project onto FRESH's first endpoint. At tau=1 s the
projection moves into segment 0, alpha=0.6622355452892925. Cross-track distance
decreases across the sampled conditions, tangent-direction mismatch stays
constant, and interpolated pose-yaw mismatch increases at the last condition.
The unchanged source does not produce a large direction discrepancy; no source,
motion, threshold or score was selected to manufacture one.

## Research question and frozen source

Where is the continuous closest point on FRESH for each controlled boundary,
and how do spatial distance, incoming/tangent direction difference,
boundary/interpolated-pose orientation difference and along-path progress vary?

Source: `data/robotless_controlled_staleness/20260914T082941Z/`, described in the
[previous controlled-staleness report](ROBOTLESS_CONTROLLED_STALENESS_CHARACTERIZATION.md).
The existing source validator passed before any derived output was created and
passed again in the final audit. All **37** controlled-run files and all **54**
transitive successive-run files were verified unchanged, including the exact
sets of filenames. The transitive source is
`data/robotless_successive_chunks/20260914T073030Z/`.

| Frozen input | SHA-256 |
| --- | --- |
| Controlled `derived/boundaries.npy` | `73677a09c4b847829a719b5ba24cca189d2e7a72dc485997ee76d7086d88e900` |
| Successive `raw/chunk_000.npy` (OLD) | `928cbb6935b565a5cc059216a61d4c3ee38a003eef7b6787b6e8dc115a48d6ce` |
| Successive `raw/chunk_001.npy` (FRESH) | `cc0b216983f36c2b938ee10e0ac839ed3c4c381e84ef136a48a39f25373a491d` |
| Successive `derived/chunk_000_world.npy` | `c23982987ccef88654264c74a0c20aa3e7c838bb5f863bfa60cbcd89c981c42f` |
| Successive `derived/chunk_001_world.npy` | `2c44792f4f4fd2725a45176c14c87f02ee812d28b1a56b689eeb95b08ee1cd37` |

The task started at `b27ed41c2d5525b1bd686b082a9b160f09a26941`, also confirmed
as `origin/main` after a successful fetch. It is the reviewed controlled-source
implementation. The earlier successive implementation is
`913bf235806b769cfe81cbef233be66a7b1be7a8`; original capture-time research SHA is
`1572ea407bc26b6cc5ff94f967ed12316de729e3`. These provenance events remain distinct.

## Frames, motion and time

All metric XY positions are in the unchanged right-handed Isaac world, metres,
+Z up; yaw and directions are CCW about +Z, in radians. Agent +X is forward,
+Y left and +Z up. Both source chunks have 10 cumulative local poses. We load
the already saved world arrays directly, whose source transform was:

```text
T_world_F_j = T_world_R_obs * T_R_obs_F_j
R_obs = [19.0, 27.0, 1.5707963267948963]
```

No new trajectory transform, re-integration or reanchoring is performed. The
saved boundaries, also loaded directly, came from the previous configuration:

```text
v = 0.25 m/s; omega = 0 rad/s
tau = [0.0, 0.2, 0.5, 1.0] s
B(tau) = R_obs * [v*tau, 0, 0]
phi_in = yaw(B)
```

The precise meaning of phi_in is **configured controlled body-forward motion
direction**. Positive v and zero omega are required. It is neither measured
robot velocity nor an OLD follower command. OLD is visualization context only.

Tau is controlled delay, not measured LightNav latency, actual execution time,
waypoint time or process waiting. The original FRESH observation UTC remains
`2026-09-14T07:36:52.886816Z`; its monotonic and simulation timestamps are retained
from the source. Original readiness is available through the source provenance;
new-inference readiness and actual execution are explicitly null. Analysis
creation UTC is `2026-09-14T09:53:03.396461Z`; viewport capture completion is
`2026-09-14T09:54:09.603053Z`. Their monotonic timestamps are saved separately.
The Isaac timeline stays stopped at 0 s and the logical agent stays at R_obs.
The four B markers represent controlled counterfactual conditions.

## Definitions and implementation

For each segment d_j = F_(j+1).xy - F_j.xy:

```text
alpha_j = clip(dot(B.xy - F_j.xy, d_j) / dot(d_j, d_j), 0, 1)
Q_j = F_j.xy + alpha_j * d_j
(j, alpha, Q) = candidate with smallest ||B.xy - Q_j||
e_perp = ||B.xy - Q||
```

The implementation reuses the previous `point_to_polyline` helper. Exact
distance ties select the lowest segment index. A zero-length segment is a
point, with alpha=0. `projection_is_interior` is true only for a nonzero winning
segment with 0<alpha<1. Endpoint and degenerate-point locations are explicit.
Here cross-track means the unsigned minimum segment distance, including
clamped endpoint cases; it is not always perpendicular distance to an infinite
line and is not a signed lateral error.

```text
L_j = ||d_j||
s_Q = sum(L_m for m < j) + alpha * L_j
normalized_progress = s_Q / sum(L_m)
phi_F = atan2(d_j.y, d_j.x)
e_dir = wrap(phi_F - phi_in)
theta_Q = wrap(yaw(F_j) + alpha * wrap(yaw(F_(j+1)) - yaw(F_j)))
e_yaw = wrap(theta_Q - yaw(B))
```

`wrap` reuses the SE(2) utility, mapping to [-pi,pi). Tangent phi_F and pose yaw
theta_Q are separate quantities. Both signed residuals, absolute radians and
absolute degrees are saved. A degenerate winner has null tangent/direction
metrics and an explicit reason; no adjacent tangent is substituted. Its yaw
uses the first endpoint under alpha=0. Zero total XY arc length, including a
singleton path, is explicitly rejected because normalized progress is undefined.

The artifact validator recomputes all metrics from the frozen inputs and
requires e_perp to match previous d_poly within absolute tolerance **1e-12 m**.
This is a numerical consistency tolerance, not a handoff threshold. Previous
d_entry and d_poly remain reference fields. No total score, gate, correspondence
factor, correction or optimization is added.

## Actual values

Generated run:
[`data/robotless_projection_handoff/20260914T095303Z/`](../data/robotless_projection_handoff/20260914T095303Z/).
[JSON](../data/robotless_projection_handoff/20260914T095303Z/derived/projection_metrics.json)
and [CSV](../data/robotless_projection_handoff/20260914T095303Z/derived/projection_metrics.csv)
retain full precision; the tables below are rounded.

Every B has yaw **1.5707963267948963 rad**. Every winning segment is **j=0**
and has an available tangent. Segment 0 length is **0.15037044184592332 m**;
total FRESH XY arc length is **1.3550827435038912 m**.

| tau (s) | B.xy (m) | alpha | Q.x (m) | Q.y (m) | Location |
| --- | --- | --- | --- | --- | --- |
| 0.0 | (19, 27) | 0 | 18.999944041196 | 27.150419220328 | start endpoint |
| 0.2 | (19, 27.05) | 0 | 18.999944041196 | 27.150419220328 | start endpoint |
| 0.5 | (19, 27.125) | 0 | 18.999944041196 | 27.150419220328 | start endpoint |
| 1.0 | (19, 27.25) | 0.6622355452892925 | 18.999830742795 | 27.249999807427 | interior |

| tau (s) | e_perp (m) | Previous d_poly (m) | Previous d_entry (m) | s_Q (m) | Normalized progress |
| --- | --- | --- | --- | --- | --- |
| 0.0 | 0.150419230737 | 0.150419230737 | 0.150419230737 | 0 | 0 |
| 0.2 | 0.100419235920 | 0.100419235920 | 0.100419235920 | 0 | 0 |
| 0.5 | 0.025419281923 | 0.025419281923 | 0.025419281923 | 0 | 0 |
| 1.0 | 0.000169257314 | 0.000169257314 | 0.099580795395 | 0.099580651551 | 0.073486768265 |

The stored e_perp-minus-previous-d_poly difference is **exactly zero** for all
four conditions. At tau=1 the projection is 7.348676826% along FRESH by XY arc
length, with a 0.169257314 mm B-Q distance. Alpha was computed from source
geometry, never hard-coded.

| tau (s) | phi_in (rad) | phi_F (rad) | Signed e_dir (rad) | abs_e_dir (rad) | abs_e_dir (deg) |
| --- | --- | --- | --- | --- | --- |
| 0.0 | 1.570796326795 | 1.571934082202 | +0.001137755407 | 0.001137755407 | 0.065188582957 |
| 0.2 | 1.570796326795 | 1.571934082202 | +0.001137755407 | 0.001137755407 | 0.065188582957 |
| 0.5 | 1.570796326795 | 1.571934082202 | +0.001137755407 | 0.001137755407 | 0.065188582957 |
| 1.0 | 1.570796326795 | 1.571934082202 | +0.001137755407 | 0.001137755407 | 0.065188582957 |

| tau (s) | theta_Q (rad) | Signed e_yaw (rad) | abs_e_yaw (rad) | abs_e_yaw (deg) |
| --- | --- | --- | --- | --- |
| 0.0 | 1.571397287424 | +0.000600960630 | 0.000600960630 | 0.034432507725 |
| 0.2 | 1.571397287424 | +0.000600960630 | 0.000600960630 | 0.034432507725 |
| 0.5 | 1.571397287424 | +0.000600960630 | 0.000600960630 | 0.034432507725 |
| 1.0 | 1.573384062807 | +0.002587736012 | 0.002587736012 | 0.148266351998 |

Direction residual remains constant because the winning segment and boundary
yaw are unchanged. Pose-yaw residual changes when alpha moves into that segment,
because endpoint pose yaws differ. This is why tangent and pose orientation
cannot be merged. The first three endpoint projections have zero along-path
progress despite decreasing distance. The fourth is close to the path while
being about 0.09958 m from its entry.

## Actual Isaac views and plots

Isaac 6.0.1 loaded the unchanged Hospital asset. The inventory has 1,936 prims,
126 collision prims, zero robot paths/articulations/rigid bodies/physics scenes,
and no advanced dynamics. The agent pose was unchanged before/after capture.

| Display | Meaning |
| --- | --- |
| Magenta path | Fixed FRESH world polyline from its original F_0 |
| Blue path | Original OLD world polyline, context only |
| Yellow point | Selected B; all four B positions in overview |
| White point | Closest continuous Q; first three Q coincide |
| Green connector | Straight B.xy to Q.xy distance |
| Yellow arrow | Configured controlled incoming direction at B |
| Cyan arrow | Geometric FRESH tangent phi_F at Q |
| Orange arrow | Interpolated FRESH pose yaw theta_Q at Q |
| Thin vertical stems | Display-height layers sharing the same B/Q XY origin |

The [overview](../data/robotless_projection_handoff/20260914T095303Z/evidence/isaac_projection_overview.png)
shows the complete FRESH, OLD context and all B/Q positions. Detail views for
[tau=0](../data/robotless_projection_handoff/20260914T095303Z/evidence/tau_000_detail.png),
[tau=.2](../data/robotless_projection_handoff/20260914T095303Z/evidence/tau_001_detail.png),
[tau=.5](../data/robotless_projection_handoff/20260914T095303Z/evidence/tau_002_detail.png)
and [tau=1](../data/robotless_projection_handoff/20260914T095303Z/evidence/tau_003_detail.png)
show all three direction arrows. Detail cameras are centered using each B's
frame; only the display camera changes, never source geometry or agent pose.

Paths are drawn at world Z=.12 m and B/Q points/connectors at .13 m. Incoming,
tangent and pose-yaw arrows use .16/.22/.28 m heights and .12/.18/.15 m lengths,
respectively. Their lengths are display choices, not velocity magnitudes. Thin
vertical stems identify the common XY origins of layered arrows. Camera XY
offsets use the existing observation-to-world utility with the overview R_obs
or selected detail B; camera Z values are absolute display heights. All values
are in the frozen configuration. No XY scaling, angular magnification or scene
hiding is applied, and no observation-to-FRESH-entry segment is added to FRESH.

The tau=1 B/Q centers and connector are below reliable screenshot resolution.
The tiny angle differences also cannot be measured visually in these images.
The separate layers make the quantities visible without exaggerating them;
the numerical audit establishes their values. The interactive UI provides an
overview button, four tau buttons and a color legend. Viewport PNGs contain
geometry only, so this report supplies their legend.

Four plots read the saved metric JSON:
[cross-track](../data/robotless_projection_handoff/20260914T095303Z/evidence/tau_vs_cross_track.png),
[direction](../data/robotless_projection_handoff/20260914T095303Z/evidence/tau_vs_direction.png),
[pose yaw](../data/robotless_projection_handoff/20260914T095303Z/evidence/tau_vs_yaw.png),
and [progress](../data/robotless_projection_handoff/20260914T095303Z/evidence/tau_vs_progress.png).
Their axes explicitly say controlled delay; lines connect the four samples for
readability and are not additional observations. Input hashes, units and plotted
arrays are retained in `plot_manifest.json`. All nine actual images were
inspected and bound by SHA-256 in `visual_review.json`.

## Reproduction and validation

Use a new output directory for RUN; existing generated artifacts are immutable.

```bash
.venv/bin/python scripts/validate_robotless_controlled_staleness.py \
  data/robotless_controlled_staleness/20260914T082941Z
.venv/bin/python scripts/characterize_robotless_projection_handoff.py RUN \
  --config configs/robotless_projection_handoff.yaml
scripts/isaac/run_robotless_projection_handoff.sh RUN --no-hold
# Inspect five Isaac images and four plots, then write hash-bound visual_review.json.
.venv/bin/python scripts/validate_robotless_projection_handoff.py RUN --write

# Reopen with interactive overview/detail controls without rewriting evidence:
scripts/isaac/run_robotless_projection_handoff.sh RUN --view-only

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
bash -n scripts/isaac/run_robotless_projection_handoff.sh
git diff --check
```

- New pure/synthetic tests: **61 passed in 9.92 s**. They cover interior and
  endpoint projections, multiple segments, exact ties, degenerate points and
  unavailable tangents, progress, atan2 quadrants, distinct tangent/yaw,
  shortest-angle interpolation across +/-pi, signed residuals, unchanged arrays,
  arbitrary N, invalid input, previous d_poly agreement, source/metric corruption
  and fail-closed visual review. Mocked renderers are schema tests only.
- Full host suite: **911 passed in 33.66 s**. Compileall and launcher shell
  syntax passed. Whitespace and staged-scope checks passed before commit.
- Actual frozen-source analysis, four plots and the actual Isaac process exited
  successfully. The complete validator has no missing artifacts or failures.
  Unit/synthetic test results are separate from this actual runtime evidence.
- An independent scalar audit used explicit dot products, clamping, `hypot`,
  arc sums, atan2 and modular angle wrapping without project geometry helpers.
  Maximum numerical difference is **2.7755575615628914e-17**. It also checked
  all 91 source files and 13 processing-code snapshots. This is arithmetic
  verification, not physical execution evidence.
- The first host Isaac request did not execute because automatic approval
  review was at capacity. After scope/source checks, the same request succeeded;
  no rejection was bypassed and no runtime requirement remains blocked.
  Isaac DLSS/readback/plugin-release warnings remain in its log. Matplotlib
  used a temporary `/tmp` cache because its default cache was not writable.

## Files and provenance

The focused change contains 12 files:

| Area | Files |
| --- | --- |
| Configuration | `configs/robotless_projection_handoff.yaml` |
| Pure geometry | `src/reconciliation/robotless_projection_handoff.py` |
| Pipeline/validation | `scripts/characterize_robotless_projection_handoff.py`, `scripts/robotless_projection_artifacts.py`, `scripts/validate_robotless_projection_handoff.py` |
| Isaac | `scripts/isaac/robotless_projection_handoff.py`, `scripts/isaac/run_robotless_projection_handoff.sh` |
| Tests | `tests/test_robotless_projection_handoff.py`, `tests/test_robotless_projection_artifacts.py` |
| Documentation | This report, `README.md`, append-only `docs/WORK_LOG.md` |

Existing source and shared utilities are reused without modification. The two
unrelated local Stage0 YAML edits are preserved and excluded from this commit.
The README addition is two sentences following successful runtime validation.

`source.json` binds controlled-source validation, all file hashes, raw/world
array references, observation timing, B/R_obs motion semantics, processing code
and configuration. Derived JSON/CSV identify their input hashes. The ignored run
also holds all images, manifests, code snapshots, logs, independent audit and
validation. Raw outputs and prior generated artifacts are never overwritten.
Final starting/final Git identities and the focused file list are in that run's
`git_completion.json`. The final commit can also be located with
`git log -1 -- docs/ROBOTLESS_PROJECTION_HANDOFF_GEOMETRY.md`.

## Interpretation limits

These measurements distinguish spatial separation, tangent-direction mismatch,
pose-orientation mismatch and closest along-path position. The task's examples
of small/large combinations are qualitative interpretation examples; this
implementation neither assigns those classes nor adds thresholds or weights.
The unchanged source gives sub-degree angular residuals in every condition.

The supported claim is that these five geometric quantities can be measured
consistently for the fixed FRESH and controlled boundaries. Q and progress are
descriptive closest positions, not optimal correspondence or graph factors.
No automatic correspondence, reconciliation, graph, rigid alignment, objective
weight, gate, controller, dynamics or new LightNav collection is implemented.

This single saved output does not establish a LightNav defect, latency-caused
navigation failure, a need for reconciliation, graph superiority or improved
closed-loop performance. Distance/angles are numerical results, not physical
accuracy, collision-free feasibility or successful execution. The remote
Hospital asset tree is identified by asset URL/version, not recursively hashed.
