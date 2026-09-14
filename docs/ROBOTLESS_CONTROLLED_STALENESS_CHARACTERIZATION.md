# Robotless controlled FRESH staleness and handoff geometry

Decision: **ROBOTLESS_CONTROLLED_STALENESS_VALIDATED**.

The frozen successive run was validated, four controlled boundary conditions
were computed, and the original OLD/FRESH paths and all four boundaries were
inspected in actual Isaac Sim. No new LightNav inference was performed.

For this saved FRESH and the prescribed forward motion, neither distance is
larger than its tau=0 baseline at any sampled condition. Entry distance first
decreases and then increases; polyline distance decreases throughout the sampled
conditions. This result is retained without selecting alternative observations,
velocities or delays to make mismatch increase.

## Research question and source

How does boundary-to-FRESH geometry change when the boundary moves before use,
while raw FRESH remains fixed in its observation-time world frame?

The source of truth is the validated
[successive-chunk run](ROBOTLESS_ISAAC_LIGHTNAV_SUCCESSIVE_CHUNKS.md):
`data/robotless_successive_chunks/20260914T073030Z/`.
The existing validator was run before generating outputs and again during the
complete audit. It returned `ROBOTLESS_SUCCESSIVE_CHUNKS_VALIDATED`.
All 54 existing source-run files were hashed and verified unchanged afterward.
No raw or derived source artifact was copied for modification or overwritten.

| Source input | SHA-256 |
| --- | --- |
| `raw/chunk_000.npy` (OLD) | `928cbb6935b565a5cc059216a61d4c3ee38a003eef7b6787b6e8dc115a48d6ce` |
| `raw/chunk_001.npy` (FRESH) | `cc0b216983f36c2b938ee10e0ac839ed3c4c381e84ef136a48a39f25373a491d` |
| `derived/chunk_000_world.npy` | `c23982987ccef88654264c74a0c20aa3e7c838bb5f863bfa60cbcd89c981c42f` |
| `derived/chunk_001_world.npy` | `2c44792f4f4fd2725a45176c14c87f02ee812d28b1a56b689eeb95b08ee1cd37` |

Both source chunks contain 10 cumulative local poses. The existing world arrays
are loaded directly; there is no re-integration or new world-frame anchoring.
OLD remains context only and does not determine B.

The FRESH observation is R1:

```text
R_obs = [19.0, 27.0, 1.5707963267948963]  # metres, radians
observation UTC = 2026-09-14T07:36:52.886816Z
```

The source's capture-time research SHA was `1572ea407bc26b6cc5ff94f967ed12316de729e3`;
its reviewed implementation commit was `913bf235806b769cfe81cbef233be66a7b1be7a8`.
This task started at that latter SHA, also confirmed by a successful fetch of
`origin/main`. These different provenance events are retained explicitly.

## Controlled motion and clocks

The versioned configuration is
[`configs/robotless_controlled_staleness.yaml`](../configs/robotless_controlled_staleness.yaml).

```text
v = 0.25 m/s; omega = 0 rad/s
tau = [0.0, 0.2, 0.5, 1.0] s
B(tau) = R_obs * [v*tau, 0, 0]
```

Agent +X is forward, +Y left, +Z up, positive yaw CCW. World is right-handed,
metre-scale and +Z up. At this observation yaw, local forward is world +Y.
The implementation reuses `reconciliation.se2.compose_poses`, `relative_pose`
and `se2_log`. Translation-only motion preserves the recorded yaw representation
and makes B(0) exactly equal to R_obs. Nonzero omega is rejected for this stage.

Tau is an explicit controlled simulation variable, independent of host UTC,
monotonic time and Isaac timeline time. No process sleeps for tau and no
waypoint timing is assigned. The logical agent stays at R_obs in the static
Isaac view; the four B markers show counterfactual boundary states. The entire
timeline remains stopped at 0.0 s.

The source's measured FRESH RTT was 192.592501 ms with no concurrent motion.
It is not used to construct tau, B, a switch timestamp or any metric. The new
run records creation/view timestamps, retains the original observation event,
and sets actual execution and new inference-readiness events to null.

## Fixed FRESH and metric definitions

For every condition:

```text
T_world_F_j = T_world_R_obs * T_R_obs_F_j  # already saved in the source
FRESH world = fixed
B(tau)     = changing
```

One read-only FRESH world array is used for all conditions. Each condition
references exactly the same source path and SHA-256. There is no B-relative
reanchoring, OLD-endpoint anchoring, correction or alignment.

- **Boundary motion:** `Delta_B = R_obs^-1 * B`. Save ordinary SE(2) components,
  XY norm and yaw change separately.
- **Entry residual:** `r_entry = Log(B^-1 * F_0)`. Save tangent components
  `[dx,dy,dyaw]` in the B frame and the norm of its translational components.
  Also save the ordinary relative pose `B^-1 * F_0`. Log translation and
  ordinary relative XY are distinct when relative yaw is nonzero.
- **Entry distance:** `d_entry = ||B_xy - F_0_xy||`, in metres.
- **Polyline distance:** minimum Euclidean point-to-segment distance across
  segments `[F_j.xy,F_(j+1).xy]`, using clamped interior projections. No connector
  from R_obs to F_0 is included. Repeated rows form zero-length segments; a
  singleton trajectory is treated as a point. Ties select the first segment.
- **Signed changes:** `Delta_d_entry = d_entry(tau)-d_entry(0)` and likewise for
  `d_poly`. Negative values mean reduced mismatch under this particular metric.

The dedicated XY projection helper avoids importing historical diagnostic
modules that also import controller code. No graph, correspondence or weighted
translation/yaw score is used.

Tau=0 supplies the requested pre-motion geometric baseline. These formulas
measure R_obs-to-FRESH geometry; they do not by themselves measure a pairwise
OLD/FRESH trajectory discrepancy. OLD is available visually as unchanged context.

## Actual numerical results

Generated run:
[`data/robotless_controlled_staleness/20260914T082941Z/`](../data/robotless_controlled_staleness/20260914T082941Z/).
All positions below are world metres; every B has yaw `1.5707963267948963` rad.

| tau (s) | B.x | B.y | Controlled forward displacement (m) | d_entry (m) | d_poly (m) | Delta_d_entry (m) | Delta_d_poly (m) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.0 | 19.0 | 27.0 | 0.000 | 0.150419231 | 0.150419231 | 0.000000000 | 0.000000000 |
| 0.2 | 19.0 | 27.05 | 0.050 | 0.100419236 | 0.100419236 | -0.049999995 | -0.049999995 |
| 0.5 | 19.0 | 27.125 | 0.125 | 0.025419282 | 0.025419282 | -0.124999949 | -0.124999949 |
| 1.0 | 19.0 | 27.25 | 0.250 | 0.099580795 | 0.000169257 | -0.050838435 | -0.150249973 |

Values above are rounded for readability; JSON/CSV preserve full precision.
The computed `Delta_B` lateral component is about `5.4e-16` to `6.1e-16` m from
floating-point inverse/composition. Yaw change is zero. Expected translations
are satisfied within numerical tolerance; B(0) is exactly the saved R_obs.

Entry Log residuals, in B-frame metres/radians:

| tau (s) | log dx | log dy | log dyaw |
| --- | --- | --- | --- |
| 0.0 | 0.150419232616 | 0.000010760788 | 0.000600960630 |
| 0.2 | 0.100419234121 | 0.000025784804 | 0.000600960630 |
| 0.5 | 0.025419236378 | 0.000048320827 | 0.000600960630 |
| 1.0 | -0.099580759860 | 0.000085880867 | 0.000600960630 |

The saved FRESH entry is about 0.150419 m forward of R_obs. Forward boundary
motion initially approaches it. At tau=1, B has passed the entry, explaining
why d_entry rises relative to tau=.5 while remaining below its tau=0 baseline.
The closest polyline point now lies inside the first segment, at fraction
`0.6622355452892925`; its distance is only `0.169257` mm. Treating d_entry as
the complete handoff quality would miss this distinction.

An independent scalar audit used sin/cos motion, the half-angle closed form
for the SE(2) Log and explicit segment projections, without the project's metric
or SE(2) helpers. Its maximum absolute difference from the saved numerical
results was below `5.1e-15`. This verifies arithmetic, not physical execution.

## Isaac visualization and plots

Actual Isaac 6.0.1 loaded the same Hospital scene and produced the
[overview screenshot](../data/robotless_controlled_staleness/20260914T082941Z/evidence/isaac_handoff_overview.png)
at `2026-09-14T08:30:39.130250Z`. The scene has 1,936 prims and 126 collision
prims, with zero robot-named paths, articulations, rigid bodies or physics scenes.
The static logical agent remained at R_obs before/after rendering.

| Display | Meaning |
| --- | --- |
| Blue | Original OLD world polyline, context only |
| Magenta | One unchanged FRESH world polyline, beginning at its original F_0 |
| Yellow tick | R_obs = B(0) |
| Cyan tick | B(.2), 0.05 m forward |
| Orange tick | B(.5), 0.125 m forward |
| Red tick | B(1), 0.25 m forward |
| Green | Straight controlled boundary-motion path |

All four markers and both trajectories are visible together. A separate Isaac
UI legend identifies the conditions; the viewport PNG contains colored geometry.
The helper reads saved arrays/config and independently checks B with scalar
translation; it never reads metrics.json or metrics.csv. Actual drawn trajectory
rows and boundary poses are retained in `visualization.json` for verification.
Display height offsets are 0.12 m for paths and 0.16 m for boundary markers;
source XY and yaw are unchanged. There is no geometry scaling or scene hiding.

The [entry plot](../data/robotless_controlled_staleness/20260914T082941Z/evidence/latency_vs_entry_gap.png)
and [polyline plot](../data/robotless_controlled_staleness/20260914T082941Z/evidence/latency_vs_polyline_gap.png)
each show absolute distance, the tau=0 baseline and signed change. Their input
is the saved metrics JSON, with hashes and plotted arrays in `plot_manifest.json`.
The filenames follow the requested examples; the axes explicitly say controlled
delay, not actual LightNav latency. Lines connect the four sampled conditions
for readability and do not represent additional measurements.

The actual screenshot and both plots were inspected, and their hashes bound to
an explicit `visual_review.json`. The overview SHA-256 is
`024085934e4b671aa4cfd3f8584f762ab24dbf1959f0ac7cfcb2631b59bd1e92`.

## Reproduction and validation

From the repository root, use a fresh output directory for `RUN`:

```bash
git fetch origin
.venv/bin/python scripts/validate_robotless_successive_chunks.py \
  data/robotless_successive_chunks/20260914T073030Z
.venv/bin/python scripts/characterize_robotless_controlled_staleness.py RUN \
  --config configs/robotless_controlled_staleness.yaml
scripts/isaac/run_robotless_controlled_staleness.sh RUN --no-hold
# Inspect the viewport and plots; record an explicit hash-bound visual_review.json.
.venv/bin/python scripts/validate_robotless_controlled_staleness.py RUN --write

# Reopen existing geometry without rewriting evidence:
scripts/isaac/run_robotless_controlled_staleness.sh RUN --view-only

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest
.venv/bin/python -m compileall src scripts tests
git diff --check
```

- Actual source validation, analysis/plot generation and Isaac visualization
  exited successfully. The complete artifact validator returned validated with
  no missing artifacts or failures.
- New pure Python tests: **84 passed in 4.54 s**. Tests cover exact B(0), rotated
  forward motion, invariant yaw/raw arrays, arbitrary path lengths and degenerate
  segments, true Log residuals, signed baseline changes, invalid inputs, source
  and derived corruption, and independent visual inputs. Synthetic artifacts
  and mocked source-validation tests are tests only.
- Final host full suite: **850 passed in 24.70 s** with
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest`.
- The full sandbox suite reported **848 passed, 2 failed in 24.43 s**. Both
  failures are existing Unix-socket tests blocked by sandbox `PermissionError`,
  not controlled-staleness assertions. Four host rerun requests did not execute
  because automatic approval review was at capacity. After the scope and actual
  socket failures were checked, the fifth identical host request succeeded.
  The host full-suite pass above resolves the two sandbox-only failures.
- Compileall and launcher shell syntax passed. Runtime logs retain Isaac DLSS,
  readback and plugin-release warnings. Matplotlib used a temporary `/tmp` cache
  because the default cache directory was not writable; both plots were produced.

## Files and provenance

The focused change contains 12 files:

| Area | Files |
| --- | --- |
| Config | `configs/robotless_controlled_staleness.yaml` |
| Pure metrics | `src/reconciliation/robotless_controlled_staleness.py` |
| Analysis/artifact validation | `scripts/characterize_robotless_controlled_staleness.py`, `scripts/robotless_controlled_staleness_artifacts.py`, `scripts/validate_robotless_controlled_staleness.py` |
| Isaac | `scripts/isaac/robotless_controlled_staleness.py`, `scripts/isaac/run_robotless_controlled_staleness.sh` |
| Tests | `tests/test_robotless_controlled_staleness.py`, `tests/test_robotless_controlled_staleness_artifacts.py` |
| Documentation | This report, `README.md`, append-only `docs/WORK_LOG.md` |

Existing successive code is reused without modification. README contains only
the success-only two-sentence addition for this task.

`source.json` records the validated source path, all source hashes, array inputs,
R_obs and observation time, research SHA, processing-source hashes and frozen
configuration. `derived/` holds boundaries and metrics only. Plots, screenshot,
review/validation, source snapshots and logs stay in the ignored generated run.
Final Git identity is recorded in its `git_completion.json`; use
`git log -1 -- docs/ROBOTLESS_CONTROLLED_STALENESS_CHARACTERIZATION.md` to locate
the focused implementation commit. Neither unrelated Stage0 YAML edit is staged.

## Interpretation limits

This validates measurement of controlled boundary-to-fixed-FRESH geometry for
one previously observed model output. The boundary motion is a configured
kinematic surrogate; it is not measured robot motion or actual OLD execution.
The result does not show that measured LightNav latency caused mismatch, that
graph or rigid reconciliation is needed, that navigation improves, or that
controller discontinuity decreases. No such method or controller was added.
The scene's remote asset tree is not recursively content-pinned. Submillimetre
computed polyline distance is a numerical geometric result, not evidence of
physical simulator/robot accuracy or successful collision-free handoff.
