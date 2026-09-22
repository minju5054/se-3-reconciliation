# Saved native handoff execution: what reconciliation should improve

The measured problem in the 13 previously screened sources is delayed/unstable
attachment to original FRESH, rather than a demonstrated collision. B is a fixed
initial state, and its existing offset is not something an optimizer can erase
retroactively. A smaller planned GP objective is also not proof of better actual
execution. This audit measures what happens **after** the recorded switch.

Starting/fetched main: `cb45aa91f1e81c141257671b4f265362d9ecc04a`.
Only saved records were read. New LightNav, GP/rigid optimization, MPC solve,
rollout, simulator and GUI calls: zero. Re-evaluating held commands solely
reconstructs the already saved states; it is not a new execution.

## Definitions and scope

All 13 source-inventory cases remain included, with no new outcome-based
selection. Eleven have interior FRESH projections, while two have nearly
exhausted endpoint geometry and are reported separately. They belong to seven
episodes and 13 ordered raw pairs; this selected development subset does not
estimate population failure frequency.

The observation window starts at original B and ends at the next actual chunk
application, with that next chunk's command excluded. Recorded lifetimes are
0.966667–1.550000 s. Original world frames, physical u_minus and separate MPC
previous-control values are preserved. Every recorded command reconstructs its
following stored pose to within 1e-10. Model rows remain spatial rows without
intrinsic timestamps.

The unchanged JOIN01/ATTACH01 evaluator projects executed states onto the
original FRESH polyline with forward-only original-row progress and shortest
yaw interpolation. It is an evaluation correspondence, not a new selector.
Sampled sustained attachment requires position <=.10m and yaw <=15deg over a
complete following .30s. No new numerical relaxation was introduced. Failure
to observe this before another chunk arrives yields N/A, never a substituted
3s join time or a fixed-horizon failure claim.

For equal exposure, costs also use the first 54 recorded integration intervals
of every source: nominal .9s, actual .900000046938658s. No resampling, fabricated
states or time-axis rescaling is used. Full variable-lifetime costs are retained
separately. The four scalar costs are reported in their own units:

- position error area: integral d_pos(t) dt [m s];
- position excess area: integral max(d_pos(t)-.10,0) dt [m s];
- yaw error/excess area [rad s], using the original 15deg threshold;
- linear/angular command total variation [m/s, rad/s], including the first
  command's difference from the actual physical u_minus.

No arbitrary weighted total combines them. Lower command variation can simply
mean insufficient turning or stopping, so it is not an improvement by itself.
Likewise, error-area zero is not asserted achievable with the fixed B and limits.

## Measured attachment evidence

| Classification within the recorded reference lifetime | All 13 | Interior 11 | Endpoint caveat 2 |
|---|---:|---:|---:|
| Complete sampled .30s attachment observed | 1 | 1 | 0 |
| Never entered joint position/yaw tube | 7 | 5 | 2 |
| Entered late; next switch truncated the .30s dwell | 4 | 4 | 0 |
| Entered transiently, then exited | 1 | 1 | 0 |

The one observed join is `episode_013_repeat_01/handoff_029`, at1.016667s,
with .300000016s recorded dwell. The late-entry cases are001/01/013,
016/01/007,001/01/011 and013/00/020; their observed tube spans are
.233333/.250000/.200000/.033333s. They are **right-censored**, not proven unable
to complete attachment. Case013/01/021 enters at1.116667s but leaves after
.116667s. Every N/A and all remaining traces remain in the artifacts.

| Case | Initial distance | Maximum distance / time | First .9s mean distance | First .9s position AUC | First .9s excess AUC | Attachment observation |
|---|---:|---:|---:|---:|---:|---|
| 013/01/024 | .279113m | .376956m / .500000s | .343891m | .309502m s | .219502m s | No tube entry before1.55s switch |
| 008/01/023 | .184683m | .239120m / .366667s | .209629m | .188667m s | .098667m s | No tube entry before1.50s switch |
| 001/01/013 | .127644m | .163895m / .316667s | .145560m | .131004m s | .041004m s | Entry1.016667s; only.233333s observable |
| 013/01/029 | .133713m | .162414m / .316667s | .144394m | .129955m s | .039955m s | Complete join1.016667s |

These show initial **growth** of the already-existing separation while the
tracker turns, followed by convergence or persistent error. The candidate
research benefit is reducing this post-boundary error area and attaining a
sustained attachment earlier, under identical fixed correspondence/duration
conditions and unchanged safety/motion acceptance. These measurements alone do
not establish how much of that cost is physically avoidable.

All-event common-prefix mean AUC is .173989m s; equally weighted episode-mean
AUC is .179703m s. Ordered-pair weighting equals event weighting here because
all13 ordered pairs are unique. These are descriptive costs, not significance
tests or evidence that an optimizer outperforms native execution.

## Safety and command interpretation

All13 post-switch paths pass the reused direct Hospital swept-circle checker
with radius.20m, nominal edge clearance.05m, original geometry uncertainty, and
held-command arc/chord bound max(|v*omega|*dt^2/8). This is the saved logical-agent
geometry, not a physical robot experiment. For013/01/024,008/01/023,001/01/013,
minimum clearance lower bounds are1.843379/.380789/.757224m. No new required gate
applies to these cases. Fixed3s goal/dwell success is not inferred from their
shorter, changing-reference online lifetimes.

One nominal10Hz command-difference flag occurs in008/01/023: dv=-.400000m/s
at B+1.35s gives4.000000m/s² if divided by .1s. Saved application records show
the previous changed command was actually applied .200000010s earlier, yielding
~2m/s² over that interval. Solver302 returned an intermediate .23295696m/s
command that was not applied; solver303 used it as previous-control memory.
The previous applied command was .43295696m/s, then .03295696m/s was applied.
Both the nominal flag and these logs are retained under
`command.nominal_grid_timing_context`. This is not isolated evidence of a
reconciliation-induced physical acceleration failure. Continuous acceleration
is not represented by these instantaneous held-command changes.

The initial memory discrepancy in013/00/026 is also preserved, not reset. No
controller repair, scheduling change or new diagnosis experiment was performed.

## Connection to the optimization being researched

Current generic `GPProblem.evaluate` optimizes GP prior cost plus mean normalized
SE(2) residual to the fixed **planned** common-reference rows. It constrains B,
initial twist and plan motion/environment/goal conditions. It does not evaluate
the official MPC's executed attachment time or error area inside its objective.

`FixedAttachView` removes FRESH preservation before supplied T and adds the
four support attachment-tube samples from T through T+.3s. It still measures
the planned supports; it does not guarantee the same tube under MPC execution.
T remains an external condition. This audit adds neither an optimized join time
nor new correspondence search, weights, constraints or controller terms.

For the contribution to be established, a valid optimized plan must improve
actual sustained attachment / same-window error relative to unchanged native
and rigid baselines while retaining safety, motion and task progress. Compute
and command variation are separate costs. GP prior reduction, lateral-plan
feasibility alone, rigid-vs-GP objective values, or visually prettier paths do
not establish this benefit.

There is also a task-level limitation: staying closer to FRESH is a declared
attachment objective, not automatically better navigation. A different safe
path could reach the intended goal efficiently. These13 sources do not yet
prove collision prevention, reduced energy, shorter navigation time or a
necessary obstacle-constrained transition. Those must not be claimed merely
from the measured tracking errors. The strongest current evidence is a
measurable local attachment cost; its reduction by the proposed method remains
unverified.

## Reproduction and validation

Final saved-only result:
`data/saved_handoff_execution_loss/audit_20260922T091708Z/`.
It includes frozen protocol, source/core hashes, all13 numerical records,
metrics.csv, summary, three PNGs with numeric/source sidecars, static index and
saved-only validation. Audit + plotting + recomputation takes2.155s on this run.

The initial saved-only pass `audit_20260922T091255Z` is retained. The final pass
adds censoring categories and asynchronous command-application context; source
cohort, thresholds, costs and join flags are unchanged. It is analysis of the
same saved execution, not a new scientific rollout or optimizer retry.

```bash
MPLCONFIGDIR=/tmp/handoff_loss_mpl OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python scripts/audit_saved_handoff_losses.py \
  --output data/saved_handoff_execution_loss/NEW_UNIQUE_RUN
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/audit_saved_handoff_losses.py \
  --output data/saved_handoff_execution_loss/audit_20260922T091708Z --validate
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q \
  tests/test_handoff_execution_loss.py tests/test_gp_se2_join01.py \
  tests/test_gp_se2_attach01.py tests/test_genuine_source_scan_gui.py \
  tests/test_online_handoff_analysis.py tests/test_gp_se2_environment.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

138 relevant tests pass. Validator recomputes saved metrics/commands/geometry,
CSV and figure numbers and verifies source/code hashes. Synthetic tests cover
transient crossing, late-entry censoring, wrap, forward progress, unchanged raw
arrays, integration reconstruction and distinct nominal command-grid semantics.
No model, optimization or MPC calls are made by the tests. Raw source arrays,
original metrics, numerical core, external sources and unrelated Stage0 edits
remain unchanged.
