# GP-SE2-REF-03: frozen source-progress selector transfer

## Question and frozen scope

Does the exact REF-02 source-progress selector recover execution on additional
saved handoffs, and does it introduce goal, route, clearance or motion failures?
This is a bounded diagnostic transfer within the existing development corpus.
It is not a held-out benchmark, a GP experiment, online navigation or deployment
safety evidence. The starting revision is
`22c655ea421c77c1bc7bd7c70da5fec7c9ae7620`.

The primary output is
`data/robotless_gp_se2_ref_03/primary_20260919T141000Z/`.
The execution revision and final authoritative validation will be recorded after
the frozen one-pass schedule. No REF-03 primary MPC solves have occurred at this
protocol-writing stage.

## Source-only cohort, before execution

The original GP-SE2-01 catalog contains 881 events, of which 730 satisfy its
original source/timing/B/goal/future-suffix/route eligibility. The union of the
GP01/GP02/REF01/REF02 selected manifests excludes 10 previously selected events,
leaving 720 additional eligible records. All source integrity checks pass.
Additional prior dataset visual-review usage is recorded, not hidden behind a
claim of complete novelty. Original source data, authoritative validation,
environment exports, results, figures, external MPC and unrelated local edits
remain protected by before/after hashes.

| Stratum | Eligible flagged events | Selected | Shortfall |
|---|---:|---:|---:|
| O: obstacle near | 35 | 6 | 0 |
| R: large rotation | 16 | 6 | 0 |
| P: large mismatch | 95 | 6 | 0 |
| S: original small straight | 378 | 6 | 0 |

Flags overlap, but assignment does not. O uses valid original **future suffix**
footprint edge clearance at most 0.15 m, retaining the original required 0.05 m
and uncertainty rule and excluding an artificial B connector. R uses at least
45 degrees of accumulated absolute wrapped yaw change *inside* the suffix.
Its rotation-dominant priority uses suffix XY arc at most 0.20 m and at least
30 degrees of yaw variation. P uses original `e_perp >= 0.10 m` or reliable
incoming/window direction difference at least 30 degrees. S uses the unchanged
GP01 `A_SMALL_STRAIGHT` predicate.

Assignment order is O, R, P, S. At every pick prefer an unused episode, then an
unused ordered raw pair, then (R only) rotation-dominant, then lexical SHA256 of
`REF03-v1:` plus the full case ID, and finally the ID. Diversity sets start empty
for the additional cohort; controls do not seed them. No replacement, relaxed
threshold or extra stratum fill is allowed. All 24 selected additional events
have distinct episodes and ordered raw pairs. All six selected R events are
rotation-dominant. Repeated underlying raw arrays can still occur across pairs;
these are not independent population samples or an episode train/test split.

Known controls, excluded from new transfer evidence, run first:

1. `episode_014_repeat_01/handoff_026`: REF-02 large-turn reproduction.
2. `episode_001_repeat_01/handoff_002`: REF-02 benign reproduction.
3. `episode_017_repeat_00/handoff_007`: known obstacle/route stress, preserving
   the original directed gate, geometry and ordered crossing predicate.

The frozen manifest/catalog retain every selected and nonselected reason, source
hash, overlap flag, diversity count and source-associated memory discrepancy.

## Frozen inputs, selector and MPC computation

A uses the unchanged original FRESH world rows and official next-row selector.
B uses original `prepare_reference`/REF01 lineage and the same official selector.
C uses byte-identical B arrays and lineage with the exact REF-02 selector:

```
j = original weighted-pose argmin on installed dense rows
q_h = min(s_j + h, s_last), h = 1,...,5
i_h = searchsorted(s, q_h, side="left")
reference = installed_rows[i_h].copy()
```

Stride is exactly 1.0 original-row units; strict float64 search has zero search
tolerance. Each q uses the same unrounded s_j. No new interpolation, accumulated
overshoot, progress memory, suffix recut, native-row substitution or reanchoring
is introduced. Sequential yaw unwrap and authenticated constant-source handling
are unchanged. Original progress denotes row order, not time or distance.

`gp_se2_ref02_reference.py` and `gp_se2_ref02_rollout.py`, eight other numerical
and reference files, official MPC revision/settings and original evaluation
configuration are hash-pinned in `configs/gp_se2_ref03.yaml` and run provenance.
The wrapper only registers tracker construction for partial fault evidence;
all computational methods and the existing per-instance actual-input recorder
remain inherited. MPC computation is unchanged; A/B/C reference selection is
the intervention. HORIZON=5, dt=0.1 s, control rate=10 Hz, original gains/limits.

All candidates are expressed in the original capture frame through its inverse
transform and audited after official world installation. B, physical u_minus
and recorded previous_control remain separate. Original goal, footprint 0.20 m,
required clearance 0.05 m, workspace, route, goal tolerances 0.15 m/15 degrees
and final 0.20 s dwell are unchanged. LightNav intrinsic waypoint_dt is null;
the 0.1..3 s resampling clock is only the existing row-order convention.

## Once-only schedule, outcomes and timing

The frozen 27-event schedule plans 81 independent 3 s rollouts, 2,430 primary MPC
solves and 1,620 selector-only calls. Per event: A executes 30 solves, its actual
30 solve-input poses provide the B/C matched-state probe bank, then B and C each
execute once. All execution uses exact held-command unicycle integration at
60 Hz, yielding 181 states/180 ticks for a complete rollout. No historical path
fallback, VLA update, GP/rigid solve, special stopping rule or retry is allowed.
Scientific failures do not remove later planned cases. Missing computation is
explicit null and does not masquerade as a failed complete rollout or zero error.

The original full success conjunction remains authoritative. Additional spatial
reference checks do not filter methods from this diagnostic rollout. Gate passage
is required where applicable. Physical-command/controller-memory first-step
violations stay in the original success predicate and are separately labeled as
source-associated, without asserting selector causality.

Report B-success/C-failure and A-success/C-failure before recoveries; also inspect
new safety, motion, route and goal reasons when the baseline already failed.
Six requested transition flags deliberately overlap; the eight A/B/C Boolean
patterns are disjoint. Successful-pair quality is conditioned on both methods
passing all original predicates. Nonarrival stays N/A. Event, episode and ordered
raw-pair equal-weight summaries and memberships remain explicit; controls and
additional strata are never pooled into a novel-evidence rate.

Timing separates preparation, environment load, official MPC solves, full rollout,
selector probes, evaluation and reporting/validation. Inclusive components are
labeled and are not summed as exclusive costs. Wall time does not advance the
simulation clock. No online performance claim is planned.

## Evidence and reproducibility

Every case receives all seven required static figures with numeric/source/config
sidecars. B/C shared reference is drawn once, in metres with equal aspect; yaw
wrap cuts are not drawn as physical turns. Actual gate geometry remains visible.
Representative selection includes all new C safety/route regressions, then the
first hash-ordered B-failure/C-success and same-B/C-success-status case in each
additional stratum, plus all controls. Same status does not imply identical
commands or trajectories. The complete index retains all 27 cases and N/A.
The small review ZIP excludes RGB, full dataset/environment and external source.
GUI is not run: these are saved static diagnostics.

Commands (run once per phase; all output writers reject overwrite):

```bash
.venv/bin/python scripts/run_gp_se2_ref03.py prepare --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
.venv/bin/python scripts/run_gp_se2_ref03.py freeze --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
.venv/bin/python scripts/run_gp_se2_ref03.py execute --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
.venv/bin/python scripts/run_gp_se2_ref03.py evaluate --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
MPLCONFIGDIR=/tmp/ref03_mpl .venv/bin/python scripts/plot_gp_se2_ref03.py --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
.venv/bin/python scripts/package_gp_se2_ref03_review.py --run data/robotless_gp_se2_ref_03/primary_20260919T141000Z
```

Execution delegates to the existing official isolated MPC environment; no runtime
upgrade or system Python modification. Saved-record validation performs no new
MPC/GP solve. The report completion section will record all outcomes, limitations,
exact validation commands, code revisions and the next single experiment.
