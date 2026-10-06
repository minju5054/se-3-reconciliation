# DIRECT_TRANSITION_HARD_EVAL_02

## Research question

Does `[B,E*,original suffix]` remain sufficient for meaningful direct transitions
with a large incoming-direction mismatch? If intermediate geometry helps, does the
unchanged V2 graph add execution benefit beyond deterministic Hermite geometry?
This is a source-disjoint controlled evaluation within a saved development corpus.
It changes source selection and failure attribution, not the graph formulation.

## Repository-confirmed pre-execution facts

Starting HEAD and fetched origin/main:
`b7113842ca3bd61922a37fec24413e572d35af6e`.
The previous STAGING_GRAPH_NECESSITY_EVAL_01 config, selection, metrics,
classification and figures remain immutable. Unrelated Stage0 config edits and
`scripts/plot_gpu_memory_snapshot.py` are preserved and excluded from commits.

The authenticated catalog contains 881 saved genuine handoff events. The prior
registry excludes 33 recorded episodes; adding the entire six previous evaluation
episodes excludes 39. Previous E1–E6 are all excluded:
episode_011_repeat_01, episode_015_repeat_01, episode_018_repeat_00,
episode_003_repeat_01, episode_026_repeat_01, episode_024_repeat_01.
Exclusion is per recorded episode including repeat, not only per handoff.
The prior registry still covers GP01/REF03/Genuine13/ATTACH and S2–S4. OSA03 S1
is from a separate acquisition lineage and is not in this corpus.

After unchanged base gates and episode exclusions there are 90 candidates.
The meaningful-transition gate leaves **4 candidates in 4 episodes**. Target6 is
unavailable; the predeclared minimum4 is met. No gate is relaxed and no new source
is acquired. All four eligible episodes are selected by the two-axis rule.

| ID | Source | Chord m | d_F m | rho_jump | delta_phi rad | Remaining arc m | Hermite / V2 M |
|---|---|---:|---:|---:|---:|---:|---|
| E1 | episode_009_repeat_00/handoff_001 | 0.051677232427019916 | 0.15013808045914007 | 0.3441980360278006 | 1.7674650496284894 | 1.2075100566025427 | 2 / 2 |
| E2 | episode_017_repeat_01/handoff_008 | 0.04777018782454907 | 0.1497471861994649 | 0.31900557891564424 | 1.7360893514213158 | 0.9681734577863171 | 2 / 2 |
| E3 | episode_007_repeat_01/handoff_001 | 0.06094376778888808 | 0.15037734419790505 | 0.40527227099238206 | 1.7034222438876716 | 1.1910917472641231 | 2 / 2 |
| E4 | episode_016_repeat_00/handoff_008 | 0.030876991436327626 | 0.1484140539069248 | 0.20804627744816925 | 1.6745277653828212 | 1.0313633846680688 | 2 / 2 |

E1–E3 are the directional axis selections; E4 is the remaining episode selected
by descending rho_jump. These are nonzero centimetre-scale gaps, but all rho_jump
values are below1. This available subset does not test long jumps larger than the
original row spacing. This limitation is known before execution.
Eight official installation/restoration preflights (Native bits plus methods per
source) pass with zero numerical MPC calls. Full-cap budgets are4 V2 solves,
16 rollouts,480 MPC solves and472 scheduled successful releases; safety or invalid
references can reduce realized counts. No historical rollout is repeated.

## Frozen protocol

### Authentication, coordinates and selection

Read the prior experiment and bridge/correspondence/source scan implementations
at starting HEAD. Reuse the REF03 catalog and its exact hashes, complete raw
acquisition manifests, Hospital environment and authoritative validation hashes.
The new config pins all authority hashes and the previous selected-source identity
manifest. Read prior evaluation identities only to exclude their episodes; no
prior method outcomes enter ranking. Original acquisition metrics contribute only
the unchanged timing flags, never attachment/error/convergence scores.

Raw LightNav is Nx3 `[forward, lateral-left, yaw-CCW]`, metres/radians, relative
to observation pose A. World FRESH is `F_j = A F_local,j`. B is the saved
application pose at t_switch. Observation, request, host readiness, installation
and switch clocks remain explicit and distinct. Raw FRESH is immutable. Never
re-anchor it at B, and never assign waypoint timestamps or dt.

Base eligibility is imported unchanged from STAGING_GRAPH_NECESSITY_EVAL_01:
finite OLD/FRESH Nx3, N>=2, positive FRESH arc, exact saved clocks/hashes and
observation transforms; source timing gates; complete original FRESH safe;
B safe; recorded observation-to-B swept history safe; physical v_minus>.20m/s
AND accumulated travel>=.02m; remaining original arc>=.50m; suffix rows>=5;
complete B_ENTRY safe; defined spatial directions; episode disjointness.

P is the actual saved pose at B_tick-1, authenticated against the saved context.
Use unchanged shortest-wrap convention:
`delta_phi = abs(wrap(direction(B,E*) - direction(P,B)))`.
`d_F` is the unchanged median XY edge length of `[E*, original suffix]`, including
the first partial edge. `rho_jump = ||E*-B||/d_F`. Nonfinite inputs or incoming,
chord or suffix edges <=1e-12m fail closed. No new direction-reliability threshold.

The additional meaningful gate is **chord>=.02m AND rho_jump>=.10**, inclusive
without an extra tolerance. Keep counts before and after this gate. Never rank
by abs(log rho). Direction and jump are independent axes, not a combined score
or a requirement that both be large. First take up to3 distinct episodes by
descending delta_phi; then up to3 remaining episodes by descending rho_jump.
Exact lexical case-ID ties. No outcome-based replacement.

The selector accepts only the frozen closed SelectionRecord fields: case_id,
episode_id, eligibility, delta_phi_rad, rho_jump, B_to_entry_chord_m, d_F_m,
remaining_arc_m. No path/outcome fields or file access exist in the selector.
Tests reject other record types and outcome keyword arguments.

If selected<4, classify INSUFFICIENT_MEANINGFUL_TRANSITION_SOURCES and perform
zero scientific V2 solves and zero rollouts. No threshold changes, prior source
reuse or acquisition. Empty figures would explicitly say no execution evidence.

### Unchanged references and solver

Reuse exact C3_FORWARD_SE2 analytic piecewise shortest-yaw interpolation,
.10m/15deg normalization, C1 XY lower bound and earliest-arc 1e-12 tie handling.
Prepare and validate exact C3 parity, fixed B/E*, exact Native-installed world
suffix bits and original observation-local suffix bits. No raw FRESH mutation.
The Native install is zero-solve authentication, not a Native rollout.

Methods in frozen order:

1. C3_ENTRY_SUFFIX: `[E*, original suffix]`.
2. B_ENTRY_STAGE: `[B, E*, original suffix]`, no optimization or deformation.
3. HERMITE_BRIDGE: existing `hermite_bridge`, chord-scaled unit incoming/outgoing
   tangents, equal spatial arc spacing and shortest-yaw interpolation. Preserve
   `M=max(2,ceil(Hermite_arc/d_F))`; no M tuning.
4. V2_GRAPH_BRIDGE: existing DiagnosticProblem V2, M=2, one editable X1,
   initialized by the existing M=2 Hermite sampler. Fixed B/E*/suffix.

Only derived rows use A^-1 X to install via the unchanged adapter. Downstream
original local rows are copied exactly. Derived installation roundoff is recorded
and checked at1e-12; planning boundaries remain exact.

V2 uses the unchanged vector boundary targets (initial Hermite edge lengths times
original unit tangents), normalized by d_F; existing SE(2) edge smoothness with
(d_F,d_F,10deg) normalization; existing spacing residual divided by d_F; all
weights1. No P->B distance target, controller factor, GP or temporal regularizer.
Right-local LM is unchanged: max80, central finite difference1e-6, damping.001,
reject x10, accept x.3, maximum1e12, gradient/step1e-9 and cost1e-12.
Keep all eight existing post-solve gates, including bridge min-edge **>1e-5m**,
convergence, exact boundaries/suffix, safe full reference and no bridge crossing.
Keep >1e-12 edge domain feasibility and the full-reference safety acceptance.
Do not modify the numerical gate because earlier short bridges failed it.
No repair, projection, fallback, V3, M/weight/solver sweep or retry.

### Safety and logical execution

Use unchanged direct Hospital checker: radius.20m, footprint-edge clearance.05m,
existing workspace/uncertainty reserve1e-7m. C3 checks its suffix only; B_ENTRY,
Hermite and V2 check their entire actual B-to-entry-plus-suffix reference.
Keep abort-only command guard; unsafe proposals remain unapplied and retain the
valid censored prefix. Unsafe/unstable V2 receives SKIPPED_REFERENCE_INVALID.

Use source-specific authenticated common states and periodic schedules through
unchanged `load_source` and `run_relative_factor_multisource01.run_method`.
Restore exact B/A, B tick/time, physical u_minus provenance, applied u_B_plus,
previous_control/memory, generation/version and dt=float32(1/60).
The same saved first post-B completion lag and absolute six-tick grid determine
the source schedule by the existing periodic_schedule rule before execution.
Freeze all pairs and validate attempted/accepted/released/applied ticks, including
abort prefixes; no unexplained mismatch is an acceptable causal comparison.

Official MPC SHA256:
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
H5 nearest+1, Q/R, limits, acceleration bounds, endpoint repetition,
previous_control and stale/generation semantics remain unchanged. A slow solve
pauses simulation; only predetermined logical ticks release its real result.
This is a **timing-controlled offline causal reference comparison**, not a real
asynchronous deployment or latency benchmark. Primary/full windows are54/180
integration intervals. The nominal3s cap uses actual saved dt.

Separate MPC solves, scheduler releases, distinct new results physically carried
by applied integration commands, withheld results, guard-blocked result
applications and integration intervals. The preexisting u_B_plus is restored,
not counted as a new numerical solve/application. A release on an abort tick
is not a physical application. Record all zero acquisition/rerun counts.

### Freeze and one execution

Prepare authenticates sources, selects geometry-only records, saves references,
V2 initial states and schedules, and performs zero-solve preflights. Tests and
synthetic report fixtures are verification, never experimental evidence.
Commit/push code/config/tests/protocol plus source freeze; verify remote SHA and
all frozen blobs/hashes before any new scientific call.

Phase1: one literal V2 planning solve per selected source in E1–E4 order, all
planning before any rollout. Phase2: source order, then C3/B_ENTRY/Hermite/V2.
Exclusive directories and start records reject repeats. Save invalid last
iterates and skip only that method; no result-driven correction/retry.
Saved-only validation recomputes metrics and factor/safety/solver-trace audits
without invoking the optimizer or MPC. Historical implementations stay untouched;
new orchestration imports their geometry, solver, controller and evaluator.

### Evaluation and classification

The primary target is **full ORIGINAL FRESH**. Reuse forward-only continuous
projection, shortest-angle yaw, AUC .3/.9, max XY first.5, initial separation,
attachment row/arc/remaining, endpoint/cap and control/safety metrics.
Sustained attachment: position<=.10m, yaw<=15deg throughout the complete following
.30s sampled execution interval. Original-FRESH endpoint dwell uses the identical
tube/dwell around the original final pose. Null stays null; never substitute cap.
T_post_attach only exists when both dwells are observed. No navigation-goal claim.
Keep abort-only prefixes and missing .9s/cap metrics visibly censored.

BOTH_DWELLS requires both observed with valid planning and safe full-cap execution.
Graph-specific positive requires valid safe V2 BOTH_DWELLS and either:
(a) neither B_ENTRY nor Hermite has BOTH_DWELLS, or (b) all three have both and V2
attaches at least one tick before BOTH while endpoint dwell is no later than BOTH.
One tick threshold=integration_dt_s-1e-9; scalar tolerance=1e-9.
Intermediate positive: B_ENTRY lacks both, Hermite and V2 both have both.
AUC alone never qualifies as either positive.

SHARED_EXECUTION_FEASIBILITY_FAILURE means **all four** methods guard-abort.
It is not V2 regression. Remove it only from the evaluable denominator; keep its
selected-source denominator, metrics, valid prefixes and safety facts visible.
V2-specific regression means unsafe V2 while both simpler references are valid;
V2 guard-abort while neither simpler method guard-aborts; or loss of attachment
or endpoint retained by BOTH safe valid simpler methods. Shared failure suppresses
all such regression flags. Catastrophic regression retains the previous exact
criterion: unsafe V2, attributable guard abort or loss of endpoint; attachment
loss is separately counted. Do not attribute a common failure to V2.

Frozen precedence:

1. TECHNICAL_BLOCKED: authentication, restoration, schedule or validator failure.
2. INSUFFICIENT_MEANINGFUL_TRANSITION_SOURCES: selected<4, no scientific calls.
3. GRAPH_SPECIFIC_GAIN_SUPPORTED: >=2 graph positives, no V2-specific catastrophic
   regression.
4. INTERMEDIATE_GEOMETRY_NEEDED: >=2 intermediate positives; prior rule unmet.
5. STAGING_HARD_TRANSITION_SUPPORTED: **nonempty** evaluable set, all B_ENTRY
   BOTH_DWELLS and graph-positive count exactly zero. An all-shared-failure set
   cannot satisfy this rule vacuously.
6. GRAPH_FORMULATION_NOT_ROBUST: >=2 invalid/nonconverged V2 with BOTH simpler
   methods having safe valid full-cap executions. They need not have dwells.
7. MIXED_EVIDENCE: otherwise.

Report selected/shared/evaluable separately and B_ENTRY successes over both
selected and evaluable denominators. Preserve first official H5 identities,
E* sampled exposure, first original H5 and first differing actual commands from
naturally saved logs via the existing compare_commands. These are descriptive,
not interventions or proof of selector causality.

### Outputs and claim boundary

Raw arrays, execution logs, solver traces and complete input hashes remain under
ignored `data/direct_transition_hard_eval_02/primary_20261006`. Compact JSON/CSV
and exactly3 PNGs live under `results/direct_transition_hard_eval_02`:
`direct_transition_geometry.png`, `execution_outcome.png`,
`severity_vs_method_difference.png`. Equal world axes, visible coincident suffix
rows, no cap-imputed N/A, and shared failures labelled. Figure numeric sidecars
and hashes must match saved data. No HTML dependency.

## Research interpretation

Pending the single frozen execution. No outcome has been used to tune selection,
formulation or classification.

## Not demonstrated

Population generalization; unseen real-world evaluation; asynchronous online VLA
deployment; real robot improvement; navigation task success; semantic intent;
a full editable-FRESH graph; automatic correspondence; graph superiority beyond
this bounded saved-corpus comparison. No subsequent graph is implemented here.

## Pre-science verification and commands

- Full relevant regression suite: **810 passed, 1 skipped**,396.39s. The skip is
  the already-absent ignored EXP-01B/EXP-02B corpus, not a new experiment source.
- Final focused suite: **65 passed**,9.17s, including4 abort-prefix checks added
  after the full suite collected its61 new tests. Thus814 distinct passing cases
  are covered across these runs; historical code was unchanged.
- Prior STAGING_GRAPH_NECESSITY_EVAL_01 saved-only `--check-only`: PASS;
  original selection/metrics/classification/result hashes/figures unchanged.
- `compileall`, working diff and staged diff whitespace checks: PASS.
- Synthetic3-PNG fixture inspected before freeze; synthetic data are not evidence.
-3,988 source/provenance/environment hashes authenticated;4 distinct selected raw
  FRESH arrays. B ticks E1/E2/E3/E4=188/673/189/649; first submit ticks=192/678/192/654;
  releases=193/679/193/655. All source schedules have30 submits;30/29/30/29 releases
  before their fixed caps. Saved B simulation seconds remain in common_state.json.

Commands use the repository `.venv`; no system/external environment is modified.
The exact long regression command is retained in `commands.txt` in the result
folder. Scientific commands, once the freeze is pushed:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_direct_transition_hard_eval02.py --run data/direct_transition_hard_eval_02/primary_20261006 --mode inventory
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_direct_transition_hard_eval02.py --run data/direct_transition_hard_eval_02/primary_20261006 --mode prepare
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_direct_transition_hard_eval02.py --run data/direct_transition_hard_eval_02/primary_20261006 --mode freeze
# Commit and normal push; execute itself authenticates pushed HEAD.
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_direct_transition_hard_eval02.py --run data/direct_transition_hard_eval_02/primary_20261006 --mode execute
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/direct02_mpl .venv/bin/python scripts/validate_direct_transition_hard_eval02.py --run data/direct_transition_hard_eval_02/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/direct02_mpl .venv/bin/python scripts/report_direct_transition_hard_eval02.py --run data/direct_transition_hard_eval_02/primary_20261006
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/direct02_mpl .venv/bin/python scripts/validate_direct_transition_hard_eval02.py --run data/direct_transition_hard_eval_02/primary_20261006 --check-only
.venv/bin/python -m compileall -q src scripts tests
 git diff --check
 git diff --cached --check
```
