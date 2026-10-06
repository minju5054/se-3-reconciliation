# DIRECT_TRANSITION_HARD_EVAL_02

**Result: STAGING_HARD_TRANSITION_SUPPORTED.** B_ENTRY has both dwells in 4/4
evaluable sources; graph-specific and intermediate positives are both 0. C3 still
reaches original-FRESH endpoint dwell earlier in all four sources.

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


## Repository-confirmed execution results

Scientific freeze: `8a42e3973587eea1fd6e63f2ee240609479787af`, normally pushed and
verified against origin/main before any scientific call. Execution completed on
2026-10-06. All frozen 843 code/config/test hashes, 138 prepared inputs and 3,988
source/provenance/environment hashes remain unchanged. Final saved-only validation
and figure sidecar/hash checks pass. Historical results were not rerun or edited.

### Counts, denominators and classification

| Quantity | Saved result |
|---|---:|
| Saved events inspected | 881 |
| Excluded recorded episodes (includes previous E1–E6) | 39 |
| Base eligible before meaningful gate | 90 |
| Meaningful eligible candidates / episodes | 4 / 4 |
| Selected / shared safety failure / evaluable | 4 / 0 / 4 |
| B_ENTRY BOTH_DWELLS over selected / evaluable | 4/4 / 4/4 |
| Graph-specific / intermediate positives | 0 / 0 |
| V2-specific regressions | 0 |
| V2 solves / converged / reference-valid | 4 / 4 / 4 |
| V2 retries / invalid skips / nonconvergence | 0 / 0 / 0 |
| C3 / B_ENTRY / Hermite / V2 rollouts | 4 / 4 / 4 / 4 |
| Official MPC solves | 480 |
| Logical releases / physical new-result applications | 472 / 472 |
| Results withheld at cap / guard-blocked applications | 8 / 0 |
| Applied integration intervals | 2880 |
| Controller numerical failures / safety aborts | 0 / 0 |
| LightNav / RGB / Isaac / new acquisition / historical reruns | 0 / 0 / 0 / 0 / 0 |

All 16 rollouts terminate OBSERVATION_CAP after 180 intervals; actual cap is
3.0000001564621925s. All primary/full logical schedules and common-state/memory
checks pass. The 8 withheld results are the cap-ending solve in E2/E4 for each
method; they are solved but never applied. No censored scientific rollout,
missing dwell or failed reference is hidden in this result.
Frozen classification: **STAGING_HARD_TRANSITION_SUPPORTED**.

### Transition and downstream metrics

All times are actual seconds after B. AUC position units are m·s; yaw AUC units
are rad·s. Values below are rounded to 9 decimal places; CSV/JSON preserve the
saved floating-point values. A zero attachment is an observed dwell beginning at
B, not missing data. E2–E4 have zero attachment for all methods.

| Source | Method | Position AUC .3 | Position AUC .9 | Yaw AUC .3 | Yaw AUC .9 | T_attach s |
|---|---|---|---|---|---|---|
| E1 | C3 | 0.025283379 | 0.087231440 | 0.065456349 | 0.103839863 | 0.716666704 |
| E1 | B_ENTRY | 0.023948687 | 0.082096545 | 0.066946253 | 0.102576395 | 0.616666699 |
| E1 | Hermite | 0.024213861 | 0.090278874 | 0.074287274 | 0.122050263 | 0.900000047 |
| E1 | V2 | 0.024176967 | 0.084943712 | 0.071961600 | 0.110635528 | 0.716666704 |
| E2 | C3 | 0.018809105 | 0.056820725 | 0.031193089 | 0.058796210 | 0.000000000 |
| E2 | B_ENTRY | 0.018632748 | 0.058065860 | 0.034563037 | 0.060167525 | 0.000000000 |
| E2 | Hermite | 0.018863349 | 0.061563834 | 0.038947605 | 0.065401770 | 0.000000000 |
| E2 | V2 | 0.018863355 | 0.061563959 | 0.038947723 | 0.065401934 | 0.000000000 |
| E3 | C3 | 0.020545208 | 0.055546467 | 0.013471263 | 0.043940685 | 0.000000000 |
| E3 | B_ENTRY | 0.020473984 | 0.055381064 | 0.013521959 | 0.043917067 | 0.000000000 |
| E3 | Hermite | 0.020560072 | 0.056806923 | 0.014719617 | 0.044039705 | 0.000000000 |
| E3 | V2 | 0.020560080 | 0.056806994 | 0.014719688 | 0.044039750 | 0.000000000 |
| E4 | C3 | 0.012205005 | 0.035541755 | 0.018904664 | 0.038771254 | 0.000000000 |
| E4 | B_ENTRY | 0.012148935 | 0.036553020 | 0.021320588 | 0.040031712 | 0.000000000 |
| E4 | Hermite | 0.012252648 | 0.037896369 | 0.023151306 | 0.042072181 | 0.000000000 |
| E4 | V2 | 0.012252649 | 0.037896386 | 0.023151328 | 0.042072207 | 0.000000000 |

| Source | Method | Attach row | Attach arc fraction | Arc left m | Endpoint dwell s | Post-attach s | Cap error m |
|---|---|---|---|---|---|---|---|
| E1 | C3 | 4.631946080 | 0.512727063 | 0.661280233 | 1.450000076 | 0.733333372 | 0.047143937 |
| E1 | B_ENTRY | 3.580120003 | 0.396477722 | 0.819042723 | 1.550000081 | 0.933333382 | 0.042247435 |
| E1 | Hermite | 5.064575823 | 0.560596874 | 0.596315904 | 1.566666748 | 0.666666701 | 0.050168114 |
| E1 | V2 | 4.103884851 | 0.454307017 | 0.740562333 | 1.550000081 | 0.833333377 | 0.044405777 |
| E2 | C3 | 2.608703515 | 0.287039710 | 0.968173458 | 1.133333392 | 1.133333392 | 0.039124078 |
| E2 | B_ENTRY | 2.608703515 | 0.287039710 | 0.968173458 | 1.166666728 | 1.166666728 | 0.041104026 |
| E2 | Hermite | 2.608703515 | 0.287039710 | 0.968173458 | 1.166666728 | 1.166666728 | 0.044906474 |
| E2 | V2 | 2.608703515 | 0.287039710 | 0.968173458 | 1.166666728 | 1.166666728 | 0.044906624 |
| E3 | C3 | 1.082061005 | 0.120442240 | 1.191091747 | 1.383333405 | 1.383333405 | 0.031983958 |
| E3 | B_ENTRY | 1.082061005 | 0.120442240 | 1.191091747 | 1.400000073 | 1.400000073 | 0.026863271 |
| E3 | Hermite | 1.082061005 | 0.120442240 | 1.191091747 | 1.416666741 | 1.416666741 | 0.028631669 |
| E3 | V2 | 1.082061005 | 0.120442240 | 1.191091747 | 1.416666741 | 1.416666741 | 0.028631719 |
| E4 | C3 | 1.995137357 | 0.225367117 | 1.031363385 | 1.183333395 | 1.183333395 | 0.024432880 |
| E4 | B_ENTRY | 1.995137357 | 0.225367117 | 1.031363385 | 1.200000063 | 1.200000063 | 0.026796924 |
| E4 | Hermite | 1.995137357 | 0.225367117 | 1.031363385 | 1.200000063 | 1.200000063 | 0.027752635 |
| E4 | V2 | 1.995137357 | 0.225367117 | 1.031363385 | 1.200000063 | 1.200000063 | 0.027752647 |

`primary.csv` also preserves initial XY/yaw, maximum XY in first .5 s, initial
separation growth, first endpoint tube entry, path length and mean commanded
speed before endpoint dwell, original projected progress/remaining arc at cap,
and all abort/termination fields. These use the unchanged original-FRESH evaluator.
No method-reference metric replaces the primary target.

### Safety and control

All complete references pass the unchanged checker. Minimum reference clearance
across the16 references is 0.3593838391311876m. Minimum swept execution clearance
across the16 rollouts is 0.38401638804443955m, above the required .05 m. There are
zero safety abort ticks and zero unsafe applied commands. The observed generous
clearance does not establish obstacle generalization.

| Source | Method | Swept clearance m | Linear TV | Angular TV | Max abs(v) m/s | Max abs(omega) rad/s |
|---|---|---|---|---|---|---|
| E1 | C3 | 2.537979796 | 0.800000012 | 3.034602379 | 0.800000000 | 1.455500336 |
| E1 | B_ENTRY | 2.537995094 | 1.600000032 | 3.214469925 | 0.800000000 | 1.206340539 |
| E1 | Hermite | 2.537988565 | 1.600000046 | 2.985862078 | 0.800000000 | 1.139803951 |
| E1 | V2 | 2.537993222 | 1.600000035 | 2.700230256 | 0.800000000 | 1.296230670 |
| E2 | C3 | 0.566258863 | 1.020028881 | 2.305058399 | 0.800000000 | 1.127767667 |
| E2 | B_ENTRY | 0.566263994 | 1.420028904 | 1.989961852 | 0.800000000 | 0.967275161 |
| E2 | Hermite | 0.566263143 | 1.420028907 | 2.329088882 | 0.800000000 | 0.998688695 |
| E2 | V2 | 0.566263144 | 1.420028907 | 2.329089571 | 0.800000000 | 0.998682762 |
| E3 | C3 | 0.396152141 | 0.799999996 | 1.443018274 | 0.800000000 | 0.776921665 |
| E3 | B_ENTRY | 0.395481424 | 0.983307099 | 1.432638381 | 0.800000000 | 0.776921665 |
| E3 | Hermite | 0.396714166 | 1.200000007 | 1.453867623 | 0.800000000 | 0.776921665 |
| E3 | V2 | 0.396714246 | 1.200000007 | 1.453886570 | 0.800000000 | 0.776921665 |
| E4 | C3 | 0.384016388 | 0.800000009 | 1.259677864 | 0.800000000 | 0.608791098 |
| E4 | B_ENTRY | 0.384739716 | 1.200000020 | 1.304985634 | 0.800000000 | 0.520465292 |
| E4 | Hermite | 0.386081179 | 1.200000023 | 1.679581569 | 0.800000000 | 0.554579416 |
| E4 | V2 | 0.386081195 | 1.200000023 | 1.679586155 | 0.800000000 | 0.554579984 |

### Planning diagnostics

Hermite M=2 and V2 M=2 for every source. All V2 stability gates pass. All
solves use 2 accepted steps, no rejected steps and no unsafe improving proposal.
E1–E3 terminate on step tolerance; E4 on cost tolerance. The existing edge cutoff
remains strictly >1e-5m. No solver setting, reference repair or retry changed.

| Source | Initial total | E_in | E_out | E_smooth | E_space | Final total | Min bridge edge m |
|---|---|---|---|---|---|---|---|
| E1 | 0.130643656 | 0.066324450 | 0.063585012 | 0.000157987 | 0.000044148 | 0.130111596 | 0.025358902 |
| E2 | 0.110296470 | 0.056492193 | 0.053509565 | 0.000070323 | 0.000026945 | 0.110099026 | 0.023502590 |
| E3 | 0.175231966 | 0.089526656 | 0.085426631 | 0.000067637 | 0.000028198 | 0.175049122 | 0.030077047 |
| E4 | 0.045559576 | 0.023098132 | 0.022404824 | 0.000012623 | 0.000004574 | 0.045520154 | 0.015282029 |

All downstream original suffix rows coincide exactly. The following differences
are descriptive saved-array comparisons; no additional solve was made.

| Source | Max Hermite–V2 execution XY difference m | Max Hermite–V2 bridge row XY difference m |
|---|---|---|
| E1 | 0.011799985 | 0.001411666 |
| E2 | 0.000000230 | 0.000855217 |
| E3 | 0.000000114 | 0.000825712 |
| E4 | 0.000000028 | 0.000378975 |

E2–E4 Hermite/V2 maximum executed XY differences are approximately 2.30e-7,
1.14e-7 and 2.76e-8 m. E1 differs by 0.01179998457835085m, with different attachment
threshold-crossing times. These small observed differences are not repeatability
or robustness estimates. Factor-cost improvement alone is not execution benefit.

### What the official selector consumed

All methods share each source's exact state at its first submit. The first
submitted result already differs between C3/B_ENTRY, B_ENTRY/Hermite and
B_ENTRY/V2; it first applies at the frozen release tick. No state is advanced by
host solve time. Threshold for command-difference diagnostics remains1e-9.

| Source | Method | First official H5 identities | Submits including E* | First original H5 submit tick |
|---|---|---|---|---|
| E1 | C3 | F_2, F_3, F_4, F_5, F_6 | 0 | 192 |
| E1 | B_ENTRY | E*, F_1, F_2, F_3, F_4 | 2 | 204 |
| E1 | Hermite | bridge_1, E*, F_1, F_2, F_3 | 2 | 204 |
| E1 | V2 | bridge_1, E*, F_1, F_2, F_3 | 2 | 204 |
| E2 | C3 | F_4, F_5, F_6, F_7, F_8 | 0 | 678 |
| E2 | B_ENTRY | E*, F_3, F_4, F_5, F_6 | 1 | 684 |
| E2 | Hermite | bridge_1, E*, F_3, F_4, F_5 | 1 | 684 |
| E2 | V2 | bridge_1, E*, F_3, F_4, F_5 | 1 | 684 |
| E3 | C3 | F_2, F_3, F_4, F_5, F_6 | 0 | 192 |
| E3 | B_ENTRY | E*, F_2, F_3, F_4, F_5 | 1 | 198 |
| E3 | Hermite | bridge_1, E*, F_2, F_3, F_4 | 1 | 198 |
| E3 | V2 | bridge_1, E*, F_2, F_3, F_4 | 1 | 198 |
| E4 | C3 | F_3, F_4, F_5, F_6, F_7 | 0 | 654 |
| E4 | B_ENTRY | E*, F_2, F_3, F_4, F_5 | 1 | 660 |
| E4 | Hermite | bridge_1, E*, F_2, F_3, F_4 | 1 | 660 |
| E4 | V2 | bridge_1, E*, F_2, F_3, F_4 | 1 | 660 |

C3 never exposes E* in a sampled successful H5 on these sources. B_ENTRY begins
with E* in H5; Hermite/V2 begin with bridge_1 then E*. The rows after E* are exact
original identities. First differing submit ticks are 192/678/192/654 and first
differing application ticks 193/679/193/655 for E1/E2/E3/E4. `selector_exposure.csv`
and result_summary.json retain first H5 poses, sampled E* exposure, first original
row and first different submitted/applied commands. These records describe actual
controller inputs; they do not identify a unique causal mechanism for the outcomes.

### Trade-offs without a scalar winner

- E1 B_ENTRY attaches 0.10000000521540642s earlier than C3 but reaches endpoint
  dwell 0.10000000521540642s later. It has 0.1577624894980182m more original arc
  left at attachment. Fast attachment is not fast endpoint dwell.
- E1 V2 improves over Hermite: attachment earlier by 0.18333334289491177s and
  endpoint dwell earlier by 0.01666666753590107s. It still attaches 0.10000000521540642s
  later than B_ENTRY and only ties B_ENTRY endpoint dwell; no graph-specific positive.
- E2–E4 all methods already satisfy sustained attachment at B. V2 and Hermite
  endpoint dwell times are identical. B_ENTRY ties them in E2/E4 and precedes
  them by one tick in E3. A recovery claim is inappropriate for these sources.
- B_ENTRY position AUC.9 is lower than both bridges in all 4 sources. Compared to
  C3 it is lower in E1/E3 and higher in E2/E4. C3 endpoint dwell precedes B_ENTRY
  by 6/2/1/1 ticks across E1–E4, respectively.

## Research interpretation

Under the tested saved-corpus conditions, explicit B-aware staging remains
sufficient even after excluding near-zero B→E* transition artifacts. No
graph-specific necessity is supported.

This conclusion concerns the frozen BOTH_DWELLS criterion, not universal speed
or optimality. All methods have both dwells, so the experiment establishes no
new recovery advantage for staging or graph. B_ENTRY has lower early position
AUC than either bridge and preserves both dwells; C3 has earlier endpoint dwell
in every source. V2's E1 improvement over Hermite does not exceed the simpler
B_ENTRY comparison required by the frozen graph-positive rule.

The corrected gate removes the micrometre-gap selection artifact. It does not
guarantee hard execution: three selected sources are attached from B, and all
rho_jump values remain below 1. Only 4 episodes survive the fixed gates. This is
useful negative evidence about graph necessity within this available subset,
not proof that meaningful geometry never needs intermediate reconciliation.
No new graph or future study is implemented here.

## Post-execution validation, artifacts and deviations

Saved-only validation reproduces source selection, initial state, exact C3/raw
suffix, complete reference safety, V2 costs and accepted/rejected trace semantics,
official selector/memory, every integration state, logical schedule, guard and
original-FRESH/endpoint metrics. Hash and numeric-sidecar `--check-only` passes.
Validation performs zero new scientific solves or rollouts.

Exactly 3 final PNGs were generated and inspected:

1. [Direct-transition geometry](../results/direct_transition_hard_eval_02/figures/direct_transition_geometry.png)
2. [Execution outcome](../results/direct_transition_hard_eval_02/figures/execution_outcome.png)
3. [Severity versus method differences](../results/direct_transition_hard_eval_02/figures/severity_vs_method_difference.png)

[Primary CSV](../results/direct_transition_hard_eval_02/primary.csv),
[result summary](../results/direct_transition_hard_eval_02/result_summary.json),
[call accounting](../results/direct_transition_hard_eval_02/call_accounting.json)
and [validation](../results/direct_transition_hard_eval_02/validation.json) provide
machine-readable facts. Large arrays/logs remain ignored under data/.

No scientific protocol deviation: one solve/source, one rollout/method/source,
no retry/tuning/acquisition, unchanged classifier and historical implementations.
After visual inspection, a separate saved-only presentation script added scatter
label padding/alternating offsets to prevent overlap. It leaves the frozen
reporter, all numeric sidecars and all scientific hashes unchanged; the final
figure manifest records its hash. No extra final PNG is retained.

Claim boundaries remain those listed above: no population, real-world, real robot,
async VLA, navigation-task-success, semantic-intent, full editable-FRESH-graph or
automatic-correspondence claim. Normal freeze and result commits/pushes are used;
no history rewrite or environment modification.
