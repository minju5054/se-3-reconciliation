# PROJECT-AUDIT-01: native LightNav and reconciliation

## Definition, scope and verdict

Here **native LightNav** means the preserved official LightNav inference,
prompt/preprocessing/history/action decoder and official MPC, embedded in this
repository's robotless Isaac observation/async scheduling/SE(2) integration
collector. It does **not** mean an unmodified end-to-end official robot/demo
deployment. Camera placement, Hospital scene, instruction, observation cadence,
transport and physical execution surrogate belong to the research harness.

**Primary Q2 verdict: PROMISING_BUT_UNPROVEN.** A minority of genuine saved
transitions have substantial reference-relative execution cost, including after
matching intrinsic FRESH turn and speed. Most safe native events attach, next
chunks often recover, and no existing GP/M4 result establishes improved attachment
on the moving hard subset. Necessity, causal attribution and method novelty remain
unproven. Q1's unsafe upstream FRESH is a separate problem, not evidence that a
transition optimizer is needed.

**Q1:** LightNav changes observable output under obstacle interventions, but the
resulting spatial trajectory does not create sufficient lateral clearance in the
tested conditions. The unsafe raw trajectory exists before downstream reference
preparation or MPC. Brightness and persistence affect geometry without recovering
a bypass. **INTERNAL_CAUSE_UNRESOLVED_FROM_AVAILABLE_OBSERVABLES**.

This is a SAVED-ONLY + SOURCE-ONLY audit. New scientific model, MPC, optimizer,
simulator and rollout calls: **zero**. Software regression fixtures are not
experimental evidence. Previous failed, rejected and partial runs remain intact.

Authoritative local run:
`data/project_audit_native_lightnav/audit_20260922T_project01_v3/`.
[Review index](../data/project_audit_native_lightnav/audit_20260922T_project01_v3/index.html),
[machine summary](../data/project_audit_native_lightnav/audit_20260922T_project01_v3/audit_summary.json),
[frozen protocol](../configs/project_audit_native_lightnav.yaml).
Derived artifacts are ignored by Git; source code, configuration, tests and this
report are versioned. No raw RGB, checkpoint, external source or recordings are
copied into the review or commit.

## Provenance and historical modification audit

Initial HEAD and freshly fetched origin/main both were
`6b6cb01d8d44b890dbd60845e139f86de74874ac` on `main`. Remote:
`https://github.com/minju5054/se-3-reconciliation.git`. The two pre-existing edits
to `stage0_jackal_controller_validation.yaml` and
`stage0_lightnav_single_chunk.yaml` are hash-preserved and excluded from this
commit. Final commit/push identity is recorded separately in the ignored run's
`git_completion.json`, avoiding a self-referential tracked hash.

Directly rehashed external checkout:
`/home/gpuadmin/Workspace/external/LightNav-0-official-demo`, clean at
`c6f40e3220edbf7011e4f17eaf2c865416737d4d`.
Checkpoint repository `LightOriginsHQ/LightNav-0`, preserved revision
`7221d418bfff55cfcbadd09f7a26aaab81e1f8a6`; all 16 listed checkpoint files match
all five historical launch manifests. The 9.695 GB weights SHA-256 is
`ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18`.
Evaluation configuration SHA-256:
`7f475edbef4a13d2db99fa12eff5e9dedf48916969ec777f75e6e85e2a492b57`.
This verifies local file identity against preserved acquisition provenance; it
does not independently attest the original remote download transaction.

Official MPC SHA-256:
`2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
Horizon 5, dt 0.1 s, control rate 10 Hz, effective v limit 0.8 m/s,
omega limit 3 rad/s, acceleration limits 2 m/s² and 5 rad/s², Q=(10,10,1),
R=(0.1,0.1). The declared TRACK_V_MAX=1.5 is not the effective solve bound.
Weighted XY/wrapped-yaw nearest selection takes the next five rows, repeats the
endpoint and unwraps yaw sequentially. There are no obstacle, gate or map
constraints in the official MPC. Its previous-command memory updates at solves,
including a computed command that may never be physically applied.

Pipeline: `vln -> vlnce -> vlnce_traj`, `vllm_local`, bfloat16. Source defaults:
temperature=0, top_p=1, top_k=0, traj_top1=false; SOURCE02–04 explicitly set the
generation environment. Original corpus/JOIN01 launch records do not serialize
every inherited environment variable: complete retrospective sampling-setting
attestation is unavailable. Repeated sham equality supports determinism for the
observed pairs, not universal absence of stochastic effects.

Instruction and current/history RGB form the official task prompt. Saved/wire
JPEGs are 480×270, quality 95; official preprocessing stretches to 256×448 with
bilinear resize and maps pixels to [-1,1]. SlowFast sampling uses accumulated
episode history, not a 64-frame ring; nominal history capacity is not actual
sample count. Temporal patch size is 2, with no extra frame subsampling. Three
RVQ codebooks are summed in float32, scaled by joint weights, reshaped to 10×3
SE(2) differences and composed into absolute ego poses; these output rows are
not incremental world displacements. STOP handling is explicit. Waypoint
intrinsic dt remains **null**; MPC dt and later 3 s evaluation timing are separate.

The collector's camera height 0.65 m, 4 Hz observation history, 60 Hz exact
held-command SE(2) integration and async scheduler are harness choices. They are
not identical to the official demo's camera/body offsets (including 0.165 m).
Clean official files therefore cannot rule out a scene/viewpoint domain mismatch.

Compared freezes are original `eed5f2c`, JOIN01 `0de41fa`, SOURCE02 `3e08222`,
SOURCE03 `1904bd2`, SOURCE04 `5f06f07`, import-only correction `7682acd`, and
current `6b6cb01`. Full hashes and scoped adjacent diffs are in
`metrics/historical_component_hashes.csv` and `metrics/history_*.diff`.

<!-- AUDIT_TABLE:historical_modification_matrix -->
| Component | Official / historical behavior | Later research change | Active during obstacle inference? | Could alter raw FRESH? | Evidence |
|---|---|---|---|---|---|
| Official model/task/checkpoint | Official pipeline and pinned files | None in external checkout | Official inference active | No accidental source change found | provenance.json; 16 checkpoint hashes; clean checkout |
| Online model worker/server | Serialize official requests and preserve response | No change across audited freezes | Yes | Inputs can; no changed inference implementation found | scoped history; wire parity |
| RGB/history/instruction | Harness supplies official input contract | Brightness, target pose, H16/H32, last-K presence | Yes, explicitly declared | Yes: intentional input interventions | paired manifests and validators |
| RVQ/action decoding | Official codebooks and SE2 composition | No research replacement | Yes | No observed decode discrepancy | 42 literal token-to-array reproductions |
| Capture transform | World = T_world_obs times ego pose | se2_exp/log small-angle threshold changed 1e-8 to 1e-4 | Conversion itself unchanged | No effect on raw generation or this rigid transform | se2 scoped diff; world parity |
| Reference preparation | Native raw rows installed at capture pose | Suffix/resampling/rigid/GP derived references | Not in source-only inference | No; operates after response | reference and experiment runners |
| MPC worker/adapter | Official source; persistent previous command | Online worker/adapter unchanged | JOIN01 execution only; no SOURCE02–04 controller | No; downstream only | component hashes; official source |
| REF02 selector | Private per-instance offline wrapper | Source-progress lookahead | No online activation | No | gp_se2_ref02_rollout.py and runner |
| Collector intervention | Existing observations, wire and command logs | Optional before/after capture hooks and fresh gate | JOIN01 scene reveal only | Declared image change, not decoder change | collector diff |
| Evaluation | Footprint/map and timing checks | More diagnostics, dense checks and audit metrics | After acquisition | No | separate derived directories |
| Scene | Hospital export plus runtime box/cart | Reveal, placement, lighting interventions | Yes | Through rendered observations | runtime/capture manifests |

## Raw FRESH trace and coordinate/time conventions

The audited trace is Isaac capture → saved JPEG → base64 wire payload → official
server task/history → raw response → APOS/OPOS/action tokens → official RVQ
decode → immutable raw local N×3 → capture-pose world transform → reference
installation → official MPC. Audit representative original event is
`episode_013_repeat_01/handoff_024` (both OLD and FRESH). All obstacle-family
terminal predictions and both JOIN01 chunks are included: **42 raw traces**.
Every raw array equals the preserved response array and official token decode
literally; every world transform agrees at 1e-12 m/rad tolerance. All representative
online episode and JOIN01 JPEGs equal transmitted bytes. Family validators also
reconstruct paired wire/history input equality.

World: Isaac XY metres, +Z up, yaw CCW radians. Ego/body: +x forward, +y left.
`T_world_obs` is the only local-to-world anchor. Neither ready nor B reanchors
FRESH. Classification checks all original ten rows and swept segments before
suffix preparation. A geometrically safe full polyline is not a dynamically
feasible connection from B or a guarantee of safe MPC execution.

B is the pre-integration saved state at the first applied FRESH command.
`u_minus` is the preceding physically applied command; controller previous-control
memory is a separate field. Observation, request, receipt/ready, install and first
application timestamps remain distinct. Host monotonic/UTC clocks and simulation
seconds are never subtracted from one another. The causal timeline figure has
separate clock panels; host receipt is not invented as a simulator timestamp.

## Q1: four separate obstacle experiments

### JOIN01: genuine online reveal

| Attempt | Obs sim s | Ready seen sim s | First FRESH/B sim s | Host request→receipt s | Obs→B travel m | Obs edge clearance m | B edge clearance m | Raw / suffix minimum m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| placement_01 | 1.533333 | 1.800000 | 1.900000 | 0.290080 | 0.293333 | 0.980000 | 0.686667 | -0.200 / -0.200 |
| placement_02 | 1.283333 | 1.550000 | 1.683333 | 0.216171 | 0.266667 | 0.920000 | 0.653333 | -0.200 / -0.200 |
| placement_03 | 1.283333 | 1.550000 | 1.666667 | 0.199467 | 0.256667 | 0.720000 | 0.463333 | -0.200 / -0.200 |

Obstacle reveal and triggering observation share the simulation tick. OLD was
observed at 0.783333 s and first active at 1.216667/1.166667/1.166667 s.
Physical v_minus=0.8 m/s and omega is near zero. Recorded OLD commands remain
active while FRESH is pending. Raw/suffix geometry and the first unsafe row,
segment and arc brackets are preserved in `metrics/q1_recomputed.json`; all
three raw paths physically overlap the introduced box. Original OLD was safe
before reveal (minimum about 1.0145 m) and overlaps after it (-0.2 m).
Maximum OLD/FRESH cross-track differences are only 0.000160226, 0.000047310,
0.000047310 m; projected yaw differences 0.01737°, 0.005289°, 0.005289°.
There is no meaningful detour.

**Was B already unsafe? No. Was a safe turn impossible? Not established.**
Positive B clearance alone is not a reachability proof; no dynamic detour solver
or emergency-stop experiment was run. Raw FRESH intersects the obstacle while
still anchored at observation, so B's later forward travel cannot explain why
that raw output was unsafe. Travel reduces available execution room and is a
separate contributor to attachment/execution difficulty.

Actual history counts were 6–7. Attempt 1 fails the original pacing criterion
(RTF 1.903869); attempts 2/3 pass (1.161577/1.071644). Semantic annotation was
all BACKGROUND, so visibility rests on preserved RGB/runtime geometry, not a
valid class mask. Source qualification failed; no optimizer or common-B method
comparison followed. The short recorded post-switch segment is not a completed
collision rollout. A negative reference clearance is not a fabricated physical
collision.

### SOURCE02: OFF / ON / SHAM

Twelve declared source conditions yielded two complete H16 triplets and ten
unavailable conditions; there are zero confirmation cases. Same 15 past frames,
same terminal pose (19.2046215,23.6233363,-1.5685636), same instruction; only
declared final cart visibility changes. The two observation-to-cart centre
distances are 0.983029 and 1.004575 m, only about 2.15 cm apart. These are not
two independent navigation domains.

OFF and SHAM actions/arrays match exactly. ON changes tokens and geometry, but
both full ON references have clearance -0.2 m. Lateral change is 0.006775 m;
maximum tangent/yaw response is 16.411°/31.046°. Arc decreases from 1.356219 to
0.848790 m; target-directed progress is 0.762572 m, and endpoint remains about
0.487/0.509 m before the cart rear plane. Local side-passage probes retain
positive margins 0.07376/0.0684 m; these are local geometric probes, not a proof
that the generated path or a dynamically reachable detour is safe.

Result: **OUTPUT_CHANGED**, not **SAFE_DETOUR_GENERATED**. All six are terminal
predictions; there is no moving online B or MPC rollout for these triplets.

### SOURCE03: brightness, target, history, observation distance

| Condition | ON full clearance m | ON arc m | First unsafe row (zero-based) | Interpretation |
|---|---:|---:|---:|---|
| core dark H16 | -0.200000 | 0.848790 | 3 | SOURCE02 baseline reproduced |
| core bright H16 | -0.111384 | 0.545469 | 3 | Less overlap, shorter near-straight output |
| target pose H16 | -0.080926 | 0.510888 | 3 | Still unsafe and model target-visible=false |
| target pose H32 | -0.200000 | 0.848790 | 3 | No recovery from more history |
| near centre 0.7 m | -0.200000 | 1.310908 | 1 | Curves right, but early collision |
| medium centre 0.9 m | -0.192836 | 0.545469 | 2 | Still intersects |
| far centre 1.1 m | -0.200000 | 0.848790 | 4 | More safe prefix, no bypass |

All seven ON paths overlap; all seven OFF/SHAM pairs reproduce exactly.
Brightness raises mean luma 12.271445→77.479331 and cart median 12.2→81.37;
core clearance improves 8.8616 cm, largely by shortening, with lateral response
under 7 mm. Only the terminal image is bright here; prior history remains dark.
The near condition ends at local [1.287630,-0.552365] m, yaw -0.890956 rad:
substantial late curvature does not rescue its unsafe early prefix. Side-passage
geometry, exact observed centre distances and first unsafe arc brackets are
retained per condition, not replaced by B distance. Tested 0.7–1.1 m spacing
does not establish that obstacle distance can never matter.

The target pose has 29,221 renderer target pixels, while all 21 model predictions
report target-visible=false. Renderer visibility does not establish model
grounding. H16/H32 share the final JPEG but differ in sampled history/session
positions: this does not isolate internal attention capacity or count alone.
There is no moving online B in these terminal diagnostics.

### SOURCE04: all-bright moving-pose history, fixed world cart

The same 16 saved moving poses were re-rendered bright. Exactly the last K
frames contain the fixed world cart, K={0,1,2,4,8}; OFF and K1 sham predictions
match their originals. Seven terminal predictions and 105 buffer-only requests
are preserved. This is counterfactual observation-history construction, not a
new executed obstacle-reveal episode.

| K | ON-world min clearance m | Raw arc m | Lateral displacement m | First unsafe row / segment |
|---|---:|---:|---:|---|
| 0 OFF | -0.200000 hypothetical cart; actual OFF 0.935417 | 1.356219 | 0.000393 | 3 / 2 in hypothetical cart world |
| 1 | -0.143250 | 0.565197 | 0.006376 | 3 / 2 |
| 2 | -0.080955 | 0.510888 | 0.000214 | 3 / 2 |
| 4 | -0.111384 | 0.545469 | 0.005876 | 3 / 2 |
| 8 | -0.111384 | 0.545469 | 0.005876 | 3 / 2 |

Persistence changes action/yaw/APOS tokens and improves clearance by 3.19–6.23 cm
versus K1, non-monotonically. It mostly changes/shortens a forward trajectory;
no tested K creates a bypass. All ON APOS points are bottom-clamped, OPOS is
not-visible and target-visible=false. The full token strings, pointing, clamping
and row/segment geometry remain in numeric artifacts. A clamped APOS is censored,
not an exact free-space waypoint. Output change alone is not evidence of internal
obstacle understanding. The preserved pre-request import failures were fixed at
the import boundary; they contain zero predictions and are not hidden trials.

## Q1 root-cause evidence matrix

Statuses apply to the stated scope; they are not posterior probabilities.

<!-- AUDIT_TABLE:q1_root_cause_matrix -->
| Candidate explanation | Supporting evidence | Contradicting evidence | Isolating evidence / limitation | Current status |
|---|---|---|---|---|
| Accidental official LightNav modification | Harness differs from official full demo | Clean pinned external source; checkpoint hashes match all launches | Direct source/hash/history audit; not a domain-equivalence test | STRONGLY_DISFAVORED |
| Accidental MPC modification | Research selector experiments exist | Online worker/adapter unchanged; official file identical | Scoped Git history; selector isolated offline | STRONGLY_DISFAVORED |
| Wrapper/decode bug | Custom transport boundary exists | Wire JPEG, response, official RVQ and arrays agree | 42 decoded traces plus family validators; no proof of all possible bugs | STRONGLY_DISFAVORED |
| Coordinate-frame/reanchoring bug | Async observation and B differ | All audited world arrays use observation pose | Explicit transform parity; unchanged conversion | STRONGLY_DISFAVORED |
| FRESH reference preparation/resampling | Later preparation can degrade execution | Unsafe full raw output predates preparation | Full-row safety evaluated before suffix | NOT_APPLICABLE_TO_RAW_FRESH_GENERATION |
| B progressed too close during inference | 0.257–0.293 m travel reduces execution room | B safe; observation-anchored raw already overlaps | JOIN01 temporal order; execution contribution only | NOT_APPLICABLE_TO_RAW_FRESH_GENERATION |
| Obstacle too close at observation | Early unsafe prefix, especially near condition | All 0.7–1.1 m tests fail; local side space exists | SOURCE03 distance intervention; no dynamic feasibility proof | POSSIBLE_BUT_NOT_ISOLATED |
| Short local trajectory horizon | Bright outputs shorten and often end before rear plane | Unsafe early segments also exist; longer near path still collides | No horizon-only intervention or intrinsic waypoint dt | POSSIBLE_BUT_NOT_ISOLATED |
| Low illumination | Bright terminal improves core clearance 8.86 cm | Bright and all-bright history still unsafe | SOURCE03 brightness intervention; contribution to severity only | CONTRIBUTING_FACTOR_SUPPORTED |
| Insufficient history count | JOIN01 history only 6–7 | H16/H32 and all-bright H16 do not recover bypass | Bounded tests; history content/session confounded | DISFAVORED |
| Sudden single-frame reveal | K2 improves clearance over K1 | K4/K8 remain unsafe; effect non-monotonic | SOURCE04 last-K intervention, severity only | CONTRIBUTING_FACTOR_SUPPORTED |
| Target grounding weakness | Visible renderer target but model visible=false | Does not show grounding caused trajectory collision | SOURCE03 pose intervention changes other view content | POSSIBLE_BUT_NOT_ISOLATED |
| APOS/spatial affordance limitation | ON APOS bottom-clamped | Censored APOS cannot locate intended free-space point | No uncensored internal spatial-state evidence | UNRESOLVED |
| Trajectory/action generation limitation | Tokens decode to insufficient lateral clearance | No isolated latent/planner/tokenizer mechanism | Observable failure at output boundary only | POSSIBLE_BUT_NOT_ISOLATED |
| Stochastic sampling | Original inherited environment not fully archived | Greedy defaults/explicit later settings; exact sham equality | Paired repetitions only; not a seed sweep | DISFAVORED |
| Scene/render mismatch | Research camera/scene differ from official deployment | Saved JPEG and activated geometry checks agree | No domain-matched official end-to-end control | POSSIBLE_BUT_NOT_ISOLATED |
| Official MPC lacks independent obstacle avoidance | Source has no map/obstacle constraints; REF04 tracking can violate clearance | Cannot modify a previously generated raw response | Source inspection; execution capability limitation | NOT_APPLICABLE_TO_RAW_FRESH_GENERATION |

## Q2: independent saved execution recomputation

The unchanged source scan reproduces **all 881 ledger rows literally**. Selected
13 use the original safe-full-FRESH/common/B/prefix, moving-B and mismatch filters,
including v_minus>0.2 m/s, obs→B travel≥0.02 m and lateral≥0.1 m or direction/yaw≥20°.
They are seven episodes, not 13 independent experiments. Eleven project to an
interior segment and two to an endpoint; none has retained-suffix obstacle
clearance ≤0.2 m. This is not obstacle-avoidance evidence or a population failure
rate. All 13 saved attachment metrics and execution geometry reproduce literally.

The attachment tube uses original FRESH: distance≤0.10 m and wrapped yaw≤15°
for a sampled 0.30 s dwell. There is no continuous tube guarantee between samples.
The measured window begins at actual B and ends before the next reference's
first applied command, retaining the endpoint state; episode termination is
explicit censoring. Common-window AUC uses the first 54 saved intervals
(0.9000000469 s with resolved dt), never padding or changing the time base.
An endpoint not reached after replacement is not counted as a native failure.

| Group | Events / episodes | Inside tube at B | Sampled sustained attachment | Other observed outcomes | Common position AUC event / episode equal (m·s) |
|---|---:|---:|---:|---|---:|
| All valid source events | 881 / 60 | 709 | 839 | 30 no entry; 7 censored dwell; 5 transient | 0.023676 / 0.020087 |
| Safe entire raw FRESH | 765 / 60 | 637 | 740 | 14 no entry; 6 censored dwell; 5 transient | 0.016350 / 0.015890 |
| Also safe B and observed prefix | 757 / 60 | 632 | 734 | 12 no entry; 6 censored dwell; 5 transient | 0.014818 / 0.014889 |
| Original hard subset | 13 / 7 | 0 | 1 | 7 no entry; 4 censored dwell; 1 transient | 0.173989 / 0.179703 |
| Full seven episode sequences | 189 / 7 | 150 | 169 | 12 no entry; 5 censored dwell; 3 transient | 0.031635 / 0.032630 |

One event in the broad groups has insufficient common-window exposure and is
omitted from AUC only (880/764/756 denominators). Full-window attachment still
includes it. For safe raw events, episode-equal B-inside and attachment fractions
are 84.145% and 97.348%; pooled event fractions are 83.268% and 96.732%.
585/765 have some initial distance growth above the frozen 1 micrometre reporting
floor, but its median is only 0.0000216 m and 90th percentile 0.01094 m.
That count must not be described as 76% harmful transitions. Continuous scatter
plots expose scale rather than substituting a new tuned severity threshold.

758/765 safe-raw events retain executed 0.05 m footprint clearance; the seven
exceptions are explicitly retained, including two physical overlaps and cases
already unsafe at B. Safe reference is not guaranteed safe execution or a claim
that switching caused these exceptions. All 13 selected executions are safe.
The corpus is 60 fixed robotless episodes from 30 starting conditions with two
repetitions, not deployment sampling or real robot evidence.

The hard subset lifetimes are 0.966667–1.550000 s. Only 013/01/029 has a complete
sampled attachment, at 1.016667 s. 013/01/021 enters then exits. For
013/01/024, distance starts 0.279113 m, reaches 0.376956 m near 0.5 s, and ends
0.182382 m; yaw decreases 36.18598°→8.35201°. First-common-window AUC is
0.309502 m·s, mean distance 0.343891 m. No tube entry occurs during its 1.55 s
active lifetime. This is appreciable transient tracking cost, not a collision
or proven loss of final task success.

Command TV, actual first commands, stop/deceleration interval counts, turn
reversals, original-row progress and original-endpoint distance reduction are
reported separately per event. Row progress is dimensionless and is not global
target progress. These are never collapsed into a weighted success score.
The 008/01/023 nominal 0.1 s command-change flag is not proof of excessive
physical acceleration: actual application separation is 0.2 s, and intervening
solver memory includes a computed-but-unapplied command. Physical u_minus and
solver memory must not be conflated.

## Turn versus transition: matched controls and sequence evidence

The YAML rule was frozen before new outcome computation, with prior published
13-case outcomes openly acknowledged. A control must have safe full raw/B/prefix,
|lateral|≤0.05 m, pose yaw and reliable direction mismatch≤10°, same source
category, and a different episode. Calipers: physical speed 0.10 m/s, lifetime
0.25 s, intrinsic raw yaw variation 20°, intrinsic tangent variation 20°, arc
0.40 m. Intrinsic variation is computed only along raw FRESH, independent of B.
Lexicographic yaw/tangent/speed/lifetime/arc differences then case ID choose one
control with replacement. No relaxation, outcome lookup or successful-control
fallback is allowed. The input feature and match ledgers are saved before losses.

**8/13 matched**, spanning four hard episodes and six distinct controls; five
remain unmatched. All eight hard AUCs exceed the matched controls. Mean paired
position-AUC difference is +0.152209 m·s; episode-equal difference is also
+0.152209 m·s (two pairs in each of four hard episodes). All controls have
sampled attachment, starting at B; hard partners have one observed attachment,
four censored dwell entries and three no-entry outcomes. This association is
stronger than merely observing a large necessary turn, but is **not a causal
estimate**: initial error partly defines the selected group and contributes
mechanically to AUC; lifetime is realized exposure and may be post-treatment;
route intent, exact world position, OLD history, yaw sign and MPC memory are not
fully balanced. Control reuse and same starting-condition repeats also limit
independence. No significance test or generalization probability is reported.

At the identical boundary pose, OLD-to-new reference distance increases in all
13 cases (0.0421–0.2562 m). For 013/01/024 it jumps 0.022870→0.279113 m without
physical pose change, establishing a reference-switch discontinuity. It does not
show that the new intended route is wrong or that a feasible connector can avoid
all of the transient.

Every saved applied interval from all seven episodes is available through the
source execution CSVs and the audit's sequence artifacts. The 189 handoff windows
are evaluated against their contemporaneous original FRESH. Figures include
active reference IDs, observation poses, B, executed XY, v/omega, switch markers,
tracking error and projection progress. Initial pre-first-handoff execution is
shown as context, not assigned an invented handoff metric.

| Event / active chunk | Lifetime s | OLD error at B m | New error at B m | End error m | Original-FRESH outcome |
|---|---:|---:|---:|---:|---|
| 013/01/022 / C023 | 1.366667 | 0.154128 | 0.049942 | 0.070603 | Sampled attachment |
| 013/01/023 / C024 | 1.333333 | 0.070603 | 0.029173 | 0.022870 | Sampled attachment |
| 013/01/024 / C025 | 1.550000 | 0.022870 | 0.279113 | 0.182382 | No entry |
| 013/01/025 / C026 | 1.466667 | 0.182382 | 0.000352 | 0.088501 | Sampled attachment |
| 013/01/026 / C027 | 1.316667 | 0.088501 | 0.005929 | 0.000960 | Sampled attachment |

The next reference can largely remove the *measured* mismatch at unchanged pose:
C026 begins 0.000352 m away where C025 was 0.182382 m away. It does not physically
erase past cost or establish arrival at C025's endpoint. Of 13 hard cases, 12
have a next saved handoff: 10 next boundaries are already in the joint tube, and
11 next windows show sampled attachment (one transient). One episode ends and
cannot answer the next-chunk question. Therefore neither “every chunk causes a
fresh failure” nor “next chunks never fix it” is supported. There are recurrent
localized transients amid mostly well-attached execution.

## Existing method and baseline evidence

Historical counterfactual 3 s experiments hold a selected reference past its
actual native replacement time and do not advance simulation during compute.
Their goal/dwell criteria answer a different question from actual active-lifetime
attachment. They remain useful paired interface/solver diagnostics, not direct
online improvement evidence. The following results are audited from preserved
reports/source, not new model/controller/optimizer calls.

<!-- AUDIT_TABLE:method_evidence_matrix -->
| Method / experiment | Saved execution evidence | Localized issue | Implication |
|---|---|---|---|
| M0_NATIVE | GP01 6/10 successes; GP02 hard 1/3; benign succeeds | Some native references difficult, but native also outperforms prepared versions | Baseline cannot be assumed inadequate |
| M0_ADAPTER / REF01 | GP01 7/10; GP02 hard 0/3; resample-only large-turn final yaw 37.30°, adapter 38.83° vs native 0.333° | Resampling changes nearest-row lookahead, not model planning | Preparation can create apparent reconciliation need; suffix alone succeeded there |
| REF02 selector | Same 30 rows; large-turn yaw 0.255°; benign goal time 2.59→1.28 s; higher TV | Reference-consumption semantics can recover selected cases | Simple interface fix is mandatory comparator |
| REF03 / REF04 transfer | Additional 24 distinct episodes: nearest/raw A 23, dense B 21, progress C 23 successes; known stress A fail/B pass/C fail | Both selector recoveries were already native successes; predicted MPC tail unsafe despite safe selected targets | No general selector gain; map-free MPC does not guarantee clearance |
| M1_RIGID | GP01 7/10; GP02 hard 0/3; large-turn final yaw 45.446° | Global deformation can hurt intent and execution | No robust gain over native demonstrated |
| M2 / M3 GP and DIAG01–08 | GP01 0/10 available GP candidates; GP02 M2 one valid hard candidate but no hard success; M3 none | Feasibility/derivative/collocation and tracking-goal gaps; lower objective not execution gain | Numerical improvements are real but insufficient for contribution claim |
| ATTACH M4 | One nearly stationary event: native/adapter attach 0.155 s, M3 0.165 s, M4 0.170 s; goal native 0.77 vs M4 1.17 s | Linear TV 1.6→1.02895, angular TV slightly higher, error worse | Smoother linear commands did not produce faster attachment; no moving-hard validation |
| Simple splice / reference governor | No controlled common-B moving-hard comparison in saved evidence | A central non-triviality baseline remains absent | Optimization necessity unresolved |

REF04's known stress localizes a controller/environment mismatch: safe selected
targets do not prevent unsafe predicted tail at t=0; the first applied unsafe
prediction appears at 0.2 s, with executed clearance violation bracket
[0.261875,0.2625] s and a gate crossing 15.158 mm outside. There is no hidden
obstacle avoidance term to rescue arbitrary reference transitions.

DIAG01 recovered a feasible benign seed without objective improvement. DIAG02
verified derivatives and converged in about 1.74–2.13 s on that benign case,
reducing objective 95.10%, without proving execution improvement. DIAG03 showed
collocation-pass/interior-fail motion violations. DIAG04 added quarter equalities
and regressed numerically; DIAG05 recovered convergence but not full validity;
DIAG06 removed observed nonlateral violations while hard lateral failure remained.
DIAG07 executed explicitly rejected hard references diagnostically: safe motion
but final position about 0.16993 m exceeds the unchanged 0.15 m threshold.
DIAG08's fixed 4 cm endpoint reserve introduced other motion failures and admitted
no hard rollout. Rejected plans remain rejected, and repeated starts of one
event are not independent success cases.

ATTACH01 source 021/01/003 has v≈1.8e-9 m/s, travel 0.008435 m and gap 0.141201 m
at the along-path endpoint, not a moving lateral discrepancy. M4 T=1.4 s is
valid, but this source cannot validate moving-B reconciliation. The T=1.0 s
nominal tube check excluded an approximately 1.7e-11 m excess without relaxing
tolerance; that conservative numerical edge is disclosed, not silently repaired.

## What current SE(2) optimization represents

Current GP formulation uses 31 support poses/body twists over fixed 3 s, 0.1 s
spacing, with 150 free chart variables after fixing X(0)=B and
nu(0)=[physical v_minus,0,omega_minus]. Original raw FRESH is immutable; a
derived, corresponded/resampled future is editable. The objective combines a
whitened GP prior with absolute SE(2) residuals to F_common. It does not contain
an explicit relative-FRESH-motion preservation factor; shape is indirectly
regularized. Motion, workspace and terminal constraints exist; M3 adds obstacle
constraints. Finite enforcement/full checks are not a continuous safety theorem.
M4 removes pre-T preservation terms and imposes a sampled planned attachment tube
at a fixed supplied correspondence/T. No correspondence factor or new method was
implemented in this audit.

<!-- AUDIT_TABLE:q2_contribution_matrix -->
| Observed execution loss | Current formulation can represent? | Objective directly targets it? | Constraints address it? | Evidence of improved execution? | Missing mechanism |
|---|---|---|---|---|---|
| Position/yaw mismatch at actual B | Yes; fixed B and editable SE2 future | Absolute F_common deviation, not actual tracking AUC | X0=B; M4 planned attachment samples | None on moving hard subset | Execution-aware cost/verification against original FRESH |
| Physical command discontinuity | Initial twist represents physical u_minus | GP prior indirectly regularizes | Sampled twist/acceleration bounds | M4 linear TV reduction only; attachment slower | MPC memory and actual applied-command relationship |
| Initial error growth / slow attachment | A connector can geometrically reduce discrepancy | No executed time-to-attach or AUC term | M4 planned tube, not executed tube | No positive attachment-speed result | Closed-loop tracking and active reference lifetime |
| Required navigation turn | SE2 poses can retain it | Absolute future residual, indirect shape prior | Goal/route and workspace | Resampling/rigid/GP can worsen turn tracking | Preserve route intent without rewarding turn removal |
| Unsafe generated FRESH | Could deform a path if constraints feasible | Does not recover perception/intent | M3 obstacles and map checks | No native reveal bypass source qualified | Separate upstream/local-planning problem; valid future prerequisite |
| MPC leaves safe reference | Plan variables only | No MPC-in-objective | Planned clearance does not bind tracking error | REF04 counterexample | Controller-aware feasibility or verified execution margins |
| Inference plus optimization delay | Boundary can use latest available B | No online compute-time objective | No movement during optimization in saved rollouts | No online optimization benefit | Delay-inclusive boundary/committed execution handling |
| Minimal FRESH deformation | Absolute pose residual and GP prior | Yes for derived F_common; no proof of minimum feasible execution change | Fixed correspondence/T | Objective improvements only | Original-FRESH deformation/shape metrics and simple-baseline benefit |

The defensible research target is: given actual B, physical u_minus, a fixed
externally supplied correspondence and a **valid FRESH future**, find a feasible
small deformation that reduces actual attachment cost to **ORIGINAL FRESH** while
preserving its intended route. Necessary 90° turns must remain. Report attachment
time, same-window XY/yaw AUC, growth, progress, safety, deformation and compute
latency separately. Smoothness or closeness to the optimized path itself is not
the primary goal. Fixed T/correspondence is a legitimate bounded scope, but cannot
support automatic correspondence or time-optimality claims.

## Closest-work audit (primary sources, accessed 2026-09-22)

This is a scoped novelty comparison, not an exhaustive priority review. “Not
explicit” means the inspected mechanism does not provide that constraint; it
does not claim the system never encounters obstacles. Comparisons to our proposed
scope are inferences from the sources, not measured head-to-head results.

<!-- AUDIT_TABLE:closest_work_matrix -->
| Work / primary source | Input → output | Latency model / OLD–new combination | Optimize SE2? | Preserve committed OLD? | Preserve new intent? | Obstacle constraints? | Execution evidence | Difference / novelty limitation |
|---|---|---|---|---|---|---|---|---|
| [RTC](https://arxiv.org/html/2506.07339v1) | Observation + previous timed actions → chunk | Delay prefix fixed; decaying overlap guidance during diffusion/flow inpainting | No explicit SE2 graph | Yes, frozen unavailable prefix | New policy conditioning; not hard raw-trajectory preservation | Not explicit geometric constraints | Real robot and delay tests | Already addresses async transition continuity without retraining; external constrained waypoint repair must show added value |
| [TIC-VLA](https://arxiv.org/html/2602.02459v2) | Cached semantic KV + current vision/state → actions | Latency plus elapsed-time/egomotion offsets; temporal attention and latency-consistent training | No external SE2 optimizer | Not an explicit committed-prefix equality | Learned semantics and current inputs | No explicit map-constrained SE2 repair | Dynamic navigation simulation and real robot | Rich internal model interface/training differs from black-box pose chunks; latency compensation itself is not novel |
| [AsyncVLA](https://arxiv.org/html/2602.13476v1) | Remote semantics + delayed/current RGB → onboard 2D pose chunks | Edge adapter handles network/inference staleness; learned reactive adjustment | No factor-graph SE2 repair | Not explicit prefix equality | Learned adaptation, not raw FRESH equality | Learned behavior; no inspected explicit geometric guarantee | Real navigation with network delay | Same practical delay problem, different learned model-side solution |
| [ACT / ALOHA](https://tonyzhaozh.github.io/aloha/aloha.pdf) | Images/state → joint-action chunks | Temporal ensemble of overlapping action predictions | No | Past actions remain executed; future ensemble not committed-prefix constraint | Weighted overlapping predictions | Not explicit geometry | Real manipulation | Cheap blend/ensemble is a continuity comparator; not directly timed LightNav SE2 rows |
| [Apollo trajectory stitching](https://apollo.baidu.com/docs/apollo/9.0/classapollo_1_1planning_1_1TrajectoryStitcher.html) | Vehicle state/time + previous timed path → stitching prefix | Time/position matching, preserved points, offset-triggered reinitialization | Stitcher itself no SE2 optimization | Retains selected previous prefix; replan fallback | New planner joins prefix; no raw VLA future guarantee | Stitcher itself no obstacle optimizer | Production source; not a matched VLA benchmark | Joining successive plans at current state is established; fixed-correspondence splice must be compared |
| [Reference/command governors](https://merl.com/publications/docs/TR2016-102.pdf) | Desired reference + closed-loop state → modified reference | Feedback/prediction-based constrained reference modification | General systems, not specifically SE2 | Causally applied past immutable; future input adjusted | Minimize/reference-limit deviation | State/input constraints can encode safety | Survey includes practical applications | Closest external-controller interface concept; current formulation omits closed-loop prediction used here |
| [GPMP2](https://www.roboticsproceedings.org/rss12/p01.pdf) | Start/goal/environment → smooth trajectory | Sparse GP prior, obstacle factors, incremental replanning | General configuration-space trajectory graph | Boundary conditions; no async VLA commitment mechanism | Goal/prior, not supplied FRESH semantic preservation | Yes | Motion-planning experiments, not this async VLA task | GP/factor-graph constrained smoothing alone is established |
| [Lie-group GP trajectory optimization](https://dongjing3309.github.io/files/Dong18icra.pdf) | Lie-group states/boundaries/environment → trajectory | Continuous GP representation with sparse optimization | Matrix Lie groups, includes rigid motion | Supplied boundaries, not VLA delayed-prefix semantics | No supplied raw FRESH intent guarantee | Trajectory planning constraints/factors | Planning demonstrations | SE2/Lie-group GP representation alone is not a new contribution |

A possible distinction is external, model-agnostic, constraint-aware minimal
repair of untimed observation-anchored SE(2) chunks at actual application B,
with fixed correspondence and original-FRESH execution metrics. Current evidence
does not yet show that this distinction produces a useful advantage over simple
stitching/reference management, or that it is substantively novel enough for a
paper. No first/unique/state-of-the-art claim is justified.

## Contribution assessment and one minimal next experiment

Problem reality: **supported for a selected minority**, in a robotless saved
online corpus. Attribution: **partial**; boundary discontinuities and matched
intrinsic-turn controls support a transition association, not avoidable causal
loss. Non-triviality: **unresolved**; selector/interface fixes solve some prior
failures, and a simple connector remains untested on the moving subset. Method
distinction: **plausible but overlaps established stitching/governor/GP work**.
Actual benefit: **not demonstrated** for attachment. Generality: seven episodes
support observation, but repeated development on 013/01/024 and a stationary M4
example do not establish method generalization. Claim discipline is achievable
only by keeping upstream safety, perception, physical robots and correspondence
outside demonstrated claims.

**Exactly one proposed next experiment, not run:** freeze a paired common-B
reference-interface benchmark on all 13 already-selected moving cases plus their
six distinct frozen controls. Compare native rows, the existing source-progress
selector, one predeclared fixed-correspondence simple SE(2) splice, and unchanged
existing M4 (unavailable/rejected candidates count explicitly). Use identical
saved B, physical u_minus, separate MPC memory, official controller, source goal,
map, and each actual native reference lifetime. No new source search, VLA calls,
weights, thresholds or retries. Charge measured preparation/optimization delay
as OLD continuation before application, preserving committed execution; report
any method unavailable before the next scheduled switch. Primary paired outputs
are original-FRESH XY/yaw AUC and sampled attachment, with safety/progress and
deformation gates, event and episode summaries, no scalar weighted score.
This single method-comparison experiment directly asks whether the remaining
cost is avoidable by a simple interface correction and whether current M4 adds
value. It is a proposed counterfactual controller experiment, not online robot
evidence; implementing its simple comparator requires separate authorization.

## Repository claim audit

Historical reports are not silently rewritten. Most already state limitations
carefully. “Too strong” rows below are tempting cross-report inferences, not
fabricated quotations attributed to a historical author. Exact claim scopes
and locations are preserved here and in the CSV.

<!-- AUDIT_TABLE:claim_audit -->
| Source / statement under review | Classification | Evidence | Correct wording / retained limit |
|---|---|---|---|
| README: official model/controller preserved | SUPPORTED_BUT_LIMITED | Source and checkpoint hashes; research harness differs | Official inference/decoder/MPC within custom robotless harness; full deployment equivalence not established |
| JOIN01: native FRESH is unsafe after reveal | VERIFIED | Three full raw and suffix clearances -0.2 m | Tested source-reference geometry, not three completed collision rollouts |
| SOURCE02: obstacle response changed but unsafe | VERIFIED | Paired tokens/arrays differ; ON overlap; OFF sham exact | Observable response does not establish obstacle understanding |
| SOURCE03: brighter scene does not recover bypass | VERIFIED | All tested ON paths overlap | Brightness contributes to severity; tested configurations only |
| SOURCE04: persistence does not recover bypass | VERIFIED | K1/2/4/8 overlap; near-straight shorter paths | More history persistence is not sufficient in this fixed bank |
| APOS as exact spatial affordance target | TOO_STRONG | ON APOS bottom-clamped | Censored location; exact affordance intention unresolved |
| GENUINE_SOURCE scan: 13 moving safe-FRESH mismatches | VERIFIED | Literal 881-row reproduction with unchanged filters | Selected subset, not prevalence estimate |
| SAVED_HANDOFF loss: 1/13 sampled joins | VERIFIED | All 13 outcomes and geometry reproduced | Applies during actual recorded reference lifetimes, with censoring |
| Inferring most native chunks fail from 13-case result | TOO_STRONG | 740/765 safe raw events attach in broad saved audit | Minority selected transients; most safe events attach |
| GP01/02: optimizer benefit not demonstrated | VERIFIED | No robust hard execution gain, candidate failures | Maintain negative and unavailable outcomes |
| REF01/02: reference consumption affects execution | VERIFIED | Resample-only regression and selector-only recovery | Specific interface mechanism, not universal selector superiority |
| DIAG02 objective decrease as execution improvement | TOO_STRONG | Numerical objective only; later execution evidence separate | Verified numerical improvement on benign event |
| ATTACH01 smoother commands / faster attachment | SUPPORTED_BUT_LIMITED | Linear TV improves; attachment 0.170 vs 0.155 s | Smoother linear command TV, no faster attachment or lower overall cost |
| Safe FRESH implies safe MPC execution | TOO_STRONG | REF04 and seven broad-corpus exceptions | Full raw polyline safety does not guarantee connection/tracking safety |
| Native LightNav obstacle avoidance as universal capability/failure | AMBIGUOUS | Limited Hospital interventions; no internal causal isolation | Report each tested scene/input condition and observed geometry |
| SE2/GP/factor graph establishes paper contribution | TOO_STRONG | Existing GPMP/RTC/stitching/governor work; no execution advantage | Research hypothesis with unresolved novelty and benefit |
| Online optimization improvement / real robot benefit | TOO_STRONG | Existing optimization rollouts are offline counterfactual | No such demonstration exists in this evidence |
| Earlier fixed-3s failure as current active-lifetime failure | OUTDATED | Native references replaced after about 1–1.5 s | Keep historical 3s result but use active-lifetime attachment for this question |

## Integrity, limitations and reproducibility

The core audit verifies 12,073 input hashes before/after its computation, including
all listed checkpoint files, audited family records, source scan inputs and
source/config/document inventory. It reproduces 42 decode chains, 881 source
records and the 13 historical loss/safety results. This is direct current
identity and consistency evidence, not proof that an unrecorded historical file
was never edited. Preserved acquisition hashes and untouched Git scopes provide
the historical chain. Source/output manifests link every derived figure to its
numeric inputs, protocol and units. XY geometry has equal axis scaling.

Concerns, including minor ones:

- Original generation environment is incompletely archived; later explicit
  settings and sham pairs do not retrospectively fill the gap.
- Scene/viewpoint domain, all-BACKGROUND reveal masks, target grounding and
  censored APOS remain limitations despite valid wire/geometry provenance.
- Terminal SOURCE02–04 tests cannot answer moving-B execution questions. JOIN01
  attempt 1 fails real-time pacing, and its short post-switch execution is limited.
- Known Hospital footprint geometry uses radius 0.20 m, required edge clearance
  0.05 m and projected geometry; it is not a physical robot collision oracle.
- Safe reference and unsafe execution, geometric feasibility and dynamic
  reachability, plan validity and diagnostic execution are separate predicates.
- Original selection deliberately conditions on mismatch and safety; current
  matching is retrospective with known published outcomes, reused controls,
  realized lifetimes and incomplete route/controller covariates. No held-out
  claim, IID inference or statistical significance is made.
- Many diagnostics repeatedly use one hard event. SOURCE02 has two nearby
  complete conditions and no confirmations; unavailable/rejected cases remain
  visible. No thresholds were relaxed to obtain a positive result.
- Model rows are untimed. First 54 simulator intervals are slightly over 0.9 s;
  dwell and observed entry are sampled/censored, not exact continuous events.
- Nominal controller dt and actual application interval differ; computed-but-
  unapplied memory must be considered before blaming physical acceleration.
- Fixed 3 s historical rollouts pause simulated movement during computation and
  outlast native reference replacement. They cannot substantiate online speedup.
- M4 T=1.0 nominal-bound roundoff is preserved; no acceptance repair was introduced.
- Two incomplete audit output directories remain: the first exposed an evaluator
  schema field (`worker_exit_code`), the second duplicate feature/outcome
  `end_reason` keys. Both were audit serialization issues, before completed
  outcome publication; neither triggered inference, altered thresholds or
  changed historical evidence. The successful v3 verifies end-reason agreement.
- README/WORK_LOG hashes in the core input manifest describe the pre-audit source
  snapshot at 6b6cb01. The required new WORK_LOG append is an explicit later
  documentary edit, not raw-input drift; final completion records distinguish it.

Reproduce with an unused output directory (existing output is refused):

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/project_audit_mpl .venv/bin/python scripts/audit_project_native_lightnav.py --output data/project_audit_native_lightnav/NEW_RUN
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/project_audit_mpl .venv/bin/python scripts/report_project_native_lightnav.py --run data/project_audit_native_lightnav/NEW_RUN
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q --deselect tests/test_gp_se2_rollout.py::test_real_official_historical_solve_reproducibility_when_available
.venv/bin/python -m compileall -q src scripts tests
git diff --check
```

The deselected historical integration test would execute the real official MPC,
which this audit prohibits. Numerical solver unit fixtures in the regression
suite remain software tests, not new scientific experiments. Focused audit,
SE(2), timing and saved-loss tests: **70 passed**, including three added protocol,
presentation and forbidden-call checks. The full regression run before these
three additional tests had **2,619 passed, 19 skipped, 1 deliberately deselected**
and two sandbox-only AF_UNIX socket failures; those exact two tests then passed
outside the socket restriction (2 passed). Thus all 2,624 exercised tests pass
across the full/focused/IPC runs; no scientific integration test was substituted.
The 19 existing skips concern unavailable historical fixtures or a declared
representation diagnostic. Independent corpus validation passes for 60 episodes
and 881 valid handoffs; SOURCE02/03/04 pass **1,507 / 2,019 / 4,686** checks.
These are artifact/correctness validations, not evidence of method benefit.

Claims supportable today: immutable official inference/controller provenance
within the stated harness; insufficient obstacle clearance under the tested
interventions; a selected minority of native reference transitions with measurable
transient cost; partial turn-controlled association; dominant native recovery in
the broad corpus; and no demonstrated current optimization attachment advantage.
Claims not supportable: internal neural cause, universal obstacle incapacity,
graph necessity, faster attachment/online improvement, real-robot benefit,
automatic correspondence, continuous safety, or established paper novelty.
