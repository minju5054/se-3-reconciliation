# GP-SE2-REF-01: suffix selection and resampling factor isolation

This fixed diagnostic asks which FRESH preparation operation changes official
MPC targets and execution on the GP-SE2-02 large-turn regression, with one benign
control. It separates input changes, matched-state selection, closed-loop commands
and original goal/dwell outcomes. No GP/rigid optimization, new VLA input,
controller modification, horizon extension or preprocessing repair is performed.

## Frozen source and design

Starting main and fetched origin/main are
`47668b868e84173fab4516ab7d5edb65ef75b841`. Both unrelated Stage 0 configuration
edits remain untouched. Authoritative GP-SE2-02 and GP-SE2-01 validations and
published artifact hashes are verified; their raw/derived arrays, results,
images, validated environment and pinned external MPC are preserved.

Primary source is `data/robotless_gp_se2_02/primary_20260919T024000Z/`; original
reference/configuration source resolves to
`data/robotless_gp_se2_01/primary_20260918T054000Z/`. The environment is the
validated original copy identified by that provenance, not a regenerated map.
New output is `data/robotless_gp_se2_ref_01/primary_20260919T062000Z/`.

| Case/order | Event | Native rows N | Original nearest j | Fixed suffix k | Removed original rows |
|---|---|---:|---:|---:|---|
| PRIMARY_LARGE_TURN | episode_014_repeat_01/handoff_026 | 10 | 0 | 1 | 0 |
| BENIGN_CONTROL | episode_001_repeat_01/handoff_002 | 10 | 1 | 2 | 0, 1 |

Original `prepare_reference` determines k once from F_native at original B using
the original weighted XY/wrapped-yaw distance. It is never recalculated on a
resampled array or recut externally during rollout. For F=F_native and S=F[k:]:

| Variant | Suffix factor | Resample factor | Reference | Large-turn / benign rows |
|---|---|---|---|---|
| R00_NATIVE | off | off | F | 10 / 10 |
| R10_SUFFIX_ONLY | on | off | S | 9 / 8 |
| R01_RESAMPLE_ONLY | off | on | R(F) | 30 / 30 |
| R11_CURRENT_ADAPTER | on | on | R(S), identical to F_common | 30 / 30 |

R uses unchanged `interpolate_rows`: linear XY, shortest-angle yaw, source row
times linspace(0.1,3.0,L), queries arange(1,31)*0.1, original constant-reference
behavior for L=1. These are interpolation conventions; intrinsic LightNav
waypoint timing is null and the MPC receives only pose rows. All references keep
the original world frame and goal endpoint. Periodic endpoint pose equivalence
and literal bitwise equality are separately recorded.

Row provenance preserves original left/right indices, alpha, fractional row
coordinate, XY, wrapped/unwrapped yaw and original/interpolated identity. Row
progress is neither time nor distance. Geometry audits retain removed prefix
poses, duplicate/rotation rows, corner/yaw knots, path length and yaw variation.
Bidirectional point-to-polyline distances use at most 1 mm sample spacing with
at least 100 subdivisions per nonzero segment; they are sampled diagnostics,
not continuous geometric equivalence proofs. Near-zero translation is a
predeclared 1 mm diagnostic, separate from the 1e-12 m rotation-only classification.

## Official selection and execution semantics

The pinned MPC has HORIZON=5, MPC_DT_S=0.1 and CONTROL_RATE_HZ=10. It selects the
five rows following the nearest weighted pose, clamps to/repeats the final row,
and sequentially unwraps each selected yaw relative to the preceding yaw,
starting at the current pose. Gains, objective, limits, tie-breaking and this
next-row policy are unchanged. Local row numbers across variants have different
meanings, so comparisons use original fractional coordinates and world poses.

All 30 GP-SE2-02 Native control input poses per case are frozen probe states.
The official selector and independent `selection_audit` evaluate all four
references at each identical state: 240 selector calls, no state integration,
memory update, tracker construction or MPC solve. All probes precede primary
rollouts. Logs include per-row XY/yaw cost contributions, tie margins, source
mapping, actual unwrapped target yaw, physical lookahead and goal-row inclusion.

Primary order is large-turn then benign, each R00,R10,R01,R11 once. Each fresh
official tracker starts at original B with original physical u_minus and
recorded previous_control separately restored. World rows pass through the
inverse original capture transform and official path installation, with
roundtrip checks; never a B reanchor. Original counterfactual_rollout and
_synchronous_solve are reused unchanged. Each rollout has 30 solves at
0,0.1,...,2.9 s, 180 exact held-command unicycle ticks at 60 Hz and 181 states
through 3 s. Total planned primary solves are 240; historical/other solves zero.
Failure/hold/collision handling is unchanged. Wall time never advances simulation.

R00/R11 compare against the original stored Native/Adapter results. Before
primary, selected-reference tolerance is fixed at 1e-12, command tolerance at
1e-6 (original audit values), and accumulated state/endpoint diagnostic tolerance
at 1e-6 absolute metres/radians with rtol=0. Indices, original inputs/settings
and success predicates must agree exactly. Literal bitwise equality is reported
separately. These diagnostic tolerances do not change physical acceptance.

## Evaluation and interpretation

Original evaluate_rollout supplies unchanged footprint/clearance/workspace,
motion, original route, 0.15 m position, 15-degree yaw and final 0.20 s dwell
predicates. `terminal_goal_dwell_s` remains required duration; a separately named
sampled achieved dwell diagnostic is added. First goal entry/time-to-goal stays
N/A if absent. Commands, acceleration and yaw come from newly applied MPC controls,
not reference derivatives. Three-second failure does not imply permanent failure.

Each continuous metric reports all four values and five contrasts: suffix
without/with resampling, resampling without/with suffix, and
y11-y10-y01+y00 interaction. Nulls remain null. Observation flags use a frozen
1e-6 native-unit numerical floor, not a practical-benefit or significance
threshold. Success flags are shown separately, not combined into a score.
The two dependent development-corpus events do not support population inference.

Full results, implementation SHA, independent validation, figures and measured
mechanism are appended only after actual execution. Static evidence is primary;
no new GUI is planned for this narrow diagnostic.
