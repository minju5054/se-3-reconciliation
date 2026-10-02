# B_TO_ENTRY_BRIDGE_01

## Frozen protocol

Starting fetched HEAD / origin/main: `50b4fd7afdbb7183ab03b4222fe12514d293fb3f`.
**Timing-controlled offline causal reference comparison.** Four-source development
mechanism experiment, with S3 as the unresolved case and S1/S2/S4 as controls.
Historical Native and C3 Entry-Suffix execution records are authenticated and
reused without modification or rerun. No Full/Half execution is requested.

| ID | Frozen source |
|---|---|
| S1 | OSA03_R00 |
| S2 | episode_001_repeat_01/handoff_013 |
| S3 | episode_008_repeat_01/handoff_023 |
| S4 | episode_013_repeat_00/handoff_020 |

The parent result summary SHA256 is
`e86083eb2f812e84c898885f2039b6520d336dd802c391dcaf29342cf17aa948`.
Its complete result ledger, input/code chain, and saved-only entry, correspondence,
transport, multisource, R00 and R01 validators must pass. Official MPC source SHA256
remains `2de99fdf75b60c6836a645ae995c686417c93a5ccbc7df6d8b984d20c1d83ce1`.
Run identifier: `data/b_to_entry_bridge_01/primary_20261002T020000Z`.
Actual processing UTC timestamps are recorded separately from this identifier.

### Fixed frames, entry and downstream identities

World XY metres, yaw radians counterclockwise, +Z up; observation-local x forward,
y left. Raw FRESH stays observation-anchored, `F=A F_local`; A and B stay fixed.
C3 is reproduced by the unchanged frozen continuous correspondence implementation,
including its tie and wrapped-yaw handling, and must exactly equal the prior record.

P is the actual recorded pose at B_tick-1, authenticated through the original
execution records and the saved OLD-to-B array. Both endpoint poses B and E* are
copied bit-for-bit into every bridge. The complete reference is
`[B, interior bridge rows, E*, original downstream suffix]`. B is a derived boundary
row, not a raw FRESH row. Downstream world rows are copied from the exact historical
Native `restoration.installed_world` array. Their original observation-local raw
rows are passed unchanged to the existing official installation path, reproducing
those installed downstream bits exactly. Only derived bridge rows are represented
as `A^-1 X` for this interface. A world/local round trip can cause <=1e-12 numerical
roundoff on derived installed bridge rows; fixed planning B/E bits are never
optimized or replaced. That representation roundoff is reported explicitly.

### Hermite baseline and frozen density

Let `u_in=normalize(B.xy-P.xy)`, `u_out=normalize(F_next.xy-E*.xy)`, and
`d_BE=||E*.xy-B.xy||`. Cubic Hermite XY endpoint derivatives are exactly
`m0=d_BE u_in`, `m1=d_BE u_out`. There is no tangent-magnitude tuning.
The retained suffix for the row scale is `[E*, original downstream poses]`;
`d_F` is the median of its XY segment lengths, including its first partial segment.

Compute continuous Hermite arc length with SciPy quadrature, absolute/relative
error tolerances 1e-12, limit 200, requiring error estimate <=1e-10. Freeze
`M=max(2,ceil(L_H/d_F))` before any graph solve or rollout. Invert cumulative arc
with Brent root finding (`xtol=1e-14`, `rtol=4*float64 epsilon`), and sample M equal
arc intervals. Shortest-angle yaw B→E follows normalized arc. Boundary bits are
copied after interpolation. Curve parameter and arc fraction are spatial, untimed
quantities; there is no waypoint timestamp or waypoint dt.

### SE(2) graph

Only X_1..X_(M-1) vary. X_0=B and X_M=E* are fixed. Initialize exactly from the
frozen Hermite bridge. The downstream suffix is excluded from variables.

- Incoming: `wrap(direction(B,X_1)-direction(P,B))/15 deg`.
- Outgoing: `wrap(direction(X_(M-1),E*)-direction(E*,F_next))/15 deg`.
- Smoothness: `D_j=X_j^-1 X_(j+1)`;
  `Log(D_(j-1)^-1 D_j)` divided by `(d_F,d_F,10 deg)` for j=1..M-1.
- Spacing: `(d_(j+1)-d_j)/d_F` for consecutive bridge edge lengths.

Each factor cost is the unaveraged sum of its squared normalized residual
components. All four lambda values are one; total cost has no half factor.
Spatial smoothness/spacing do not represent acceleration/speed. P→B supplies
only direction; its timed displacement magnitude is absent from the objective.
Spatial yaw deformation does not directly command robot angular velocity.

Use unchanged right-local LM: max80, central FD1e-6, damping .001, increase10,
decrease.3, maximum1e12, gradient/step tolerances1e-9, cost tolerance1e-12.
Every improving proposal must pass the existing direct complete-polyline checker
and have bridge XY edges >1e-12 m so endpoint directions are defined. Collapsed
proposal edges have finite atan2(0,0) residual evaluation but are rejected by this
domain check; no additional penalty or smoothness term is introduced.
No repair, projection, resampling or smoothing follows optimization. Nonconvergence
or a numerical solver exception is recorded with no graph rollout and classified
TECHNICAL_BLOCKED. Unsafe initial/returned reference is recorded as a reference
failure, without repair or retuning.

### Execution, safety and metrics

Freeze exact common B state/clock, physical u_minus provenance, already-applied
u_B_plus, controller previous_control, generation/version and original dt. Reuse
the exact source-specific transport-experiment schedule bytes. The unchanged
logical scheduler waits in wall time at each submitted state, and releases results
only at the predetermined tick. Official MPC remains H=5, nearest+1, unchanged
Q/R, limits, acceleration bounds, memory and stale-result behavior. This is not
real asynchronous deployment timing or a latency benchmark.

Full reference safety now explicitly includes B→bridge→E*→suffix: circular
radius .20 m, required footprint-edge clearance .05 m, unchanged Hospital/cart,
workspace and 1e-7 m numerical reserve. The abort-only execution guard remains
unchanged; unsafe commands are unapplied and valid prefixes are retained.
Compare attempted/accepted submit ticks, application ticks, integration counts
and initial provenance through both 54 and 180 intervals. Abort cases remain
censored; any unexplained schedule mismatch is TECHNICAL_BLOCKED.

Reuse the exact original-FRESH continuous forward projection and shortest-yaw
metrics: position/yaw AUC .3/.9, attachment (.10 m/15 deg for complete following
.30 s), attachment original row/arc fractions and remaining arc. Reuse the exact
original-FRESH endpoint dwell metric and null policy from the parent experiment.
Report T_endpoint-T_attach only if both are observed, endpoint error at the
historical 180-step nominal 3 s cap, swept clearance, command TV, max |v|/|omega|,
termination and abort. Attachment and endpoint dwell are distinct execution
outcomes. Neither gives LightNav rows timestamps or establishes navigation task
completion. Own-reference metrics remain secondary.

Report bridge arc/chord/ratio, chord turning max/RMS, min/max segment length,
separate factor costs, full reference clearance, polyline self-intersection,
and first official H5 selected bridge/original identities. Self-intersection uses
Shapely LineString.is_simple and is diagnostic only. Bridge nodes have bridge
identities; do not assign them fabricated original-FRESH progress identities.

### Predeclared interpretation

No single score. Use the requested categories with this fixed precedence:

1. TECHNICAL_BLOCKED: authentication, restoration, solver, validator or unexplained
   schedule failure.
2. BRIDGE_REFERENCE_FAILURE: unsafe reference, official numerical controller failure
   after successful restoration, or guard abort attributable to the new reference.
3. BRIDGE_TRADEOFF: S3 transition/recovery improves but any easy source loses an
   observed endpoint dwell or its dwell is >.10 s later than C3. Safety failures
   take precedence as BRIDGE_REFERENCE_FAILURE.
4. GRAPH_BRIDGE_SUPPORTED: Graph recovers S3 attachment/endpoint beyond C3/Hermite,
   with no easy-source endpoint loss or clear completion regression. Additional
   recovery means a dwell when Hermite has neither, or endpoint dwell when Hermite
   lacks endpoint dwell. Faster already-observed attachment alone is insufficient.
5. SIMPLE_BRIDGE_SUFFICIENT: Hermite recovers S3 attachment or endpoint, Graph adds
   no such recovery, and Hermite has no later easy-source endpoint beyond 1e-9 s.
6. BRIDGE_PARTIAL_ONLY: S3 .9 position AUC is lower by >1e-9 m s, but neither bridge
   obtains S3 attachment/endpoint and there is no major easy-source regression.
7. BRIDGE_INSUFFICIENT: remaining cases without meaningful supported recovery.

The AUC sign threshold is numerical, not a claim of practical effect size; report
absolute and relative magnitudes, .3 AUC and yaw tradeoffs too. Compare against C3
for mechanism effects and retain Native as historical context. Any improvement
without preserved safety is never support. No post-hoc category or tuned cutoff.

### Freeze and outputs

Authenticate → construct/save Hermite and entries/M → zero-solve official preflight
→ focused/regression tests → review → commit/normal push → record freeze SHA.
Then source order S1..S4, each with one Hermite rollout, one graph solve and one
eligible graph rollout. Maximum/exact expected: 4 graph solves, 8 new rollouts;
no retry, Native/C3 rerun, LightNav, RGB, Isaac, source search or instruction change.
Synthetic tests are not scientific evidence or scientific budget calls.

Saved-only validators recompute metrics, factor residuals and safety without a
new optimizer or controller solve. Small result JSON/CSV and exactly four PNGs are
tracked; large arrays and runtime traces remain under ignored data. Final PNGs:
`world_execution_overview.png`, `bridge_geometry.png`,
`transition_and_completion_metrics.png`, `attachment_vs_completion.png`.
No HTML dependency or extra final PNGs. Plot equal-axis XY and distinguish overlaps.

## Repository-confirmed facts

Scientific freeze **`274c44b550c4e37164d963c30d7927030b637e54`** was normally pushed
before any scientific solve or rollout. Overall classification: **TECHNICAL_BLOCKED**.
All four Graph solves reached the frozen 80-iteration limit without convergence.
The predeclared failure policy withheld all four Graph rollouts. Their execution
metrics are **N/A because no rollout occurred**, distinct from Native/C3 S3 nulls
where 3 s of execution occurred without a complete dwell. No last iterate was
silently substituted for a returned Graph reference.

Exactly **4 graph planning attempts, 0 converged graph plans, 4 Hermite rollouts,
0 Graph rollouts, 120 official MPC solves, 119 new result applications**. The final
S2 result was withheld beyond the frozen cap, as in the historical schedule.
Eight historical Native/C3 rollouts were authenticated and reused byte-for-byte.
Native/C3 reruns=0; LightNav=0; RGB=0; Isaac=0; retries=0. Each new Hermite rollout
completed all 180 intervals without a controller error or safety abort.

All executed Native/C3/Hermite comparisons match the source-specific common state
and attempted/accepted submit and application sequences through both 54 and 180
intervals. The four-method gate is false solely because Graph is absent. It is
not an observed timing mismatch among the three executed/reused methods.
No execution inference about Graph is available.

The saved-only validator passes, including the six historical experiment
validators, exact fixed boundaries/suffix, source hashes, command-memory and
stale/generation behavior, safety, original-FRESH evaluation and independent
endpoint-dwell parity. Its `valid=true` means the records are consistent; the
experiment classification remains TECHNICAL_BLOCKED. Pre-freeze tests:
**29 focused passed; 600 regression passed, 1 skipped** (missing ignored historical
EXP-01B/EXP-02B corpus). Compileall and diff checks pass. No scientific code or
configuration changed after the pushed freeze.

### Exact frozen C3 entries

These are world poses with yaw in radians. Entries exactly reproduce the prior
diagnostic; no retuning or source change.

| Source | l* m | Original arc fraction | Segment | beta | E* [x,y,yaw] |
| --- | --- | --- | --- | --- | --- |
| S1 | 0.18474273839738128 | 0.1365182708172272 | 1.000000000 | 0.22582990794399482 | [19.29589449218289, 24.196826321126334, -1.0453675393771789] |
| S2 | 0.3245809061559166 | 0.23958430386696167 | 2.000000000 | 0.15302563680147266 | [18.747621139721804, 9.891583390188735, -1.708655151798606] |
| S3 | 0.46193137914572885 | 0.3662231723212672 | 2.000000000 | 0.9725845948085713 | [17.89727685765006, 31.99213199452277, -0.3242954780329643] |
| S4 | 0.3392920662039436 | 0.264393095332806 | 2.000000000 | 0.22054154941072646 | [19.516333811852085, 9.044315276940615, -1.8652517646249573] |

### Original-FRESH transition and downstream dwell

Native and C3 are reused; Hermite is new; Graph has no execution. Fractions below
are original XY arc fractions at sustained attachment. All times are actual
execution seconds after B. N/A is never replaced by the cap. Full machine
precision and original fractional row identities are in `primary.csv`.

| Source | Method | Pos AUC .9 m s | T_attach s | Arc fraction at attach | Remaining m | T_endpoint s | T_post s | Endpoint error at 3 s m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | Native | 0.169065633 | 1.466666743 | 0.968605162 | 0.042484924 | 1.516666746 | 0.050000003 | 0.096439021 |
| S1 | C3 | 0.169065632 | 1.466666743 | 0.968605161 | 0.042484925 | 1.516666746 | 0.050000003 | 0.096439021 |
| S1 | Hermite | 0.157503975 | 1.466666743 | 0.857096753 | 0.193383179 | 1.666666754 | 0.200000010 | 0.077380067 |
| S1 | Graph | N/A | N/A | N/A | N/A | N/A | N/A | N/A |
| S2 | Native | 0.131833111 | 1.050000055 | 0.854497878 | 0.197121472 | 1.250000065 | 0.200000010 | 0.079382367 |
| S2 | C3 | 0.131833111 | 1.050000055 | 0.854497878 | 0.197121472 | 1.250000065 | 0.200000010 | 0.079382367 |
| S2 | Hermite | 0.132991597 | 1.100000057 | 0.825298436 | 0.236679912 | 1.350000070 | 0.250000013 | 0.076037005 |
| S2 | Graph | N/A | N/A | N/A | N/A | N/A | N/A | N/A |
| S3 | Native | 0.189624122 | N/A | N/A | N/A | N/A | N/A | 0.115238019 |
| S3 | C3 | 0.186553953 | N/A | N/A | N/A | N/A | N/A | 0.112354395 |
| S3 | Hermite | 0.182253441 | 1.466666743 | 0.767945556 | 0.292699199 | 1.766666759 | 0.300000016 | 0.060442205 |
| S3 | Graph | N/A | N/A | N/A | N/A | N/A | N/A | N/A |
| S4 | Native | 0.198189216 | 1.283333400 | 0.912753530 | 0.111962209 | 1.400000073 | 0.116666673 | 0.084539314 |
| S4 | C3 | 0.198189216 | 1.283333400 | 0.912753530 | 0.111962209 | 1.400000073 | 0.116666673 | 0.084539314 |
| S4 | Hermite | 0.158973357 | 1.050000055 | 0.637306486 | 0.465439657 | 1.550000081 | 0.500000026 | 0.030448754 |
| S4 | Graph | N/A | N/A | N/A | N/A | N/A | N/A | N/A |

### Position and yaw AUC

Yaw AUC uses shortest-angle error in rad s. These primary errors are measured
against the full ORIGINAL FRESH. Own-reference diagnostics are kept separately
in `result_summary.json`.

| Source | Method | Position .3 | Position .9 | Yaw .3 | Yaw .9 |
| --- | --- | --- | --- | --- | --- |
| S1 | Native | 0.047920362 | 0.169065633 | 0.122763625 | 0.193368049 |
| S1 | C3 | 0.047920362 | 0.169065632 | 0.122763625 | 0.193368049 |
| S1 | Hermite | 0.046197811 | 0.157503975 | 0.124983476 | 0.196380785 |
| S1 | Graph | N/A | N/A | N/A | N/A |
| S2 | Native | 0.045302491 | 0.131833111 | 0.045624120 | 0.107103978 |
| S2 | C3 | 0.045302491 | 0.131833111 | 0.045624120 | 0.107103978 |
| S2 | Hermite | 0.044751346 | 0.132991597 | 0.049715665 | 0.103789439 |
| S2 | Graph | N/A | N/A | N/A | N/A |
| S3 | Native | 0.064363537 | 0.189624122 | 0.076190590 | 0.175616043 |
| S3 | C3 | 0.063699869 | 0.186553953 | 0.074971489 | 0.173208231 |
| S3 | Hermite | 0.060019535 | 0.182253441 | 0.075087771 | 0.123986312 |
| S3 | Graph | N/A | N/A | N/A | N/A |
| S4 | Native | 0.061198617 | 0.198189216 | 0.120660784 | 0.230599468 |
| S4 | C3 | 0.061198617 | 0.198189216 | 0.120660784 | 0.230599468 |
| S4 | Hermite | 0.054386064 | 0.158973357 | 0.112389488 | 0.205457145 |
| S4 | Graph | N/A | N/A | N/A | N/A |

### Control, safety and termination

| Source | Method | Swept clearance m | Linear TV | Angular TV | max abs(v) m/s | max abs(omega) rad/s | Termination |
| --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | Native | 0.133561094 | 0.800000000 | 3.629555771 | 0.800000000 | 1.548333658 | OBSERVATION_CAP |
| S1 | C3 | 0.133561095 | 0.800000016 | 3.629555770 | 0.800000000 | 1.548333658 | OBSERVATION_CAP |
| S1 | Hermite | 0.152790687 | 2.000000061 | 3.803257746 | 0.800000000 | 1.242649202 | OBSERVATION_CAP |
| S1 | Graph | N/A | N/A | N/A | N/A | N/A | PLANNING_FAILED |
| S2 | Native | 0.757224041 | 0.806041069 | 2.546626244 | 0.800000000 | 1.052845289 | OBSERVATION_CAP |
| S2 | C3 | 0.757224041 | 0.806041069 | 2.546626244 | 0.800000000 | 1.052845289 | OBSERVATION_CAP |
| S2 | Hermite | 0.757234605 | 1.606041099 | 2.869934475 | 0.800000000 | 0.896706981 | OBSERVATION_CAP |
| S2 | Graph | N/A | N/A | N/A | N/A | N/A | PLANNING_FAILED |
| S3 | Native | 0.380788848 | 1.306411557 | 4.091433300 | 0.800000000 | 1.970742708 | OBSERVATION_CAP |
| S3 | C3 | 0.380789059 | 1.306411561 | 4.053281894 | 0.800000000 | 1.963120875 | OBSERVATION_CAP |
| S3 | Hermite | 0.380828239 | 2.435623192 | 4.117208335 | 0.800000000 | 1.236641469 | OBSERVATION_CAP |
| S3 | Graph | N/A | N/A | N/A | N/A | N/A | PLANNING_FAILED |
| S4 | Native | 0.870803818 | 1.453529666 | 4.825967444 | 0.800000000 | 2.271346994 | OBSERVATION_CAP |
| S4 | C3 | 0.870803818 | 1.453529666 | 4.825967444 | 0.800000000 | 2.271346994 | OBSERVATION_CAP |
| S4 | Hermite | 0.872961422 | 2.222189933 | 4.116810390 | 0.800000000 | 1.624542830 | OBSERVATION_CAP |
| S4 | Graph | N/A | N/A | N/A | N/A | N/A | PLANNING_FAILED |

All new executed prefixes passed the unchanged guard; no unsafe command was
applied. Hermite's minimum swept bound across the four sources is
0.15279068650318456 m, above the required .05 m. All four complete Hermite
references pass the direct .20 m footprint/.05 m edge checker. Graph execution
safety is untested because no Graph reference was admitted for execution.

### Returned Hermite geometry

XY length here is the sampled returned polyline; the continuous Hermite arc
length used to choose M is listed under prepared constants. With M=2 there is
one chord-turn angle, so maximum and RMS are equal. All returned Hermite bridges
and complete references have self-intersection=false. Graph returned geometry
is N/A; the next table is explicitly an unexecuted solver diagnostic.

| Source | XY length m | Chord m | Length/chord | Max turn deg | RMS turn deg | Min edge m | Max edge m | Ref clearance m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | 0.106838302 | 0.106648569 | 1.001779041 | 6.833978540 | 6.833978540 | 0.051664408 | 0.055173893 | 0.229486890 |
| S2 | 0.127678200 | 0.127644317 | 1.000265452 | 2.640491797 | 2.640491797 | 0.062673649 | 0.065004551 | 0.757253287 |
| S3 | 0.184775132 | 0.184683190 | 1.000497837 | 3.616027841 | 3.616027841 | 0.090304340 | 0.094470792 | 0.211489855 |
| S4 | 0.161197131 | 0.160884226 | 1.001944908 | 7.145148804 | 7.145148804 | 0.077889553 | 0.083307578 | 0.878001187 |

### Factor costs and failed Graph solves

| Source | State | E_in | E_out | E_smooth | E_space | Total |
| --- | --- | --- | --- | --- | --- | --- |
| S1 | Hermite initial | 67.819091046 | 33.402277113 | 0.018678462 | 0.000546069 | 101.240592690 |
| S1 | Graph last iterate, UNCONVERGED | 0.000000288 | 35.998167995 | 0.504377389 | 0.504268168 | 37.006813839 |
| S2 | Hermite initial | 50.238256547 | 34.954364037 | 0.005858113 | 0.000239827 | 85.198718524 |
| S2 | Graph last iterate, UNCONVERGED | 0.000000290 | 35.984215141 | 0.719216245 | 0.719191062 | 37.422622737 |
| S3 | Hermite initial | 54.575621509 | 34.600094962 | 0.023409069 | 0.000708930 | 89.199834469 |
| S3 | Graph last iterate, UNCONVERGED | 0.000000069 | 36.000543328 | 1.393015230 | 1.392893101 | 38.786451728 |
| S4 | Hermite initial | 69.057008471 | 33.291099555 | 0.040753610 | 0.001268192 | 102.390129828 |
| S4 | Graph last iterate, UNCONVERGED | 0.000006706 | 36.000621847 | 1.118258300 | 1.118210143 | 38.237096996 |

All four solve attempts terminated at `maximum_iterations`, iterations=80,
converged=false. Unsafe improving-proposal rejections=0 for every source.
The final saved internal states retained exact B/E and downstream rows. Their
first bridge edge lengths were 2.2400526872570977e-6, 1.8538375079497577e-6,
2.3147443132026727e-6 and 2.544209239606101e-6 m (S1–S4). Thus the first edge became
micrometre-sized while the other edge remained near the B→E chord length.
These are diagnostic last accepted iterates, not returned or installed references.
They were not repaired, restarted or executed. Full geometry/factor diagnostics
with trace hashes are in `solver_failure_diagnostics.json`; that saved-only
calculation made zero additional optimizer/MPC calls.

### First official H5 identities

The first legal submits are at ticks 96, 1002, 1806 and 1524 for S1–S4.
Native/C3/Hermite first-submit poses are identical within each source. Bridge
rows have explicit derived identities; they are not relabeled as raw FRESH rows.

| Source | Method | First H5 selected identities |
| --- | --- | --- |
| S1 | Native | F_3, F_4, F_5, F_6, F_7 |
| S1 | C3 | F_2, F_3, F_4, F_5, F_6 |
| S1 | Hermite | bridge_1, E*, F_2, F_3, F_4 |
| S1 | Graph | N/A |
| S2 | Native | F_4, F_5, F_6, F_7, F_8 |
| S2 | C3 | F_4, F_5, F_6, F_7, F_8 |
| S2 | Hermite | bridge_1, E*, F_3, F_4, F_5 |
| S2 | Graph | N/A |
| S3 | Native | F_4, F_5, F_6, F_7, F_8 |
| S3 | C3 | F_3, F_4, F_5, F_6, F_7 |
| S3 | Hermite | bridge_1, E*, F_3, F_4, F_5 |
| S3 | Graph | N/A |
| S4 | Native | F_3, F_4, F_5, F_6, F_7 |
| S4 | C3 | F_3, F_4, F_5, F_6, F_7 |
| S4 | Hermite | bridge_1, E*, F_3, F_4, F_5 |
| S4 | Graph | N/A |

All raw downstream identities after E* remained bit-identical to the historical
Native world installation. Derived Hermite installation roundoff maxima were
3.552713678800501e-15, 3.552713678800501e-15, 1.7763568394002505e-14 and
5.329070518200751e-15 (S1–S4), below the frozen 1e-12 interface check. Planning
B/E themselves are exact copies. Original raw FRESH was never overwritten or
re-anchored at B.

### PNGs and numerical inspection

Exactly four final PNGs were generated and visually inspected, with matching
numeric sidecars and hashes in `figure_manifest.json`:

1. [World execution overview](../results/b_to_entry_bridge_01/figures/world_execution_overview.png).
   Only Native, C3 and Hermite executions exist. **The Graph legend has no curve:
   all Graph rollouts were withheld after nonconvergence.**
2. [Bridge geometry](../results/b_to_entry_bridge_01/figures/bridge_geometry.png).
   Graph returned geometry is explicitly N/A. Arrows show prescribed continuous
   endpoint tangents; coarse sampled chords do not exactly match those arrows.
3. [Transition and completion metrics](../results/b_to_entry_bridge_01/figures/transition_and_completion_metrics.png).
4. [Attachment versus completion](../results/b_to_entry_bridge_01/figures/attachment_vs_completion.png).
   Graph missing executions and S3 Native/C3 missing dwell remain separate N/A rows.

Native/C3 curves overlap exactly in S2 and S4; the authenticated S1 maximum XY
difference is 5.344081835845081e-10 m. Staggered markers and different line styles
expose those overlaps. No unobserved time is plotted at a fabricated numeric cap.

### Protocol outcome and deviations

The expected eight new rollouts became **four completed Hermite rollouts and four
withheld Graph rollouts** because all four fixed-budget graph attempts failed to
converge. This follows the failure policy declared before freeze; it prevents a
complete four-method causal comparison. No retry, solver-budget change, relaxed
termination, alternate initialization, result-driven method change, source change,
MPC modification or additional scientific solve occurred. A supplementary saved
last-iterate diagnostic JSON was added for transparent failure reporting; it is
not an additional reference, execution or final PNG.


## Research interpretation

Hermite recovers the unresolved S3 case under this frozen controller/schedule:
both sustained attachment and original-FRESH endpoint dwell become observable.
Its S3 .9 position AUC falls by 0.00430051282231636 m s (2.3052381070416517%) versus
C3; endpoint error at the cap falls from .11235439505980861 to .0604422053494252 m.
This is evidence that an explicit spatial transition can affect this source's
execution. It does not establish a graph benefit.

The easy controls show a cost. Hermite preserves all three previously observed
endpoint dwells but delays them by 9, 6 and 9 integration ticks in S1/S2/S4
(approximately .15/.10/.15 s). S2 .9 position AUC increases by .8787520089330147%.
S1 attaches at the same time with more remaining arc. S4 attaches .23333334550261497 s
earlier, at original arc fraction .6373064861628691 instead of .9127535301115488,
leaving .46543965739221604 m instead of .11196220914994415 m; its endpoint dwell is
.15000000782310963 s later. This is an observed faster-attachment/slower-endpoint
trade-off. Hermite therefore does not meet the requested non-regression condition
or support a SIMPLE_BRIDGE_SUFFICIENT conclusion for the joint four-source task.

Overall classification remains **TECHNICAL_BLOCKED**, taking precedence over the
observed Hermite trade-off. Graph recovery and Graph-versus-Hermite execution are
not identifiable because no Graph rollout exists. A lower last-iterate objective
is not evidence of improved execution.

The graph traces show near-collapse of the first edge despite the finite spacing
penalty. Its micrometre scale is close to the 1e-6 finite-difference perturbation,
which suggests poor numerical conditioning near a direction singularity. This is
an inference from saved iterates, not a proved explanation of nonconvergence or
permission to retune this experiment. The fixed M=2 construction may also restrict
how well both endpoint directions can be represented. No graph repair or new
factor was implemented.

### Answers to the research question

- **Can this bridge recover S3?** Hermite did: attachment 1.4666667431592941 s,
  endpoint dwell 1.7666667588055134 s, remaining arc at attachment .2926991990444493 m.
  Graph execution is unavailable.
- **Without degrading S1/S2/S4?** No for downstream dwell timing: all three are
  later with Hermite. Safety and observation of endpoint dwell are preserved.
- **Is simple Hermite sufficient?** It demonstrates S3 recovery, but does not meet
  the combined recovery/non-regression criterion.
- **Is graph optimization necessary or superior?** Not established. All four
  bounded graph solves failed to converge; no Graph execution comparison exists.
- **What follows?** Investigate the frozen graph's short-edge numerical behavior
  in a separate explicitly authorized study before making another execution claim.
  This experiment does not freeze a final reconciliation method. Any later frozen
  method needs held-out obstacle geometries for evaluation.


## Limitations

Four-source development experiment; offline controlled timing; intrinsic original
FRESH tracking difficulty; fixed official controller; no controller-aware factor;
no semantic-intent proof. No obstacle/population/real-world generalization, online
VLA improvement, navigation goal completion, graph superiority beyond these sources,
or final-method claim. If a method is frozen later, evaluate it on newly collected
held-out obstacle geometries in a separate experiment.

### Prepared spatial constants (before scientific execution)

| Source | M | d_F m | Continuous Hermite XY arc m |
|---|---:|---:|---:|
| S1 | 2 | 0.1501826264886162 | 0.12379645828894754 |
| S2 | 2 | 0.15051322580364157 | 0.14702648743330163 |
| S3 | 2 | 0.15648214800275878 | 0.21321503574581863 |
| S4 | 2 | 0.15214175181339587 | 0.18683023531723233 |

All four prepared Hermite polylines passed reference safety and official
zero-numerical-solve installation/restoration. M=2 means exactly one editable
interior pose in each graph. The controller receives the sampled polyline; the
continuous Hermite endpoint derivative does not imply its coarse first/last
chord exactly matches that tangent. Both errors are measured by E_in/E_out.

### Reproduction commands

Use the existing repository `.venv` without modifying it or external environments.
The focused test contains 29 tests. The full relevant command is:

```bash
git fetch origin main
git rev-parse HEAD
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_spatial_entry_suffix_execution01.py --run data/spatial_entry_suffix_execution_01/primary_20261002T000000Z --check-only
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z --mode prepare
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/bridge-mpl .venv/bin/python -m pytest -q tests/test_b_to_entry_bridge01.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/bridge-mpl .venv/bin/python -m pytest -q tests/test_b_to_entry_bridge01.py tests/test_spatial_entry_suffix_execution01.py tests/test_spatial_correspondence_selector_diag01.py tests/test_state_shift_transport_scale01.py tests/test_relative_factor_multisource01.py tests/test_osa03_relative_factor_replication01.py tests/test_osa03_relative_factor_ablation01.py tests/test_osa03_common_b.py tests/test_local_se2_reconciliation.py tests/test_local_se2_saved.py tests/test_se2.py tests/test_se2_graph.py tests/test_se2_lie.py tests/test_trajectory.py tests/test_transition_graph.py tests/test_exp02d_lookahead_direction.py tests/test_spatial_entry.py tests/test_osa03_native.py tests/test_osa03_native_validation.py tests/test_osa03_trackability.py tests/test_online_mpc_adapter.py tests/test_robotless_online.py tests/test_robotless_online_replay.py tests/test_robotless_online_validator.py tests/test_handoff_execution_loss.py tests/test_join_online02.py tests/test_obstacle_source03.py tests/test_join_source02_geometry.py tests/test_gp_se2_environment.py tests/test_gp_se2_diag02_environment.py tests/test_handoff_delay_attribution.py tests/test_genuine_source_scan.py
.venv/bin/python -m compileall -q src scripts tests
git diff --check
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z --mode freeze
# Review/stage only this namespace, protocol and append-only work log; commit and normal push.
git diff --cached --check
git commit -m "Freeze fixed-boundary B-to-entry bridge comparison"
git push origin main
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 .venv/bin/python scripts/run_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z --mode execute
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/bridge-mpl .venv/bin/python scripts/report_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/validate_b_to_entry_bridge01.py --run data/b_to_entry_bridge_01/primary_20261002T020000Z --check-only
```
