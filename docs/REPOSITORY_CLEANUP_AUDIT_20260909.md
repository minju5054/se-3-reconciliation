# Repository Cleanup Audit — 2026-09-09

## Scope and decision rule

This audit is a forward-only repository cleanup. It does not introduce a dataset,
experiment, formulation, controller change, LightNav inference, or simulator run. The
working tree and fetched `origin/main` were inspected before deletion. Two pre-existing user
edits in `configs/stage0_jackal_controller_validation.yaml` and
`configs/stage0_lightnav_single_chunk.yaml` were left untouched.

The labels mean:

- `KEEP_ACTIVE`: current reusable repository infrastructure or current-state documentation.
- `KEEP_EVIDENCE`: an artifact or its exact reproducer/validator supports a recorded claim.
- `ARCHIVE_CANDIDATE`: not an active dependency, but potentially useful historical evidence;
  no deletion was made without a later explicit decision.
- `DELETE_CANDIDATE`: no unique research claim or active dependency was found; retained only
  because it was not in the explicitly safe deletion class.

## Storage measurement

Measurements use `du -sh` and therefore report allocated filesystem size.

| Scope | Before | Immediately after cleanup |
|---|---:|---:|
| Repository, including `.venv` and `.git` | 817M | 429M |
| `data/` | 626M | 241M |
| `.venv/` | 182M | 182M (untouched) |

The explicitly enumerated project targets accounted for 382,630,936 bytes. Seventeen
DATA-02 runtime directories below `/tmp` accounted for another 195,388 bytes. The difference
between byte totals and `du -sh` deltas is filesystem allocation and display rounding.

## Removed now

| Removed path or family | Pre-delete size | Reason and dependency result |
|---|---:|---|
| `data/data02_lightnav_diverse_transitions/` | 343,287,660 B | All local DATA-02 qualification/smoke/probe outputs; DATA-02 was abandoned and no retained experiment consumes it. |
| DATA-02 config, document, collector/launcher, summarizer, validator, library, and test | 8 tracked files total | Dedicated abandoned pipeline. The six-line DATA-02 allowance in `scripts/lightnav/serve_online_lightnav.py` was also reverted. Only append-only history remains in `docs/WORK_LOG.md`. |
| `docs/DATA_01_FROZEN_LIGHTNAV_TRANSITION_BANK.md` | tracked document | Final DATA-01-only tracked residue. Its config/build/plot/validate/library/test pipeline was already removed by the preceding forward retirement commit; the generated bank was already absent. |
| repository `__pycache__`, `.pytest_cache`, and `src/se3_reconciliation.egg-info` | 2,766,918 B | Regenerable Python/test/build caches; `.venv` was excluded. |
| `/tmp/data02-*` | 195,388 B | Seventeen abandoned DATA-02 IPC/probe runtime directories. |
| EXP-01B extension/redesign smoke runs | 859,835 B | Explicit smoke-only output, not source evidence. |
| `data/exp02a/exp02a-20260905T-test/` | 1,983,862 B | Explicit test output; the documented frozen run is retained. |
| EXP-02B-R prep-smoke and `pre_fix`/`pre_annotation` subtrees | 6,773,436 B | Superseded generated views/summaries inside or next to the preserved primary run. |
| Nine non-final EXP-02B GUI diagnosis runs | about 0.8M | Smoke/visual/intermediate GUI outputs. The documented `...T-final` run is retained. |
| `data/exp02b/exp02b-controller-aware-20260906T150300Z/` | 572,435 B | `docs/WORK_LOG.md` explicitly records it as incomplete after exposing missing `ISAAC_SIM_ROOT`; the corrected T150400Z run supersedes it. |
| Stage 0 smoke, failed-plot, and explicitly `superseded` outputs | about 25M | Regenerable smoke products or outputs whose directory names and work log identify a corrected replacement. Primary Stage 0-D/E/F runs remain intact. |

No raw LightNav source file, frozen EXP-01B source, primary EXP-02B/02B-R/02C result, Stage
0-D/E/F primary result, system Python, ROS 2, CUDA, Isaac Sim, or LightNav environment was
modified.

## `KEEP_ACTIVE`

| Path/family | Size after cleanup | Why active | Active reference |
|---|---:|---|---|
| `README.md`, `docs/WORK_LOG.md`, this audit | about 132K | Current state, append-only provenance, and cleanup decisions. | Repository entry point and research rules. |
| `src/reconciliation/` excluding generated caches | about 528K | Reusable SE(2), trajectory, controller, metrics, IPC, and experiment validation code. | Imported by current scripts and the full test suite. |
| `tests/` and `tests/fixtures/` | about 240K | Regression coverage for retained behavior and timing/SE(2) contracts. | Full pytest suite. |
| `scripts/` and `configs/` not otherwise retired | about 1.0M | Launch, validation, summarization, and visualization of retained experiments/evidence. | README and experiment documents. |
| `.venv/` | 182M | Repository test environment. It is ignored and was not altered. | Documented test/install commands. |

There is no active formulation-development dataset collector and EXP-02D has not started.

## `KEEP_EVIDENCE`

| Artifact | Size | Why it must stay | Exact reference/dependency |
|---|---:|---|---|
| `data/stage0/lightnav_single_chunk/20260903T010425Z/` | 3.9M | Validated Stage 0-C input and LightNav warm-up source. | `configs/exp01a_lightnav_latency.yaml`, `configs/exp01b_online_raw_switch.yaml`, `configs/exp01b_controlled_latency.yaml`, and Stage 0-C/EXP-01A docs. |
| `data/exp01a/lightnav_latency/exp01a-20260903T062524Z/` | 108K | Final latency benchmark. | `docs/EXP_01A_LIGHTNAV_LATENCY.md`, `docs/WORK_LOG.md`. |
| `data/exp01b/exp01b-20260903T155402Z/` | 728K | Validated original EXP-01B source and later EXP-02/02A dependency. | EXP-01B/02/02A configs and docs. |
| `data/exp01b_extension/exp01b-extension-20260905T002500Z/` | 7.8M | Completed extension evidence and controlled-latency input. | `docs/EXP_01B_EXTENSION.md`, `scripts/isaac/exp01b_online_raw_switch.py`, work log. |
| Three configured EXP-01B qualification cohorts: `...-fixed`, `...-candidate2`, `...-g2candidate5` | 972K, 948K, 248K | Frozen scenario-selection provenance. | Exact artifacts in `configs/exp01b_controlled_latency.yaml`. |
| `data/exp01b_redesign/exp01b-controlled-primary-20260906T-frozen/` | 8.9M | Primary frozen EXP-01B source evidence. | EXP-02B and Stage 0-F configs/docs; retained independently of retired DATA-01. |
| `data/exp02/exp02-20260904T053813Z/` | 584K | Documented oracle-graph pilot. | `docs/EXP_02_ORACLE_GRAPH.md`, work log. |
| `data/exp02a/exp02a-20260904T162149Z/` | 3.8M | Documented final EXP-02A pilot. | `docs/EXP_02A_SPATIAL_ENTRY_RECONCILIATION.md`, work log. |
| `data/exp02b/exp02b-controller-aware-20260906T150400Z/` | 3.5M | Frozen primary EXP-02B result and downstream source. | EXP-02B/02B-R/02C/Stage 0-D/E configs, docs, and tests. |
| `data/exp02b_gui_diagnosis/...T-final/` | 104K | Final feedback-1 GUI/telemetry diagnosis. | `docs/EXP_02B_GUI_DIAGNOSIS.md`, work log. |
| `data/stage0/controller_validation/stage0b-official-primitives-20260903/` and `stage0b-closed-loop-accepted-20260903/` | 224K, 108K | Final Stage 0-B primitive and closed-loop validation sessions. | `docs/STAGE_00_CONTROLLER_VALIDATION.md`, work log. |
| `data/stage0/execution_calibration/stage0d-20260907T102700Z/` | 55M | Frozen execution-layer calibration evidence/model source. | Stage 0-D/E and EXP-02B-R configs/docs. |
| `data/stage0/closed_loop_execution_validation/stage0e-20260908T021433Z/` | 24M | Frozen closed-loop validation evidence. | Stage 0-E/F configs/docs. Failed plot subtree alone was removed. |
| `data/stage0/lightnav_execution_envelope/stage0f-20260908T043300Z/` | 25M | Frozen observed LightNav execution-envelope evidence. | Stage 0-F and EXP-02B-R configs/docs. Superseded plot subtree alone was removed. |
| `data/exp02b_calibrated_reeval/exp02b-r-20260908T054233Z/` | 14M | Frozen EXP-02B-R primary result. | `configs/exp02b_calibrated_reeval.yaml`, EXP-02B-R docs/work log. |
| `data/exp02c_factor_isolation/exp02c-factor-isolation-20260908T133000Z/` | 13M | Frozen EXP-02C attribution result. | `configs/exp02c_factor_isolation.yaml`, EXP-02C docs/work log. |

The matching configs, source modules, scripts, tests, and experiment documents are also
`KEEP_EVIDENCE`: removing them would leave the listed results without their declared
reproducer, validator, schema, or claim boundary.

## Requested legacy code/config audit

All entries below are Git-tracked unless the row says otherwise. README now links to the
relevant experiment document rather than duplicating historical commands.

| Candidate | Size | README/docs reference | Source/test use | Primary-result role | Decision |
|---|---:|---|---|---|---|
| `configs/exp01.yaml` | 4K | Historical EXP-01 document/work log; EXP-01A explicitly distinguishes its timing from this example. | No direct runtime import or focused config test. | Synthetic/offline scaffold only. | `ARCHIVE_CANDIDATE`; keep pending a second cleanup decision. |
| `configs/exp01a_lightnav_latency.yaml` | 4K | EXP-01A doc/work log. | Default of `scripts/lightnav/benchmark_repeated_inference.py`; associated tests cover the analysis contract. | Configures the final T062524Z benchmark and Stage 0-C source. | `KEEP_EVIDENCE`. |
| `configs/exp01b_extension.yaml` | 4K | EXP-01B extension doc/work log. | Default of `run_exp01b_extension.sh`; extension source/tests remain. | Configures the completed T002500Z evidence. | `KEEP_EVIDENCE`. |
| `configs/exp01b_online_raw_switch.yaml` | 4K | EXP-01B doc/work log. | Used by the Isaac runner, launcher, and shared LightNav server; source/tests remain. | Reproduces the validated T155402Z run. | `KEEP_EVIDENCE`. |
| `configs/exp02_oracle_graph.yaml` | 4K | EXP-02 oracle doc/work log. | Default of `run_exp02_oracle_graph.py`; oracle/graph modules and tests remain. | Reproduces the documented oracle-graph pilot. | `KEEP_EVIDENCE`. |
| `configs/oracles/exp02_lightnav_development_pair.yaml` | 8K directory allocation | EXP-02 work log. | Referenced exactly by `configs/exp02_oracle_graph.yaml`. | Frozen oracle input for the pilot. | `KEEP_EVIDENCE`. |
| Six `configs/stage0_*.yaml` files | 32K total | Stage 0, B, C, D, E, and F documents. | Exact defaults/inputs for retained launchers and validators; covered by associated tests. | Reproducer/provenance for retained platform evidence. | `KEEP_EVIDENCE`; the two user-modified configs were not staged. |
| Historical EXP-01 scripts and summarizers | 132K total | EXP-01/01A/01B documents and work log. | Import retained modules and are covered by the associated tests. | Reproduce/validate recorded EXP-01-family claims. | `KEEP_EVIDENCE`. |
| Historical EXP-02 scripts/modules/tests, including oracle/synthetic code | 456K total | EXP-02/02A/02B/02B-R/02C documents and work log. | Cross-imported by runners/validators and exercised by tests. | Reproduce frozen graph and attribution evidence; synthetic products remain demonstrations only. | `KEEP_EVIDENCE`. |
| All remaining `*gui*.py` and GUI launchers | 172K total | Each is linked from its Stage 0, EXP-02B diagnosis/B-R, or EXP-02C document. | Imports retained runtime/visualization helpers. | Qualitative inspection of preserved evidence; no unreferenced obsolete GUI script was found. | `KEEP_EVIDENCE`. |
| All remaining `scripts/summarize_*.py` | 156K total | Referenced by the matching experiment document or launcher. | Imports retained validators and is indirectly/focally tested. | Reconstructs summaries and enforces frozen artifact schemas. | `KEEP_EVIDENCE`; no obsolete summarizer was found. |
| `results/` | 84K; only its 4K README is tracked | EXP-01 directory contract. | No active source dependency on generated files. | The JSON/PNG are synthetic demo output, not evidence. | Directory README `KEEP_EVIDENCE`; generated demo files `DELETE_CANDIDATE`. |

## `ARCHIVE_CANDIDATE` — retained pending confirmation

| Artifact | Size | Why not active | Reference audit |
|---|---:|---|---|
| `configs/exp01.yaml` | 4K | Example timing/threshold config for the initial offline scaffold. | Historical docs/work log only; no direct runtime import and not primary evidence. |
| EXP-01A runs `...T062315Z`, `...T062418Z` | 108K each | Earlier latency runs before the documented final T062524Z. | No current config/doc exact-path dependency found. |
| EXP-01B runs other than T155402Z (six directories) | 4.1M total | Earlier/later raw-switch attempts; T155402Z is the validated source. | No current exact-path dependency found; preserve until raw-attempt archival is approved. |
| EXP-01B qualifications `...-g2candidate3`, `...-g2candidate4` | 252K each | Not among the three artifacts frozen in the config. | No current exact-path dependency found; may retain scenario-selection history. |
| EXP-02A runs T081500Z and T083000Z | 3.8M each | Later uncited reruns; T162149Z is the documented frozen result. | No current exact-path dependency found. |
| EXP-02B T150000Z, T150100Z, and T150200Z | 1.2M, 3.5M, 3.5M | Pre-final protocol/result runs; T150400Z is the cited frozen run. | No current exact-path dependency found; unlike deleted T150300Z, not explicitly recorded as failed. |
| EXP-02C T120000Z, T123000Z, and T130000Z | 9.8M, 12M, 12M | Earlier factor-isolation outputs; T133000Z is final. | No current exact-path dependency found; retained because they may document diagnostic evolution. |
| Seven Stage 0-C single-chunk runs other than T010425Z | about 27M | Not the configured warm-up/source run. | No current exact-path dependency found; raw inference cost makes archival preferable to automatic deletion. |
| Stage 0-B dated/open-loop/initial sessions other than the two final named sessions | about 0.66M | Superseded by official-primitives and closed-loop-accepted. | No current exact-path dependency found; retain diagnostic history pending confirmation. |

## `DELETE_CANDIDATE` — retained pending confirmation

| Artifact | Size | Reason | References found |
|---|---:|---|---|
| `data/exp02b/qualification-gui-diagnostic-20260907/` | 24K | GUI-only qualification fragment; not a primary result. | No exact-path dependency found. |
| Three remaining `data/stage0/jackal_trajectory/` run directories | 44K each | Small demonstration outputs; the pipeline and validation contract are retained. | No exact run-id reference found. |
| `results/exp01/synthetic_demo.json` and `.png` | about 69K | Regenerable synthetic demonstration output, not research evidence. | Only `results/exp01/README.md` describes the directory contract. |

No artifact in the last two sections was deleted in this cleanup. Their classification is the
required dependency audit for a later, explicit archive/delete decision.
