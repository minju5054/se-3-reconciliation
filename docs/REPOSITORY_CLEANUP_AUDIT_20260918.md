# Repository cleanup — 2026-09-18

The user requested removal of older files and folders unrelated to the current
robotless research. After being told that the previous Jackal, EXP and DATA-02
final raw records were not in Git and could not be recovered by reverting a
commit, the user explicitly authorized:

> 현재 robotless 연구와 무관한 이전 최종 원본도 삭제

The approved deletion completed at **2026-09-18 01:50:06 UTC / 10:50:06 KST**.
This audit supersedes the local-retention decisions in the
[2026-09-09 audit](REPOSITORY_CLEANUP_AUDIT_20260909.md). Earlier work-log entries
and research reports remain historical records; their removed raw-data paths
are no longer available locally.

## Removed artifacts

The explicit allowlist contained 37 targets: 26 legacy data roots, two PDF roots,
two synthetic result files and seven Python/pytest cache roots. Removal covered
**91,181 regular files, 3,392 symlinks and 18,893 directories**, followed by removal
of the two empty `output/` and `tmp/` parents.

| Group | Targets | Regular files | File contents, bytes | Allocated bytes |
|---|---:|---:|---:|---:|
| `data/exp01*`, `data/exp02*` | 18 | 30,247 | 4,539,198,869 | 4,676,108,288 |
| `data/data02_*` | 6 | 43,939 | 2,564,242,855 | 2,710,372,352 |
| `data/stage0`, `data/controller_effect_check` | 2 | 16,524 | 1,202,357,273 | 1,251,241,984 |
| `output/pdf`, `tmp/pdfs` | 2 | 69 | 48,789,432 | 48,947,200 |
| `results/exp01/synthetic_demo.{json,png}` | 2 | 2 | 68,910 | 73,728 |
| Python/pytest caches | 7 | 400 | 7,564,113 | 8,466,432 |
| **Total** | **37** | **91,181** | **8,362,221,452** | **8,695,209,984** |

Allocated size is the pre-removal sum of `lstat().st_blocks * 512` over unique
inodes, including target directories and symlinks, excluding the two empty
parents. This is **8.70 GB / 8.10 GiB** of removed allocation. The filesystem's
free-space counter increased by 8,695,074,816 bytes during the operation; this
counter can also reflect concurrent system activity. Audit files occupy new
space and are not included in the removed-allocation total.

The deleted legacy data roots were:

```text
controller_effect_check
data02_collection_demo
data02_collection_demo_selection
data02_combined_v1_v2
data02_high_motion_demo
data02_online_successive_v1
data02_online_successive_v2
exp01a
exp01b
exp01b_extension
exp01b_redesign
exp02
exp02a
exp02b
exp02b_calibrated_reeval
exp02b_failure_demo
exp02b_gui_diagnosis
exp02b_presentation
exp02c_factor_isolation
exp02c_presentation
exp02d_lookahead_direction
exp02d_objective_execution
exp02d_physical_execution
exp02d_presentation
exp02d_turning_search
stage0
```

This intentionally removed final original recordings as well as derivatives.
For example, the EXP-02B/02D presentation trees contained 3,971,197,178 bytes of
actual GUI/physics `raw_frames` captures; those were not regenerable caches.
The old PDF/ZIP/video package was a final historical deliverable. The user's
explicit authorization covered both. Retained source code, MP4 hashes and the
deletion manifest cannot reconstruct the removed raw recordings.

The removed cache roots were `.pytest_cache`, `scripts/__pycache__`,
`scripts/isaac/__pycache__`, `scripts/lightnav/__pycache__`,
`src/reconciliation/__pycache__`, `src/reconciliation/controllers/__pycache__`
and `tests/__pycache__`. The tracked `results/exp01/README.md` remains.

## Preserved research and dependencies

All 12 existing `data/robotless_*` / `data/reference_reproduction` roots were
preserved: **36,684 regular files, 2,504,027,893 content bytes**, with no symlinks.
Every file's SHA-256 and recorded filesystem metadata matched before and after
removal. No retained raw output, coordinate convention, timestamp, trajectory,
plot or analysis result was rewritten.

The preservation boundary includes:

- The current 60-episode, **881-valid-handoff** primary under
  `data/robotless_online_handoffs_v1/primary_20260915T091900Z/`, its live RGB,
  raw chunks, executed trajectories, event plots and HTML index.
- Every online technical run, especially `smoke_20260915_01` server provenance
  and `smoke_20260915_10` GUI/validation evidence referenced by the primary.
- The trajectory-shape analysis and recorded online GUI replay, including
  their source hashes and processing configuration.
- All robotless predecessor evidence: single/successive chunks, controlled
  staleness, projection geometry, screening, OLD-conditioned continuation,
  OLD-consistent observations and the OLD/FRESH problem GUI.
- The separate official LightNav MuJoCo reference reproduction.

The current tools import earlier robotless validators and shared SE(2), timing,
I/O and controller modules. A conservative code-dependency audit identified
46 Python files and 41 directly related test files. All tracked code, tests,
configurations and historical reports were retained, including
`configs/robotless_handoff_screening.yaml`, which the frozen schedule hashes.
No legacy generated root in the deletion allowlist is required to validate,
analyse or replay the current online primary.

The two pre-existing user configuration edits were preserved byte-for-byte:

| File | SHA-256 |
|---|---|
| `configs/stage0_jackal_controller_validation.yaml` | `48b4d635b01276fcef29d08e73d8ed97888bf31bf4c91099aba68f692cd92a80` |
| `configs/stage0_lightnav_single_chunk.yaml` | `0b0aed65e5c3c0deb0bf44b0e7fd997371319ef5c1be9edad3ac4e9da2271762` |

External LightNav source/checkpoints, Isaac installations, virtual environments
and repository metadata were outside the deletion scope. No new simulation or
model inference was performed.

## Local audit records and method

Detailed records are ignored local artifacts in
`data/cleanup_audits/20260918_robotless_scope/`:

- `plan.json`: authorization, exact targets, per-target sizes, preservation
  boundary, starting commit and protected user-file hashes.
- `deletion_manifest.json`: every removed entry, original regular-file hashes,
  sizes, metadata and symlink targets.
- `preservation_manifest.json`: every preserved data entry and original hashes.
- `deletion_result.json`: completion, all targets absent, full preservation
  comparison and disk-space counters.
- `cleanup_robotless_20260918.py` and `code_keep_closure.json`: the one-off
  cleanup implementation and conservative code-dependency audit.
- Validation results, independent provenance/link checks and test logs.

Manifest SHA-256 values:

```text
deletion_manifest.json
fb587c3f3d37cfeb6ef37782d766d70ac1c4d5b4aab4dd6efd14d91f607aa07b
preservation_manifest.json
0d474536342bd9033ffff64a8ca8909b72cc347ceb715333fe5780b339a886ec
```

The one-off script first rejected unknown data roots, overlapping targets,
symlink roots and any tracked deletion target. It hashed all regular files,
then rechecked the complete target and preservation inventories before applying
the allowlist. Directory removal did not follow nested symlinks. It verified
that all targets were absent and the entire preserved inventory was unchanged
afterward. The manifests identify deleted records; they are not backups.

Starting Git commit: `5ea2bbf6008fe8133e06b5c2a7bafd8b0be175ac` on `main`.
This audit, README, the append-only work log and three test files handling
optional historical corpora belong to the cleanup commit; generated data and
the two user configuration changes are excluded.

## Validation after deletion

The full current-primary validator, including PNG checks, exited 0 with
`valid=true`, `causal_valid=true`, `schedule_complete=true`, `errors=[]`,
60 validated episodes, 881 valid handoffs and 1,762 valid-event PNGs. Its result
is JSON-equal to the original preserved validation. The original status remains
`ROBOTLESS_ONLINE_HANDOFF_DATASET_COLLECTED_WITH_LIMITATIONS`: cleanup does not
resolve the existing pacing or history-length limitations.

An independent post-deletion check at 01:51:08 UTC verified all 2,884 shape
analysis input hashes, the processing-script hash, all 97 replay input hashes
and all 3,041 local HTML index links. No link was missing or outside the primary
run. These checks read existing evidence and do not create experimental data.

Commands, run from the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python scripts/validate_robotless_online_handoffs.py \
  data/robotless_online_handoffs_v1/primary_20260915T091900Z \
  > data/cleanup_audits/20260918_robotless_scope/online_validation_after.json
.venv/bin/python -B /tmp/robotless_cleanup_independent_20260918.py \
  > /tmp/robotless_cleanup_independent_20260918.log 2>&1
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv/bin/python -m pytest -q -p no:cacheprovider
```

The first full test run found 11 failures caused by references to the removed
legacy raw corpora, alongside 1,281 passing and seven already-optional skipped
tests. Three historical test files were adjusted to skip their corpus-dependent
checks when those untracked corpora are absent. Configuration/hash checks remain
independent of raw-data availability, and mathematical/coordinate/timing tests
continue to run. Missing or corrupt files within a present corpus still fail;
no synthetic replacement was presented as historical evidence.

The focused tests passed **50, with 11 skipped**. The final full suite passed
**1,282, with 18 skipped, in 34.84 seconds**, with no failures. All skips require
the retired historical corpora. The G2 split adds one separately counted test;
it preserves the always-running configuration/hash coverage. Logs are
`pytest_after_cleanup.log` (initial failures),
`pytest_legacy_optional_evidence.log` (focused) and `pytest_final.log` (final).
The suite ran on the host for its Unix-socket IPC tests, with bytecode and pytest
cache writing disabled. No production source was changed.
