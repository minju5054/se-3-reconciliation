#!/usr/bin/env python3
"""Bounded SOURCE03 terminal predictions; reuse SOURCE02's exact wire/session code."""
import argparse
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'scripts/lightnav')]
from reconciliation.join_source03 import read, save, sha
from join_source02_paired import validate_manifest, run_branches
from online_lightnav_worker import audited_startup
from robotless_single_frame_inference import git_state, resolve_path, verify_checkpoint_unchanged


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--condition', required=True)
    a = p.parse_args()
    run = a.run.resolve()
    # Parent runner checks conditional eligibility and pushed hashes; child
    # independently refuses undeclared inputs and overwrites.
    manifest = validate_manifest(read(run/'input_manifests'/f'{a.condition}.json'), read(run/'protocol.json'))
    if not manifest['eligible']:
        raise ValueError('frozen technical input gate failed')
    out = run/'paired_diagnostics'/a.condition
    out.mkdir(parents=True, exist_ok=False)
    save(out/'frozen_input_manifest.json', manifest)
    start = time.monotonic()
    config, contract, sampler, ready = audited_startup(run/'config_snapshot.yaml')
    save(out/'official_startup.json', ready)
    results = run_branches(out, manifest, config, contract, sampler)
    verify_checkpoint_unchanged(ready['checkpoint'])
    unchanged = git_state(resolve_path(config['paths']['lightnav_checkout'])) == ready['source']
    save(out/'summary.json', dict(condition_id=a.condition, branches=results,
        checkpoint_stat_unchanged=True, source_unchanged=unchanged,
        input_sha256=sha(run/'input_manifests'/f'{a.condition}.json'),
        wall_s=time.monotonic()-start,
        actual_terminal_predictions=sum(x['wire_parity'].get('actual_terminal_predictions_sent', 0) for x in results.values()),
        actual_buffer_only=sum(x['wire_parity'].get('actual_next_requests_sent', 0)-x['wire_parity'].get('actual_terminal_predictions_sent', 0) for x in results.values()),
        new_MPC_solves=0, new_GP_solves=0, retry_count=0))


if __name__ == '__main__':
    main()
