#!/usr/bin/env python3
"""Fail-closed saved-source, reconstruction, runtime and screenshot audit."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

import old_fresh_problem_gui_artifacts as a
from reconciliation.old_fresh_problem_gui import CASE_IDS, agree
from reconciliation.robotless_single_chunk import sha256_file, save_json_exclusive

REVIEW_FLAGS = ('all_case_buttons_visible', 'metrics_legible', 'RGB1_visible', 'before_FRESH_hidden',
                'after_FRESH_visible', 'B_Q_and_color_legend', 'true_direction_comparison',
                'window_arrows_labeled', 'overview_full_context', 'no_robot_or_correction')


def require(condition, message):
    if not condition: raise ValueError(message)


def validate_run(run):
    run = Path(run).resolve()
    result = {'run_directory': str(run), 'status': a.RUNTIME_NOT_VALIDATED, 'failures': [], 'case_ids': list(CASE_IDS)}
    try:
        record = a.read_json(run/'source.json')
        source = Path(record['source_pilot_run'])
        require(source == a.SOURCE, 'wrong primary source')
        validation, files = a.verify_source(source, record['files'])
        require(record['validation'] == validation, 'saved source validation differs')
        cases, records = a.load_cases(source)
        require(records == record['case_records'], 'saved source provenance differs')
        audits = a.read_json(run/'geometry_audit.json')
        require(set(audits) == set(CASE_IDS), 'wrong GUI geometry case set')
        for case_id, case in cases.items():
            require(audits[case_id] == a.case_audit(case), 'geometry audit differs from reconstructed arrays')
            require(np.array_equal(case.marker(.30), case.b), 'marker endpoint differs from B')
            require(np.array_equal(case.marker(0.), case.r0), 'marker initial pose differs from R0')
        processing = a.read_json(run/'processing_sources.json')
        require('scripts/validate_old_fresh_problem_gui.py' in processing, 'validator provenance missing')
        for path, digest in processing.items():
            require(sha256_file(a.ROOT/path) == digest, 'processing source changed: '+path)
        config = yaml.safe_load((run/'config_snapshot.yaml').read_text())
        require(config == yaml.safe_load((a.ROOT/'configs/robotless_old_fresh_problem_gui.yaml').read_text()), 'GUI configuration changed')
        require(config['display_playback_only'] and config['display_geometry']['xy_scale'] == 1 and
                config['display_geometry']['angular_magnification'] == 1, 'scaling/timing convention changed')
        required = ('runtime_started.json','runtime_controls.json','persistence.json','visual_review.json')
        missing = [p for p in required if not (run/p).is_file()]
        if missing:
            result.update(missing_runtime_artifacts=missing, source_file_count=len(files), source_validation=validation['status'])
            return result
        runtime, controls, persistence, review = [a.read_json(run/p) for p in required]
        require(runtime['persistent_default'] and not runtime['no_hold'], 'persistent hold mode was not run')
        require(runtime['scene_load_count'] == 1 and not runtime['scene_visibility_modifications'], 'scene changed/hidden')
        require(runtime['scene']['runtime_inventory']['no_robot_model'], 'robot present')
        require(not runtime['dynamics_advanced'] and runtime['new_lightnav_inference_count'] == 0 and
                runtime['execution_time'] is None and runtime['display_playback_only'], 'runtime scope changed')
        require(persistence['app_is_running'] and persistence['elapsed_after_ready_s'] >= 12 and
                persistence['render_updates_after_ready'] > 10 and not persistence['no_hold'] and
                persistence['source_files_unchanged'], 'persistence not observed')
        require(persistence['time']['simulation_time_s'] == 0, 'simulation timeline advanced')
        require([c['episode_id'] for c in controls['checks']] == list(CASE_IDS), 'all case controls were not exercised')
        for check in controls['checks']:
            require(all(check[k] for k in ('case_reset','replay_old_endpoint_exact','pause','reveal','reset','window_toggle','rgb_toggle')),
                    'a runtime control did not pass')
            require(check['full_replay_phases'] == ['HANDOFF_PAUSE','FRESH_REVEALED'] and
                    check['camera_modes'] == ['handoff','overview'], 'runtime replay/camera incomplete')
        for sample in controls['samples']:
            state = sample['state']; case = cases[state['case_id']]
            require(np.array_equal(case.marker(state['progress_m']), sample['marker_world']), 'runtime marker deviated')
            require(sample['time']['display_playback_only'] and sample['time']['execution_time'] is None, 'physical replay time claimed')
        events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
        event_times = [e['time']['host_monotonic_ns'] for e in events]
        require(event_times == sorted(event_times), 'display events reverse monotonic time')
        require(review['reviewer'] and review['method'] and review['all_screenshots_inspected'], 'visual inspection absent')
        screenshots = []
        for case_id in CASE_IDS:
            require(all(review['cases'][case_id].get(flag) is True for flag in REVIEW_FLAGS), 'visual review incomplete: '+case_id)
            for suffix in ('before_fresh','after_fresh','window_tangents','overview'):
                image = run/f'evidence/{case_id}_{suffix}.png'
                meta = a.read_json(image.with_suffix('.json'))
                digest = sha256_file(image)
                require(meta['image_sha256'] == digest == review['image_hashes'][image.name], 'screenshot bytes changed')
                with Image.open(image) as im:
                    require(im.format == 'PNG' and min(im.size) >= 700 and np.asarray(im).std() > 1, 'invalid/blank GUI screenshot')
                source_hashes = records[case_id]['source_hashes']
                for key,path in [('source_OLD_hash','raw/chunk_000.npy'),('source_FRESH_hash','raw/chunk_001.npy'),('metric_JSON_hash','derived/metrics.json')]:
                    require(meta[key] == source_hashes[path], 'screenshot source hash mismatch')
                require(meta['episode_id'] == case_id and Path(meta['source_pilot_run']) == source, 'screenshot source/case mismatch')
                require(meta['reveal_state'] == (suffix != 'before_fresh'), 'screenshot reveal state mismatch')
                require(meta['camera_mode'] == ('overview' if suffix == 'overview' else 'handoff'), 'camera mode mismatch')
                require(meta['state']['show_window'] == (suffix == 'window_tangents'), 'window diagnostic state mismatch')
                require(meta['state']['show_rgb'] and meta['state']['progress_m'] == .30, 'screenshot lacks RGB or exact boundary')
                require(meta['geometry'] == audits[case_id], 'screenshot geometry differs')
                require(meta['geometry_scaling'] == meta['angular_magnification'] == 1., 'geometry exaggerated')
                require(len(meta['research_git_sha']) == 40 and meta['capture_time']['host_unix_ns'] > 0 and
                        meta['capture_time']['display_playback_only'], 'screenshot provenance/timing incomplete')
                screenshots.append(str(image.relative_to(run)))
        a.verify_hashes(source, record['files'])
        result.update(status=a.VALIDATED, source_validation=validation['status'], source_file_count=len(files),
            screenshots=screenshots, screenshot_count=len(screenshots), source_and_geometry_unchanged=True,
            persistent_runtime_observed=True, runtime_sample_count=len(controls['samples']),
            metrics={k: {m:c.saved_metrics[m] for m in ('e_perp_m','abs_e_dir_local_deg','abs_e_dir_window_deg','abs_e_yaw_deg')}
                     for k,c in cases.items()})
    except (OSError, ValueError, KeyError, TypeError, AssertionError) as error:
        result.update(status=a.FAILED, failures=[str(error)])
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--save', action='store_true', help='write validation.json once, never overwrite')
    args = parser.parse_args()
    result = validate_run(args.run_directory)
    if args.save: save_json_exclusive(args.run_directory/'validation.json', result)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['status'] == a.VALIDATED else 1)
