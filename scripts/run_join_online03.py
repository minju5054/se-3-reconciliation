#!/usr/bin/env python3
"""Destination-first instruction rerun, reusing the unchanged ONLINE02 pipeline."""
import argparse
from copy import deepcopy
import html
import json
from pathlib import Path
import subprocess
import sys
import zipfile

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
from run_join_online02 import read, save, sha, git, verify, start, environments
from reconciliation.join_online02 import ORDER, guard_check, ABORT
from reconciliation.join_source03 import mpc_audit
from reconciliation.online_history import history_contract
from analyze_join_online02 import analyze, call_counts, csvread, jsonlines
from validate_robotless_online_handoffs import validate_episode, equal_record

INSTRUCTION = ('Go to the far end of the hallway. Pass around the supply cart without touching it, '
               'and stop only when you reach the end of the hallway.')
CONFIG = ROOT / 'configs/join_online_03_destination_instruction.yaml'


def variant_config(original, declaration):
    if declaration['instruction'] != INSTRUCTION or declaration['order'] != ORDER:
        raise ValueError('undeclared instruction or execution order')
    result = deepcopy(original)
    result['join_online02']['instruction'] = INSTRUCTION
    result['join_online02']['experiment'] = declaration['experiment']
    return result


def initialize(run):
    declaration = yaml.safe_load(CONFIG.read_text())
    baseline = (ROOT / declaration['baseline_run']).resolve()
    if not read(baseline / 'validation.json')['valid']:
        raise ValueError('baseline validation failed')
    original = yaml.safe_load((baseline / 'config_snapshot.yaml').read_text())
    config = variant_config(original, declaration)
    run.mkdir(parents=True, exist_ok=False)
    (run / 'logs').mkdir()
    with (run / 'config_snapshot.yaml').open('x') as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    names = ['scenario.json', 'start_selection.json', 'side_passages.json']
    for name in names:
        with (run / name).open('xb') as stream:
            stream.write((baseline / name).read_bytes())
    assert read(run / 'start_selection.json')['selected']['distance_m'] == 5.
    source = read(baseline / 'source.json')
    preserved = dict(source['preserved'])
    for name in [*names, 'source.json', 'protocol.json', 'validation.json',
                 'config_snapshot.yaml', 'aggregate/analysis.json', 'aggregate/call_counts.json']:
        preserved[str(baseline / name)] = sha(baseline / name)
    for path, expected in preserved.items():
        assert sha(path) == expected, path
    save(run / 'source.json', dict(starting_sha=git('rev-parse', 'HEAD'), origin_main=git('rev-parse', 'origin/main'),
         baseline_run=str(baseline), source04_run=source['source04_run'], preserved=preserved, new_source_experiment=True))
    protocol = read(baseline / 'protocol.json')
    protocol.update(declaration=config['join_online02'], experiment=declaration,
                    historical_protocol=str(baseline / 'protocol.json'), fixed_source_start=True,
                    comparison='instruction intervention with fresh live online observations; not identical RGB/history')
    save(run / 'protocol.json', protocol)
    schedule = read(baseline / 'episode_schedule.json')
    for episode in schedule['episodes']:
        episode['instruction'] = INSTRUCTION
    save(run / 'episode_schedule.json', schedule)
    audit = mpc_audit(ROOT)
    assert audit['status'] == 'MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL'
    save(run / 'mpc_audit.json', audit)
    save(run / 'reporting_revisions.json', dict(files={}))
    save(run / 'config_comparison.json', dict(allowed_changed_fields=['join_online02.instruction', 'join_online02.experiment'],
         baseline_config_sha256=sha(baseline / 'config_snapshot.yaml'), new_config_sha256=sha(run / 'config_snapshot.yaml'),
         unchanged_except_declared=variant_config(original, declaration) == config,
         baseline_instruction=original['join_online02']['instruction'], new_instruction=INSTRUCTION))
    print(run)


def freeze(run):
    assert read(run / 'technical_preflight/result.json')['valid']
    files = [*ROOT.glob('scripts/*join_online02*.py'), *ROOT.glob('scripts/isaac/*join_online02*.py'),
             Path(__file__).resolve(), CONFIG, ROOT / 'tests/test_join_online03.py',
             ROOT / 'src/reconciliation/join_online02.py', ROOT / 'tests/test_join_online02.py',
             ROOT / 'configs/join_online_02_far_approach.yaml']
    for name in ['scripts/isaac/robotless_online_handoffs.py', 'scripts/online_lightnav_worker.py',
                 'scripts/online_mpc_worker.py', 'src/reconciliation/online_mpc_adapter.py',
                 'src/reconciliation/online_history.py', 'src/reconciliation/robotless_online.py',
                 'src/reconciliation/se2.py', 'src/reconciliation/join_source02_geometry.py',
                 'src/reconciliation/gp_se2_environment.py', 'scripts/isaac/join_source02_preflight.py',
                 'scripts/isaac/join_source03_render.py', 'scripts/lightnav/robotless_online_server.py',
                 'scripts/lightnav/robotless_successive_server.py', 'scripts/validate_robotless_online_handoffs.py']:
        files.append(ROOT / name)
    save(run / 'freeze.json', dict(preparation_sha=git('rev-parse', 'HEAD'),
         source_sha256={str(p.relative_to(ROOT)): sha(p) for p in files},
         input_sha256={str(p): sha(p) for p in run.rglob('*') if p.is_file()}, scientific_episodes=ORDER))


def comparison(run):
    baseline = Path(read(run / 'source.json')['baseline_run'])
    rows = []
    for label, path in [('historical', baseline), ('new', run)]:
        result = read(path / 'aggregate/analysis.json')
        for episode in result['episodes']:
            s = episode['summary']
            rows.append(dict(provenance=label, episode=s['episode'], termination=s['termination'],
                predictions=s['predictions'], applied=s['activated_chunks'], outcome=s['outcome'],
                minimum_execution_edge_m=s['actual_minimum_curve_bounded_clearance_m'],
                final_longitudinal_m=s['final_hallway_longitudinal_m'],
                first_onset=s['first_onset'], first_bypass=s['first_bypass'],
                physically_past_rear=s['physically_past_rear'],
                episode_RTF=s['episode_rtf'], maximum_loop_stall_s=s['max_loop_stall_s'],
                request_RTFs=[r.get('inflight_rtf') for r in episode['rows']],
                chunks=[dict(chunk=r['chunk_id'], raw_text=r.get('raw_text'), pointing=r.get('pointing'),
                             arc_m=r.get('raw_arc_m'), lateral_m=r.get('max_lateral_m'),
                             center_distance_m=r.get('observation_cart_center_distance_m'),
                             raw_safe=r.get('raw_safe'), t_obs=r.get('t_obs'), t_apply=r.get('t_apply'))
                        for r in episode['rows']]))
    return dict(rows=rows, instruction=INSTRUCTION,
                input_comparison=read(run / 'config_comparison.json'),
                caveat='Same declared scenario, different live histories; not a same-image causal pair. STOP is not verified goal arrival.',
                source_sha256={str(p): sha(p) for p in [baseline / 'aggregate/analysis.json', run / 'aggregate/analysis.json']})


def presentation(run):
    from report_join_online02 import presentation as legacy_presentation
    # Scientific plotting implementation reused literally; its legacy-labelled
    # helper index is retained under review/. The primary index is run/index.html.
    legacy_presentation(run)
    report = comparison(run)
    save(run / 'aggregate/instruction_comparison.json', report)
    header = ('<meta charset="utf-8"><style>body{font:16px sans-serif;margin:30px}img{max-width:100%}'
              'td,th{border:1px solid #aaa;padding:6px}table{border-collapse:collapse}</style>'
              '<h1>JOIN-ONLINE-03: destination-first instruction</h1><blockquote>' + html.escape(INSTRUCTION) + '</blockquote>'
              '<p>ACTUAL ONLINE EXECUTION. New live RGB; no GP/reconciliation or controller change. '
              'Cart is absent throughout OFF and present throughout ON. STOP does not establish hallway-end arrival.</p>')
    header += '<table><tr><th>provenance</th><th>episode</th><th>termination</th><th>responses</th><th>edge m</th><th>longitudinal m</th><th>bypass</th></tr>'
    for r in report['rows']:
        values = [r['provenance'], r['episode'], r['termination'], r['predictions'],
                  round(r['minimum_execution_edge_m'], 6), round(r['final_longitudinal_m'], 6), r['first_bypass']]
        header += '<tr>' + ''.join('<td>' + html.escape(str(v)) + '</td>' for v in values) + '</tr>'
    header += '</table><p>OFF geometry involving the cart is hypothetical. '
    header += 'Timing validity is separate; see <a href="review/timing_and_calls.json">timing and call counts</a>.</p>'
    for item in read(run / 'review/plot_manifest.json')['figures']:
        n = item['name']
        header += f'<h2>{n}</h2><a href="review/{n}.json">numeric/hash sidecar</a><br><img src="review/{n}.png">'
    with (run / 'index.html').open('x') as stream:
        stream.write(header)
    with zipfile.ZipFile(run / 'instruction_review_bundle.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in [run / 'index.html', *sorted((run / 'review').glob('*.png')), *sorted((run / 'review').glob('*.json')),
                  run / 'aggregate/instruction_comparison.json', run / 'aggregate/chunks.csv',
                  run / 'aggregate/call_counts.json', run / 'protocol.json', run / 'source.json', run / 'config_snapshot.yaml']:
            archive.write(p, p.relative_to(run))
    print(run / 'index.html')


def validate(run):
    verify(run, reporting=True)
    cfg = yaml.safe_load((run / 'config_snapshot.yaml').read_text())
    baseline = Path(read(run / 'source.json')['baseline_run'])
    declaration = yaml.safe_load(CONFIG.read_text())
    assert cfg == variant_config(yaml.safe_load((baseline / 'config_snapshot.yaml').read_text()), declaration)
    for name in ['scenario.json', 'start_selection.json', 'side_passages.json']:
        assert (run / name).read_bytes() == (baseline / name).read_bytes()
    completion = read(run / 'schedule_completion.json')
    assert completion['no_retry'] and [x['episode'] for x in completion['episodes']] == ORDER
    contract, sampler = history_contract((ROOT / cfg['paths']['lightnav_checkout']).resolve(),
                                        (ROOT / cfg['paths']['checkpoint_path']).resolve())
    base, on, _, _ = environments(run)
    reports, sessions = [], []
    for eid in ORDER:
        ep = run / 'episodes' / eid
        meta = read(ep / 'metadata.json')
        dt = meta['resolved_integration_dt_s']
        reports.append(validate_episode(ep, run, contract, sampler, require_plots=False, integration_dt_s=dt))
        assert meta['instruction'] == INSTRUCTION and meta['cart_present'] == eid.startswith('ON_')
        assert meta['R0'] == read(run / 'start_selection.json')['selected']['pose_world']
        assert meta['activated_handoffs'] <= 20
        if meta['initial_activation_sim_time_s'] is not None:
            assert meta['end_sim_time_s']-meta['initial_activation_sim_time_s'] <= 25+dt+1e-8
        sessions.append(read(ep / 'session_open.json')['connection_id'])
        for p, h in read(ep / 'acquisition_extension_manifest.json')['files'].items():
            assert sha(ep / p) == h
        states, commands, guards = csvread(ep / 'execution.csv'), csvread(ep / 'commands.csv'), jsonlines(ep / 'guard.jsonl')
        assert len(states) == len(commands)+1 and len(guards) == len(commands)+(meta['status'] == ABORT)
        for i, g in enumerate(guards):
            expected = guard_check(on if meta['cart_present'] else base, g['start_pose'], g['command'], dt)
            for k, v in expected.items():
                equal_record(g[k], v, 'guard.'+k)
            assert g['cart_present'] == meta['cart_present']
            np.testing.assert_allclose(g['start_pose'], [float(states[i][k]) for k in ['x', 'y', 'yaw']], rtol=0, atol=1e-10)
            if g['safe']:
                assert all(float(commands[i][k]) == g['command'][k] for k in ['v_mps', 'omega_radps'])
                assert commands[i]['reason'] == g['command']['reason']
            else:
                assert i == len(commands) and not read(ep / 'guard_abort.json')['command_applied']
        captures = {r['frame_id']: r for r in jsonlines(ep / 'capture.jsonl')}
        visibility = jsonlines(ep / 'visibility.jsonl')
        assert set(captures) == {r['frame_id'] for r in visibility}
        for v in visibility:
            frame = captures[v['frame_id']]
            assert v['same_render_product_state'] and not v['model_input']
            assert v['pose_world'] == frame['pose_world'] and v['camera'] == frame['camera']
            assert v['state_id'] == frame['rendered_state_id'] and v['sim_time_s'] == frame['capture_sim_time_s']
            assert v['cart_transform'] == read(run / 'scenario.json')['prop']['wrapper_matrix_column']
            mask = np.load(ep / v['mask_path'])['mask']
            assert sha(ep / v['mask_path']) == v['mask_sha256']
            prefix = read(run / 'scenario.json')['prop']['runtime_prim']
            ids = [int(k) for k, label in v['instance']['idToLabels'].items() if str(label).startswith(prefix)]
            assert sorted(ids) == sorted(v['instance']['matched_instance_ids'])
            assert int(np.isin(mask, ids).sum()) == v['instance']['visible_pixels']
            if not meta['cart_present']:
                assert v['instance']['visible_pixels'] == 0
    assert len(set(sessions)) == 4
    equal_record(read(run / 'aggregate/analysis.json'), analyze(run), 'all_saved_analysis')
    equal_record(read(run / 'aggregate/call_counts.json'), call_counts(run), 'calls')
    equal_record(read(run / 'aggregate/instruction_comparison.json'), comparison(run), 'comparison')
    manifest = read(run / 'review/plot_manifest.json')
    for p, h in manifest['source_hashes'].items():
        assert sha(p) == h
    for item in manifest['figures']:
        assert sha(run / 'review' / (item['name']+'.png')) == item['png_sha256']
        assert sha(run / 'review' / (item['name']+'.json')) == item['sidecar_sha256']
    for name, field in [('clearance', 'minimum_clearance_m'), ('lateral', 'max_lateral_m'), ('arc', 'raw_arc_m')]:
        expected = [dict(episode=r['episode'], chunk=r['chunk_id'], x=r['observation_cart_center_distance_m'],
                    y=r['raw_geometry']['whole'][field] if name == 'clearance' else r[field])
                    for e in read(run / 'aggregate/analysis.json')['episodes'] for r in e['rows'] if 'world' in r]
        equal_record(read(run / 'review' / (name+'_vs_distance.json'))['data'], expected, 'figure_numbers')
    return dict(valid=True, reports=reports, declaration_parity=True, source_preserved=True,
                new_model_predictions=0, new_MPC_solves=0, new_GP_solves=0, new_execution=0,
                scientific_failure_is_not_artifact_failure=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--mode', choices=['init', 'freeze', 'verify', 'start', 'stop', 'analyze', 'report', 'validate'], required=True)
    a = p.parse_args(); run = a.run.resolve()
    if a.mode == 'init': initialize(run)
    elif a.mode == 'freeze': freeze(run)
    elif a.mode == 'verify': verify(run, pushed=True)
    elif a.mode == 'start': start(run)
    elif a.mode == 'stop':
        subprocess.run([sys.executable, str(ROOT / 'scripts/lightnav/robotless_online_server.py'), 'stop', str(run)], check=True)
    elif a.mode == 'analyze':
        subprocess.run([sys.executable, str(ROOT / 'scripts/analyze_join_online02.py'), '--run', str(run)], check=True)
    elif a.mode == 'report': presentation(run)
    elif a.mode == 'validate':
        result = validate(run); save(run / 'validation.json', result); print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
