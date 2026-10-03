#!/usr/bin/env python3
"""Authenticate saved V2, freeze installation, execute only four V2 rollouts."""
import argparse
import json
from pathlib import Path
import shutil
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
import run_relative_factor_multisource01 as runtime
import run_b_to_entry_bridge01 as bridge_history
from reconciliation.join_source03 import read, save, sha
from reconciliation.vector_bridge_execution03 import *
from reconciliation.relative_factor_multisource import geometry, reference_safety
from reconciliation.robotless_online import stamp
from run_osa03_native_continuation import git, plain
CONFIG = ROOT/'configs/b_to_entry_vector_bridge_execution_03.yaml'
RESULTS = ROOT/'results/b_to_entry_vector_bridge_execution_03'
DOC = ROOT/'docs/B_TO_ENTRY_VECTOR_BRIDGE_EXECUTION_03.md'
FILES = [CONFIG, ROOT/'src/reconciliation/vector_bridge_execution03.py',
    *[ROOT/'scripts'/f'{s}_b_to_entry_vector_bridge_execution03.py' for s in ['run','validate','report']],
    ROOT/'tests/test_b_to_entry_vector_bridge_execution03.py']


def authenticate(cfg, deep=False):
    for key in ['diag_result', 'bridge_result', 'entry_result']:
        assert sha(ROOT/cfg[key]) == cfg[key+'_sha256'], key
    d, h = [read(ROOT/cfg[k]) for k in ['diag_result', 'bridge_result']]
    assert d['summary']['scientific_freeze_sha'] == cfg['diag_freeze_sha']
    assert git('show', cfg['diag_result_commit']+':'+cfg['diag_result']) == (ROOT/cfg['diag_result']).read_text().strip()
    from run_b_to_entry_graph_formulation_diag02 import verify as verify_diag
    verify_diag(Path(d['run'])); bridge_history.verify(Path(h['run']))
    for result in [d, h, read(ROOT/cfg['entry_result'])]:
        run = Path(result['run'])
        assert sha(run/'result_hashes.json') == result['result_hashes_sha256']
        for p, digest in read(run/'result_hashes.json').items(): assert sha(run/p) == digest, p
    if deep:
        from validate_b_to_entry_graph_formulation_diag02 import validate
        ds, dv = validate(Path(d['run']))
        assert ds == d['summary'] and dv['valid'] and dv['all_historical_validators']
    assert cfg['sources'] == SOURCE_IDS and cfg['order'] == ORDER and cfg['new_method'] == VECTOR
    assert cfg['optimizer_calls'] == 0 and cfg['new_rollouts'] == 4 and cfg['M'] == 2
    assert cfg['integration_steps'] == 180 and cfg['primary_intervals'] == 54
    assert cfg['scalar_atol'] == ATOL and cfg['time_tolerance'] == 'integration_dt_s - 1e-9'
    for s in d['summary']['selected_sources']:
        q = d['summary']['sources'][s['label']]['V2']; f = Path(s['folder'])/'planning/V2'
        assert q['stable'] and q['converged'] and q['M'] == 2
        for name in ['last_accepted_bridge.npy', 'converged_reference.npy']:
            assert str((f/name).relative_to(Path(d['run']))) in read(Path(d['run'])/'result_hashes.json')
        assert np.load(f/'last_accepted_bridge.npy').tobytes() == np.asarray(q['last_accepted_bridge']).tobytes()
    return d, h


def prepare(run):
    with no_reconciliation_optimizer():
        cfg = yaml.safe_load(CONFIG.read_text()); d, h = authenticate(cfg, deep=True)
        run.mkdir(parents=True, exist_ok=False); save(run/'protocol.json', cfg); selected = []
        for spec in d['summary']['selected_sources']:
            old = Path(spec['historical_folder']); diag = Path(spec['folder']); sid = spec['id']
            folder = run/'sources'/spec['label']; (folder/'references').mkdir(parents=True)
            copies = {}
            for name in ['common_state.json','source_manifest.json','schedule.json','scenario.json',
                         'metric_protocol.json','recorded_old_to_B.npy','source_audit.json','entry.json',
                         'bridge_input.json','suffix_native_installed.npy','hermite_bridge.npy']:
                shutil.copyfile(old/name, folder/name); copies[name] = dict(path=str(old/name), sha256=sha(old/name))
            save(folder/'protocol.json', cfg)
            refs = read(old/'final_references.json'); reuse_old = read(old/'reuse.json')
            methods = {n: dict(folder=str(old/'methods'/n) if n == HERMITE else reuse_old['methods'][n]['folder'], reference=refs[n]) for n in ORDER[:-1]}
            for rec in methods.values(): rec['hashes_sha256'] = sha(Path(rec['folder'])/'hashes.json')
            save(folder/'reuse.json', dict(copies=copies, methods=methods, historical_folder=str(old), diag_folder=str(diag)))
            c = read(folder/'common_state.json'); b = read(folder/'bridge_input.json'); p = diag/'planning/V2'
            shutil.copyfile(p/'last_accepted_bridge.npy', folder/'v2_bridge.npy')
            wp = folder/'references'/f'{VECTOR}_world.npy'; shutil.copyfile(p/'converged_reference.npy', wp)
            raw = np.load(refs[NATIVE]['local_path']); before = raw.tobytes()
            w, local, labels = installed_reference(np.load(folder/'v2_bridge.npy'), np.load(wp),
                np.load(folder/'suffix_native_installed.npy'), c['fresh_capture_pose'], raw, b['original_ids'], b['B'], b['E'])
            assert raw.tobytes() == before
            lp = folder/'references'/f'{VECTOR}_local.npy'
            with lp.open('xb') as stream: np.save(stream, local, allow_pickle=False)
            native = np.asarray(read(Path(methods[NATIVE]['folder'])/'restoration.json')['installed_world'])
            assert w[3:].tobytes() == native[b['original_ids']].tobytes()
            safe = reference_safety(w, geometry(folder))
            ref = dict(world_path=str(wp), world_sha256=sha(wp), local_path=str(lp), local_sha256=sha(lp),
                labels=labels, safety=safe, status='REFERENCE_GEOMETRY_SAFE' if safe['clearance_valid'] else 'REFERENCE_GEOMETRY_UNSAFE')
            save(folder/'prepared_references.json', {VECTOR: ref})
            save(folder/'references.json', {**refs, VECTOR: ref})
            shutil.copyfile(refs[NATIVE]['world_path'], folder/'references/M0_NATIVE_world.npy')
            install = dict(performed=False, numerical_MPC_calls=0)
            if safe['clearance_valid']:
                runtime.prior.preflight(folder)
                pre = read(folder/'restoration_preflight.json'); installed = np.asarray(pre['rows'][0]['installed_world'])
                assert installed[3:].tobytes() == native[b['original_ids']].tobytes()
                ip = folder/'installed_preflight_world.npy'
                with ip.open('xb') as stream: np.save(stream, installed, allow_pickle=False)
                actual_safe = reference_safety(installed, geometry(folder)); assert actual_safe['clearance_valid']
                install = dict(performed=True, numerical_MPC_calls=0, labels=labels, installed_path=str(ip),
                    installed_sha256=sha(ip), installed_safety=actual_safe,
                    derived_row_roundoff_max_abs=float(np.max(abs(installed[:3]-w[:3]))), downstream_bit_exact=True)
            save(folder/'installation.json', install)
            save(folder/'planning_reuse.json', dict(diag_bridge_path=str(p/'last_accepted_bridge.npy'),
                diag_bridge_sha256=sha(p/'last_accepted_bridge.npy'), diag_reference_path=str(p/'converged_reference.npy'),
                diag_reference_sha256=sha(p/'converged_reference.npy'), optimizer_calls=0, V2_stable=True))
            selected.append(dict(spec, folder=str(folder), diag_folder=str(diag)))
        save(run/'selected_sources.json', selected)
        save(run/'authentication.json', dict(DIAG02_and_all_historical_validators=True, optimizer_calls=0,
            numerical_MPC_preflight_calls=0, exact_saved_V2_references=4, historical_rollouts_reused=12))
        print(json.dumps(dict(prepared=str(run), optimizer_calls=0, numerical_MPC_calls=0)))


def freeze(run):
    cfg = yaml.safe_load(CONFIG.read_text()); d, _ = authenticate(cfg); assert cfg == read(run/'protocol.json')
    files = list(dict.fromkeys([Path(p) for p in read(Path(d['run'])/'freeze.json')['files']] + FILES))
    with (run/'protocol_document.md').open('xb') as f: f.write(DOC.read_bytes())
    save(run/'freeze.json', dict(files={str(p):sha(p) for p in files}, inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()},
        optimizer_calls=0, maximum_V2_rollouts=4, retries=0))
    RESULTS.mkdir(parents=True, exist_ok=True)
    save(RESULTS/'freeze_summary.json', dict(run=str(run), freeze=read(run/'freeze.json'),
        authentication=read(run/'authentication.json'), selected_sources=read(run/'selected_sources.json'),
        sources={s['label']:{k:read(Path(s['folder'])/(k+'.json')) for k in ['planning_reuse','common_state','schedule','references','installation']}
                 for s in read(run/'selected_sources.json')}))


def verify(run, pushed=False):
    authenticate(read(run/'protocol.json'))
    f = read(run/'freeze.json')
    for group in ['files', 'inputs']:
        for p, digest in f[group].items(): assert sha(p) == digest, p
    if pushed:
        assert git('rev-parse','HEAD') == git('ls-remote','origin','refs/heads/main').split()[0]
        for p in f['files']:
            assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT))) == git('hash-object',p), p
        assert git('rev-parse','HEAD:'+str(DOC.relative_to(ROOT))) == git('hash-object',str(run/'protocol_document.md'))
        assert not git('diff','HEAD','--',str(RESULTS/'freeze_summary.json'))


def execute(run):
    with no_reconciliation_optimizer():
        verify(run, pushed=True)
        save(run/'execution_start.json', dict(sha=git('rev-parse','HEAD'), started=stamp(), optimizer_calls=0, retries=0))
        for spec in read(run/'selected_sources.json'):
            folder = Path(spec['folder']); ref = read(folder/'references.json')[VECTOR]
            if ref['safety']['clearance_valid']: runtime.run_method(folder, VECTOR)
            else: save(folder/'methods'/VECTOR/'skipped.json', dict(status=ref['status'], metrics=None, rollouts=0))
        verify(run)
        save(run/'completion.json', dict(completed=stamp(), optimizer_calls=0, historical_reruns=0, V3_rollouts=0,
            LightNav=0, RGB=0, Isaac=0, retries=0))


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--run', type=Path, required=True)
    p.add_argument('--mode', choices=['prepare','freeze','execute'], required=True)
    a=p.parse_args();globals()[a.mode](a.run.resolve())
