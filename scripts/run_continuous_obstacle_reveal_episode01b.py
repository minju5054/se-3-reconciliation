#!/usr/bin/env python3
"""01B provenance and process-local launch; the scientific 01 collector is unchanged."""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
from reconciliation.join_source03 import read, save, sha
from reconciliation.robotless_online import stamp
from run_continuous_obstacle_reveal_episode01 import verify as verify01, start as start01
from run_join_source05 import git

CONFIG = ROOT/'configs/continuous_obstacle_reveal_episode_01b.yaml'
OUT = ROOT/'results/continuous_obstacle_reveal_episode_01b'
COPIED = ('selected_pose11.json', 'scenario.json', 'side_passages.json',
          'config_snapshot.yaml', 'episode_schedule.json', 'external_authority.json')


def declaration():
    return yaml.safe_load(CONFIG.read_text())


def namespace(run):
    run = Path(run).resolve()
    if run.parent != ROOT/'data/continuous_obstacle_reveal_episode_01b':
        raise ValueError('01B must use its own run namespace')
    return run


def sanitized_environment(parent, cfg):
    env = dict(parent)
    for key in cfg['unset_environment']:
        env.pop(key, None)
    env.update(cfg['set_environment'])
    return env


def historical_launch(cfg):
    lines = (ROOT/cfg['launch_source']).read_text().splitlines()
    line, = [s for s in lines if s.startswith('env -u LD_LIBRARY_PATH ') and
             'scripts/isaac/obstacle_source03_online.py --run ' in s]
    argv = shlex.split(line); i = 1; unset = []; explicit = {}
    while argv[i] == '-u':
        unset.append(argv[i+1]); i += 2
    while '=' in argv[i]:
        k, v = argv[i].split('=', 1); explicit[k] = v; i += 1
    assert unset == cfg['unset_environment']
    assert explicit == cfg['set_environment']
    assert argv[i] == cfg['isaac_python']
    return dict(command=line, unset=unset, explicit=explicit,
                source=cfg['launch_source'], source_sha256=sha(ROOT/cfg['launch_source']))


def launch_argv(run, cfg):
    return (['env'] + [v for k in cfg['unset_environment'] for v in ('-u', k)] +
            [f'{k}={v}' for k, v in cfg['set_environment'].items()] +
            [cfg['isaac_python'], cfg['collector'], '--run', str(run)])


def tree_hashes(path):
    return {str(p.resolve()): sha(p) for p in sorted(path.rglob('*')) if p.is_file()}


def authenticate01(cfg):
    old = ROOT/cfg['historical_run']; out = ROOT/cfg['historical_results']
    verify01(old)
    # Every tracked historical result must still match the diagnosed commit.
    for p in out.rglob('*'):
        if p.is_file():
            blob = subprocess.check_output(['git', 'show', cfg['historical_commit']+':'+str(p.relative_to(ROOT))], cwd=ROOT)
            assert p.read_bytes() == blob, str(p)
    result = read(out/'result_summary.json')
    assert result['classification'] == 'TECHNICAL_EXECUTION_BLOCKED'
    assert result['calls']['scientific_episodes_initialized'] == 0
    assert not (old/'episodes').exists()
    assert read(out/'blocked_evidence_validation.json')['valid_technical_evidence']
    for name, h in result['evidence']['input_sha256'].items():
        assert sha(old/name) == h
    return {**tree_hashes(old), **tree_hashes(out)}


def equivalence(old, run):
    """Whole records plus named scientific fields; only experiment labels differ."""
    p0, p1 = read(old/'protocol.json'), read(run/'protocol.json')
    a, b = deepcopy(p0), deepcopy(p1)
    for p in (a, b):
        p.pop('experiment'); p['declaration'].pop('experiment')
    if a != b:
        raise ValueError('scientific protocol differs from frozen 01')
    c0 = yaml.safe_load((old/'config_snapshot.yaml').read_text())
    c1 = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    if c0 != c1:
        raise ValueError('scientific runtime config differs from frozen 01')
    for name in COPIED:
        assert (old/name).read_bytes() == (run/name).read_bytes(), name
    def fields(p, c, scene, run):
        d = p['declaration']
        return dict(initial_pose=p['initial_pose_world'], initial_command=p['initial_physical_command'],
            instruction=p['instruction'], scene_provenance=scene,
            cart_transform=scene['prop']['wrapper_matrix_column'], camera=c['camera'],
            render_settings=dict(capture=c['capture'], lighting=c['lighting'], scene=c['scene']),
            LightNav_checkpoint_config=dict(paths=c['paths'], lightnav=c['lightnav']),
            MPC_source_config=dict(authority=read(run/'external_authority.json')['mpc_audit'], execution=c['execution']),
            geometry_safety=dict(gates=p['gates'], guard=c['join_online02']['guard'], raw_policy=d['raw_unsafe_policy']),
            capture_cadence=c['execution']['capture_hz'], MPC_cadence=c['execution']['control_hz'],
            integration_cadence=c['execution']['integration_hz'],
            reveal_rule=dict(implementation_sha256=sha(ROOT/'scripts/isaac/obstacle_source03_online.py'),
                             initial_cart=False, next_capture_strictly_after_C0_application=True),
            request_rule=d['request_rule'], max_terminal_predictions=d['maximum_terminal_predictions'],
            max_handoffs=d['maximum_handoff_attempts'], postroll=d['postroll_sim_s'],
            one_inflight=dict(policy_sha256=sha(ROOT/'src/reconciliation/continuous_obstacle_reveal_episode01.py'), maximum=1),
            no_reset=p['reset_after_initialization'], raw_observation_anchoring=p['frames'],
            waypoint_dt=p['waypoint_dt'], evolution_thresholds=d['evolution'],
            classification_rules=d['classification_priority'], retry=d['retry'],
            episode_schedule=read(run/'episode_schedule.json'), whole_runtime_config=c,
            whole_scientific_protocol={k:v for k,v in p.items() if k not in ('experiment','declaration')})
    f0 = fields(p0, c0, read(old/'scenario.json'), old)
    f1 = fields(p1, c1, read(run/'scenario.json'), run)
    checks = {k:dict(equal=f0[k] == f1[k], original=f0[k], new=f1[k]) for k in f0}
    assert all(r['equal'] for r in checks.values())
    return dict(all_scientific_fields_equal=True, fields=checks,
        byte_equal_files={n:sha(run/n) for n in COPIED},
        administrative_difference='separate 01B run/output namespace and experiment labels only',
        intentional_difference='process_launch_environment_only')


def loader_checks(run, cfg):
    cases = []
    for label, env in [('inherited', dict(os.environ)), ('sanitized', sanitized_environment(os.environ, cfg))]:
        command = ['ldd', '-r', cfg['libcusparse']]
        r = subprocess.run(command, env=env, capture_output=True, text=True, check=False)
        out = run/'technical_preflight'/f'loader_{label}.txt'
        with out.open('x') as f: f.write(r.stdout+r.stderr)
        match = re.search(r'libnvJitLink\.so\.12 => (\S+)', r.stdout)
        selected = match.group(1) if match else None
        missing = re.findall(r'undefined symbol: ([^,\s]+)', r.stdout+r.stderr)
        cases.append(dict(case=label, command=command, selected_path=selected,
            resolved_path=None if not selected else str(Path(selected).resolve()),
            library_sha256=None if not selected else sha(selected), undefined_symbols=missing,
            returncode=r.returncode, output=str(out), output_sha256=sha(out)))
    valid = cases[1]['returncode'] == 0 and cases[1]['selected_path'] is not None and not cases[1]['undefined_symbols']
    result = dict(valid=valid, cases=cases, libcusparse_sha256=sha(cfg['libcusparse']),
        process_only=True, full_SimulationApp_starts=0,
        status='PASS' if valid else 'TECHNICAL_PREFLIGHT_BLOCKED')
    save(run/'loader_verification.json', result)
    return result


def prepare(run):
    cfg = declaration(); historical = historical_launch(cfg)
    preserved = authenticate01(cfg); old = ROOT/cfg['historical_run']
    from validate_obstacle_source_acquisition03 import validate
    source = Path(read(old/'source.json')['OSA03']); v = validate(source)
    assert v['valid'] and v['rows'][0]['qualified']
    run.mkdir(parents=True, exist_ok=False); (run/'logs').mkdir(); (run/'technical_preflight').mkdir()
    for name in COPIED:
        with (run/name).open('xb') as f: f.write((old/name).read_bytes())
    p = read(old/'protocol.json')
    p['experiment'] = p['declaration']['experiment'] = cfg['experiment']
    save(run/'protocol.json', p)
    save(run/'technical_preflight/OSA03_saved_revalidation.json', v)
    save(run/'historical01_preservation.json', dict(files=preserved,
        historical_freeze='2334b8e528b77395e24bde956f31cc0ee5cb474b',
        blocked_result='a8409a0013d4fab91b9384ac6511bdaa9b3512f3', diagnosis=cfg['historical_commit']))
    prior = read(old/'source.json')
    save(run/'source.json', {**prior, 'starting_sha':git('rev-parse','HEAD'),
        'origin_main':git('rev-parse','origin/main'), 'preserved':{**prior['preserved'], **preserved}})
    save(run/'protocol_equivalence.json', equivalence(old, run))
    save(run/'launch_environment.json', dict(**historical, argv=launch_argv(run,cfg),
        inherited_removed_values={k:os.environ.get(k) for k in cfg['unset_environment']},
        authorized_launches=1, retries=0, full_preflight_startups=0,
        external_sha256={p:sha(p) for p in (cfg['isaac_python'], str(Path(cfg['isaac_python']).parent/'setup_python_env.sh'))}))
    loader = loader_checks(run, cfg)
    for name in ('protocol_equivalence.json','launch_environment.json','loader_verification.json'):
        with (OUT/name).open('x') as f:
            f.write((run/name).read_text())
    save(OUT/'protocol_inputs.json', dict(experiment=cfg['experiment'], run=str(run),
        starting_sha=git('rev-parse','HEAD'), prepared_sha256={str(q.relative_to(run)):sha(q) for q in run.rglob('*') if q.is_file()},
        preserved01_files=len(preserved), inherited_authority_files=len(prior['preserved']),
        loader_preflight_pass=loader['valid'], scientific_calls=0))
    if not loader['valid']: raise ValueError('TECHNICAL_PREFLIGHT_BLOCKED; no launch authorized')


def freeze(run):
    cfg = declaration(); assert read(run/'loader_verification.json')['valid']
    inherited = read(ROOT/cfg['historical_run']/'freeze.json')['source_sha256']
    for p,h in inherited.items(): assert sha(ROOT/p) == h, p
    files = set(inherited)
    files.update(str(p.relative_to(ROOT)) for pattern in ('scripts/*continuous_obstacle_reveal*01b.py',
        'tests/test_continuous_obstacle_reveal_episode01b.py','configs/continuous_obstacle_reveal_episode_01b.yaml') for p in ROOT.glob(pattern))
    save(run/'freeze.json', dict(source_sha256={p:sha(ROOT/p) for p in sorted(files)},
        input_sha256=tree_hashes(run), episodes=['EPISODE_00'], maximum_terminal_predictions=4,
        retries=0, optimizer_calls=0, maximum_SimulationApp_starts=1))
    verify(run)
    save(OUT/'freeze_manifest.json', dict(freeze_file_sha256=sha(run/'freeze.json'),
        source_files=len(files), prepared_input_files=len(read(run/'freeze.json')['input_sha256']),
        run=str(run), scientific_calls_before_freeze=0))


def verify(run, pushed=False):
    result = verify01(run, pushed)
    cfg = declaration(); historical_launch(cfg)
    assert read(run/'protocol_equivalence.json') == equivalence(ROOT/cfg['historical_run'], run)
    for p,h in read(run/'historical01_preservation.json')['files'].items(): assert sha(p) == h, p
    for p,h in read(run/'launch_environment.json')['external_sha256'].items(): assert sha(p) == h, p
    loader=read(run/'loader_verification.json')
    assert loader['valid'] and sha(cfg['libcusparse'])==loader['libcusparse_sha256']
    for case in loader['cases']:
        assert sha(case['selected_path'])==case['library_sha256']
        assert sha(case['output'])==case['output_sha256']
    return result


def reserve_launch(run, argv):
    if (run/'execution_start.json').exists() or (run/'episodes').exists():
        raise FileExistsError('no second startup/episode')
    save(run/'launch_attempt.json', dict(at=stamp(), scientific_freeze_sha=git('rev-parse','HEAD'),
        argv=argv, full_startup_budget=1, retries=0))


def observe_process(pid):
    """Read-only /proc sample; never communicate with or pause the child."""
    root = Path(f'/proc/{pid}')
    env = dict(s.split('=',1) for s in (root/'environ').read_bytes().decode().split('\0') if '=' in s)
    maps = (root/'maps').read_text().splitlines()
    paths = sorted({s.split()[-1] for s in maps if 'libnvJitLink.so' in s or 'libcusparse.so' in s})
    wanted = declaration()['unset_environment'] + list(declaration()['set_environment'])
    return dict(pid=pid, at=stamp(), relevant_environment={k:env.get(k) for k in wanted},
                libraries=paths, argv=(root/'cmdline').read_bytes().decode().split('\0'))


def launch(run):
    verify(run, True); cfg=declaration()
    assert read(run/'generation_actual.json')['valid']
    argv = launch_argv(run,cfg); assert argv == read(run/'launch_environment.json')['argv']
    reserve_launch(run,argv)
    samples=[]; observed=set()
    with (run/'logs/isaac.log').open('x') as log:
        child = subprocess.Popen(argv, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        while child.poll() is None:
            if (run/'execution_start.json').exists():
                try:
                    pid=read(run/'execution_start.json')['pid']
                    sample=observe_process(pid)
                    # Persist initial environment and each new selected-library set.
                    key=tuple(sample['libraries'])
                    if key not in observed:
                        observed.add(key); samples.append(sample)
                        save(run/'process_observations'/f'{len(samples):03}.json',sample)
                except (FileNotFoundError, ProcessLookupError, json.JSONDecodeError):
                    pass
            time.sleep(.1)
    save(run/'launch_result.json', dict(returncode=child.returncode, at=stamp(),
        full_launch_attempts=1, episode_directory_exists=(run/'episodes').exists(),
        scene_initialized=(run/'acquisition_scene.json').exists(),
        schedule_completed=(run/'schedule_completion.json').exists(),
        actual_library_sha256={p:sha(p) for s in samples for p in s['libraries']},
        no_retry=True, result_is_not_inferred_from_exit_code=True))
    print(read(run/'launch_result.json'),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['prepare','freeze','verify','start','launch','stop'],required=True)
    a=p.parse_args(); run=namespace(a.run)
    if a.mode=='prepare':
        OUT.mkdir(parents=True,exist_ok=True); prepare(run)
    elif a.mode=='verify': verify(run,True)
    elif a.mode=='start': verify(run,True); start01(run)
    elif a.mode=='stop':
        subprocess.run([sys.executable,str(ROOT/'scripts/lightnav/robotless_online_server.py'),'stop',str(run)],check=True)
    else: {'freeze':freeze,'launch':launch}[a.mode](run)


if __name__=='__main__': main()
