#!/usr/bin/env python3
"""Exploratory 02 authentication and one sanitized, zero-extra-reserve acquisition."""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
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

CONFIG = ROOT/'configs/continuous_obstacle_reveal_exploratory_02.yaml'
OUT = ROOT/'results/continuous_obstacle_reveal_exploratory_02'
COPIED = ('selected_pose11.json', 'scenario.json', 'side_passages.json',
          'config_snapshot.yaml', 'episode_schedule.json', 'external_authority.json')


def declaration():
    return yaml.safe_load(CONFIG.read_text())


def namespace(run):
    run = Path(run).resolve()
    if run.parent != ROOT/'data/continuous_obstacle_reveal_exploratory_02':
        raise ValueError('Exploratory 02 must use its own run namespace')
    return run


from run_continuous_obstacle_reveal_episode01b import (
    sanitized_environment, historical_launch, launch_argv, tree_hashes, loader_checks,
    reserve_launch, observe_process)
from reconciliation.continuous_obstacle_reveal_exploratory02 import validate_policy


def authenticate01(cfg):
    old = ROOT/cfg['historical_run']; out = ROOT/cfg['historical_results']
    from validate_continuous_obstacle_reveal_episode01b import validate
    from validate_robotless_online_handoffs import equal_record
    equal_record(validate(old), read(old/'attempt_validation.json'))
    for p in out.rglob('*'):
        if p.is_file():
            blob = subprocess.check_output(['git','show',cfg['historical_commit']+':'+str(p.relative_to(ROOT))],cwd=ROOT)
            assert p.read_bytes() == blob, str(p)
    return {**tree_hashes(old), **tree_hashes(out)}


def equivalence(old, run):
    p0, p1 = read(old/'protocol.json'), read(run/'protocol.json')
    a, b = deepcopy(p0), deepcopy(p1)
    policy = b.pop('exploratory_clearance')
    validate_policy(policy)
    b['declaration']['classification_priority'] = a['declaration']['classification_priority']
    for p in (a,b):
        p.pop('experiment'); p['declaration'].pop('experiment')
    if a != b:
        raise ValueError('undeclared scientific protocol difference')
    for name in COPIED:
        if (old/name).read_bytes() != (run/name).read_bytes():
            raise ValueError('historical runtime/input bytes changed: '+name)
    return dict(unchanged_runtime_and_model_input_files={n:sha(run/n) for n in COPIED},
        unchanged_old_protocol_except_namespace_classification_and_policy=True,
        sole_scientific_intervention=dict(old_required_extra_clearance_m=.05,new_policy=policy),
        scope='reference acceptance and execution guard only; not a LightNav input',
        administrative_changes=['new namespace','classification/dual diagnostics','passive observer OSError handling'])


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
    p['exploratory_clearance'] = validate_policy(cfg['exploratory_clearance'])
    p['declaration']['classification_priority'] = cfg['classification_priority']
    save(run/'protocol.json', p)
    save(run/'technical_preflight/OSA03_saved_revalidation.json', v)
    save(run/'historical01b_preservation.json', dict(files=preserved,
        historical_freeze='86c54fbd78b66282c4eb6707a8b9f38e5faba12e', result=cfg['historical_commit']))
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
    files.update(str(p.relative_to(ROOT)) for pattern in ('scripts/*continuous_obstacle_reveal*02.py', 'scripts/isaac/*continuous_obstacle_reveal*02.py',
        'src/reconciliation/continuous_obstacle_reveal_exploratory02.py',
        'tests/test_continuous_obstacle_reveal_exploratory02.py','configs/continuous_obstacle_reveal_exploratory_02.yaml') for p in ROOT.glob(pattern))
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
    for p,h in read(run/'historical01b_preservation.json')['files'].items(): assert sha(p) == h, p
    for p,h in read(run/'launch_environment.json')['external_sha256'].items(): assert sha(p) == h, p
    loader=read(run/'loader_verification.json')
    assert loader['valid'] and sha(cfg['libcusparse'])==loader['libcusparse_sha256']
    for case in loader['cases']:
        assert sha(case['selected_path'])==case['library_sha256']
        assert sha(case['output'])==case['output_sha256']
    return result


def launch(run):
    verify(run, True); cfg=declaration()
    assert read(run/'generation_actual.json')['valid']
    argv = launch_argv(run,cfg); assert argv == read(run/'launch_environment.json')['argv']
    reserve_launch(run,argv)
    samples=[]; observed=set(); observer_errors=[]
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
                except (OSError, json.JSONDecodeError) as exc:
                    record = dict(error=repr(exc), at=stamp())
                    if not observer_errors or observer_errors[-1]['error'] != record['error']:
                        observer_errors.append(record)
                        save(run/'observer_errors'/f'{len(observer_errors):03}.json',record)
            time.sleep(.1)
    save(run/'launch_result.json', dict(returncode=child.returncode, at=stamp(), observer_errors=observer_errors,
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
