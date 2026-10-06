#!/usr/bin/env python3
"""Authenticate OSA03; freeze one continuous native acquisition before any call."""
import argparse
from pathlib import Path
import subprocess
import sys
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
from reconciliation.join_source03 import read, save, sha
from run_join_source05 import verify as inherited_verify, start as inherited_start, git
from reconciliation.continuous_obstacle_reveal_episode01 import EPISODES
CONFIG = ROOT/'configs/continuous_obstacle_reveal_episode_01.yaml'


def verify(run, pushed=False):
    result = inherited_verify(run, pushed)
    audit = read(run/'external_authority.json')
    for p,h in audit['sha256'].items():
        assert sha(p)==h,p
    return result


def start(run):
    verify(run, True)
    inherited_start(run)


def prepare(run):
    cfg = yaml.safe_load(CONFIG.read_text()); source = ROOT/cfg['authority']
    from validate_obstacle_source_acquisition03 import validate
    validated = validate(source)
    assert validated['valid'] and validated['rows'][0]['qualified']
    run.mkdir(parents=True, exist_ok=False); (run/'logs').mkdir()
    save(run/'technical_preflight/OSA03_saved_revalidation.json', validated)
    for name in ('selected_pose11.json', 'scenario.json', 'side_passages.json'):
        with (run/name).open('xb') as f: f.write((source/name).read_bytes())
    old = read(source/'protocol.json')
    resolved = yaml.safe_load((source/'config_snapshot.yaml').read_text())
    resolved['online']['maximum_handoff_attempts'] = cfg['maximum_handoff_attempts']
    assert resolved['online']['maximum_active_sim_s'] == cfg['maximum_active_sim_s']
    assert resolved['online']['postroll_sim_s'] == cfg['postroll_sim_s']
    with (run/'config_snapshot.yaml').open('x') as f: yaml.safe_dump(resolved, f, sort_keys=False)
    save(run/'protocol.json', dict(experiment=cfg['experiment'], declaration=cfg,
        candidate=old['candidate'], initial_pose_world=old['initial_pose_world'],
        initial_physical_command=old['initial_physical_command'], instruction=old['instruction'],
        side=old['side'], prior_side_caveat=old['prior_side_caveat'], gates=old['gates'],
        frames='Isaac world XY metres +Z up CCW yaw radians; local +x forward +y left; world=A_k*raw_k',
        waypoint_dt=None, clock_rule='host latency only host differences; simulation latency only simulation differences',
        no_reconciliation=True, reset_after_initialization=0))
    save(run/'episode_schedule.json', dict(episodes=[dict(episode_id=EPISODES[0], R0=old['initial_pose_world'],
        instruction=old['instruction'], cart_present_initial=False, cart_present_dynamic=True)]))
    previous = read(source/'source.json')
    preserved = {**previous['preserved'], **{str(q.resolve()):sha(q) for q in source.rglob('*') if q.is_file()}}
    save(run/'source.json', dict(starting_sha=git('rev-parse','HEAD'), origin_main=git('rev-parse','origin/main'),
        OSA03=str(source.resolve()), authority_episode=cfg['authority_episode'],
        source04_run=previous['source04_run'], preserved=preserved))
    save(ROOT/'results/continuous_obstacle_reveal_episode_01/protocol_inputs.json', dict(
        starting_sha=git('rev-parse','HEAD'), run=str(run), authority=str(source),
        authority_episode=cfg['authority_episode'], initial_pose=old['initial_pose_world'],
        instruction=old['instruction'], prepared_files={str(q.relative_to(run)):sha(q)
            for q in run.iterdir() if q.is_file()}, preserved_file_count=len(preserved),
        declaration=cfg))


def freeze(run):
    source = Path(read(run/'source.json')['OSA03'])
    cfg = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    from reconciliation.join_source03 import mpc_audit
    audit = mpc_audit(ROOT)
    assert audit['status']=='MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL'
    assert not audit['current']['external_git_status']
    checkpoint = (ROOT/cfg['paths']['checkpoint_path']).resolve()
    hashes = {str(checkpoint/p):sha(checkpoint/p) for p in cfg['lightnav']['checkpoint_sha256']}
    assert hashes == {str(checkpoint/p):h for p,h in cfg['lightnav']['checkpoint_sha256'].items()}
    hashes[audit['current']['mpc_source']] = audit['current']['mpc_source_sha256']
    save(run/'external_authority.json',dict(sha256=hashes,mpc_audit=audit,checkpoint_revision=cfg['lightnav']['checkpoint_revision']))
    inherited = read(source/'freeze.json')['source_sha256']
    for p, h in inherited.items():
        assert sha(ROOT/p) == h, p
    files = [str(p.relative_to(ROOT)) for pattern in (
        'src/reconciliation/continuous_obstacle_reveal*.py', 'scripts/*continuous_obstacle_reveal*.py',
        'scripts/isaac/continuous_obstacle_reveal*.py', 'tests/test_continuous_obstacle_reveal*.py',
        'configs/continuous_obstacle_reveal*.yaml') for p in ROOT.glob(pattern)]
    # All reused analysis/runtime modules are bound, including transitive imports.
    files += [p for p in git('ls-files','src','scripts').splitlines() if p.endswith('.py')]
    save(run/'freeze.json', dict(source_sha256={**inherited, **{p:sha(ROOT/p) for p in files}},
        input_sha256={str(p.resolve()):sha(p) for p in run.rglob('*') if p.is_file()},
        episodes=EPISODES, maximum_terminal_predictions=4, retries=0, optimizer_calls=0))
    save(ROOT/'results/continuous_obstacle_reveal_episode_01/freeze_manifest.json',dict(
        freeze_file_sha256=sha(run/'freeze.json'),external_authority_sha256=sha(run/'external_authority.json'),
        frozen_code_file_count=len(read(run/'freeze.json')['source_sha256']),
        input_sha256=read(run/'freeze.json')['input_sha256'],scientific_calls_before_freeze=0))


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--run', type=Path, required=True)
    p.add_argument('--mode', choices=['prepare','freeze','verify','start','stop'], required=True)
    a = p.parse_args(); run = a.run.resolve()
    if a.mode == 'verify': verify(run, True)
    elif a.mode == 'stop':
        subprocess.run([sys.executable, str(ROOT/'scripts/lightnav/robotless_online_server.py'), 'stop', str(run)], check=True)
    else: dict(prepare=prepare, freeze=freeze, start=start)[a.mode](run)
