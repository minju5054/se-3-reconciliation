"""Authenticate the historical S3 chain; load authoritative arrays/state only."""
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import numpy as np
import yaml
from reconciliation.join_source03 import read,sha
from reconciliation.se2 import local_trajectory_to_world
from reconciliation.spatial_entry_suffix import suffix_reference
from reconciliation.boundary_row_ablation04 import staged_reference
from reconciliation.s3_isaac_ablation01 import ORDER
import run_b_to_entry_boundary_row_ablation04 as historical


def inputs(cfg):
    path=ROOT/cfg['historical_result']
    assert sha(path)==cfg['historical_result_sha256']
    assert path.read_bytes()==subprocess.check_output(['git','show',cfg['historical_result_commit']+':'+cfg['historical_result']],cwd=ROOT)
    result=read(path);run=Path(result['run']);historical.verify(run)
    assert sha(run/'result_hashes.json')==result['result_hashes_sha256']
    for rel,h in read(run/'result_hashes.json').items():assert sha(run/rel)==h,rel
    for rel,h in result['table_hashes'].items():assert sha(path.parent/rel)==h,rel
    spec=next(s for s in result['summary']['selected_sources'] if s['id']==cfg['source_id'])
    assert spec['label']==cfg['source_label']=='S3' and cfg['order']==ORDER
    folder=Path(spec['folder']);manifest=read(folder/'source_manifest.json')
    for p,h in manifest['hashes'].items():assert sha(p)==h,p
    copies=read(folder/'reuse.json')['copies']
    for name,row in copies.items():assert sha(folder/name)==sha(row['path'])==row['sha256']
    c=read(folder/'common_state.json');context=read(folder/'source_audit.json')['context']
    assert c['B']==context['B'] and c['fresh_capture_pose']==context['R_obs']
    assert c['B_tick']==context['switch_state_id']==cfg['checks_only']['B_tick']
    assert c['next_submit_after_B']==cfg['checks_only']['first_submit_tick']
    P=np.asarray(context['P']);past=np.load(folder/'recorded_old_to_B.npy')
    np.testing.assert_array_equal(past[-2],P);np.testing.assert_array_equal(past[-1],c['B'])
    refs=read(folder/'references.json');rawref=refs['M0_NATIVE']
    for frame in ['local','world']:assert sha(rawref[frame+'_path'])==rawref[frame+'_sha256']
    reuse=read(folder/'reuse.json')['methods']['M0_NATIVE'];method=Path(reuse['folder'])
    assert sha(method/'hashes.json')==reuse['hashes_sha256']
    for rel,h in read(method/'hashes.json').items():assert sha(method/rel)==h,rel
    native=read(method/'rollout.json');installed=np.array(read(method/'restoration.json')['installed_world'])
    F=np.load(rawref['world_path']);raw=np.load(rawref['local_path'])
    assert F.tobytes()==local_trajectory_to_world(c['fresh_capture_pose'],raw).tobytes()
    np.testing.assert_allclose(installed,F,rtol=0,atol=1e-12)
    assert native['phase']==c and native['termination']=='OBSERVATION_CAP' and len(native['commands'])==180
    assert rawref['local_sha256']==manifest['FRESH_sha256'] and rawref['world_sha256']==manifest['FRESH_world_sha256']
    entry=read(folder/'entry.json');S,Sl,parity=suffix_reference(installed,raw,c['fresh_capture_pose'],c['B'],entry['correspondence'])
    assert all(entry[k]==v for k,v in parity.items())
    saved=np.load(folder/'suffix_native_installed.npy')
    assert saved[1:].tobytes()==S[1:].tobytes();np.testing.assert_allclose(saved[0],S[0],atol=1e-12,rtol=0)
    for key,target in [('arc_m','entry_arc_m'),('normalized_progress','entry_progress'),('segment','entry_segment'),('alpha','entry_alpha')]:
        assert entry['correspondence'][key]==cfg['checks_only'][target]
    w,l,labels=staged_reference(installed,raw,c['fresh_capture_pose'],c['B'],entry,S)
    bentry=refs['B_ENTRY_STAGE']
    assert w.tobytes()==np.load(bentry['world_path']).tobytes() and l.tobytes()==np.load(bentry['local_path']).tobytes()
    schedule=read(folder/'schedule.json');assert schedule['B_tick']==c['B_tick']
    assert schedule['integration_steps']==cfg['checks_only']['integration_steps']==180
    assert schedule['integration_dt_s']==c['integration_dt_s']
    assert len(schedule['pairs'])==native['new_MPC_solved']==30
    assert [r['tick'] for r in native['submit_requests']]==schedule['full']['attempted_submit_ticks']
    scenario=read(folder/'scenario.json');assert scenario['cart_present']==cfg['checks_only']['cart_present']==False
    source=Path(manifest['source']);config_path=source/'config_snapshot.yaml';config=yaml.safe_load(config_path.read_text())
    assert manifest['hashes'][str(config_path)]==sha(config_path)
    episode=source/'episodes'/cfg['source_id'].split('/')[0];meta=read(episode/'metadata.json')
    mpc=manifest['mpc'];assert mpc==meta['mpc_worker']['provenance']
    assert mpc['effective_linear_velocity_limit_m_s']==mpc['official_settings']['OBJNAV_V_MAX']==cfg['checks_only']['effective_speed_mps']
    envpath=Path(manifest['environment_export']);scene=read(envpath/'scene_provenance.json')
    assert all(scene['scene'][k]==v for k,v in config['scene'].items())
    assert all(meta['scene'][k]==v for k,v in config['scene'].items())
    # Preserve exact historical schedule bytes; this derived view supplies harness keys only.
    s=dict(start_tick=c['B_tick'],stop_tick=c['B_tick']+schedule['integration_steps'],integration_steps=schedule['integration_steps'],
        pairs=schedule['pairs'],attempted_submit_ticks=schedule['full']['attempted_submit_ticks'],
        accepted_submit_ticks=schedule['full']['accepted_submit_ticks'],application_ticks=[r['application_tick'] for r in schedule['full']['applications']],
        original_generation=c['original_generation'],source_path=str(folder/'schedule.json'),source_sha256=sha(folder/'schedule.json'))
    chains={}
    for name in ['spatial_entry_suffix_execution_01','b_to_entry_bridge_01','b_to_entry_vector_bridge_execution_03','b_to_entry_boundary_row_ablation_04']:
        p=ROOT/'results'/name/'result_summary.json';r=read(p)
        chains[name]=dict(path=str(p),sha256=sha(p),run=r['run'],scientific_freeze_sha=r['summary']['scientific_freeze_sha'],result_hashes_sha256=r['result_hashes_sha256'])
    auth=dict(valid=True,source_id=cfg['source_id'],historical_result_commit=cfg['historical_result_commit'],historical_chain=chains,
        source_manifest=manifest,source_manifest_path=str(folder/'source_manifest.json'),source_manifest_sha256=sha(folder/'source_manifest.json'),
        historical_native_folder=str(method),historical_native_rollout_sha256=sha(method/'rollout.json'),
        historical_folder=str(folder),source_config_path=str(config_path),source_config_sha256=sha(config_path),
        scene_provenance_path=str(envpath/'scene_provenance.json'),scene_provenance_sha256=sha(envpath/'scene_provenance.json'),
        P=P.tolist(),A=c['fresh_capture_pose'],B=c['B'],clocks={k:context[k] for k in ['t_obs','t_request','t_ready_host','t_ready_seen_sim','t_install','t_switch']},
        C3_exact=True,B_ENTRY_exact=True,raw_A_anchored_bit_exact=True,cart_present=False,
        historical_native_installation_roundoff_max=float(abs(installed-F).max()),
        zero_scientific_calls=True)
    return dict(auth=auth,folder=folder,common=c,schedule=s,historical_schedule=schedule,historical=native,
        original=F,native_installed=installed,raw=raw,suffix=S,suffix_local=Sl,entry=entry,P=P,config_path=config_path,
        references={'RAW':(F,raw,[f'F_{i}' for i in range(len(F))]),'B_ENTRY':(w,l,labels)},mpc=mpc)
