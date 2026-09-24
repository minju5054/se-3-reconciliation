#!/usr/bin/env python3
"""Model-free geometry audit and immutable source-acquisition freeze."""
import argparse
from copy import deepcopy
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha,mpc_audit
from reconciliation.blind_corner_source import INSTRUCTION,ray_first_hit,project_camera,left_corner
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source02_geometry import project_prop,compose_environment
from run_join_source05 import verify,start,git
CONFIG=ROOT/'configs/blind_corner_source_acquisition_01.yaml'
PREFLIGHT=ROOT/'data/blind_corner_source_acquisition_01/preflight_20260924T000000Z/technical_preflight'


def audit_geometry():
    cfg=yaml.safe_load(CONFIG.read_text());base=HospitalEnvironment.load(ROOT/cfg['environment'])
    full=np.load(ROOT/cfg['environment']/'geometry/world_triangles.npz');meshes=read(ROOT/cfg['environment']/'geometry/meshes.json')
    expected={r['identifier']:r['composed_layer_text_sha256'] for r in read(ROOT/cfg['environment']/'scene_provenance.json')['used_layers'] if not r['anonymous']}
    actual=read(PREFLIGHT/'scene.json')['static_layers'];assert expected==actual,'Hospital provenance mismatch'
    out=[]
    for c in cfg['candidates']:
        f=PREFLIGHT/c['id'];m=read(f/'manifest.json');assert c==m['candidate']
        tri=np.load(f/'triangles.npz')['triangles'];proj=project_prop(tri);on=compose_environment(base,proj,True)
        records=[read(f/f'probe_{i:02}.json') for i in range(len(c['probe_poses']))]
        target=(tri.min(axis=(0,1))+tri.max(axis=(0,1)))/2
        cam=records[0]['camera'];origin=np.array(cam['T_world_camera'])[:3,3]
        hit=ray_first_hit(origin,target,full['triangles']);screen=project_camera(target,cam)
        if hit:hit['mesh']=meshes[int(full['mesh_ids'][hit['triangle_index']])]['prim_path']
        poses=[on.check_polyline([r['agent_pose_world'][:2]]) for r in records]
        passage=[on.check_polyline(p) for p in c['passage_probes']]
        first=next((i for i,r in enumerate(records) if r['instance']['visible_pixels']>=20),None)
        overlap=float(proj['obstacle_geometry'].intersection(base.obstacles).area)
        flags=dict(left_corner=left_corner(c['incoming_xy'],c['outgoing_xy']),
            initial_hidden=records[0]['instance']['visible_pixels']<20,
            later_visible=first is not None and first>0,
            wall_occlusion=bool(hit and any(hit['mesh'].startswith(x) for x in c['wall_prefixes']) and screen['in_frustum']),
            initial_clearance=poses[0]['clearance_valid'],
            visible_probe_safe=bool(first is not None and poses[first]['clearance_valid'] and poses[first]['minimum_clearance_m']>=.30),
            passage=any(p['clearance_valid'] for p in passage),
            cart_no_existing_overlap=overlap<=1e-6,
            synchronized=all(r['same_render_product'] and r['stable_camera_pose_and_simulation_time'] for r in records))
        out.append(dict(candidate_id=c['id'],qualified=all(flags.values()),gates=flags,failure_reasons=[k for k,v in flags.items() if not v],
            pixels=[r['instance']['visible_pixels'] for r in records],probe_clearance=poses,passage_checks=passage,
            cart_existing_overlap_area_m2=overlap,wall_ray=hit,cart_center_screen=screen,
            projection=proj['metadata'],prop=read(f/'cart.json'),candidate=c,preflight=str(f)))
    return dict(valid=True,rows=out,eligible=[r['candidate_id'] for r in out if r['qualified']],
        scientific_model_calls=0,MPC_calls=0,scope='MODEL-FREE DIAGNOSTIC POSES; NOT EXECUTION')


def prepare(run):
    cfg=yaml.safe_load(CONFIG.read_text());assert cfg['instruction']==INSTRUCTION
    result=audit_geometry();run.mkdir(parents=True,exist_ok=False);(run/'logs').mkdir()
    save(run/'geometry_qualification.json',result)
    base=yaml.safe_load((ROOT/cfg['source_config']).read_text());base['experiment']=cfg['experiment']
    base['online'].update(minimum_active_before_prediction_sim_s=0.,maximum_active_sim_s=8.,postroll_sim_s=.10,maximum_handoff_attempts=1)
    save(run/'protocol.json',dict(declaration=cfg,eligible_order=result['eligible'],instruction=INSTRUCTION,
        selection_rule=cfg['selection'],maximum_terminal_predictions=2*len(result['eligible']),
        first_crossing_only=True,no_later_frame_substitution=True,cart_constant=True,coordinates='world=T_world_observation*T_local; raw rows no intrinsic timestamps',
        scope='SOURCE ACQUISITION ONLY; no optimization/continuation/trackability',geometry_freeze=sha(CONFIG)))
    for row in result['rows']:
        if not row['qualified']:continue
        c=row['candidate'];folder=run/'candidates'/c['id'];folder.mkdir(parents=True);(folder/'logs').mkdir()
        cc=deepcopy(base);cc['lighting']['translation_world_m']=c['fill_translation'];cc['join_online02']['instruction']=INSTRUCTION
        (folder/'config_snapshot.yaml').write_text(yaml.safe_dump(cc,sort_keys=False))
        inp=dict(environment_export=str((ROOT/cfg['environment']).resolve()),projection=row['projection'])
        save(folder/'geometry_input.json',inp)
        tri=np.load(Path(row['preflight'])/'triangles.npz')['triangles'];forward=np.array(c['outgoing_xy']);d=(tri[:,:,:2]-c['cart_center_xy'])@forward
        scenario=dict(source03_input=str(folder/'geometry_input.json'),center_xy=c['cart_center_xy'],forward_xy=c['outgoing_xy'],
            prop=row['prop'],triangles_path=str(Path(row['preflight'])/'triangles.npz'),cart_extents=[float(d.min()),float(d.max())])
        save(folder/'scenario.json',scenario);save(folder/'episode_schedule.json',dict(episodes=[dict(episode_id=c['id'],R0=c['approach_pose'],instruction=INSTRUCTION,cart_present_initial=True,cart_present_dynamic=False)]))
    (run/'config_snapshot.yaml').write_text(yaml.safe_dump(base,sort_keys=False))
    preserved={str(p.resolve()):sha(p) for p in PREFLIGHT.rglob('*') if p.is_file()}
    osa=ROOT/'data/obstacle_source_acquisition_03/primary_20260923T085200Z'
    preserved.update(read(osa/'source.json')['preserved'])
    preserved.update({str(p.resolve()):sha(p) for p in osa.rglob('*') if p.is_file()})
    save(run/'source.json',dict(starting_sha=git('rev-parse','HEAD'),origin_main=git('rev-parse','origin/main'),
        source04_run=read(osa/'source.json')['source04_run'],preserved=preserved,preflight=str(PREFLIGHT)))
    save(run/'mpc_audit.json',mpc_audit(ROOT));assert read(run/'mpc_audit.json')['status']=='MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL'
    print({k:result[k] for k in ['eligible','valid']})


def freeze(run):
    inherited=read(ROOT/'data/obstacle_source_acquisition_03/primary_20260923T085200Z/freeze.json')['source_sha256']
    for p,h in inherited.items():assert sha(ROOT/p)==h,p
    files=['configs/blind_corner_source_acquisition_01.yaml','src/reconciliation/blind_corner_source.py',
        'scripts/run_blind_corner_source.py','scripts/isaac/blind_corner_preflight.py','scripts/isaac/blind_corner_online.py',
        'scripts/validate_blind_corner_source.py','scripts/report_blind_corner_source.py','tests/test_blind_corner_source.py']
    save(run/'freeze.json',dict(source_sha256={**inherited,**{p:sha(ROOT/p) for p in files}},
        input_sha256={str(p.resolve()):sha(p) for p in run.rglob('*') if p.is_file()},
        geometry_candidates=read(run/'protocol.json')['eligible_order']))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--mode',choices=['prepare','freeze','verify','start','stop','geometry'],required=True);a=p.parse_args();run=a.run.resolve()
    if a.mode=='verify':verify(run,True)
    elif a.mode=='geometry':print(__import__('json').dumps(audit_geometry(),indent=2))
    elif a.mode=='stop':subprocess.run([sys.executable,str(ROOT/'scripts/lightnav/robotless_online_server.py'),'stop',str(run)],check=True)
    else:dict(prepare=prepare,freeze=freeze,start=start)[a.mode](run)
