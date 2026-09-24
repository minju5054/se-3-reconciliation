#!/usr/bin/env python3
"""Saved-only Acquisition 02 validation; no model/controller calls."""
import argparse
from pathlib import Path
import sys
import numpy as np
from shapely.geometry import LineString,Point
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.blind_corner_source import ray_first_hit,project_camera,turn_descriptor,select_representative
from reconciliation.blind_corner_source02 import attributed_clearance,historical_scale
from run_blind_corner_source02 import audit,verify
from run_join_online02 import environments
from validate_blind_corner_source import episode,bundle
from validate_robotless_online_handoffs import equal_record
from analyze_join_online02 import jsonlines


def validate(run):
    verify(run)
    g=audit(run);equal_record(g,read(run/'geometry_qualification.json'),'geometry')
    declaration=read(run/'declaration.json');cfg=declaration['config']
    equal_record(historical_scale(ROOT/cfg['historical_corpus']),read(run/'historical_scale.json'),'historical spatial scale')
    order=read(run/'protocol.json')['eligible_order'];assert order==g['eligible_order']
    full=np.load(ROOT/cfg['environment']/'geometry/world_triangles.npz');meshes=read(ROOT/cfg['environment']/'geometry/meshes.json')
    rows=[]
    for cid in order:
        r=episode(run,cid);ep=run/'candidates'/cid/'episodes'/cid
        c=next(x['candidate'] for x in g['rows'] if x['candidate_id']==cid)
        base,on,cart,_=environments(run/'candidates'/cid)
        parts=read(ROOT/cfg['environment']/'geometry/projected/obstacle_parts.json')
        attributed={}
        for label in ['OLD','FRESH']:
            rec=r[label]
            if not rec or 'world' not in rec:attributed[label]=None;continue
            w=np.asarray(rec['world']);v=attributed_clearance(w,base,cart)
            path=LineString(w[:,:2]) if len(w)>1 else Point(w[0,:2])
            part=int(base.tree.nearest(path));v['nearest_Hospital_mesh']=meshes[int(parts[part]['mesh_id'])]['prim_path']
            v['first_margin_invalid_segment']=next((i for i in range(len(w)-1) if not on.check_polyline(w[i:i+2])['clearance_valid']),None)
            v['limiting_prim']=v['nearest_Hospital_mesh'] if v['limiting_geometry']=='Hospital' else c['id']+'/runtime_supply_cart'
            np.testing.assert_allclose(v['combined_m'],rec['geometry_on']['minimum_clearance_m'],rtol=0,atol=1e-12)
            attributed[label]=v
        old=r['OLD'];hit=screen=None;positive=False
        if old and 'world' in old:
            cap=next(x for x in jsonlines(ep/'capture.jsonl') if x['frame_id']==old['observation']['frame_id'])
            tri=np.load(run/'technical_preflight'/cid/'triangles.npz')['triangles']
            target=(tri.min(axis=(0,1))+tri.max(axis=(0,1)))/2;origin=np.array(cap['camera']['T_world_camera'])[:3,3]
            hit=ray_first_hit(origin,target,full['triangles']);screen=project_camera(target,cap['camera'])
            if hit:hit['mesh']=meshes[int(full['mesh_ids'][hit['triangle_index']])]['prim_path']
            desc=turn_descriptor(old['raw_local'],old['raw_local'])['OLD']
            yaw=np.unwrap(np.asarray(old['raw_local'])[:,2]);positive=bool(desc['net_tangent_turn_rad'] is not None and desc['net_tangent_turn_rad']>0 and yaw[-1]-yaw[0]>0)
        flags=r['gates'];flags.update(geometry_preflight_pass=True,OLD_positive_left=positive,
            OLD_actual_wall_occlusion=bool(hit and screen['in_frustum'] and any(hit['mesh'].startswith(p) for p in c['wall_prefixes'])),
            OLD_cart_attribution=bool(attributed['OLD'] and attributed['OLD']['Hospital_check']['clearance_valid'] and attributed['OLD']['cart_only_m']<.05))
        r.update(gates=flags,qualified=all(flags.values()),failure_reasons=[k for k,v in flags.items() if not v],clearance_attribution=attributed,OLD_occlusion=dict(wall_ray=hit,screen=screen))
        events=jsonlines(ep/'controller/events.jsonl')
        # Full first-FRESH solve input/output and command source lineage, not a synthetic controller state.
        first_solve=next((e for e in events if e.get('type')=='solve_result' and e.get('chunk_id')=='chunk_001'),None)
        r['first_FRESH_solve_result']=first_solve
        r['controller_events_sha256']=sha(ep/'controller/events.jsonl')
        rows.append(r)
    sessions=[read(run/'candidates'/cid/'episodes'/cid/'session_open.json')['connection_id'] for cid in order]
    assert len(set(sessions))==len(order)
    assert sorted(p.name for p in (run/'candidates').iterdir())==sorted(order) if order else True
    representative=select_representative(rows,order)
    technical=any(read(run/'candidates'/cid/'completion.json')['status']=='TECHNICAL_RUNTIME_FAILURE' for cid in order)
    classification=('TECHNICAL_EXECUTION_BLOCKED' if technical else 'QUALIFIED_BLIND_CORNER_OBSTACLE_HANDOFF_SOURCE' if representative else
                    'BLIND_CORNER_02_LIGHTNAV_SOURCE_NOT_QUALIFIED' if order else 'BLIND_CORNER_02_GEOMETRY_UNAVAILABLE')
    return dict(valid=True,geometry=g,rows=rows,representative=representative,classification=classification,
        validation_model_MPC_optimizer_calls=0,initial_sha=read(run/'source.json')['starting_sha'],
        scientific_sha=read(run/'candidates'/order[0]/'execution_start.json')['sha'] if order else None)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    r=validate(a.run.resolve());save(a.run/'validation.json',r);bundle(a.run.resolve(),r)
    print({k:r[k] for k in ['valid','representative','classification']})
