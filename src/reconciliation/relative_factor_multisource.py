"""Saved-source authentication and descriptive multisource analysis; no controller."""
from pathlib import Path
from copy import deepcopy
import csv
import hashlib
import json
import numpy as np
from .join_source03 import read, save, sha
from .se2 import local_trajectory_to_world, relative_pose, wrap_angle
from .osa03_relative_ablation import ORDER, planning
from .osa03_relative_replication import EFFECT_KEYS
from .osa03_common_b import schedule
from .gp_se2_environment import HospitalEnvironment
from .handoff_delay_attribution import boundary_state
from .osa03_native import pose, command
from .online_mpc_adapter import PINNED_MPC_SHA256, selection_audit
ROOT=Path(__file__).resolve().parents[2]
REQUIRED_PNG=['world_execution_overview.png','benchmark_primary_metrics.png',
              'no_relative_minus_full_deltas.png','reference_deformation_overview.png']
SELECTOR_PNG='selector_diagnostic_summary.png'


def value_hash(raw):
    return hashlib.sha256(np.asarray(raw,dtype='<f8').tobytes()).hexdigest()


def unique_sources(rows):
    if not 3<=len(rows)<=6:raise ValueError('source shortage or excessive benchmark size')
    for key in ['raw_sha256','raw_value_sha256']:
        if len({r[key] for r in rows})!=len(rows):raise ValueError('duplicate raw local FRESH geometry')
    if sum(r['kind']=='osa' for r in rows)>1:raise ValueError('only one OSA03 representative')
    return True


def candidate_inventory(cfg):
    rows=[]
    for r in read(ROOT/cfg['scan_run']/'ledger.json'):
        if not r['flags']['candidate']:continue
        p=r['source_paths']['fresh_raw_local'];a=np.load(p);d=np.diff(a[:,:2],axis=0)
        rows.append(dict(id=r['case_id'],raw_sha256=sha(p),raw_value_sha256=value_hash(a),
            source_qualified=True,arc_m=float(np.linalg.norm(d,axis=1).sum()),
            first_last_local=a[[0,-1]].tolist(),yaw_range_deg=float(np.rad2deg(np.ptp(np.unwrap(a[:,2])))),
            chord_direction_range_deg=float(np.rad2deg(np.ptp(np.unwrap(np.arctan2(d[:,1],d[:,0]))))),
            selected=any(s['id']==r['case_id'] for s in cfg['selected'])))
    return rows


def periodic_schedule(common, pair, provenance, steps=180):
    """Freeze the source grid and its first observed completion lag, not wall time."""
    b=common['B_tick'];first=pair['submit_tick'];lag=pair['application_tick']-first
    if first!=common['next_submit_after_B'] or first%6 or not b<=first<b+6 or not 0<lag<6:
        raise ValueError('source schedule incompatible with one outstanding result')
    pairs=[dict(submit_tick=t,application_tick=t+lag) for t in range(first,b+steps,6)]
    def window(n):
        end=b+n
        return dict(attempted_submit_ticks=[p['submit_tick'] for p in pairs if p['submit_tick']<=end],
            accepted_submit_ticks=[p['submit_tick'] for p in pairs if p['submit_tick']<=end],
            applications=[p for p in pairs if p['application_tick']<=end and p['application_tick']<b+steps],
            complete_primary_exposure=True,endpoint_inclusive_absolute_tick=end)
    return dict(pairs=pairs,primary=window(54),full=window(steps),B_tick=b,
        integration_steps=steps,integration_dt_s=common['integration_dt_s'],provenance=provenance,
        rule='repeat first saved post-cut application lag on original absolute 6-tick grid',release_lag_ticks=lag)


def geometry(folder):
    m=read(Path(folder)/'source_manifest.json')
    if m['kind']=='osa':
        from run_osa03_common_b import geometry as osa_geometry
        return osa_geometry(Path(m['source']))
    base=HospitalEnvironment.load(m['environment_export'])
    return dict(base=base,on=base,cart=None,env_source={'environment_export':m['environment_export']})


def reference_safety(world,env):
    if env['cart'] is not None:
        from run_local_se2_reconciliation_formulation01 import independent_clearance
        return independent_clearance(world,env)
    from shapely.geometry import LineString
    from shapely.ops import nearest_points
    on=env['on'];xy=np.asarray(world)[:,:2];path=LineString(xy)
    clear=float(path.distance(on.obstacles)-.2);eps=on.numerical_tolerance_m
    known=bool(on.workspace.covers(path) and path.distance(on.workspace.boundary)>=.2+eps)
    native=on.check_polyline(world)
    np.testing.assert_allclose(clear,native['minimum_clearance_m'],rtol=0,atol=eps)
    assert native['clearance_valid']==(known and clear>=.05+eps)
    parts=[LineString(xy[i:i+2]) for i in range(len(xy)-1)]
    ds=[float(p.distance(on.obstacles)-.2) for p in parts];j=int(np.argmin(ds));q,o=nearest_points(parts[j],on.obstacles)
    return dict(minimum_clearance_m=clear,native_clearance_m=native['minimum_clearance_m'],
        union_minus_parts_m=clear-native['minimum_clearance_m'],limiting_geometry='Hospital',cart_present=False,
        first_minimum_segment=j,minimum_path_xy=[q.x,q.y],nearest_obstacle_xy=[o.x,o.y],
        per_segment_clearance_m=ds,workspace_known=known,clearance_valid=native['clearance_valid'],
        required_edge_clearance_m=.05,footprint_radius_m=.2,numerical_tolerance_m=eps,
        query='full union direct GEOS; no cart in this source; no B connector')


def reference_record(folder,name,world,raw,c,fresh,env):
    from run_osa03_native_continuation import plain
    local=raw.copy() if name=='M0_NATIVE' else relative_pose(c['fresh_capture_pose'],world)
    np.testing.assert_allclose(local_trajectory_to_world(c['fresh_capture_pose'],local),world,atol=1e-12,rtol=0)
    record={}
    for frame,a in [('world',world),('local',local)]:
        p=folder/'references'/f'{name}_{frame}.npy'
        with p.open('xb') as f:np.save(f,a,allow_pickle=False)
        record.update({frame+'_path':str(p),frame+'_sha256':sha(p)})
    safety=reference_safety(world,env)
    record.update(safety=safety,descriptor=plain(planning(fresh,world,c['B'])),
                  status='REFERENCE_GEOMETRY_SAFE' if safety['clearance_valid'] else 'REFERENCE_GEOMETRY_UNSAFE')
    return record


def load_source(cfg,spec):
    """Read original state, actual poll memory and first FRESH command. Zero solves."""
    from run_osa03_native_continuation import source_phase
    from reconciliation.osa03_common_b import common_state
    hashes={}
    if spec['kind']=='osa':
        import run_osa03_relative_factor_ablation01 as prior
        old,manifest,sched=prior.authenticate(prior.yaml.safe_load(prior.CONFIG.read_text()))
        source=Path(manifest['source']);phase,fresh,raw,mpc=source_phase(source);common=common_state(phase)
        assert common==read(old/'common_state.json')
        hashes.update(manifest['hashes'])
        state=read(source/'source_bundle/REPEAT_00/state_and_timing.json');c=state['context']
        ep=source/'episodes/REPEAT_00';rows=list(csv.DictReader((ep/'execution.csv').open()))
        past=[pose(r).tolist() for r in rows if c['obs_state_id']<=int(r['state_id'])<=c['switch_state_id']]
        scene=read(source/'scenario.json');audit=dict(context=c,boundary=state['B_state'],raw_value_sha256=value_hash(raw))
        manifest=dict(manifest,kind='osa',hashes=hashes)
        return dict(common=common,fresh=fresh,raw=raw,manifest=manifest,schedule=sched,scene=scene,past=past,audit=audit)
    scan=ROOT/cfg['scan_run'];inventory=read(scan/'source.json');anchored=inventory['source_hashes']
    for p,h in cfg['inventory_hashes'].items():assert sha(ROOT/p)==h
    row=next(r for r in read(scan/'ledger.json') if r['case_id']==spec['id']);assert row['flags']['candidate']
    cp=Path(row['source_paths']['context']);ep=cp.parents[2];c=read(cp)
    def auth(p,required=True):
        p=Path(p);h=sha(p)
        if required:assert anchored[str(p)]==h,p
        hashes[str(p)]=h
    for p in row['source_paths'].values():auth(p)
    for name in ['execution.csv','commands.csv','controller/events.jsonl']:auth(ep/name)
    auth(ep/'metadata.json',False)
    rows=list(csv.DictReader((ep/'execution.csv').open()));commands=list(csv.DictReader((ep/'commands.csv').open()))
    events=[json.loads(l) for l in (ep/'controller/events.jsonl').read_text().splitlines()]
    meta=read(ep/'metadata.json');mpc=meta['mpc_worker']['provenance'];dt=meta['resolved_integration_dt_s']
    assert mpc['mpc_source_sha256']==PINNED_MPC_SHA256==sha(mpc['mpc_source']);auth(mpc['mpc_source'],False)
    settings=meta['execution'];assert settings['integration_hz']/settings['control_hz']==6
    assert settings['controller_dt_s']==.1 and settings['command_hold_timeout_s']==.5
    assert dt==float(np.float32(1/60))
    assert all(abs(float(y['sim_time_s'])-float(x['sim_time_s'])-dt)<1e-12 for x,y in zip(rows,rows[1:]))
    fresh=np.load(row['source_paths']['fresh_world']);raw=np.load(row['source_paths']['fresh_raw_local'])
    np.testing.assert_allclose(local_trajectory_to_world(c['R_obs'],raw),fresh,rtol=0,atol=1e-12)
    boundary=boundary_state(c,rows,commands,events,'DELAYED');assert boundary['available']
    b=c['switch_state_id'];first=c['first_fresh_solve'];sid=first['solve_id'];saved=next(e for e in events if e.get('type')=='solve_result' and e['solve_id']==sid)
    assert all(first[k]==v for k,v in saved.items())
    assert first['status']=='command' and first['official_generation']==first['result_generation']
    assert first['input_state_id']<b and first['input_state_id']%6==0
    assert first['chunk_id']==c['fresh_chunk_id'] and first['reference_version']==c['fresh_reference_version']
    np.testing.assert_array_equal(first['input_pose'],pose(rows[first['input_state_id']]))
    assert selection_audit(fresh,first['input_pose'],first['selection']['reference_world'],horizon=mpc['official_settings']['HORIZON'],weights=mpc['official_settings']['Q_WEIGHTS'])==first['selection']
    assert int(commands[b]['application_state_id'])==b and commands[b]['solve_id']==sid and commands[b]['reason']=='new_solve'
    assert first['seen_sim_time_s']==float(rows[b]['sim_time_s'])
    assert command(commands[b])==first['command']==boundary['memory']['previous_control']
    cut=c['t_switch']['host_monotonic_s']
    pending=[e['solve_id'] for e in events if e.get('status')=='submitted' and e['submit_host_monotonic_s']<=cut and not any(
        x.get('type')=='solve_result' and x.get('solve_id')==e['solve_id'] and x.get('seen_in_isaac',{}).get('host_monotonic_s',float('inf'))<=cut for x in events)]
    assert not pending,'cannot discard a pending official solve at B'
    nxt=next(e for e in events if e.get('status')=='submitted' and e['input_state_id']>=b)
    assert nxt['submit_host_monotonic_s']>=cut and nxt['chunk_id']==c['fresh_chunk_id']
    assert nxt['input_state_id']==((b+5)//6)*6
    app=next(r for r in commands if r['solve_id']==nxt['solve_id'] and r['reason']=='new_solve')
    result=next(e for e in events if e.get('type')=='solve_result' and e.get('solve_id')==nxt['solve_id'])
    assert result['status']=='command' and command(app)==result['command']
    assert result['seen_in_isaac']['sim_time_s']==float(app['sim_time_s'])
    assert nxt['previous_command']==first['command']==result['previous_command']
    prior=[r for r in commands[:b] if r['reason']=='new_solve'][-1]
    common=dict(B=c['B'],B_tick=b,B_sim_s=float(rows[b]['sim_time_s']),u_minus=boundary['u_minus'],u_B_plus=first['command'],
        u_mem_B=boundary['memory']['previous_control'],delta_u_B=(np.array(first['command'])-boundary['u_minus']).tolist(),
        previous_command_application_sim_s=float(prior['sim_time_s']),fresh_capture_pose=c['R_obs'],fresh_chunk_id=c['fresh_chunk_id'],
        fresh_version=c['fresh_reference_version'],original_generation=first['official_generation'],integration_dt_s=dt,
        next_submit_after_B=nxt['input_state_id'],first_FRESH_solve=first)
    pair=dict(submit_tick=nxt['input_state_id'],application_tick=int(app['application_tick']))
    provenance=dict(context_path=str(cp),context_sha256=sha(cp),commands_sha256=sha(ep/'commands.csv'),
        events_sha256=sha(ep/'controller/events.jsonl'),metadata_sha256=sha(ep/'metadata.json'),first_pair=pair,first_result_id=nxt['solve_id'])
    sched=periodic_schedule(common,pair,provenance)
    envpath=Path(inventory['environment_path'])
    for p in envpath.rglob('*'):
        if p.is_file():auth(p,False)
    for p in cfg['inventory_hashes']:auth(ROOT/p,False)
    source=ep.parents[1];auth(source/'config_snapshot.yaml',False)
    manifest=dict(kind='genuine',source=str(source),source_id=spec['id'],environment_export=str(envpath),mpc=mpc,
        FRESH_sha256=sha(row['source_paths']['fresh_raw_local']),FRESH_world_sha256=sha(row['source_paths']['fresh_world']),
        OLD_sha256=sha(row['source_paths']['old_raw_local']),hashes=hashes)
    scene=dict(center_xy=c['B'][:2],forward_xy=[float(np.cos(c['B'][2])),float(np.sin(c['B'][2]))],
        cart_present=False,axis_semantics='legacy evaluator cart_* fields are B-heading coordinates only; no cart exists')
    past=[pose(r).tolist() for r in rows if c['obs_state_id']<=int(r['state_id'])<=b]
    audit=dict(context=c,boundary=boundary,pending_at_B=pending,raw_value_sha256=value_hash(raw),
        source_row=row,first_following_submit=nxt,first_following_application=app,
        frame='world XY metres/yaw rad CCW; raw observation-local x-forward y-left; F=A raw, derived local=A^-1 X')
    return dict(common=common,fresh=fresh,raw=raw,manifest=manifest,schedule=sched,scene=scene,past=past,audit=audit)


def cross_source(summaries,tolerance=1e-9):
    deltas={};signs=[];support=[];opposing=[];safe=[];edge=[];attach=[];early=[]
    for sid,s in summaries.items():
        f=s['primary_metrics']['FULL_LOCAL_SE2'];n=s['primary_metrics']['NO_RELATIVE'];d={}
        for k in EFFECT_KEYS:
            a,b=(s['planning'].get('NO_RELATIVE'),s['planning'].get('FULL_LOCAL_SE2')) if k.startswith('relative_') else (n,f)
            d[k]=None if not s['schedule_gate']['comparable'] or a is None or b is None or a.get(k) is None or b.get(k) is None else a[k]-b[k]
        deltas[sid]=d
        primary=['max_position_error_05_m','position_auc_03_m_s','position_auc_09_m_s']
        early.append(all(d[k] is not None and d[k]<=tolerance for k in primary))
        attach.append(d['sustained_attachment_s'] is not None and d['sustained_attachment_s']<=tolerance)
        edge.append(all(d[k] is not None and d[k]>tolerance for k in EFFECT_KEYS[-2:]))
        safe.append(all(s['reference_safety'][m]['clearance_valid'] and s['primary_metrics'][m] is not None and
            s['primary_metrics'][m]['execution_clearance_lower_bound_m']>=.05 and s['termination'][m]=='OBSERVATION_CAP' for m in ORDER[2:]))
        support.append(any(d[k] is not None and d[k]<-tolerance for k in primary+['sustained_attachment_s']))
        opposing.append(any(d[k] is not None and d[k]>tolerance for k in primary+['sustained_attachment_s']))
    expected={k:(1 if k in ['yaw_auc_03_rad_s','remaining_arc_at_attachment_m','execution_clearance_lower_bound_m','linear_command_TV']+EFFECT_KEYS[-2:] else -1) for k in EFFECT_KEYS}
    for k in EFFECT_KEYS:
        vals=[d[k] for d in deltas.values() if d[k] is not None]
        signs.append(dict(metric=k,negative=sum(v<-tolerance for v in vals),zero=sum(abs(v)<=tolerance for v in vals),
            positive=sum(v>tolerance for v in vals),unavailable=len(deltas)-len(vals),median=None if not vals else float(np.median(vals)),
            reversals_vs_R00_R01=[sid for sid,d in deltas.items() if d[k] is not None and d[k]*expected[k]<-tolerance]))
    count=len(summaries);criteria=dict(attachment_earlier_or_equal=sum(attach),all_early_position_nonworse=sum(early),
        safety_preserved=sum(safe),edge_RMS_and_max_larger=sum(edge),denominator=count)
    technical=any(not s['schedule_gate']['comparable'] or any(t in ['CONTROLLER_ERROR','HOLD_TIMEOUT'] for t in s['termination'].values()) for s in summaries.values())
    if technical or count<3:classification='TECHNICAL_BLOCKED'
    elif any(support) and any(opposing):classification='MIXED_EVIDENCE'
    elif all(n>count/2 for k,n in criteria.items() if k!='denominator'):classification='CONSISTENT_MULTISOURCE_PATTERN'
    elif any(support):classification='MIXED_EVIDENCE'
    else:classification='NO_SUPPORT_FOR_PATTERN'
    return dict(classification=classification,deltas=deltas,sign_summary=signs,criteria=criteria,sign_tolerance=tolerance)
