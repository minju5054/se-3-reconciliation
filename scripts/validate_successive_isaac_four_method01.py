#!/usr/bin/env python3
"""Recompute metrics from saved real USD traces; never launch or solve."""
import argparse
import csv
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from reconciliation.successive_isaac_four_method01 import ORDER,ProgressGraph,metrics,parity,fixed_references
from reconciliation.osa03_common_b import distortion
from reconciliation.se2 import wrap_angle,local_trajectory_to_world
from reconciliation.robotless_online import integrate_unicycle
from reconciliation.join_online02 import guard_check
from reconciliation.join_source02 import whole_raw_polyline_check
from reconciliation.online_mpc_adapter import selection_audit
from continuous_obstacle_reveal_exploratory_geometry02 import environments
from run_successive_isaac_four_method01 import verify,inputs,OUT,namespace
from run_osa03_native_continuation import plain
from validate_robotless_online_handoffs import equal_record


def validate(run):
    verify(run);cfg=read(run/'protocol.json');source,ep,auth,b,arr,c,s,hist=inputs(cfg)
    if (OUT/'raw_result_integrity.json').exists():
        seal=read(OUT/'raw_result_integrity.json')
        assert seal['run']==str(run) and seal['hash_manifest_sha256']==sha(run/'result_hashes.json')
        for relative,digest in read(run/'result_hashes.json').items():
            assert sha(run/relative)==digest,relative
    assert read(run/'common_state.json')==c and read(run/'historical_prefix.json')==hist and read(run/'schedule.json')==s
    refs0,suffix,entry,hermite,problem=fixed_references(arr['fresh_world'],arr['fresh_raw_local'],b['A'],b['B'],b['P'])
    assert entry==read(run/'entry.json') and hermite==read(run/'hermite.json')
    assert suffix.tobytes()==np.load(run/'suffix.npy').tobytes()
    refs=read(run/('references.json' if (run/'references.json').exists() else 'prepared_references.json'))
    env=environments(source)[1];geometry={};summaries={};rollouts={};selectors={}
    graph=read(run/'graph_planning.json');execution=read(run/'execution_summary.json')
    for name,ref in refs.items():
        for frame in ['local','world']:assert sha(ref[frame+'_path'])==ref[frame+'_sha256']
        w=np.load(ref['world_path']);local=np.load(ref['local_path'])
        safe=whole_raw_polyline_check(w,env);equal_record(safe,ref['safety'])
        np.testing.assert_allclose(local_trajectory_to_world(b['A'],local),w,rtol=0,atol=1e-12)
        if name in refs0:
            assert w.tobytes()==refs0[name][0].tobytes() and local.tobytes()==refs0[name][1].tobytes()
        seg=np.diff(w[:,:2],axis=0);length=np.linalg.norm(seg,axis=1)
        original_exact=[i for i,row in enumerate(arr['fresh_world']) if any(row.tobytes()==x.tobytes() for x in w)]
        phi=np.arctan2(seg[0,1],seg[0,0]);incoming=np.arctan2(*(np.asarray(b['B'])[:2]-b['P'][:2])[::-1])
        geometry[name]=dict(rows=len(w),XY_arc_m=float(length.sum()),B_first_gap_m=float(np.linalg.norm(w[0,:2]-np.asarray(b['B'])[:2])),
            incoming_first_segment_mismatch_rad=float(abs(wrap_angle(phi-incoming))),min_edge_m=float(length.min()),
            reference_clearance_m=safe['minimum_clearance_m'],physical_overlap=safe['physical_overlap'],
            legacy_5cm_margin_pass=safe['legacy_5cm_margin_pass'],valid=ref['valid'],bit_identical_original_rows=original_exact)
        if name=='GRAPH':
            interior=problem.pack(w);assert problem.noncollapsed(interior) and graph['solver']['converged']
            equal_record(problem.costs(interior),graph['final_costs'])
            geometry[name].update(deformation_against_original_suffix=distortion(suffix,w),
                factor_costs=problem.costs(interior),endpoint_bit_exact=w[-1].tobytes()==arr['fresh_world'][-1].tobytes())
    for name in ORDER:
        path=run/'methods'/name/'rollout.json'
        if not path.exists(): summaries[name]=None;continue
        r=read(path);rollouts[name]=r;assert r['phase']==c
        commands=r['commands'];states=r['states'];dt=c['integration_dt_s']
        assert len(states)==len(commands)+1 or r['error'] is not None
        if r['controller_restoration'] is not None:
            restore=r['controller_restoration'];assert restore['previous_control']==c['u_mem_B'] and restore['held_command']==c['u_B_plus']
            assert restore['generation']==c['original_generation'] and restore['restored_future_results']==restore['new_solve_calls']==0
            installed=np.asarray(restore['installed_world'])
            for e in r['collected_results']:
                assert e['official_generation']==e['result_generation']==c['original_generation']
                audit=selection_audit(installed,e['input_pose'],e['selection']['reference_world'],horizon=5,weights=[10,10,1])
                assert audit==e['selection']
            selectors[name]=[dict(submit_tick=e['input_state_id'],indices=e['selection']['indices'],
                labels=[refs[name]['labels'][i] for i in e['selection']['indices']]) for e in r['collected_results']]
            previous=c['u_mem_B']
            for e in r['collected_results']:
                np.testing.assert_array_equal(e['previous_command'],previous);previous=e['command']
        for i,cmd in enumerate(commands):
            st,after=states[i],states[i+1];tick=st['absolute_tick']
            assert tick==c['B_tick']+i and tick<s['stop_tick'] and cmd['application_tick']==tick
            u=[cmd['v_mps'],cmd['omega_radps']]
            np.testing.assert_allclose(integrate_unicycle(st['pose_world'],u,dt),after['pose_world'],rtol=0,atol=1e-10)
            g=guard_check(env,np.array(st['pose_world']),cmd,dt);assert g['safe']
            equal_record(g,r['guards'][i])
            assert after['Isaac']['sim_time_s']==after['sim_time_s']
            assert after['Isaac']['cart']['present'] and after['Isaac']['USD_readback_error']<=1e-10
        if r['abort'] is not None:
            assert r['abort']['command_applied'] is False
            assert not any(cmd['application_tick']==r['abort']['tick'] for cmd in commands)
        complete=len(commands)==s['integration_steps'] and r['error'] is None
        if complete:
            assert [q['tick'] for q in r['submit_requests']]==s['attempted_submit_ticks']
            assert [e['input_state_id'] for e in r['events'] if e.get('status')=='submitted']==s['accepted_submit_ticks']
            assert [q['application_tick'] for q in commands if q['reason']=='new_solve' and q['application_tick']>c['B_tick']]==s['application_ticks']
        assert all(w['sim_before']==w['sim_after'] and w['pose_before']==w['pose_after'] and w['Isaac_clock_and_pose_unchanged'] for w in r['waits'])
        summaries[name]=None if not states else metrics(r,arr['fresh_world'],env,s['integration_steps']*dt)
    gate=None
    if 'RAW' in rollouts and rollouts['RAW']['error'] is None:
        gate=parity(rollouts['RAW'],hist,s,cfg['parity']);assert gate==read(run/'RAW_parity.json')
    if any(n in rollouts for n in ORDER[1:]):assert gate is not None and gate['passed']
    assert list(rollouts)==ORDER[:len(rollouts)]
    calls=dict(graph_planning_solves=int((run/'graph_solve_start.json').exists()),
        SimulationApp_launches=int((run/'launch_attempt.json').exists()),
        method_rollouts={n:int(n in rollouts) for n in ORDER},
        MPC_solves=sum(len(r['collected_results']) for r in rollouts.values()),
        MPC_submissions=sum(sum(e.get('status')=='submitted' for e in r['events']) for r in rollouts.values()),
        LightNav=0,RGB_model_requests=0,source_acquisition=0,retries=0)
    assert calls['graph_planning_solves']==1 and calls['SimulationApp_launches']<=1 and calls['MPC_solves']<=16
    signed={}
    keys=['position_auc_03_m_s','position_auc_full_m_s','yaw_auc_03_rad_s','yaw_auc_full_rad_s',
          'abs_delta_v_from_u_B_plus','abs_delta_omega_from_u_B_plus','linear_TV','angular_TV','swept_clearance_lower_bound_m']
    for base in ORDER[:3]:
        a,z=summaries.get('GRAPH'),summaries.get(base)
        signed['GRAPH_minus_'+base]={k:None if a is None or z is None or a[k] is None or z[k] is None else a[k]-z[k] for k in keys}
    summary=dict(experiment=cfg['experiment'],run=str(run),starting_sha=cfg['starting_sha'],
        scientific_freeze_sha=read(run/'scientific_start.json')['freeze_sha'],classification=execution['classification'],
        source=str(source),handoff='C0_to_C1',horizon=s,entry=entry,graph_planning=graph,geometry=geometry,
        method_references=refs,
        RAW_parity=gate,primary_metrics={n:None if r is None else {k:v for k,v in r.items() if k!='trace'} for n,r in summaries.items()},
        signed_differences=signed,selector_progress=selectors,calls=calls,
        interpretation_rule=cfg['metrics']['classification'],multi_handoff_justified=False)
    return plain(summary),dict(valid=True,saved_only=True,new_scientific_calls=0,
        schedule_comparable=len(rollouts)==4 and all(len(r['commands'])==28 and r['error'] is None for r in rollouts.values()),
        RAW_parity_pass=None if gate is None else gate['passed'],no_retry=True),summaries


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--write',action='store_true');a=p.parse_args()
    run=namespace(a.run);s,v,traces=validate(run)
    if a.write:
        save(OUT/'result_summary.json',s);save(OUT/'validation_summary.json',v)
        save(OUT/'call_accounting.json',s['calls']);save(OUT/'parity_summary.json',s['RAW_parity'])
        save(run/'metric_traces.json',plain(traces))
        fields=['method','position_auc_03_m_s','position_auc_full_m_s','yaw_auc_03_rad_s','yaw_auc_full_rad_s',
            'abs_delta_v_from_u_B_plus','abs_delta_omega_from_u_B_plus','linear_TV','angular_TV','max_abs_v','max_abs_omega',
            'swept_clearance_lower_bound_m','termination']
        with (OUT/'primary.csv').open('x',newline='') as f:
            w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader()
            for n in ORDER:w.writerow(dict(method=n,**(s['primary_metrics'][n] or {})))
    else:
        assert read(OUT/'result_summary.json')==s and read(OUT/'validation_summary.json')==v
    print(s['classification'],s['calls'],v)


if __name__=='__main__':main()
