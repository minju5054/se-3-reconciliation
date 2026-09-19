"""Synthetic REF-03 orchestration checks; no actual controller optimization."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from reconciliation.gp_se2_ref01_reference import build_variants
from reconciliation.gp_se2_ref02_reference import METHODS
from reconciliation.gp_se2_ref02_rollout import counterfactual_rollout as frozen_rollout
from reconciliation.gp_se2_reference import prepare_reference
from test_gp_se2_ref02_rollout import fixture_runtime

spec = importlib.util.spec_from_file_location('ref03_worker_test', Path(__file__).resolve().parents[1]/'scripts/lightnav/gp_se2_ref03_mpc.py')
worker = importlib.util.module_from_spec(spec); spec.loader.exec_module(worker)


def make_loaded(native_count=7, *, case_number=0, cohort='KNOWN_CONTROLS', gate=False):
    module = fixture_runtime()
    native = np.column_stack([np.linspace(.05, .75, native_count), np.linspace(0, .1, native_count), np.linspace(.1, .6, native_count)])
    context = dict(case_id=f'episode_fixture_{case_number}/handoff_fixture', episode_id=f'episode_fixture_{case_number}',
        handoff_id='handoff_fixture', B_world=[0., 0., 0.], u_minus=[.1, .2], previous_control=[.3, -.1],
        original_capture_pose_world=[-1., 2., .8], source_files=[], fresh_world=native.tolist(),old_world=native.tolist())
    common = prepare_reference(native, context['B_world'], module.Q_WEIGHTS)['common_world']
    variants = build_variants(native, context['B_world'], module.Q_WEIGHTS, expected_common=common)['variants']
    mappings = ('R00_NATIVE','R11_CURRENT_ADAPTER','R11_CURRENT_ADAPTER')
    references = {m:variants[v]['reference_world'] for m,v in zip(METHODS,mappings)}
    lineages = {m:variants[v]['row_provenance'] for m,v in zip(METHODS,mappings)}
    metadata = {m:None for m in METHODS}
    for method in METHODS:
        progress = [r['original_fractional_row_coordinate'] for r in lineages[method]]
        if len(set(progress))==1:
            metadata[method]=dict(constant_reference=True,retained_source_row_count=1,original_source_row_index=int(progress[0]))
    case = dict(episode_id=context['episode_id'],handoff_id=context['handoff_id'],cohort=cohort,
        case_role='REPRO_BENIGN' if cohort=='KNOWN_CONTROLS' else 'S_BENIGN_STRAIGHT',methods={m:{} for m in METHODS})
    if cohort=='ADDITIONAL_TRANSFER':case['stratum']='S_BENIGN_STRAIGHT'
    return module, dict(case=case,frozen=context, references=references,lineages=lineages,metadata=metadata,
        reference_sources={m:dict(path='synthetic_reference',sha256='synthetic_not_evidence') for m in METHODS},
        method_errors={},goal_route=dict(goal_world=native[-1].tolist(),gates=[{'id':'unchanged_fixture_gate'}] if gate else []))


def test_case_order_A_then_current_A_probes_then_B_C(tmp_path):
    module, loaded = make_loaded(gate=True)
    events=[]
    rows,probes=worker.execute_case(module,loaded,tmp_path/'case','settings',0,events.append)
    assert [(e['type'],e.get('method')) for e in events]==[
        ('METHOD_STARTED',METHODS[0]),('METHOD_FINISHED',METHODS[0]),('PROBES_STARTED',None),('PROBES_FINISHED',None),
        ('METHOD_STARTED',METHODS[1]),('METHOD_FINISHED',METHODS[1]),('METHOD_STARTED',METHODS[2]),('METHOD_FINISHED',METHODS[2])]
    assert all(r['completed'] and r['attempted'] for r in rows)
    assert [r['primary_mpc_solve_count'] for r in rows]==[30,30,30]
    assert probes['selector_calls_attempted']==probes['selector_calls_completed']==60
    assert probes['new_mpc_solves']==0 and probes['probe_count']==30
    assert probes['status']=='COMPLETED' and not probes['historical_pose_fallback']
    a=json.loads((tmp_path/'case'/METHODS[0]/'rollout.json').read_text())
    assert probes['source_rollout_sha256']==hashlib.sha256((tmp_path/'case'/METHODS[0]/'rollout.json').read_bytes()).hexdigest()
    assert all(p['input_pose_world']==s['input_pose_world'] and p['time_s']==s['time_s'] for p,s in zip(probes['states'],a['controller_reference_selections']))
    assert all(tuple(p['methods'])==METHODS[1:] and p['B_C_nearest_exactly_equal'] for p in probes['states'])
    assert json.loads((tmp_path/'case'/'goal_route.json').read_text())['gates']==loaded['goal_route']['gates']
    assert len(a['states'])==181 and len(a['commands'])==180
    assert a['initial_physical_command']!=a['initial_previous_control']


def test_construction_recording_preserves_all_inherited_methods_and_outputs():
    module,loaded=make_loaded()
    tracked=worker._TrackingModule(module)
    for method in ('submit','_solve','poll','set_body_path','reset','close'):
        assert getattr(tracked.MpcTracker,method) is getattr(module.MpcTracker,method)
    method=METHODS[2]
    kwargs=dict(original_goal_row_index=len(loaded['frozen']['fresh_world'])-1)
    plain=frozen_rollout(module,loaded['frozen'],loaded['references'][method],loaded['lineages'][method],method,**kwargs)
    observed=frozen_rollout(tracked,loaded['frozen'],loaded['references'][method],loaded['lineages'][method],method,**kwargs)
    assert len(tracked.instances)==1
    for key in ('states','commands','installed_reference_world','candidate_capture_local','initial_previous_control','initial_physical_command'):
        assert plain[key]==observed[key]
    assert all(observed['wrapper_identity'].values())
    capture=worker._partial_capture(tracked)
    assert capture['actual_controller_calls']==capture['actual_selector_calls_recorded']==30


@pytest.mark.parametrize('native_count',[1,2,4,13])
def test_arbitrary_native_length_and_constant_source_metadata(tmp_path,native_count):
    module,loaded=make_loaded(native_count)
    rows,probes=worker.execute_case(module,loaded,tmp_path/'case','settings',0,lambda e:None)
    assert all(r['completed'] for r in rows)
    assert probes['status']=='COMPLETED'
    assert len(loaded['references'][METHODS[0]])==native_count
    assert len(loaded['references'][METHODS[1]])==30
    if native_count<=2:
        c=json.loads((tmp_path/'case'/METHODS[2]/'rollout.json').read_text())
        assert all(s['selection']['indices']==[29]*5 for s in c['controller_reference_selections'])
        assert loaded['metadata'][METHODS[2]]['retained_source_row_count']==1


def test_original_controller_failures_are_completed_not_pruned(tmp_path):
    _,loaded=make_loaded()
    module=fixture_runtime(failure_at=1)
    rows,probes=worker.execute_case(module,loaded,tmp_path/'case','settings',0,lambda e:None)
    assert all(r['completed'] and r['controller_failure_count']==1 for r in rows)
    assert sum(r['primary_mpc_solve_count'] for r in rows)==90
    assert probes['status']=='COMPLETED' and probes['selector_calls_completed']==60
    for method in METHODS:
        rollout=json.loads((tmp_path/'case'/method/'rollout.json').read_text())
        assert rollout['commands'][6]['command']==[0.,0.]
        assert rollout['controller_reference_selections'][2]['previous_control']==[0.,0.]


def test_technical_partial_A_preserved_no_historical_probe_fallback_remaining_methods_run(tmp_path):
    module,loaded=make_loaded()
    original=module.build_pose_aligned_reference;calls=[]
    def fail_fourth(*args,**kwargs):
        calls.append(None)
        if len(calls)==4:raise RuntimeError('synthetic selector technical fault')
        return original(*args,**kwargs)
    module.build_pose_aligned_reference=fail_fourth
    rows,probes=worker.execute_case(module,loaded,tmp_path/'case','settings',0,lambda e:None)
    assert rows[0]['attempted'] and not rows[0]['completed'] and rows[0]['status']=='TECHNICAL_FAILURE'
    assert rows[0]['actual_controller_calls']==3 and rows[0]['completed_control_cycles'] is None
    assert not (tmp_path/'case'/METHODS[0]/'rollout.json').exists()
    assert (tmp_path/'case'/METHODS[0]/'partial_actual_calls.json').is_file()
    assert rows[1]['completed'] and rows[2]['completed']
    assert probes['status']=='UNAVAILABLE' and probes['unavailable_reason']=='CURRENT_A_ROLLOUT_INCOMPLETE_OR_UNAVAILABLE'
    assert probes['selector_calls_attempted']==0 and not probes['historical_pose_fallback']
    assert sum(r['primary_mpc_solve_count'] for r in rows)==63


def test_missing_A_input_preserves_ledger_and_does_not_skip_B_C(tmp_path):
    module,loaded=make_loaded()
    loaded['method_errors'][METHODS[0]]='missing original native file'
    rows,probes=worker.execute_case(module,loaded,tmp_path/'case','settings',0,lambda e:None)
    assert not rows[0]['attempted'] and rows[0]['status']=='INPUT_UNAVAILABLE'
    assert all(r['completed'] for r in rows[1:])
    assert probes['status']=='UNAVAILABLE'
    assert len(json.loads((tmp_path/'case'/'ledger.json').read_text())['methods'])==3


def test_bad_case_context_keeps_all_planned_methods_unavailable(tmp_path):
    _,loaded=make_loaded();loaded['case_error']='source context mismatch'
    rows,probes=worker.execute_case(fixture_runtime(),loaded,tmp_path/'case','settings',0,lambda e:None)
    assert len(rows)==3 and all(not r['attempted'] and not r['completed'] for r in rows)
    assert probes['selector_calls_completed']==0


def test_probe_failure_records_partial_audits_and_does_not_cancel_rollouts(tmp_path,monkeypatch):
    module,loaded=make_loaded();original=worker.selector_result;calls=[]
    def broken(*args,**kwargs):
        calls.append(None)
        if len(calls)==2:raise ValueError('synthetic one-probe audit failure')
        return original(*args,**kwargs)
    monkeypatch.setattr(worker,'selector_result',broken)
    rows,probes=worker.execute_case(module,loaded,tmp_path/'case','settings',0,lambda e:None)
    assert all(r['completed'] for r in rows)
    assert probes['status']=='TECHNICAL_FAILURE' and probes['selector_calls_attempted']==60 and probes['selector_calls_completed']==59
    assert probes['states'][0]['methods'][METHODS[2]]['status']=='TECHNICAL_FAILURE'


def test_next_independent_case_runs_after_first_case_scientific_failure(tmp_path):
    module,first=make_loaded();first['method_errors'][METHODS[2]]='synthetic input failure'
    _,second=make_loaded(case_number=1,cohort='ADDITIONAL_TRANSFER')
    a,_=worker.execute_case(module,first,tmp_path/'first','settings',0,lambda e:None)
    b,p=worker.execute_case(module,second,tmp_path/'second','settings',3,lambda e:None)
    assert not a[2]['completed'] and all(r['completed'] for r in b)
    assert [r['planned_execution_ordinal'] for r in b]==[3,4,5]
    assert p['selector_calls_completed']==60


def test_fixed_request_case_cohort_method_schedule_no_retry_or_fallback():
    _,control=make_loaded();_,transfer=make_loaded(case_number=1,cohort='ADDITIONAL_TRANSFER')
    cases=[control['case'],transfer['case']]
    request=dict(cases=cases,frozen_case_order=[c['episode_id']+'/'+c['handoff_id'] for c in cases])
    assert worker.verify_request(request)
    for mutation in ('frozen_order','cohort','method','stride','fallback'):
        bad=deepcopy(request)
        if mutation=='frozen_order':bad['frozen_case_order'].reverse()
        elif mutation=='cohort':bad['cases'].reverse();bad['frozen_case_order'].reverse()
        elif mutation=='method':bad['cases'][0]['methods']=dict(reversed(list(bad['cases'][0]['methods'].items())))
        elif mutation=='stride':bad['source_progress_stride']=.5
        else:bad['cases'][0]['methods'][METHODS[1]]=None
        with pytest.raises(ValueError):worker.verify_request(bad)


def test_output_refuses_overwrite(tmp_path):
    module,loaded=make_loaded()
    worker.execute_case(module,loaded,tmp_path/'case','settings',0,lambda e:None)
    with pytest.raises(FileExistsError):worker.execute_case(module,loaded,tmp_path/'case','settings',0,lambda e:None)


def test_input_loader_authenticates_sources_gate_and_constant_metadata(tmp_path,monkeypatch):
    _,loaded=make_loaded(native_count=2,gate=True)
    case=deepcopy(loaded['case'])
    for name,value in [('input_context',loaded['frozen']),('goal_route',loaded['goal_route'])]:
        path=tmp_path/f'{name}.json';worker.write_new(path,value);case[name+'_path']=str(path)
    for method in METHODS:
        folder=tmp_path/method;folder.mkdir()
        path=folder/'reference.npy';np.save(path,loaded['references'][method])
        lineage=folder/'lineage.json';worker.write_new(lineage,loaded['lineages'][method])
        case['methods'][method]=dict(reference_path=str(path),lineage_path=str(lineage))
        if loaded['metadata'][method]:
            metadata=folder/'metadata.json';worker.write_new(metadata,loaded['metadata'][method])
            case['methods'][method]['lineage_metadata_path']=str(metadata)
    monkeypatch.setattr(worker,'load_frozen_context',lambda *a:deepcopy(loaded['frozen']))
    records=[];result=worker.load_case(tmp_path,case,records)
    assert not result.get('case_error') and not result['method_errors']
    assert result['goal_route']==loaded['goal_route']
    assert result['metadata'][METHODS[2]]==loaded['metadata'][METHODS[2]]
    assert all(Path(r['path']).is_file() and len(r['sha256'])==64 for r in records)
    # Unknown invalid lineage stays a missing method, never a substituted path.
    bad=deepcopy(case);bad['methods'][METHODS[2]]['lineage_path']=str(tmp_path/'missing_lineage.json')
    result=worker.load_case(tmp_path,bad,[])
    assert set(result['method_errors'])=={METHODS[2]}
    assert METHODS[2] not in result['references']


def test_declared_request_counts_must_equal_matrix():
    _,loaded=make_loaded();case=loaded['case']
    r=dict(cases=[case],frozen_case_order=[case['episode_id']+'/'+case['handoff_id']],planned_rollouts=3,primary_mpc_solves=90,selector_probes=60)
    assert worker.verify_request(r)
    for key in ('planned_rollouts','primary_mpc_solves','selector_probes'):
        wrong=deepcopy(r);wrong[key]+=1
        with pytest.raises(ValueError,match='counts'):worker.verify_request(wrong)
