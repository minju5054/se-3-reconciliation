"""Synthetic protocol/checker tests only; no actual model or execution calls."""
import ast
import base64
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import box

from reconciliation.join_source03 import save,path_geometry
from reconciliation.join_source04 import motion_geometry
from reconciliation.join_source05 import (INSTRUCTIONS,ORDER,KS,GENERATION,condition,
    request_parity,verify_bank,classify_instruction)
from reconciliation.join_source02 import observation_anchored_world
from reconciliation.gp_se2_environment import HospitalEnvironment
from test_join_source04 import bank


def test_exact_frozen_order_instructions_and_sham():
    assert len(ORDER)==12
    assert INSTRUCTIONS['I1']=='Avoid the supply cart and continue straight down the hallway.'
    assert INSTRUCTIONS['I2']=='Avoid the supply cart and continue to the end of the hallway.'
    for i in ('I1','I2'):
        a=condition(bank(),f'{i}_K1');b=condition(bank(),f'{i}_K1_SHAM')
        assert a['frames']==b['frames'] and a['instruction']==b['instruction']
    with pytest.raises(ValueError):condition(bank(),'I1_K16')


@pytest.mark.parametrize('k',KS)
def test_reused_H16_no_render_same_poses(k):
    b=bank();a=condition(b,f'I1_K{k}');c=condition(b,f'I2_K{k}')
    assert a['frames']==c['frames']
    assert [f['path'].endswith('_ON.jpg') for f in a['frames']]==[False]*(16-k)+[True]*k
    assert [f['pose_world'] for f in a['frames']]==[r['source_pose_world'] for r in b]
    assert all(f['all_bright'] for f in a['frames'])


def wire_fixture(tmp_path):
    frames=[];old=[];new=[]
    for i in range(16):
        raw=f'actual-JPEG-placeholder-{i}'.encode();p=tmp_path/f'{i}.jpg';p.write_bytes(raw)
        frames.append(dict(path=str(p),sha256=hashlib.sha256(raw).hexdigest()))
        d=dict(seq=i,image=base64.b64encode(raw).decode(),instruction=INSTRUCTIONS['I0'] if i==15 else '')
        old.append(d);new.append({**d,'instruction':INSTRUCTIONS['I1'] if i==15 else ''})
    return new,old,frames


def test_only_terminal_instruction_wire_byte_identity(tmp_path):
    new,old,frames=wire_fixture(tmp_path)
    assert request_parity(new,old,frames,INSTRUCTIONS['I1'])['valid']
    new[3]['instruction']='hidden direction';assert not request_parity(new,old,frames,INSTRUCTIONS['I1'])['valid']


@pytest.mark.parametrize('mutation',['bytes','order','terminal','extra','count'])
def test_payload_drift_rejected(tmp_path,mutation):
    new,old,frames=wire_fixture(tmp_path)
    if mutation=='bytes':new[0]['image']=base64.b64encode(b'edited RGB').decode()
    elif mutation=='order':new[0],new[1]=new[1],new[0]
    elif mutation=='terminal':new[-1]['instruction']+=' Go left.'
    elif mutation=='extra':new[-1]['depth']='forbidden'
    else:new.pop()
    assert not request_parity(new,old,frames,INSTRUCTIONS['I1'])['valid']


def test_all_bank_hashes_and_no_overwrite(tmp_path):
    b=bank();hashes={}
    for i,row in enumerate(b):
        for name in ('OFF','ON'):
            r=row[name]
            for key in ('path','mask_path','depth_path'):
                p=tmp_path/f'{i}_{name}_{key}';p.write_bytes(p.name.encode());hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
                if key=='path':r['frame'].update(path=str(p),sha256=hashes[str(p)])
                else:r[key]=str(p)
            p=tmp_path/f'{i}_{name}.json';save(p,dict(rgb_jpeg_sha256=r['frame']['sha256'],mask_sha256=hashes[r['mask_path']]))
            r['capture_record']=str(p)
    assert len(verify_bank(b,hashes))==96
    Path(b[0]['OFF']['depth_path']).write_bytes(b'changed')
    with pytest.raises(ValueError):verify_bank(b,hashes)
    with pytest.raises(FileExistsError):save(p,{})


@pytest.mark.parametrize('n',[1,3,10,17])
def test_generic_rows_observation_anchor_unchanged(n):
    raw=np.column_stack([np.linspace(.1,1,n),np.zeros(n),np.zeros(n)]);before=raw.copy()
    world=observation_anchored_world(raw,[2,3,np.pi/2])
    assert np.allclose(world[:,0],2) and np.allclose(world[:,1],3+raw[:,0])
    assert np.array_equal(before,raw)
    m=motion_geometry(raw,world,[2,3,np.pi/2],[2,6],[2,4],[0,1],[-.2,.2]);assert m['N']==n


def test_swept_whole_path_no_trim_no_connector():
    env=HospitalEnvironment(box(.49,-.1,.51,.1),box(-5,-5,5,5))
    g=path_geometry(np.array([[0,0,0],[1,0,0]]),env)
    assert g['first_unsafe_waypoint_zero_based'] is None and g['first_unsafe_segment_zero_based']==0
    assert not g['whole']['clearance_valid'] and g['safe_prefix_boundary'] is not None
    assert not g['connector_included']


def sample(clearance=-.1,safe=False,y=0):
    a=[[0,y,0],[1,y,0]]
    return dict(raw_local=a,world=a,raw_text='tokens',stop=False,
        geometry_on=dict(whole=dict(clearance_valid=safe,minimum_clearance_m=clearance)),
        motion=dict(max_abs_lateral_m=abs(y),broad_side='left' if y>=.2 else 'ambiguous',endpoint_beyond_rear_m=.1),
        hallway_endpoint_forward_m=1)


def test_taxonomy_no_forced_recovery_and_text_control():
    influence={} # Classification fixture; actual projection has separate regressions.
    import reconciliation.join_source05 as module
    original=module.compare
    module.compare=lambda a,b,inf:dict(equivalent=a['raw_local']==b['raw_local'],
        meaningful=abs(a['raw_local'][0][1]-b['raw_local'][0][1])>=.2,
        clearance_gain_m=b['geometry_on']['whole']['minimum_clearance_m']-a['geometry_on']['whole']['minimum_clearance_m'])
    try:
        h={('K0_OFF' if k==0 else f'K{k}'):sample() for k in KS}
        r={c:sample() for c in ORDER if c.startswith('I1')}
        assert classify_instruction(r,h,'I1',influence)['classification']=='NO_MATERIAL_INSTRUCTION_EFFECT'
        r['I1_K4']=sample(clearance=-.07)
        assert classify_instruction(r,h,'I1',influence)['classification']=='INSTRUCTION_IMPROVES_BUT_REMAINS_UNSAFE'
        r['I1_K4']=sample(clearance=.1,safe=True,y=.3)
        assert classify_instruction(r,h,'I1',influence)['classification']=='INSTRUCTION_RECOVERS_VISUAL_CONDITIONED_SAFE_DETOUR'
        for c in r:r[c]=sample(clearance=.1,safe=True,y=.3)
        assert classify_instruction(r,h,'I1',influence)['classification']=='INSTRUCTION_ONLY_STEERING'
        r['I1_K1_SHAM']=sample(y=.4)
        assert classify_instruction(r,h,'I1',influence)['classification']=='INSTRUCTION_EFFECT_INCONCLUSIVE'
    finally:module.compare=original


def test_no_hidden_model_render_execution_in_saved_evaluator():
    root=Path(__file__).resolve().parents[1]
    tree=ast.parse((root/'src/reconciliation/join_source05.py').read_text())
    names={n.id for n in ast.walk(tree) if isinstance(n,ast.Name)}
    assert not names.intersection({'Session','MpcTracker','integrate_unicycle','minimize','solve_gp','SimulationApp'})
    assert GENERATION==dict(VLN_EVAL_TEMPERATURE='0',VLN_EVAL_TOP_P='1',VLN_EVAL_TOP_K='0',VLN_EVAL_TRAJ_TOP1='0')


def test_saved_replay_cross_boot_clock_preserves_source_and_official_history():
    from reconciliation.join_source05_history import SavedReplayHistory
    from reconciliation.online_history import SessionHistory
    from test_online_history import frame,contract
    frames=[frame(i,capture_monotonic_ns=10**15+i*250_000_000) for i in range(16)]
    c=contract(64,'ring');h=SavedReplayHistory(INSTRUCTIONS['I1'],c,None,frames)
    original=SessionHistory(INSTRUCTIONS['I1'],c,None)
    for x in (h,original):x.login();x.reset()
    for i,f in enumerate(frames):
        a=h.begin(f,predict=i==15,send_monotonic_ns=100+i)
        b=original.begin(f,predict=i==15,send_monotonic_ns=f['capture_monotonic_ns']+100)
        assert a['frame']==b['frame'] and a['frame']['capture_monotonic_ns']==f['capture_monotonic_ns']
        assert a['send_monotonic_ns']==100+i
        aa=h.complete(i,server_actions_step=16 if i==15 else None)
        bb=original.complete(i,server_actions_step=16 if i==15 else None)
        clock=aa.pop('clock_semantics');assert not clock['cross_boot_subtraction_performed']
        assert aa==bb
    assert h.prediction_count==1 and h.next_seq==16


def test_saved_replay_refuses_new_frames_and_extra_predictions():
    from reconciliation.join_source05_history import SavedReplayHistory
    from test_online_history import frame,contract
    frames=[frame(i) for i in range(16)]
    h=SavedReplayHistory(INSTRUCTIONS['I1'],contract(64,'ring'),None,frames);h.login();h.reset()
    with pytest.raises(ValueError):h.begin(frames[0],predict=True,send_monotonic_ns=10)
    changed=deepcopy(frames[0]);changed['pose_world'][0]+=1
    with pytest.raises(ValueError):h.begin(changed,predict=False,send_monotonic_ns=10)


def test_replay_worker_has_no_research_geometry_dependency():
    p=Path(__file__).resolve().parents[1]/'scripts/lightnav/join_source05_predict.py'
    tree=ast.parse(p.read_text())
    modules=[n.module or '' for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    assert 'reconciliation.join_source05' not in modules
    assert 'online_lightnav_worker' in modules
