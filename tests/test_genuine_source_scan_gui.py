"""Synthetic saved-display tests; not experimental evidence."""
import ast
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from reconciliation.gp_se2_reference import prepare_reference
from reconciliation.se2 import local_trajectory_to_world

ROOT=Path(__file__).resolve().parents[1]
path=ROOT/'scripts/isaac/genuine_source_scan_gui.py'
spec=importlib.util.spec_from_file_location('source_inventory_gui',path)
gui=importlib.util.module_from_spec(spec);spec.loader.exec_module(gui)


def fixture(tmp,n):
    ep=tmp/'episode';ctx=ep/'handoffs/handoff/context.json';ctx.parent.mkdir(parents=True)
    chunk=ep/'chunks/fresh';chunk.mkdir(parents=True)
    c=dict(R_old_obs=[1.,2.,0.],R_obs=[1.,2.,0.],B=[1.2,2.,0.],obs_state_id=0,switch_state_id=2,fresh_chunk_id='fresh')
    ctx.write_text(json.dumps(c));paths={'context':str(ctx)}
    raw=np.column_stack([np.linspace(.1,1,n),np.zeros(n),np.zeros(n)])
    world=local_trajectory_to_world(c['R_obs'],raw)
    for kind in ('old','fresh'):
        for frame,a in [('raw_local',raw),('world',world)]:
            p=ep/(kind+'_'+frame+'.npy');np.save(p,a);paths[kind+'_'+frame]=str(p)
    with (ep/'execution.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=['state_id','sim_time_s','x','y','yaw']);w.writeheader()
        for i in range(3):w.writerow(dict(state_id=i,sim_time_s=i*.1,x=1+i*.1,y=2.,yaw=0.))
    rgb=ep/'observation.jpg';rgb.write_bytes(b'fixture only')
    (chunk/'metadata.json').write_text(json.dumps(dict(observation=dict(path=rgb.name,sha256=gui.sha(rgb),pose_world=c['R_obs']))))
    common=prepare_reference(world,c['B'],[10.,10.,1.])['common_world']
    return dict(source_paths=paths,common_value_sha256=hashlib.sha256(common.tobytes()).hexdigest())


@pytest.mark.parametrize('n',[3,10,17])
def test_loader_preserves_raw_world_common_and_past(tmp_path,n):
    row=fixture(tmp_path,n);s=gui.SavedSources.__new__(gui.SavedSources);s.hashes={};c=s.load_case(row)
    assert c['fresh'].shape==(n,3) and not c['fresh'].flags.writeable
    assert np.array_equal(c['past'][0],c['context']['R_obs']) and np.array_equal(c['past'][-1],c['context']['B'])
    assert c['common'].shape==(30,3) and not c['common'].flags.writeable
    assert all(gui.sha(p)==h for p,h in s.hashes.items())


def test_loader_rejects_reanchoring(tmp_path):
    row=fixture(tmp_path,10);p=Path(row['source_paths']['fresh_world']);a=np.load(p);a[:,0]+=.5;np.save(p,a)
    s=gui.SavedSources.__new__(gui.SavedSources);s.hashes={}
    with pytest.raises(ValueError,match='observation anchor'):s.load_case(row)


def test_common_reference_hash_required(tmp_path):
    row=fixture(tmp_path,3);row['common_value_sha256']='wrong';s=gui.SavedSources.__new__(gui.SavedSources);s.hashes={}
    with pytest.raises(ValueError,match='common reference'):s.load_case(row)


def test_saved_clock_selects_samples_without_interpolation():
    s=gui.SavedSources.__new__(gui.SavedSources);s.cases=[dict(past=np.array([[0.,0.,0.],[.1,0,0],[.2,0,0]]),times=np.array([0,.1,.2]))]
    s.select(0);assert s.sample==2 and not s.playing
    s.playing=True;s.seek(.15);assert s.sample==1
    s.seek(1);assert s.sample==2 and not s.playing
    s.seek(-1);assert s.sample==0


def test_no_controller_optimizer_physics_or_integrator_calls():
    tree=ast.parse(path.read_text());names=set()
    for n in ast.walk(tree):
        if isinstance(n,ast.Call):
            if isinstance(n.func,ast.Name):names.add(n.func.id)
            if isinstance(n.func,ast.Attribute):names.add(n.func.attr)
    assert not names.intersection({'integrate_unicycle','counterfactual_rollout','solve_gp','solve_rigid','MpcTracker','Session','simulate','step_physics'})
