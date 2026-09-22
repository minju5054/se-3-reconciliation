"""Saved-only GUI loader tests; synthetic data is not source evidence."""
import ast
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from reconciliation.join_source02 import observation_anchored_world

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('source03_gui',ROOT/'scripts/isaac/join_source03_saved_gui.py')
viewer=importlib.util.module_from_spec(spec);spec.loader.exec_module(viewer)


def fixture(tmp,n):
    def write(p,d):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d))
    run=tmp/'run';pose=[1.,2.,np.pi/2]
    write(run/'validation_v2.json',{'valid':True});(run/'config_snapshot.yaml').write_text(yaml.safe_dump({'test':True}))
    tri=run/'tri.npz';np.savez(tri,triangles=np.zeros((1,3,3)))
    m={'observation_pose_world':pose,'triangles_path':str(tri),'final_frames':{}};e={'details':{}}
    raw=np.column_stack([np.arange(n)*.1,np.zeros(n),np.zeros(n)])
    for b in ('A','B','SHAM'):
        rgb=run/(b+'.jpg');rgb.write_bytes(b'synthetic fixture; not model input');m['final_frames'][b]={'path':str(rgb)}
        folder=run/'paired_diagnostics/distance_near_H16/branches'/b/'chunks/terminal';folder.mkdir(parents=True)
        np.save(folder/'raw_local.npy',raw);np.save(folder/'world.npy',observation_anchored_world(raw,pose));e['details'][b]={'raw_local':raw.tolist()}
    write(run/'input_manifests/distance_near_H16.json',m);write(run/'paired_diagnostics/distance_near_H16/evaluation.json',e)
    return run


@pytest.mark.parametrize('n',[1,10,17])
def test_original_world_arrays_remain_read_only_and_generic(tmp_path,n):
    run=fixture(tmp_path,n);s=viewer.load_saved(run)
    assert s['arrays']['B'].shape==(n,3) and not s['arrays']['B'].flags.writeable
    assert np.allclose(s['arrays']['B'][0],[1.,2.,np.pi/2])
    assert all(viewer.sha(p)==h for p,h in s['sources'].items())


def test_no_reanchoring_or_corrupt_saved_raw(tmp_path):
    run=fixture(tmp_path,3);p=run/'paired_diagnostics/distance_near_H16/branches/B/chunks/terminal/world.npy';w=np.load(p);w[:,0]+=1;np.save(p,w)
    with pytest.raises(ValueError,match='observation anchor'):viewer.load_saved(run)


def test_loader_rejects_unvalidated_source(tmp_path):
    run=fixture(tmp_path,3);(run/'validation_v2.json').write_text('{"valid":false}')
    with pytest.raises(ValueError,match='validation invalid'):viewer.load_saved(run)


def test_display_does_not_invoke_inference_controller_or_integrator():
    tree=ast.parse((ROOT/'scripts/isaac/join_source03_saved_gui.py').read_text())
    calls={n.func.id for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)}
    assert not calls.intersection({'Session','MpcTracker','solve_gp','solve_rigid','integrate_unicycle','minimize','counterfactual_rollout'})
