#!/usr/bin/env python3
"""Unchanged native loop/guard/workers with twelve-request policy and fixed pose."""
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts/isaac'),str(ROOT/'scripts'),str(ROOT/'src')]
import continuous_obstacle_reveal_episode01 as collector
from long_continuous_obstacle_reveal_source01 import ConfiguredWorker,ExploratoryGeometry
from reconciliation.successive_native_source_acquisition02 import RequestPolicy
from reconciliation.join_source03 import read,save
_original_setup=collector.setup_scene

def setup_scene(run,cfg,pose,app):
    candidate=read(run/'candidate.json')
    np.testing.assert_array_equal(pose,candidate['pose_world'])
    result=_original_setup(run,cfg,pose,app)
    record=result[-1]
    for k in ('T_world_agent','T_world_camera','T_agent_camera'):
        np.testing.assert_allclose(record['camera'][k],candidate[k],rtol=0,atol=1e-12)
    assert candidate['geometry_valid']
    save(run/'candidate_scene_validation.json',dict(candidate_id=candidate['candidate_id'],
        valid=True,actual_camera=record['camera'],before_model_requests=True))
    return result

if __name__=='__main__':
    collector.RequestPolicy=RequestPolicy
    collector.ContinuousGeometry=ExploratoryGeometry
    collector.Worker=ConfiguredWorker
    collector.setup_scene=setup_scene
    collector.main()
