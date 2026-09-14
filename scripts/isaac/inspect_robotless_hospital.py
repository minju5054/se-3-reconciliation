#!/usr/bin/env python3
"""Pre-inference Hospital RGB/geometry inspection; no model or robot."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/robotless_lightnav_successive_chunks.yaml')
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': False})
    try:
        import json
        import numpy as np
        from pxr import Usd, UsdGeom
        from reconciliation.robotless_single_chunk import load_config, save_json_exclusive, sha256_file
        from robotless_runtime import runtime_scene, set_agent_pose, capture_observation, stopped_time
        config = load_config(args.config)
        poses = json.loads(args.poses.read_text())
        args.output.mkdir(parents=True, exist_ok=False)
        stage, agent, camera, annotator, scene = runtime_scene(config, np.array(poses[0]['pose']), app=app)
        observations = []
        for index, item in enumerate(poses):
            location = args.output / f'view_{index:03d}'
            (location / 'raw').mkdir(parents=True)
            entry = {'view_id': index, 'intended_context': item['context'], 'pose': item['pose']}
            try:
                set_agent_pose(agent, np.array(item['pose']), z_m=config['agent']['z_m'])
                observation, check = capture_observation(config, location, agent, camera, annotator, np.array(item['pose']), 0)
                entry.update(observation=observation, capture_check=check,
                    image=str((location/'raw/observation_000.jpg').relative_to(args.output)))
            except Exception as error:
                entry.update(capture_invalid=True, reason=f'{type(error).__name__}: {error}')
            observations.append(entry)
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        bounds = []
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            if prim.IsA(UsdGeom.Mesh):
                box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                if not box.IsEmpty():
                    bounds.append({'prim':str(prim.GetPath()), 'min_world_m':list(box.GetMin()),'max_world_m':list(box.GetMax())})
        save_json_exclusive(args.output/'inspection.json', {'scene':scene,'observations':observations,
            'mesh_world_bounds':bounds,'created_time':stopped_time(),'lightnav_inference_count':0,
            'purpose':'Predeclared bank design from current RGB/geometry only; no predictions or metrics consulted',
            'poses_input_sha256':sha256_file(args.poses),'source_sha256':sha256_file(Path(__file__))})
        print('HOSPITAL_GEOMETRY_INSPECTION_SAVED', flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
