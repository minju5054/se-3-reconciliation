#!/usr/bin/env python3
"""One native continuous episode using the byte-unchanged OSA03 runtime."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'scripts/isaac')]
from reconciliation.join_source03 import read, save, sha
from reconciliation.robotless_online import stamp
from reconciliation.continuous_obstacle_reveal_episode01 import RequestPolicy
from robotless_online_handoffs import Worker, collect_episode
from join_online02_collect import GeometryWorker, setup_scene
from obstacle_source03_online import SuddenReveal


class ContinuousReveal(SuddenReveal):
    def __init__(self, inst, prop, geometry, protocol):
        super().__init__(inst, prop, geometry, protocol)
        self.policy = RequestPolicy()
        self.active = self.applied = None

    def before_capture(self, ep, pose, state, active, last_activation):
        super().before_capture(ep, pose, state, active, last_activation)
        self.active, self.applied = active, last_activation

    def after_capture(self, ep, frame):
        super().after_capture(ep, frame)
        from pxr import Usd, UsdGeom
        matrix = np.asarray(UsdGeom.Xformable(self.prop['prim']).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        np.testing.assert_allclose(matrix, self.prop['record']['wrapper_matrix_column'], atol=1e-12, rtol=0)
        visible = str(UsdGeom.Imageable(self.prop['prim']).ComputeVisibility()) != 'invisible'
        assert visible == self.present == self.geometry.present
        save(ep/'cart_states'/f'{frame["frame_id"]}.json', dict(frame_id=frame['frame_id'],
            actual_world_matrix_column=matrix.tolist(), renderer_present=visible,
            oracle_present=self.geometry.present, sim_time_s=frame['capture_sim_time_s']))
        cid = self.policy.capture(frame, self.active, self.applied)
        if cid:
            save(ep/'eligible_observations'/f'{cid}.json', dict(
                frame=frame, preceding_active=self.active['chunk_id'],
                preceding_application_sim_s=self.applied, cart_present=self.present,
                terminal_inflight=self.policy.inflight))

    def allow_fresh(self, frame):
        return self.policy.allow(frame)


class ContinuousGeometry(GeometryWorker):
    def __init__(self, run):
        self.log = (run/'logs/geometry.log').open('x')
        env = {k:v for k,v in os.environ.items() if k not in ('LD_LIBRARY_PATH', 'PYTHONPATH')}
        env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
        self.p = subprocess.Popen([str(ROOT/'.venv/bin/python'),
            str(ROOT/'scripts/continuous_obstacle_reveal_geometry_worker.py'), str(run)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log,
            text=True, bufsize=1, env=env, cwd=ROOT)
        assert json.loads(self.p.stdout.readline()) == {'ready': True}


class NativePolicyWorker:
    """Forward official worker messages; gate only eligibility and installation.

    Original worker response/NPY files remain untouched. A rejected reference is
    exposed to the historical collector as SCENE_INVALID, with RAW_UNSAFE recorded
    separately. This is an abort, never a replacement command or reference.
    """
    def __init__(self, worker, hook, geometry, ep):
        self.worker, self.hook, self.geometry, self.ep = worker, hook, geometry, ep
        self.opens = 0

    def send(self, op, **payload):
        if op == 'open':
            if self.opens:
                raise ValueError('no reset/reopen after episode initialization')
            self.opens += 1
        if op == 'frame':
            self.hook.policy.send(payload['frame'], payload['predict'], payload.get('chunk_id'))
        return self.worker.send(op, **payload)

    def await_type(self, *args, **kwargs):
        return self.worker.await_type(*args, **kwargs)

    def drain(self):
        out = []
        for msg in self.worker.drain():
            if msg.get('type') == 'result' and msg.get('kind') == 'prediction':
                self.hook.policy.received(msg['chunk_id'])
                if msg['status'] == 'PREDICTION_READY':
                    check = self.geometry.ask('reference', world=msg['world'])
                    save(self.ep/'reference_checks'/f'{msg["chunk_id"]}.json', dict(
                        check=check, cart_present=self.geometry.present,
                        raw_ref=msg['raw_local_ref'], world_ref=msg['world_ref'],
                        before_install=True, host=stamp()))
                    if not check['clearance_valid']:
                        msg = {**msg, 'status': 'SCENE_INVALID', 'error': 'RAW_UNSAFE: complete returned native polyline'}
                        save(self.ep/'raw_unsafe_abort.json', dict(chunk_id=msg['chunk_id'],
                            reference_installed=False, policy_reason='RAW_UNSAFE', check=check))
            out.append(msg)
        return out


def main():
    p = argparse.ArgumentParser(); p.add_argument('--run', type=Path, required=True)
    run = p.parse_args().run.resolve()
    # Geometry/report dependencies live in the repository environment, not Isaac's.
    subprocess.run([str(ROOT/'.venv/bin/python'),str(ROOT/'scripts/run_continuous_obstacle_reveal_episode01.py'),
        '--mode','verify','--run',str(run)],check=True,
        env={k:v for k,v in os.environ.items() if k not in ('LD_LIBRARY_PATH','PYTHONPATH')})
    assert read(run/'generation_actual.json')['valid']
    if (run/'execution_start.json').exists() or (run/'episodes').exists():
        raise FileExistsError('single episode already attempted; no retry')
    save(run/'execution_start.json', dict(sha=subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
        at=stamp(), pid=os.getpid(), scientific_episodes=1, retries=0))
    cfg = yaml.safe_load((run/'config_snapshot.yaml').read_text())
    specs = read(run/'episode_schedule.json')['episodes']; assert len(specs) == 1
    spec = specs[0]
    from isaacsim import SimulationApp
    app = SimulationApp(dict(headless=True, renderer='RayTracedLighting', anti_aliasing=0, display_options=0))
    model = mpc = geo = hook = None
    try:
        stage, agent, camera, rgb, inst, sem, prop, scene = setup_scene(run, cfg, spec['R0'], app)
        save(run/'acquisition_scene.json', scene)
        from join_source02_preflight import set_present
        set_present(prop, False)
        from isaacsim.core.api import World
        world = World(physics_dt=1/60, rendering_dt=1/60, stage_units_in_meters=1.)
        checkout = (ROOT/cfg['paths']['lightnav_checkout']).resolve()
        model = Worker([str(checkout/'.venv/bin/python'), str(ROOT/'scripts/online_lightnav_worker.py'),
            '--config', str(run/'config_snapshot.yaml')], run/'logs/lightnav_worker.log')
        mpc = Worker([str(checkout/'mujoco_demo/.venv/bin/python'), str(ROOT/'scripts/online_mpc_worker.py'),
            '--lightnav-checkout', str(checkout)], run/'logs/mpc_worker.log')
        geo = ContinuousGeometry(run); geo.set(False)
        mr, cr = model.await_type('ready'), mpc.await_type('ready')
        save(run/'workers.json', dict(model=mr, mpc=cr))
        hook = ContinuousReveal(inst, prop, geo, read(run/'protocol.json'))
        ep = run/'episodes'/spec['episode_id']
        policy_model = NativePolicyWorker(model, hook, geo, ep)
        result = collect_episode(spec, run, cfg, app, world, agent, camera, rgb, scene,
            policy_model, mpc, None, mr, cr, intervention=hook, command_guard=geo.guard)
        mpc.shutdown(); mpc.reader.join(timeout=5)
        save(ep/'controller/post_episode_messages.json',dict(messages=mpc.drain(),
            physically_applied=False, simulation_advanced=False))
        mpc = None
        hook.close(); hook = None
        save(ep/'acquisition_extension_manifest.json', dict(files={str(q.relative_to(ep)):sha(q)
            for q in sorted(ep.rglob('*')) if q.is_file()}))
        save(run/'schedule_completion.json', dict(episodes=[dict(episode=spec['episode_id'],
            status=result['status'], predictions=result['predictions_requested'])], end=stamp(), no_retry=True))
    except BaseException as exc:
        save(run/'collection_failure.json', dict(error=repr(exc), at=stamp(), no_retry=True))
        raise
    finally:
        if hook: hook.close()
        if model: model.shutdown('shutdown')
        if mpc: mpc.shutdown()
        if geo: geo.close()
        app.close()

if __name__ == '__main__':
    main()
