#!/usr/bin/env python3
"""Declare and audit the bounded, model-free Acquisition 02 geometry bank."""
import argparse
from pathlib import Path
from copy import deepcopy
import subprocess
import sys
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
from reconciliation.join_source03 import read, save, sha
from reconciliation.blind_corner_source import ray_first_hit, project_camera
from reconciliation.blind_corner_source02 import historical_scale, construct_bank, geometric_probes, geometry_order
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source02_geometry import project_prop

CONFIG = ROOT/'configs/blind_corner_source_acquisition_02.yaml'


def prepare(run):
    cfg = yaml.safe_load(CONFIG.read_text())
    scale = historical_scale(ROOT/cfg['historical_corpus'])
    bank = construct_bank(cfg, scale['finite_arc_m'])
    run.mkdir(parents=True, exist_ok=False)
    (run/'logs').mkdir()
    save(run/'historical_scale.json', scale)
    save(run/'declaration.json', dict(config=cfg, config_sha256=sha(CONFIG), candidates=bank,
        historical_scale_sha256=sha(run/'historical_scale.json'),
        purpose='MODEL-FREE DIAGNOSTIC POSES; NO EXECUTION OR LightNav OUTPUT',
        declared_before_render=True, maximum_new_candidates=3))
    preserve = [ROOT/cfg['preserve_acquisition_01'], ROOT/cfg['environment']]
    files = {str(p.resolve()): sha(p) for folder in preserve for p in folder.rglob('*') if p.is_file()}
    save(run/'source.json', dict(starting_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        origin_main=subprocess.check_output(['git','rev-parse','origin/main'],text=True).strip(),
        preserved=files, scientific_model_calls=0, MPC_calls=0, optimizer_calls=0))
    print(dict(run=str(run), candidates=[c['id'] for c in bank], finite_arc_m=scale['finite_arc_m']))


def audit(run):
    declaration = read(run/'declaration.json'); cfg = declaration['config']; out = run/'technical_preflight'
    assert sha(CONFIG) == declaration['config_sha256']
    assert sha(run/'historical_scale.json') == declaration['historical_scale_sha256']
    scale = read(run/'historical_scale.json')
    assert construct_bank(cfg, scale['finite_arc_m']) == declaration['candidates']
    for p, h in read(run/'source.json')['preserved'].items():
        assert sha(p) == h, p
    expected = {r['identifier']:r['composed_layer_text_sha256'] for r in read(ROOT/cfg['environment']/'scene_provenance.json')['used_layers'] if not r['anonymous']}
    assert expected == read(out/'scene.json')['static_layers'], 'Hospital layers changed'
    base = HospitalEnvironment.load(ROOT/cfg['environment'])
    full = np.load(ROOT/cfg['environment']/'geometry/world_triangles.npz')
    meshes = read(ROOT/cfg['environment']/'geometry/meshes.json'); rows = []
    for c in declaration['candidates']:
        folder = out/c['id']; manifest = read(folder/'manifest.json')
        assert manifest['candidate'] == c
        tri = np.load(folder/'triangles.npz')['triangles']; projection = project_prop(tri)
        cart = projection['obstacle_geometry']; records = []
        for i, pose in enumerate(c['probe_poses']):
            r = read(folder/f'probe_{i:02}.json')
            np.testing.assert_allclose(r['agent_pose_world'], pose, rtol=0, atol=1e-12)
            assert r['same_render_product'] and r['stable_camera_pose_and_simulation_time']
            assert sha(folder/r['rgb_file']) == r['rgb_jpeg_sha256']
            assert sha(folder/r['mask_file']) == r['mask_sha256']
            mask = np.load(folder/r['mask_file'])['mask']
            assert int(np.isin(mask, r['instance']['matched_instance_ids']).sum()) == r['instance']['visible_pixels']
            static = read(folder/f'probe_{i:02}_static.json')
            assert static['snapshot_sha256'] == sha(folder/f'probe_{i:02}.json')
            assert static['static_cart_state'] == manifest['initial_cart_state']
            assert static['static_cart_state']['present'] and static['static_cart_state']['authored_visibility'] == 'inherited'
            records.append(r)
        assert manifest['initial_cart_state'] == manifest['final_cart_state'] and manifest['mesh_unchanged']
        target = (tri.min(axis=(0,1))+tri.max(axis=(0,1)))/2
        camera = records[0]['camera']; origin = np.asarray(camera['T_world_camera'])[:3,3]
        hit = ray_first_hit(origin, target, full['triangles']); screen = project_camera(target, camera)
        if hit:
            hit['mesh'] = meshes[int(full['mesh_ids'][hit['triangle_index']])]['prim_path']
        cart_hit = ray_first_hit(origin, target, tri)
        probes = geometric_probes(c, base, cart, cfg, scale['finite_arc_m'])
        pixels = [r['instance']['visible_pixels'] for r in records]
        first = next((i for i, n in enumerate(pixels) if n >= 20), None)
        visibility_arc = None if first is None else c['probe_arcs_m'][first]
        conflicts = [p['conflict_arc_m'] for p in probes['nominal'] if p['qualifies']]
        reaction = None if visibility_arc is None or not conflicts else min(conflicts)-visibility_arc
        overlap = float(cart.intersection(base.obstacles).area)
        initial = base.check_polyline([c['approach_pose']])
        from reconciliation.blind_corner_source02 import attributed_clearance
        at_visible = None if first is None else attributed_clearance([c['probe_poses'][first]], base, cart)
        flags = dict(initial_hidden=pixels[0]<20, later_visible=first is not None and first>0,
            real_wall_occlusion=bool(hit and cart_hit and hit['distance_m']<cart_hit['distance_m'] and screen['in_frustum'] and
                                    any(hit['mesh'].startswith(p) for p in c['wall_prefixes'])),
            initial_Hospital_safe=initial['clearance_valid'],
            initial_cart_safe=attributed_clearance([c['approach_pose']], base, cart)['combined_m']>=.05,
            visible_probe_safe=bool(at_visible and at_visible['workspace_known'] and at_visible['combined_m']>=.05),
            nominal_turn_finite_conflict=probes['nominal_conflict'],
            bypass_reserve=probes['bypass_pass'], outgoing_free_reserve=probes['outgoing_pass'],
            interaction_in_typical_chunk=any(p['influence_entry_arc_m'] is not None and p['influence_entry_arc_m']<=scale['finite_arc_m']-.20 for p in probes['nominal']),
            reaction_length=bool(reaction is not None and reaction>=cfg['geometry_bank']['reaction_to_conflict_m']),
            editable_spatial_reserve=bool(probes['outgoing_pass'] and cfg['geometry_bank']['outgoing_free_length_m']-cfg['geometry_bank']['reaction_to_conflict_m']>=.60-1e-12),
            cart_no_Hospital_overlap=overlap<=1e-6,
            static_synchronized=True)
        rows.append(dict(candidate_id=c['id'], candidate=c, qualified=all(flags.values()), gates=flags,
            failure_reasons=[k for k,v in flags.items() if not v], pixels=pixels,
            expected_first_visible_probe=first, expected_visibility_arc_m=visibility_arc,
            reaction_to_conflict_m=reaction, wall_ray=hit, cart_ray=cart_hit, cart_center_screen=screen,
            probes=probes, initial_clearance=initial, visible_clearance=at_visible,
            cart_Hospital_overlap_area_m2=overlap, prop=read(folder/'cart.json'), projection=projection['metadata']))
    return dict(valid=True, rows=rows, eligible_order=geometry_order(rows),
                scientific_calls=dict(LightNav_terminal=0, buffers=0, MPC=0, optimization=0),
                scope='MODEL-FREE GEOMETRY PREFLIGHT; NOT EXECUTION',
                classification='BLIND_CORNER_02_GEOMETRY_UNAVAILABLE' if not geometry_order(rows) else None)


def science_prepare(run):
    from run_join_source05 import git
    from reconciliation.join_source03 import mpc_audit
    result=audit(run)
    assert result==read(run/'geometry_qualification.json')
    cfg=read(run/'declaration.json')['config'];base=yaml.safe_load((ROOT/cfg['source_config']).read_text())
    base['experiment']=cfg['experiment']
    base['online'].update(minimum_active_before_prediction_sim_s=0.,maximum_active_sim_s=8.,postroll_sim_s=.10,maximum_handoff_attempts=1)
    save(run/'protocol.json',dict(declaration=cfg,eligible_order=result['eligible_order'],instruction=cfg['instruction'],
        selection_rule=cfg['selection'],maximum_terminal_predictions=2*len(result['eligible_order']),
        first_crossing_only=True,no_later_frame_substitution=True,cart_constant=True,
        coordinates='world=T_world_observation*T_local; raw rows have no intrinsic timestamps',
        scope='SOURCE ACQUISITION ONLY; no optimization/continuation/trackability',geometry_freeze=sha(CONFIG)))
    for row in result['rows']:
        if not row['qualified']:continue
        c=row['candidate'];folder=run/'candidates'/c['id'];folder.mkdir(parents=True);(folder/'logs').mkdir()
        cc=deepcopy(base);cc['lighting']['translation_world_m']=c['fill_translation'];cc['join_online02']['instruction']=cfg['instruction']
        with (folder/'config_snapshot.yaml').open('x') as f:yaml.safe_dump(cc,f,sort_keys=False)
        save(folder/'geometry_input.json',dict(environment_export=str((ROOT/cfg['environment']).resolve()),projection=row['projection']))
        tri_path=run/'technical_preflight'/c['id']/'triangles.npz'
        tri=np.load(tri_path)['triangles'];d=(tri[:,:,:2]-c['cart_center_xy'])@np.array(c['outgoing_xy'])
        save(folder/'scenario.json',dict(source03_input=str(folder/'geometry_input.json'),center_xy=c['cart_center_xy'],forward_xy=c['outgoing_xy'],
            prop=row['prop'],triangles_path=str(tri_path),cart_extents=[float(d.min()),float(d.max())]))
        save(folder/'episode_schedule.json',dict(episodes=[dict(episode_id=c['id'],R0=c['approach_pose'],instruction=cfg['instruction'],cart_present_initial=True,cart_present_dynamic=False)]))
    with (run/'config_snapshot.yaml').open('x') as f:yaml.safe_dump(base,f,sort_keys=False)
    osa=ROOT/'data/obstacle_source_acquisition_03/primary_20260923T085200Z'
    prior=read(run/'source.json')
    preserved={**prior['preserved'],**read(osa/'source.json')['preserved']}
    preserved.update({str(p.resolve()):sha(p) for p in (run/'technical_preflight').rglob('*') if p.is_file()})
    save(run/'scientific_source.json',dict(**{k:v for k,v in prior.items() if k!='preserved'},preserved=preserved,
        source04_run=read(osa/'source.json')['source04_run'],prior_source_sha256=sha(run/'source.json')))
    save(run/'mpc_audit.json',mpc_audit(ROOT))
    assert read(run/'mpc_audit.json')['status']=='MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL'


def freeze(run):
    inherited=read(ROOT/'data/obstacle_source_acquisition_03/primary_20260923T085200Z/freeze.json')['source_sha256']
    for p,h in inherited.items():assert sha(ROOT/p)==h,p
    files=['configs/blind_corner_source_acquisition_02.yaml','src/reconciliation/blind_corner_source02.py',
        'scripts/run_blind_corner_source02.py','scripts/isaac/blind_corner_preflight02.py','scripts/isaac/blind_corner_online02.py',
        'scripts/validate_blind_corner_source02.py','scripts/report_blind_corner_source02.py','tests/test_blind_corner_source02.py',
        'scripts/isaac/blind_corner_online.py','scripts/validate_blind_corner_source.py','scripts/report_blind_corner_source.py',
        'src/reconciliation/blind_corner_source.py','docs/BLIND_CORNER_SOURCE_ACQUISITION_02.md']
    save(run/'freeze.json',dict(source_sha256={**inherited,**{p:sha(ROOT/p) for p in files if not p.startswith('docs/')}},
        protocol_document_sha256=sha(ROOT/files[-1]),input_sha256={str(p.resolve()):sha(p) for p in run.rglob('*') if p.is_file()},
        geometry_candidates=read(run/'protocol.json')['eligible_order']))


def verify(run,pushed=False):
    from run_join_source05 import git
    f=read(run/'freeze.json')
    for group in [f['source_sha256'],f['input_sha256'],read(run/'scientific_source.json')['preserved']]:
        for p,h in group.items():assert sha(ROOT/p)==h,p
    if pushed:
        assert git('rev-parse','HEAD')==git('rev-parse','@{upstream}'),'push freeze first'
        assert not git('status','--porcelain','--',*f['source_sha256']),'uncommitted implementation'
    return f


def start(run):
    # Existing audited server launcher reads source_run/verify through its module globals.
    # Bind this experiment's immutable manifest; no server/model settings change.
    import run_join_source05 as server
    server.verify=verify
    server.source_run=lambda r:Path(read(r/'scientific_source.json')['source04_run'])
    return server.start(run)


if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('--run', type=Path, required=True)
    p.add_argument('--mode', choices=['prepare','audit','science_prepare','freeze','verify','start','stop'], required=True); a=p.parse_args(); run=a.run.resolve()
    if a.mode=='prepare': prepare(run)
    elif a.mode=='audit':
        result=audit(run); save(run/'geometry_qualification.json',result)
        print(dict(eligible_order=result['eligible_order'], classification=result['classification'],
                   failures={r['candidate_id']:r['failure_reasons'] for r in result['rows']}))

    elif a.mode=='verify':verify(run,True)
    elif a.mode=='stop':subprocess.run([sys.executable,str(ROOT/'scripts/lightnav/robotless_online_server.py'),'stop',str(run)],check=True)
    else:dict(science_prepare=science_prepare,freeze=freeze,start=start)[a.mode](run)
