"""Synthetic plot interfaces only; no experimental result, inference or MPC solve."""
import importlib.util
import json
from pathlib import Path
import zipfile

import numpy as np
from PIL import Image
import pytest
from shapely.geometry import box
import yaml
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.se2 import wrap_angle

ROOT = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    loaded = importlib.util.module_from_spec(spec); spec.loader.exec_module(loaded)
    return loaded


plots = module('ref01_plots_test', ROOT / 'scripts/plot_gp_se2_ref01.py')
package = module('ref01_package_test', ROOT / 'scripts/package_gp_se2_ref01_review.py')


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


@pytest.fixture
def fixture_run(tmp_path):
    run = tmp_path / 'synthetic'; run.mkdir()
    cfg = dict(footprint={'radius_m':.2, 'required_clearance_m':.05},
               formulation={'goal_position_tolerance':.15, 'goal_yaw_tolerance':np.pi/12},
               evaluation={'terminal_goal_dwell_s':.2})
    (run / 'config_snapshot.yaml').write_text(yaml.safe_dump(cfg))
    write(run / 'source.json', {'synthetic_only':True, 'environment_path':str(tmp_path / 'environment')})
    write(run / 'protocol.json', {'experiment':'GP-SE2-REF-01', 'synthetic_only':True})
    row = {'case_id':'synthetic/handoff', 'case_directory':'synthetic__handoff', 'case_role':'SYNTHETIC_TEST_ONLY'}
    write(run / 'case_manifest.json', {'selected':[row]})
    case = run / 'cases' / row['case_directory']
    native = np.array([[0., 0., 3.], [.1, 0., 3.1], [.1, 0., -3.1], [.2, 0., -3.]])
    context = dict(B_world=[0., 0., 3.], u_minus=[.2, .3], previous_control=[.1, -.1],
                   old_world=[[-.2, 0., 3.], [0., 0., 3.]], fresh_world=native.tolist())
    write(case / 'input_context.json', context)
    write(case / 'goal_route.json', {'goal_world':native[-1].tolist(), 'gates':[]})
    write(case / 'actual_past_execution.json', {'poses_world':[[-.3, 0., 3.], [0., 0., 3.]], 'times_relative_to_B_s':[-1.5, 0.], 'synthetic_only':True})
    write(case / 'reference_factorization.json', {'k':1, 'synthetic_only':True})
    write(case / 'reproduction_check.json', {'synthetic_only':True})
    matched = [dict(probe_index=i, time_s=i/10., input_pose_world=[i/150., 0., float(wrap_angle(3.+.01*i))], variants={}) for i in range(30)]
    variants = {}
    for number, name in enumerate(plots.VARIANTS):
        folder = case / 'variants' / name
        x = np.arange(4) if number == 0 else np.arange(1, 4) if number == 1 else np.linspace(0 if number == 2 else 1, 3, 30)
        yaw = np.interp(x, np.arange(4), np.unwrap(native[:, 2]))
        ref = np.column_stack([np.interp(x, np.arange(4), native[:, 0]), np.zeros(len(x)), wrap_angle(yaw)])
        lineage = [dict(derived_row_index=i, original_left_row_index=int(np.floor(s)), original_right_row_index=int(np.ceil(s)),
                        interpolation_alpha=float(s % 1), original_fractional_row_coordinate=float(s), world_xy=r[:2].tolist(),
                        wrapped_yaw=float(r[2]), unwrapped_yaw=float(yaw[i]), original_row=bool(s % 1 == 0), interpolated=bool(s % 1 != 0)) for i, (s, r) in enumerate(zip(x, ref))]
        write(folder / 'row_provenance.json', lineage); np.save(folder / 'reference_world.npy', ref)
        write(folder / 'geometry_audit.json', {'row_count':len(ref)})
        rows = []
        for i, probe in enumerate(matched):
            indices = np.minimum(np.arange(1, 6)+i//6, len(ref)-1)
            selected = ref[indices].copy(); selected[:, 2] = np.unwrap(selected[:, 2])
            d = dict(reference_world=selected.tolist(), nearest_original_fractional_row_coordinate=float(x[min(i//6, len(x)-1)]),
                     selected_original_fractional_row_coordinates=x[indices].tolist(), selected_first_target_distance_m=.1,
                     selected_last_target_distance_m=.2, horizon_xy_arc_length_m=.1, horizon_yaw_span_rad=float(selected[-1, 2]-selected[0, 2]),
                     final_goal_row_in_horizon=bool(indices[-1] == len(ref)-1), endpoint_repeated=len(set(indices)) < 5,
                     nearest_tie_margin=.01, indices=indices.tolist(), nearest_index=min(i//6, len(x)-1))
            probe['variants'][name] = d
            rows.append(dict(time_s=i/10., input_pose_world=probe['input_pose_world'], selection_diagnostic=d,
                             command=[.2-number*.01, .3-number*.04]))
        t = np.linspace(0, 3, 181); a = np.column_stack([t/15., np.zeros(len(t)), wrap_angle(3.+t*.1)])
        rollout = dict(states=[dict(time_s=float(v), pose_world=r.tolist()) for v, r in zip(t, a)],
                       controller_reference_selections=rows)
        success = number != 3
        metrics = dict(primary_success=success, failure_reasons=[] if success else ['goal_dwell_failure'],
            execution=dict(terminal_position_error_m=.02+number*.01, terminal_yaw_error_rad=.01+number*.1,
                terminal_goal_dwell_pass=success, terminal_goal_dwell_s=.2, time_to_goal_s=1.2 if success else None,
                motion_limits_pass=True, controller_failure_count=0), minimum_clearance_m=.4,
            dense_times_s=t.tolist(), dense_poses_world=a.tolist(), environment={'clearance_samples_m':[.4]*len(t)})
        write(folder / 'rollout.json', rollout); write(folder / 'metrics.json', metrics)
        variants[name] = dict(rollout=rollout, metrics=metrics)
    write(case / 'matched_state_probes.json', {'states':matched, 'new_mpc_solves':0})
    for name in ('input_audit.csv', 'rollout_outcomes.csv', 'factor_contrasts.csv', 'selection_diagnostics.csv'):
        path = run / 'aggregate' / name; path.parent.mkdir(exist_ok=True); path.write_text('synthetic_only\ntrue\n')
    write(run / 'aggregate/summary.json', {'synthetic_only':True})
    return run, case, variants


def test_plots_preserve_factor_rows_failures_commands_and_yaw_semantics(fixture_run, monkeypatch):
    run, case, variants = fixture_run
    env = HospitalEnvironment(box(2, -2, 2.2, 2), box(-3, -3, 3, 3))
    monkeypatch.setattr(plots.HospitalEnvironment, 'load', lambda path:env)
    assert plots.main(run) == dict(case_count=1, image_count=11, gui_runtime_validated=False)
    manifest = plots.read(run / 'plot_manifest.json')
    assert [Path(r['path']).stem for r in manifest['images']] == list(plots.PLOT_NAMES)
    for row in manifest['images']:
        path = run / row['path']; side = plots.read(path.with_suffix('.json'))
        assert side['image_sha256'] == plots.file_sha256(path)
        assert side['source_hashes'][str(run / 'config_snapshot.yaml')] == plots.file_sha256(run / 'config_snapshot.yaml')
        assert not side['new_execution'] and not side['new_optimization'] and not side['gui_runtime_validated']
        with Image.open(path) as im:
            assert im.size[0] > 1000 and min(im.info['dpi']) >= 159.9
    xy = plots.read(case / 'plots/input_rows_world.json')['numeric_data']
    assert [len(xy['variants'][n]['reference_world']) for n in plots.VARIANTS] == [4, 3, 30, 30]
    actual = plots.read(case / 'plots/actual_trajectory_overlay.json')['numeric_data']
    assert xy['axes_world_m'] == actual['axes_world_m']
    assert xy['detail_axes_world_m'] == actual['detail_axes_world_m']
    outcome = plots.read(case / 'plots/outcome_summary.json')['numeric_data']['rows'][-1]
    assert not outcome['success'] and outcome['time_to_goal_s'] is None
    command = plots.read(case / 'plots/angular_command_vs_time.json')['numeric_data']['R11_CURRENT_ADAPTER']
    assert command['applied_commands'] == [variants['R11_CURRENT_ADAPTER']['rollout']['controller_reference_selections'][0]['command'][1]]*31
    assert command['initial_physical_command'] == .3 and command['initial_controller_memory'] == -.1
    yaw = plots.read(case / 'plots/actual_yaw_and_goal_error.json')['numeric_data']['variants']['R00_NATIVE']
    assert np.max(np.abs(np.diff(yaw['actual_wrapped_yaw_rad']))) > 6
    assert np.max(np.abs(np.diff(yaw['actual_visual_unwrapped_yaw_rad']))) < .01
    probe = plots.read(case / 'plots/matched_state_reference_targets.json')['numeric_data']['snapshots']
    assert [r['time_s'] for r in probe] == [0., 1., 2.]
    assert all(name in (run / 'index.html').read_text() for name in ('R00', 'R10', 'R01', 'R11'))
    with pytest.raises(FileExistsError):
        plots.main(run)
    result = package.package(run)
    assert result['image_count'] == 11 and not result['gui_runtime_validated']
    with zipfile.ZipFile(run / 'review_bundle.zip') as archive:
        names = archive.namelist()
        assert len([n for n in names if n.endswith('.png')]) == 11
        assert not any(n.endswith('.npy') or '/environment/' in n or n.endswith('rollout.json') for n in names)
    with pytest.raises(FileExistsError):
        package.package(run)


def test_selection_series_keeps_official_yaw_distinct_from_display_unwrap():
    rows = []
    for i, yaw in enumerate((3.12, -3.12)):
        d = dict(reference_world=[[0., 0., yaw]]*5,
            nearest_original_fractional_row_coordinate=0., selected_original_fractional_row_coordinates=[1.]*5,
            selected_first_target_distance_m=0., selected_last_target_distance_m=0., horizon_xy_arc_length_m=0.,
            horizon_yaw_span_rad=0., final_goal_row_in_horizon=True, endpoint_repeated=True, nearest_tie_margin=0.)
        rows.append(dict(time_s=i*.1, input_pose_world=[0., 0., yaw], variants={v:d for v in plots.VARIANTS}))
    result = plots.selection_series(rows, matched=True)['R00_NATIVE']
    assert result['official_reference_unwrapped_yaw_rad'][1][0] == -3.12
    assert result['reference_yaw_visual_continuous_rad'][1][0] > np.pi
    assert result['nearest_tie_margin'] == [0., 0.]


def test_missing_variant_or_lineage_mismatch_refuses_plot(fixture_run):
    run, case, _ = fixture_run
    p = case / 'variants/R10_SUFFIX_ONLY/row_provenance.json'
    write(p, [])
    with pytest.raises(ValueError, match='row count'):
        plots.load_variants(case)
    p.unlink()
    with pytest.raises(FileNotFoundError):
        plots.load_variants(case)
    with pytest.raises(FileNotFoundError):
        package.package(run)
    assert not (run / 'review_bundle').exists()
