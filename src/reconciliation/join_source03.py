"""Saved-source and image/geometry diagnostics; no controller or model import.

Image statistics never modify an input. Polylines are untimed model output,
not executed motion. Missing observations remain None.
"""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from .online_mpc_adapter import PINNED_LIGHTNAV_SHA, PINNED_MPC_SHA256, SETTING_NAMES
from .se2 import wrap_angle


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False,
                  default=lambda x: x.tolist() if isinstance(x, np.ndarray) else str(x))


def literal_settings(source):
    """Read pinned module constants without constructing a solver."""
    result = {}
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in SETTING_NAMES:
                    try:
                        result[target.id] = ast.literal_eval(node.value)
                    except ValueError:
                        pass
    return result


def compare_mpc(current, historical):
    if not historical:
        return 'MPC_AUDIT_INCONCLUSIVE'
    for item in [current, *historical]:
        if (item.get('lightnav_sha') != PINNED_LIGHTNAV_SHA
                or item.get('mpc_source_sha256') != PINNED_MPC_SHA256
                or item.get('external_git_status') != ''):
            return 'MPC_PROVENANCE_MISMATCH'
    keys = ('HORIZON', 'MPC_DT_S', 'CONTROL_RATE_HZ', 'Q_WEIGHTS', 'R_WEIGHTS',
            'OBJNAV_V_MAX', 'W_MAX', 'A_MAX_V', 'A_MAX_W')
    for item in historical:
        for key in keys:
            a, b = current['official_settings'].get(key), item['official_settings'].get(key)
            if a is None or b is None:
                return 'MPC_AUDIT_INCONCLUSIVE'
            if not np.array_equal(a, b):
                return 'MPC_PROVENANCE_MISMATCH'
    return 'MPC_PROVENANCE_MATCHES_PINNED_OFFICIAL'


def mpc_audit(root):
    root = Path(root)
    checkout = root.parent / 'external/LightNav-0-official-demo'
    source = checkout / 'mujoco_demo/vln_mujoco/mpc.py'
    def git(*args):
        return subprocess.check_output(['git', '-C', str(checkout), *args], text=True).strip()
    current = dict(lightnav_sha=git('rev-parse', 'HEAD'), external_git_status=git('status', '--porcelain'),
                   mpc_source=str(source), mpc_source_sha256=sha(source),
                   official_settings=literal_settings(source.read_text()))
    references = [
        ('dataset', 'data/robotless_online_handoffs_v1/primary_20260915T091900Z/logs/workers_1789464022363822358.json', 'mpc'),
        ('JOIN01', 'data/robotless_gp_se2_join_01/primary_20260921T083025Z/source_event/workers.json', 'mpc'),
        ('SOURCE02', 'data/robotless_join_source_02/source_20260921T112111Z/development/history_acquisition/workers.json', 'mpc'),
        ('REF01', 'data/robotless_gp_se2_ref_01/primary_20260919T062000Z/source.json', 'official_mpc'),
        ('REF02', 'data/robotless_gp_se2_ref_02/primary_20260919T081000Z/source.json', 'official_mpc'),
        ('REF03', 'data/robotless_gp_se2_ref_03/primary_20260919T141000Z/source.json', 'official_mpc'),
    ]
    history = []
    for label, path, key in references:
        d = read(root / path)[key]
        p = d.get('provenance', d)
        history.append(dict(label=label, source=path, source_sha256=sha(root / path), **p))
    files = ['scripts/online_mpc_worker.py', 'src/reconciliation/online_mpc_adapter.py',
             'scripts/isaac/robotless_online_handoffs.py', 'scripts/isaac/join_source02_collect.py',
             'scripts/lightnav/join_source02_paired.py', 'scripts/online_lightnav_worker.py',
             'src/reconciliation/gp_se2_ref02_rollout.py']
    return dict(status=compare_mpc(current, history), current=current, historical=history,
                inspected_research_files={p: sha(root / p) for p in files},
                source_chain='online collector -> online_mpc_worker -> load_official/audited_tracker -> official MpcTracker',
                ref_scope='REF01 changes input preparation; REF02/03 per-instance selector differs; official calculation unchanged; not online default',
                paired_terminal_scope='independent sessions with identical restored history; no terminal MPC invocation',
                limitation='unchanged MPC can still affect earlier acquired RGB indirectly through motion',
                new_model_calls=0, new_MPC_solves=0)


def luminance_statistics(rgb, mask=None):
    """Rec.709 luma of stored sRGB code values (not physical radiance)."""
    a = np.asarray(rgb)
    if a.ndim != 3 or a.shape[2] != 3 or a.dtype != np.uint8:
        raise ValueError('expected untouched uint8 RGB')
    y = a.astype(np.float64) @ np.array([.2126, .7152, .0722])
    v = y.ravel() if mask is None else y[np.asarray(mask, dtype=bool)]
    if not len(v):
        return None
    return dict(pixels=int(len(v)), mean=float(v.mean()), median=float(np.median(v)),
                p10=float(np.percentile(v, 10)), p50=float(np.percentile(v, 50)),
                p90=float(np.percentile(v, 90)), near_black_fraction=float(np.mean(v < 10)),
                saturated_fraction=float(np.mean(v >= 250)),
                histogram=np.histogram(v, bins=np.arange(257))[0].tolist(),
                unit='sRGB-code luma 0..255; diagnostic, no image transformation')


def lighting_gate(dark, bright, thresholds):
    checks = dict(
        mean_gain=bright['whole']['mean'] - dark['whole']['mean'] >= thresholds['minimum_mean_gain'],
        median_gain=bright['whole']['median'] - dark['whole']['median'] >= thresholds['minimum_median_gain'],
        saturation=bright['whole']['saturated_fraction'] <= thresholds['maximum_saturated_fraction'],
        cart_visible=bright['cart_pixels'] >= thresholds['minimum_instance_pixels'],
        cart_luminance=bright['cart'] is not None and bright['cart']['median'] >= thresholds['minimum_cart_median'],
        cart_not_saturated=bright['cart'] is not None and bright['cart']['saturated_fraction'] <= thresholds['maximum_crop_saturated_fraction'],
        target_preserved=dark['target_pixels'] < thresholds['minimum_instance_pixels'] or bright['target_pixels'] >= thresholds['minimum_instance_pixels'],
    )
    return dict(valid=all(checks.values()), checks=checks)


def trajectory_equal(a, b, xy_tolerance=1e-6, yaw_tolerance=1e-6):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return (a.shape == b.shape and bool(np.all(np.abs(a[:, :2]-b[:, :2]) <= xy_tolerance))
            and bool(np.all(np.abs(wrap_angle(a[:, 2]-b[:, 2])) <= yaw_tolerance)))


def lighting_classification(dark, bright, *, equivalent, material_clearance_gain_m=.02):
    if not dark['safe'] and bright['safe'] and bright['meaningful_response'] and not bright['stop']:
        return 'BRIGHT_RECOVERS_SAFE_RESPONSE'
    if equivalent:
        return 'DARK_AND_BRIGHT_BEHAVIOR_EQUIVALENT'
    if (not bright['safe'] and bright['clearance_m'] - dark['clearance_m'] >= material_clearance_gain_m):
        return 'BRIGHT_IMPROVES_CLEARANCE_BUT_UNSAFE'
    return 'BRIGHT_CHANGES_OUTPUT_WITHOUT_SAFETY_GAIN'


def history_suffix(frames, count, terminal_frame_id):
    idx = next(i for i, f in enumerate(frames) if f['frame_id'] == terminal_frame_id)
    if idx+1 < count:
        return None
    selected = frames[idx-count+1:idx+1]
    times = [f['capture_sim_time_s'] for f in selected]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('authentic history must be chronological, no cloned/future frames')
    if len({f['path'] for f in selected}) != count:
        raise ValueError('duplicate source capture path')
    return selected


def path_geometry(world, environment, *, required=.05):
    """Whole raw polyline plus node/segment diagnostics; no unsafe-row trimming."""
    world = np.asarray(world, dtype=np.float64)
    if world.ndim != 2 or world.shape[1] != 3 or len(world) == 0 or not np.isfinite(world).all():
        raise ValueError('finite nonempty Nx3 required')
    checks = environment.check_polyline(world)
    nodes = [environment.query(p[:2]) for p in world]
    margins = [float(q['clearance_m']) for q in nodes]
    segments = [environment.check_polyline(world[i:i+2]) for i in range(len(world)-1)]
    first_row = next((i for i, c in enumerate(margins) if c < required), None)
    first_segment = next((i for i, c in enumerate(segments) if not c['clearance_valid']), None)
    lengths = np.linalg.norm(np.diff(world[:, :2], axis=0), axis=1)
    # Safe-prefix distance is sampled along the first failing segment at <=1mm;
    # endpoint safety never certifies a segment. The authoritative result above
    # uses the independent swept checker, not this localization sampling.
    bracket = None
    if first_segment is not None:
        i = first_segment
        n = max(1, int(np.ceil(lengths[i]/.001)))
        for k in range(n+1):
            alpha = k/n
            q = environment.query(world[i, :2]*(1-alpha)+world[i+1, :2]*alpha)
            if q['clearance_m'] < required or q['status'] == 'UNKNOWN_WORKSPACE':
                bracket = dict(segment_index=i, alpha=[max(0, k-1)/n, alpha],
                    distance_from_first_row_m=[float(lengths[:i].sum()+lengths[i]*max(0,k-1)/n),
                                              float(lengths[:i].sum()+lengths[i]*alpha)])
                break
    return dict(whole=checks, row_count=len(world), node_clearance_m=margins,
                first_unsafe_waypoint_zero_based=first_row,
                first_unsafe_segment_zero_based=first_segment,
                safe_prefix_boundary=bracket, segment_checks=segments,
                total_arc_m=float(lengths.sum()), segment_lengths_m=lengths.tolist(),
                connector_included=False, row_timestamps=None)
