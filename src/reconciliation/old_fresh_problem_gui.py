"""Saved-only GUI geometry and display-time state; no simulator dependencies."""
from dataclasses import dataclass
import copy

import numpy as np

from reconciliation.robotless_old_conditioned_handoff import interpolate_arc, window_tangent
from reconciliation.robotless_projection_handoff import projection_geometry
from reconciliation.robotless_single_chunk import observation_to_world, validate_waypoints
from reconciliation.se2 import wrap_angle

CASE_IDS = ('episode_006', 'episode_016', 'episode_027')
ADVANCE_M = .30
WINDOW_M = .10
MOTION_S, REVEAL_S, FINISH_S = 3., 4., 8.
METRIC_KEYS = ('e_perp_m', 'abs_e_dir_local_deg', 'abs_e_dir_window_deg', 'abs_e_yaw_deg')


def allowed_case(case_id):
    if case_id not in CASE_IDS:
        raise ValueError(f'unsupported GUI case: {case_id}')
    return case_id


def agree(actual, expected, label='metric', atol=1e-10):
    """Compare reconstructed fields, including nested diagnostic definitions."""
    if isinstance(actual, dict):
        for key, value in actual.items():
            if key not in expected:
                raise ValueError(f'{label}.{key} missing')
            agree(value, expected[key], f'{label}.{key}', atol)
    elif actual is None or isinstance(actual, (str, bool)):
        if actual != expected:
            raise ValueError(f'{label} mismatch')
    elif not np.allclose(actual, expected, rtol=0, atol=atol, equal_nan=False):
        raise ValueError(f'{label} mismatch: {actual} != {expected}')


def readonly(value):
    array = np.array(value, dtype=np.float64, copy=True)
    if not np.all(np.isfinite(array)):
        raise ValueError('nonfinite geometry')
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class CaseGeometry:
    case_id: str
    r0: np.ndarray
    old: np.ndarray
    fresh: np.ndarray
    augmented: np.ndarray
    b: np.ndarray
    reconstructed: dict
    saved_metrics: dict

    def marker(self, progress_m):
        if not np.isfinite(progress_m) or not 0 <= progress_m <= ADVANCE_M:
            raise ValueError('display progress outside [0, 0.30] m')
        # Retain the exact initial pose even when the connector has zero XY arc.
        if progress_m == 0:
            return self.r0.copy()
        if progress_m == ADVANCE_M:
            return self.b.copy()
        return np.array(interpolate_arc(self.augmented, progress_m)['pose'])


def reconstruct(case_id, metadata, raw_old, raw_fresh, old_world, fresh_world, saved):
    """Build geometry only from source arrays/observation metadata, then audit metrics.

    Local arrays: +X forward, +Y left, yaw CCW radians. World: fixed Isaac XY,
    +Z up, metres. T_world_observation uses each actual saved observation pose.
    The loaded world arrays are displayed unchanged; no B re-anchoring occurs.
    """
    allowed_case(case_id)
    if metadata['episode_id'] != case_id or saved['episode_id'] != case_id or saved['status'] != 'VALID_PAIR':
        raise ValueError('source case/status mismatch')
    old, fresh = validate_waypoints(old_world), validate_waypoints(fresh_world)
    agree(old, observation_to_world(metadata['R0'], raw_old), 'OLD observation transform')
    agree(fresh, observation_to_world(metadata['R1'], raw_fresh), 'FRESH observation transform')
    r0 = np.array(metadata['R0'], dtype=float)
    augmented = np.vstack([r0, old])
    sample = interpolate_arc(augmented, ADVANCE_M)
    b = np.array(sample['pose'])
    # B is independently computed before looking at any saved geometry list.
    if not np.array_equal(b, metadata['planned_R1_old']) or not np.array_equal(b, saved['plan']['R1_old']):
        raise ValueError('reconstructed B differs from saved R1_old')
    agree(b, metadata['R1'], 'actual observation readback', atol=1e-7)
    p = projection_geometry(b, fresh)
    ow = window_tangent(augmented, ADVANCE_M, WINDOW_M)
    fw = window_tangent(fresh, p['s_Q_m'], WINDOW_M)
    if any(v is None for v in (sample['phi_local_rad'], p['phi_F_rad'], ow['phi_rad'], fw['phi_rad'])):
        raise ValueError('selected case requires available local and window tangents')
    local = wrap_angle(p['phi_F_rad'] - sample['phi_local_rad'])
    window = wrap_angle(fw['phi_rad'] - ow['phi_rad'])
    rebuilt = {
        'primary_tau_s': 0., 'B_world': b.tolist(), 'fresh_projection': p,
        'old_window': ow, 'fresh_window': fw,
        'phi_old_local_rad': sample['phi_local_rad'], 'phi_fresh_local_rad': p['phi_F_rad'],
        'phi_old_window_rad': ow['phi_rad'], 'phi_fresh_window_rad': fw['phi_rad'],
        'e_perp_m': p['e_perp_m'], 'e_dir_local_rad': local, 'e_dir_window_rad': window,
        'e_yaw_rad': p['e_yaw_rad'], 'abs_e_dir_local_deg': float(np.degrees(abs(local))),
        'abs_e_dir_window_deg': float(np.degrees(abs(window))), 'abs_e_yaw_deg': p['abs_e_yaw_deg'],
    }
    agree(sample, saved['plan']['sample'], 'OLD arc sample')
    agree(rebuilt, saved['metrics'], 'saved metric')
    return CaseGeometry(case_id, readonly(r0), readonly(old), readonly(fresh), readonly(augmented),
                        readonly(b), rebuilt, copy.deepcopy(saved['metrics']))


def arrow_segments(xy, phi, length_m=.24, z_m=.15):
    """True XY heading; fixed annotation length is not trajectory scaling."""
    if not np.isfinite(phi) or not np.isfinite(length_m) or length_m <= 0:
        raise ValueError('finite heading and positive annotation length required')
    start = np.array([*xy[:2], z_m], dtype=float)
    end = start + length_m*np.array([np.cos(phi), np.sin(phi), 0.])
    tips = [end + .22*length_m*np.array([np.cos(phi+a), np.sin(phi+a), 0.])
            for a in (2.6, -2.6)]
    return [(start.tolist(), end.tolist()), (end.tolist(), tips[0].tolist()), (end.tolist(), tips[1].tolist())]


@dataclass
class ReplayState:
    case_id: str = CASE_IDS[0]
    phase: str = 'READY'
    progress_m: float = 0.
    elapsed_s: float = 0.
    playing: bool = False
    full: bool = False
    revealed: bool = False
    camera: str = 'handoff'
    show_rgb: bool = False
    show_window: bool = False

    def __post_init__(self):
        allowed_case(self.case_id)

    def reset(self):
        self.phase, self.progress_m, self.elapsed_s = 'READY', 0., 0.
        self.playing = self.full = self.revealed = self.show_window = False

    def select(self, case_id):
        self.case_id = allowed_case(case_id)
        self.reset()

    def replay(self, full=False):
        self.reset()
        self.playing, self.full, self.phase = True, bool(full), 'OLD_MOTION'

    def handoff(self):
        self.progress_m, self.phase, self.playing, self.revealed = ADVANCE_M, 'HANDOFF_PAUSE', False, False
        self.elapsed_s = MOTION_S

    def reveal(self):
        self.progress_m, self.phase, self.playing, self.revealed = ADVANCE_M, 'FRESH_REVEALED', False, True
        self.elapsed_s = max(self.elapsed_s, REVEAL_S)

    def advance(self, dt):
        if not np.isfinite(dt) or dt < 0:
            raise ValueError('display-clock delta must be finite and nonnegative')
        if not self.playing:
            return
        self.elapsed_s = min(FINISH_S, self.elapsed_s + dt)
        self.progress_m = ADVANCE_M * min(1., self.elapsed_s / MOTION_S)
        if self.elapsed_s < MOTION_S:
            self.phase = 'OLD_MOTION'
        elif not self.full:
            self.handoff()
        elif self.elapsed_s < REVEAL_S:
            self.phase = 'HANDOFF_PAUSE'
        else:
            self.phase, self.revealed = 'FRESH_REVEALED', True
            self.playing = self.elapsed_s < FINISH_S
