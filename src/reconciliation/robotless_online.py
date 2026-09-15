"""Causal finite-rate execution and immutable online dataset primitives.

No renderer, model, controller optimization, or offline trajectory substitution.
All state transitions are driven by an explicitly applied command and actual dt.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np
from reconciliation.se2 import wrap_angle


STATUSES = (
    'VALID_HANDOFF_MOVING', 'VALID_HANDOFF_STATIONARY', 'MODEL_STOP',
    'MODEL_ERROR', 'NONFINITE_OUTPUT', 'PROTOCOL_ERROR', 'CONTROLLER_ERROR',
    'EXECUTOR_STALLED', 'REFERENCE_EXHAUSTED', 'SCENE_INVALID', 'TECHNICAL_INVALID',
)


def integrate_unicycle(pose, command, dt: float) -> np.ndarray:
    """Exact constant-command SE(2) exponential; metres, radians, seconds.

    sinc form is continuous through zero angular velocity. Rotation-only steps
    retain their time and yaw; no waypoint, projection, or pose snap is involved.
    """
    p, u = np.asarray(pose, dtype=float), np.asarray(command, dtype=float)
    if p.shape != (3,) or u.shape != (2,) or not np.isfinite(p).all() or not np.isfinite(u).all():
        raise ValueError('finite SE(2) pose and [v,omega] required')
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('positive finite integration dt required')
    angle = float(u[1] * dt)
    distance = float(u[0] * dt * np.sinc(angle / (2 * np.pi)))
    midpoint = float(p[2] + angle / 2)
    return np.array([p[0] + distance * math.cos(midpoint),
                     p[1] + distance * math.sin(midpoint), wrap_angle(p[2] + angle)])


def stamp(sim_time_s: float | None = None, episode_time_s: float | None = None) -> dict:
    ns = time.monotonic_ns()
    out = {'host_monotonic_ns': ns, 'host_monotonic_s': ns / 1e9,
           'host_utc': datetime.now(timezone.utc).isoformat()}
    if sim_time_s is not None:
        out['sim_time_s'] = float(sim_time_s)
    if episode_time_s is not None:
        out['episode_time_s'] = float(episode_time_s)
    return out


def file_record(path: Path, root: Path) -> dict:
    path = Path(path)
    return {'path': str(path.relative_to(root)), 'bytes': path.stat().st_size,
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def dump_new(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write('\n')


def raw_manifest(episode: Path) -> dict:
    """Immutable acquisition files, excluding later derived reports and plots."""
    names = ('rgb', 'chunks', 'requests', 'controller')
    paths = [p for name in names for p in (episode / name).rglob('*') if p.is_file()]
    paths += [episode / name for name in ('execution.csv', 'commands.csv', 'rgb_index.csv',
              'loop.jsonl', 'wire.jsonl', 'capture.jsonl', 'metadata.json', 'session_open.json', 'session_close.json', 'bootstrap.json', 'started.json') if (episode / name).is_file()]
    paths += list((episode / 'handoffs').glob('*/context.json'))
    return {'files': [file_record(p, episode) for p in sorted(set(paths))]}


def verify_resume(episode: Path) -> bool:
    """Completed episodes can only be skipped after verifying all raw hashes.

    A started/incomplete episode is never overwritten or silently rerun.
    """
    if not episode.exists():
        return False
    completion = episode / 'completion.json'
    if not completion.is_file():
        raise FileExistsError(f'episode already started without completion: {episode}')
    for entry in json.loads(completion.read_text())['raw_manifest']['files']:
        actual = file_record(episode / entry['path'], episode)
        if actual != entry:
            raise ValueError(f'completed raw file changed: {entry["path"]}')
    return True


@dataclass
class AbsolutePacer:
    origin_host_s: float
    origin_sim_s: float
    target_rtf: float = 1.0
    tolerance_s: float = 1 / 60
    missed_deadlines: int = 0
    maximum_lateness_s: float = 0.0

    def remaining(self, sim_s: float, host_s: float) -> float:
        if self.target_rtf <= 0 or not math.isfinite(self.target_rtf):
            raise ValueError('positive target RTF required')
        return self.origin_host_s + (sim_s - self.origin_sim_s) / self.target_rtf - host_s

    def observe(self, sim_s: float, host_s: float) -> dict:
        lag = max(0.0, -self.remaining(sim_s, host_s))
        missed = lag > self.tolerance_s
        self.missed_deadlines += int(missed)
        self.maximum_lateness_s = max(self.maximum_lateness_s, lag)
        elapsed = host_s - self.origin_host_s
        rtf = (sim_s - self.origin_sim_s) / elapsed if elapsed > 0 else None
        return {'deadline_missed': missed, 'lateness_s': lag, 'rtf': rtf}


class CommandActivation:
    """Install generations immediately; activate only on command application.

    Held OLD commands remain OLD until a current-generation solved command is
    actually applied. Numeric command equality cannot suppress a handoff.
    """
    def __init__(self, hold_timeout_s: float = 1.0):
        if hold_timeout_s <= 0 or not math.isfinite(hold_timeout_s):
            raise ValueError('positive command hold timeout required')
        self.hold_timeout_s = hold_timeout_s
        self.installed: dict | None = None
        self.active: dict | None = None
        self.latest: dict | None = None
        self.pending_result: dict | None = None
        self.last_result_sim_s: float | None = None
        self.activations: list[dict] = []
        self.rejected: list[dict] = []

    def install(self, chunk_id: str, version: int, context: dict) -> None:
        if self.installed is not None and version <= self.installed['reference_version']:
            raise ValueError('reference versions must strictly increase')
        self.installed = {'chunk_id': chunk_id, 'reference_version': version, 'context': context}
        self.pending_result = None

    def accept(self, result: dict, seen_sim_s: float) -> bool:
        if (self.installed is None or result.get('status') != 'command'
                or result.get('reference_version') != self.installed['reference_version']
                or result.get('chunk_id') != self.installed['chunk_id']):
            self.rejected.append(result)
            return False
        u = np.asarray(result['command'], dtype=float)
        if u.shape != (2,) or not np.isfinite(u).all():
            raise ValueError('invalid controller command')
        self.pending_result = dict(result)
        self.pending_result['seen_sim_time_s'] = seen_sim_s
        return True

    def apply(self, state: dict, previous_state: dict | None, previous_command: dict | None) -> tuple[dict, dict | None]:
        event = None
        held = self.pending_result is None
        if self.pending_result is not None:
            solved = self.pending_result
            self.pending_result = None
            switched = self.active is None or self.active['reference_version'] != solved['reference_version']
            if switched:
                event = {'old': self.active, 'fresh': self.installed,
                         'B': [state[k] for k in ('x','y','yaw')],
                         'P': None if previous_state is None else [previous_state[k] for k in ('x','y','yaw')],
                         'switch_state_id': state['state_id'],
                         'previous_state_id': None if previous_state is None else previous_state['state_id'],
                         't_switch': {k: state[k] for k in ('sim_time_s','episode_time_s','host_monotonic_ns','host_monotonic_s','host_utc')},
                         'pre_switch_command': previous_command,
                         'bootstrap': self.active is None}
                self.active = self.installed
                self.activations.append(event)
            self.latest = solved
            self.last_result_sim_s = state['sim_time_s']
        timeout = (self.latest is not None and state['sim_time_s'] - self.last_result_sim_s > self.hold_timeout_s)
        u = [0.0, 0.0] if self.latest is None or timeout else self.latest['command']
        cmd = {'v_mps': float(u[0]), 'omega_radps': float(u[1]),
               'reference_version': None if self.active is None else self.active['reference_version'],
               'chunk_id': None if self.active is None else self.active['chunk_id'],
               'solve_id': None if self.latest is None else self.latest['solve_id'],
               'held': held, 'reason': 'controller_timeout' if timeout else ('bootstrap' if self.latest is None else ('hold' if held else 'new_solve')),
               'application_state_id': state['state_id'], 'application_tick': state['tick'],
               **{k: state[k] for k in ('sim_time_s','episode_time_s','host_monotonic_ns','host_monotonic_s','host_utc')}}
        if event is not None:
            event['first_fresh_command'] = cmd
        return cmd, event


def without_automatic_physics(settings, operation):
    """Isaac render/timeline updates must not add uncommanded physics ticks.

    Mirrors SimulationContext.render's supported setting, restoring prior state
    even on failure. Direct World.step(render=False) remains outside this guard.
    """
    key='/app/player/playSimulations'
    previous=settings.get(key)
    settings.set_bool(key,False)
    try:
        return operation()
    finally:
        settings.set_bool(key,previous)
