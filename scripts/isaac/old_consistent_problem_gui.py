#!/usr/bin/env python3
"""Interactive saved OLD/FRESH geometry; default persists until the app closes."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
import numpy as np
import yaml
import old_fresh_problem_gui_artifacts as a
from reconciliation.old_fresh_problem_gui import CASE_IDS, ReplayState, arrow_segments, MOTION_S, REVEAL_S, FINISH_S
from reconciliation.robotless_single_chunk import save_json_exclusive, sha256_file, observation_to_world

COLORS = {'old': (0.10,.36,1.,1.), 'fresh': (1.,.08,.75,1.), 'b': (1.,.90,.05,1.),
          'q': (1.,1.,1.,1.), 'connector': (.15,1.,.35,1.), 'old_local': (.35,1.,.08,1.),
          'fresh_local': (0.,.95,1.,1.), 'fresh_yaw': (1.,.40,.04,1.),
          'b_yaw': (.95,.70,.28,1.), 'old_window': (.72,1.,.58,1.), 'fresh_window': (.5,1.,1.,1.)}
TITLE = 'OLD / FRESH - saved handoff geometry'


def timestamp():
    return {'utc': datetime.now(timezone.utc).isoformat(), 'host_unix_ns': time.time_ns(),
            'host_monotonic_ns': time.monotonic_ns(), 'display_playback_only': True,
            'simulation_time_s': 0., 'execution_time': None}


def config_check(config):
    if config['schema_version'] != 1 or (ROOT/config['source_run']).resolve() != a.SOURCE.resolve():
        raise ValueError('only the frozen primary source is supported')
    if config['episode_ids'] != list(CASE_IDS) or config['old_arc_advance_m'] != .30 or config['tangent_window_m'] != .10:
        raise ValueError('fixed GUI spatial convention changed')
    if config['display_timeline_s'] != {'old_motion_end': MOTION_S, 'fresh_reveal': REVEAL_S, 'comparison_end': FINISH_S}:
        raise ValueError('display timing configuration differs from tested state machine')
    if config['display_playback_only'] is not True or config['display_geometry']['xy_scale'] != 1 or config['display_geometry']['angular_magnification'] != 1:
        raise ValueError('only unscaled display-only geometry is allowed')


class Demo:
    def __init__(self, args, app, output, source_record, cases, records, config):
        import omni.ui as ui
        from isaacsim.util.debug_draw import _debug_draw
        from robotless_runtime import runtime_scene, git_output
        self.args, self.app, self.output, self.cases, self.records, self.config = args, app, output, cases, records, config
        self.source = a.SOURCE.resolve()
        self.source_record = source_record
        self.sha = git_output(ROOT, 'rev-parse', 'HEAD')
        self.state = ReplayState(case_id=args.case, show_rgb=args.show_rgb)
        self.draw = _debug_draw.acquire_debug_draw_interface()
        self.updates, self.events, self.samples = 0, [], []
        self.control_checks, self.last_clock = [], time.monotonic()
        scene_config = yaml.safe_load((self.source/'config_snapshot.yaml').read_text())
        self.scene_config = scene_config
        self.stage, self.agent, _, _, self.scene = runtime_scene(scene_config, self.case.b, app=app)
        self.make_panel(ui)
        self.set_camera()
        self.refresh()
        self.event('GUI_READY')

    @property
    def case(self):
        return self.cases[self.state.case_id]

    def event(self, action):
        record = {'action': action, 'state': asdict(self.state), 'time': timestamp()}
        self.events.append(record)
        with (self.output/'events.jsonl').open('a') as stream:
            stream.write(json.dumps(record)+'\n')

    def action(self, action, case_id=None):
        if action == 'case': self.state.select(case_id)
        elif action == 'Replay OLD': self.state.replay()
        elif action == 'Pause at Handoff': self.state.handoff()
        elif action == 'Reveal FRESH': self.state.reveal()
        elif action == 'Replay Full': self.state.replay(full=True)
        elif action == 'Reset': self.state.reset()
        elif action == 'Show RGB1': self.state.show_rgb = not self.state.show_rgb
        elif action == 'Toggle Local / Window Tangent': self.state.show_window = not self.state.show_window
        elif action == 'Overview Camera': self.state.camera = 'overview'
        elif action == 'Handoff Camera': self.state.camera = 'handoff'
        else: raise ValueError(action)
        if action == 'case' or action.endswith('Camera'): self.set_camera()
        self.last_clock = time.monotonic()
        self.refresh()
        self.event(action)

    def make_panel(self, ui):
        import carb
        # Keep the complete control/metric/RGB panel visible at the recorded window size.
        carb.settings.get_settings().set_float('/app/window/dpiScaleOverride', 1.)
        for _ in range(4): self.app.update()
        self.panel = ui.Window(TITLE, width=600, height=1180, position_x=10, position_y=45)
        with self.panel.frame:
            with ui.VStack(spacing=4):
                ui.Label('OLD / FRESH HANDOFF', height=34, style={'font_size': 27})
                ui.Label('OLD-CONSISTENT SPATIAL SURROGATE REPLAY', height=24, style={'font_size': 17})
                with ui.HStack(height=40, spacing=5):
                    for case_id, title in zip(CASE_IDS, ('006 Challenging', '016 Challenging', '027 Benign Reference')):
                        ui.Button(title, clicked_fn=lambda c=case_id: self.action('case', c))
                self.case_label = ui.Label('', height=27, style={'font_size': 21})
                self.title_label = ui.Label('', height=25, style={'font_size': 19})
                self.phase_label = ui.Label('', height=28, style={'font_size': 20, 'color': 0xFF66DFFF})
                with ui.HStack(height=36, spacing=5):
                    for title in ('Replay OLD', 'Pause at Handoff', 'Reveal FRESH'):
                        ui.Button(title, clicked_fn=lambda t=title: self.action(t))
                with ui.HStack(height=36, spacing=5):
                    for title in ('Replay Full', 'Reset', 'Show RGB1'):
                        ui.Button(title, clicked_fn=lambda t=title: self.action(t))
                with ui.HStack(height=36, spacing=5):
                    for title in ('Overview Camera', 'Handoff Camera'):
                        ui.Button(title, clicked_fn=lambda t=title: self.action(t))
                ui.Button('Toggle Local / Window Tangent', height=32,
                          clicked_fn=lambda: self.action('Toggle Local / Window Tangent'))
                self.playback_label = ui.Label('', height=22)
                self.camera_label = ui.Label('', height=22)
                ui.Separator(height=3)
                ui.Label('SAVED METRICS  |  primary tau = 0', height=24, style={'font_size': 18})
                self.metric_label = ui.Label('', height=162, style={'font_size': 19})
                self.explanation = ui.Label('', height=56, word_wrap=True, style={'font_size': 17})
                ui.Separator(height=3)
                ui.Label('BLUE  OLD path     MAGENTA  raw FRESH', height=22, style={'font_size': 17})
                ui.Label('YELLOW  B marker     WHITE  Q     GREEN  B to Q', height=22)
                ui.Label('LIME  OLD incoming local tangent (at B)', height=22, style={'color': 0xFF33FFAA})
                ui.Label('CYAN  FRESH local tangent (at Q)', height=22, style={'color': 0xFFFFFF55})
                ui.Label('ORANGE  FRESH pose yaw (at Q)', height=22, style={'color': 0xFF3399FF})
                ui.Label('GOLD short arrow  B pose yaw; separate from tangent', height=22)
                self.window_label = ui.Label('', height=38, word_wrap=True)
                self.rgb_box = ui.VStack(height=238, spacing=4)
                with self.rgb_box:
                    ui.Label('FRESH OBSERVATION  |  saved RGB1', height=22, style={'font_size': 18})
                    self.rgb = ui.Image('', height=202, fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                ui.Label('3 s OLD motion / 1 s pause / 4 s comparison: display only.', height=22)
                ui.Label('Saved trajectory geometry. No robot failure or graph improvement.', height=30, word_wrap=True)
        for _ in range(4): self.app.update()
        stage = ui.Workspace.get_window('Stage')
        if stage: self.panel.dock_in(stage, ui.DockPosition.SAME)
        for name in ('Property', 'Content', 'Console', 'Stage', 'Render Settings'):
            window = ui.Workspace.get_window(name)
            if window: window.visible = False
        for _ in range(4): self.app.update()
        ui.Workspace.set_dock_id_width(self.panel.dock_id, 660)
        self.panel.focus()

    def set_camera(self):
        from isaacsim.core.utils.viewports import set_camera_view
        from robotless_old_consistent_observation import set_precise_agent_pose
        g = self.config['display_geometry']
        set_precise_agent_pose(self.agent, self.case.b, z_m=self.scene_config['agent']['z_m'])
        if self.state.camera == 'overview':
            offsets = np.array([g['overview_eye_R0_frame_m'], g['overview_target_R0_frame_m']])
            xy = observation_to_world(self.case.r0, np.column_stack([offsets[:, :2], np.zeros(2)]))
            eye, target = [*xy[0, :2], offsets[0, 2]], [*xy[1, :2], offsets[1, 2]]
        else:
            # Nearly normal to the fixed XY plane: preserve visual angle and fit the annotation arrows.
            b = self.case.b
            eye = [b[0], b[1]-.001, g['handoff_eye_above_B_m']]
            target = [b[0], b[1], g['handoff_target_z_m']]
        self.camera_record = {'mode': self.state.camera, 'eye_world_m': eye, 'target_world_m': target,
                              'path_coordinates_modified': False}
        set_camera_view(eye=eye, target=target, camera_prim_path='/OmniverseKit_Persp')

    def refresh(self):
        s, c = self.state, self.case
        m = c.saved_metrics
        self.case_label.text = c.case_id
        self.title_label.text = 'BENIGN DIRECTION / YAW REFERENCE' if c.case_id == 'episode_027' else 'CHALLENGING EXAMPLE'
        self.phase_label.text = {'READY': 'A  |  OLD only - ready', 'OLD_MOTION': 'A  |  OLD spatial replay',
                                 'HANDOFF_PAUSE': 'B  |  Handoff paused - FRESH hidden',
                                 'FRESH_REVEALED': 'C  |  Saved raw FRESH revealed'}[s.phase]
        self.playback_label.text = f'Display clock {s.elapsed_s:.2f} s  |  arc {s.progress_m:.3f} / 0.300 m'
        self.camera_label.text = f'Camera: {s.camera.upper()}  |  XY scale 1  /  angle scale 1'
        self.metric_label.text = (f'Cross-track:  {m["e_perp_m"]:.9f} m\n'
            f'OLD / FRESH local direction:  {m["abs_e_dir_local_deg"]:.5f} deg\n'
            f'0.10 m window direction:  {m["abs_e_dir_window_deg"]:.5f} deg\n'
            f'Boundary / FRESH pose yaw:  {m["abs_e_yaw_deg"]:.5f} deg\n'
            f'FRESH projection segment:  {m["fresh_projection"]["segment_index"]}\n'
            f'FRESH alpha:  {m["fresh_projection"]["alpha"]:.9f}')
        self.explanation.text = ('Directions / yaw nearly agree. The first FRESH spatial reference starts ahead of its observation anchor.'
            if c.case_id == 'episode_027' else 'Small positional separation does not imply directional continuity.')
        self.window_label.text = ('WINDOW overlay ON: pale green OLD / pale cyan FRESH, 0.10 m chord tangents; longer diagnostic arrows.'
            if s.show_window else 'LOCAL arrows shown. Window toggle adds separate, longer pale diagnostic arrows.')
        self.rgb_box.visible = s.show_rgb
        rgb_url = f'file:{self.records[c.case_id]["RGB1_path"]}'
        if self.rgb.source_url != rgb_url: self.rgb.source_url = rgb_url
        self.draw_geometry()

    def draw_geometry(self):
        s, c, draw, g = self.state, self.case, self.draw, self.config['display_geometry']
        m, z = c.reconstructed, g['annotation_z_m']
        draw.clear_lines(); draw.clear_points()
        def lines(pairs, color, width):
            draw.draw_lines([p[0] for p in pairs], [p[1] for p in pairs], [COLORS[color]]*len(pairs), [width]*len(pairs))
        def path(array, color):
            p = [[float(x), float(y), g['path_z_m']] for x,y in array[:, :2]]
            if len(p)>1: lines(list(zip(p[:-1],p[1:])), color, 4.)
        def point(xy, color, size):
            draw.draw_points([(float(xy[0]),float(xy[1]),z)], [COLORS[color]], [size])
        def arrow(xy, phi, color, length, width=4.):
            lines(arrow_segments(xy,phi,length,z),color,width)
        path(c.augmented, 'old')
        marker = c.marker(s.progress_m)
        point(marker, 'b', 16.)
        if s.progress_m == .30:
            arrow(c.b, m['phi_old_local_rad'], 'old_local', g['local_arrow_length_m'])
            # Pose yaw is explicitly distinct from geometric incoming direction.
            if s.revealed: arrow(c.b, c.b[2], 'b_yaw', g['boundary_yaw_arrow_length_m'], 2.)
            if s.show_window: arrow(c.b, m['phi_old_window_rad'], 'old_window', g['window_arrow_length_m'], 2.)
        if s.revealed:
            q = m['fresh_projection']['Q_xy_world_m']
            path(c.fresh, 'fresh')
            lines([([*c.b[:2],z],[*q,z])], 'connector', 3.)
            point(q, 'q', 9.)
            point(c.b, 'b', 6.)
            arrow(q, m['phi_fresh_local_rad'], 'fresh_local', g['local_arrow_length_m'], 6.)
            arrow(q, m['fresh_projection']['theta_Q_rad'], 'fresh_yaw', g['local_arrow_length_m'], 3.)
            if s.show_window: arrow(q, m['phi_fresh_window_rad'], 'fresh_window', g['window_arrow_length_m'], 2.)

    def tick(self):
        now = time.monotonic()
        previous = self.state.phase
        self.state.advance(now-self.last_clock)
        self.last_clock = now
        self.refresh()
        self.app.update(); self.updates += 1
        if self.state.playing or previous != self.state.phase:
            self.samples.append({'state': asdict(self.state), 'marker_world': self.case.marker(self.state.progress_m).tolist(),
                                 'time': timestamp()})
        if previous != self.state.phase: self.event('automatic phase transition')

    def settle(self, seconds=.8):
        end = time.monotonic()+seconds
        while self.app.is_running() and time.monotonic()<end:
            self.refresh(); self.app.update(); self.updates += 1
        self.last_clock = time.monotonic()

    def capture(self, suffix):
        import omni.kit.renderer.capture as capture
        from PIL import Image
        self.settle(1.0)
        path = self.output/f'evidence/{self.state.case_id}_{suffix}.png'
        if path.exists(): raise FileExistsError(path)
        interface = capture.acquire_renderer_capture_interface()
        interface.capture_next_frame_swapchain(str(path))
        deadline = time.monotonic()+25
        while time.monotonic()<deadline:
            self.app.update(); self.updates += 1
            if path.is_file() and path.stat().st_size:
                try:
                    with Image.open(path) as im:
                        im.load()
                        if np.asarray(im).std() < 1: raise ValueError('blank application screenshot')
                    break
                except OSError: pass
        else: raise RuntimeError('application screenshot unavailable')
        c = self.case
        save_json_exclusive(path.with_suffix('.json'), {
            'source_pilot_run': str(self.source), 'episode_id': c.case_id,
            'source_OLD_hash': self.records[c.case_id]['source_hashes']['raw/chunk_000.npy'],
            'source_FRESH_hash': self.records[c.case_id]['source_hashes']['raw/chunk_001.npy'],
            'metric_JSON_hash': self.records[c.case_id]['source_hashes']['derived/metrics.json'],
            'image_sha256': sha256_file(path), 'camera_mode': self.state.camera, 'camera': self.camera_record,
            'reveal_state': self.state.revealed, 'state': asdict(self.state), 'capture_time': timestamp(),
            'research_git_sha': self.sha, 'geometry': a.case_audit(c),
            'display_playback_only': True, 'capture_kind': 'Isaac application swapchain including interactive UI',
            'ui_dpi_scale': 1.,
            'geometry_scaling': 1., 'angular_magnification': 1.})
        self.last_clock = time.monotonic()

    def exercise(self):
        """Exercise the same callbacks as the real buttons in the running renderer."""
        for case_id in CASE_IDS:
            self.action('case', case_id)
            assert self.state.progress_m == 0 and not self.state.revealed
            self.action('Handoff Camera')
            self.action('Replay OLD')
            while self.state.playing and self.app.is_running(): self.tick()
            assert self.state.phase == 'HANDOFF_PAUSE' and np.array_equal(self.case.marker(self.state.progress_m), self.case.b)
            self.capture('before_fresh')
            self.action('Reveal FRESH')
            assert self.state.revealed
            self.capture('after_fresh')
            self.action('Toggle Local / Window Tangent')
            assert self.state.show_window
            self.capture('window_tangents')
            self.action('Toggle Local / Window Tangent')
            self.action('Overview Camera')
            self.capture('overview')
            self.action('Show RGB1'); self.action('Show RGB1')
            self.action('Reset')
            assert self.state.progress_m == 0 and not self.state.revealed and not self.state.show_window
            self.action('Pause at Handoff')
            assert self.state.progress_m == .30 and not self.state.revealed
            self.action('Replay Full')
            start = len(self.events)
            while self.state.playing and self.app.is_running(): self.tick()
            phases = [e['state']['phase'] for e in self.events[start:]]
            assert phases == ['HANDOFF_PAUSE', 'FRESH_REVEALED'], phases
            assert self.state.revealed and self.state.elapsed_s == FINISH_S
            self.control_checks.append({'episode_id': case_id, 'case_reset': True, 'replay_old_endpoint_exact': True,
                'pause': True, 'reveal': True, 'reset': True, 'full_replay_phases': phases,
                'camera_modes': ['handoff', 'overview'], 'window_toggle': True, 'rgb_toggle': True})
        save_json_exclusive(self.output/'runtime_controls.json', {'checks': self.control_checks,
                            'method': 'same button callbacks exercised inside actual Isaac GUI/render loop', 'samples': self.samples})
        self.action('case', self.args.case)
        self.action('Handoff Camera')
        if self.args.auto_replay: self.action('Replay Full')

    def run(self):
        if self.args.capture_evidence: self.exercise()
        elif self.args.auto_replay: self.action('Replay Full')
        started, initial_updates = time.monotonic(), self.updates
        save_json_exclusive(self.output/'runtime_started.json', {'pid': os.getpid(), 'time': timestamp(),
            'no_hold': self.args.no_hold, 'persistent_default': True, 'scene': self.scene,
            'scene_visibility_modifications': [], 'scene_load_count': 1, 'new_lightnav_inference_count': 0,
            'display_playback_only': True, 'dynamics_advanced': False, 'execution_time': None,
            'colors': COLORS, 'display_geometry': self.config['display_geometry']})
        print(f'OLD_FRESH_GUI_READY {self.output}', flush=True)
        persistence_written = False
        while self.app.is_running():
            self.tick()
            if not persistence_written and time.monotonic()-started >= 12:
                a.verify_hashes(self.source, self.source_record['files'])
                from robotless_runtime import stopped_time
                stopped_time()
                save_json_exclusive(self.output/'persistence.json', {'time': timestamp(), 'app_is_running': self.app.is_running(),
                    'elapsed_after_ready_s': time.monotonic()-started, 'render_updates_after_ready': self.updates-initial_updates,
                    'no_hold': self.args.no_hold, 'state': asdict(self.state), 'source_files_unchanged': True})
                persistence_written = True
                print('OLD_FRESH_GUI_PERSISTENCE_OBSERVED', flush=True)
            if self.args.no_hold and persistence_written and not self.state.playing: break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASE_IDS, default=CASE_IDS[0])
    parser.add_argument('--no-hold', action='store_true', help='exit after replay and 12-second runtime observation')
    parser.add_argument('--auto-replay', action='store_true')
    parser.add_argument('--show-rgb', action='store_true')
    parser.add_argument('--capture-evidence', action='store_true', help='exercise all cases and save application screenshots')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    config = yaml.safe_load((ROOT/'configs/robotless_old_fresh_problem_gui.yaml').read_text())
    config_check(config)
    output = args.output or ROOT/'data/robotless_old_consistent_problem_gui'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = a.guard_output(output, a.SOURCE)
    # Use the repository's tested validator environment before creating any Isaac stage.
    checked = subprocess.run([str(ROOT/'.venv/bin/python'), str(ROOT/'scripts/old_fresh_problem_gui_artifacts.py'), str(a.SOURCE)],
                             check=True, text=True, capture_output=True)
    source_record = json.loads(checked.stdout)
    cases, records = a.load_cases(a.SOURCE)
    output.mkdir(parents=True)
    (output/'evidence').mkdir()
    source_record.update(source_pilot_run=str(a.SOURCE), case_records=records, created_time=timestamp())
    save_json_exclusive(output/'source.json', source_record)
    with (output/'config_snapshot.yaml').open('x') as stream: yaml.safe_dump(config, stream, sort_keys=False)
    processing = ['scripts/isaac/old_consistent_problem_gui.py', 'scripts/isaac/run_old_consistent_problem_gui.sh',
        'scripts/old_fresh_problem_gui_artifacts.py', 'src/reconciliation/old_fresh_problem_gui.py',
        'configs/robotless_old_fresh_problem_gui.yaml', 'scripts/validate_old_fresh_problem_gui.py', 'scripts/isaac/robotless_runtime.py',
        'scripts/isaac/robotless_old_consistent_observation.py', 'src/reconciliation/robotless_old_conditioned_handoff.py',
        'src/reconciliation/robotless_projection_handoff.py', 'src/reconciliation/robotless_single_chunk.py', 'src/reconciliation/se2.py']
    save_json_exclusive(output/'processing_sources.json', {p: sha256_file(ROOT/p) for p in processing})
    save_json_exclusive(output/'geometry_audit.json', {k: a.case_audit(c) for k,c in cases.items()})
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': False, 'window_width': 2400, 'window_height': 1320, 'width': 1600, 'height': 1100})
    try:
        Demo(args, app, output, source_record, cases, records, config).run()
    finally:
        a.verify_hashes(a.SOURCE, source_record['files'])
        app.close()


if __name__ == '__main__':
    main()
