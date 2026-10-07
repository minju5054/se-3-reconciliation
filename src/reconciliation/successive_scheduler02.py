"""Saved-only capture-phase audit; no change to collection or old frozen audits."""

def capture_phase_audit(rows, frames):
    expected = []
    exemptions = []
    for i, row in enumerate(rows):
        terminal = [m for m in row['model_received'] if m.get('type') == 'result'
                    and m.get('kind') == 'prediction'
                    and m.get('status') in ('SCENE_INVALID', 'RAW_UNSAFE', 'MODEL_STOP')]
        before_capture = (i == len(rows)-1 and row['sim_steps'] == 0
            and row['inference_in_flight_at_start'] and terminal
            and not row['capture_happened'] and row['render_readback_s'] is None
            and not row['mpc_submitted'] and row['wire_request_kind'] is None)
        if before_capture:
            exemptions.append(dict(state_id=row['start_state_id'], terminal_results=terminal))
        elif row['start_state_id'] % 15 == 0:
            expected.append(row['start_state_id'])
    actual = [f['rendered_state_id'] for f in frames]
    by_state = {r['start_state_id']:r for r in rows}
    exact = len(by_state) == len(rows) and all(
        f['rendered_state_id'] in by_state
        and by_state[f['rendered_state_id']]['capture_happened']
        and f['capture_sim_time_s'] == by_state[f['rendered_state_id']]['simulation_start_s']
        and f['render_sim_time_before_s'] == f['capture_sim_time_s'] == f['render_sim_time_after_s']
        for f in frames)
    flags = [r['start_state_id'] for r in rows if r['capture_happened']]
    return dict(valid=actual == expected == flags and exact, expected_capture_ticks=expected,
        capture_ticks=actual, exact_capture_state_and_time=exact,
        before_capture_terminal_exemptions=exemptions,
        rule='Scheduled capture required only when the loop reaches capture phase; final in-flight terminal zero-step abort before capture is exempt.')
