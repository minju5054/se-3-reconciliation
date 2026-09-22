"""Saved-replay history across host boots; model input and capture clocks unchanged.

The original online history correctly requires capture/send on one host clock.
This offline adapter instead authenticates each frozen pre-existing frame. It
never compares a historical boot's monotonic time with a new boot's send time.
It preserves original capture values and real current request/RTT clocks.
"""
from copy import deepcopy

from .online_history import SessionHistory, validate_frame, _after


class SavedReplayHistory(SessionHistory):
    def __init__(self, instruction, contract, sampler, frozen_frames):
        super().__init__(instruction, contract, sampler)
        if len(frozen_frames) != 16:
            raise ValueError('exact frozen H16 required')
        self.frozen_frames = deepcopy(frozen_frames)

    def begin(self, frame, *, predict, send_monotonic_ns, instruction=None):
        # Preserve the original state machine and chronological checks; the only
        # exception is the inapplicable cross-boot capture <= send comparison.
        if self.failed: raise ValueError('failed session cannot be reused')
        if self.login_count != 1 or self.reset_count != 1:
            raise ValueError('login and one initial reset required')
        if self.pending is not None: raise ValueError('maximum one outstanding request')
        if type(predict) is not bool: raise ValueError('predict must be boolean')
        if instruction is not None and instruction != self.instruction:
            raise ValueError('instruction cannot change inside session')
        frame = validate_frame(frame)
        if frame['frame_id'] in self._seen: raise ValueError('duplicate frame')
        if type(send_monotonic_ns) is not int or send_monotonic_ns < 0:
            raise ValueError('real current host monotonic required')
        index = len(self.frames)
        if index >= 16: raise ValueError('undeclared extra frame')
        expected = self.frozen_frames[index]
        for key in ('frame_id','sha256','pose_world','capture_sim_time_s','capture_monotonic_ns','rendered_state_id'):
            if frame[key] != expected[key]: raise ValueError('frozen replay mismatch: '+key)
        if predict != (index == 15): raise ValueError('fifteen empty buffers then one prediction')
        if self.frames: _after(self.frames[-1], frame)
        seq = self.next_seq; self.next_seq += 1
        frame['server_frame_index_reconstructed'] = len(self.frames)
        frame['wire_seq'] = seq
        self.frames.append(frame); self._seen.add(frame['frame_id'])
        pending = dict(seq=seq,predict=predict,frame=frame,send_monotonic_ns=send_monotonic_ns,
                       prediction_index=self.prediction_count if predict else None)
        if predict: self.prediction_count += 1
        self.pending = pending
        return deepcopy(pending)

    def snapshot(self, server_actions_step=None):
        result = super().snapshot(server_actions_step)
        result['clock_semantics'] = dict(capture='original SOURCE04 host clock; values unchanged',
            request='current replay host clock; actual monotonic send/receipt',
            capture_to_request_age_s=None, cross_boot_subtraction_performed=False,
            model_positions='unchanged official delivered indices; capture monotonic not transmitted')
        return result
