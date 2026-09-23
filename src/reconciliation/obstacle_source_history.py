"""Frozen H8 saved replay; original online history/clock semantics stay untouched."""
from copy import deepcopy
from .online_history import SessionHistory, validate_frame, _after


class FrozenHistory(SessionHistory):
    def __init__(self, instruction, contract, sampler, frames):
        super().__init__(instruction, contract, sampler)
        if len(frames) != 8:
            raise ValueError('frozen H8 required')
        self.frozen = deepcopy(frames)

    def begin(self, frame, *, predict, send_monotonic_ns, instruction=None):
        if self.failed or self.login_count != 1 or self.reset_count != 1 or self.pending is not None:
            raise ValueError('invalid independent session state')
        if type(predict) is not bool or instruction not in (None, self.instruction):
            raise ValueError('instruction/prediction mismatch')
        if type(send_monotonic_ns) is not int or send_monotonic_ns < 0:
            raise ValueError('actual current request clock required')
        frame = validate_frame(frame); index = len(self.frames)
        if index >= len(self.frozen) or predict != (index == len(self.frozen)-1):
            raise ValueError('seven buffers then one terminal request only')
        if frame['frame_id'] in self._seen:
            raise ValueError('duplicate frame')
        for k in ('frame_id','sha256','pose_world','capture_sim_time_s','capture_monotonic_ns','rendered_state_id'):
            if frame[k] != self.frozen[index][k]:
                raise ValueError('frozen replay mismatch: '+k)
        if self.frames:
            _after(self.frames[-1], frame)
        seq = self.next_seq; self.next_seq += 1
        frame['server_frame_index_reconstructed'] = index; frame['wire_seq'] = seq
        self.frames.append(frame); self._seen.add(frame['frame_id'])
        pending = dict(seq=seq,predict=predict,frame=frame,send_monotonic_ns=send_monotonic_ns,
                       prediction_index=self.prediction_count if predict else None)
        if predict:
            self.prediction_count += 1
        self.pending = pending
        return deepcopy(pending)

    def snapshot(self, server_actions_step=None):
        r = super().snapshot(server_actions_step)
        r['clock_semantics'] = dict(capture='unchanged historical render clock',
            request='actual current replay host clock', capture_to_request_age_s=None,
            cross_boot_subtraction_performed=False)
        return r
