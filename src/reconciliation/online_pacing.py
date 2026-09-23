"""Opt-in wall scheduler. Fixed simulation ticks never repay blocking-time debt."""
import math
from .robotless_online import AbsolutePacer

POLICY = 'minimum_wall_step_no_catchup_v1'


class NoCatchupPacer(AbsolutePacer):
    """Next completion >= last completion + simulated increment / target RTF.

    A slow render may lower wall throughput. It never changes simulation dt,
    scheduled capture/control tick IDs, commands, observation clocks or RTTs.
    No missed simulation ticks are skipped or synthesized.
    """
    def __post_init__(self):
        self.last_sim_s = self.origin_sim_s
        self.last_host_s = self.origin_host_s

    def deadline(self, sim_s):
        if self.target_rtf <= 0 or not math.isfinite(self.target_rtf):
            raise ValueError('positive target RTF required')
        if sim_s < self.last_sim_s:
            raise ValueError('simulation clock moved backwards')
        return self.last_host_s + (sim_s-self.last_sim_s)/self.target_rtf

    def remaining(self, sim_s, host_s):
        return self.deadline(sim_s)-host_s

    def observe(self, sim_s, host_s):
        result = super().observe(sim_s, host_s)
        result.update(pacing_policy=POLICY, intended_wall_deadline_s=self.deadline(sim_s),
                      wall_step_s=host_s-self.last_host_s,
                      simulation_step_s=sim_s-self.last_sim_s)
        if host_s < self.last_host_s:
            raise ValueError('host clock moved backwards')
        self.last_sim_s, self.last_host_s = sim_s, host_s
        return result

    def __init__(self, origin_host_s, origin_sim_s, target_rtf=1., tolerance_s=1/60):
        super().__init__(origin_host_s, origin_sim_s, target_rtf, tolerance_s)
        self.__post_init__()


def make_pacer(policy, origin_host, origin_sim, target_rtf, tolerance):
    if policy == 'absolute':
        return AbsolutePacer(origin_host, origin_sim, target_rtf, tolerance)
    if policy == POLICY:
        return NoCatchupPacer(origin_host, origin_sim, target_rtf, tolerance)
    raise ValueError('unknown pacing policy')
