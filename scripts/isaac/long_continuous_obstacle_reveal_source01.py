#!/usr/bin/env python3
"""Reuse native collector, changing only terminal budget and pre-solve bound loader."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'scripts/isaac'), str(ROOT/'scripts'), str(ROOT/'src')]
import continuous_obstacle_reveal_episode01 as collector
from continuous_obstacle_reveal_exploratory02 import ExploratoryGeometry
from reconciliation.long_continuous_obstacle_reveal_source01 import RequestPolicy
from reconciliation.join_source03 import save


def worker_argv(argv, run):
    result = list(argv)
    if result[1] == str(ROOT/'scripts/online_mpc_worker.py'):
        result[1] = str(ROOT/'scripts/long_source_mpc_worker01.py')
        result += ['--run', str(run)]
    elif result[1] != str(ROOT/'scripts/online_lightnav_worker.py'):
        raise ValueError('unexpected worker')
    return result


class ConfiguredWorker(collector.Worker):
    def __init__(self, argv, log):
        run = log.parent.parent
        configured = worker_argv(argv, run)
        if configured != argv:
            save(run/'mpc_worker_launch.json', dict(original_argv=argv, argv=configured,
                configuration_before_tracker=True, command_postprocessing=False))
        super().__init__(configured, log)


if __name__ == '__main__':
    collector.RequestPolicy = RequestPolicy
    collector.ContinuousGeometry = ExploratoryGeometry
    collector.Worker = ConfiguredWorker
    collector.main()
