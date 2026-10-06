#!/usr/bin/env python3
"""Run the historical native collector with the isolated exploratory checker."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts/isaac'),str(ROOT/'scripts'),str(ROOT/'src')]
import continuous_obstacle_reveal_episode01 as collector
from continuous_obstacle_reveal_episode01 import GeometryWorker, json, os, subprocess

class ExploratoryGeometry(GeometryWorker):
    def __init__(self, run):
        self.log=(run/'logs/geometry.log').open('x')
        env={k:v for k,v in os.environ.items() if k not in ('LD_LIBRARY_PATH','PYTHONPATH')}
        env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        self.p=subprocess.Popen([str(ROOT/'.venv/bin/python'),
            str(ROOT/'scripts/continuous_obstacle_reveal_exploratory_geometry02.py'),str(run)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,
            text=True,bufsize=1,env=env,cwd=ROOT)
        assert json.loads(self.p.stdout.readline()) == {'ready': True}

if __name__=='__main__':
    collector.ContinuousGeometry=ExploratoryGeometry
    collector.main()
