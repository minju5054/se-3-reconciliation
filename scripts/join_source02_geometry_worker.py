#!/usr/bin/env python3
"""CPU research-venv geometry IPC; no model, controller or optimization imports."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from reconciliation.gp_se2_environment import HospitalEnvironment
from reconciliation.join_source02_acquisition import held_interval_check
from reconciliation.join_source02_geometry import project_prop, compose_environment


def main():
    p=argparse.ArgumentParser();p.add_argument('--environment',type=Path,required=True);a=p.parse_args()
    base=HospitalEnvironment.load(a.environment);on=None;present=False
    print(json.dumps({'ready':True}),flush=True)
    for line in sys.stdin:
        start=time.monotonic()
        try:
            q=json.loads(line);op=q['op']
            if op=='close':break
            if op=='prop':
                projection=project_prop(np.load(q['triangles'],allow_pickle=False)['triangles'])
                on=compose_environment(base,projection,True)
                reply=dict(metadata=projection['metadata'])
            elif op=='set':
                if q['present'] and on is None:raise ValueError('obstacle geometry is unavailable')
                present=bool(q['present']);reply=dict(present=present)
            elif op=='held':reply=held_interval_check(on if present else base,q['start'],q['end'])
            elif op=='polyline':reply=(on if q.get('present',present) else base).check_polyline(q['poses'])
            elif op=='query':reply=(on if q.get('present',present) else base).query(q['xy'])
            else:raise ValueError('unknown geometry request')
            reply={'ok':True,'result':reply,'wall_s':time.monotonic()-start}
        except Exception as e:reply={'ok':False,'error':f'{type(e).__name__}: {e}'}
        print(json.dumps(reply,allow_nan=False),flush=True)


if __name__=='__main__':main()
