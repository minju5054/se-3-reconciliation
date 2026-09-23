#!/usr/bin/env python3
"""Direct geometry IPC for abort-only online acquisition; no solvers."""
import argparse,json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from run_join_online02 import environments,read
from reconciliation.join_online02 import guard_check

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    base,on,_,_=environments(a.run);present=False
    print(json.dumps({'ready':True}),flush=True)
    for line in sys.stdin:
        q=json.loads(line);start=time.monotonic()
        try:
            if q['op']=='close':break
            if q['op']=='set':present=q['present'];r={'present':present}
            elif q['op']=='guard':r=guard_check(on if present else base,q['pose'],q['command'],q['dt'])
            else:raise ValueError('undeclared geometry operation')
            print(json.dumps({'ok':True,'result':r,'wall_s':time.monotonic()-start}),flush=True)
        except Exception as e:print(json.dumps({'ok':False,'error':repr(e)}),flush=True)
if __name__=='__main__':main()
