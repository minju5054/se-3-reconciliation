#!/usr/bin/env python3
"""Unchanged direct guard plus pre-install whole raw reference check, no fitting."""
import json
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
from run_join_online02 import environments
from reconciliation.join_online02 import guard_check
from reconciliation.join_source02 import whole_raw_polyline_check

def main():
    base, on, _, _ = environments(Path(sys.argv[1]))
    present = False
    print(json.dumps({'ready': True}), flush=True)
    for line in sys.stdin:
        try:
            q = json.loads(line)
            if q['op'] == 'close':
                break
            if q['op'] == 'set':
                present = q['present']; r = {'present': present}
            elif q['op'] == 'guard':
                r = guard_check(on if present else base, q['pose'], q['command'], q['dt'])
            elif q['op'] == 'reference':
                r = whole_raw_polyline_check(q['world'], on if present else base)
            else:
                raise ValueError('unknown operation')
            print(json.dumps({'ok': True, 'result': r}), flush=True)
        except Exception as e:
            print(json.dumps({'ok': False, 'error': repr(e)}), flush=True)

if __name__ == '__main__':
    main()
