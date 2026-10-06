#!/usr/bin/env python3
"""Historical geometry RPC and guard; only the explicit margin adapter differs."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
import continuous_obstacle_reveal_geometry_worker as worker
from run_join_online02 import environments as original_environments
from reconciliation.join_source03 import read
from reconciliation.continuous_obstacle_reveal_exploratory02 import ExploratoryEnvironment

def environments(run):
    base,on,cart,source=original_environments(run)
    policy=read(run/'protocol.json')['exploratory_clearance']
    return ExploratoryEnvironment(base,policy),ExploratoryEnvironment(on,policy),cart,source

if __name__=='__main__':
    worker.environments=environments
    worker.main()
