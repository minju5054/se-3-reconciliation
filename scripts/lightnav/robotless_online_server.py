#!/usr/bin/env python3
"""Persistent official server with source-supported GPU memory fraction only."""
import argparse
import os
from pathlib import Path
import robotless_successive_server as server

_original_arguments=server.server_arguments

def online_arguments(config,run):
    return _original_arguments(config,run)+['--gpu_memory_utilization',str(config['lightnav']['gpu_memory_utilization'])]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('start','stop'))
    parser.add_argument('run', type=Path)
    parser.add_argument('--timeout-s',type=float,default=300)
    args=parser.parse_args()
    server.server_arguments=online_arguments
    if args.mode=='start':
        for key in ('VLN_TIMESTAMP_RELATIVE','VLLM_QUANT','VLN_KV_CACHE_GIB','VLN_VLLM_ENFORCE_EAGER','ASPECT_MODE'):
            os.environ.pop(key,None)
        server.start(args.run.resolve(),args.timeout_s)
    else:
        # Same identity-checked shutdown, with this collector's accurate reason.
        original=server.save_json_exclusive
        def save(path,data):
            if Path(path).name=='server_shutdown.json':
                data['reason']='online collection complete; release the persistent model server GPU allocation'
            original(path,data)
        server.save_json_exclusive=save
        server.stop(args.run.resolve(),args.timeout_s)

if __name__=='__main__': main()
