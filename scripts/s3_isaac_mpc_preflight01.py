#!/usr/bin/env python3
"""Zero-solve installation of S3 references using the unchanged official worker."""
import argparse,json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src')]
from osa03_common_b_mpc_worker import preflight
from reconciliation.join_source03 import read
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
    m=read(a.run/'mpc_provenance.json')
    r=preflight(m['lightnav_checkout'],read(a.run/'common_state.json'),read(a.run/'preflight_references.json'))
    r=json.loads(json.dumps(r,allow_nan=False))
    assert r['provenance']['official_settings']==m['official_settings']
    assert r['provenance']['effective_linear_velocity_limit_m_s']==m['effective_linear_velocity_limit_m_s']
    print(json.dumps(r,allow_nan=False))
