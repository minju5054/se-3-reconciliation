"""Regression for observed outcome types; synthetic fixtures, no inference."""
from reconciliation.join_source05 import ORDER, KS, classify_instruction
from test_join_source05 import sample


def test_safe_shortening_and_stop_cannot_be_promoted_to_detour(monkeypatch):
    import reconciliation.join_source05 as module
    monkeypatch.setattr(module,'compare',lambda a,b,_:dict(equivalent=False,meaningful=False,
        clearance_gain_m=b['geometry_on']['whole']['minimum_clearance_m']-a['geometry_on']['whole']['minimum_clearance_m']))
    historical={('K0_OFF' if k==0 else f'K{k}'):sample() for k in KS}
    r={c:sample(clearance=.3,safe=True,y=.001) for c in ORDER if c.startswith('I1')}
    r['I1_K8']['stop']=True;r['I1_K8']['hallway_endpoint_forward_m']=0
    d=classify_instruction(r,historical,'I1',{})
    assert d['classification']=='INSTRUCTION_EFFECT_INCONCLUSIVE'
    assert not d['visual_conditioned_safe_detours'] and not d['complete_bypasses']
    assert all(x['safe_on'] for x in d['comparisons'].values())
