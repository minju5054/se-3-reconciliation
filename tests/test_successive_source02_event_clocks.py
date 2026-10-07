import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from report_successive_source02_event_clocks import event_rows

def test_native_nanoseconds_preserved_distinct_from_simulation_seconds():
    r=dict(chunk_id='c',generated=True,A=[0,0,0],B=[1,0,0],
        observation=dict(capture_sim_time_s=1.,capture_monotonic_ns=900000000000001),
        t_request_host=dict(monotonic_ns=900000001000001,utc='request'),
        t_receipt_host=dict(monotonic_ns=900000003000001,utc='receipt'),
        t_ready_seen_sim=dict(sim_time_s=1.2),t_install=dict(sim_time_s=1.2),t_application=dict(sim_time_s=1.3))
    x=event_rows(dict(chunks=[r]))[0]
    assert x['request_host_monotonic_ns']==900000001000001 and x['receipt_host_monotonic_ns']==900000003000001
    assert x['observation_sim_s']==1. and x['application_sim_s']==1.3

def test_unapplied_never_gets_application_stamp_or_B():
    r=dict(chunk_id='c',generated=True,A=[0,0,0],observation=dict(capture_sim_time_s=1.,capture_monotonic_ns=1),
        t_request_host=dict(monotonic_ns=2,utc='request'),t_receipt_host=dict(monotonic_ns=3,utc='receipt'))
    x=event_rows(dict(chunks=[r]))[0]
    assert x['B'] is None and x['application_sim_s'] is None and x['install_sim_s'] is None
