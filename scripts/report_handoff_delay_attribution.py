#!/usr/bin/env python3
"""Reporting-only saved analysis. Never imports or calls a controller runtime."""
import argparse,json,sys,zipfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from run_handoff_delay_attribution import read,save,csvsave,sha,flat
from reconciliation.handoff_delay_attribution import CONDITIONS,AUC,paired

def diagnostics(run):
    rows=read(run/'event_records.json');events=read(run/'events.json');result=[]
    for r in rows:
        if not r['command']:continue
        e=next(e for e in events if e['case_id']==r['case_id']);key='LATENCY_FREE' if r['condition'].startswith('LATENCY_FREE') else 'DELAYED'
        start=e['initials'][key];u=np.array(r['command']['commands']);a=np.abs(np.diff(np.vstack([start['u_minus'],u]),axis=0))*10
        bad=np.flatnonzero((a[:,0]>2+1e-5)|(a[:,1]>5+1e-5))
        result.append(dict(case_id=r['case_id'],cohort=r['cohort'],condition=r['condition'],violating_intervals=bad.tolist(),
           first_linear_acceleration_m_s2=float(a[0,0]),first_angular_acceleration_rad_s2=float(a[0,1]),
           later_motion_violation=bool(np.any(bad>0)),source_memory_minus_physical=start['memory_minus_physical'],
           source_state_id=start['state_id'],source_memory_record_row=start['memory']['source_event_row']))
    return result

def tables(run):
    rows=read(run/'event_records.json');gaps,trans=paired(rows);summary=read(run/'summary.json');lines=[]
    lines+=['\n## Completed measurements\n',
      'Corrected execution freeze: `'+read(run/'freeze.json')['execution_sha']+'`. Main run: `'+str(run.relative_to(ROOT))+'`. All84 independent rollouts completed,54 intervals and9 solves each: **756 saved MPC solves,4536 integrated intervals**. VLA/GP/rigid/graph/RGB/Isaac calls: **0**. The preserved failed IPC run adds **one issued MPC request with no saved numerical response**; total issued requests in this task757. No scientific output was used to tune or replace a condition.\n',
      'DN/DL = delayed native/prepared+source-progress; FN/FL = corresponding latency-free conditions. AUC units are m·s. Gap columns are DN−FN and DL−FL. N/A attachment remains N/A.\n']
    for cohort in ['GENUINE_13','ONLINE03_ONSET']:
        lines+=['### '+cohort+' — every frozen event\n','| Event | DN AUC | DL AUC | FN AUC | FL AUC | Native delay gap | Prepared+SP delay gap |','|---|---:|---:|---:|---:|---:|---:|']
        rr=[r for r in rows if r['cohort']==cohort]
        for cid in dict.fromkeys(r['case_id'] for r in rr):
            cr=[next(r for r in rr if r['case_id']==cid and r['condition']==c) for c in CONDITIONS]
            vals=[r['primary']['position_auc_m_s'] for r in cr]
            lines.append('| '+cid+' | '+' | '.join(f'{v:.6f}' for v in vals+[vals[0]-vals[2],vals[1]-vals[3]])+' |')
        lines+=['\n| Event | DN attachment | DL attachment | FN attachment | FL attachment |','|---|---|---|---|---|']
        labels={'NO_TUBE_ENTRY_OBSERVED':'no entry','TUBE_ENTERED_DWELL_RIGHT_CENSORED':'censored','TRANSIENT_ENTRY_THEN_EXIT':'transient'}
        for cid in dict.fromkeys(r['case_id'] for r in rr):
            cr=[next(r for r in rr if r['case_id']==cid and r['condition']==c) for c in CONDITIONS]
            vals=[f"join {r['primary']['join_time_s']:.6f}s" if r['primary']['join_success'] else labels[r['primary']['observation_status']] for r in cr]
            lines.append('| '+cid+' | '+' | '.join(vals)+' |')
        lines+=['\n| Condition | Mean position AUC | Mean yaw AUC [rad·s] | Sustained attach | Motion invalid | Mean linear TV [m/s] | Mean angular TV [rad/s] |','|---|---:|---:|---:|---:|---:|---:|']
        for c in CONDITIONS:
            cr=[r for r in rr if r['condition']==c]
            vals=[np.mean([r['primary']['position_auc_m_s'] for r in cr]),np.mean([r['primary']['yaw_auc_rad_s'] for r in cr])]
            lines.append(f"| {c} | {vals[0]:.6f} | {vals[1]:.6f} | {sum(r['primary']['join_success'] for r in cr)}/{len(cr)} | {sum(not r['command']['nominal_command_grid_valid'] for r in cr)}/{len(cr)} | {np.mean([r['command']['linear_TV_mps'] for r in cr]):.6f} | {np.mean([r['command']['angular_TV_radps'] for r in cr]):.6f} |")
        lines+=['\n| Contrast | Event mean position | Episode-equal position | Event mean excess position | Event mean yaw | Event mean excess yaw |','|---|---:|---:|---:|---:|---:|']
        for name,g in summary['cohorts'][cohort]['paired'].items():
            m=g['event_mean'];lines.append(f"| {name} | {m[AUC[0]]:.6f} | {g['episode_equal_mean'][AUC[0]]:.6f} | {m[AUC[1]]:.6f} | {m[AUC[2]]:.6f} | {m[AUC[3]]:.6f} |")
        lines.append('\nAll four metrics also have episode-equal values and event-level signed contrasts in `summary.json` and `paired_delay_gaps.csv`. Interface contrasts are native minus prepared+SP, not selector-only effects.\n')
    return '\n'.join(lines)+'\n'

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--package-only',action='store_true');p.add_argument('--validate-only',action='store_true');a=p.parse_args();run=a.run.resolve()
    if a.package_only:return package(run)
    if a.validate_only:return validate_report(run)
    assert read(run/'validation.json')['valid']
    rows=read(run/'event_records.json');ds=diagnostics(run);save(run/'motion_diagnostics.json',ds)
    csvsave(run/'motion_diagnostics.csv',[{**r,'violating_intervals':json.dumps(r['violating_intervals']),'source_memory_minus_physical':json.dumps(r['source_memory_minus_physical'])} for r in ds])
    save(run/'interpretation.json',dict(interpretation='MIXED_ATTRIBUTION',
        genuine13='delay sensitivity remains under the frozen complete-state-package comparison; not motion-valid paired execution',
        online03='all four attach; delay usually reduces position AUC; partial onset only',
        primary_limitation='all26 genuine delayed trajectories violate the first physical command acceleration limits with actual post-poll memory; no later violation',
        not_demonstrated=['graph benefit','recoverable maximum','obstacle-constrained improvement','online improvement','real robot benefit','population prevalence']))
    (run/'report_tables.md').write_text(tables(run))
    # Independent recomputation of this supplemental report, no primary re-execution.
    assert read(run/'motion_diagnostics.json')==diagnostics(run)
    assert (run/'report_tables.md').read_text()==tables(run)
    transcript=[json.loads(x) for x in (run/'mpc_worker.jsonl').read_text().splitlines()]
    requests=[r['record'] for r in transcript if r['direction']=='request']
    calls=sum(q['op']=='solve' for q in requests);assert calls==read(run/'summary.json')['actual_MPC_solves']==756
    save(run/'report_validation.json',dict(valid=True,report_only=True,MPC_requests_from_durable_transcript=calls,independent_instances=sum(q['op']=='init' for q in requests),
         diagnostic_records=len(ds),all_violations_first_interval=not any(r['later_motion_violation'] for r in ds),new_solver_calls=0,
         reporting_script_sha256=sha(Path(__file__)),primary_validation_sha256=sha(run/'validation.json')))
    # Small Git-reviewable table: no raw arrays, environment or RGB.
    save(ROOT/'docs/results/handoff_delay_attribution_01.json',dict(execution_sha=read(run/'freeze.json')['execution_sha'],run=str(run.relative_to(ROOT)),
         input_manifest_sha256=sha(run/'source_manifest.json'),event_metrics=[flat(r) for r in rows],summary=read(run/'summary.json'),
         interpretation=read(run/'interpretation.json'),preserved_failed_run_issued_requests=1))
    package(run)

def package(run):
    files=[*run.glob('*.csv'),run/'protocol.json',run/'source_manifest.json',run/'summary.json',run/'interpretation.json',run/'validation.json',run/'report_validation.json',run/'report_tables.md',run/'index.html',*sorted((run/'plots').glob('*.png')),*sorted((run/'plots').glob('*.json'))]
    with zipfile.ZipFile(run/'review_bundle_complete.zip','x',compression=zipfile.ZIP_DEFLATED) as z:
        for q in files:z.write(q,str(q.relative_to(run)))
    print(json.dumps(dict(packaged=True,figures=len(list((run/'plots').glob('*.png'))),review_zip_bytes=(run/'review_bundle_complete.zip').stat().st_size)))
def validate_report(run):
    from run_handoff_delay_attribution import verify,collect,summarize
    from plot_handoff_delay_attribution import validate_plots
    verify(run)
    rows=collect(run)
    assert rows==read(run/'event_records.json')
    gaps,trans=paired(rows)
    assert summarize(rows,gaps,trans)==read(run/'summary.json')
    assert diagnostics(run)==read(run/'motion_diagnostics.json')
    assert tables(run)==(run/'report_tables.md').read_text()
    validate_plots(run)
    archive=run/'review_bundle_complete.zip'
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():assert z.read(name)==(run/name).read_bytes()
        assert len([n for n in z.namelist() if n.endswith('.png')])==27
        assert all('plots/'+p.name in z.namelist() for p in (run/'plots').glob('*.json'))
    report=read(ROOT/'docs/results/handoff_delay_attribution_01.json')
    assert report['event_metrics']==[flat(r) for r in rows]
    assert report['summary']==read(run/'summary.json')
    result=dict(valid=True,source_core_unchanged=True,metrics_recomputed=True,report_tables_recomputed=True,
        review_zip_sha256=sha(archive),public_numeric_table_sha256=sha(ROOT/'docs/results/handoff_delay_attribution_01.json'),
        current_reporting_script_sha256=sha(Path(__file__)),
        reporting_revision='packaging/validation only; numerical diagnostics unchanged from initial report validation',
        counts=read(run/'test_and_call_accounting.json'),new_MPC_VLA_calls=0)
    save(run/'report_validation_final.json',result)
    print(json.dumps(result))

if __name__=='__main__':main()
