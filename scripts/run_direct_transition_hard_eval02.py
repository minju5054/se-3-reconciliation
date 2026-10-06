#!/usr/bin/env python3
"""Outcome-blind scan, freeze, one V2 solve per source, then four method executions."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'scripts/isaac')]
import numpy as np
import yaml
import run_relative_factor_multisource01 as runtime
from run_b_to_entry_graph_formulation_diag02 import solve_variant, load_problem, planning_only
from run_b_to_entry_bridge01 import write_array
from run_osa03_native_continuation import plain, git
from reconciliation.join_source03 import read,save,sha
from reconciliation.robotless_online import stamp
from reconciliation.graph_optimizer import SolverConfig
from reconciliation.direct_transition_hard_eval02 import *
from reconciliation.direct_transition_sources02 import exclusion_registry,scan,authenticate
from reconciliation.boundary_row_ablation04 import staged_reference,identity_mapping
from reconciliation.spatial_entry_suffix import suffix_reference,no_reconciliation_optimizer
from reconciliation.b_to_entry_bridge import hermite_bridge,BridgeProblem,reference_arrays
from reconciliation.b_to_entry_graph_diag02 import sample_hermite,describe
from reconciliation.relative_factor_multisource import load_source,geometry,reference_safety
from reconciliation.se2 import relative_pose
CONFIG=ROOT/'configs/direct_transition_hard_eval_02.yaml'
RESULTS=ROOT/'results/direct_transition_hard_eval_02'
DOC=ROOT/'docs/DIRECT_TRANSITION_HARD_EVAL_02.md'


def record_reference(folder,name,world,local,labels):
    rec={}
    for frame,a in [('world',world),('local',local)]:
        p=folder/'references'/f'{name}_{frame}.npy';write_array(p,a)
        rec.update({frame+'_path':str(p),frame+'_sha256':sha(p)})
    safe=reference_safety(world,geometry(folder))
    return dict(**rec,labels=labels,safety=safe,status='REFERENCE_GEOMETRY_SAFE' if safe['clearance_valid'] else 'REFERENCE_GEOMETRY_UNSAFE')


def inventory(run):
    with no_reconciliation_optimizer():
        cfg=yaml.safe_load(CONFIG.read_text());run.mkdir(parents=True,exist_ok=False)
        save(run/'protocol.json',cfg);reg=exclusion_registry(ROOT,cfg);save(run/'development_exclusion_registry.json',reg)
        r=scan(ROOT,cfg,reg);save(run/'candidate_ledger.json',r['rows']);save(run/'selection.json',r['selection']);save(run/'selection_trace.json',r['selection']['selection_trace'])
        save(run/'source.json',{k:v for k,v in r.items() if k not in ['rows','selection']})
        print(json.dumps(r['selection']),flush=True)


def loader_config(run):
    return dict(scan_run=str(run),inventory_hashes={str(run/n):sha(run/n) for n in ['source.json','ledger.json']})


def prepare(run):
    with no_reconciliation_optimizer():
        cfg=read(run/'protocol.json');assert cfg==yaml.safe_load(CONFIG.read_text())
        assert cfg['solver']==asdict(SolverConfig()) and cfg['methods']==METHODS and cfg['M']==2 and cfg['V2_variant']=='V2'
        authenticate(ROOT,cfg)
        shutil.copyfile(run/'candidate_ledger.json',run/'ledger.json')
        selection=read(run/'selection.json');selected=[]
        if selection['selected_count']<4:
            save(run/'selected_sources.json',[dict(id=sid,label=f'E{i}',kind='genuine')
                for i,sid in enumerate(selection['selected_ids'],1)])
            save(run/'expected_calls.json',dict(optimizer=0,rollouts=0,MPC_solves=0,MPC_applications=0));return
        rows={r['case_id']:r for r in read(run/'candidate_ledger.json')}
        for i,sid in enumerate(selection['selected_ids'],1):
            spec=dict(id=sid,kind='genuine',label=f'E{i}');data=load_source(loader_config(run),spec)
            f=run/'sources'/spec['label'];(f/'references').mkdir(parents=True,exist_ok=False)
            for name,key in [('common_state','common'),('source_manifest','manifest'),('schedule','schedule'),('scenario','scene'),('source_audit','audit')]:save(f/(name+'.json'),data[key])
            save(f/'protocol.json',cfg);save(f/'selection_geometry.json',rows[sid])
            # Exact historical evaluator configuration; no method-specific metric change.
            metric=read(ROOT/'data/osa03_relative_factor_ablation_01/primary_20260929T050000Z/metric_protocol.json')
            save(f/'metric_protocol.json',metric);write_array(f/'recorded_old_to_B.npy',data['past'])
            fresh,raw=data['fresh'],data['raw'];c=data['common'];before=raw.tobytes()
            native=record_reference(f,'M0_NATIVE',fresh,raw,[f'F_{j}' for j in range(len(raw))])
            save(f/'prepared_references.json',{'M0_NATIVE':native});runtime.prior.preflight(f)
            pre=read(f/'restoration_preflight.json');installed=np.asarray(pre['rows'][0]['installed_world'])
            (f/'restoration_preflight.json').rename(f/'native_installation_preflight.json')
            (f/'prepared_references.json').rename(f/'native_reference.json')
            suffix,local,entry=suffix_reference(fresh,raw,c['fresh_capture_pose'],c['B'],rows[sid]['entry']['correspondence'])
            ids=entry['row_original_identities'][1:];suffix[1:]=installed[ids]
            # E* is fixed from the original C3 computation. Only original suffix bits
            # are copied from the authenticated official zero-solve Native install.
            suffix[0]=entry['correspondence']['target_world']
            local=np.vstack([relative_pose(c['fresh_capture_pose'],suffix[0]),raw[ids]])
            save(f/'entry.json',entry);write_array(f/'suffix.npy',suffix);write_array(f/'native_installed.npy',installed)
            refs={METHODS[0]:record_reference(f,METHODS[0],suffix,local,['E*',*[f'F_{j}' for j in ids]])}
            w,l,labels=staged_reference(installed,raw,c['fresh_capture_pose'],c['B'],entry,suffix)
            refs[METHODS[1]]=record_reference(f,METHODS[1],w,l,labels)
            assert refs[METHODS[1]]['safety']['clearance_valid']
            P=data['past'][-2];h,metadata=hermite_bridge(P,c['B'],suffix)
            hp=BridgeProblem(P,c['B'],suffix,metadata['d_F_m'],metadata['M'])
            w,l,labels=reference_arrays(hp,h,c['fresh_capture_pose'],raw,ids)
            refs[METHODS[2]]=record_reference(f,METHODS[2],w,l,labels)
            write_array(f/'hermite_bridge.npy',h);save(f/'hermite_input.json',metadata)
            v,sampling=sample_hermite(P,c['B'],suffix,2);write_array(f/'initial_M2.npy',v)
            save(f/'input.json',dict(P=P,B=c['B'],d_F_m=metadata['d_F_m'],historical_folder=str(f),original_ids=ids,sampling=sampling))
            p=load_problem(f,'V2');save(f/'V2_initial_safety.json',reference_safety(p.reference(v[1:-1]),geometry(f)))
            assert raw.tobytes()==before
            save(f/'prepared_references.json',refs)
            save(f/'reference_safety.json',{n:r['safety'] for n,r in refs.items()})
            runtime.prior.preflight(f);pre=read(f/'restoration_preflight.json')
            audits={}
            for r in pre['rows']:
                name=r['identity'];actual=np.asarray(r['installed_world']);n=len(refs[name]['labels'])-len(ids)
                assert actual[n:].tobytes()==installed[ids].tobytes()
                safe=reference_safety(actual,geometry(f));assert safe['clearance_valid']==refs[name]['safety']['clearance_valid']
                audits[name]=dict(derived_roundoff=float(np.max(abs(actual[:n]-np.load(refs[name]['world_path'])[:n]))),
                    downstream_bit_exact=True,safety=safe)
            save(f/'installation.json',audits)
            save(f/'identity_mapping.json',{n:identity_mapping(r['labels'],entry,installed) for n,r in refs.items()})
            selected.append(dict(**spec,folder=str(f),Hermite_M=metadata['M'],V2_M=2,
                severity=rows[sid]['severity'],remaining_arc_m=rows[sid]['remaining_arc_m']))
        save(run/'selected_sources.json',selected)
        calls=sum(len(read(Path(s['folder'])/'schedule.json')['pairs']) for s in selected)
        apps=sum(len(read(Path(s['folder'])/'schedule.json')['full']['applications']) for s in selected)
        save(run/'expected_calls.json',dict(optimizer=len(selected),rollouts=4*len(selected),MPC_solves=4*calls,MPC_applications=4*apps,
            conditional='V2 invalid final references are skipped; all abort prefixes retained; no retry'))
        print(json.dumps(dict(prepared=selected,expected=read(run/'expected_calls.json'))),flush=True)


def freeze(run):
    assert read(run/'protocol.json')==yaml.safe_load(CONFIG.read_text())
    files=[ROOT/p for p in git('ls-files','src','scripts','tests','configs').splitlines()
           if p not in ['configs/stage0_jackal_controller_validation.yaml','configs/stage0_lightnav_single_chunk.yaml']]
    files+= [CONFIG,ROOT/'src/reconciliation/direct_transition_hard_eval02.py',ROOT/'src/reconciliation/direct_transition_sources02.py',
        *[ROOT/'scripts'/f'{s}_direct_transition_hard_eval02.py' for s in ['run','validate','report']],ROOT/'tests/test_direct_transition_hard_eval02.py']
    shutil.copyfile(DOC,run/'protocol_document.md')
    save(run/'freeze.json',dict(files={str(p):sha(p) for p in files},inputs={str(p):sha(p) for p in run.rglob('*') if p.is_file()}))
    RESULTS.mkdir(parents=True,exist_ok=True)
    save(RESULTS/'freeze_summary.json',dict(run=str(run),freeze=read(run/'freeze.json'),selection=read(run/'selection.json'),
        selected_sources=read(run/'selected_sources.json'),expected_calls=read(run/'expected_calls.json')))
    for n in ['protocol.json','development_exclusion_registry.json','selection.json','selection_trace.json','selected_sources.json']:
        shutil.copyfile(run/n,RESULTS/n)


def verify(run,pushed=False):
    for group in ['files','inputs']:
        for p,h in read(run/'freeze.json')[group].items():assert sha(p)==h,p
    for p,h in read(run/'source.json')['source_hashes'].items():assert sha(p)==h,p
    if pushed:
        assert git('rev-parse','HEAD')==git('ls-remote','origin','refs/heads/main').split()[0]
        for p in read(run/'freeze.json')['files']:
            assert git('rev-parse','HEAD:'+str(Path(p).relative_to(ROOT)))==git('hash-object',p),p
        for p,q in [(DOC,run/'protocol_document.md'),(RESULTS/'freeze_summary.json',RESULTS/'freeze_summary.json')]:
            assert git('rev-parse','HEAD:'+str(p.relative_to(ROOT)))==git('hash-object',str(q))


def execute(run):
    verify(run,pushed=True);selected=read(run/'selected_sources.json');freeze_sha=git('rev-parse','HEAD')
    save(run/'execution_start.json',dict(sha=freeze_sha,started=stamp(),selected_count=len(selected)))
    if len(selected)<4:
        save(run/'completion.json',dict(status='INSUFFICIENT_MEANINGFUL_TRANSITION_SOURCES',optimizer=0,MPC=0));return
    # Every planning solve precedes every rollout. Literal V2; no V3 or retry path.
    with planning_only():
        for s in selected:solve_variant(Path(s['folder']),'V2',freeze_sha)
    save(run/'planning_complete.json',dict(completed=stamp(),optimizer_calls=len(selected)))
    with no_reconciliation_optimizer():
        for s in selected:
            f=Path(s['folder']);p=load_problem(f,'V2');out=f/'planning/V2';result=read(out/'result.json')
            bridge=np.load(out/'last_accepted_bridge.npy');w=p.reference(bridge[1:-1]);safe=reference_safety(w,geometry(f))
            status=describe(p,bridge[1:-1],result['solver'],read(out/'trace.json'),safe,result['wall_s'])
            save(out/'status.json',plain(status));refs=read(f/'prepared_references.json')
            c=read(f/'common_state.json');raw=np.load(read(f/'native_reference.json')['M0_NATIVE']['local_path'])
            ids=read(f/'input.json')['original_ids'];l=np.vstack([relative_pose(c['fresh_capture_pose'],bridge),raw[ids]])
            ref=record_reference(f,METHODS[3],w,l,['B','bridge_1','E*',*[f'F_{j}' for j in ids]])
            ref['planning_valid']=status['stable'];refs[METHODS[3]]=ref
            if not status['stable']:ref['status']='SKIPPED_REFERENCE_INVALID'
            save(f/'references.json',refs)
            save(f/'final_reference_safety.json',{n:r['safety'] for n,r in refs.items()})
            mapping=read(f/'identity_mapping.json');mapping[METHODS[3]]=identity_mapping(ref['labels'],read(f/'entry.json'),np.load(f/'native_installed.npy'))
            save(f/'final_identity_mapping.json',mapping)
            for name in METHODS:
                valid=refs[name]['safety']['clearance_valid'] and (name!=METHODS[3] or status['stable'])
                if valid:runtime.run_method(f,name)
                else:
                    skipped=f/'methods'/name;skipped.mkdir(parents=True,exist_ok=False)
                    save(skipped/'skipped.json',dict(status='SKIPPED_REFERENCE_INVALID',reference_safe=refs[name]['safety']['clearance_valid'],metrics=None))
        verify(run);save(run/'completion.json',dict(completed=stamp(),optimizer=len(selected),retries=0,LightNav=0,RGB=0,Isaac=0))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['inventory','prepare','freeze','execute'],required=True)
    a=p.parse_args();globals()[a.mode](a.run.resolve())
