#!/usr/bin/env python3
"""Post-run saved-only wall/cart attribution; does not alter frozen source gates."""
import argparse
from pathlib import Path
import sys
import numpy as np
from shapely.geometry import LineString
from shapely.ops import nearest_points
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from run_join_online02 import environments
from report_blind_corner_source import geometries


def audit(run):
    v=read(run/'validation.json');results=[]
    for row in v['rows']:
        cid=row['candidate_id'];base,on,cart,source=environments(run/'candidates'/cid)
        parts=read(Path(source['environment_export'])/'geometry/projected/obstacle_parts.json');meshes=read(Path(source['environment_export'])/'geometry/meshes.json')
        result=dict(candidate_id=cid,source_validation_sha256=sha(run/'validation.json'),changed_qualification=False,paths={})
        for label in ['OLD','FRESH']:
            record=row[label]
            if not record or 'world' not in record:continue
            path=LineString(np.asarray(record['world'])[:,:2]);j=int(base.tree.nearest(path));a,b=nearest_points(path,base.parts[j]);k=int(parts[j]['mesh_id'])
            points=[(i,LineString(np.asarray(record['world'])[i:i+2,:2])) for i in range(record['N']-1)]
            crossing=next((i for i,s in points if s.distance(base.obstacles)-.2<.05+base.numerical_tolerance_m),None)
            result['paths'][label]=dict(cart_only_edge_clearance_m=float(path.distance(cart)-.2),
                hospital_only_edge_clearance_m=float(path.distance(base.obstacles)-.2),
                nearest_Hospital_mesh=meshes[k]['prim_path'],nearest_path_xy=list(a.coords)[0],nearest_obstacle_xy=list(b.coords)[0],
                first_margin_violating_segment=crossing,raw_sha256=record['raw_local_ref']['sha256'])
        results.append(result)
        if 'FRESH' not in result['paths']:continue
        r=result['paths']['FRESH'];xy=np.array(r['nearest_path_xy']);fig,ax=plt.subplots(figsize=(7,6))
        geometries(ax,base.obstacles,color='gray',alpha=.7)
        geometries(ax,base.obstacles.buffer(.05),color='gray',alpha=.15)
        w=np.array(row['FRESH']['world']);ax.plot(w[:,0],w[:,1],'r--o',ms=4,label='Raw FRESH prediction only')
        ax.add_patch(Circle(xy,.20,fill=False,color='red',lw=2,label='20 cm footprint at raw-path minimum'))
        ax.plot(*np.array([r['nearest_path_xy'],r['nearest_obstacle_xy']]).T,'k:')
        ax.set(xlim=(xy[0]-.55,xy[0]+.55),ylim=(xy[1]-.55,xy[1]+.55),xlabel='World X [m]',ylabel='World Y [m]',
            title=f'Predicted footprint edge: {r["hospital_only_edge_clearance_m"]:.5f} m\nRequired .05 m; margin violation, no physical overlap')
        ax.set_aspect('equal');ax.legend(fontsize=8);fig.tight_layout();fig.savefig(run/'review'/f'{cid}_wall_clearance.png',dpi=160);plt.close(fig)
        save(run/'review'/f'{cid}_wall_clearance.json',result)
    save(run/'supplemental_clearance_attribution.json',dict(rows=results,scope='SAVED-ONLY DESCRIPTIVE CHECK; primary qualification unchanged',model_MPC_calls=0,
        script_sha256=sha(__file__)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();audit(a.run.resolve())
