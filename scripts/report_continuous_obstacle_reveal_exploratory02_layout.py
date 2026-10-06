#!/usr/bin/env python3
"""Saved-only RGB panel spacing correction after visual QA; no scientific calls.

Keep the frozen reporter and its original PNG bytes unchanged in an ignored
archive. Rerender only RGB from the exact saved JPEGs, with dedicated metadata
axes so no text overlaps another image. Record both rendering provenances.
"""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha
from run_continuous_obstacle_reveal_exploratory02 import namespace,OUT
from report_continuous_obstacle_reveal_exploratory02 import validate_report


def correct(run):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    validate_report(run,OUT)
    v=read(run/'validation.json');ep=Path(v['episode'])
    name='request_rgb_sequence.png';path=OUT/'figures'/name
    archive=run/'reporting/original_frozen_report';archive.mkdir(parents=True,exist_ok=False)
    for n in (name,'figure_manifest.json'):
        source=path if n==name else OUT/n
        (archive/n).write_bytes(source.read_bytes())
    fig=plt.figure(figsize=(12,8.5),layout='constrained')
    grid=fig.add_gridspec(4,2,height_ratios=[.22,1,.22,1])
    for i,r in enumerate(v['chunks']):
        row,col=2*(i//2),i%2
        title=fig.add_subplot(grid[row,col]);title.axis('off')
        ax=fig.add_subplot(grid[row+1,col]);ax.axis('off')
        if not r['generated']:
            title.text(.5,.5,f'C{i}: N/A — not generated',ha='center',va='center');continue
        source=ep/r['observation']['path'];assert sha(source)==r['observation']['sha256']
        status='applied' if r['applied'] else 'REJECTED before installation'
        text=(f'C{i} | {status} | cart pixels {r["cart_visible_pixels"]}\n'
            f'obs sim {r["observation"]["capture_sim_time_s"]:.6f} s\n'
            f'A{i} = ({r["A"][0]:.6f}, {r["A"][1]:.6f}, {r["A"][2]:.6f})')
        title.text(.5,.5,text,ha='center',va='center',fontsize=10)
        with Image.open(source) as im: ax.imshow(im)
    fig.suptitle('Exact request RGB sequence — metadata outside images; original JPEG bytes preserved',fontsize=12)
    fig.savefig(path,dpi=160,bbox_inches='tight');plt.close(fig)
    m=read(OUT/'figure_manifest.json');m['PNG_sha256'][name]=sha(path)
    m['layout_correction']=dict(script=str(Path(__file__).resolve()),script_sha256=sha(__file__),
        original_manifest=str(archive/'figure_manifest.json'),original_manifest_sha256=sha(archive/'figure_manifest.json'),
        original_RGB_PNG_sha256=sha(archive/name),reason='dedicated metadata axes prevent title overlap',
        new_scientific_calls=0)
    (OUT/'figure_manifest.json').write_text(__import__('json').dumps(m,indent=2)+'\n')
    result=validate_report(run,OUT)
    for field in ['script','original_manifest']:
        assert sha(m['layout_correction'][field])==m['layout_correction'][field+'_sha256']
    save(run/'reporting/layout_validation.json',result)
    print(result)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    correct(namespace(p.parse_args().run))
