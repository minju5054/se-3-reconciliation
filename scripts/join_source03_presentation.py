#!/usr/bin/env python3
"""Reporting-only compatibility adapter for frozen SOURCE03 presentation.

The first renderer could not draw an empty/mixed GeometryCollection returned by
clipping the original Hospital map. Preserve it and its partial output. Only
output-name literals are replaced in the frozen report/validator AST; geometry,
numbers, classifications and experiment source remain unchanged. This adapter
performs no model/controller call and refuses existing versioned output.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys
import types

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from reconciliation.join_source03 import read,save,sha


def load_frozen(path, names):
    tree=ast.parse(path.read_text())
    class OutputNames(ast.NodeTransformer):
        def visit_Constant(self,node):
            if isinstance(node.value,str) and node.value in names:
                return ast.copy_location(ast.Constant(names[node.value]),node)
            return node
    tree=ast.fix_missing_locations(OutputNames().visit(tree))
    module=types.ModuleType('_source03_frozen_presentation')
    module.__file__=str(path)
    exec(compile(tree,str(path),'exec'),module.__dict__)
    return module


def collection_plot(polygon, *, ax, **kwargs):
    from shapely.plotting import plot_polygon
    if polygon.is_empty:
        return None
    if polygon.geom_type=='GeometryCollection':
        return [collection_plot(g,ax=ax,**kwargs) for g in polygon.geoms]
    if polygon.geom_type in ('Polygon','MultiPolygon'):
        return plot_polygon(polygon,ax=ax,**kwargs)
    if polygon.geom_type in ('LineString','LinearRing'):
        x,y=polygon.xy
        return ax.plot(x,y,color=kwargs.get('color','gray'),alpha=kwargs.get('alpha',1))
    if hasattr(polygon,'geoms'):
        return [collection_plot(g,ax=ax,**kwargs) for g in polygon.geoms]
    return None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--mode',choices=['report','validate'],required=True)
    a=p.parse_args();run=a.run.resolve()
    name='scripts/report_join_source03.py' if a.mode=='report' else 'scripts/validate_join_source03.py'
    source=ROOT/name
    if sha(source)!=read(run/'freeze.json')['source_sha256'][name]:
        raise ValueError('presentation adapter requires the original frozen source')
    names={'review':'review_v2','review/figure_manifest.json':'review_v2/figure_manifest.json',
           'review_bundle.zip':'review_bundle_v2.zip','validation.json':'validation_v2.json'}
    module=load_frozen(source,names)
    if a.mode=='report':
        module.plot_polygon=collection_plot
        def preserve_existing_summary(path,value):
            if path==run/'aggregate/summary.json' and path.exists():
                if read(path)!=value:raise ValueError('reporting must not change scientific summary')
                return
            if path.suffix=='.json' and isinstance(value,dict) and 'numbers' in value:
                value=dict(value,rendering_adapter_sha256=sha(__file__),frozen_renderer_sha256=sha(source))
            save(path,value)
        module.save=preserve_existing_summary
    sys.argv=[str(source),'--run',str(run)]
    result=module.main()
    save(run/f'presentation_{a.mode}_v2.json',dict(source=str(source),source_sha256=sha(source),
        wrapper_sha256=sha(__file__),output_name_mapping=names,numerical_records_unchanged=True,
        new_model_calls=0,new_MPC_calls=0))
    return result


if __name__=='__main__':raise SystemExit(main())
