"""Independent OpenCascade distance witnesses, supplementary to full SAT sweep."""
import json
import numpy as np
from .model import Parameters,boxes,poses


def distance(p,angle,names):
    import cadquery as cq
    bs=boxes(p)
    c,a=poses(p,bs,[angle])
    solids=[]
    for name in names:
        i=next(i for i,b in enumerate(bs) if b.name==name)
        solids.append(cq.Workplane('XY').box(*bs[i].size)
                      .rotate((0,0,0),(0,0,1),float(np.rad2deg(a[0,i])))
                      .translate(tuple(c[0,i])).val())
    return float(solids[0].distance(solids[1]))


def run(out):
    report=json.loads((out/'collision_report.json').read_text())
    result={}
    for key in ['candidate','selected']:
        if key=='candidate':
            p=Parameters(**report['candidate_parameters'])
            v=report['candidate']
        else:
            p=Parameters(**report['selected']['parameters'])
            v=report['selected']['verification']
        closest=v['refined'] or v['coarse']
        result[key]=dict(angle_deg=closest['angle'],pair=closest['pair'],
                         opencascade_distance_mm=distance(p,closest['angle'],closest['pair']),
                         note='Exact B-rep distance at ONE witness pose, not full-sweep exact minimum')
    (out/'cad_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    from pathlib import Path
    print(json.dumps(run(Path(__file__).resolve().parents[1]/'outputs'),indent=2))
