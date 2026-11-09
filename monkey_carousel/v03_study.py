"""Reproducible V0.3 study. Run python -m monkey_carousel.v03."""
from dataclasses import asdict,replace
from pathlib import Path
import json
import hashlib
import numpy as np
from .v03 import Design,scan,phase_certificate,closing_gap,stop_audit,bodies,prism_gaps
from .gears import comparison,mesh_audit

OUT=Path(__file__).resolve().parents[1]/'outputs'/'v03'


def source_hashes():
    root=Path(__file__).parent
    return {name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in
            ['model.py','v03.py','gears.py','indexing.py','v03_visual.py','v03_study.py']}


def split_plan(d):
    # 180 mm square bed assumption, excludes un-designed joint protrusions.
    return dict(bed_xy_mm=[180,180],status='split concept only; joints and inserts not load-verified',
                parts=[dict(name='panels',count=12,bbox_mm=[d.radius-2*d.end_trim,6,d.height/2]),
                       dict(name='five_plate_inner_sectors',count=60,bbox_mm=[(d.radius+40)/2,(d.radius+40)*np.sin(np.pi/12),12]),
                       dict(name='five_plate_outer_sectors',count=60,bbox_mm=[(d.radius+40)*(1-.5*np.cos(np.pi/12)),2*(d.radius+40)*np.sin(np.pi/12),12]),
                       dict(name='sun200_sectors',count=8,bbox_mm=[151.5,2*151.5*np.sin(np.pi/8),12]),
                       dict(name='door40_gears',count=6,bbox_mm=[63,63,12]),
                       dict(name='reducer_gears',count=4,max_bbox_mm=[153,153,12])],
                note='180mm bed: each of five plates has 12 angular sectors x 2 radial pieces. At 256mm bed six sectors/plate suffice. Counts exclude joints, hubs, rails, fasteners, dome; final CAD counts pending.')


def run():
    hashes=source_hashes()
    OUT.mkdir(exist_ok=True);d=Design()
    print('V0.3 coarse full scene',flush=True);coarse=scan(d,1)
    print('V0.3 0.1deg refined full scene',flush=True);fine=scan(d,.1)
    if not fine['pass']:raise RuntimeError('V0.3 clearance failed; modify design and rerun')
    if not closing_gap(d)['closed_appearance_pass']:raise RuntimeError('Closed appearance failed')
    print('V0.3 search and phase certificates',flush=True)
    search=[]
    for r in (140,160,180,200):
        p=replace(d,radius=r);v=scan(p,1,False,False)
        search.append(dict(radius_mm=r,closing=closing_gap(p),nominal_scan={k:v[k] for k in
                           ['samples','min_nominal_sat_gap_mm','penetrating_pair_samples','clearance_fail_pair_samples']},
                           note='Body-only coarse screen; gear centers valid only for selected R180'))
    phases=[dict(collision=phase_certificate(fine,p),appearance=closing_gap(replace(d,phase_deg=p)))
            for p in (0,.1,.25,.5,1)]
    print('V0.3 0.1deg tooth profile mesh audit',flush=True);gears=mesh_audit(.1)
    if not gears['sampled_pass']:raise RuntimeError('Nominal sampled teeth penetrate')
    stops=stop_audit(d)
    if not stops['geometrically_blocks_overtravel'] or not stops['full_stroke_no_penetration']:
        raise RuntimeError('Stop geometry failed')
    from .indexing import report as indexing_report,detent
    if source_hashes()!=hashes:
        raise RuntimeError('Source changed during study: rerun before publishing artifacts')
    report=dict(version='0.3',source_sha256=hashes,units='mm, deg, N m',design=asdict(d),closing=closing_gap(d),
                coarse=coarse,refined=fine,search=search,independent_phase_cases=phases,
                transmission=comparison(),gear_profile_audit=gears,stop_audit=stops,indexing=indexing_report(),
                status='Conditional rigid-body geometry validation; not manufacturing release',
                limitations=['surface_error is an assumed Hausdorff displacement cap per solid, not measured process capability',
                'Gear contact/clearance proof applies only to nominal sampled polyline profiles, not printed teeth or body phase errors',
                'Spring detent, clutch friction, strength, wear, creep, tilt and dynamic stop impact require physical tests',
                'Excluded fits, fixed-to-fixed joins, bolts, dome and split joints require detailed manufacturing CAD'])
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (OUT/'split_plan.json').write_text(json.dumps(split_plan(d),indent=2)+'\n')
    a=np.linspace(0,48,481);v=detent(a)
    np.savetxt(OUT/'detent_curve.csv',np.c_[a,v['lift_mm'],v['force_N'],v['energy_J'],v['cam_resisting_torque_Nm']],
               delimiter=',',header='sun_deg,lift_mm,force_N,energy_J,cam_torque_Nm',comments='')
    print('V0.3 export',flush=True)
    from .v03_visual import export
    export(d,OUT)
    print('V0.3 complete',fine['continuous_robust_lower_bound_mm'],closing_gap(d),flush=True)
