"""Run: python -m monkey_carousel.study (writes outputs/, no production STL)."""
import json
from pathlib import Path
from dataclasses import asdict
import numpy as np
from .model import Parameters, scan, certify

OUT = Path(__file__).resolve().parents[1] / 'outputs'


def local_refinement():
    """Local integer-mm study; keep nominal recommendation with extra margin."""
    rows=[]
    for length in (88,89,90):
        for radius in (137,138,139):
            p=Parameters(radius=radius,length=length)
            rows.append(dict(parameters=asdict(p),verification=certify(p)))
    return rows


def transmission():
    """Explicit provisional friction scenarios, NOT measured PETG properties."""
    cases = []
    for door_torque, efficiency in [(0.02,0.85),(0.08,0.7),(0.2,0.5)]:
        for ratio in (1,2,3):
            torque = 6*door_torque/(ratio*efficiency)
            cases.append(dict(door_torque_Nm=door_torque,efficiency=efficiency,
                              knob_turns_per_door_turn=ratio,
                              knob_travel_deg=240*ratio,
                              knob_torque_Nm=torque,
                              hand_force_at_25mm_N=torque/.025))
    return dict(status='unverified scenario estimates, no measured friction',
                formula='T_knob=6*T_door/(ratio*efficiency); angle_knob=240*ratio',
                cases=cases)


def run():
    OUT.mkdir(exist_ok=True)
    candidate = certify(Parameters())
    print('Candidate',candidate['continuous_envelope_pass'],flush=True)
    grid = []
    frontier = []
    # Exhaustive ascending radius grid up to FIRST certified pass for each L.
    # This proves only a minimum ON THIS GRID, not a global optimum.
    for length in range(90,141,10):
        found = False
        for radius in range(90,186,5):
            p = Parameters(radius=radius,length=length)
            s = scan(p)
            row = dict(radius_mm=radius,length_mm=length,
                       min_sat_gap_mm=s['min_sat_gap_mm'],
                       sampled_pass=s['violations']==0)
            if s['violations']==0 or -2 < s['min_sat_gap_mm'] < 3:
                cert = certify(p)
                row['continuous_pass'] = cert['continuous_envelope_pass']
                row['refined_min_sat_gap_mm'] = (cert['refined'] or cert['coarse'])['min_sat_gap_mm']
                if row['continuous_pass']:
                    frontier.append(dict(parameters=asdict(p),verification=cert))
                    found = True
            grid.append(row)
            if found:
                break
        print('Grid length',length,'first certified radius',radius if found else None,flush=True)
    if not frontier:
        raise RuntimeError('No feasible grid result; extend search, do not claim success')
    best = min(frontier,key=lambda r:(r['parameters']['radius'],r['parameters']['length']))
    p = Parameters(**best['parameters'])
    # Sensitivity tests: synchronization errors are NOT part of nominal certificate.
    # Thickness and foot-size worst variants rerun complete nominal motion.
    sensitivity = []
    for updates in [dict(foot_radius=30),dict(board_thickness=7,padding=6,strap_extra=5),
                    dict(doll_depth=55),dict(radius=p.radius-1),dict(clearance=3)]:
        values = asdict(p)|updates
        sensitivity.append(dict(updates=updates,verification=certify(Parameters(**values))))
    fine=local_refinement()
    result = dict(units='mm, deg, N m',candidate_parameters=asdict(Parameters()),
                  candidate=candidate,grid=grid,frontier=frontier,selected=best,
                  local_refinement=fine,
                  sensitivity=sensitivity,transmission=transmission(),
                  limitations=['Bounding-envelope rigid geometry only; no structural certificate',
                  'Provisional two-level belt and pulley envelopes included; teeth, tensioners, bearings and detailed contacts absent',
                  'Perfect synchrony and vertical axes; no phase/backlash/load deformation certification',
                  'Grid optimum limited to L=90..140 by 10, R=90..185 by 5 mm',
                  'Doll envelope width/depth constant over 270 mm; straps extra 4 mm assumed',
                  '2 mm total geometric clearance budget is an assumption, not measured print capability'])
    (OUT/'collision_report.json').write_text(json.dumps(result,indent=2)+'\n')
    from .visualize import export
    export(p,OUT)
    from .audit import run as audit
    audit(OUT)
    print('Selected',asdict(p),flush=True)
    print('Continuous lower bound',best['verification']['continuous_clearance_lower_bound_mm'],flush=True)


if __name__ == '__main__':
    run()
