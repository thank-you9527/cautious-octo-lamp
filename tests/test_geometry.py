import json
from pathlib import Path
import numpy as np
import pytest
from monkey_carousel.model import (Parameters,boxes,poses,scan,certify,
                                  sat_separations,allowed_pair,sweep_radius)


def sat_two(c1,c2,s1=(2,2,2),s2=(2,2,2),a1=0,a2=0):
    return sat_separations(np.array([[c1,c2]],float),np.array([[a1,a2]]),
                           np.array([s1,s2],float),[(0,1)])[0,0]


def test_z_separation_is_not_top_view_collision():
    assert sat_two((0,0,0),(0,0,5)) == 3
    assert sat_two((0,0,0),(0,0,1)) == -1


def test_touch_overlap_and_separation():
    assert sat_two((0,0,0),(2,0,0)) == 0
    assert sat_two((0,0,0),(1,0,0)) < 0
    assert sat_two((0,0,0),(4,0,0)) == 2


def test_rotated_non_axis_aligned_interference():
    assert sat_two((0,0,0),(2,0,0),a2=np.pi/4)<0
    assert sat_two((0,0,0),(2.5,0,0),a2=np.pi/4)>0


def test_sat_against_independent_opencascade_solids():
    import cadquery as cq
    rng = np.random.default_rng(9527)
    overlap=separate=0
    for _ in range(45):
        centers = rng.uniform(-2,2,(2,3))
        sizes = rng.uniform(.7,4,(2,3))
        yaw = rng.uniform(-np.pi,np.pi,2)
        gap = sat_two(*centers,*sizes,*yaw)
        solids = [cq.Workplane('XY').box(*sizes[i])
                  .rotate((0,0,0),(0,0,1),float(np.rad2deg(yaw[i])))
                  .translate(tuple(centers[i])).val() for i in range(2)]
        volume = solids[0].intersect(solids[1]).Volume()
        assert (gap < -1e-8) == (volume > 1e-8)
        overlap += gap < 0
        separate += gap > 0
    assert overlap>5 and separate>5  # Exercise both outcomes, not a zero-hit test.


def test_fixed_side_hinges_and_orientation():
    p=Parameters(radius=140,length=90)
    bs=boxes(p)
    c,a=poses(p,bs,[0,120,240])
    for j,b in enumerate(bs):
        if b.group<0:
            assert np.allclose(c[:,j],b.center)
        if '/hub10' in b.name:
            assert np.allclose(c[:,j,:2],c[0,j,:2])
        if '/board' in b.name:
            radial=np.deg2rad(b.group*60)
            front0=np.array([-np.sin(a[0,j]),np.cos(a[0,j])])
            front1=np.array([-np.sin(a[-1,j]),np.cos(a[-1,j])])
            r=np.array([np.cos(radial),np.sin(radial)])
            assert front0@r < -.85 and front1@r > .85
            assert np.isclose(a[0,j]-a[-1,j],np.deg2rad(240))


def test_collision_filters_do_not_exempt_other_doors_or_fixed_parts():
    bs=boxes(Parameters())
    b=next(x for x in bs if x.name=='door0/board')
    assert not allowed_pair(b,next(x for x in bs if x.name=='fixed/shaft0'))
    for name in ['fixed/shaft1','fixed/roof','fixed/column','door1/board']:
        assert allowed_pair(b,next(x for x in bs if x.name==name))


def test_candidate_does_not_meet_clearance():
    result=certify(Parameters())
    assert not result['continuous_envelope_pass']
    assert result['refined']['samples']==2401
    assert result['refined']['min_sat_gap_mm'] < 2
    from monkey_carousel.audit import distance
    witness=result['refined']
    assert 0 < distance(Parameters(),witness['angle'],witness['pair']) < 2


def test_selected_entire_travel_has_continuous_certificate():
    p=Parameters(radius=140,length=90)
    result=certify(p)
    assert result['coarse']['samples']==241
    assert result['coarse']['pairs_per_sample']>1000
    assert result['continuous_envelope_pass']
    assert result['continuous_clearance_lower_bound_mm'] >= 2
    assert not result['coarse']['violations']
    # Independent off-grid checks supplement the analytic displacement bound.
    extra=scan(p,np.linspace(.37,239.37,100))
    assert extra['min_sat_gap_mm']>=2


def test_nonfeasible_configuration_really_penetrates():
    result=scan(Parameters(radius=100,length=90))
    assert result['penetrating_pairs_samples']>0


def test_sweep_bound_contains_all_moving_vertices():
    from monkey_carousel.visualize import corners
    p=Parameters(radius=140,length=90)
    bs=boxes(p)
    c,a=poses(p,bs,np.arange(0,241,13))
    bound=p.radius+sweep_radius(p)
    for t in range(len(c)):
        for j,b in enumerate(bs):
            if b.group>=0:
                vertices=corners(c[t,j],b.size,a[t,j])
                assert np.linalg.norm(vertices[:,:2],axis=1).max()<=bound+1e-8


def test_exported_artifacts_are_valid_3d_envelopes():
    import trimesh
    import cadquery as cq
    out=Path(__file__).resolve().parents[1]/'outputs'
    for label in ['storage','display']:
        scene=trimesh.load(out/f'{label}_envelopes.glb')
        assert len(scene.geometry)==len(boxes(Parameters()))
        assert all(m.is_watertight for m in scene.geometry.values())
        shape=cq.importers.importStep(str(out/f'{label}_envelopes.step'))
        assert len(shape.solids().vals())==len(boxes(Parameters()))
    from PIL import Image
    with Image.open(out/'motion.gif') as im:
        assert im.n_frames==61
    assert not list(out.glob('*.stl'))


def test_report_matches_current_geometry():
    path=Path(__file__).resolve().parents[1]/'outputs/collision_report.json'
    report=json.loads(path.read_text())
    p=Parameters(**report['selected']['parameters'])
    s=scan(p)
    assert s['pairs_per_sample']==report['selected']['verification']['coarse']['pairs_per_sample']
    assert s['min_sat_gap_mm']==pytest.approx(report['selected']['verification']['coarse']['min_sat_gap_mm'])


def test_invalid_width_is_rejected():
    with pytest.raises(ValueError):
        Parameters(length=80)


def test_transmission_ratio_and_virtual_work_estimates():
    from monkey_carousel.study import transmission
    bs=boxes(Parameters())
    _,yaw=poses(Parameters(),bs,[0,240])
    input_idx=next(i for i,b in enumerate(bs) if b.name=='input20T')
    door_idx=next(i for i,b in enumerate(bs) if b.name=='door0/board')
    assert (yaw[1,input_idx]-yaw[0,input_idx]) == pytest.approx(
        3*(yaw[1,door_idx]-yaw[0,door_idx]))
    for row in transmission()['cases']:
        input_work=row['knob_torque_Nm']*np.deg2rad(row['knob_travel_deg'])
        output_work=6*row['door_torque_Nm']*np.deg2rad(240)
        assert input_work*row['efficiency']==pytest.approx(output_work)
    assert 6*140+2*np.pi*(40/np.pi)==pytest.approx(920)
