"""V0.3 adds tests without replacing or weakening the original suite."""
import json
from pathlib import Path
import numpy as np
import pytest
from monkey_carousel.v03 import (Design,Body,rectangle,bodies,transformed,
    prism_gaps,exclusion,closing_gap,invariant_circle_clearance,phase_certificate,stop_audit)
from monkey_carousel.gears import comparison,outline
from monkey_carousel.indexing import detent

OUT=Path(__file__).resolve().parents[1]/'outputs'/'v03'


@pytest.fixture(scope='module')
def report():
    return json.loads((OUT/'report.json').read_text())


def test_prism_sat_independent_opencascade_intersections():
    import cadquery as cq
    from scipy.spatial import ConvexHull
    rng=np.random.default_rng(303)
    hits=misses=0
    for _ in range(24):
        objects=[]
        for i in range(2):
            points=rng.normal(size=(9,2))*2
            polygon=points[ConvexHull(points).vertices]
            z0=rng.uniform(-2,2);height=rng.uniform(1,4)
            objects.append(Body(str(i),i,polygon,(z0,z0+height),tuple(rng.uniform(-2,2,2)),float(rng.uniform(-180,180)),1))
        a,b=objects
        gap=prism_gaps(a,b,[13.7])[0]
        solids=[]
        for ob in objects:
            xy=transformed(ob,[13.7])[0]
            solids.append(cq.Workplane('XY').polyline(xy.tolist()).close().extrude(ob.z[1]-ob.z[0]).translate((0,0,ob.z[0])).val())
        volume=solids[0].intersect(solids[1]).Volume()
        assert (gap<-1e-7)==(volume>1e-7)
        hits+=volume>1e-7;misses+=volume<=1e-7
    assert hits>3 and misses>3


def test_height_is_part_of_collision():
    a=Body('a',0,rectangle(0,0,3,3),(0,1))
    b=Body('b',1,rectangle(0,0,3,3),(4,5))
    assert prism_gaps(a,b,[0])[0]==3
    b.z=(.5,1.5)
    assert prism_gaps(a,b,[0])[0]<0


def test_all_angle_post_bound_against_independent_shape_distances():
    from shapely.geometry import Polygon
    d=Design();bs=bodies(d)
    panel=next(b for b in bs if b.name=='door0/panel')
    for name in ('fixed/corner_tube0','fixed/corner_tube1'):
        post=next(b for b in bs if b.name==name)
        bound=invariant_circle_clearance(panel,post)
        stationary=Polygon(transformed(post,[0])[0])
        for angle in np.linspace(-15,375,50):
            assert Polygon(transformed(panel,[angle])[0]).distance(stationary)>=bound-1e-8


def test_closed_wall_is_frame_plus_long_panels(report):
    d=Design();gap=closing_gap(d)
    assert gap['actual_panel_outer_face_length_mm']==167
    lo,hi=gap['robust_closed_gap_range_mm']
    assert 1<=lo<=hi<=3
    assert report['closing']==gap
    assert not closing_gap(Design(phase_deg=1))['closed_appearance_pass']


def test_six_fixed_side_axes_and_orientation():
    d=Design();bs=bodies(d)
    assert len([b for b in bs if 'corner_tube' in b.name])==6
    for b in bs:
        if '/panel' not in b.name:continue
        p=transformed(b,[0,240])
        assert np.linalg.norm(p[0].mean(axis=0)-b.origin)>70
        radial=np.array(b.origin)/d.radius
        front=lambda angle:np.array([-np.sin(np.deg2rad(angle)),np.cos(np.deg2rad(angle))])
        assert front(b.initial)@radial<-.85
        assert front(b.initial-240)@radial>.85


def test_full_report_certifies_declared_uncertainty_only(report):
    from monkey_carousel.v03_study import source_hashes
    assert report['source_sha256']==source_hashes()
    assert report['design']['phase_deg']==.25
    assert report['refined']['samples']==2401
    assert report['refined']['pairs']>7000
    assert report['refined']['penetrating_pair_samples']==0
    assert report['refined']['pass']
    assert report['refined']['continuous_robust_lower_bound_mm']>=1
    cert=phase_certificate(report['refined'],.25)
    assert cert['pass_']
    assert cert['continuous_lower_bound_mm']==pytest.approx(report['refined']['continuous_robust_lower_bound_mm'])


def test_intended_contacts_do_not_hide_corner_posts_or_stop_pairs():
    bs=bodies(Design());by={b.name:b for b in bs}
    assert exclusion(by['door0/panel'],by['fixed/corner_tube0']) is None
    assert exclusion(by['door0/panel'],by['fixed/shaft0']) is None
    assert exclusion(by['drive/sun200'],by['door0/gear40']) is not None
    assert exclusion(by['door0/panel'],by['fixed/stop_closed']) is None


def test_stop_independent_of_teeth_and_blocks_both_overtravel_directions(report):
    r=stop_audit(Design())
    assert r['full_stroke_no_penetration'] and r['geometrically_blocks_overtravel']
    assert max(abs(x) for x in r['min_signed_gap_mm'].values())<1e-7
    assert r==report['stop_audit']


def test_detent_has_two_minima_and_is_reversible_conservative():
    a=np.linspace(0,48,4801);v=detent(a)
    assert v['energy_J'][0]==v['energy_J'][-1]==0
    assert v['energy_J'][2400]>0
    assert v['cam_resisting_torque_Nm'][100]>0
    assert v['cam_resisting_torque_Nm'][-100]<0
    assert np.trapezoid(v['cam_resisting_torque_Nm'],np.deg2rad(a))==pytest.approx(0,abs=1e-12)
    assert np.trapezoid(v['cam_resisting_torque_Nm'][:601],np.deg2rad(a[:601]))==pytest.approx(v['energy_J'][600],rel=1e-5)


def test_reverted_reducers_centers_virtual_work_and_speed_penalty():
    rows=comparison()['gear_schemes']
    direct=rows[0];primary=rows[3]
    assert direct['knob_stroke_deg']==40
    assert primary['knob_stroke_deg']==720
    assert primary['reducer_centers_mm']==[90,90]
    for row in rows:
        assert row['center_distance_mm']==row['axis_radius_mm']
        for c in row['torque_scenarios']:
            work=c['knob_Nm']*np.deg2rad(row['knob_stroke_deg'])*c['efficiency']
            assert work==pytest.approx(6*c['per_door_Nm']*np.deg2rad(240))
    assert direct['torque_scenarios'][1]['knob_Nm']>4


def test_nominal_polyline_gear_meshes_have_separate_evidence(report):
    r=report['gear_profile_audit']
    assert len(r['rows'])==8 and r['step_deg']==.1
    assert r['sampled_pass']
    assert r['continuous_nominal_polyline_profiles_certified']
    for row in r['rows']:
        assert row['samples']==2401 and row['maximum_intersection_volume_mm3']<1e-7


def test_profile_is_valid_and_has_expected_addendum():
    from shapely.geometry import Polygon
    for n in (20,30,40,90,100,200):
        xy=outline(n)
        assert Polygon(xy).is_valid
        assert np.linalg.norm(xy,axis=1).max()==pytest.approx(1.5*(n/2+1))


def test_artifacts_split_bounds_and_no_manufacturing_stl():
    import trimesh
    import cadquery as cq
    from PIL import Image
    for label in ('storage','display'):
        scene=trimesh.load(OUT/f'{label}_research.glb')
        assert len(scene.geometry)==len(bodies(Design()))
        assert all(m.is_watertight for m in scene.geometry.values())
        assert len(cq.importers.importStep(str(OUT/f'{label}_clearance.step')).solids().vals())==len(bodies(Design()))
    assert len(cq.importers.importStep(str(OUT/'panel_split_concept.step')).solids().vals())==2
    plan=json.loads((OUT/'split_plan.json').read_text())
    for part in plan['parts']:
        assert max(part.get('bbox_mm',part.get('max_bbox_mm')))<180
    with Image.open(OUT/'motion.gif') as im:assert im.n_frames==61
    assert not list(OUT.glob('*.stl'))
