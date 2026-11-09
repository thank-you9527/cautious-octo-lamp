"""V0.4 proofs, integration and adversarial decoration input checks."""
import csv,json
from pathlib import Path
import numpy as np
import pytest
import shapely
from shapely.geometry import Polygon,Point,box
from monkey_carousel.v03 import Design,bodies,transformed
from monkey_carousel.freedom import (MapSettings,sweep,outer_buffer,frame_body,
                                    decoration_budgets,relative_vertices,face_map)
from monkey_carousel.freedom_study import OUT,source_hashes
from monkey_carousel.decoration_check import Placement,from_box,from_shape,from_step,check


@pytest.fixture(scope='module')
def report():return json.loads((OUT/'report.json').read_text())


@pytest.fixture(scope='module')
def voxels():return np.load(OUT/'voxel_map.npz')


def test_outer_buffer_does_not_use_inscribed_arc_as_certificate():
    poly=outer_buffer(Point(0,0),2)
    for a in np.linspace(0,2*np.pi,721):
        assert poly.covers(Point(2*np.cos(a),2*np.sin(a)))
    with pytest.raises(ValueError):outer_buffer(Point(0,0),-1)


def test_world_sweep_contains_unsampled_phase_and_error_positions():
    d=Design();s=MapSettings();b=next(b for b in bodies(d) if b.name=='door0/panel')
    r,e=sweep(b,d,s)
    assert e['time_padding_mm']>0 and e['phase_padding_mm']>0
    for theta in (.371,1.499,37.781,119.499,239.501):
        for phase in (-.25,.25):
            xy=transformed(b,[theta],phase)[0]
            assert r.poly.covers(Polygon(xy))
            for vertex in xy:
                for offset in ((.2,0),(-.2,0),(0,.2),(0,-.2)):
                    assert r.poly.covers(Point(vertex+offset))


def test_relative_sweep_covers_independent_phases_not_only_synchronous_error():
    d=Design();s=MapSettings();frame=frame_body(d,0)
    other=next(b for b in bodies(d) if b.name=='door1/panel')
    region,e=sweep(other,d,s,frame)
    assert e['phase_coefficient_mm_per_radian']>180
    for theta in (.37,33.77,119.87,239.73):
        for pa,pb in ((-.25,.25),(.25,-.25),(.13,-.21)):
            xy=transformed(other,[theta],pb)[0]-frame.origin
            a=np.deg2rad(frame.initial+frame.rate*(theta+pa))
            local=xy@np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
            assert region.poly.covers(Polygon(local))


def test_same_door_phase_cancels_without_ignoring_own_hardware():
    d=Design();f=frame_body(d,0);b=next(b for b in bodies(d) if b.name=='door0/upper_tab')
    r,e=sweep(b,d,MapSettings(),f)
    assert e['samples']==1 and e['time_padding_mm']==e['phase_padding_mm']==0
    assert r.poly.covers(Polygon(relative_vertices(b,[19.3],f)[0]))


def test_height_prefilter_uses_declared_clearance_not_a_hardcoded_margin():
    _,e=face_map(Design(),MapSettings(clearance_mm=50),0)
    assert 'fixed/roof' not in e['z_rejected_bodies']
    assert 'fixed/base' not in e['z_rejected_bodies']


def test_source_parameters_and_all_modeled_bodies_are_recorded(report):
    assert report['source_sha256']==source_hashes()
    settings=json.loads((Path(__file__).resolve().parents[1]/'freedom_settings.json').read_text())
    assert report['settings']==settings
    names={r['name'] for r in report['sweeps']}
    assert names>={b.name for b in bodies(Design())}
    assert any(n.startswith('budget/') for n in names)
    assert report['design']['phase_deg']==.25 and report['design']['surface_error_mm']==.2


def test_D_voxels_cover_off_grid_motion_vertices(report,voxels):
    s=MapSettings(**report['settings']);mask=voxels['mask_D']
    original={b.name:b for b in bodies(Design())+decoration_budgets(Design(),s.thickness_cap_mm)}
    for name,b in original.items():
        theta=37.413;phase=.25 if 0<=b.actor<6 else 0
        xy=transformed(b,[theta],phase)[0]
        for z in b.z:
            for point in xy[::max(1,len(xy)//4)]:
                index=np.floor((np.r_[point,z]-s.roi_min)/s.cell_mm).astype(int)
                assert mask[index[2],index[1],index[0]],name


def test_A_cells_are_whole_volume_safe_not_center_only(report,voxels):
    s=MapSettings(**report['settings']);k=int((100-s.roi_min[2])/s.cell_mm)
    a=voxels['mask_A'][k];danger=voxels['mask_D'][k]|voxels['mask_E'][k]|voxels['mask_C'][k]
    edge=a&(np.roll(danger,1,0)|np.roll(danger,-1,0)|np.roll(danger,1,1)|np.roll(danger,-1,1))
    indices=np.argwhere(edge)
    assert len(indices)>5
    active=shapely.union_all([shapely.from_wkb(r['xy_wkb_hex']) for r in report['sweeps']+report['reservations'] if r['z_mm'][0]<=110 and r['z_mm'][1]>=100])
    for y,x in indices[::max(1,len(indices)//20)]:
        x0=s.roi_min[0]+x*s.cell_mm;y0=s.roi_min[1]+y*s.cell_mm
        assert not active.intersects(box(x0,y0,x0+s.cell_mm,y0+s.cell_mm))


def test_compressed_exports_exactly_reconstruct_voxel_masks(report,voxels):
    boxes=json.loads((OUT/'voxel_boxes.json').read_text());s=MapSettings(**report['settings'])
    for name in ('A','C','D','E'):
        actual=np.zeros_like(voxels[f'mask_{name}'])
        for b in boxes[name]:
            lo=np.rint((np.array(b[:3])-s.roi_min)/s.cell_mm).astype(int)
            hi=np.rint((np.array(b[3:])-s.roi_min)/s.cell_mm).astype(int)
            actual[lo[2]:hi[2],lo[1]:hi[1],lo[0]:hi[0]]=True
        assert np.array_equal(actual,voxels[f'mask_{name}'])


def test_unfinished_corner_mounts_are_structure_E_not_only_service_C(voxels):
    # Boundary-touching full voxel at [200,210]x[20,30]x[-40,-30] hits the
    # unfinished OD-corner mounting reservation, outside the bearing housing.
    assert voxels['mask_E'][4,42,60]
    assert voxels['classes'][4,42,60]==5


def test_six_thickness_csvs_are_nonuniform_and_cap_is_explicit(report):
    for door in range(6):
        rows=list(csv.DictReader((OUT/f'door{door}_thickness_mm.csv').open()))
        assert len(rows)==340
        values=[float(r['max_certified_mm']) for r in rows]
        assert min(values)==0 and max(values)>3
        assert any(r['cap_limited']=='True' for r in rows)
        for r in rows:
            if r['verification_level']!='VERIFIED':assert float(r['max_certified_mm'])==0
            if float(r['max_certified_mm'])>0:
                assert float(r['x0_mm'])>=27 and float(r['x1_mm'])<=153
                assert any(float(r['z0_mm'])>=a and float(r['z1_mm'])<=b for a,b in ((22,135.5),(159.5,273)))


def test_certified_entire_door_cell_passes_independent_checker():
    rows=list(csv.DictReader((OUT/'door0_thickness_mm.csv').open()))
    row=next(r for r in rows if float(r['max_certified_mm'])==40)
    x0,x1,z0,z1,t=[float(row[k]) for k in ('x0_mm','x1_mm','z0_mm','z1_mm','max_certified_mm')]
    p=Placement('door0',((x0+x1)/2,-3-t/2,(z0+z1)/2))
    result=check(from_box((x1-x0,t,z1-z0),p),p)
    assert result['geometry_pass'] and result['verification_level']=='VERIFIED'


@pytest.mark.parametrize('name,level,verdict',[
    ('fixed_clear','VERIFIED','CERTIFIED_CLEAR'),
    ('fixed_collision','UNVERIFIED','ENVELOPE_INTERFERENCE'),
    ('roof_conditional','CONDITIONAL','CERTIFIED_CLEAR'),
    ('clip_insert_flex_remove','CONDITIONAL','CERTIFIED_CLEAR')])
def test_example_verdicts_keep_geometry_and_reservations_distinct(name,level,verdict):
    result=json.loads((OUT/'checker_examples.json').read_text())[name]
    assert result['verification_level']==level and result['geometry_verdict']==verdict
    assert 'limiting_sample_command_deg' in result['limiting_pair']


def test_STEP_and_parametric_workplane_inputs_keep_all_solids(tmp_path):
    import cadquery as cq
    safe=cq.Solid.makeBox(4,4,4,cq.Vector(378,-2,98))
    unsafe=cq.Solid.makeBox(4,4,4,cq.Vector(178,-2,98))
    workplane=cq.Workplane('XY').newObject([safe,unsafe])
    parts=from_shape(workplane)
    assert len(parts)==2
    assert not check(parts)['geometry_pass']
    path=tmp_path/'two_solids.step';cq.exporters.export(cq.Compound.makeCompound([safe,unsafe]),str(path))
    imported=from_step(path)
    assert len(imported)==2 and not check(imported)['geometry_pass']


def test_arbitrary_STEP_holes_are_conservative_not_false_safety(tmp_path):
    import cadquery as cq
    ring=cq.Workplane('XY').circle(9).circle(7).extrude(5).translate((180,0,100))
    path=tmp_path/'ring.step';cq.exporters.export(ring,str(path))
    result=check(from_step(path))
    assert not result['geometry_pass']  # Per-solid AABB fills hole: honest rejection.
    assert 'AABB' in result['scope']


def test_host_contact_cannot_hide_a_part_embedded_in_the_panel():
    p=Placement('door0',(90,0,50))
    result=check(from_box((10,2,10),p),p)
    assert not result['geometry_pass']
    assert not result['intended_contacts']


def test_cover_removal_is_continuous_vertical_prism_not_discrete_poses(voxels):
    manifest=json.loads((OUT/'cover_and_clip_manifest.json').read_text())
    data=np.load(OUT/'conditional_covers.npz');s=MapSettings()
    assert data['roof'].sum()>0 and data['base'].sum()>0
    occupied=voxels['mask_D']|voxels['mask_E']
    for k in range(len(data['roof'])):
        if data['roof'][k].any():
            assert not (data['roof'][k]&occupied[k:k+23].any(axis=0)).any()
            assert np.all(voxels['mask_C'][k:k+23][:,data['roof'][k]])
    assert manifest['roof_dome']['verification_level']=='CONDITIONAL'
    assert not manifest['column_wrap']['practical_PETG_cover_verified']
    assert all(x['maximum_ideal_circular_radial_thickness_mm']<.4 for x in manifest['column_wrap']['ideal_only_columns'])


def test_clip_envelope_covers_every_linear_insertion_flex_combination():
    m=json.loads((OUT/'cover_and_clip_manifest.json').read_text())['clip']
    bounds=m['boxes_local_mm']['insertion_flex_removal']
    for translation in np.linspace(-15,0,13):
        for flex in np.linspace(-.8,.8,9):
            low=np.array([85+flex,-6+translation,95]);high=np.array([95+flex,-3+translation,105])
            assert np.all(low>=np.array(bounds[:3])-1e-9) and np.all(high<=np.array(bounds[3:])+1e-9)
    assert m['verification_level']=='CONDITIONAL' and 'receiver' in m['note']


def test_base_removal_envelopes_contain_continuous_linear_paths():
    m=json.loads((OUT/'cover_and_clip_manifest.json').read_text())['base_wrap']
    assert m['removal_routes']
    for row in m['removal_routes'][::max(1,len(m['removal_routes'])//10)]:
        b=row['source_box_mm'];p=shapely.from_wkb(row['path_xy_wkb_hex'])
        v=np.array(row['translation_mm'][:2])
        for t in (.13,.37,.91):
            q=box(b[0]+v[0]*t,b[1]+v[1]*t,b[3]+v[0]*t,b[4]+v[1]*t)
            assert p.buffer(1e-7).covers(q)
    assert m['verification_level']=='CONDITIONAL'
    report=json.loads((OUT/'report.json').read_text())
    reserved=next(r for r in report['reservations'] if r['name']=='base_fasteners_and_tools')
    reserved_xy=shapely.from_wkb(reserved['xy_wkb_hex'])
    for row in m['removal_routes']:
        assert reserved_xy.covers(shapely.from_wkb(row['path_xy_wkb_hex']))
        assert reserved['z_mm'][0]<=row['z_mm'][0] and reserved['z_mm'][1]>=row['z_mm'][1]


def test_region_artifacts_and_no_production_stl():
    import trimesh
    import cadquery as cq
    from PIL import Image
    for name in ('zones','moving_safe_installed','roof_dome_conditional','base_wrap_conditional','clip_paths_local'):
        scene=trimesh.load(OUT/f'{name}.glb')
        assert scene.geometry and all(m.is_watertight for m in scene.geometry.values())
        assert all(m.visual.material.alphaMode=='BLEND' for m in scene.geometry.values())
        assert (OUT/f'{name}.step').stat().st_size>1000
    assert cq.importers.importStep(str(OUT/'door0_safe_local.step')).solids().vals()
    for door in range(6):
        with Image.open(OUT/f'door{door}_thickness.png') as im:assert im.width>=700 and im.height>=690
        assert (OUT/f'door{door}_thickness.svg').stat().st_size>1000
    assert not list(OUT.rglob('*.stl'))


def test_reject_invalid_inputs_and_unknown_host():
    with pytest.raises(ValueError):MapSettings(angular_step_deg=2)
    with pytest.raises(ValueError):from_box((1,-1,1))
    with pytest.raises(ValueError):from_box((1,1,1),Placement('door8'))
    with pytest.raises(ValueError):check(from_box((1,1,1)),Placement(host='missing/component'))
