"""Generate the V0.4 map: python -m monkey_carousel.freedom_study."""
from dataclasses import asdict
from pathlib import Path
import argparse,csv,hashlib,json
import numpy as np
import shapely
from shapely.geometry import Polygon,Point,box
from .v03 import Design,bodies,transformed
from .freedom import (MapSettings,Region,sweep,local_reserves,fixed_reserves,
                      face_map,union_at,outer_buffer,frame_body,decoration_budgets)

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs'/'v04'
COLORS={'A':(40,185,110,65),'B':(20,190,220,180),'C':(245,170,45,110),
        'D':(225,65,70,95),'E':(90,100,210,150)}


def source_hashes():
    names=['v03.py','model.py','freedom.py','decoration_check.py','freedom_study.py']
    return {n:hashlib.sha256((ROOT/'monkey_carousel'/n).read_bytes()).hexdigest() for n in names}


def serialized(r):
    return dict(name=r.name,z_mm=r.z,category=r.category,verification_level=r.level,
                frame=r.frame,note=r.note,xy_wkb_hex=r.poly.wkb_hex)


def world_regions(d,s):
    dynamic=[];structure=[];conditional=[];evidence=[]
    for b in bodies(d):
        r,ev=sweep(b,d,s);dynamic.append(r);evidence.append(ev)
        if b.actor<0:
            structure.append(Region('structure/'+b.name,outer_buffer(Polygon(transformed(b,[0])[0]),.2),
                                    (b.z[0]-.2,b.z[1]+.2),'E','VERIFIED',note='Existing modeled structure; no attachment approval'))
    for b in decoration_budgets(d,s.thickness_cap_mm):
        r,ev=sweep(b,d,s);r.note+='; planned moving decoration budget, not a current V0.3 part'
        dynamic.append(r);evidence.append(ev)
    for region in fixed_reserves(d):
        (structure if region.category=='E' else conditional).append(region)
    for door in range(6):
        for b,cat,level,note in local_reserves(d,door):
            r,ev=sweep(b,d,s,category=cat);r.level=level;r.note=note;evidence.append(ev)
            (structure if cat=='E' else conditional).append(r)
    return dynamic,structure,conditional,evidence


def voxelize(dregions,eregions,cregions,s):
    lo=np.array(s.roi_min,float);hi=np.array(s.roi_max,float);pitch=s.cell_mm
    count=np.rint((hi-lo)/pitch).astype(int)
    if not np.allclose(lo+count*pitch,hi):raise ValueError('ROI must divide exactly by cell size')
    nx,ny,nz=count
    x,y=np.meshgrid(lo[0]+np.arange(nx)*pitch,lo[1]+np.arange(ny)*pitch)
    cells=shapely.box(x.ravel(),y.ravel(),x.ravel()+pitch,y.ravel()+pitch)
    flags={c:np.zeros((nz,ny,nx),bool) for c in ('D','E','C')}
    for k in range(nz):
        z0=lo[2]+k*pitch;z1=z0+pitch
        for name,regions in [('D',dregions),('E',eregions),('C',cregions)]:
            poly=union_at(regions,z0,z1)
            shapely.prepare(poly)
            flags[name][k]=shapely.intersects(poly,cells).reshape(ny,nx)
    flags['A']=~(flags['D']|flags['E']|flags['C'])
    classes=np.zeros((nz,ny,nx),np.uint8)
    classes[flags['C']]=3;classes[flags['D']]=4;classes[flags['E']]=5
    return flags,classes


def rectangles(mask):
    active={};out=[]
    for y,row in enumerate(mask):
        edges=np.diff(np.r_[False,row,False].astype(int));starts=np.flatnonzero(edges==1);ends=np.flatnonzero(edges==-1)
        current={(int(a),int(b)) for a,b in zip(starts,ends)}
        for key,(y0,y1) in list(active.items()):
            if key not in current:out.append((*key,y0,y1));del active[key]
        for key in current:
            active[key]=(active[key][0],y+1) if key in active else (y,y+1)
    out.extend((*key,*value) for key,value in active.items())
    return out


def compress(mask,s):
    active={};out=[]
    for z,layer in enumerate(mask):
        current=set(rectangles(layer))
        for key,(z0,z1) in list(active.items()):
            if key not in current:out.append((*key,z0,z1));del active[key]
        for key in current:active[key]=(active[key][0],z+1) if key in active else (z,z+1)
    out.extend((*key,*value) for key,value in active.items())
    p=s.cell_mm;o=s.roi_min
    return [(o[0]+x0*p,o[1]+y0*p,o[2]+z0*p,o[0]+x1*p,o[1]+y1*p,o[2]+z1*p)
            for x0,x1,y0,y1,z0,z1 in out]


def heatmap(rows,out):
    from PIL import Image,ImageDraw
    xs=sorted(set(r['x0_mm'] for r in rows));zs=sorted(set(r['z0_mm'] for r in rows))
    w,h=700,690;im=Image.new('RGB',(w,h),'white');draw=ImageDraw.Draw(im)
    draw.text((20,15),f"Door {rows[0]['door']} | local X/Z | outward -Y | mm",fill='black')
    draw.text((20,38),'Maximum CERTIFIED cell thickness; CAP means search limit, not physical maximum',fill='black')
    for r in rows:
        i=xs.index(r['x0_mm']);j=zs.index(r['z0_mm'])
        u=r['max_certified_mm']/r['search_cap_mm']
        color=(int(40+180*u),int(85+140*u),int(220-160*u)) if u>0 else ((180,180,180) if r['verification_level']=='CONDITIONAL' else (230,100,100))
        x0=80+i*32;y0=90+(len(zs)-j-1)*25
        draw.rectangle((x0,y0,x0+31,y0+24),fill=color,outline='white')
        text=f"{r['max_certified_mm']:g}" if u>0 else ('C' if r['verification_level']=='CONDITIONAL' else '0')
        draw.text((x0+2,y0+6),text,fill='black')
    draw.text((30,80),'Z295',fill='black');draw.text((35,580),'Z0',fill='black')
    draw.text((80,605),'X6.5 -> 173.5 mm | cell widths <=10, heights <=15 mm',fill='black')
    draw.text((20,630),'Grey C: unfinished joints/service. Red 0: no certified thickness in this cell.',fill='black')
    draw.text((20,650),'0..40 mm search; whole cell prisms checked, not only their centers.',fill='black')
    im.save(out)


def box_scene(boxes,color):
    import trimesh
    scene=trimesh.Scene()
    for i,b in enumerate(boxes):
        low=np.array(b[:3]);high=np.array(b[3:]);t=np.eye(4);t[:3,3]=(low+high)/2
        mesh=trimesh.creation.box(high-low,t);mesh.visual.face_colors=color
        scene.add_geometry(mesh,node_name=f'cell_union_{i}')
    return scene


def b_boxes(rows):
    # Greedy union in X/Z per equal thickness; boxes have an exact common Y band.
    out=[]
    xs=sorted(set([r['x0_mm'] for r in rows]+[r['x1_mm'] for r in rows]))
    zs=sorted(set([r['z0_mm'] for r in rows]+[r['z1_mm'] for r in rows]))
    for t in sorted(set(r['max_certified_mm'] for r in rows)-{0}):
        mask=np.zeros((len(zs)-1,len(xs)-1),bool)
        for r in rows:
            if r['max_certified_mm']==t:mask[zs.index(r['z0_mm']),xs.index(r['x0_mm'])]=True
        for x0,x1,z0,z1 in rectangles(mask):out.append((xs[x0],-3-t,zs[z0],xs[x1],-3,zs[z1]))
    return out


def export_models(compressed,door_rows,d,s,out):
    import trimesh,cadquery as cq
    (out/'layers').mkdir(exist_ok=True)
    complete=trimesh.Scene();step=cq.Assembly(name='V04_design_freedom_regions')
    for label,boxes in compressed.items():
        scene=box_scene(boxes,COLORS[label]);scene.export(str(out/'layers'/f'{label}.glb'))
        solids=[]
        for i,b in enumerate(boxes):
            mesh=next(iter(box_scene([b],COLORS[label]).geometry.values()))
            complete.add_geometry(mesh,node_name=f'{label}_{i}')
            solids.append(cq.Solid.makeBox(b[3]-b[0],b[4]-b[1],b[5]-b[2],cq.Vector(*b[:3])))
        if solids:
            compound=cq.Compound.makeCompound(solids)
            layer=cq.Assembly(name=f'{label}_regions');layer.add(compound,name=label,color=cq.Color(*[v/255 for v in COLORS[label][:3]]))
            layer.export(str(out/'layers'/f'{label}.step'))
            step.add(compound,name=label,color=cq.Color(*[v/255 for v in COLORS[label][:3]]))
    moving=trimesh.Scene();moving_step=cq.Assembly(name='B_moving_installed_at_zero')
    for door,rows in enumerate(door_rows):
        boxes=b_boxes(rows);local=box_scene(boxes,COLORS['B'])
        local.export(str(out/f'door{door}_safe_local.glb'))
        solids=[];f=frame_body(d,door)
        for i,b in enumerate(boxes):
            solid=cq.Solid.makeBox(b[3]-b[0],b[4]-b[1],b[5]-b[2],cq.Vector(*b[:3]));solids.append(solid)
            mesh=next(iter(box_scene([b],COLORS['B']).geometry.values()))
            angle=np.deg2rad(f.initial);t=trimesh.transformations.rotation_matrix(angle,[0,0,1]);t[:2,3]=f.origin
            mesh.apply_transform(t);moving.add_geometry(mesh,node_name=f'door{door}_patch{i}')
            complete.add_geometry(mesh.copy(),node_name=f'B_door{door}_patch{i}')
        if solids:
            compound=cq.Compound.makeCompound(solids)
            assembly=cq.Assembly(name=f'B_door{door}_local');assembly.add(compound,color=cq.Color(.1,.75,.85))
            assembly.export(str(out/f'door{door}_safe_local.step'))
            global_solid=compound.rotate((0,0,0),(0,0,1),f.initial).translate((*f.origin,0))
            moving_step.add(global_solid,name=f'door{door}',color=cq.Color(.1,.75,.85))
            step.add(global_solid,name=f'B_door{door}',color=cq.Color(.1,.75,.85))
    moving.export(str(out/'moving_safe_installed.glb'));moving_step.export(str(out/'moving_safe_installed.step'))
    complete.export(str(out/'zones.glb'));step.export(str(out/'zones.step'))


def cross_sections(classes,s,out):
    from PIL import Image,ImageDraw
    palette=np.array([(40,185,110),(0,0,0),(20,190,220),(245,170,45),(225,65,70),(90,100,210)],np.uint8)
    for name,data in [('xy_z100',classes[int((100-s.roi_min[2])/s.cell_mm)]),
                      ('xz_y0',classes[:,int((0-s.roi_min[1])/s.cell_mm),:])]:
        im=Image.fromarray(palette[data][::-1]).resize((800,800),resample=Image.Resampling.NEAREST)
        canvas=Image.new('RGB',(850,880),'white');canvas.paste(im,(25,40));dr=ImageDraw.Draw(canvas)
        dr.text((20,10),f'V0.4 {name} | 10mm conservative cells | A green C orange D red E blue',fill='black')
        dr.text((20,850),'B exists in moving local frames; not a globally empty world region.',fill='black')
        canvas.save(out/f'layer_{name}.png')


def column_certificate(d,s):
    # A full circular wrap outside each tube; bore/host are intended contacts.
    from shapely.geometry import Point
    rows=[]
    for i in range(6):
        center=np.array(frame_body(d,i).origin);limit=1e9;limiting=None
        for b in bodies(d):
            if b.name in (f'fixed/corner_tube{i}',f'fixed/shaft{i}'):continue
            zgap=max(30-b.z[1],b.z[0]-270)
            if zgap>=s.clearance_mm+d.surface_error_mm+s.decoration_error_mm:continue
            if b.actor>=0:
                dist=np.linalg.norm(np.array(b.origin)-center);rmax=np.linalg.norm(b.xy,axis=1).max()
                rmin=Polygon(b.xy).distance(Point(0,0));distance=max(rmin-dist,dist-rmax,0)
            else:
                distance=Polygon(transformed(b,[0])[0]).distance(Point(center))
            candidate=distance-s.clearance_mm-d.surface_error_mm-s.decoration_error_mm
            if candidate<limit:limit=candidate;limiting=b.name
        thickness=max(0,float(limit)-d.post_radius)
        rows.append(dict(column=i,z_range_mm=[30,270],maximum_ideal_circular_radial_thickness_mm=thickness,
                         limiting_body=limiting,geometric_level='VERIFIED',assembled_cover_level='CONDITIONAL',
                         practical_PETG_cover_verified=False,
                         note='All-angle circular clearance only. Mounts, tube fit, seams absent; thickness below practical 0.4mm feature target is NOT a printable cover.'))
    return rows


def run(s=MapSettings()):
    OUT.mkdir(exist_ok=True);d=Design();hashes=source_hashes()
    print('V0.4 continuous outer world sweeps',flush=True)
    dr,er,cr,evidence=world_regions(d,s)
    for r in dr:
        x0,y0,x1,y1=r.poly.bounds
        if min(x0-s.roi_min[0],y0-s.roi_min[1],r.z[0]-s.roi_min[2],s.roi_max[0]-x1,s.roi_max[1]-y1,s.roi_max[2]-r.z[1])<0:
            raise ValueError('ROI clips a modeled sweep: enlarge it')
    print('V0.4 full-cell world classification',flush=True)
    flags,classes=voxelize(dr,er,cr,s)
    np.savez_compressed(OUT/'voxel_map.npz',classes=classes,origin_mm=np.array(s.roi_min),cell_mm=s.cell_mm,**{f'mask_{k}':v for k,v in flags.items()})
    compressed={k:compress(v,s) for k,v in flags.items()}
    (OUT/'voxel_boxes.json').write_text(json.dumps(compressed)+'\n')
    door_rows=[];door_evidence=[]
    for door in range(6):
        print('V0.4 door thickness',door,flush=True)
        rows,ev=face_map(d,s,door);door_rows.append(rows);door_evidence.append(ev)
        with (OUT/f'door{door}_thickness_mm.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        heatmap(rows,OUT/f'door{door}_thickness.png')
    from .decoration_check import Placement,from_box,check
    print('V0.4 checker and clip path examples',flush=True)
    examples={}
    for name,size,p in [('fixed_clear',[4,4,4],Placement('fixed',(380,0,100))),
                        ('fixed_collision',[4,4,4],Placement('fixed',(180,0,100))),
                        ('roof_conditional',[4,4,4],Placement('fixed',(230,0,550))),
                        ('clip_insert_flex_remove',[11.6,18,10],Placement('door0',(90,-12,100)))]:
        examples[name]=check(from_box(size,p,d),p,d,s)
    columns=column_certificate(d,s)
    (OUT/'column_wrap_limits.json').write_text(json.dumps(columns,indent=2)+'\n')
    (OUT/'checker_examples.json').write_text(json.dumps(examples,indent=2)+'\n')
    report=dict(version='0.4',design=asdict(d),settings=asdict(s),source_sha256=hashes,
       verification_scope='VERIFIED means geometry only against the declared V0.3 proxies and reservations; never manufacturing/strength validation',
       simultaneous_A_B='Fixed A avoids complete moving decoration budgets; certified B cells are subsets of these budgets. No guarantee for decorations outside them.',
       class_legend=dict(A='fixed decoration: whole world cells outside D/E/C',B='door-local certified extrusion cells; simultaneous budgets',
                         C='conditional tools, fingers, removal and missing parts',D='continuous OUTER motion/error/clearance enclosure',
                         E='structure, missing mounts, fasteners and split seams'),
       roi_volume_mm3=float(np.prod(np.array(s.roi_max)-s.roi_min)),
       independent_mask_volume_mm3={k:int(v.sum())*s.cell_mm**3 for k,v in flags.items()},
       export_box_counts={k:len(v) for k,v in compressed.items()},sweep_evidence=evidence,
       sweeps=[serialized(r) for r in dr],reservations=[serialized(r) for r in er+cr],
       door_evidence=door_evidence,door_summary=[dict(door=i,cells=len(rows),positive_certified_cells=sum(r['max_certified_mm']>0 for r in rows),
          maximum_certified_mm=max(r['max_certified_mm'] for r in rows),cap_limited_cells=sum(r['cap_limited'] for r in rows)) for i,rows in enumerate(door_rows)],
       conditional_cover_spaces=dict(roof='Removable annular dome band; open hand aperture radius85; remove before gear servicing',
                                     columns='Only ideal tiny circular clearance; no printable PETG cover verified',
                                     base='Outer wrap candidate separated from motions; mounts/tools remain conditional'),
       clip=dict(load_scope='light trim only',forbidden_sole_support=['structure','bearings','stops','main transmission'],
                 insertion_translation_mm=[0,-15,0],lateral_flex_mm=.8,release_route='reverse insertion',
                 receiver_geometry='UNVERIFIED',PETG_fatigue_strength='UNVERIFIED',assembled_level='CONDITIONAL'),
       limitations=['40mm is a configurable search cap, not an exact unrestricted maximum',
                    '10mm world voxels are outer for D/E/C and inner for A; B has its own full-cell X/Z resolution',
                    'Unknown future hardware must stay in reservations or maps must be regenerated',
                    'Installation, clips, cover removal and human hand dimensions remain conditional despite geometric envelopes'])
    if source_hashes()!=hashes:raise RuntimeError('Source changed during map computation; regenerate')
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('V0.4 STEP/GLB region exports',report['export_box_counts'],flush=True)
    export_models(compressed,door_rows,d,s,OUT);cross_sections(classes,s,OUT)
    print('V0.4 complete',report['door_summary'],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--settings',type=Path,default=ROOT/'freedom_settings.json')
    a=p.parse_args();run(MapSettings(**json.loads(a.settings.read_text())))
