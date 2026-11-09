"""V0.4 conditional cover spaces and explicit snap insertion/flex/removal paths.

Run AFTER freedom_study. These zones remain CONDITIONAL, even if their installed
geometry and a proposed vertical removal corridor avoid modeled mechanisms.
"""
from dataclasses import asdict
from pathlib import Path
import csv,hashlib,json
import numpy as np
import shapely
from shapely.geometry import MultiPoint
from .freedom import MapSettings,Region,union_at
from .freedom_study import OUT,compress,box_scene,source_hashes
from .v03 import Design


def cover_masks(data,s):
    shape=data['classes'].shape;nz,ny,nx=shape
    x0=s.roi_min[0]+np.arange(nx)*s.cell_mm;y0=s.roi_min[1]+np.arange(ny)*s.cell_mm
    x,y=np.meshgrid(x0,y0);p=s.cell_mm
    dx=np.maximum(np.maximum(x,-x-p),0);dy=np.maximum(np.maximum(y,-y-p),0)
    rmin=np.hypot(dx,dy)
    rmax=np.maximum.reduce([np.hypot(x,y),np.hypot(x+p,y),np.hypot(x,y+p),np.hypot(x+p,y+p)])
    roof=np.zeros(shape,bool);base=np.zeros(shape,bool)
    occupied=data['mask_D']|data['mask_E']
    for k in range(nz):
        lo=s.roi_min[2]+k*p;hi=lo+p
        if 340<=lo and hi<=500:inner,outer=245,275
        elif 500<=lo and hi<=580:
            # Complete cell fits a sloping annular band (not only its center).
            inner=275-1.5*(lo-500)-30;outer=275-1.5*(hi-500)
        elif 580<=lo and hi<=590:inner,outer=85,155
        else:inner,outer=1,0
        target=(rmin>=inner)&(rmax<=outer)
        end=k+int(np.ceil(220/p))+1
        if end<=nz:
            route_free=~occupied[k:end].any(axis=0)
            roof[k]=target&route_free
        if -60<=lo and hi<=-30:
            base[k]=(rmin>=230)&(rmax<=250)&~occupied[k]
    return roof,base


def annulus_mesh(center,r0,r1,z0,z1,n=96):
    import trimesh
    a=np.arange(n)*2*np.pi/n
    r0=r0/np.cos(np.pi/n)  # hole OUTER polygon: mesh stays inside safe annulus
    rings=[np.c_[center[0]+r*np.cos(a),center[1]+r*np.sin(a),np.full(n,z)]
           for z,r in [(z0,r0),(z0,r1),(z1,r0),(z1,r1)]]
    faces=[]
    for i in range(n):
        j=(i+1)%n
        for q in [(i,j,n+j,n+i),(2*n+i,3*n+i,3*n+j,2*n+j),
                  (i,2*n+i,2*n+j,j),(n+i,n+j,3*n+j,3*n+i)]:
            faces.extend([[q[0],q[1],q[2]],[q[0],q[2],q[3]]])
    mesh=trimesh.Trimesh(np.vstack(rings),faces,process=True);mesh.fix_normals();return mesh


def transparent_layers():
    """glTF needs explicit BLEND; vertex alpha alone defaults to OPAQUE."""
    import trimesh
    for path in OUT.rglob('*.glb'):
        scene=trimesh.load(path,force='scene')
        for mesh in scene.geometry.values():
            if hasattr(mesh.visual,'face_colors'):
                color=mesh.visual.face_colors[0]
            else:color=mesh.visual.material.main_color
            material=trimesh.visual.material.PBRMaterial(baseColorFactor=color,alphaMode='BLEND',doubleSided=True)
            mesh.visual=trimesh.visual.TextureVisuals(material=material)
        scene.export(str(path))


def publication_heatmaps():
    """Final standalone figures: physical mm axes, colorbar, vector SVG."""
    import os
    os.environ.setdefault('MPLCONFIGDIR','/tmp/monkey_carousel_mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    for door in range(6):
        rows=list(csv.DictReader((OUT/f'door{door}_thickness_mm.csv').open()))
        cap=float(rows[0]['search_cap_mm'])
        xs=sorted(set([float(r['x0_mm']) for r in rows]+[float(r['x1_mm']) for r in rows]))
        zs=sorted(set([float(r['z0_mm']) for r in rows]+[float(r['z1_mm']) for r in rows]))
        values=np.zeros((len(zs)-1,len(xs)-1));status=np.zeros_like(values)
        for r in rows:
            j=zs.index(float(r['z0_mm']));i=xs.index(float(r['x0_mm']))
            values[j,i]=float(r['max_certified_mm']);status[j,i]=r['verification_level']!='CONDITIONAL'
        fig,ax=plt.subplots(figsize=(8,9),dpi=110,layout='constrained')
        ax.pcolormesh(xs,zs,status,cmap=ListedColormap(['#bbbbbb','#e97878']),vmin=0,vmax=1,edgecolors='white',linewidth=.4)
        field=ax.pcolormesh(xs,zs,np.ma.masked_where(values==0,values),cmap='viridis',vmin=0,vmax=cap,edgecolors='white',linewidth=.4)
        for r in rows:
            value=float(r['max_certified_mm'])
            label=(f'{value:g}'+('*' if r['cap_limited']=='True' else '')) if value>0 else ('C' if r['verification_level']=='CONDITIONAL' else '0')
            ax.text((float(r['x0_mm'])+float(r['x1_mm']))/2,(float(r['z0_mm'])+float(r['z1_mm']))/2,label,ha='center',va='center',fontsize=5.5,color='black')
        ax.set(xlabel='Door-local X (mm)',ylabel='Door-local Z (mm)',aspect='equal',
               title=f'Door {door}: maximum CERTIFIED outward thickness\nOutward normal -Y; whole cells, not sample points')
        fig.colorbar(field,ax=ax,label=f'Certified thickness (mm); * = {cap:g} mm search cap',shrink=.75)
        ax.legend(handles=[Patch(color='#bbbbbb',label='C: unfinished joints / service'),Patch(color='#e97878',label='0: no positive thickness certificate')],loc='upper center',bbox_to_anchor=(.5,-.09),fontsize=7)
        fig.savefig(OUT/f'door{door}_thickness.png')
        fig.savefig(OUT/f'door{door}_thickness.svg');plt.close(fig)


def run():
    import trimesh,cadquery as cq
    report=json.loads((OUT/'report.json').read_text())
    if report['source_sha256']!=source_hashes():raise RuntimeError('Stale core map; regenerate or refresh validated checker evidence first')
    s=MapSettings(**report['settings']);d=Design(**report['design'])
    data=np.load(OUT/'voxel_map.npz');roof,base=cover_masks(data,s)
    np.savez_compressed(OUT/'conditional_covers.npz',roof=roof,base=base)
    manifest={}
    base_boxes=[]
    for name,mask in [('roof_dome',roof),('base_wrap',base)]:
        boxes=compress(mask,s);scene=box_scene(boxes,(245,170,45,180))
        if name=='base_wrap':base_boxes=boxes
        scene.export(str(OUT/f'{name}_conditional.glb'))
        solids=[cq.Solid.makeBox(b[3]-b[0],b[4]-b[1],b[5]-b[2],cq.Vector(*b[:3])) for b in boxes]
        a=cq.Assembly(name=name+'_CONDITIONAL')
        if solids:a.add(cq.Compound.makeCompound(solids),name='conditional_volume',color=cq.Color(.96,.66,.18))
        if solids:a.export(str(OUT/f'{name}_conditional.step'))
        manifest[name]=dict(verification_level='CONDITIONAL',volume_mm3=int(mask.sum())*s.cell_mm**3,
            installed_modeled_geometry_clear=True,requires_dome_removal_for_service=name=='roof_dome',
            removal=name=='roof_dome' and '220mm vertical lift of the entire cell-volume: complete straight corridor checked against D/E',
            note='Not fixed A: unresolved clips, mounts, tools and human clearances; no permanent enclosure')
    # Each base patch belongs to one of twelve removable sector modules. The
    # convex hull of a box and its translated copy encloses EVERY intermediate
    # straight translation, not just installed/removed snapshots.
    routes=[];scene=trimesh.Scene();a=cq.Assembly(name='base_removal_paths_CONDITIONAL')
    from .v03_visual import prism_mesh
    obstacles=[Region(r['name'],shapely.from_wkb(r['xy_wkb_hex']),tuple(r['z_mm']),r['category'],r['verification_level'])
               for r in report['sweeps']+report['reservations'] if r['category'] in ('D','E')]
    for i,b in enumerate(base_boxes):
        corners=np.array([[b[0],b[1]],[b[3],b[1]],[b[3],b[4]],[b[0],b[4]]])
        center=corners.mean(axis=0);sector_index=int(np.rint(np.rad2deg(np.arctan2(center[1],center[0]))/30))%12
        angle=np.deg2rad(sector_index*30);translation=40*np.array([np.cos(angle),np.sin(angle)])
        poly=MultiPoint(np.vstack([corners,corners+translation])).convex_hull
        if poly.intersects(union_at(obstacles,b[2],b[5])):
            raise RuntimeError('Proposed base removal corridor hits D/E; change segmentation')
        xy=np.asarray(poly.exterior.coords)[:-1]
        mesh=prism_mesh(xy,(b[2],b[5]));mesh.visual.face_colors=(245,170,45,130)
        scene.add_geometry(mesh,node_name=f'sector{sector_index}_route{i}')
        solid=cq.Workplane('XY').polyline(xy.tolist()).close().extrude(b[5]-b[2]).translate((0,0,b[2]))
        a.add(solid,name=f'route{i}',color=cq.Color(.96,.66,.18))
        routes.append(dict(sector=sector_index,source_box_mm=b,translation_mm=[*translation,0],
                           path_xy_wkb_hex=poly.wkb_hex,z_mm=[b[2],b[5]],verification_level='CONDITIONAL'))
    scene.export(str(OUT/'base_wrap_removal_paths.glb'));a.export(str(OUT/'base_wrap_removal_paths.step'))
    manifest['base_wrap']['removal_routes']=routes
    manifest['base_wrap']['removal_note']='Sector modules translate40mm outward; continuous convex corridors clear modeled D/E. Joints, latches and unknown fasteners remain unresolved.'
    columns=json.loads((OUT/'column_wrap_limits.json').read_text())
    scene=trimesh.Scene();a=cq.Assembly(name='column_ideal_clearance_CONDITIONAL_NOT_PRINT_RELEASE')
    for row in columns:
        i=row['column'];phi=i*np.pi/3;center=d.radius*np.array([np.cos(phi),np.sin(phi)])
        t=np.floor(max(0,row['maximum_ideal_circular_radial_thickness_mm']-1e-6)/.025)*.025
        row['exported_ideal_thickness_mm']=float(t)
        if t>0:
            mesh=annulus_mesh(center,5,5+t,30,270);mesh.visual.face_colors=(245,170,45,220)
            scene.add_geometry(mesh,node_name=f'column{i}_ideal_only')
            solid=cq.Workplane('XY').circle(5+t).circle(5).extrude(240).translate((*center,30))
            a.add(solid,name=f'column{i}',color=cq.Color(.96,.66,.18))
    if scene.geometry:
        scene.export(str(OUT/'column_wrap_ideal_only.glb'));a.export(str(OUT/'column_wrap_ideal_only.step'))
    manifest['column_wrap']=dict(verification_level='CONDITIONAL',practical_PETG_cover_verified=False,
                                ideal_only_columns=columns,installation_removal='UNVERIFIED: no practical PETG cover exists yet',
                                note='Thin radial clearance is not a printable wall or a tested attachment')
    # Explicit path envelope: translations [0,-15,0] plus lateral flex +/-0.8.
    clip_boxes={'installed':(85,-6,95,95,-3,105),
                'insertion_flex_removal':(84.2,-21,95,95.8,-3,105)}
    for frame in ('local','world_at_zero'):
        scene=trimesh.Scene();a=cq.Assembly(name='clip_paths_CONDITIONAL_'+frame)
        for name,b in clip_boxes.items():
            mesh=next(iter(box_scene([b],(245,170,45,150)).geometry.values()))
            solid=cq.Solid.makeBox(b[3]-b[0],b[4]-b[1],b[5]-b[2],cq.Vector(*b[:3]))
            if frame=='world_at_zero':
                T=trimesh.transformations.rotation_matrix(np.deg2rad(120),[0,0,1]);T[:2,3]=(180,0)
                mesh.apply_transform(T);solid=solid.rotate((0,0,0),(0,0,1),120).translate((180,0,0))
            scene.add_geometry(mesh,node_name=name);a.add(solid,name=name,color=cq.Color(.96,.66,.18))
        scene.export(str(OUT/f'clip_paths_{frame}.glb'));a.export(str(OUT/f'clip_paths_{frame}.step'))
    manifest['clip']=dict(verification_level='CONDITIONAL',boxes_local_mm=clip_boxes,
        insertion_translation_mm=[0,-15,0],flex_mm=[-.8,.8],flex_axis='local X',
        note='Envelope includes all straight insertion/removal and lateral flex positions; receiver, retention, forces and PETG fatigue absent. Never sole structure/bearing/stop/transmission support.')
    manifest['source_sha256']=source_hashes()|{'freedom_covers.py':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    with (OUT/'clip_candidate_sites.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['door','local_x_mm','local_y_mm','local_z_mm','insertion_mm','flex_mm','verification_level','receiver','strength'])
        for i in range(6):writer.writerow([i,90,-4.5,100,15,.8,'CONDITIONAL','UNVERIFIED','UNVERIFIED'])
    levels=dict(A_fixed='VERIFIED',B_moving='VERIFIED',C_aggregate='UNVERIFIED',
                D='VERIFIED',E_aggregate='UNVERIFIED',roof_dome='CONDITIONAL',base_wrap='CONDITIONAL',
                column_wrap='CONDITIONAL',clip_candidate='CONDITIONAL',
                note='A/B/D levels refer only to declared continuous geometry and error bounds. C/E source-level distinctions are in report.reservations; aggregate does not upgrade any concept. No strength or manufacture release.')
    (OUT/'verification_levels.json').write_text(json.dumps(levels,indent=2)+'\n')
    from .v03 import bodies
    inventory=dict(modeled_motion=[dict(name=b.name,actor=b.actor,rate=b.rate,
        geometry='V0.3 proxy, not final manufactured part',
        incomplete_contacts_or_parts=any(k in b.name for k in ('clutch','plunger','detent','stop','bearing')),
        envelope_layer='D; incomplete details additionally reserved in C/E') for b in bodies(d)],
        reserved_only=[dict(name=r['name'],layer=r['category'],level=r['verification_level'],note=r['note'])
                       for r in report['reservations'] if r['verification_level']!='VERIFIED'])
    (OUT/'motion_inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
    (OUT/'cover_and_clip_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    transparent_layers()
    publication_heatmaps()
    print('Conditional covers and clip paths complete',manifest['roof_dome']['volume_mm3'],manifest['base_wrap']['volume_mm3'])


if __name__=='__main__':run()
