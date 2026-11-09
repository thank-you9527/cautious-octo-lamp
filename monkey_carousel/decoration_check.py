"""V0.4 automatic decoration check: boxes, CadQuery shapes and STEP solids.

STEP is conservatively represented by exact-geometric OCCT AABBs per solid,
not an inscribed tessellation. Filled holes/concavities can cause false rejects.
Results distinguish sampled model interference, inconclusive bounds, conditional
reservations and VERIFIED geometry. Never certifies strength or unknown mounts.
"""
from dataclasses import dataclass,asdict
import argparse,json
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon
from .v03 import (Design,Body,bodies,rectangle,prism_gaps,transformed,
                   motion_radius,invariant_circle_clearance)
from .freedom import MapSettings,local_reserves,fixed_reserves,frame_body,decoration_budgets


@dataclass(frozen=True)
class Placement:
    frame: str = 'fixed'
    at_mm: tuple = (0,0,0)
    yaw_deg: float = 0
    host: str | None = None


def from_box(size,placement=Placement(),d=Design()):
    size=np.asarray(size,float)
    if size.shape!=(3,) or not np.isfinite(size).all() or min(size)<=0:
        raise ValueError('Box must have three finite positive sizes')
    b=Body('decoration/box',-2,rectangle(0,0,*size[:2]),(-size[2]/2,size[2]/2))
    return install([b],placement,d)


def from_shape(shape,placement=Placement(),d=Design()):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    objects=shape.vals() if hasattr(shape,'vals') else [shape]
    solids=[solid for obj in objects for solid in obj.Solids()]
    if not solids:raise ValueError('Decoration must contain closed solid(s), not only faces/wires')
    result=[]
    for i,solid in enumerate(solids):
        b=Bnd_Box()
        BRepBndLib.AddOptimal_s(solid.wrapped,b,False,True)
        x0,y0,z0,x1,y1,z1=b.Get()
        result.append(Body(f'decoration/solid{i}',-2,rectangle((x0+x1)/2,(y0+y1)/2,x1-x0,y1-y0),(z0,z1)))
    return install(result,placement,d)


def from_step(path,placement=Placement(),d=Design()):
    import cadquery as cq
    shape=cq.importers.importStep(str(path))
    return from_shape(shape,placement,d)


def install(parts,p,d):
    if p.frame!='fixed' and p.frame not in [f'door{i}' for i in range(6)]:
        raise ValueError('frame must be fixed or door0..door5')
    xyz=np.asarray(p.at_mm,float)
    if xyz.shape!=(3,) or not np.isfinite(xyz).all() or not np.isfinite(p.yaw_deg):
        raise ValueError('Installation requires finite XYZ and yaw')
    a=np.deg2rad(p.yaw_deg);rot=np.array([[np.cos(a),np.sin(a)],[-np.sin(a),np.cos(a)]])
    result=[]
    for b in parts:
        xy=b.xy@rot+xyz[:2];z=(b.z[0]+xyz[2],b.z[1]+xyz[2])
        if p.frame=='fixed':result.append(Body(b.name,-2,xy,z))
        else:
            f=frame_body(d,int(p.frame[-1]))
            result.append(Body(b.name,f.actor,xy,z,f.origin,f.initial,-1))
    return result


def reservation_bodies(d):
    result=[]
    for r in fixed_reserves(d):
        result.append((Body(r.name,-1,np.asarray(r.poly.exterior.coords)[:-1],r.z),r.category,r.level,r.note))
    for i in range(6):result.extend(local_reserves(d,i))
    return result


def pair_check(a,b,d,s,step):
    same=(0<=a.actor<6 and a.actor==b.actor)
    angles=np.array([0.]) if same or (a.rate==0 and b.rate==0) else np.linspace(0,240,int(np.ceil(240/step))+1)
    gaps=prism_gaps(a,b,angles);idx=int(gaps.argmin());sample=float(gaps[idx])
    loss=0 if len(angles)==1 else (motion_radius(a)+motion_radius(b))*np.deg2rad(step/2)
    phase=0 if same else sum(motion_radius(x) for x in (a,b) if 0<=x.actor<6)*np.deg2rad(d.phase_deg)
    zgap=max(b.z[0]-a.z[1],a.z[0]-b.z[1])
    radial=float(np.linalg.norm(np.array(a.origin)-b.origin)-np.linalg.norm(a.xy,axis=1).max()-np.linalg.norm(b.xy,axis=1).max())
    invariant=max(zgap,radial,invariant_circle_clearance(a,b))
    lower=max(sample-loss-phase,invariant)-d.surface_error_mm-s.decoration_error_mm
    va,vb=transformed(a,[angles[idx]])[0],transformed(b,[angles[idx]])[0]
    witness_distance=float(np.hypot(Polygon(va).distance(Polygon(vb)),max(zgap,0)))
    return dict(part=a.name,obstacle=b.name,sampled_sat_lower_bound_mm=sample,
                continuous_clearance_lower_bound_mm=float(lower),
                limiting_sample_command_deg=float(angles[idx]),
                command_interval_deg=([0,240] if len(angles)==1 else [max(0,float(angles[idx])-step/2),min(240,float(angles[idx])+step/2)]),
                witness_exact_enclosure_distance_mm=witness_distance,
                sampled_enclosure_penetrations=int((gaps<-1e-7).sum()),samples=len(angles))


def check(parts,placement=Placement(),d=Design(),s=MapSettings(),step=1):
    if not parts:raise ValueError('No decoration solids supplied')
    if not 0<step<=1:raise ValueError('Step must be in (0,1]')
    scene=bodies(d);by={b.name:b for b in scene}
    scene+=decoration_budgets(d,s.thickness_cap_mm)
    host=placement.host
    if host is None and placement.frame.startswith('door'):host=placement.frame+'/panel'
    if host and host not in by:raise ValueError('Unknown host component')
    rows=[];contacts=[]
    for part in parts:
        for b in scene:
            if b.name.startswith('budget/') and 0<=part.actor<6 and b.actor==part.actor:
                continue
            if b.name==host:
                if b.actor!=part.actor and not (part.actor==-2 and b.actor<0):
                    raise ValueError('Host belongs to a different installation frame')
                # Only zero-volume intended surface contact, never an arbitrary
                # skip of the owner assembly or a deep pocket into a host.
                va,vb=transformed(part,[0])[0],transformed(b,[0])[0]
                volume=Polygon(va).intersection(Polygon(vb)).area*max(0,min(part.z[1],b.z[1])-max(part.z[0],b.z[0]))
                contact_distance=np.hypot(Polygon(va).distance(Polygon(vb)),max(b.z[0]-part.z[1],part.z[0]-b.z[1],0))
                numerical_face_contact=(host.endswith('/panel') and b.actor>=0 and np.max(part.xy[:,1])<=-3+1e-5)
                if contact_distance<=1e-5 and (volume<=1e-7 or numerical_face_contact):
                    contacts.append(dict(part=part.name,host=host,intended_surface_contact_only=True,numerical_contact_tolerance_mm=1e-5))
                    continue
            row=pair_check(part,b,d,s,step)
            if row['continuous_clearance_lower_bound_mm']<s.clearance_mm and not row['sampled_enclosure_penetrations'] and step>.1:
                row=pair_check(part,b,d,s,.1)
            rows.append(row)
    worst=min(rows,key=lambda r:r['continuous_clearance_lower_bound_mm'])
    geometry_pass=all(r['continuous_clearance_lower_bound_mm']>=s.clearance_mm for r in rows)
    penetrations=sum(r['sampled_enclosure_penetrations'] for r in rows)
    reserved=[]
    for part in parts:
        for b,cat,level,note in reservation_bodies(d):
            row=pair_check(part,b,d,s,step)
            if row['continuous_clearance_lower_bound_mm']<=0:
                reserved.append(dict(name=b.name,category=cat,level=level,note=note))
    outside=False
    for b in parts:
        if b.actor==-2:
            xy=transformed(b,[0])[0]
            lo,hi=xy.min(axis=0),xy.max(axis=0)
        else:
            r=np.linalg.norm(b.xy,axis=1).max()
            lo,hi=np.array(b.origin)-r,np.array(b.origin)+r
        outside |= bool(np.any(lo<np.array(s.roi_min[:2])) or np.any(hi>np.array(s.roi_max[:2])) or b.z[0]<s.roi_min[2] or b.z[1]>s.roi_max[2])
    level='VERIFIED' if geometry_pass and not reserved and not outside else ('CONDITIONAL' if geometry_pass and not outside else 'UNVERIFIED')
    verdict='CERTIFIED_CLEAR' if geometry_pass else ('ENVELOPE_INTERFERENCE' if penetrations else 'INCONCLUSIVE_BOUND')
    return dict(version='0.4',verification_level=level,geometry_verdict=verdict,geometry_pass=geometry_pass,
                minimum_continuous_clearance_lower_bound_mm=worst['continuous_clearance_lower_bound_mm'],
                required_clearance_mm=s.clearance_mm,limiting_pair=worst,
                reservation_hits=reserved,outside_map_domain=outside,intended_contacts=contacts,
                installation=asdict(placement),design=asdict(d),settings=asdict(s),
                scope='Installed rigid geometry only; STEP AABBs may falsely reject; holes/strength/mount manufacture are not certified',
                other_new_decorations='Other-door complete B budgets checked; arbitrary additions outside fixed A/moving B require a new joint scene',
                dangerous_pairs=[r for r in rows if r['continuous_clearance_lower_bound_mm']<s.clearance_mm][:20])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--box',nargs=3,type=float,metavar=('X','Y','Z'))
    source.add_argument('--step',type=Path)
    p.add_argument('--frame',default='fixed');p.add_argument('--at',nargs=3,type=float,default=[0,0,0])
    p.add_argument('--yaw',type=float,default=0);p.add_argument('--host');p.add_argument('--output',type=Path)
    p.add_argument('--settings',type=Path)
    a=p.parse_args();placement=Placement(a.frame,tuple(a.at),a.yaw,a.host)
    parts=from_box(a.box,placement) if a.box else from_step(a.step,placement)
    settings=MapSettings(**json.loads(a.settings.read_text())) if a.settings else MapSettings()
    result=check(parts,placement,s=settings)
    text=json.dumps(result,indent=2)+'\n'
    if a.output:a.output.write_text(text)
    else:print(text,end='')


if __name__=='__main__':main()
