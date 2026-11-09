"""V0.4: certified OUTER sweep enclosures, reusing V0.3 bodies and frames.

All dimensions are mm. A finite sample union alone is never a certificate.
Each sample is padded by an explicit speed/phase bound, the two surface error
caps and the chosen clearance. Z is padded separately (no fictitious Z motion).
"""
from dataclasses import dataclass,asdict
import numpy as np
import shapely
from shapely.geometry import Polygon,box
from .v03 import Design,Body,bodies,transformed,rectangle,outer_circle


@dataclass(frozen=True)
class MapSettings:
    angular_step_deg: float = 1
    local_step_deg: float = .5
    clearance_mm: float = 1
    decoration_error_mm: float = .2
    cell_mm: float = 10
    thickness_cap_mm: float = 40
    thickness_quantum_mm: float = .25
    face_cell_x_mm: float = 10
    face_cell_z_mm: float = 15
    roi_min: tuple = (-400,-400,-80)
    roi_max: tuple = (400,400,850)

    def __post_init__(self):
        if not (0<self.angular_step_deg<=1 and 0<self.local_step_deg<=1):
            raise ValueError('Sweep steps must be >0 and <=1 degree')
        if min(self.clearance_mm,self.cell_mm,self.thickness_cap_mm,self.thickness_quantum_mm)<=0 or self.decoration_error_mm<0:
            raise ValueError('Invalid clearance, grid or error cap')


@dataclass
class Region:
    name: str
    poly: object
    z: tuple
    category: str
    level: str
    frame: str = 'world'
    note: str = ''


def outer_buffer(poly,margin,quad=4):
    """GEOS round buffers use inscribed arcs: enlarge so every chord is OUTER.

    Max chord half-angle is pi/(4*quad). The 1e-7 guard covers float noise,
    not manufacturing error. Positive margins only, never negative erosion.
    """
    if margin<0:raise ValueError('Outer padding cannot be negative')
    return shapely.buffer(poly,(margin+1e-7)/np.cos(np.pi/(4*quad)),quad_segs=quad)


def frame_body(d,door):
    return next(b for b in bodies(d) if b.name==f'door{door}/panel')


def relative_vertices(b,angles,frame=None):
    world=transformed(b,angles)
    if frame is None:return world
    a=np.deg2rad(frame.initial+frame.rate*np.asarray(angles))
    q=world-np.array(frame.origin)
    return np.stack([np.cos(a)[:,None]*q[:,:,0]+np.sin(a)[:,None]*q[:,:,1],
                    -np.sin(a)[:,None]*q[:,:,0]+np.cos(a)[:,None]*q[:,:,1]],axis=-1)


def sweep(b,d,settings,frame=None,category='D'):
    step=settings.angular_step_deg if frame is None else settings.local_step_deg
    same=frame is not None and b.actor==frame.actor
    r=float(np.linalg.norm(b.xy,axis=1).max())
    if frame is None:
        speed=r*abs(b.rate)
        phase_speed=speed if 0<=b.actor<6 else 0
    elif same:
        speed=phase_speed=0
    else:
        dist=float(np.linalg.norm(np.array(b.origin)-frame.origin))
        speed=abs(frame.rate)*dist+abs(b.rate-frame.rate)*r
        phase_speed=(dist+r)*abs(frame.rate)+(r*abs(b.rate) if 0<=b.actor<6 else 0)
    time_pad=speed*np.deg2rad(step/2)
    phase_pad=phase_speed*np.deg2rad(d.phase_deg)
    pair_pad=d.surface_error_mm+settings.decoration_error_mm+settings.clearance_mm
    angles=np.linspace(0,240,int(np.ceil(240/step))+1)
    # For concentric rotating circular proxies use an invariant disk enclosing
    # ALL proxy vertices, not a fixed polygon that might miss rotated corners.
    circular=(frame is None and b.rate!=0 and np.ptp(np.linalg.norm(b.xy,axis=1))<1e-8)
    if circular:
        poly=Polygon(outer_circle(r,32)+b.origin)
        envelope=outer_buffer(poly,pair_pad)
        time_pad=phase_pad=0;samples=1
        method='rotation-invariant enclosing disk'
    else:
        if speed==0 and phase_speed==0:angles=np.array([0.])
        vertices=relative_vertices(b,angles,frame)
        polygons=shapely.polygons(vertices)
        envelope=shapely.union_all(outer_buffer(polygons,time_pad+phase_pad+pair_pad))
        samples=len(angles);method='sample union padded by time, independent phase, surface caps and clearance'
    reg=Region(b.name,envelope,(b.z[0]-pair_pad,b.z[1]+pair_pad),category,'VERIFIED',
               'world' if frame is None else f'door{frame.actor}',method)
    evidence=dict(name=b.name,samples=samples,step_deg=step,method=method,
                  time_padding_mm=time_pad,phase_padding_mm=phase_pad,
                  surface_and_clearance_mm=pair_pad,z_padding_mm=pair_pad,
                  speed_mm_per_command_radian=speed,phase_coefficient_mm_per_radian=phase_speed,
                  source_actor=b.actor,proof='OUTER enclosure; not the exact swept solid')
    return reg,evidence


def local_reserves(d,door):
    f=frame_body(d,door);x0,x1=d.panel[:,0].min(),d.panel[:,0].max()
    result=[]
    def add(name,xy,z,cat='E',level='CONDITIONAL',note='Unmodelled fastening/joint; no decoration clearance claim'):
        b=Body(f'reserve/door{door}/{name}',door,xy,z,f.origin,f.initial,-1)
        result.append((b,cat,level,note))
    add('hinge_fasteners',rectangle((x0+27)/2,-5,27-x0,40),(-16,311))
    add('free_edge_joint',rectangle((x1+153)/2,-5,x1-153,40),(-5,300))
    add('panel_split_seam',rectangle(d.radius/2,-5,x1-x0,40),(135.5,159.5))
    add('lower_rail_fixings',rectangle(d.radius/2,-5,x1-x0,40),(-16,22))
    add('upper_rail_fixings',rectangle(d.radius/2,-5,x1-x0,40),(273,311))
    # Hypothetical light trim clip; its receiver and load capacity are absent.
    add('clip_insert_flex_remove',rectangle(90,-13.5,11.6,21),(94,106),'C','CONDITIONAL',
        'Candidate clip: 15 mm insertion/removal, +/-0.8 mm lateral flex; receiver missing; no PETG fatigue claim')
    return result


def fixed_reserves(d):
    out=[]
    def add(name,xy,z,cat='C',level='CONDITIONAL',note='Conceptual service/unfinished geometry reservation'):
        out.append(Region(name,Polygon(xy),z,cat,level,'world',note))
    add('roof_unfinished_joints_and_clutch',outer_circle(240,32),(332,530))
    add('gearbox_vertical_extraction',outer_circle(240,32),(332,680),note='Remove dome first; straight lift is a proposed route, not a proven disassembly sequence')
    add('knob_hand_operating_space',outer_circle(75,32),(480,690),note='Hand radius75 and overhead reach are assumptions; no anthropometric certificate')
    add('removable_dome_lift_path',outer_circle(280,32),(330,820),level='UNVERIFIED',
        note='Encloses installed dome band to radius275/Z590 and its FULL220mm lift plus allowance. Attachment/shape not designed; remove before servicing.')
    add('base_fasteners_and_tools',outer_circle(300,32),(-75,-20),
        note='Includes base wrap radius250 plus40mm outward removal and allowance; joints/tools not finalized')
    add('roof_fasteners_and_tool_approach',outer_circle(245,32),(300,345))
    for i in range(6):
        a=i*np.pi/3;o=d.radius*np.array([np.cos(a),np.sin(a)])
        for name,z in [('corner_lower',(-48,24)),('corner_upper',(270,338))]:
            add(f'{name}{i}_unmodelled_mount',rectangle(*o,40,40),z,'E')
    return out


def decoration_budgets(d,cap):
    # Simultaneous decoration guarantee: every certified patch is checked
    # against a SUPERSET of every other door's patches (not just bare V0.3).
    result=[]
    for i in range(6):
        f=frame_body(d,i)
        for j,(lo,hi) in enumerate(((22,135.5),(159.5,273))):
            result.append(Body(f'budget/door{i}/{j}',i,rectangle(90,-3-cap/2,126,cap),
                               (lo,hi),f.origin,f.initial,-1))
    return result


def union_at(regions,z0,z1):
    return shapely.union_all([r.poly for r in regions if r.z[0]<=z1 and r.z[1]>=z0])


def thickness_limit(poly,x0,x1,cap):
    cut=poly.intersection(box(x0,-3-cap,x1,-3))
    if cut.is_empty:return cap,True
    return float(max(0,min(cap,-3-cut.bounds[3])-1e-6)),False


def face_map(d,s,door,progress=None):
    frame=frame_body(d,door);scene=bodies(d)
    pair_pad=d.surface_error_mm+s.decoration_error_mm+s.clearance_mm
    obstacles=[];evidence=[];excluded_z=[]
    for b in scene+decoration_budgets(d,s.thickness_cap_mm):
        if b.name==frame.name or (b.name.startswith('budget/') and b.actor==door):continue
        if b.z[1]+pair_pad<0 or b.z[0]-pair_pad>d.height:
            excluded_z.append(b.name);continue
        reg,ev=sweep(b,d,s,frame);obstacles.append(reg);evidence.append(ev)
    reserves=[]
    # All other local reservations also reserve world space along their motion.
    for i in range(6):
        for b,cat,level,note in local_reserves(d,i):
            reg,ev=sweep(b,d,s,frame,cat);reg.level=level;reg.note=note
            reserves.append(reg)
    # A fixed reserve becomes a rotating obstacle in this local frame.
    for r in fixed_reserves(d):
        if r.z[1]+pair_pad<0 or r.z[0]-pair_pad>d.height:continue
        b=Body(r.name,-1,np.asarray(r.poly.exterior.coords)[:-1],r.z)
        reg,_=sweep(b,d,s,frame,r.category);reg.level=r.level;reg.note=r.note
        reserves.append(reg)
    xs=np.r_[np.arange(d.panel[:,0].min(),d.panel[:,0].max(),s.face_cell_x_mm),d.panel[:,0].max()]
    zs=np.r_[np.arange(0,d.height,s.face_cell_z_mm),d.height]
    rows=[]
    for z0,z1 in zip(zs[:-1],zs[1:]):
        obs=union_at(obstacles,z0,z1);reserved=union_at(reserves,z0,z1)
        for x0,x1 in zip(xs[:-1],xs[1:]):
            geometric,capped=thickness_limit(obs,x0,x1,s.thickness_cap_mm)
            reservation,_=thickness_limit(reserved,x0,x1,s.thickness_cap_mm)
            raw=min(geometric,reservation)
            certified=np.floor(raw/s.thickness_quantum_mm)*s.thickness_quantum_mm
            level='VERIFIED' if certified>0 else ('CONDITIONAL' if reservation<=1e-6 else 'UNVERIFIED')
            rows.append(dict(door=door,x0_mm=float(x0),x1_mm=float(x1),z0_mm=float(z0),z1_mm=float(z1),
                             outward_normal_local='0,-1,0',max_certified_mm=float(certified),
                             modeled_geometry_limit_mm=float(geometric),reservation_limit_mm=float(reservation),
                             verification_level=level,cap_limited=bool(capped and raw>=s.thickness_cap_mm-1e-5),
                             face_y_mm=-3,search_cap_mm=s.thickness_cap_mm))
    return rows,dict(door=door,sweep_evidence=evidence,z_rejected_bodies=excluded_z,
                    simultaneous_other_door_budgets=True,host_contact='ONLY declared panel outward face; fasteners/joints reserved')
