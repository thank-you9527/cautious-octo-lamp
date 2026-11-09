"""V0.3 vertical convex-prism motion, independent door phase uncertainty.

Reuses the original parameter constraints and 3D SAT conventions. Horizontal
polygon edge normals plus Z form the complete prism SAT. Units: mm, degrees.
No manufacturing STL; all contact exclusions are named in the JSON report.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import json
import numpy as np
from .model import Parameters


@dataclass(frozen=True)
class Design:
    radius: float = 180
    gap: float = 2.2
    outer_y: float = 5.5
    thickness: float = 6
    height: float = 295
    phase_deg: float = .25
    surface_error_mm: float = .2  # per body, bounding displacement, NOT sigma
    required_clearance: float = 1
    foot_radius: float = 25
    layout: str = 'framed'
    end_trim: float = 6.5
    post_radius: float = 5

    def __post_init__(self):
        Parameters(radius=self.radius,length=self.radius,foot_radius=self.foot_radius)
        if not (1 <= self.gap <= 3) or min(self.outer_y,self.thickness)>100:
            raise ValueError('Invalid wall dimensions')
        if min(self.outer_y,self.thickness,self.height,self.required_clearance)<=0 or min(self.phase_deg,self.surface_error_mm)<0:
            raise ValueError('Invalid dimension or uncertainty')

    @property
    def panel(self):
        if self.layout=='framed':
            return rectangle(self.radius/2,0,self.radius-2*self.end_trim,self.thickness)
        # Miter cuts leave a normal gap of `gap` at each closed hexagon corner.
        s=self.gap/np.sqrt(3)
        y0,y1=self.outer_y,self.outer_y+self.thickness
        return np.array([[s+y0/np.sqrt(3),y0],
                         [self.radius-s-y0/np.sqrt(3),y0],
                         [self.radius-s-y1/np.sqrt(3),y1],
                         [s+y1/np.sqrt(3),y1]])


@dataclass
class Body:
    name: str
    actor: int
    xy: np.ndarray
    z: tuple
    origin: tuple = (0,0)
    initial: float = 0
    rate: float = 0  # yaw degrees / door command degrees


def rectangle(cx,cy,sx,sy):
    return np.array([[cx-sx/2,cy-sy/2],[cx+sx/2,cy-sy/2],
                     [cx+sx/2,cy+sy/2],[cx-sx/2,cy+sy/2]])


def outer_circle(r,n=16):
    a=(np.arange(n)+.5)*2*np.pi/n
    return r/np.cos(np.pi/n)*np.stack([np.cos(a),np.sin(a)],axis=1)


def sector(r0,r1,a,b):
    aa,bb=np.deg2rad([a,b])
    return np.array([[r0*np.cos(aa),r0*np.sin(aa)],
                     [r1*np.cos(aa),r1*np.sin(aa)],
                     [r1*np.cos(bb),r1*np.sin(bb)],
                     [r0*np.cos(bb),r0*np.sin(bb)]])


def bodies(d,drive=True):
    result=[]
    for i in range(6):
        phi=i*60
        origin=tuple(d.radius*np.array([np.cos(np.deg2rad(phi)),np.sin(np.deg2rad(phi))]))
        def add(name,poly,z):
            result.append(Body(f'door{i}/{name}',i,poly,z,origin,phi+120,-1))
        x=d.radius/2
        front=d.thickness/2 if d.layout=='framed' else d.outer_y+d.thickness
        add('panel',d.panel,(0,d.height))
        add('padding',rectangle(x,front+2.5,70,5),(10,290))
        add('doll',rectangle(x,front+5+25,70,50),(15,285))
        for name,z in [('leg_strap',75),('torso_strap',225)]:
            # Same 69 mm strap depth and 4 mm extra as legacy model, translated.
            add(name,rectangle(x,front-3+27.5,74,69),(z-9,z+9))
        add('foot',rectangle(x,front+d.foot_radius/2,2*d.foot_radius,d.foot_radius),(9,15))
        for name,z in [('lower',(-12,-4)),('upper',(299,307))]:
            add(name+'_hub',outer_circle(8),z)
            add(name+'_arm',rectangle(12,6,24,12),z)
        add('lower_tab',rectangle(18,front-3,20,6),(-12,15))
        add('upper_tab',rectangle(18,front-3,20,6),(280,307))
        if drive:
            add('gear40',outer_circle(31.5),(350,362))
        result.append(Body(f'fixed/shaft{i}',-1,outer_circle(3),(-40,380),origin))
        if d.layout=='framed':
            # Stationary metal tube OD10/ID8 surrounding the rotating Ø6 shaft.
            # Occupied outside envelope is checked; internal bore is intentional.
            result.append(Body(f'fixed/corner_tube{i}',-1,outer_circle(d.post_radius,32),(0,295),origin))
        for name,z in [('lower',(-43,-20)),('upper',(310,332))]:
            result.append(Body(f'fixed/{name}_bearing{i}',-1,rectangle(0,0,24,24),z,origin))
    result += [Body('fixed/base',-1,outer_circle(d.radius+40),(-55,-43)),
               Body('fixed/roof',-1,outer_circle(d.radius+40),(320,332)),
               Body('fixed/column',-1,outer_circle(12),(-43,320))]
    if not drive:
        return result
    # m1.5 200/40 sun, two-level reverted reducer 20/100 then 30/90.
    result += [Body('drive/sun200',6,outer_circle(151.5),(350,362),rate=.2),
               Body('drive/output90',6,outer_circle(69),(420,432),rate=.2),
               Body('drive/lay30',7,outer_circle(24),(420,432),(90,0),rate=-.6),
               Body('drive/lay100',7,outer_circle(76.5),(460,472),(90,0),rate=-.6),
               Body('drive/input20',8,outer_circle(16.5),(460,472),rate=3),
               Body('drive/knob_clutch',8,outer_circle(25),(506,520),rate=3),
               Body('drive/detent_cam',6,outer_circle(45),(375,381),rate=.2),
               Body('drive/stop_lug',6,sector(50,60,-2,2),(375,381),rate=.2),
               Body('fixed/stop_closed',-1,sector(50,65,-6,-2),(375,394)),
               Body('fixed/stop_display',-1,sector(50,65,50,54),(375,394)),
               Body('fixed/ball_plunger',-1,rectangle(0,58,10,26),(375,394)),
               Body('fixed/output_shaft',-1,outer_circle(5),(332,438)),
               Body('fixed/lay_shaft',-1,outer_circle(4),(400,490),(90,0)),
               Body('fixed/input_shaft',-1,outer_circle(3),(448,515)),
               Body('fixed/support_plate1',-1,outer_circle(d.radius+40),(394,402)),
               Body('fixed/support_plate2',-1,outer_circle(d.radius+40),(442,448)),
               Body('fixed/support_plate3',-1,outer_circle(d.radius+40),(490,496))]
    for name,x,y,r,z in [('output_low',0,0,15,(338,344)),('output_high',0,0,15,(366,372)),
                         ('lay_low',90,0,16,(403,410)),('lay_high',90,0,16,(480,487)),
                         ('input_low',0,0,13,(448,454)),('input_high',0,0,13,(490,496))]:
        result.append(Body('fixed/'+name+'_bearing',-1,rectangle(0,0,2*r,2*r),z,(x,y)))
    for i in range(6):
        a=np.deg2rad(i*60+30)
        result.append(Body(f'fixed/roof_post{i}',-1,outer_circle(4),(332,500),
                           tuple((d.radius+30)*np.array([np.cos(a),np.sin(a)]))))
    return result


def exclusion(a,b):
    if a.actor==b.actor:
        return 'same rigid assembly / fixed-to-fixed; internal joints unverified'
    names={a.name,b.name}
    for door in (a,b):
        if 0<=door.actor<6 and f'fixed/shaft{door.actor}' in names:
            if any(door.name.endswith(s) for s in ('_hub','_arm','gear40')):
                return 'own shaft bore: intended fit, not fit-certified'
    meshes=[('drive/sun200',f'door{i}/gear40') for i in range(6)]
    meshes += [('drive/output90','drive/lay30'),('drive/lay100','drive/input20')]
    if any(names==set(pair) for pair in meshes):
        return 'meshing addendum envelopes; audited separately, no tooth-strength claim'
    for name,actor in [('fixed/output_shaft',6),('fixed/lay_shaft',7),('fixed/input_shaft',8)]:
        if name in names and any(x.actor==actor for x in (a,b)):
            return 'own drive shaft bore'
    if 'drive/stop_lug' in names and names & {'fixed/stop_closed','fixed/stop_display'}:
        return 'zero-clearance intended end-stop; independently swept for penetration'
    if names=={'drive/detent_cam','fixed/ball_plunger'}:
        return 'intentional compliant follower; spring/contact force unverified'
    return None


def transformed(body,angles,phase=0):
    a=np.deg2rad(body.initial+body.rate*(np.asarray(angles)+phase))
    c,s=np.cos(a),np.sin(a)
    poly=body.xy
    return np.stack([c[:,None]*poly[:,0]-s[:,None]*poly[:,1]+body.origin[0],
                     s[:,None]*poly[:,0]+c[:,None]*poly[:,1]+body.origin[1]],axis=-1)


def prism_gaps(a,b,angles,phase_a=0,phase_b=0):
    va,vb=transformed(a,angles,phase_a),transformed(b,angles,phase_b)
    normals=[]
    for poly in (va,vb):
        e=np.roll(poly,-1,axis=1)-poly
        normals.append(np.stack([-e[:,:,1],e[:,:,0]],axis=-1)/np.linalg.norm(e,axis=-1)[:,:,None])
    n=np.concatenate(normals,axis=1)
    pa,pb=np.einsum('avi,aki->akv',va,n),np.einsum('avi,aki->akv',vb,n)
    horizontal=np.maximum(pb.min(axis=-1)-pa.max(axis=-1),pa.min(axis=-1)-pb.max(axis=-1)).max(axis=-1)
    vertical=max(b.z[0]-a.z[1],a.z[0]-b.z[1])
    return np.maximum(horizontal,vertical)


def motion_radius(b):
    return float(np.linalg.norm(b.xy,axis=1).max()*abs(b.rate))


def invariant_circle_clearance(a,b):
    """All-angle radial certificate against a fixed circumscribed disk.

    Distance from a fixed point to a rotating polygon is bounded by its radial
    interval. This avoids unnecessarily subtracting phase displacement from a
    shaft/post clearance that is already safe through an entire revolution.
    """
    if a.actor<0:
        a,b=b,a
    if a.actor<0 or b.actor>=0 or len(b.xy)<12:
        return -1e9
    radii=np.linalg.norm(b.xy,axis=1)
    if np.ptp(radii)>1e-8:
        return -1e9
    from shapely.geometry import Polygon,Point
    rmin=Polygon(a.xy).distance(Point(0,0))
    rmax=np.linalg.norm(a.xy,axis=1).max()
    dist=np.linalg.norm(np.array(a.origin)-b.origin)
    return float(max(rmin-dist,dist-rmax,0)-radii.max())


def scan(d,step=1,drive=True,robust=True):
    bs=bodies(d,drive)
    angles=np.linspace(0,240,round(240/step)+1)
    result=dict(step_deg=step,samples=len(angles),pairs=0,penetrating_pair_samples=0,
                clearance_fail_pair_samples=0,min_nominal_sat_gap_mm=1e9,
                continuous_robust_lower_bound_mm=1e9,exclusions=[],pair_certificates=[])
    for i,a in enumerate(bs):
        for b in bs[i+1:]:
            why=exclusion(a,b)
            if why:
                result['exclusions'].append([a.name,b.name,why])
                continue
            result['pairs']+=1
            # Static Z separation is invariant to all modelled angular errors.
            zgap=max(b.z[0]-a.z[1],a.z[0]-b.z[1])
            radial=float(np.linalg.norm(np.array(a.origin)-b.origin)-np.linalg.norm(a.xy,axis=1).max()-np.linalg.norm(b.xy,axis=1).max())
            invariant=max(zgap,radial)
            circular=invariant_circle_clearance(a,b)
            invariant=max(invariant,circular)
            if invariant>=max(10,d.required_clearance+2*d.surface_error_mm):
                lower=invariant-(2*d.surface_error_mm if robust else 0)
                sampled=invariant
                idx=0
            else:
                gaps=prism_gaps(a,b,angles)
                idx=int(gaps.argmin());sampled=float(gaps[idx])
                result['penetrating_pair_samples']+=int((gaps < -1e-7).sum())
                result['clearance_fail_pair_samples']+=int((gaps<d.required_clearance).sum())
                # Hausdorff displacement bound covers ALL independent phases,
                # not only 64 phase corners. z-only separation can tighten bound.
                temporal=(motion_radius(a)+motion_radius(b))*np.deg2rad(step/2)
                phase=sum(motion_radius(x) for x in (a,b) if 0<=x.actor<6)*np.deg2rad(d.phase_deg)
                lower=max(sampled-temporal-(phase if robust else 0),zgap)
                lower=max(lower,invariant)
                if robust:
                    lower-=2*d.surface_error_mm
            result['pair_certificates'].append(dict(pair=[a.name,b.name],sampled_gap_mm=float(sampled),
                angular_loss_per_degree_mm=(motion_radius(a)+motion_radius(b))*np.pi/360,
                phase_loss_per_degree_mm=sum(motion_radius(x) for x in (a,b) if 0<=x.actor<6)*np.pi/180,
                invariant_lower_mm=float(invariant),sample_angle_deg=float(angles[idx])))
            if sampled<result['min_nominal_sat_gap_mm']:
                result['min_nominal_sat_gap_mm']=sampled
            if lower<result['continuous_robust_lower_bound_mm']:
                result['continuous_robust_lower_bound_mm']=float(lower)
                result['limiting_pair']=[a.name,b.name]
                result['limiting_sample_deg']=float(angles[idx])
    result['pass']=bool(result['continuous_robust_lower_bound_mm']>=d.required_clearance)
    return result


def phase_certificate(result,phase_deg,surface_error_mm=.2,required_clearance=1):
    rows=[]
    for row in result['pair_certificates']:
        lower=max(row['sampled_gap_mm']-row['angular_loss_per_degree_mm']*result['step_deg']
                  -row['phase_loss_per_degree_mm']*phase_deg,row['invariant_lower_mm'])-2*surface_error_mm
        rows.append((lower,row['pair']))
    lower,pair=min(rows,key=lambda r:r[0])
    return dict(phase_each_door_deg=phase_deg,surface_error_each_body_mm=surface_error_mm,
                continuous_lower_bound_mm=lower,limiting_pair=pair,pass_=bool(lower>=required_clearance),
                note='A failed conservative certificate is not a collision witness')


def closing_gap(d):
    if d.layout=='framed':
        # Exact extrema of the exterior free-end to the neighbouring post over
        # +/- phase; stationary root-to-own-post is also included. Angular
        # candidates include derivative zeros, not only error-box corners.
        x=d.radius-d.end_trim;y=-d.thickness/2
        candidates=[-d.phase_deg,d.phase_deg,0]
        critical=np.rad2deg(np.arctan2(y,x))
        candidates += [critical+k*180 for k in range(-2,3) if abs(critical+k*180)<=d.phase_deg]
        distances=[np.hypot(x*np.cos(np.deg2rad(q))+y*np.sin(np.deg2rad(q))-d.radius,
                            -x*np.sin(np.deg2rad(q))+y*np.cos(np.deg2rad(q)))-d.post_radius for q in candidates]
        lower=min(d.end_trim-d.post_radius,min(distances))-2*d.surface_error_mm
        upper=max(np.hypot(d.end_trim,d.thickness/2)-d.post_radius,max(distances))+2*d.surface_error_mm
        return dict(style='six OD10 fixed corner tubes; gap measured door-to-tube',
                    nominal_gap_range_mm=[d.end_trim-d.post_radius,float(np.hypot(d.end_trim,d.thickness/2)-d.post_radius)],
                    robust_closed_gap_range_mm=[float(lower),float(upper)],
                    closed_appearance_pass=bool(lower>=1 and upper<=3),
                    actual_panel_outer_face_length_mm=d.radius-2*d.end_trim)
    # Corresponding exterior edge vertices: separation projected on the miter
    # normal is `gap`; full 3D Euclidean gap may be larger at surface endpoints.
    bs=bodies(d,False)
    a=next(b for b in bs if b.name=='door0/panel')
    b=next(b for b in bs if b.name=='door1/panel')
    va,vb=transformed(a,[0])[0],transformed(b,[0])[0]
    return dict(nominal_miter_normal_gap_mm=d.gap,
                outer_edge_endpoint_distance_mm=float(np.linalg.norm(va[1]-vb[0])),
                actual_panel_outer_face_length_mm=float(d.panel[1,0]-d.panel[0,0]))


def stop_audit(d,step=.1):
    bs=bodies(d)
    lug=next(b for b in bs if b.name=='drive/stop_lug')
    stops=[b for b in bs if b.name in ('fixed/stop_closed','fixed/stop_display')]
    angles=np.linspace(0,240,round(240/step)+1)
    minima={b.name:float(prism_gaps(lug,b,angles).min()) for b in stops}
    beyond=[float(prism_gaps(lug,stops[0],[-1])[0]),float(prism_gaps(lug,stops[1],[241])[0])]
    return dict(min_signed_gap_mm=minima,overtravel_signed_gap_mm=beyond,
                geometrically_blocks_overtravel=all(x<0 for x in beyond),
                full_stroke_no_penetration=all(x>=-1e-7 for x in minima.values()),
                sun_stroke_deg=48,buffer_engages_sun_deg=[.5,47.5],
                note='Ideal hard stop contact only; cushion compression and strength unverified')


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--probe',action='store_true')
    args=parser.parse_args()
    if args.probe:
        for r in (140,160,180,200,220):
            d=Design(radius=r)
            v=scan(d,1,False,False)
            print(r,closing_gap(d),v['min_nominal_sat_gap_mm'],v['limiting_pair'],flush=True)
    else:
        from .v03_study import run
        run()
