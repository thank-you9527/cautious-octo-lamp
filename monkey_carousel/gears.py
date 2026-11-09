"""Sampled involute research profiles, not manufacturing tooth/root geometry.

20 degree pressure angle; zero profile shift. Root joins are radial, NOT a
hobbing trochoid or load-rated fillet. Backlash is total pair pitch-line space.
"""
import numpy as np


def outline(teeth,module=1.5,backlash=.2,resolution=8):
    if teeth<20 or not 0<=backlash<module:
        raise ValueError('Study supports >=20 teeth and nonnegative small backlash')
    rp=module*teeth/2;rb=rp*np.cos(np.deg2rad(20));rf=rp-1.25*module;ra=rp+module
    inv=lambda a:np.tan(a)-a
    def half(r):
        return np.pi/(2*teeth)-backlash/(4*rp)+inv(np.deg2rad(20))-inv(np.arccos(rb/r))
    radii=np.linspace(max(rb,rf),ra,resolution)
    flank=half(radii)
    points=[]
    for k in range(teeth):
        phi=k*2*np.pi/teeth
        polar=[(rf,-flank[0]),*zip(radii,-flank),
               *[(ra,a) for a in np.linspace(-flank[-1],flank[-1],5)[1:]],
               *zip(radii[-2::-1],flank[-2::-1]),(rf,flank[0])]
        end=2*np.pi/teeth-flank[0]
        polar += [(rf,a) for a in np.linspace(flank[0],end,5)[1:-1]]
        for r,a in polar:
            pt=[r*np.cos(phi+a),r*np.sin(phi+a)]
            if not points or np.linalg.norm(np.array(pt)-points[-1])>1e-10:
                points.append(pt)
    return np.array(points)


def positioned(poly,angle,center):
    c,s=np.cos(np.deg2rad(angle)),np.sin(np.deg2rad(angle))
    return poly@np.array([[c,s],[-s,c]])+center


def mesh_audit(step=1,backlash=.2):
    """2D profile intersection extruded through a known common 12 mm Z interval.

    Area*12 is the actual 3D intersection volume for these prismatic sampled
    profiles. This special 2D reduction is valid because their Z intervals
    coincide; scene clearance is separately full 3D. No continuous mesh claim.
    """
    from shapely.geometry import Polygon
    from shapely import affinity
    profiles={n:Polygon(outline(n,backlash=backlash)) for n in (20,30,40,90,100,200)}
    def moved(n,a,c):
        return affinity.translate(affinity.rotate(profiles[n],a,origin=(0,0)),*c)
    rows=[]
    pairs=[]
    for i in range(6):
        phi=i*60;c=180*np.array([np.cos(np.deg2rad(phi)),np.sin(np.deg2rad(phi))])
        pairs.append((f'sun200/door{i}_40',200,40,(0,0),c,0,180+4.5+6*phi,.2,-1))
    pairs += [('output90/lay30',90,30,(0,0),(90,0),0,180+6,.2,-.6),
              ('input20/lay100',20,100,(0,0),(90,0),0,180+1.8,3,-.6)]
    # Complete door command 0..240; reducer phases span multiple mesh periods.
    for name,na,nb,ca,cb,pa,pb,ka,kb in pairs:
        worst=0;distance=1e9;at=0;count=0
        for theta in np.linspace(0,240,round(240/step)+1):
            a=moved(na,pa+ka*theta,ca);b=moved(nb,pb+kb*theta,cb)
            area=a.intersection(b).area
            if area>worst:worst=area;at=float(theta)
            distance=min(distance,a.distance(b));count+=area>1e-7
        motion=((1.5*na/2+1.5)*abs(ka)+(1.5*nb/2+1.5)*abs(kb))*np.deg2rad(step/2)
        rows.append(dict(pair=name,samples=round(240/step)+1,
                         maximum_intersection_volume_mm3=float(worst*12),
                         minimum_sampled_distance_mm=float(distance),worst_angle_deg=at,
                         penetrating_samples=int(count),sampled_no_penetration=bool(count==0),
                         continuous_nominal_profile_lower_bound_mm=float(distance-motion)))
    return dict(step_deg=step,total_pair_backlash_mm=backlash,rows=rows,
                sampled_pass=all(r['sampled_no_penetration'] for r in rows),
                continuous_nominal_polyline_profiles_certified=all(r['continuous_nominal_profile_lower_bound_mm']>=0 for r in rows),
                limitations=['Sampled polyline involutes; no root fillet, strength, printed error or phase uncertainty certificate',
                             'Nominal mesh phase only; door phase error envelope audit does not certify tooth contact'])


def comparison():
    rows=[]
    for name,r,m,sun,door,reduction,count,stages in [
        ('R140 direct',140,1,240,40,1,13,[]),
        ('R140 16:1',140,1,240,40,16,17,[(24,96),(24,96)]),
        ('R180 direct',180,1.5,200,40,1,15,[]),
        ('R180 15:1 PRIMARY',180,1.5,200,40,15,19,[(20,100),(30,90)])]:
        gain=sun/door/reduction
        cases=[dict(per_door_Nm=t,efficiency=e,knob_Nm=6*t*gain/e,
                    force_at_25mm_N=6*t*gain/e/.025) for t,e in ((.02,.85),(.08,.7),(.2,.5))]
        rows.append(dict(name=name,axis_radius_mm=r,module_mm=m,sun_teeth=sun,door_teeth=door,
                         pitch_diameters_mm=[m*sun,m*door],center_distance_mm=m*(sun+door)/2,
                         reducer_teeth=stages,reducer_centers_mm=[m*(a+b)/2 for a,b in stages],
                         signed_door_per_knob=-gain,knob_stroke_deg=240/gain,
                         estimated_gear_print_pieces=count,torque_scenarios=cases))
    r1,r2=20/np.pi,60/np.pi;C=180;delta=r2-r1
    belt=2*np.sqrt(C*C-delta*delta)+np.pi*(r1+r2)+2*delta*np.arcsin(delta/C)
    return dict(gear_schemes=rows,GT2_backup=dict(pitch_mm=2,distribution_teeth=40,
                distribution_pitch_diameter_mm=80/np.pi,distribution_length_mm=1160,
                input_teeth=[20,60],input_pitch_diameters_mm=[40/np.pi,120/np.pi],
                input_centers_mm=180,input_length_mm=float(belt),signed_door_per_knob=1/3,
                knob_stroke_deg=720,estimated_pulley_print_pieces=8,nominal_wrap_deg=60,
                nominal_engaged_teeth=40/6),
                status='Friction/efficiency scenarios only; acquisition availability and strength not verified')
