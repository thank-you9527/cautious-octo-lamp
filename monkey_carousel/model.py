"""Exact SAT for vertical 3D oriented boxes, conservative component envelopes.

Boxes are solid 3D geometry, not projected line segments. Their separating axes
are the two horizontal axes of each box and Z. For these vertical prisms this
is the complete 3D OBB SAT (other cross products are parallel or zero).
Positive SAT separation is a LOWER BOUND on Euclidean clearance, not its exact
value. Rounded components are enclosed by boxes, so rejection can be conservative.
"""
from dataclasses import dataclass, asdict
import numpy as np


@dataclass(frozen=True)
class Parameters:
    radius: float = 145
    length: float = 130
    board_thickness: float = 6
    height: float = 295
    doll_width: float = 70
    doll_height: float = 270
    doll_depth: float = 50
    padding: float = 5
    strap_extra: float = 4
    foot_radius: float = 25
    shaft_diameter: float = 6
    hub_radius: float = 8
    clearance: float = 2

    def __post_init__(self):
        if min(asdict(self).values()) <= 0:
            raise ValueError('All dimensions and clearance must be positive')
        if self.length < self.doll_width + self.hub_radius + 10:
            raise ValueError('Require 5 mm margin on each side of doll')
        if not 20 <= self.foot_radius <= 30:
            raise ValueError('Foot radius outside specified 20–30 mm range')


@dataclass(frozen=True)
class Box:
    name: str
    group: int  # 0..5 moving assemblies; -1 fixed
    center: tuple
    size: tuple
    yaw: float = 0
    own_shaft: int = -1


def boxes(p):
    """Door local X runs from side hinge to free edge; +Y is doll/front.

    At zero degrees X points along a hexagon side (+120 deg from radial).
    Negative rotation is outward, 240 deg total. No rotating central platform.
    Doll centered in usable board width [8,L]; feet at Z=15, head at Z=285.
    Strap envelopes cover the WHOLE board width and front of doll conservatively.
    """
    result = []
    x = (p.hub_radius + p.length) / 2
    t = p.board_thickness
    for i in range(6):
        def add(name, c, s):
            result.append(Box(f'door{i}/{name}', i, c, s))
        add('board', (x, 0, p.height/2), (p.length-p.hub_radius,t,p.height))
        add('padding', (x,t/2+p.padding/2,150), (p.doll_width,p.padding,280))
        add('doll', (x,t/2+p.padding+p.doll_depth/2,150),
            (p.doll_width,p.doll_depth,p.doll_height))
        for z, name in [(75,'leg_strap'), (225,'torso_strap')]:
            add(name, (x, (p.padding+p.doll_depth)/2,z),
                (p.doll_width+4,t+p.padding+p.doll_depth+2*p.strap_extra,18))
        r = p.foot_radius
        # Box encloses an actual semicircle lying forward of the board.
        add('foot', (x,t/2+r/2,12), (2*r,r,6))
        for z in (10,285):
            add(f'hub{z}', (0,0,z), (2*p.hub_radius,2*p.hub_radius,12))
        # Provisional GT2 distribution pulley: 40 teeth, pitch radius 40/pi,
        # conservative 15 mm outer radial envelope, above roof.
        add('distribution_pulley', (0,0,333), (30,30,10))
        if i == 0:
            add('input_driven_60T', (0,0,360), (44,44,12))
    for i in range(6):
        a = np.deg2rad(i*60)
        result.append(Box(f'fixed/shaft{i}',-1,
                          (p.radius*np.cos(a),p.radius*np.sin(a),175),
                          (p.shaft_diameter,p.shaft_diameter,380), own_shaft=i))
        for z in (-7,302):
            # 606-2RS nominal bearing 6x17x6 mm enclosed by a provisional
            # 24x24x6 seat. Bearing bores / seat fits are NOT manufacturing CAD.
            result.append(Box(f'fixed/bearing606_seat{i}_{z}',-1,
                              (p.radius*np.cos(a),p.radius*np.sin(a),z), (24,24,6)))
        # Distribution belt occupancy: straight tangent + enclosing corner box.
        # Belt material moves, but its occupied volume is stationary.
        mid = a+np.pi/6
        rr = 40/np.pi+1.5
        result.append(Box(f'fixed/distribution_belt_segment{i}',-1,
                          ((p.radius*np.cos(np.pi/6)+rr)*np.cos(mid),
                           (p.radius*np.cos(np.pi/6)+rr)*np.sin(mid),333),
                          (p.radius,3,8),yaw=mid+np.pi/2))
        result.append(Box(f'fixed/distribution_belt_corner{i}',-1,
                          (p.radius*np.cos(a),p.radius*np.sin(a),333), (32,32,8)))
    result.extend([
        Box('fixed/base',-1,(0,0,-14),(2*(p.radius+16),2*(p.radius+16),12)),
        Box('fixed/roof',-1,(0,0,311),(2*(p.radius+16),2*(p.radius+16),12)),
        Box('fixed/column',-1,(0,0,148),(24,24,312)),
        Box('fixed/knob',-1,(0,0,334),(50,50,20)),
        Box('fixed/central_input_shaft',-1,(0,0,351),(6,6,38),own_shaft=6),
        # Conservative complete input-belt corridor, two heights separate belts.
        Box('fixed/input_belt_corridor',-1,(p.radius/2+6.5,0,360),
            (p.radius+31,48,8)),
        Box('input20T',6,(0,0,360),(18,18,12)),
    ])
    return result


def allowed_pair(a,b):
    # Internal contacts in a door and its intended hinge/shaft interface are
    # excluded ONLY here. Detailed hinge fit and internal assembly not certified.
    if a.group == b.group:
        return False
    if a.group >= 0 and b.own_shaft == a.group:
        return False
    if b.group >= 0 and a.own_shaft == b.group:
        return False
    # Explicit intentional transmission contacts; no general moving/fixed skip.
    names = (a.name,b.name)
    if any('distribution_pulley' in n for n in names) and any('distribution_belt' in n for n in names):
        return False
    if any(('input_driven' in n or n=='input20T') for n in names) and any('input_belt_corridor' in n for n in names):
        return False
    return True


def poses(p, bs, angles):
    angles = np.atleast_1d(angles)
    centers = np.broadcast_to(np.array([b.center for b in bs],float),
                              (len(angles),len(bs),3)).copy()
    yaw = np.zeros((len(angles),len(bs)))
    for j,b in enumerate(bs):
        if b.group < 0:
            yaw[:,j] = b.yaw
            continue
        if b.group == 6:
            yaw[:,j] = np.deg2rad(-3*angles)
            continue
        phi = np.deg2rad(b.group*60)
        a = phi + np.deg2rad(120-angles)
        c,s = np.cos(a),np.sin(a)
        centers[:,j,0] = p.radius*np.cos(phi)+c*b.center[0]-s*b.center[1]
        centers[:,j,1] = p.radius*np.sin(phi)+s*b.center[0]+c*b.center[1]
        yaw[:,j] = a
    return centers,yaw


def sat_separations(centers,yaw,sizes,pairs):
    """Return signed SAT separation for every angle and pair, in mm."""
    ia,ib = np.asarray(pairs).T
    d = centers[:,ib]-centers[:,ia]
    u = np.stack((np.cos(yaw),np.sin(yaw)),axis=-1)
    v = np.stack((-np.sin(yaw),np.cos(yaw)),axis=-1)
    axes = np.stack((u[:,ia],v[:,ia],u[:,ib],v[:,ib]),axis=2)
    gap = np.abs(np.einsum('api,apki->apk',d[:,:,:2],axes))
    for ids in (ia,ib):
        gap -= (np.abs(np.einsum('api,apki->apk',u[:,ids],axes))*sizes[ids,0][None,:,None]/2
                + np.abs(np.einsum('api,apki->apk',v[:,ids],axes))*sizes[ids,1][None,:,None]/2)
    zgap = np.abs(d[:,:,2])-(sizes[ia,2]+sizes[ib,2])[None,:]/2
    return np.maximum(gap.max(axis=-1),zgap)


def scan(p, angles=None):
    if angles is None:
        angles = np.arange(241,dtype=float)
    angles = np.asarray(angles,float)
    bs = boxes(p)
    pairs = [(i,j) for i in range(len(bs)) for j in range(i+1,len(bs))
             if allowed_pair(bs[i],bs[j])]
    c,a = poses(p,bs,angles)
    gaps = sat_separations(c,a,np.array([b.size for b in bs]),pairs)
    row,col = np.unravel_index(np.argmin(gaps),gaps.shape)
    bad = gaps < p.clearance
    hits = np.argwhere(bad)
    examples = []
    for r,k in hits[:30]:
        i,j = pairs[k]
        examples.append(dict(angle=float(angles[r]),a=bs[i].name,b=bs[j].name,
                             sat_gap_mm=float(gaps[r,k])))
    i,j = pairs[col]
    return dict(min_sat_gap_mm=float(gaps[row,col]),angle=float(angles[row]),
                pair=[bs[i].name,bs[j].name],samples=len(angles),
                pairs_per_sample=len(pairs),violations=int(bad.sum()),
                penetrating_pairs_samples=int((gaps < -1e-8).sum()),
                angles_with_violations=angles[bad.any(axis=1)].tolist(),
                examples=examples)


def sweep_radius(p):
    # Full circular sweep upper bound; exceeds or equals actual 0..240 sweep.
    local = 0
    for b in boxes(p):
        if 0 <= b.group < 6:
            local = max(local,np.hypot(abs(b.center[0])+b.size[0]/2,
                                       abs(b.center[1])+b.size[1]/2))
    # The input rotates 3x faster: bound its displacement per door radian too.
    return float(max(local,3*np.hypot(9,9)))


def certify(p, near_band=1.0):
    """1 deg scan plus 0.1 deg refinement near design clearance.

    Also certify between EVERY 1 deg sample by Lipschitz displacement bound.
    At any angle a nearest sample is <=0.5 deg away. Two moving bodies can
    approach by at most 2*r*delta radians. Fixed/moving pairs are bounded too.
    Recompute uniformly at 0.1 deg if the 1 deg bound is inconclusive.
    This is a sufficient continuous guarantee for these envelopes only.
    """
    coarse = scan(p)
    r = sweep_radius(p)
    step = 1.0
    result = coarse
    if coarse['min_sat_gap_mm'] < p.clearance+near_band or coarse['min_sat_gap_mm']-r*np.deg2rad(step) < p.clearance:
        step = 0.1
        # Chunk to bound memory; all 2401 samples including exact endpoints.
        chunks = [scan(p,a) for a in np.array_split(np.linspace(0,240,2401),12)]
        result = min(chunks,key=lambda x:x['min_sat_gap_mm']).copy()
        result['samples'] = sum(x['samples'] for x in chunks)
        result['violations'] = sum(x['violations'] for x in chunks)
        result['penetrating_pairs_samples'] = sum(x['penetrating_pairs_samples'] for x in chunks)
        result['angles_with_violations'] = [v for x in chunks for v in x['angles_with_violations']]
    lower = result['min_sat_gap_mm']-r*np.deg2rad(step)
    return dict(coarse=coarse,refined=result if step<1 else None,
                step_deg=step,continuous_clearance_lower_bound_mm=lower,
                continuous_envelope_pass=bool(lower>=p.clearance),
                required_clearance_mm=p.clearance,
                full_sweep_diameter_upper_bound_mm=2*(p.radius+r))
