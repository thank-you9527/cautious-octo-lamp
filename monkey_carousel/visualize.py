"""Visual outputs depict collision envelopes, never manufacturing geometry."""
import numpy as np
from PIL import Image, ImageDraw
from .model import boxes,poses


def corners(center,size,yaw):
    local = np.array([[x,y,z] for z in (-.5,.5) for y in (-.5,.5) for x in (-.5,.5)])*size
    c,s = np.cos(yaw),np.sin(yaw)
    rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    return local@rotation.T+center


COLORS = [(230,157,55),(80,155,210),(160,115,205),(70,175,140),(220,105,120),(165,180,60)]
FACES = [(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)]


def frame(p,angle):
    bs = boxes(p)
    centers,yaw = poses(p,bs,[angle])
    img = Image.new('RGB',(1000,800),(245,246,250))
    draw = ImageDraw.Draw(img)
    polygons = []
    # Orthographic oblique view: true Z is visible; painter's depth for preview.
    view = np.array([[.707,-.707,0],[.32,.32,-.89],[.63,.63,.45]])
    scale = min(1.2,600/(2*(p.radius+p.length+65)))
    for i,b in enumerate(bs):
        if b.name in ('fixed/roof','fixed/base'):
            continue  # Intentionally hidden in GIF only to expose the envelopes.
        pts = corners(centers[0,i],np.array(b.size),yaw[0,i])@view.T
        pts[:,:2] = pts[:,:2]*scale+[500,520]
        color = COLORS[b.group%6] if b.group>=0 else (100,105,115)
        if '/doll' in b.name:
            color = tuple(min(255,int(c*.5+120)) for c in color)
        for f in FACES:
            polygons.append((pts[list(f),2].mean(),pts[list(f),:2],color))
    for _,poly,color in sorted(polygons,key=lambda v:v[0]):
        draw.polygon([tuple(v) for v in poly],fill=color,outline=(55,60,65))
    draw.text((25,20),'MONKEY CAROUSEL | PHASE 1 - COLLISION ENVELOPES',fill='black')
    draw.text((25,45),f'Outward rotation {angle:.0f}/240 deg | R={p.radius:g} L={p.length:g} mm',fill='black')
    draw.text((25,70),'Six fixed side hinges | roof and base hidden ONLY in preview',fill='black')
    draw.text((25,750),'Animation is NOT verification. See collision_report.json and design_report.md.',fill='black')
    return img


def export(p,out):
    import trimesh
    import cadquery as cq
    bs = boxes(p)
    for angle,label in [(0,'storage'),(240,'display')]:
        centers,yaw = poses(p,bs,[angle])
        scene = trimesh.Scene()
        assembly = cq.Assembly(name=f'MONKEY_CAROUSEL_ENVELOPES_{label}')
        for i,b in enumerate(bs):
            transform = trimesh.transformations.rotation_matrix(yaw[0,i],[0,0,1])
            transform[:3,3] = centers[0,i]
            mesh = trimesh.creation.box(extents=b.size,transform=transform)
            mesh.visual.face_colors = [*(COLORS[b.group%6] if b.group>=0 else (100,105,115)),180]
            scene.add_geometry(mesh,node_name=b.name)
            solid = cq.Workplane('XY').box(*b.size).rotate((0,0,0),(0,0,1),float(np.rad2deg(yaw[0,i]))).translate(tuple(centers[0,i]))
            assembly.add(solid,name=b.name.replace('/','_'))
        scene.export(str(out/f'{label}_envelopes.glb'))
        assembly.export(str(out/f'{label}_envelopes.step'))
        frame(p,angle).save(out/f'{label}.png')
    frames = [frame(p,a) for a in np.arange(0,241,4)]
    frames[0].save(out/'motion.gif',save_all=True,append_images=frames[1:],
                   duration=90,loop=0,optimize=True)
