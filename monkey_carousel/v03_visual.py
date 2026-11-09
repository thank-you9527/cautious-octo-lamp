"""V0.3 research artifacts, including actual sampled involute visual meshes."""
import numpy as np
from PIL import Image,ImageDraw
from .v03 import bodies,transformed
from .gears import outline,positioned
from .visualize import COLORS


def prism_mesh(xy,z):
    import trimesh
    n=len(xy);center=xy.mean(axis=0)
    vertices=np.vstack([np.c_[xy,np.full(n,z[0])],np.c_[xy,np.full(n,z[1])],
                        [*center,z[0]],[*center,z[1]]])
    faces=[]
    for i in range(n):
        j=(i+1)%n
        faces += [[j,i,2*n],[i+n,j+n,2*n+1],[i,j,j+n],[i,j+n,i+n]]
    return trimesh.Trimesh(vertices=vertices,faces=faces,process=True)


def tooth_polygon(b,angle):
    if b.name=='drive/detent_cam':
        from .indexing import cam_outline
        return cam_outline(angle/5)
    if b.name.endswith('/gear40'):
        return positioned(outline(40),184.5-angle,b.origin)
    phases={'drive/sun200':(200,0,.2),'drive/output90':(90,0,.2),
            'drive/lay30':(30,186,-.6),'drive/lay100':(100,181.8,-.6),'drive/input20':(20,0,3)}
    if b.name not in phases:return None
    n,phase,rate=phases[b.name]
    return positioned(outline(n),phase+rate*angle,b.origin)


def render(d,angle):
    im=Image.new('RGB',(1100,950),(245,246,250));draw=ImageDraw.Draw(im)
    view=np.array([[.707,-.707,0],[.32,.32,-.89],[.63,.63,.45]])
    polys=[]
    for b in bodies(d):
        if 'support_plate' in b.name or b.name in ('fixed/base','fixed/roof'):
            continue
        # Faceted circular envelopes in GIF keep it responsive/readable; GLB
        # uses actual sampled gear teeth. No decorative parts are included.
        xy=transformed(b,[angle])[0];n=len(xy)
        pts=np.vstack([np.c_[xy,np.full(n,b.z[0])],np.c_[xy,np.full(n,b.z[1])]])@view.T
        pts[:,:2]=pts[:,:2]*.95+[550,650]
        color=COLORS[b.actor%6] if b.actor>=0 else (110,115,125)
        if '/doll' in b.name:color=tuple(int(c*.5+120) for c in color)
        faces=[list(range(n)),list(range(n,2*n))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
        for face in faces:polys.append((pts[face,2].mean(),pts[face],color))
    # True orthographic depth buffer. Centroid painter sorting can incorrectly
    # draw an interior doll over a closed wall, particularly at polygon cycles.
    pixels=np.asarray(im).copy();depth=np.full((950,1100),-np.inf)
    for _,poly,color in polys:
        # Reconstruct per-vertex depth from the same world faces below; `polys`
        # retains XYZ so occlusion is interpolated rather than centroid sorted.
        for i in range(1,len(poly)-1):
            t=poly[[0,i,i+1]]
            x0=max(0,int(np.floor(t[:,0].min())));x1=min(1099,int(np.ceil(t[:,0].max())))
            y0=max(0,int(np.floor(t[:,1].min())));y1=min(949,int(np.ceil(t[:,1].max())))
            if x1<x0 or y1<y0:continue
            den=(t[1,1]-t[2,1])*(t[0,0]-t[2,0])+(t[2,0]-t[1,0])*(t[0,1]-t[2,1])
            if abs(den)<1e-10:continue
            xx,yy=np.meshgrid(np.arange(x0,x1+1)+.5,np.arange(y0,y1+1)+.5)
            a=((t[1,1]-t[2,1])*(xx-t[2,0])+(t[2,0]-t[1,0])*(yy-t[2,1]))/den
            b=((t[2,1]-t[0,1])*(xx-t[2,0])+(t[0,0]-t[2,0])*(yy-t[2,1]))/den
            c=1-a-b;z=a*t[0,2]+b*t[1,2]+c*t[2,2]
            crop=depth[y0:y1+1,x0:x1+1]
            mask=(a>=-1e-9)&(b>=-1e-9)&(c>=-1e-9)&(z>crop)
            crop[mask]=z[mask];pixels[y0:y1+1,x0:x1+1][mask]=color
    for _,poly,_ in polys:
        for i in range(len(poly)):
            a,b=poly[i],poly[(i+1)%len(poly)]
            edge=np.linspace(a,b,max(2,int(np.linalg.norm(a[:2]-b[:2])*2)))
            xx=np.clip(edge[:,0].astype(int),0,1099);yy=np.clip(edge[:,1].astype(int),0,949)
            visible=edge[:,2]>=depth[yy,xx]-1
            pixels[yy[visible],xx[visible]]=(60,65,70)
    im=Image.fromarray(pixels);draw=ImageDraw.Draw(im)
    lines=['MONKEY CAROUSEL V0.3 | LONG PANELS + FIXED CORNER TUBES',
           f'Door {angle:.0f}/240 deg | knob {3*angle:.0f}/720 deg | R={d.radius:g} mm',
           'm1.5 200T/40T + reverted 15:1 reducer | roof plates hidden in GIF',
           'Conservative bodies shown. Preview is NOT a mechanism/strength certificate.']
    for i,line in enumerate(lines):draw.text((25,20+22*i),line,fill='black')
    return im


def export(d,out):
    import trimesh
    import cadquery as cq
    for angle,label in [(0,'storage'),(240,'display')]:
        scene=trimesh.Scene();assembly=cq.Assembly(name='V03_clearance_envelopes_'+label)
        for b in bodies(d):
            xy=transformed(b,[angle])[0]
            visual=tooth_polygon(b,angle)
            mesh=prism_mesh(xy if visual is None else visual,b.z)
            mesh.visual.face_colors=[*(COLORS[b.actor%6] if b.actor>=0 else (110,115,125)),220]
            scene.add_geometry(mesh,node_name=b.name)
            if 'corner_tube' in b.name:
                solid=cq.Workplane('XY').circle(d.post_radius).circle(4).extrude(b.z[1]-b.z[0]).translate((*b.origin,b.z[0]))
            elif b.name=='drive/detent_cam':
                from .indexing import cam_outline
                cam=cam_outline(angle/5)
                solid=cq.Workplane('XY').polyline(cam.tolist()).close().extrude(6).translate((0,0,375))
            else:
                solid=cq.Workplane('XY').polyline(xy.tolist()).close().extrude(b.z[1]-b.z[0]).translate((0,0,b.z[0]))
            assembly.add(solid,name=b.name.replace('/','_'))
        scene.export(str(out/f'{label}_research.glb'))
        assembly.export(str(out/f'{label}_clearance.step'))
        render(d,angle).save(out/f'{label}.png')
    frames=[render(d,a) for a in range(0,241,4)]
    frames[0].save(out/'motion.gif',save_all=True,append_images=frames[1:],duration=90,loop=0)
    # Two halves prove basic split geometry, not joint or load adequacy.
    assembly=cq.Assembly(name='panel_split_concept')
    for name,z,h in [('lower',0,d.height/2),('upper',d.height/2,d.height/2)]:
        solid=cq.Workplane('XY').polyline(d.panel.tolist()).close().extrude(h).translate((0,0,z))
        assembly.add(solid,name=name)
    assembly.export(str(out/'panel_split_concept.step'))
