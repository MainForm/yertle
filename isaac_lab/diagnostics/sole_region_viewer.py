"""Read-only, static Isaac Lab inspection of the exact permitted sole mesh.

No physics stepping, PPO, checkpoints, hardware IO, asset-file edits, or API
mapping changes. Green faces come directly from load_sole_points(triangles=True),
the same geometry used by GroundContacts. Cyan points sample the 5 mm allowance;
they are diagnostic markers, not collision geometry.
"""
import argparse
from pathlib import Path
import sys,time,json
import torch
from isaaclab.app import AppLauncher

parser=argparse.ArgumentParser()
parser.add_argument('--frames',type=int,default=0,help='0: until window closes; positive: bounded verification')
AppLauncher.add_app_launcher_args(parser)
args=parser.parse_args()
app=AppLauncher(args).app

def main():
    from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Sdf,Gf
    import omni.usd
    import omni.ui as ui
    from isaacsim.core.utils.viewports import set_camera_view
    lab=Path(__file__).resolve().parents[1]
    sys.path.insert(0,str(lab.parent))
    from isaac_lab.foot_geometry import load_sole_points,FOOT_NAMES
    from isaac_lab.ground_contact import surface_distance
    usd=lab.parent/'simulation/usd/yertle.usd'
    points=load_sole_points(usd,triangles=True)
    source=Usd.Stage.Open(str(usd));cache=UsdGeom.XformCache()
    bodies={p.GetName():p for p in source.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    base=bodies['base_link'];world={str(base.GetPath()):cache.GetLocalToWorldTransform(base)}
    def frame(j,i):
        pos=(j.GetLocalPos0Attr() if i==0 else j.GetLocalPos1Attr()).Get()
        rot=(j.GetLocalRot0Attr() if i==0 else j.GetLocalRot1Attr()).Get()
        return Gf.Matrix4d(Gf.Rotation(Gf.Quatd(rot)),Gf.Vec3d(pos))
    edges=[]
    for p in source.Traverse():
        if not p.IsA(UsdPhysics.Joint):continue
        j=UsdPhysics.Joint(p);a=j.GetBody0Rel().GetTargets();b=j.GetBody1Rel().GetTargets()
        if a and b:
            axis=UsdPhysics.RevoluteJoint(p).GetAxisAttr().Get() if p.IsA(UsdPhysics.RevoluteJoint) else None
            edges.append((p.GetName(),str(a[0]),str(b[0]),frame(j,0),frame(j,1),axis))
    pending=list(edges)
    while pending:
        progress=False
        for name,a,b,l0,l1,axis in pending[:]:
            if a not in world:continue
            rot=Gf.Matrix4d(1)
            if axis:
                angle=next((value for kind,value in [('shoulder',0),('thigh',34),('shin',-85)] if name.endswith('_'+kind)),0)
                rot.SetRotate(Gf.Rotation({'X':Gf.Vec3d(1,0,0),'Y':Gf.Vec3d(0,1,0),'Z':Gf.Vec3d(0,0,1)}[axis],angle))
            world[b]=l1.GetInverse()*rot*l0*world[a]
            pending.remove((name,a,b,l0,l1,axis));progress=True
        if not progress:raise RuntimeError('Unresolved articulation hierarchy')
    omni.usd.get_context().new_stage()
    stage=omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
    stage.DefinePrim('/World','Xform')
    root=stage.DefinePrim('/World/Robot','Xform');root.GetReferences().AddReference(str(usd))
    def mapped(p):return str(p.GetPath().ReplacePrefix(source.GetDefaultPrim().GetPath(),Sdf.Path('/World/Robot')))
    root_translation=Gf.Matrix4d(1);root_translation.SetTranslate((0,0,.25))
    UsdGeom.Xformable(root).MakeMatrixXform().Set(cache.GetLocalToWorldTransform(source.GetDefaultPrim())*root_translation)
    # Static FK pose in this in-memory diagnostic stage only.
    for name,p in bodies.items():
        body=stage.GetPrimAtPath(mapped(p))
        assert body.IsValid(),str(p.GetPath())
        parent=str(p.GetParent().GetPath())
        parent_world=world.get(parent,cache.GetLocalToWorldTransform(p.GetParent()))
        UsdGeom.Xformable(body).MakeMatrixXform().Set(world[str(p.GetPath())]*parent_world.GetInverse())
    def material(path,color,opacity=1.):
        mat=UsdShade.Material.Define(stage,path)
        shader=UsdShade.Shader.Define(stage,path+'/Shader');shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(tuple(v*.35 for v in color))
        shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(opacity)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        return mat
    green=material('/World/Materials/PermittedSole',(0.05,.95,.18))
    cyan=material('/World/Materials/Allowance',(.1,.65,1.),.35)
    grey=material('/World/Materials/Robot',(.42,.46,.52))
    for p in stage.Traverse(Usd.TraverseInstanceProxies()):
        if p.GetPath().HasPrefix(root.GetPath()) and p.IsA(UsdGeom.Mesh) and not p.IsInstanceProxy():
            UsdShade.MaterialBindingAPI.Apply(p).Bind(grey,bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    overlays=[]
    def highlight(path,triangles):
        mesh=UsdGeom.Mesh.Define(stage,path)
        vertices=[Gf.Vec3f(*triangles[t][k:k+3]) for t in range(len(triangles)) for k in (0,3,6)]
        mesh.CreatePointsAttr(vertices);mesh.CreateFaceVertexCountsAttr([3]*len(triangles))
        mesh.CreateFaceVertexIndicesAttr(list(range(len(vertices))))
        mesh.CreateDoubleSidedAttr(True);mesh.CreateSubdivisionSchemeAttr('none')
        # Face overlay is exact; wire edges remain legible on coincident surfaces.
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(green)
        wire=UsdGeom.BasisCurves.Define(stage,path+'Edges')
        wire.CreateTypeAttr('linear');wire.CreateWrapAttr('nonperiodic')
        wire.CreatePointsAttr([v for i in range(0,len(vertices),3) for v in (vertices[i],vertices[i+1],vertices[i+2],vertices[i])])
        wire.CreateCurveVertexCountsAttr([4]*len(triangles));wire.CreateWidthsAttr([.00035])
        UsdShade.MaterialBindingAPI.Apply(wire.GetPrim()).Bind(green)
        tri=torch.tensor(triangles,dtype=torch.float32).reshape(-1,3,3)
        flat=tri.reshape(-1,3);low=flat.amin(0)-.005;high=flat.amax(0)+.005
        grid=torch.cartesian_prod(*[torch.arange(low[k],high[k]+.0001,.002) for k in range(3)])
        distance=torch.cat([surface_distance(chunk,tri) for chunk in grid.split(256)])
        allowed=grid[(distance<=.005)&(distance>.001)]
        cloud=UsdGeom.Points.Define(stage,path+'Allowance')
        cloud.CreatePointsAttr([Gf.Vec3f(*p) for p in allowed.tolist()])
        cloud.CreateWidthsAttr([.0012]);cloud.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        cloud.CreateDisplayColorAttr([(.1,.65,1.)])
        cloud.CreateDisplayOpacityAttr([.35])
        UsdShade.MaterialBindingAPI.Apply(cloud.GetPrim()).Bind(cyan)
        overlays.extend([mesh,wire,cloud])
        assert len(mesh.GetFaceVertexCountsAttr().Get())==len(triangles)
    for name,tris in zip(FOOT_NAMES,points):
        highlight(mapped(bodies[name])+'/PermittedSole',tris)
    # Enlarged local-shin inspection copy, not a second simulated robot.
    detail=stage.DefinePrim('/World/FootDetail','Xform')
    detail.GetReferences().AddReference(str(usd),bodies['rf_shin'].GetPath())
    detail_scale=Gf.Matrix4d(1);detail_scale.SetScale(4.)
    detail_translation=Gf.Matrix4d(1);detail_translation.SetTranslate((0,.65,.15))
    UsdGeom.Xformable(detail).MakeMatrixXform().Set(detail_scale*detail_translation)
    for p in Usd.PrimRange(detail):
        if p.IsA(UsdGeom.Mesh):UsdShade.MaterialBindingAPI.Apply(p).Bind(grey,bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    highlight('/World/FootDetail/PermittedSole',points[FOOT_NAMES.index('rf_shin')])
    from pxr import UsdLux
    light=UsdLux.DomeLight.Define(stage,'/World/Light');light.CreateIntensityAttr(1200)
    camera=UsdGeom.Camera.Define(stage,'/World/Camera')
    def full():set_camera_view(eye=(.85,-.85,.65),target=(0,0,.19),camera_prim_path='/World/Camera')
    def close():set_camera_view(eye=(.45,.20,.38),target=(0,.65,.15),camera_prim_path='/World/Camera')
    from omni.kit.viewport.utility import get_active_viewport
    viewport=get_active_viewport()
    if viewport:viewport.camera_path='/World/Camera'
    full()
    window=ui.Window('Permitted sole region | read-only',width=430,height=280)
    with window.frame:
        with ui.VStack(spacing=8):
            ui.Label('GREEN = exact Rubber collision-mesh faces',word_wrap=True)
            ui.Label('CYAN = expanded allowance sampled on a 2 mm grid.',word_wrap=True)
            ui.Label('Rubber sides + nearby top/edges allowed within 5 mm.',word_wrap=True)
            ui.Label('Classifier: surface distance <= 5 mm; gap <= 1 mm.',word_wrap=True)
            ui.Label('Static pose 0 / +34 / -85 deg. Physics is NOT running.',word_wrap=True)
            ui.Button('Whole robot',clicked_fn=full)
            ui.Button('Right-front shin enlarged x4',clicked_fn=close)
            def toggle():
                hidden=overlays[0].GetVisibilityAttr().Get()=='invisible'
                for p in overlays:p.GetVisibilityAttr().Set('inherited' if hidden else 'invisible')
            ui.Button('Toggle green surfaces',clicked_fn=toggle)
    # Strong references keep UI callback targets alive for the viewer lifetime.
    print('SOLE_REGION_VIEWER '+json.dumps({'source':str(usd),'surface':'same Rubber collision triangles as GroundContacts','triangles':dict(zip(FOOT_NAMES,map(len,points))),'pose_deg':[0,34,-85],'sole_tolerance_m':.005,'allowance_grid_m':.002,'contact_gap_m':.001,'physics_running':False}),flush=True)
    for i in range(args.frames) if args.frames>0 else iter(int,1):
        if not app.is_running():break
        app.update()
        if not args.headless:time.sleep(.01)
    print('SOLE_REGION_VIEWER_PASS geometry_and_ui_created',flush=True)

try:main()
except Exception:
    import traceback
    traceback.print_exc()
    raise
finally:app.close()
