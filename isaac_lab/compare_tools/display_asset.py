"""Create a display-only collision-free copy; never edit the source robot USD."""
from pathlib import Path

def create_display_asset(source,destination):
    from pxr import Usd,UsdPhysics
    destination=Path(destination)
    original=Usd.Stage.Open(str(source))
    original.Flatten().Export(str(destination))
    stage=Usd.Stage.Open(str(destination))
    for _ in range(8):
        instances=[prim for prim in stage.Traverse() if prim.IsInstanceable()]
        if not instances:break
        for prim in instances:prim.SetInstanceable(False)
    count=0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False);count+=1
    if not count:raise ValueError('Display asset has no verified collision shapes')
    stage.GetRootLayer().Save()
    return str(destination),count
