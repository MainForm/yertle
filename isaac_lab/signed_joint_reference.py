"""Opt-in API-signed joint coordinates; preserve legacy assets/checkpoints."""
import hashlib
import math
import uuid
from pathlib import Path

REFERENCE_DEG = {"shoulder": 0.0, "thigh": 34.4, "shin": -51.6}


def create_signed_usd(source_path):
    """Reverse thigh/shin coordinate directions without changing link geometry.

    Rotate BOTH joint frames by 180 degrees around an axis perpendicular to
    their local axis. Reverse limits and any authored angular-drive targets.
    The resulting articulation represents q_signed = -q_legacy for these
    eight joints. Original USD/URDF files are never modified.
    """
    from pxr import Gf, Usd, UsdPhysics
    source = Path(source_path).resolve()
    # Flattened asset paths are host-local; Linux and Windows must not share a
    # generated layer just because their original model bytes are identical.
    digest = hashlib.sha256(source.read_bytes() + str(source).encode("utf-8")).hexdigest()[:16]
    output = Path(__file__).parent / "runs" / "model_variants" / f"yertle_api_signed_v1_{digest}.usda"
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        cached = Usd.Stage.Open(str(output))
        if cached.GetRootLayer().customLayerData.get("yertleSignedJointVersion") == 1:
            return str(output.resolve())
    temporary = output.with_name(f"{output.stem}-{uuid.uuid4().hex}.usda")
    original = Usd.Stage.Open(str(source))
    original.Flatten().Export(str(temporary))
    stage = Usd.Stage.Open(str(temporary))
    changed = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint) or not prim.GetName().endswith(("_thigh", "_shin")):
            continue
        joint = UsdPhysics.RevoluteJoint(prim)
        axis = joint.GetAxisAttr().Get()
        perpendicular = Gf.Vec3f(0, 1, 0) if axis == "X" else Gf.Vec3f(1, 0, 0)
        flip = Gf.Quatf(0, perpendicular)
        for attr in (joint.GetLocalRot0Attr(), joint.GetLocalRot1Attr()):
            attr.Set((attr.Get() or Gf.Quatf(1)) * flip)
        lower, upper = joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()
        joint.GetLowerLimitAttr().Set(-upper)
        joint.GetUpperLimitAttr().Set(-lower)
        for name in ("drive:angular:physics:targetPosition", "drive:angular:physics:targetVelocity"):
            attr = prim.GetAttribute(name)
            if attr and attr.HasAuthoredValueOpinion():
                attr.Set(-attr.Get())
        changed.append(prim.GetName())
    expected = {f"{leg}_{kind}" for leg in ("lf", "rf", "lb", "rb") for kind in ("thigh", "shin")}
    if len(changed) != 8 or set(changed) != expected:
        raise ValueError(f"Expected eight unique thigh/shin revolute joints, found {changed}")
    layer = stage.GetRootLayer()
    layer.customLayerData = dict(layer.customLayerData, yertleSignedJointVersion=1)
    layer.Save()
    # Publish only a complete validated variant, not a half-converted cache.
    temporary.replace(output)
    return str(output.resolve())


def configure_signed_joint_reference(env_cfg):
    """Share one reference between action offsets, observations and rewards."""
    robot = env_cfg.scene.robot
    robot.spawn.usd_path = create_signed_usd(robot.spawn.usd_path)
    robot.init_state.joint_pos = {f".*_{kind}": math.radians(angle) for kind, angle in REFERENCE_DEG.items()}
    env_cfg.actions.joint_pos.use_default_offset = True
