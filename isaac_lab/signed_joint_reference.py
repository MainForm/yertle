"""Optional standing reference and limits; preserve original joint coordinates."""
import hashlib
import math
import uuid
from pathlib import Path

REFERENCE_DEG = {"shoulder": 0.0, "thigh": 34.0, "shin": -85.0}


def configure_reward_only_position_limits(cfg):
    # Lazy import preserves standalone reference/calibration unit-test loading.
    from isaac_lab.joint_position_limits import configure_reward_only_position_limits as configure
    configure(cfg)


def create_signed_usd(source_path):
    """Retain changed limits, but do NOT reverse axes, frames or drive targets.

    Historical function/CLI names remain as compatibility aliases, not an
    instruction to change coordinates. Axis-reversed v6.3 checkpoints are
    incompatible with this restored model.
    """
    from pxr import Usd, UsdPhysics
    source = Path(source_path).resolve()
    # Flattened asset paths are host-local; Linux and Windows must not share a
    # generated layer just because their original model bytes are identical.
    digest = hashlib.sha256(source.read_bytes() + str(source).encode("utf-8")).hexdigest()[:16]
    output = Path(__file__).parent / "runs" / "model_variants" / f"yertle_original_axes_limits_v2_{digest}.usda"
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        cached = Usd.Stage.Open(str(output))
        if cached.GetRootLayer().customLayerData.get("yertleOriginalAxesLimitsVersion") == 2:
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
        lower, upper = joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()
        joint.GetLowerLimitAttr().Set(-upper)
        joint.GetUpperLimitAttr().Set(-lower)
        changed.append(prim.GetName())
    expected = {f"{leg}_{kind}" for leg in ("lf", "rf", "lb", "rb") for kind in ("thigh", "shin")}
    if len(changed) != 8 or set(changed) != expected:
        raise ValueError(f"Expected eight unique thigh/shin revolute joints, found {changed}")
    layer = stage.GetRootLayer()
    layer.customLayerData = dict(layer.customLayerData, yertleOriginalAxesLimitsVersion=2)
    layer.Save()
    # Publish only a complete validated variant, not a half-converted cache.
    temporary.replace(output)
    return str(output.resolve())


def configure_signed_joint_reference(env_cfg):
    """Share the reference and use the requested reward-only position envelope.

    Axes/frames remain original. Position stops are removed in a separate
    spawn variant; old signed-reference checkpoints now see changed physics.
    """
    robot = env_cfg.scene.robot
    robot.spawn.usd_path = create_signed_usd(robot.spawn.usd_path)
    robot.init_state.joint_pos = {f".*_{kind}": math.radians(angle) for kind, angle in REFERENCE_DEG.items()}
    env_cfg.actions.joint_pos.use_default_offset = True
    configure_reward_only_position_limits(env_cfg)
