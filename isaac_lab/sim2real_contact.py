"""Opt-in contact settings shared by flat Yertle training and playback."""
import torch
from isaaclab.managers import EventTermCfg

from .foot_geometry import FOOT_NAMES


GROUND_STIFFNESS = 10_000.0
GROUND_DAMPING = 30.0


def verify_ground_material(env):
    import omni.usd
    from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    terrain = stage.GetPrimAtPath(env.scene.terrain.cfg.prim_path)
    colliders = [prim for prim in Usd.PrimRange(terrain) if prim.HasAPI(UsdPhysics.CollisionAPI)]
    if len(colliders) != 1:
        raise RuntimeError("The sim-to-real contact profile requires one flat ground collider")
    material = UsdShade.MaterialBindingAPI(colliders[0]).ComputeBoundMaterial("physics")[0].GetPrim()
    physx = PhysxSchema.PhysxMaterialAPI(material)
    usd = UsdPhysics.MaterialAPI(material)
    actual = (usd.GetStaticFrictionAttr().Get(), usd.GetDynamicFrictionAttr().Get(),
              physx.GetFrictionCombineModeAttr().Get(), usd.GetRestitutionAttr().Get(),
              physx.GetCompliantContactStiffnessAttr().Get(), physx.GetCompliantContactDampingAttr().Get())
    expected = (1.0, 1.0, "multiply", 0.0, GROUND_STIFFNESS, GROUND_DAMPING)
    if actual != expected:
        raise RuntimeError(f"Ground contact material {material.GetPath()}: expected {expected}, got {actual}")
    print(f"SIM2REAL_GROUND_MATERIAL static=1.0 dynamic=1.0 combine=multiply restitution=0.0 "
          f"stiffness_N_m={actual[4]} damping_N_s_m={actual[5]}", flush=True)


def set_foot_material(env, env_ids, static_friction, dynamic_friction):
    robot = env.scene["robot"]
    view = robot.root_physx_view
    props = view.get_material_properties()
    ids = (torch.arange(env.num_envs, dtype=torch.int32) if env_ids is None
           else torch.as_tensor(env_ids, dtype=torch.int32, device="cpu"))
    paths = view.link_paths[0]
    if [path.rsplit("/", 1)[-1] for path in paths] != robot.body_names:
        raise RuntimeError("Unexpected PhysX material link ordering")
    target = props.new_tensor((static_friction, dynamic_friction, 0.0))
    foot_shapes = []
    offset = 0
    for path, name in zip(paths, robot.body_names, strict=True):
        count = robot._physics_sim_view.create_rigid_body_view(path).max_shapes
        if name in FOOT_NAMES:
            if count == 0:
                raise RuntimeError(f"Missing foot collider on {name}")
            foot_shapes.append(slice(offset, offset + count))
        offset += count
    if offset != view.max_shapes or len(foot_shapes) != len(FOOT_NAMES):
        raise RuntimeError("Unexpected Yertle collider layout")
    for shapes in foot_shapes:
        props[ids, shapes] = target
    view.set_material_properties(props, ids)
    actual = view.get_material_properties()
    for shapes in foot_shapes:
        torch.testing.assert_close(actual[ids, shapes], props[ids, shapes])
    print(f"SIM2REAL_FOOT_MATERIAL static={static_friction} dynamic={dynamic_friction} restitution=0.0", flush=True)
    verify_ground_material(env)


def configure_sim2real_contact(env_cfg):
    terrain = env_cfg.scene.terrain
    terrain.physics_material = terrain.physics_material.replace(
        static_friction=1.0, dynamic_friction=1.0, friction_combine_mode="multiply", restitution=0.0,
        restitution_combine_mode="multiply", compliant_contact_stiffness=GROUND_STIFFNESS,
        compliant_contact_damping=GROUND_DAMPING,
    )
    # This startup event follows the inherited material randomization and
    # overwrites only the four foot links.
    env_cfg.events.sim2real_foot_material = EventTermCfg(
        func=set_foot_material,
        mode="startup",
        params={"static_friction": 0.40, "dynamic_friction": 0.35},
    )
