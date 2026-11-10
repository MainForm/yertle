"""Opt-in contact settings shared by flat Yertle training and playback."""
import torch
from isaaclab.managers import EventTermCfg

from .foot_geometry import FOOT_NAMES


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


def configure_sim2real_contact(env_cfg):
    terrain = env_cfg.scene.terrain
    terrain.physics_material = terrain.physics_material.replace(
        static_friction=1.0, dynamic_friction=1.0, friction_combine_mode="multiply", restitution=0.0,
    )
    # This startup event follows the inherited material randomization and
    # overwrites only the four foot links.
    env_cfg.events.sim2real_foot_material = EventTermCfg(
        func=set_foot_material,
        mode="startup",
        params={"static_friction": 0.40, "dynamic_friction": 0.35},
    )
