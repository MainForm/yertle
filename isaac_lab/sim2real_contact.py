"""Opt-in contact settings shared by flat Yertle training and playback."""
import math

import torch
from isaaclab.managers import EventTermCfg, ManagerTermBase
from isaaclab.utils.math import quat_apply

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


class TiltDrivenCenterOfMass(ManagerTermBase):
    """Shift the body CoM in local XY while preserving root-link velocity."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._robot = env.scene["robot"]
        self._body = self._robot.body_names.index("base_link")
        self._coms = self._robot.root_physx_view.get_coms().cpu().clone()
        self._nominal = self._coms[:, self._body, :3].to(env.device).clone()
        self._offsets = torch.zeros_like(self._nominal)
        self._all_cpu = torch.arange(env.num_envs, dtype=torch.int32)
        print("SIM2REAL_COM "
              f"cap_mm={cfg.params['cap_m'] * 1000:g} deadband_deg={cfg.params['deadband_deg']:g} "
              f"full_offset_deg={cfg.params['full_offset_deg']:g} filter_s={cfg.params['filter_s']:g} "
              f"period_s={env.step_dt:g}", flush=True)

    def _apply(self, offsets, env_ids):
        rows = slice(None) if env_ids is None else env_ids
        cpu_rows = self._all_cpu if env_ids is None else env_ids.to(device="cpu", dtype=torch.int32)
        robot = self._robot
        data = robot.data
        delta = quat_apply(data.root_link_quat_w[rows], offsets - self._offsets[rows])
        velocity = data.root_com_vel_w[rows].clone()
        velocity[:, :3] += torch.linalg.cross(velocity[:, 3:], delta, dim=-1)
        position = self._nominal[rows] + offsets
        self._coms[cpu_rows, self._body, :3] = position.cpu()
        robot.root_physx_view.set_coms(self._coms, cpu_rows)
        # Keep Isaac Lab's same-step CoM cache consistent with the PhysX write.
        data.body_com_pos_b[rows, self._body] = position
        robot.write_root_com_velocity_to_sim(velocity, env_ids=env_ids)
        self._offsets[rows] = offsets

    def __call__(self, env, env_ids, cap_m, deadband_deg, full_offset_deg, filter_s):
        gravity = self._robot.data.projected_gravity_b
        horizontal = gravity[:, :2]
        length = torch.linalg.vector_norm(horizontal, dim=-1)
        angle = torch.rad2deg(torch.atan2(length, -gravity[:, 2]))
        fraction = ((angle - deadband_deg) / (full_offset_deg - deadband_deg)).clamp(0, 1)
        target = horizontal * (cap_m * fraction / length.clamp_min(1e-9))[:, None]
        offsets = self._offsets.clone()
        offsets[:, :2].lerp_(target, 1.0 - math.exp(-env.step_dt / filter_s))
        self._apply(offsets, None)

    def reset(self, env_ids=None):
        if env_ids is not None:
            env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self._offsets.device)
        offsets = self._offsets if env_ids is None else self._offsets[env_ids]
        self._apply(torch.zeros_like(offsets), env_ids)


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
    control_dt = env_cfg.sim.dt * env_cfg.decimation
    env_cfg.events.sim2real_center_of_mass = EventTermCfg(
        func=TiltDrivenCenterOfMass,
        mode="interval",
        interval_range_s=(control_dt, control_dt),
        is_global_time=True,
        params={"cap_m": 0.015, "deadband_deg": 3.0, "full_offset_deg": 25.0, "filter_s": 0.1},
    )
