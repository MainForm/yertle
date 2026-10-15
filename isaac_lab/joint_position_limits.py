"""Runtime removal of independent revolute position stops, without changing frames."""
import torch
import hashlib
import uuid
from pathlib import Path


def create_unlimited_position_usd(source_path):
    from pxr import Usd, UsdPhysics
    source=Path(source_path).resolve()
    digest=hashlib.sha256(source.read_bytes()+str(source).encode()).hexdigest()[:16]
    output=Path(__file__).parent/'runs/model_variants'/f'yertle_position_unlimited_v1_{digest}.usda'
    output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists():
        return str(output.resolve())
    temporary=output.with_name(output.stem+'-'+uuid.uuid4().hex+'.usda')
    Usd.Stage.Open(str(source)).Flatten().Export(str(temporary))
    stage=Usd.Stage.Open(str(temporary));changed=[]
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):continue
        # Unauthored schema limits default to -inf/+inf; parsed as FREE at creation.
        prim.RemoveProperty('physics:lowerLimit')
        prim.RemoveProperty('physics:upperLimit')
        changed.append(prim.GetName())
    expected={f'{leg}_{kind}' for leg in ('lf','rf','lb','rb') for kind in ('shoulder','thigh','shin')}
    if len(changed)!=12 or set(changed)!=expected:
        raise ValueError(f'Unexpected revolute joint layout: {changed}')
    stage.GetRootLayer().Save();temporary.replace(output)
    return str(output.resolve())


def release_joint_position_limits(env, env_ids=None):
    """Release all 12 position stops; retain torque/speed and hardware range cost.

    The spawn USD already has no position limits. Never call set_dof_limits
    with infinities on a live limited articulation: this backend rejects it.
    Repair soft buffers explicitly, avoiding an infinite midpoint calculation.
    """
    robot = env.scene['robot']
    if len(robot.joint_names) != 12:
        raise ValueError('Expected the 12 Yertle revolute joints')
    actual = robot.root_physx_view.get_dof_limits().to(robot.device)
    # Some PhysX backends represent unlimited bounds with large finite values.
    if not ((actual[..., 0] < -1e6) & (actual[..., 1] > 1e6)).all():
        raise RuntimeError('PhysX did not release joint position stops')
    robot.data.joint_pos_limits.copy_(actual)
    robot.data.soft_joint_pos_limits.copy_(actual)
    print('JOINT_POSITION_LIMITS released=12 fixed_soft_limit_reward=disabled hardware_range_reward=retained torque_speed_limits=unchanged', flush=True)


def configure_reward_only_position_limits(cfg):
    from isaaclab.managers import EventTermCfg
    cfg.scene.robot.spawn.usd_path=create_unlimited_position_usd(cfg.scene.robot.spawn.usd_path)
    cfg.events.release_joint_position_limits = EventTermCfg(func=release_joint_position_limits, mode='startup')
