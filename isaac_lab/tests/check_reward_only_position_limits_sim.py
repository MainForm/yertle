"""Verify actual PhysX position-stop release, references and finite rollout."""
import argparse
import math
from pathlib import Path
import sys
import torch
from isaaclab.app import AppLauncher
parser=argparse.ArgumentParser();AppLauncher.add_app_launcher_args(parser)
app=AppLauncher(parser.parse_args()).app
try:
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
    import gymnasium as gym
    import isaac_lab
    from isaac_lab.flat_env_cfg import YertleFlatEnvCfg
    from isaac_lab.signed_joint_reference import configure_signed_joint_reference
    from isaac_lab.train_start_pose import configure_reset_pose_pool
    from isaac_lab.play_start_pose import load_pose_json
    cfg=YertleFlatEnvCfg();cfg.scene.num_envs=2;cfg.seed=42
    source_usd=cfg.scene.robot.spawn.usd_path
    configure_signed_joint_reference(cfg)
    from pxr import Usd,UsdPhysics
    before=Usd.Stage.Open(source_usd);after=Usd.Stage.Open(cfg.scene.robot.spawn.usd_path)
    count=0
    for prim in before.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):continue
        converted=after.GetPrimAtPath(prim.GetPath())
        for name in ('physics:axis','physics:localPos0','physics:localPos1','physics:localRot0','physics:localRot1',
                     'drive:angular:physics:targetPosition','drive:angular:physics:targetVelocity'):
            assert prim.GetAttribute(name).Get()==converted.GetAttribute(name).Get(),(prim.GetName(),name)
        joint=UsdPhysics.RevoluteJoint(converted)
        assert joint.GetLowerLimitAttr().Get()==-math.inf and joint.GetUpperLimitAttr().Get()==math.inf
        count+=1
    assert count==12
    print('POSITION_LIMIT_AXES_MAPPING_PASS 12 joints frames axes drive targets unchanged',flush=True)
    pose=load_pose_json(Path(__file__).resolve().parents[1]/'configs/standing_api_reference.json')
    configure_reset_pose_pool(cfg,[],standing_pose=pose)
    cfg.commands.base_velocity.debug_vis=False
    assert cfg.rewards.dof_pos_limits is None
    assert cfg.rewards.motor_operating_range_penalty.weight==-100
    env=gym.make('Isaac-Velocity-Flat-Yertle-v0',cfg=cfg).unwrapped
    try:
        env.reset();robot=env.scene['robot']
        limits=robot.root_physx_view.get_dof_limits()
        assert ((limits[...,0]<-1e6)&(limits[...,1]>1e6)).all()
        ref=robot.data.default_joint_pos.clone()
        ids=[i for i,n in enumerate(robot.joint_names) if n.endswith('_shin')]
        torch.testing.assert_close(ref[:,ids],torch.full_like(ref[:,ids],math.radians(-85)),atol=1e-6,rtol=0)
        action=env.action_manager.get_term('joint_pos')
        action.process_actions(torch.zeros(2,12,device=env.device))
        torch.testing.assert_close(action.processed_actions,ref[:,action._joint_ids])
        # Explicitly set a state beyond the previous -90 deg stop; no target transform changes.
        q=ref.clone();q[:,ids]=math.radians(-100)
        robot.write_joint_state_to_sim(q,torch.zeros_like(q));robot.set_joint_position_target(q)
        env.scene.write_data_to_sim();env.sim.step(render=False);env.scene.update(env.physics_dt)
        measured=robot.data.joint_pos[:,ids]
        assert (measured<math.radians(-95)).all(),measured
        for _ in range(40):
            obs,reward,*_=env.step(torch.zeros(2,12,device=env.device))
            assert torch.isfinite(reward).all()
            assert torch.isfinite(robot.data.joint_pos).all()
            assert torch.isfinite(obs['policy']).all()
        assert not torch.isnan(robot.data.soft_joint_pos_limits).any()
        print('REWARD_ONLY_POSITION_LIMITS_SIM_PASS released=12 old_shin_stop_crossed=True reference_unchanged=True finite_rollout=True motor_range_weight=-100',flush=True)
    finally:env.close()
except BaseException:
    import traceback;traceback.print_exc();print('REWARD_ONLY_POSITION_LIMITS_SIM_FAIL',flush=True);raise
finally:app.close()
