"""Independent quaternion/COM point-velocity check on the actual articulation."""
import argparse
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
    from isaac_lab.flat_env_cfg import YertleFlatEnvCfg_PLAY
    from isaac_lab.signed_joint_reference import configure_signed_joint_reference
    from isaac_lab.play_start_pose import configure_start_pose
    from isaac_lab.foot_geometry import FOOT_NAMES,foot_sole_contact_velocities
    from isaac_lab.flat_env_cfg_reward import feet_slide_sole
    from isaaclab.utils.math import quat_apply
    from isaaclab.managers import SceneEntityCfg
    cfg=YertleFlatEnvCfg_PLAY();cfg.scene.num_envs=2
    configure_signed_joint_reference(cfg);configure_start_pose(cfg,'default',None)
    env=gym.make('Isaac-Velocity-Flat-Yertle-Play-v0',cfg=cfg).unwrapped
    try:
        env.reset();robot=env.scene['robot'];data=robot.data
        sensor=env.scene.sensors['contact_forces']
        entity=SceneEntityCfg('contact_forces',body_names=list(reversed(FOOT_NAMES)),preserve_order=True)
        entity.resolve(env.scene)
        assert env.cfg.rewards.feet_slide.func is feet_slide_sole
        max_error=0.0
        for step in range(31):
            actual=foot_sole_contact_velocities(env)
            ids,points=env._yertle_sole_geometry
            rotations=data.body_quat_w[:,ids,None,:].expand(-1,-1,points.shape[1],-1)
            local=points[None].expand(2,-1,-1,-1)
            rotated=quat_apply(rotations.reshape(-1,4),local.reshape(-1,3)).reshape_as(local)
            world=data.body_pos_w[:,ids,None,:]+rotated
            valid=env._yertle_sole_vertex_mask[None]
            minimum=world[...,2].masked_fill(~valid,float('inf')).amin(dim=-1,keepdim=True)
            chosen=valid & (world[...,2]<=minimum+.001)
            centroid=(world*chosen[...,None]).sum(dim=2)/chosen.sum(dim=2)[...,None]
            expected=data.body_lin_vel_w[:,ids]+torch.linalg.cross(data.body_ang_vel_w[:,ids],centroid-data.body_com_pos_w[:,ids],dim=-1)
            torch.testing.assert_close(actual,expected,atol=2e-6,rtol=1e-5)
            max_error=max(max_error,(actual-expected).abs().max().item())
            contact=sensor.data.net_forces_w_history[:,:,entity.body_ids].norm(dim=-1).amax(dim=1)>1
            order=[FOOT_NAMES.index(sensor.body_names[i]) for i in entity.body_ids]
            expected_reward=(expected[:,order,:2].norm(dim=-1)*contact).sum(dim=1)
            torch.testing.assert_close(feet_slide_sole(env,entity),expected_reward,atol=2e-6,rtol=1e-5)
            assert torch.isfinite(actual).all()
            if step<30:
                action=.1*torch.sin(torch.arange(12,device=env.device)+step*.2).repeat(2,1)
                env.step(action)
        assert robot.num_joints==12
        print(f'SOLE_SLIDE_SIM_PASS checks=31 envs=2 joints=12 max_velocity_error_m_s={max_error:.9g} sensor_order_reversed=True actual_USD=True',flush=True)
    finally:env.close()
finally:app.close()
