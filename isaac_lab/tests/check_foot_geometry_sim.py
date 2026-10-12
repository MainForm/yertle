"""Bounded read-only model/sole diagnostic; no policy learning or hardware IO."""
import argparse
from pathlib import Path
import sys
import torch
from isaaclab.app import AppLauncher
parser=argparse.ArgumentParser();parser.add_argument('--steps',type=int,default=10)
AppLauncher.add_app_launcher_args(parser);args=parser.parse_args();app=AppLauncher(args).app
try:
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
    import gymnasium as gym
    import isaac_lab
    from isaac_lab.flat_env_cfg import YertleFlatEnvCfg
    from isaac_lab.signed_joint_reference import configure_signed_joint_reference
    from isaac_lab.foot_geometry import load_sole_points,foot_sole_heights
    from isaaclab.utils.math import quat_apply
    cfg=YertleFlatEnvCfg();cfg.scene.num_envs=1
    cfg.events.reset_base.params['pose_range']={}
    cfg.events.reset_robot_joints.params['position_range']=(1,1)
    cfg.events.base_external_force_torque=None
    cfg.observations.policy.enable_corruption=False
    configure_signed_joint_reference(cfg)
    points=load_sole_points(cfg.scene.robot.spawn.usd_path)
    print('SOLE_VERTEX_COUNTS '+str([len(p) for p in points]),flush=True)
    env=gym.make('Isaac-Velocity-Flat-Yertle-v0',cfg=cfg).unwrapped
    try:
        env.reset();robot=env.scene['robot']
        measured=foot_sole_heights(env)
        ids,padded=env._yertle_sole_geometry
        rotations=robot.data.body_quat_w[:,ids,None,:].expand(-1,-1,padded.shape[1],-1)
        expanded=padded[None].expand(rotations.shape[0],-1,-1,-1)
        full=quat_apply(rotations.reshape(-1,4),expanded.reshape(-1,3)).reshape_as(expanded)
        expected=(full[...,2]+robot.data.body_pos_w[:,ids,2,None]).amin(dim=-1)
        torch.testing.assert_close(measured,expected,rtol=0,atol=1e-6)
        origins=robot.data.body_pos_w[:,ids,2]
        assert (measured<origins-.02).all(),(measured,origins)
        print('SOLE_INITIAL heights_m='+str(measured[0].tolist())+' link_origin_heights_m='+str(origins[0].tolist()),flush=True)
        for _ in range(args.steps):env.step(torch.zeros(1,12,device=env.device))
        assert torch.isfinite(foot_sole_heights(env)).all()
        print('FOOT_GEOMETRY_SIM_PASS actual_usd_rubber_surface quaternion_transform independent_check steps='+str(args.steps),flush=True)
    finally:env.close()
except Exception:
    # Kit shutdown can otherwise hide an exception behind an exit code of 0.
    import traceback
    traceback.print_exc()
    raise
finally:app.close()
