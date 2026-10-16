"""Read-only real PhysX contact layout/classification verification."""
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
    from isaac_lab.ground_contact import BODY_NAMES, inverse_rotate, surface_distance
    cfg=YertleFlatEnvCfg_PLAY();cfg.scene.num_envs=2;cfg.seed=42
    configure_signed_joint_reference(cfg);configure_start_pose(cfg,'default',None)
    cfg.commands.base_velocity.debug_vis=False
    env=gym.make('Isaac-Velocity-Flat-Yertle-Play-v0',cfg=cfg).unwrapped
    try:
        env.reset()
        for _ in range(100):env.step(torch.zeros(2,12,device=env.device))
        contacts=env._yertle_ground_contacts
        counts=[]
        for name,view in zip(BODY_NAMES,contacts.views):
            _,p,_,gap,c,start=view.get_contact_data(dt=env.physics_dt)
            counts.append((name,c.sum().item()))
            if name=='lf_shin':
                start_id=int(start[0].reshape(-1)[0]);number=int(c[0].reshape(-1)[0])
                points=p.reshape(-1,3)[start_id:start_id+number]
                body_id=env.scene['robot'].body_names.index(name)
                data=env.scene['robot'].data
                local=inverse_rotate(data.body_quat_w[0,body_id].expand(number,-1),points-data.body_pos_w[0,body_id])
                distances=surface_distance(local,contacts.triangles[name])
                assert (distances<=.002).any() and (distances>.1).any(),distances
                print('LF_POINT_AUDIT local='+str(local.cpu().tolist())+' rubber_distance_m='+str(distances.cpu().tolist())+' gaps='+str(gap.reshape(-1)[start_id:start_id+number].cpu().tolist()),flush=True)
        assert sum(c for name,c in counts if name.endswith('_shin'))>0,counts
        flags=contacts.read(env,.001,.005)
        print('GROUND_CONTACT_COUNTS '+str(counts),flush=True)
        print('GROUND_CONTACT_FLAGS '+str(flags.cpu().tolist()),flush=True)
        assert torch.isfinite(flags.float()).all()
        index=BODY_NAMES.index('lf_shin')
        assert flags[:,index].all(),flags
        before_shin_cost=0.
        after_shin_cost=-.5*flags[:,[i for i,n in enumerate(BODY_NAMES) if n.endswith('_shin')]].float().sum(-1)
        print('CONTACT_BEFORE_AFTER shin_cost_before='+str(before_shin_cost)+' shin_cost_after='+str(after_shin_cost.cpu().tolist()),flush=True)
        assert env.action_manager.total_action_dim==12
        # Directly verify an observed contact outside the sole is classified
        # independently from force; CPU tests cover mixed contact buffers.
        print('GROUND_CONTACT_SIM_PASS envs=2 bodies=13 raw_points=True force_threshold=none observed_shin_contacts=True actions=12',flush=True)
    finally:env.close()
except BaseException:
    import traceback;traceback.print_exc();print('GROUND_CONTACT_SIM_FAIL',flush=True);raise
finally:app.close()
