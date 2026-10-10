"""Verify geometry equivalence, zero-action reference, and bounded zero-action hold."""
import argparse
import math
import sys
from pathlib import Path
import torch
from isaaclab.app import AppLauncher
parser=argparse.ArgumentParser();AppLauncher.add_app_launcher_args(parser);args=parser.parse_args()
app=AppLauncher(args).app
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import gymnasium as gym
import isaac_lab
from isaac_lab.flat_env_cfg import YertleFlatEnvCfg
from isaac_lab.signed_joint_reference import configure_signed_joint_reference
from isaac_lab.motor_operating_range import MotorRawTransform,OperatingRangeCost
from isaaclab.envs.mdp import joint_pos_rel
from isaaclab_tasks.manager_based.locomotion.velocity.mdp import stand_still_joint_deviation_l1

cfg=YertleFlatEnvCfg();cfg.scene.num_envs=2;cfg.scene.env_spacing=2.5
cfg.events.reset_robot_joints.params['position_range']=(1.,1.)
cfg.events.reset_base.params['pose_range']={};cfg.observations.policy.enable_corruption=False
cfg.events.base_external_force_torque=None
original_path=cfg.scene.robot.spawn.usd_path
configure_signed_joint_reference(cfg)
from pxr import Gf, Usd, UsdPhysics
original=Usd.Stage.Open(original_path);converted=Usd.Stage.Open(cfg.scene.robot.spawn.usd_path)
verified=0
for prim in original.Traverse():
    if not prim.IsA(UsdPhysics.RevoluteJoint) or not prim.GetName().endswith(('_thigh','_shin')):continue
    old=UsdPhysics.RevoluteJoint(prim);new=UsdPhysics.RevoluteJoint(converted.GetPrimAtPath(prim.GetPath()))
    assert new.GetLowerLimitAttr().Get()==-old.GetUpperLimitAttr().Get()
    assert new.GetUpperLimitAttr().Get()==-old.GetLowerLimitAttr().Get()
    assert new.GetLocalPos0Attr().Get()==old.GetLocalPos0Attr().Get()
    assert new.GetLocalPos1Attr().Get()==old.GetLocalPos1Attr().Get()
    axis={'X':Gf.Vec3f(1,0,0),'Y':Gf.Vec3f(0,1,0),'Z':Gf.Vec3f(0,0,1)}[old.GetAxisAttr().Get()]
    for q in [-1.2,0.,.7]:
        positive=Gf.Quatf(math.cos(q/2),axis*math.sin(q/2));negative=positive.GetInverse()
        a=new.GetLocalRot0Attr().Get()*positive*new.GetLocalRot1Attr().Get().GetInverse()
        b=old.GetLocalRot0Attr().Get()*negative*old.GetLocalRot1Attr().Get().GetInverse()
        difference=min((a-b).GetImaginary().GetLength()+abs((a-b).GetReal()),(a+b).GetImaginary().GetLength()+abs((a+b).GetReal()))
        assert difference<1e-5,(prim.GetName(),q,difference)
    verified+=1
assert verified==8
print('SIGNED_USD_GEOMETRY_PASS 8 reversed joints; equivalent relative rotations at 3 angles; reversed limits; unchanged joint origins',flush=True)
env=gym.make('Isaac-Velocity-Flat-Yertle-v0',cfg=cfg).unwrapped
try:
    env.reset();robot=env.scene['robot'];ref=robot.data.default_joint_pos.clone()
    for name,angle in [('thigh',34.4),('shin',-51.6),('shoulder',0)]:
        ids=[i for i,n in enumerate(robot.joint_names) if n.endswith('_'+name)]
        torch.testing.assert_close(ref[:,ids],torch.full_like(ref[:,ids],math.radians(angle)),rtol=0,atol=1e-6)
    assert ((ref>=robot.data.joint_pos_limits[:,:,0])&(ref<=robot.data.joint_pos_limits[:,:,1])).all()
    torch.testing.assert_close(joint_pos_rel(env),torch.zeros_like(ref),rtol=0,atol=1e-5)
    env.command_manager.get_command('base_velocity').zero_()
    torch.testing.assert_close(stand_still_joint_deviation_l1(env,'base_velocity'),torch.zeros(2,device=env.device),rtol=0,atol=1e-5)
    action=env.action_manager.get_term('joint_pos');action.process_actions(torch.zeros(2,12,device=env.device))
    torch.testing.assert_close(action.processed_actions,ref[:,action._joint_ids],rtol=0,atol=1e-6)
    initial=robot.data.body_pos_w.clone();foot_ids=[i for i,n in enumerate(robot.body_names) if n.endswith('_shin')]
    print('SIGNED_INITIAL feet_z='+str(initial[0,foot_ids,2].tolist()),flush=True)
    maximum_tilt=0.;resets=0
    for _ in range(150):
        _,_,terminated,truncated,_=env.step(torch.zeros(2,12,device=env.device))
        resets+=int((terminated|truncated).sum())
        g=robot.data.projected_gravity_b
        maximum_tilt=max(maximum_tilt,float(torch.acos((-g[:,2]).clamp(-1,1)).max())*180/math.pi)
    transform=MotorRawTransform(robot.joint_names,device=env.device);cost=OperatingRangeCost(device=env.device)
    excess=cost.excess(transform(ref))[0].tolist()
    assert maximum_tilt<45 and resets==0,(maximum_tilt,resets)
    print(f'SIGNED_REFERENCE_SIM_PASS zero_action_targets=reference relative_observation=zero standstill_deviation=zero limits_contain_reference hold=3s max_tilt_deg={maximum_tilt:.4f} resets={resets} measured_range_excess_raw={excess}',flush=True)
finally:env.close();app.close()
