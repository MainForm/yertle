"""play.py setup + real action replay. No serial/hardware or training writes.

Native implicit actuators test simulated ability to follow the SAME real goals.
This does not identify a physical PWM/current/torque plant or prove hardware PID
is the unique cause. The real trial was hand-supported then grounded, whereas
simulation has a ground plane throughout; the landing instant is unrecorded.
"""
import argparse,json,sys,hashlib,math
from pathlib import Path
import numpy as np
import torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab.app import AppLauncher
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--physics-dt',type=float,default=.005)
p.add_argument('--data-dir',type=Path,default=HERE)
p.add_argument('--steps',type=int,default=0,help='0: all recorded samples; positive: explicit short verification.')
p.add_argument('--label',default='native');AppLauncher.add_app_launcher_args(p)
args=p.parse_args();app=AppLauncher(args).app
HERE=args.data_dir.resolve()
sys.path.insert(0,'/workspace/yertle')
import gymnasium as gym
import isaac_lab
from isaac_lab.flat_env_cfg import YertleFlatEnvCfg_PLAY
from isaac_lab.signed_joint_reference import configure_signed_joint_reference
from isaac_lab.play_start_pose import configure_start_pose
from isaac_lab.play_controls import apply_command,configure_play_terminations
from isaac_lab.rsl_rl_ppo_cfg import YertleFlatPPORunnerCfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
data=json.loads((HERE/'real_replay.json').read_text());m=json.loads((HERE/'mapping.json').read_text())
B=np.array(m['B_q_per_raw']);J=np.linalg.inv(B);qseat=np.array(m['q_at_raw2048']);names=m['joint_names']
ref=np.radians([0]*4+[34]*4+[-85]*4)
rows=data['actions'];replay=np.array([x['action12'] for x in rows]);N=len(rows)
assert replay.shape==(628,12)
assert args.steps>=0
if args.steps:N=min(N,args.steps)
cfg=YertleFlatEnvCfg_PLAY();cfg.scene.num_envs=5;cfg.scene.env_spacing=2.;cfg.seed=0;cfg.sim.device=args.device
cfg.sim.dt=args.physics_dt;cfg.decimation=round(.02/args.physics_dt)
assert abs(cfg.sim.dt*cfg.decimation-.02)<1e-9
cfg.trot_frequency_hz=1.7;cfg.trot_duty_factor=.6
act=cfg.scene.robot.actuators['legs'];act.effort_limit_sim=1.024;act.velocity_limit_sim=2.3
act.stiffness=25.;act.damping=.5
configure_signed_joint_reference(cfg);configure_start_pose(cfg,'default',None);configure_play_terminations(cfg)
cfg.terminations.time_out=None;cfg.observations.policy.enable_corruption=False
for k in ['physics_material','add_base_mass','push_robot','base_external_force_torque']:
 if hasattr(cfg.events,k):setattr(cfg.events,k,None)
cfg.events.reset_base.params['pose_range']={k:(0.,0.) for k in ['x','y','yaw']}
cfg.events.reset_base.params['velocity_range']={k:(0.,0.) for k in ['x','y','z','roll','pitch','yaw']}
c=cfg.commands.base_velocity;c.heading_command=False;c.rel_heading_envs=0.;c.rel_standing_envs=0.
c.resampling_time_range=(1.e6,1.e6);c.debug_vis=False
env=gym.make('Isaac-Velocity-Flat-Yertle-Play-v0',cfg=cfg).unwrapped
wrapped=RslRlVecEnvWrapper(env);ag=YertleFlatPPORunnerCfg();ag.device=args.device
runner=OnPolicyRunner(wrapped,ag.to_dict(),log_dir=None,device=args.device)
runner.load(str(HERE/'model_6999.pt'));actor=runner.get_inference_policy(device=args.device)
with torch.inference_mode():
 real_inference=actor({'policy':torch.tensor([r['obs55'] for r in rows],dtype=torch.float32,device=args.device)}).cpu().numpy()
actor_log_difference=float(abs(real_inference-replay).max())
assert actor_log_difference<2e-5
robot=env.scene['robot'];order=[robot.joint_names.index(n) for n in names];inverse=np.argsort(order)
sensor=env.scene.sensors['contact_forces'];baseid=sensor.body_names.index('base_link') if 'base_link' in sensor.body_names else sensor.body_names.index('base')
labels=['live_policy_truth_velocity','live_policy_zero_linear_velocity','real_actions_native_final_goal','real_actions_native_profile_reference','live_policy_zero_velocity_native_profile']
def raw(q):return (q-qseat)@J.T+2048
trace={k:[] for k in ['q','goal_q','raw_measured','raw_final_goal','raw_reference','action','base_pos','base_quat','base_velocity','applied_torque','base_contact_force']}
# Profile computation uses the CURRENT production C controller, compiled locally
# into a bridge. Native PD follows its generated reference; no PWM plant guessed.
import ctypes as ct
lib=ct.CDLL(str(HERE/'libpid.so'))
class State(ct.Structure):
 _fields_=[(s,ct.c_float) for s in ['reference','reference_velocity','previous_position','velocity','integral','output','trace_error','trace_p','trace_d']]+[('trace_flags',ct.c_uint8),('initialized',ct.c_bool)]
class Config(ct.Structure):
 _fields_=[(s,ct.c_float) for s in ['kp','ki','kd','integral_limit','duty_limit']]+[('polarity',ct.c_int8)]
lib.pid_defaults.argtypes=[ct.POINTER(Config)];pc=Config();lib.pid_defaults(ct.byref(pc))
assert (pc.kp,pc.ki,pc.integral_limit,pc.duty_limit)==(5.,3.,100.,800.)
assert ct.sizeof(State)==lib.pid_state_size()
states=(State*24)();configs=(Config*24)(*[pc for _ in range(24)])
lib.pid_batch.argtypes=[ct.POINTER(State),ct.POINTER(Config)]+[ct.c_void_p]*6+[ct.c_uint]
lib.api_raw.argtypes=[ct.POINTER(ct.c_double),ct.POINTER(ct.c_uint16)]
lib.api_action_angles.argtypes=[ct.POINTER(ct.c_double),ct.POINTER(ct.c_double)]
mapping_checks=0;profile_guard={3:[],4:[]};previous=np.repeat(np.floor(raw(ref)+.5)[None,:],2,axis=0).astype(int)
try:
 with torch.inference_mode():
  env.reset()
  # Two replay cases start at the first real measured joint pose. Root/support
  # remains the explicit simulation reset pose; physical support is unknown.
  q=robot.data.joint_pos.clone();qd=robot.data.joint_vel.clone()
  first=ref+np.array(rows[0]['obs55'][12:24]);q[2:4,order]=torch.tensor(first,dtype=q.dtype,device=env.device)
  qd[2:4]=0.;robot.write_joint_state_to_sim(q,qd);env.scene.update(0.)
  for step in range(N):
   apply_command(env,torch.zeros((5,3),device=env.device))
   obs=wrapped.get_observations();obs=obs[0] if isinstance(obs,tuple) else obs
   zero=obs.clone()
   assert zero['policy'].shape==(5,55)
   zero['policy'][[1,4],:3]=0.
   action=actor(zero);action[2:4]=torch.tensor(replay[step],device=env.device)
   final_actions=action.cpu().numpy().copy()
   target=ref+action.cpu().numpy()*.25
   realgoal=np.ascontiguousarray(np.floor(raw(target[[3,4]])+.5).astype(np.uint16))
   measured=np.ascontiguousarray(np.floor(raw(robot.data.joint_pos[:,order].cpu().numpy())[[3,4]]+.5).astype(np.int16))
   # Same host speed formula: ceil(abs(new-old)/20ms), clamp200..3000.
   speed=np.clip(np.ceil(abs(realgoal.astype(int)-previous)/.02),200,3000).astype(np.uint16)
   packet=data['protocol'][step]['payload_raw'];assert len(packet)==60
   wiregoal=np.array([packet[i*5]+256*packet[i*5+1] for i in range(12)])
   np.testing.assert_array_equal(realgoal[0],wiregoal)
   speed[0]=[packet[i*5+2]+256*packet[i*5+3] for i in range(12)]
   acc=np.full((2,12),100,np.uint8);duty=np.zeros((2,12),np.int16);valid=np.zeros((2,12),np.uint8)
   lib.pid_batch(states,configs,realgoal.ctypes.data,speed.ctypes.data,acc.ctypes.data,measured.ctypes.data,duty.ctypes.data,valid.ctypes.data,24)
   for ix,e in enumerate([3,4]):
    if not valid[ix].all():profile_guard[e].append(step)
    profile=np.array([states[ix*12+j].reference for j in range(12)])
    target[e]=qseat+B@(profile-2048)
    action[e]=torch.tensor((target[e]-ref)/.25,device=env.device)
   previous=realgoal.astype(int)
   if step%50==0:
    print('TRACK_PROGRESS',step,flush=True)
    # Independent API checks: absolute angle -> raw AND action -> angle.
    for e in range(3):
     ai=(ct.c_double*12)(*action[e].cpu().numpy());qo=(ct.c_double*12)();lib.api_action_angles(ai,qo)
     np.testing.assert_allclose(qo,target[e],rtol=0,atol=2e-6)
     qi=(ct.c_double*12)(*target[e]);ro=(ct.c_uint16*12)();ok=lib.api_raw(qi,ro)
     expected=np.floor(raw(target[e])+.5)
     assert bool(ok)==bool(((expected>=0)&(expected<=4095)).all())
     if ok:np.testing.assert_array_equal(ro,expected)
     mapping_checks+=1
   # Same standard env.step path as play.py. Record post-step positions
   # against the target that was held for this20ms interval.
   env.step(action)
   actual=robot.data.joint_pos[:,order].cpu().numpy()
   processed=env.action_manager.get_term('joint_pos').processed_actions[:,order].cpu().numpy()
   np.testing.assert_allclose(processed,target,rtol=0,atol=2e-6)
   final=raw(ref+final_actions*.25)
   rawref=raw(processed)
   for k,v in dict(q=actual,goal_q=processed,raw_measured=raw(actual),raw_final_goal=final,raw_reference=rawref,
     action=action.cpu().numpy(),base_pos=robot.data.root_pos_w.cpu().numpy(),base_quat=robot.data.root_quat_w.cpu().numpy(),
     base_velocity=robot.data.root_lin_vel_b.cpu().numpy(),applied_torque=robot.data.applied_torque[:,order].cpu().numpy(),
     base_contact_force=sensor.data.net_forces_w[:,baseid,:].cpu().numpy()).items():trace[k].append(v.copy())
 arrays={k:np.array(v) for k,v in trace.items()};np.savez_compressed(HERE/f'{args.label}_trace.npz',**arrays)
 result=dict(labels=labels,physics_dt=cfg.sim.dt,control_dt=.02,steps=N,source_sha256=data['source_sha256'],mapping_checks=mapping_checks,actor_log_max_abs_difference=actor_log_difference,
  checkpoint_sha256=hashlib.sha256((HERE/'model_6999.pt').read_bytes()).hexdigest(),pid=dict(kp=pc.kp,ki=pc.ki,kd=pc.kd,integral_cap=pc.integral_limit),
  actuator=dict(effort_limit_sim=1.024,velocity_limit_sim=2.3,stiffness=25,damping=.5),profile_guard_steps=profile_guard,cases=[])
 for e,label in enumerate(labels):
   # Beyond a controller guard, continuing native reference replay is a
   # counterfactual. Retain raw samples but exclude them from valid-run metrics.
   stop=profile_guard[e][0] if e in profile_guard and profile_guard[e] else N
   err=abs(arrays['raw_final_goal'][:stop,e]-arrays['raw_measured'][:stop,e]);internal=abs(arrays['raw_reference'][:stop,e]-arrays['raw_measured'][:stop,e])
   contact=np.linalg.norm(arrays['base_contact_force'][:stop,e],axis=1)>1.
   result['cases'].append(dict(label=label,valid_steps=stop,first_pid_guard_s=stop*.02 if stop<N else None,mean_final_error_raw=float(err.mean()),max_final_error_raw=float(err.max()),
   mean_reference_error_raw=float(internal.mean()),max_reference_error_raw=float(internal.max()),
   base_contact_first_s=float(np.where(contact)[0][0]*.02) if contact.any() else None,
   per_motor_mean_error_raw=err.mean(axis=0).tolist(),per_motor_max_error_raw=err.max(axis=0).tolist()))
 (HERE/f'{args.label}_summary.json').write_text(json.dumps(result,indent=2));print('TRACK_RESULT',json.dumps(result),flush=True)
finally:
 env.close();app.close()
