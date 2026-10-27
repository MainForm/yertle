"""Production-C PID / tracking-error / base-contact experiments.

Default full-pid runs the firmware raw/PWM controller and explicitly ASSUMED
PWM motor plants, with the previous signed individual-joint torque ablation.
The separate implicit-pd reference has I=0 and different gain units. Neither
plant is identified physical hardware. No hardware or training writes.
"""
import argparse,ctypes as ct,hashlib,json,sys,traceback
from pathlib import Path
from pid_unit_conversion import MOTOR_RESPONSE_GAIN, MODEL_STALL_TORQUE_NM, MODEL_FREE_SPEED_RAD_S, convert_raw_pid, AngleTorquePID
import numpy as np
import torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab.app import AppLauncher

p=argparse.ArgumentParser()
p.add_argument('--data-dir',type=Path,required=True)
p.add_argument('--out-dir',type=Path,required=True)
p.add_argument('--backend',choices=['implicit-pd','full-pid','converted-pid','inverse-compare'],default='full-pid')
p.add_argument('--kp',type=float,nargs='+')
p.add_argument('--ki',type=float,nargs='+',default=[3.])
p.add_argument('--kd',type=float,nargs='+')
p.add_argument('--motor-gain',type=float,nargs='+',choices=[MOTOR_RESPONSE_GAIN],default=[MOTOR_RESPONSE_GAIN],
               help='Fixed at1.0; response scaling is not a PID tuning variable.')
p.add_argument('--plant',choices=['ideal','dc','both'],default='both')
p.add_argument('--initial-integral',choices=['zero','log'],default='zero')
p.add_argument('--sync-start',action='store_true',help='Copy env0 standing state after warmup, adjusted for environment origins.')
p.add_argument('--profile',choices=['off','on','both'],default='both')
p.add_argument('--repeats',type=int,default=1)
p.add_argument('--seconds',type=float,default=12.56)
p.add_argument('--physics-dt',type=float,default=.005)
p.add_argument('--label',default='sweep')
p.add_argument('--command',type=float,nargs=3,default=[0.,0.,0.],metavar=('VX','VY','YAW'),
               help='Policy command only; controller and physical parameters are unchanged.')
AppLauncher.add_app_launcher_args(p)
args=p.parse_args()
source_bytes=Path(__file__).read_bytes()
full_backend=args.backend!='implicit-pd'
if args.kp is None:args.kp=[5.] if full_backend else [25,20,15,10,7.5,5,3,2]
if args.kd is None:args.kd=[.1] if full_backend else [.1,.5,1.]
assert args.repeats>0 and args.seconds>=.02 and args.physics_dt>0
assert all(k>0 for k in args.kp) and all(k>=0 for k in args.ki+args.kd) and all(k>0 for k in args.motor_gain)
assert abs(round(.02/args.physics_dt)*args.physics_dt-.02)<1e-9
app=AppLauncher(args).app
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
import gymnasium as gym
import isaac_lab
from isaac_lab.flat_env_cfg import YertleFlatEnvCfg_PLAY
from isaac_lab.signed_joint_reference import configure_signed_joint_reference
from isaac_lab.play_start_pose import configure_start_pose
from isaac_lab.play_controls import apply_command
from isaac_lab.train_terminations import configure_base_contact_termination
from isaac_lab.rsl_rl_ppo_cfg import YertleFlatPPORunnerCfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

out=args.out_dir.resolve();out.mkdir(parents=True,exist_ok=True)
m=json.loads((args.data_dir/'mapping.json').read_text())
B=np.array(m['B_q_per_raw']);J=np.linalg.inv(B);qseat=np.array(m['q_at_raw2048']);names=m['joint_names']
ref=np.radians([0]*4+[34]*4+[-85]*4)
def raw(q):return (q-qseat)@J.T+2048
profiles=[False,True] if args.profile=='both' else [args.profile=='on']
if full_backend:
 plants=['ideal','dc'] if args.plant=='both' else [args.plant]
 controllers=['raw_pwm','motor_angle_torque'] if args.backend=='inverse-compare' else ['motor_angle_torque'] if args.backend=='converted-pid' else ['raw_pwm']
 cases=[dict(kp=kp,ki=ki,kd=kd,profile=True,motor_gain=gain,plant=plant,repeat=r,pid_units=controller,
             converted_gains=convert_raw_pid(kp,ki,kd))
        for kp in args.kp for ki in args.ki for kd in args.kd for gain in args.motor_gain for plant in plants for controller in controllers for r in range(args.repeats)]
else:
 cases=[dict(kp=kp,ki=0.,kd=kd,profile=profile,repeat=r) for kp in args.kp for kd in args.kd for profile in profiles for r in range(args.repeats)]
(out/f'{args.label}_config.json').write_text(json.dumps(dict(arguments=vars(args)|{'data_dir':str(args.data_dir),'out_dir':str(args.out_dir)},cases=cases),indent=2))
(out/f'{args.label}_script.py').write_bytes(source_bytes)
(out/'pid_unit_conversion.py').write_bytes(Path(__file__).with_name('pid_unit_conversion.py').read_bytes())
n=len(cases);steps=round(args.seconds/.02)
c=YertleFlatEnvCfg_PLAY();c.scene.num_envs=n;c.scene.env_spacing=3.;c.seed=0;c.sim.device=args.device
c.sim.dt=args.physics_dt;c.decimation=round(.02/args.physics_dt)
c.trot_frequency_hz=1.7;c.trot_duty_factor=.6
act=c.scene.robot.actuators['legs'];act.effort_limit_sim=1.024;act.velocity_limit_sim=2.3;act.stiffness=25.;act.damping=.5
configure_signed_joint_reference(c);configure_start_pose(c,'default',None);configure_base_contact_termination(c)
c.terminations.time_out=None;c.observations.policy.enable_corruption=False
for k in ['physics_material','add_base_mass','push_robot','base_external_force_torque']:
 if hasattr(c.events,k):setattr(c.events,k,None)
c.events.reset_base.params['pose_range']={k:(0.,0.) for k in ['x','y','yaw']}
c.events.reset_base.params['velocity_range']={k:(0.,0.) for k in ['x','y','z','roll','pitch','yaw']}
cmd=c.commands.base_velocity;cmd.heading_command=False;cmd.rel_heading_envs=0.;cmd.rel_standing_envs=0.
cmd.resampling_time_range=(1.e6,1.e6);cmd.debug_vis=False
# Evaluate the ORIGINAL termination predicate, latch first hit, suppress only
# automatic reset. Every statistic is censored at that first termination hit.
original_contact=c.terminations.base_contact.func
observed=None
def observe_contact(env,sensor_cfg,threshold):
 global observed
 observed=original_contact(env,sensor_cfg=sensor_cfg,threshold=threshold).clone()
 return torch.zeros_like(observed)
c.terminations.base_contact.func=observe_contact
env=gym.make('Isaac-Velocity-Flat-Yertle-Play-v0',cfg=c).unwrapped
w=RslRlVecEnvWrapper(env);ag=YertleFlatPPORunnerCfg();ag.device=args.device
runner=OnPolicyRunner(w,ag.to_dict(),log_dir=None,device=args.device)
runner.load(str(args.data_dir/'model_6999.pt'));actor=runner.get_inference_policy(device=args.device)
robot=env.scene['robot'];order=[robot.joint_names.index(k) for k in names]
sensor=env.scene.sensors['contact_forces'];baseid=sensor.body_names.index('base_link')
kp=torch.tensor([k['kp'] for k in cases],device=env.device)[:,None].repeat(1,12)
kd=torch.tensor([k['kd'] for k in cases],device=env.device)[:,None].repeat(1,12)
profile_ids=np.array([i for i,k in enumerate(cases) if k['profile']],dtype=int)
lib=ct.CDLL(str(args.data_dir/'libpid.so'))
class State(ct.Structure):
 _fields_=[(s,ct.c_float) for s in ['reference','reference_velocity','previous_position','velocity','integral','output','trace_error','trace_p','trace_d']]+[('trace_flags',ct.c_uint8),('initialized',ct.c_bool)]
class Config(ct.Structure):
 _fields_=[(s,ct.c_float) for s in ['kp','ki','kd','integral_limit','duty_limit']]+[('polarity',ct.c_int8)]
lib.pid_defaults.argtypes=[ct.POINTER(Config)];fc=Config();lib.pid_defaults(ct.byref(fc))
assert (fc.kp,fc.ki,fc.integral_limit,fc.duty_limit)==(5.,3.,100.,800.)
assert ct.sizeof(State)==lib.pid_state_size()
count=n*12;states=(State*count)();configs=(Config*count)(*[fc for _ in range(count)])
lib.position_pid_motor_polarity.argtypes=[ct.c_uint8];lib.position_pid_motor_polarity.restype=ct.c_int8
polarity=np.array([lib.position_pid_motor_polarity(i+1) for i in range(12)])
assert (polarity==-1).all()
lib.api_motor_mapping.argtypes=[ct.POINTER(ct.c_uint8),ct.POINTER(ct.c_int8)]
mi=(ct.c_uint8*12)();ms=(ct.c_int8*12)();lib.api_motor_mapping(mi,ms)
direct=np.zeros((12,12));direct[np.arange(12),np.array(mi)]=np.array(ms)
assert (abs(direct).sum(axis=0)==1).all() and (abs(direct).sum(axis=1)==1).all()
for e,k in enumerate(cases):
 if full_backend:
  for j in range(12):configs[e*12+j]=Config(k['kp'],k['ki'],k['kd'],100,800,int(polarity[j]))
lib.pid_batch.argtypes=[ct.POINTER(State),ct.POINTER(Config)]+[ct.c_void_p]*6+[ct.c_uint]
lib.api_action_angles.argtypes=[ct.POINTER(ct.c_double),ct.POINTER(ct.c_double)]
lib.api_raw.argtypes=[ct.POINTER(ct.c_double),ct.POINTER(ct.c_uint16)]
trace={k:[] for k in ['raw_before','raw_goal','raw_reference','raw_after','base_force_N','base_force_history_N','base_height_m','base_contact','pid_guard','action','pwm_raw','pid_terms','root_state_w','raw_C_pwm','angle_pid_Nm','angle_pid_terms','shadow_pwm_difference']}
first_contact=np.full(n,-1,int);first_guard=np.full(n,-1,int);mapping_checks=0
previous=np.repeat(np.floor(raw(ref)+.5)[None,:],n,axis=0)
try:
 with torch.inference_mode():
  env.reset()
  if full_backend:
   # Start from a supported standing equilibrium. Physics-only warmup keeps
   # policy episode/phase/action history at zero; no initial free-drop artifact.
   robot.set_joint_position_target(robot.data.default_joint_pos)
   for _ in range(round(1./c.sim.dt)):
    env.scene.write_data_to_sim();env.sim.step(render=False);env.scene.update(c.sim.dt)
   if args.sync_start:
    root=robot.data.root_state_w[:1].repeat(n,1)
    root[:,:3]+=env.scene.env_origins-env.scene.env_origins[:1]
    q=robot.data.joint_pos[:1].repeat(n,1);v=robot.data.joint_vel[:1].repeat(n,1)
    robot.write_root_state_to_sim(root);robot.write_joint_state_to_sim(q,v);env.scene.update(0.)
   kp.zero_();kd.zero_()
  robot.write_joint_stiffness_to_sim(kp);robot.write_joint_damping_to_sim(kd)
  # Update the implicit actuator's effort estimate too; actual physics is set
  # by the writes above and verified through the PhysX readback below.
  robot.actuators['legs'].stiffness.copy_(kp);robot.actuators['legs'].damping.copy_(kd)
  torch.testing.assert_close(robot.root_physx_view.get_dof_stiffnesses().cpu(),kp.cpu(),rtol=0,atol=1e-6)
  torch.testing.assert_close(robot.root_physx_view.get_dof_dampings().cpu(),kd.cpu(),rtol=0,atol=1e-6)
  if full_backend and args.initial_integral=='log':
   real=json.loads((args.data_dir/'real_replay.json').read_text())
   initial_pid=real['pid'][0]
   assert initial_pid['target_sequence']<real['protocol'][0]['sequence']
   initial_i=[m['i_raw'] for m in initial_pid['motors']]
   assert max(abs(v) for v in initial_i)<=100
   lib.position_pid_reset.argtypes=[ct.POINTER(State),ct.c_float]
   initial_raw=np.floor(raw(robot.data.joint_pos[:,order].cpu().numpy())+.5)
   for e in range(n):
    for j in range(12):
     lib.position_pid_reset(ct.byref(states[e*12+j]),float(initial_raw[e,j]))
     states[e*12+j].integral=initial_i[j]
  initial_measured=np.floor(raw(robot.data.joint_pos[:,order].cpu().numpy())+.5)
  initial_i=np.array([s.integral for s in states]).reshape(n,12)
  angular=AngleTorquePID([(k['kp'],k['ki'],k['kd']) for k in cases],initial_measured,initial_i)
  angle_cases=np.array([k.get('pid_units')=='motor_angle_torque' for k in cases])
  for step in range(steps):
   if step%100==0:print('ERROR_CONTACT_PROGRESS',step,'/',steps,flush=True)
   apply_command(env,torch.tensor(args.command,device=env.device).repeat(n,1))
   obs=w.get_observations();obs=obs[0] if isinstance(obs,tuple) else obs
   obs=obs.clone();obs['policy'][:,:3]=0.
   if step==0:
    initial_observation=obs['policy'].cpu().numpy().copy()
    initial_spread=float(abs(initial_observation-initial_observation[:1]).max())
    # Physics-only standing warmup can differ slightly across GPU clones;
    # preserve this measured spread rather than claim bit-identical starts.
    print('INITIAL_OBSERVATION_MAX_SPREAD',initial_spread,flush=True)
   actions=actor(obs);actor_actions=actions.cpu().numpy().copy()
   target=ref+actor_actions*.25
   before=raw(robot.data.joint_pos[:,order].cpu().numpy())
   goal=np.floor(raw(target)+.5)
   bad=(~np.isfinite(goal).all(axis=1))|((goal<0)|(goal>4095)).any(axis=1)
   measured=np.floor(before+.5);bad|=((measured<0)|(measured>4095)).any(axis=1)
   safe_goal=np.ascontiguousarray(np.where(bad[:,None],2048,goal),dtype=np.uint16)
   safe_meas=np.ascontiguousarray(np.where(bad[:,None],2048,measured),dtype=np.int16)
   speed=np.ascontiguousarray(np.clip(np.ceil(abs(goal-previous)/.02),200,3000),dtype=np.uint16)
   acc=np.full((n,12),100,np.uint8);duty=np.zeros((n,12),np.int16);valid=np.zeros((n,12),np.uint8)
   lib.pid_batch(states,configs,safe_goal.ctypes.data,speed.ctypes.data,acc.ctypes.data,safe_meas.ctypes.data,duty.ctypes.data,valid.ctypes.data,count)
   raw_C_pwm=duty.copy()
   ref_raw=np.array([s.reference for s in states]).reshape(n,12)
   angle_Nm=angular.step(ref_raw,safe_meas,.02)
   # Preserve the existing PWM integer resolution in the motor model. The
   # converted controller itself operates on rad/Nm; this is output resolution.
   normalized=angle_Nm/angular.k
   angle_pwm=(np.sign(normalized*polarity)*np.floor(abs(normalized*polarity)+.5)).astype(np.int16)
   shadow_difference=angle_pwm.astype(int)-raw_C_pwm.astype(int)
   duty[angle_cases]=angle_pwm[angle_cases]
   guard=bad.copy();guard[profile_ids]|=~valid[profile_ids].all(axis=1)
   newly=guard&(first_guard<0)&(first_contact<0);first_guard[newly]=step
   for e in profile_ids if not full_backend else []:
    reference=np.array([states[e*12+j].reference for j in range(12)])
    target[e]=qseat+B@(reference-2048)
    actions[e]=torch.tensor((target[e]-ref)/.25,device=env.device)
   previous=goal.copy()
   if step%100==0:
    for e in range(n):
     ai=(ct.c_double*12)(*actor_actions[e]);qo=(ct.c_double*12)();lib.api_action_angles(ai,qo)
     np.testing.assert_allclose(qo,ref+actor_actions[e]*.25,rtol=0,atol=2e-6)
     qi=(ct.c_double*12)(*(ref+actor_actions[e]*.25));ro=(ct.c_uint16*12)();ok=lib.api_raw(qi,ro)
     assert bool(ok)==bool(((goal[e]>=0)&(goal[e]<=4095)).all())
     if ok:np.testing.assert_array_equal(ro,goal[e])
     mapping_checks+=1
   if not full_backend:env.step(actions)
   else:
    env.action_manager.process_action(actions)
    matrix=torch.tensor(direct,dtype=torch.float32,device=env.device)
    dc=torch.tensor([k['plant']=='dc' for k in cases],device=env.device)[:,None]
    # Firmware polarity -1 is inverted by the motor's measured physical PWM
    # direction, before the unchanged signed individual-joint torque mapping.
    drive=torch.tensor(duty*polarity,dtype=torch.float32,device=env.device)/1000
    dead=torch.tensor((first_guard>=0)|(first_contact>=0),device=env.device)
    for _ in range(c.decimation):
     env.action_manager.apply_action()
     motor_velocity=robot.data.joint_vel[:,order]@matrix.T
     torque=torch.clamp(MODEL_STALL_TORQUE_NM*MOTOR_RESPONSE_GAIN*(drive-torch.where(dc,motor_velocity/MODEL_FREE_SPEED_RAD_S,0.)),-1.024,1.024)
     # Same motor model, expressed directly in torque for inverse-PID cases.
     quantized_torque=torch.tensor(angular.k*(duty*polarity),dtype=torch.float32,device=env.device)
     inverse_torque=torch.clamp(quantized_torque-MODEL_STALL_TORQUE_NM*MOTOR_RESPONSE_GAIN*torch.where(dc,motor_velocity/MODEL_FREE_SPEED_RAD_S,0.),-1.024,1.024)
     torch.testing.assert_close(inverse_torque,torque,rtol=0,atol=3e-7)
     torque=torch.where(torch.tensor(angle_cases,device=env.device)[:,None],inverse_torque,torque)
     torque[dead]=0.
     joint_torque=torque@matrix
     robot.set_joint_effort_target(joint_torque[:,np.argsort(order)])
     env.scene.write_data_to_sim();env.sim.step(render=False);env.scene.update(c.sim.dt)
    env.episode_length_buf+=1;env.common_step_counter+=1;env.termination_manager.compute()
   assert observed is not None
   contact=observed.cpu().numpy()
   newly=contact&(first_contact<0)&(first_guard<0);first_contact[newly]=step
   processed=env.action_manager.get_term('joint_pos').processed_actions[:,order].cpu().numpy()
   np.testing.assert_allclose(processed,target,rtol=0,atol=2e-6)
   reference=raw(processed) if not full_backend else ref_raw
   terms=np.array([[s.trace_error,s.trace_p,s.integral,s.trace_d] for s in states]).reshape(n,12,4)
   values=dict(raw_before=before,raw_goal=goal,raw_reference=reference,raw_after=raw(robot.data.joint_pos[:,order].cpu().numpy()),pwm_raw=duty,pid_terms=terms,root_state_w=robot.data.root_state_w.cpu().numpy(),raw_C_pwm=raw_C_pwm,angle_pid_Nm=angle_Nm,angle_pid_terms=angular.terms,shadow_pwm_difference=shadow_difference,
    base_force_N=sensor.data.net_forces_w[:,baseid,:].cpu().numpy(),base_force_history_N=sensor.data.net_forces_w_history[:,:,baseid,:].cpu().numpy(),
    base_height_m=robot.data.root_pos_w[:,2].cpu().numpy(),base_contact=contact,pid_guard=guard,action=actor_actions)
   for k,v in values.items():trace[k].append(v.copy())
   if ((first_contact>=0)|(first_guard>=0)).all():
    print('ALL_CASES_TERMINAL',step+1,flush=True);break
 steps=len(trace['raw_before'])
 x={k:np.asarray(v) for k,v in trace.items()};np.savez_compressed(out/f'{args.label}_trace.npz',initial_observation=initial_observation,**x)
 result=dict(physics_dt=args.physics_dt,control_dt=.02,seconds=steps*.02,max_seconds=args.seconds,steps=steps,linear_velocity_input='forced_xyz_zero',sync_start=args.sync_start,motor_response_gain=MOTOR_RESPONSE_GAIN,command=args.command,
  contact_sensor_update_period_s=c.scene.contact_forces.update_period,contact_history_length=c.scene.contact_forces.history_length,
  limits=dict(integral_output_raw=100,pwm_raw=800,following_error_raw=512,speed_min_raw_s=200,speed_max_raw_s=3000,acc_raw=100,
              physics_effort_Nm=1.024,physics_velocity_rad_s=2.3,plant_nominal_torque_Nm=MODEL_STALL_TORQUE_NM,plant_free_velocity_rad_s=MODEL_FREE_SPEED_RAD_S),
  motor_polarity=polarity.tolist(),individual_torque_matrix=direct.tolist(),
  target_error_raw=132.8900293240416,error_tolerance_fraction=.1,mapping_checks=mapping_checks,initial_observation_max_spread=initial_spread,
  checkpoint_sha256=hashlib.sha256((args.data_dir/'model_6999.pt').read_bytes()).hexdigest(),
  script_sha256=hashlib.sha256(source_bytes).hexdigest(),bridge_sha256=hashlib.sha256((args.data_dir/'libpid.so').read_bytes()).hexdigest(),
  termination='original mdp.illegal_contact: base_link force history >1N; auto-reset suppressed, first hit is terminal',
  controller=args.backend,initial_integral=args.initial_integral,plant_note='Full PID uses production C, actual motor polarity and existing signed individual-joint torque ablation; PWM plant is assumed, not hardware identified.',cases=[])
 for e,k in enumerate(cases):
  contact=first_contact[e];guard=first_guard[e];end=int(min([steps]+[i+1 for i in [contact] if i>=0]+[i for i in [guard] if i>=0]))
  err=abs(x['raw_goal'][:end,e]-x['raw_before'][:end,e]);internal=abs(x['raw_reference'][:end,e]-x['raw_before'][:end,e])
  rolling=np.convolve(err.mean(axis=1),np.ones(50)/50,'valid') if end>=50 else np.array([])
  matched=np.where(abs(rolling-result['target_error_raw'])<=.1*result['target_error_raw'])[0]
  result['cases'].append(dict(**k,valid_steps=end,valid_seconds=end*.02,
   base_contact=bool(contact>=0),contact_interval_s=[contact*.02,(contact+1)*.02] if contact>=0 else None,
   pid_guard_s=guard*.02 if guard>=0 else None,
   mean_final_error_raw=float(err.mean()) if end else None,max_final_error_raw=float(err.max()) if end else None,
   mean_internal_error_raw=float(internal.mean()) if end else None,
   mean_abs_pwm_raw=float(abs(x['pwm_raw'][:end,e]).mean()) if end else None,
   max_shadow_PWM_difference_raw=int(abs(x['shadow_pwm_difference'][:end,e]).max()) if end else None,
   last1s_final_error_raw=float(err[-50:].mean()) if end else None,
   first_matched_1s_window_end_s=float((matched[0]+50)*.02) if len(matched) else None,
   peak_base_force_N=float(np.linalg.norm(x['base_force_history_N'][:max(1,end),e],axis=-1).max()),
   error_per_motor_raw=err.mean(axis=0).tolist() if end else None))
 (out/f'{args.label}_summary.json').write_text(json.dumps(result,indent=2))
 (out/f'{args.label}_SUCCESS').write_text('summary, traces, gain readback and mapping checks completed\n')
 print('ERROR_CONTACT_DONE',json.dumps(result),flush=True)
except BaseException:
 traceback.print_exc();raise
finally:env.close();app.close()
