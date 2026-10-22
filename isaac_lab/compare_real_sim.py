"""Two robots: read-only measured-pose replay and independent ONNX/PhysX rollout.

No serial access, no commands to the real robot, no changes to calibrated APIs.
Existing CLI JSONL is consumed as-is; only command/run intent is shared after
an explicit initial synchronization. SIM observations always use its own state.
"""
import argparse
import asyncio
import copy
from datetime import datetime
import json
from pathlib import Path
import shutil
import sys
import tempfile
import time

HERE=Path(__file__).resolve().parent
DEFAULT_ROOT=Path(r'C:\Users\ik\Desktop\yertle-RTOS\policy_onnx')

def main():
    import h5py  # Match existing Windows native DLL ordering.
    import torch
    import numpy as np
    from isaaclab.app import AppLauncher
    from compare_tools.core import Actor,Feed,RealFrame,JOINTS,REFERENCE,independent_observation
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,default=DEFAULT_ROOT/'models/active/policy.onnx')
    source=p.add_mutually_exclusive_group()
    source.add_argument('--log',type=Path,help='Read/follow one CLI JSONL; never opens COM')
    source.add_argument('--replay',type=Path,help='Replay a completed CLI JSONL using its recorded time')
    source.add_argument('--demo',action='store_true',help='Synthetic sensor mirror, explicitly NOT real measurements')
    p.add_argument('--logs-dir',type=Path,default=DEFAULT_ROOT/'dist/v6.21.6_model_2850_cpu/logs')
    p.add_argument('--replay-speed',type=float,default=1.)
    p.add_argument('--linear-velocity',choices=['zero','measured'],default='zero',help='zero matches the real CLI --zero-velocity-test')
    p.add_argument('--effort-limit-sim',type=float,default=1.9)
    p.add_argument('--velocity-limit-sim',type=float,default=4.6)
    p.add_argument('--stale-seconds',type=float,default=.5)
    p.add_argument('--seconds',type=float,default=0.,help='0: keep the viewer open')
    p.add_argument('--steps',type=int,default=0,help='Optional independent policy-step limit')
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--capture',action='store_true',help='Save a final viewport PNG for verification')
    p.add_argument('--exit-status-file',type=Path,help=argparse.SUPPRESS)
    AppLauncher.add_app_launcher_args(p)
    p.set_defaults(device='cpu',rendering_mode='performance')
    args=p.parse_args()
    if not all(np.isfinite(v) and v>0 for v in [args.replay_speed,args.effort_limit_sim,args.velocity_limit_sim,args.stale_seconds]): p.error('Positive finite limits/speeds required')
    if not np.isfinite(args.seconds) or args.seconds<0 or args.steps<0 or args.headless and not (args.seconds or args.steps): p.error('Bound headless runs with --seconds/--steps >0')
    actor=Actor(args.model)
    output=(args.output_dir or HERE/'runs/real_sim_compare'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')).resolve()
    output.mkdir(parents=True,exist_ok=False)
    launcher=AppLauncher(args);app=launcher.app
    sys.path.insert(0,str(HERE.parent))
    import gymnasium as gym
    import isaac_lab
    import isaaclab.sim as sim_utils
    from isaac_lab.flat_env_cfg import YertleFlatEnvCfg_PLAY
    from isaac_lab.signed_joint_reference import configure_signed_joint_reference
    from isaac_lab.joint_position_limits import release_joint_position_limits
    from isaac_lab.play_start_pose import configure_start_pose
    from isaac_lab.play_controls import apply_command
    import omni.ui as ui
    from compare_tools.display_asset import create_display_asset
    from compare_tools.physics import advance
    cfg=YertleFlatEnvCfg_PLAY()
    cfg.scene.num_envs=1;cfg.seed=0;cfg.sim.device=args.device
    cfg.trot_frequency_hz=1.7;cfg.trot_duty_factor=.6
    cfg.scene.robot.actuators['legs'].effort_limit_sim=args.effort_limit_sim
    cfg.scene.robot.actuators['legs'].velocity_limit_sim=args.velocity_limit_sim
    configure_signed_joint_reference(cfg);configure_start_pose(cfg,'default',None)
    cfg.episode_length_s=1.e6;cfg.terminations.base_contact=None;cfg.terminations.time_out=None
    cfg.events.physics_material=None;cfg.events.add_base_mass=None;cfg.events.push_robot=None
    cfg.events.base_external_force_torque=None
    cfg.events.reset_base.params['pose_range']={k:(0.,0.) for k in ['x','y','yaw']}
    cfg.commands.base_velocity.heading_command=False;cfg.commands.base_velocity.rel_heading_envs=0.
    cfg.commands.base_velocity.rel_standing_envs=0.;cfg.commands.base_velocity.resampling_time_range=(1.e6,1.e6)
    cfg.commands.base_velocity.debug_vis=False
    mirror_cfg=copy.deepcopy(cfg.scene.robot)
    mirror_cfg.prim_path='{ENV_REGEX_NS}/Measured'
    mirror_cfg.init_state.pos=(0.,.7,.25)
    mirror_cfg.spawn.rigid_props.disable_gravity=True
    mirror_cfg.spawn.usd_path,display_colliders=create_display_asset(mirror_cfg.spawn.usd_path,output/'display_only.usda')
    mirror_cfg.spawn.collision_props=None
    mirror_cfg.actuators['legs'].stiffness=0.;mirror_cfg.actuators['legs'].damping=0.
    mirror_cfg.actuators['legs'].effort_limit_sim=0.
    cfg.scene.measured=mirror_cfg
    env=None
    report=dict(model=str(actor.path),onnx_sha256=actor.sha256,metadata=actor.metadata,
                hardware_access=False,source='SYNTHETIC DEMO' if args.demo else str(args.replay or args.log or args.logs_dir),
                independent_feedback='PhysX robot only; independent action history and phase',
                shared='command and start/stop intent; measured pose used only on explicit/session initial reset',
                replay='direct state replay, gravity/collisions/drives disabled; display-only aligned XY/nominal height/yaw0',
                linear_velocity=args.linear_velocity,effort_limit_sim=args.effort_limit_sim,velocity_limit_sim=args.velocity_limit_sim,
                display_colliders_disabled=display_colliders,control_dt=.02,frames=0,sim_steps=0,sim_root_writes=0,max_env_observation_error=0.,max_replay_joint_error=0.)
    error=None
    trace=(output/'timeline.jsonl').open('w',encoding='utf-8')
    try:
        env=gym.make('Isaac-Velocity-Flat-Yertle-Play-v0',cfg=cfg);env.reset();b=env.unwrapped
        # Set only the manager's automatic render schedule AFTER context creation.
        # Passing a huge rendering_dt to SimulationContext at startup stalls Kit.
        # GUI rendering below remains at 30Hz; PhysX/control stay 5ms/20ms.
        b.cfg.sim.render_interval=1000000
        robot=b.scene['robot'];mirror=b.scene['measured']
        if list(robot.joint_names)!=list(JOINTS) or mirror.joint_names!=robot.joint_names: raise ValueError('Runtime joint order mismatch')
        release_joint_position_limits(b)
        # The display replica is never an input to the policy or a contact sensor.
        from types import SimpleNamespace
        release_joint_position_limits(SimpleNamespace(scene={'robot':mirror}))
        if robot.is_fixed_base: raise ValueError('Independent robot must be free-base')
        if np.any(np.abs(robot.root_physx_view.get_dof_max_forces().cpu().numpy()-args.effort_limit_sim)>1e-5): raise ValueError('SIM effort cap mismatch')
        if mirror.root_physx_view.get_dof_max_forces().abs().max().item()!=0: raise ValueError('Replay drive is not disabled')
        from pxr import Usd,UsdPhysics
        colliders=[prim for prim in Usd.PrimRange(b.sim.stage.GetPrimAtPath('/World/envs/env_0/Measured')) if prim.HasAPI(UsdPhysics.CollisionAPI)]
        if not colliders or any(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() for prim in colliders):
            raise ValueError('Replay collision shapes were not disabled in the actual scene')
        report['runtime_replay_colliders_disabled']=len(colliders)
        feed=Feed(args.replay or args.log,None if args.replay or args.log else args.logs_dir,replay=bool(args.replay),speed=args.replay_speed)
        started=time.monotonic();next_step=started;step=0;last_action=np.zeros(12,dtype=np.float32)
        active=False;paused=False;reset_requested=False;quit_requested=False;session_key=None;current=None
        old_mode=None;message='Waiting for CLI. Viewer never connects or arms hardware.'
        replay_anchor=None
        cmd=np.zeros(3,dtype=np.float32);last_camera=0.;last_render=0.;source_key=None
        window=ui.Window('Yertle real / independent SIM',width=510,height=330)
        window.position_x=10;window.position_y=390
        with window.frame:
            with ui.VStack(spacing=5):
                ui.Label('READ ONLY | measured replay + INDEPENDENT ONNX / PhysX',height=22)
                ui.Label('A / NEAR Robot: own physics -> ONNX -> own action',height=20)
                ui.Label('B / FAR Measured: CLI obs55 -> measured joints / roll-pitch',height=20)
                ui.Label('Only commands/run intent shared. REAL yaw/XY/height not measured.',height=22)
                ui.Label('B has no contact, gravity or drive. SIM motor/PID is uncalibrated.',height=22)
                with ui.HStack(height=30):
                    def reset_click():
                        nonlocal reset_requested
                        reset_requested=True
                    def pause_click():
                        nonlocal paused
                        paused=not paused
                    def quit_click():
                        nonlocal quit_requested
                        quit_requested=True
                    ui.Button('Sync initial pose / start SIM',clicked_fn=reset_click)
                    ui.Button('Pause SIM',clicked_fn=pause_click)
                    ui.Button('Exit viewer only',clicked_fn=quit_click)
                status=ui.Label('Waiting...',height=145,word_wrap=True)

        def reset_sim(frame):
            nonlocal step,last_action,active,replay_anchor
            if frame is None or frame.obs is None: raise ValueError('Cannot synchronize without valid measured input')
            q=frame.q
            root=robot.data.default_root_state.clone();root[:,:3]=root.new_tensor([0.,0.,.25]);root[:,3:7]=root.new_tensor(frame.quaternion);root[:,7:]=0
            robot.write_root_state_to_sim(root)
            robot.write_joint_state_to_sim(root.new_tensor(q)[None,:],torch.zeros((1,12),device=b.device))
            robot.set_joint_position_target(root.new_tensor(q)[None,:]);robot.reset();b.action_manager.reset()
            step=max(0,frame.episode_step);b.episode_length_buf[:]=step
            last_action=np.zeros(12,dtype=np.float32);report['sim_root_writes']+=1;active=True
            report.setdefault('resets',[]).append(dict(source_time=frame.time_s,sim_step=step,initial_q=q.tolist()))
            if args.replay:replay_anchor=(frame.time_s,report['sim_steps'])

        while app.is_running() and not quit_requested:
            now=time.monotonic();elapsed=now-started
            if args.seconds and elapsed>=args.seconds: break
            if args.steps and report['sim_steps']>=args.steps: break
            if args.demo:
                # Deliberately diverging measured pose: proves replay is NOT SIM feedback.
                oq=np.zeros(55,dtype=np.float32);oq[8]=-1;oq[50:53]=[0,1,0];oq[53:]=[1.7,.6]
                oq[9]=-.1
                oq[19]=np.float32(.35*np.sin(elapsed*2))
                oq[23]=np.float32(-.5*np.sin(elapsed*2))
                current=RealFrame(elapsed,'POLICY',np.array([-.1,0,0],dtype=np.float32),oq,int(elapsed/.02),True)
                age=0.;compatible=True
            else:
                try:
                    source_time=None
                    if args.replay and replay_anchor is not None and active:
                        source_time=replay_anchor[0]+(report['sim_steps']-replay_anchor[1])*.02*args.replay_speed
                    current=feed.poll(source_time);age=feed.age()
                    actor.check_cli(feed.decoder.info);compatible=True
                    if current is not None and current.obs is not None and current.zero_velocity is not None:
                        if bool(current.zero_velocity)!=(args.linear_velocity=='zero'): raise ValueError('CLI/SIM linear-velocity input mode mismatch')
                    key=str(feed.path)
                    if key!=source_key: active=False;session_key=None;old_mode=None;source_key=key
                except (ValueError,OSError,json.JSONDecodeError) as exc:
                    compatible=False;age=feed.age();message=str(exc);active=False
            fresh=current is not None and age<=args.stale_seconds and not (args.replay and feed.eof)
            if fresh: cmd=current.cmd.copy()
            new_session=fresh and current.mode=='POLICY' and (old_mode!='POLICY' or session_key!=current.session)
            if reset_requested or new_session:
                try:
                    if not compatible or not fresh: raise ValueError('Need a fresh, matching CLI model/input')
                    reset_sim(current);session_key=current.session;message='SIM initialized once; subsequent REAL state is display-only.'
                except ValueError as exc:message=str(exc)
                reset_requested=False
            if fresh:
                if old_mode=='POLICY' and current.mode!='POLICY': active=False;message='CLI left POLICY; independent SIM paused.'
                old_mode=current.mode
            if not fresh:
                message='STALE / WAITING: no fresh CLI frame; SIM and last measured pose frozen.'
            if active and fresh and compatible and not paused and now>=next_step:
                apply_command(b,torch.tensor(cmd,device=b.device)[None,:])
                own=robot.data
                obs=independent_observation(own.root_lin_vel_b[0].cpu().numpy(),own.root_ang_vel_b[0].cpu().numpy(),
                    own.projected_gravity_b[0].cpu().numpy(),cmd,own.joint_pos[0].cpu().numpy(),own.joint_vel[0].cpu().numpy(),
                    last_action,step,args.linear_velocity=='zero')
                expected=b.observation_manager.compute()['policy'][0].cpu().numpy().copy()
                if args.linear_velocity=='zero':expected[:3]=0
                delta=float(np.max(np.abs(obs-expected)));report['max_env_observation_error']=max(report['max_env_observation_error'],delta)
                if delta>2.e-5:raise ValueError(f'Independent observation does not match training layout: {delta}')
                action=actor.infer(obs)
                # No autograd model is used here (ORT returns NumPy). Keep PhysX
                # lazy buffers normal tensors so later display state writes work.
                advance(b,torch.tensor(action,device=b.device)[None,:])
                last_action=action.copy();step+=1;report['sim_steps']+=1
                trace.write(json.dumps(dict(time_s=elapsed,source_time_s=current.time_s,source_sample=current.sample,
                    sim_step=step,command=cmd.tolist(),independent_obs55=obs.tolist(),independent_action12=action.tolist(),
                    sim_q=robot.data.joint_pos[0].cpu().tolist(),measured_q=current.q.tolist() if current.q is not None else None))+'\n')
                next_step=max(next_step+.02,now)  # No unbounded catch-up or forced REAL state synchronization.
                if report['sim_steps']%100==0:print('COMPARE_SIM_STEPS',report['sim_steps'],'source_sample',current.sample,flush=True)
            elif now>next_step+.1:next_step=now
            if fresh and current.q is not None:
                root=mirror.data.default_root_state.clone()
                xy=robot.data.root_pos_w[0,:2].cpu().numpy()
                root[:,:3]=root.new_tensor([float(xy[0]),float(xy[1])+.7,.25])
                root[:,3:7]=root.new_tensor(current.quaternion);root[:,7:]=0
                mirror.write_root_state_to_sim(root)
                mirror.write_joint_state_to_sim(root.new_tensor(current.q)[None,:],torch.zeros((1,12),device=b.device))
                mirror.write_data_to_sim()
                b.sim.physics_sim_view.update_articulations_kinematic()
                err=float(np.max(abs(mirror.data.joint_pos[0].cpu().numpy()-current.q)))
                report['max_replay_joint_error']=max(report['max_replay_joint_error'],err)
            if elapsed-last_camera>.2:
                pos=robot.data.root_pos_w[0].cpu().tolist()
                b.sim.set_camera_view([pos[0]+1.,pos[1]-1.4,.95],[pos[0],pos[1]+.35,.18])
                last_camera=elapsed
            mode=current.mode if current else '-'
            age_text=f'{age*1000:.0f}ms' if np.isfinite(age) else '-'
            status.text=(f'Model checkpoint: {actor.metadata.get("yertle.checkpoint_iteration","?")} | source: '+('SYNTHETIC DEMO' if args.demo else str(feed.path or 'waiting'))+
                f'\nCLI mode {mode} | age {age_text} | command {np.round(cmd,3)}\nSIM time {report["sim_steps"]*.02:.2f}s / wall {elapsed:.2f}s | phase step {step} | '+('RUNNING' if active and fresh and not paused else 'PAUSED')+
                f'\nLinear velocity input: {args.linear_velocity} | reference 0/+34/-85 deg\n'+message)
            if elapsed-last_render>=1/30 or not active:
                b.sim.render();last_render=elapsed
            elif args.headless:app.update()
            report['frames']+=1
            time.sleep(.001)
        if args.capture and not args.headless:
            from omni.kit.viewport.utility import get_active_viewport_window,capture_viewport_to_file
            async def capture():
                tmp=Path(tempfile.mkdtemp(prefix='yertle-compare-'))/'compare.png'
                viewport=get_active_viewport_window()
                task=capture_viewport_to_file(viewport.viewport_api,str(tmp));await task.wait_for_result()
                for _ in range(100):
                    import omni.kit.app
                    await omni.kit.app.get_app().next_update_async()
                    if tmp.exists() and tmp.stat().st_size>1000:
                        try:
                            from PIL import Image
                            with Image.open(tmp) as im:im.verify()
                            shutil.copy2(tmp,output/'compare.png');return
                        except (OSError,SyntaxError):pass
                raise RuntimeError('Viewport capture did not complete')
            future=asyncio.ensure_future(capture())
            while not future.done():app.update()
            future.result()
        report['simulated_seconds']=report['sim_steps']*.02;report['wall_seconds']=time.monotonic()-started
        report['passed']=report['sim_steps']>0 and (not args.steps or report['sim_steps']>=args.steps) and report['max_replay_joint_error']<1.e-5 and report['max_env_observation_error']<2.e-5
        report['final_sim_q']=robot.data.joint_pos[0].cpu().tolist()
        if current and current.q is not None:report['final_measured_q']=current.q.tolist()
        if args.demo:
            report['different_robots_verified']=bool(np.max(abs(np.array(report['final_sim_q'])-np.array(report['final_measured_q'])))>1.e-3)
            report['passed']=report['passed'] and report['different_robots_verified']
        print('COMPARE_DONE',output,'steps',report['sim_steps'],'passed',report['passed'],flush=True)
    except BaseException as exc:
        error=exc;report['passed']=False;report['error']=repr(exc)
        import traceback
        traceback.print_exc()
        raise
    finally:
        trace.close();(output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        if args.exit_status_file:args.exit_status_file.write_text('1' if error else '0',encoding='ascii')
        if env is not None:env.close()
        app.close()

if __name__=='__main__':main()
