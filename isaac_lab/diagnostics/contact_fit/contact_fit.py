"""Replay acknowledged goals or recorded PWM while PhysX determines motion."""
import bisect
import inspect
import json
import math
from pathlib import Path
import os
import textwrap

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from isaac_lab.foot_geometry import foot_sole_heights

STAGE=Path(os.environ['YERTLE_CONTACT_REPLAY_DIR'])
from mass_cases import CASES, VARIANTS, apply_mass_cases
PID_PERIOD_MS={'fit_vx025':25,'check_vx020':24}
LEGS=('rf','lf','rb','lb')

def run_trial(env):
    cases, variants = CASES, VARIANTS
    use_imu_com = os.environ.get("YERTLE_CONTACT_MODE") == "imu-com"
    if use_imu_com:
        import imu_com

        cases, variants = imu_com.CASES, imu_com.VARIANTS
    torch.set_num_threads(1)
    robot=env.scene['robot']
    drive=robot.actuators['legs']
    n=len(cases)
    assert env.num_envs==n and abs(env.physics_dt-.001)<1e-8
    dev=env.device
    to=lambda values: torch.as_tensor(np.array(values),device=dev,dtype=robot.data.joint_pos.dtype)
    raw_transform=drive.raw_transform
    def inverse(raw):
        delta=(raw-2048)*raw_transform.signs*(2*math.pi/4096)
        delta[:,::3]-=delta[:,1::3]
        q=torch.empty_like(delta)
        q[:,raw_transform.indices]=delta+raw_transform.reference
        torch.testing.assert_close(raw_transform(q),raw,atol=.001,rtol=0)
        return q
    def material():
        view=robot.root_physx_view
        props=view.get_material_properties()
        sizes=[robot._physics_sim_view.create_rigid_body_view(p).max_shapes for p in view.link_paths[0]]
        assert sum(sizes)==view.max_shapes
        offsets=np.r_[0,np.cumsum(sizes)]
        assert [p.rsplit('/',1)[-1] for p in view.link_paths[0]]==robot.body_names
        indices={}
        for leg in LEGS:
            j=robot.body_names.index(leg+'_shin')
            indices[leg]=slice(int(offsets[j]),int(offsets[j+1]))
            assert sizes[j]>0
        props[:]=torch.tensor([.8,.6,0.],dtype=props.dtype)
        for i,(_,_,values) in enumerate(cases):
            for leg,value in zip(LEGS,values):
                props[i,indices[leg]]=torch.tensor([.8*value,.6*value,0.],dtype=props.dtype)
        view.set_material_properties(props,torch.arange(n,dtype=torch.int32))
        torch.testing.assert_close(view.get_material_properties(),props)
        return {k:[v.start,v.stop] for k,v in indices.items()}

    # For PWM replay use the same voltage-sag and torque path after the PID stage.
    # Hardware polarity is -1 and the physical motor-ID signs are unchanged.
    compute=type(drive).compute
    body=textwrap.dedent(inspect.getsource(compute))
    anchor='    motor_vel = joint_to_motor_velocity(joint_vel, self.thigh, self.shin)'
    assert body.count(anchor)==1
    namespace=dict(compute.__globals__)
    exec(body.replace(anchor,'    if self.replay_pwm_enabled:\n        self.motor_pwm[self.replay_mask] = self.replay_pwm\n'+anchor),namespace)
    type(drive).compute=namespace['compute']
    drive.replay_mask=torch.tensor([mode=='pwm' for mode,_,_ in cases],device=dev)
    drive.replay_pwm_enabled=False
    drive.replay_pwm=torch.zeros((1,12),device=dev)
    raw_body_names=[leg+'_shin' for leg in LEGS]

    inputs=json.loads((STAGE/'inputs.json').read_text())
    report=dict(source=inputs['log'],cases=cases,foot_link_shapes=material(),pid_period_ms=PID_PERIOD_MS,dt_s=env.physics_dt)
    offsets, report['body_mass_properties'] = apply_mass_cases(robot, cases, variants)
    if use_imu_com:
        report['imu_com'] = {
            'update_s': imu_com.UPDATE_S,
            'filter_s': imu_com.FILTER_S,
            'deadband_deg': imu_com.DEADBAND_DEG,
            'full_offset_deg': imu_com.FULL_OFFSET_DEG,
            'variants': {name: setting[2:] for name, setting in imu_com.FITTED.items()},
        }
    (STAGE/'config.json').write_text(json.dumps(report,indent=2))
    with (STAGE/'trials.jsonl').open('w') as stream:
        for window in inputs['windows']:
            period=PID_PERIOD_MS[window['name']]
            drive.cfg.control_decimation=period
            drive.cfg.control_period_s=period*.001
            env.reset()
            drive.replay_pwm_enabled=False
            q=inverse(to(window['raw']).repeat(n,1))
            qd=to(window['qd']).repeat(n,1)
            quat=Rotation.align_vectors([[0,0,-1]],[window['g']])[0]
            pos=env.scene.env_origins.clone()
            pos[:,2]+=.4
            state=robot.data.root_state_w.clone()
            state[:,:3]=pos
            state[:,3:7]=to(quat.as_quat()[[3,0,1,2]])
            state[:,7:]=0
            robot.write_root_state_to_sim(state)
            robot.write_joint_state_to_sim(q,torch.zeros_like(q))
            robot.set_joint_position_target(q)
            env.scene.write_data_to_sim()
            env.sim.step(render=False)
            env.scene.update(env.physics_dt)
            soles=foot_sole_heights(env)
            state[:,2]+=.001-soles.min(1).values
            state[:,7:10]=to(quat.apply(window['v']))
            state[:,10:]=to(quat.apply(window['w']))
            # Preserve the same initial link velocity while its CoM moves.
            world_offsets = to(quat.apply(offsets.cpu().numpy()))
            state[:,7:10] += torch.linalg.cross(state[:,10:], world_offsets, dim=-1)
            robot.write_root_state_to_sim(state)
            robot.write_joint_state_to_sim(q,qd)
            drive.reset(None)
            drive.replay_pwm_enabled=True
            gtimes,goals=zip(*window['goals'])
            ptimes,pwms=zip(*window['pwm'])
            raw_goal=to(goals)
            q_goals=inverse(raw_goal)
            pwm_id=to(pwms)
            pwm_q=torch.zeros_like(pwm_id)
            pwm_q[:,raw_transform.indices]=-pwm_id*raw_transform.signs
            t_end=min(window['end']-window['start'],window['feedback'][-1]['t'])
            if use_imu_com:
                com_replay = imu_com.ImuComReplay(robot, window['feedback'], t_end, offsets)
            for k in range(round(t_end/env.physics_dt)+1):
                t=k*env.physics_dt
                if use_imu_com:
                    com_replay.step(k)
                g=max(0,bisect.bisect_right(gtimes,t)-1)
                p=max(0,bisect.bisect_right(ptimes,t)-1)
                robot.set_joint_position_target(q_goals[g].expand(n,-1))
                drive.replay_pwm=pwm_q[p:p+1]
                if k%20==0:
                    raw=raw_transform(robot.data.joint_pos)
                    yaw_body=robot.data.root_ang_vel_b[:,2]
                    yaw_world=robot.data.root_ang_vel_w[:,2]
                    up=-robot.data.projected_gravity_b[:,2]
                    z=robot.data.root_link_pos_w[:,2]
                    vel=robot.data.root_lin_vel_b
                    data=dict(window=window['name'],t=t,raw=raw.tolist(),goal=goals[g],yaw_body=yaw_body.tolist(),
                              yaw_world=yaw_world.tolist(),tilt=torch.rad2deg(torch.acos(up.clamp(-1,1))).tolist(),
                              z=z.tolist(),v=vel.tolist(),pid_guard=drive.pid_guard.tolist())
                    if use_imu_com:
                        data['com_offset_mm'] = (1000.0 * com_replay.offsets).tolist()
                    stream.write(json.dumps(data,separators=(',',':'))+'\n')
                    if k%1000==0:print('CONTACT_FIT',window['name'],t,flush=True)
                env.scene.write_data_to_sim()
                env.sim.step(render=False)
                env.scene.update(env.physics_dt)
            if use_imu_com:
                com_replay.reset_offsets()
            stream.flush()
            print('CONTACT_WINDOW_COMPLETE',window['name'],flush=True)
    print('CONTACT_FIT_COMPLETE',flush=True)
