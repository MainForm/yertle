"""Verify real Isaac Lab reward registration without creating or training an env."""
import argparse
import ast
from pathlib import Path
import sys
from types import SimpleNamespace

import torch  # DLL ordering before Kit
from isaaclab.app import AppLauncher

parser=argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args=parser.parse_args()
app=AppLauncher(args).app
try:
    import yaml
    root=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(root))
    from isaac_lab.flat_env_cfg import YertleFlatEnvCfg
    from isaac_lab.flat_env_cfg_reward import feet_air_time_trot, track_lin_vel_xy_cauchy, feet_slide_sole, ActionSecondDifferencePenalty
    cfg=YertleFlatEnvCfg()
    assert getattr(cfg.rewards,'body_height_barrier',None) is None
    old=yaml.load((root/'isaac_lab/runs/yertle_flat/2026-10-11_07-22-13/params/env.yaml').read_text(),Loader=yaml.BaseLoader)['rewards']
    changed={'feet_air_time','track_lin_vel_xy_cauchy','track_ang_vel_z_cauchy','phase_unscheduled_contact_penalty','phase_swing_clearance','action_second_difference_l2','nonsole_foot_contact_penalty','motor_operating_range_penalty'}
    unchanged=0
    for name,term in vars(cfg.rewards).items():
        if not hasattr(term,'weight') or name in changed:
            continue
        assert term.weight==float(old[name]['weight']),(name,term.weight,old[name]['weight'])
        unchanged+=1
    assert cfg.rewards.feet_air_time.func is feet_air_time_trot
    assert cfg.rewards.feet_slide.func is feet_slide_sole
    assert cfg.rewards.action_second_difference_l2.func is ActionSecondDifferencePenalty
    assert cfg.rewards.action_second_difference_l2.weight == -.05
    assert cfg.rewards.action_second_difference_l2.params == {'action_name': 'joint_pos'}
    assert cfg.rewards.feet_slide.weight == -.10
    assert cfg.rewards.feet_slide.params['surface_band_m'] == .001
    assert cfg.rewards.feet_air_time.weight==1.0
    assert cfg.rewards.feet_air_time.params['threshold']==.15
    assert cfg.rewards.track_lin_vel_xy_cauchy.func is track_lin_vel_xy_cauchy
    assert cfg.rewards.track_lin_vel_xy_cauchy.params['components']==(0,1)
    assert cfg.rewards.track_lin_vel_xy_cauchy.weight==2.0
    assert cfg.rewards.track_ang_vel_z_cauchy.weight==.75
    assert cfg.rewards.phase_unscheduled_contact_penalty.weight==-.4
    assert cfg.rewards.phase_swing_clearance.weight==-.20
    assert cfg.rewards.phase_swing_clearance.params['min_height_m']==.045
    assert cfg.rewards.phase_swing_clearance.params['transition_m']==.015
    assert cfg.rewards.dof_pos_limits is None
    assert cfg.rewards.motor_operating_range_penalty.weight== -100.
    assert cfg.scene.robot.actuators['legs'].velocity_limit_sim==4.6
    assert cfg.scene.robot.actuators['legs'].effort_limit_sim==1.9
    assert cfg.actions.joint_pos.scale==.25
    tree=ast.parse((root/'isaac_lab/train.py').read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    block=next(n for n in main.body if isinstance(n,ast.If) and ast.unparse(n.test)=='args_cli.vx_only')
    exec(compile(ast.Module(body=[block],type_ignores=[]),'<actual train vx-only>','exec'),{'env_cfg':cfg,'args_cli':SimpleNamespace(task='flat',vx_only=True)})
    assert cfg.rewards.track_lin_vel_xy_cauchy.params['components']==(0,)
    env=SimpleNamespace(scene={'robot':SimpleNamespace(data=SimpleNamespace(root_lin_vel_b=torch.zeros(1,3)))},command_manager=SimpleNamespace(get_command=lambda _:torch.tensor([[.1,0,0]])))
    score=cfg.rewards.track_lin_vel_xy_cauchy.func(env,**cfg.rewards.track_lin_vel_xy_cauchy.params)
    torch.testing.assert_close(score,torch.tensor([.5]))
    print(f'MOVEMENT_REWARD_CONFIG_PASS air_threshold=0.15 vx_components=(0,) phase_penalty=-0.4 unchanged_other_weights={unchanged} no_env_created_no_training',flush=True)
finally:
    app.close()
