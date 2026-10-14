"""Check real manager calls and partial resets with a two-env rollout."""
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
    from isaac_lab.flat_env_cfg_reward import ActionSecondDifferencePenalty
    from isaac_lab.signed_joint_reference import configure_signed_joint_reference
    from isaac_lab.play_start_pose import configure_start_pose
    cfg=YertleFlatEnvCfg_PLAY();cfg.scene.num_envs=2;cfg.seed=42
    configure_signed_joint_reference(cfg);configure_start_pose(cfg,'default',None)
    env=gym.make('Isaac-Velocity-Flat-Yertle-Play-v0',cfg=cfg).unwrapped
    try:
        env.reset()
        term_cfg=env.reward_manager.get_term_cfg('action_second_difference_l2')
        term=term_cfg.func
        assert isinstance(term,ActionSecondDifferencePenalty)
        assert term_cfg.weight==-.05
        index=env.reward_manager.active_terms.index('action_second_difference_l2')
        for step,sign in enumerate([1,-1,1,-1,1,-1]):
            if step==3:env._reset_idx(torch.tensor([0],device=env.device))
            env.step(torch.full((2,12),.2*sign,device=env.device))
            expected=torch.tensor([0,0] if step<2 else [0,7.68] if step in (3,4) else [7.68,7.68],device=env.device,dtype=term.value.dtype)
            torch.testing.assert_close(term(env,**term_cfg.params),expected,atol=1e-5,rtol=1e-5)
            torch.testing.assert_close(term(env,**term_cfg.params),expected,atol=1e-5,rtol=1e-5)
            torch.testing.assert_close(env.reward_manager._step_reward[:,index],expected*-.05,atol=1e-6,rtol=1e-5)
        assert env.action_manager.total_action_dim==12
        print('ACTION_SECOND_DIFF_SIM_PASS envs=2 steps=6 weight=-0.05 partial_reset=True warmup=True repeated_call=True manager_dt_once=True actions=12',flush=True)
    finally:env.close()
except BaseException:
    import traceback
    traceback.print_exc()
    print('ACTION_SECOND_DIFF_SIM_FAIL',flush=True)
    raise
finally:app.close()
