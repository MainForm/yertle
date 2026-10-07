"""Bounded real-PhysX verification of sampled seated starts and contact cost."""
import argparse
from pathlib import Path
import sys
import traceback

import h5py  # noqa: F401 -- Windows DLL ordering
import torch
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--pose-json", required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import isaac_lab
from isaac_lab.flat_env_cfg import YertleFlatEnvCfg
from isaac_lab.play_start_pose import load_pose_json
from isaac_lab.train_start_pose import configure_reset_pose_pool


@torch.inference_mode()
def main():
    cfg = YertleFlatEnvCfg()
    cfg.scene.num_envs = 64
    pose = load_pose_json(args.pose_json)
    configure_reset_pose_pool(cfg, [pose], True)
    assert cfg.terminations.base_contact is None
    assert cfg.rewards.base_contact_penalty.weight == -1.
    env = gym.make("Isaac-Velocity-Flat-Yertle-v0", cfg=cfg).unwrapped
    try:
        robot = env.scene["robot"]
        defaults = robot.data.default_joint_pos.clone()
        desired = defaults.new_tensor([pose[n] for n in robot.joint_names])
        counts = torch.zeros(2, device=env.device)
        for _ in range(4):
            env.reset()
            selected = env.reset_pose_ids
            counts += torch.bincount(selected, minlength=2)
            expected = torch.where(selected[:, None] == 0, defaults, desired)
            torch.testing.assert_close(robot.data.joint_pos, expected, atol=1e-5, rtol=0)
            torch.testing.assert_close(robot.root_physx_view.get_dof_limits().to(env.device), robot.data.joint_pos_limits)
            env.action_manager.process_action(torch.zeros(64, 12, device=env.device))
            torch.testing.assert_close(env.action_manager.get_term("joint_pos").processed_actions, defaults)
        assert (counts > 75).all(), counts
        term = env.reward_manager.get_term_cfg("base_contact_penalty")
        # Force actual ground contact, not a mocked sensor result.
        root = robot.data.root_state_w.clone()
        root[:, 2] = env.scene.env_origins[:, 2] + .005
        root[:, 7:] = 0
        robot.write_root_state_to_sim(root)
        measured = False
        for _ in range(20):
            _, reward, terminated, _, _ = env.step(torch.zeros(64, 12, device=env.device))
            cost = term.func(env, **term.params)
            assert torch.isfinite(reward).all()
            assert not terminated.any(), "Base contact unexpectedly terminated an episode"
            if (cost > 0).any():
                assert (cost * term.weight < 0).any()
                measured = True
        assert measured, "No physical base contact measured"
        torch.testing.assert_close(robot.data.default_joint_pos, defaults)
        print(f"TRAIN_START_POSE_SIM_PASS counts={counts.tolist()} repeated_reset=4 PhysX_limits_verified offsets_unchanged actual_base_contact_negative_reward_no_termination", flush=True)
    except BaseException:
        traceback.print_exc()
        print("TRAIN_START_POSE_SIM_FAIL", flush=True)
        raise
    finally:
        env.close()


try:
    main()
finally:
    app.close()
