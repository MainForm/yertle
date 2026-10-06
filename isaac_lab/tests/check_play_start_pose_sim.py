"""Bounded native Isaac Sim integration check; use --checkpoint and --headless."""

import argparse
from copy import deepcopy
from pathlib import Path
import sys
import traceback

import h5py  # noqa: F401 -- Windows HDF5 DLL order
import torch
from rsl_rl.runners import OnPolicyRunner
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", required=True)
parser.add_argument("--pose-json", help="Also check an explicit play-only limit extension using this pose.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import isaac_lab  # noqa: E402,F401
from isaac_lab.flat_env_cfg import YertleFlatEnvCfg_PLAY  # noqa: E402
from isaac_lab.play_start_pose import configure_start_pose, reset_selected_pose, load_pose_json  # noqa: E402
from isaac_lab.rsl_rl_ppo_cfg import YertleFlatPPORunnerCfg  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402


@torch.inference_mode()
def main():
    cfg = YertleFlatEnvCfg_PLAY()
    cfg.scene.num_envs = 2
    legacy_event = deepcopy(cfg.events.reset_robot_joints)
    configure_start_pose(cfg, "default", None)
    env = gym.make("Isaac-Velocity-Flat-Yertle-Play-v0", cfg=cfg)
    wrapper = None
    try:
        agent_cfg = YertleFlatPPORunnerCfg()
        wrapper = RslRlVecEnvWrapper(env, clip_actions=getattr(agent_cfg, "clip_actions", None))
        base = env.unwrapped
        robot = base.scene["robot"]
        defaults = robot.data.default_joint_pos.clone()
        limits = robot.data.joint_pos_limits.clone()
        torch.testing.assert_close(robot.data.joint_pos, defaults, atol=1e-5, rtol=0)
        torch.testing.assert_close(robot.data.joint_vel, torch.zeros_like(defaults), atol=1e-5, rtol=0)
        runner = OnPolicyRunner(wrapper, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        runner.load(args.checkpoint)
        policy = runner.get_inference_policy(device=base.device)
        for mode in ("default", "json", "random"):
            event = deepcopy(base.event_manager.get_term_cfg("reset_robot_joints"))
            if mode == "random":
                event = legacy_event
            else:
                event.func = reset_selected_pose
                angles = None
                if mode == "json":
                    # Non-default but admissible pose; distinct from the standing offset.
                    values = defaults[0] * 1.05
                    angles = dict(zip(robot.joint_names, values.tolist()))
                event.params = {"joint_angles": angles}
            base.event_manager.set_term_cfg("reset_robot_joints", event)
            for _ in range(2):
                obs, _ = wrapper.reset()
                expected = defaults if mode == "default" else defaults * 1.05
                if mode != "random":
                    torch.testing.assert_close(robot.data.joint_pos, expected, atol=1e-5, rtol=0)
                    torch.testing.assert_close(robot.data.joint_vel, torch.zeros_like(defaults), atol=1e-5, rtol=0)
                    # Zero action must STILL target training defaults, not the selected start pose.
                    base.action_manager.process_action(torch.zeros(2, 12, device=base.device))
                    torch.testing.assert_close(
                        base.action_manager.get_term("joint_pos").processed_actions, defaults, atol=1e-5, rtol=0
                    )
                with torch.inference_mode():
                    for _ in range(5):
                        action = policy(obs)
                        assert action.shape == (2, 12) and torch.isfinite(action).all()
                        obs, _, _, _ = wrapper.step(action)
                torch.testing.assert_close(robot.data.default_joint_pos, defaults)
                torch.testing.assert_close(robot.data.joint_pos_limits, limits)
            print(f"PLAY_START_POSE_SIM_PASS mode={mode} repeated_reset=2 rollout_steps=10", flush=True)
        # Invalid dynamic pose must fail before any state is written.
        before = robot.data.joint_pos.clone()
        bad = dict(zip(robot.joint_names, defaults[0].tolist()))
        bad[robot.joint_names[0]] = limits[0, 0, 1].item() + 0.5
        try:
            reset_selected_pose(base, [0], bad)
        except ValueError as exc:
            assert "outside PhysX limits" in str(exc)
        else:
            raise AssertionError("Out-of-limit pose was accepted")
        torch.testing.assert_close(robot.data.joint_pos, before)
        print("PLAY_START_POSE_SIM_PASS rejected_invalid_pose; defaults/action_offsets/limits_unchanged", flush=True)
        if args.pose_json:
            angles = load_pose_json(args.pose_json)
            desired = defaults.new_tensor([angles[name] for name in robot.joint_names]).repeat(base.num_envs, 1)
            expected_limits = limits.clone()
            expected_limits[..., 0] = torch.minimum(limits[..., 0], desired)
            expected_limits[..., 1] = torch.maximum(limits[..., 1], desired)
            event = deepcopy(base.event_manager.get_term_cfg("reset_robot_joints"))
            event.func = reset_selected_pose
            event.params = {"joint_angles": angles, "extend_limits": True}
            base.event_manager.set_term_cfg("reset_robot_joints", event)
            for _ in range(2):
                obs, _ = wrapper.reset()
                torch.testing.assert_close(robot.data.joint_pos, desired, atol=1e-5, rtol=0)
                torch.testing.assert_close(robot.data.joint_vel, torch.zeros_like(desired), atol=1e-5, rtol=0)
                torch.testing.assert_close(robot.data.joint_pos_limits, expected_limits)
                torch.testing.assert_close(robot.root_physx_view.get_dof_limits().to(base.device), expected_limits)
                base.action_manager.process_action(torch.zeros(2, 12, device=base.device))
                torch.testing.assert_close(base.action_manager.get_term("joint_pos").processed_actions, defaults)
                for _ in range(10):
                    action = policy(obs)
                    assert action.shape == (2, 12) and torch.isfinite(action).all()
                    obs, rewards, _, _ = wrapper.step(action)
                    assert torch.isfinite(rewards).all()
                torch.testing.assert_close(robot.data.default_joint_pos, defaults)
            print("PLAY_START_POSE_SIM_PASS extended_json; repeated_reset=2 rollout_steps=20 PhysX_limits_verified offsets_unchanged", flush=True)
    except BaseException:
        # Kit's forced shutdown can hide a pending exception; report it BEFORE cleanup.
        traceback.print_exc()
        print("PLAY_START_POSE_SIM_FAIL", flush=True)
        sys.stdout.flush()
        sys.stderr.flush()
        raise
    finally:
        (wrapper or env).close()


try:
    main()
finally:
    app.close()
