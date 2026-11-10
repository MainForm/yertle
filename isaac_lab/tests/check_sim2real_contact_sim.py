"""Run with Isaac Sim to check the tilt-driven CoM and partial reset."""
import argparse
import math
import sys
from pathlib import Path

import torch
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import gymnasium as gym
from isaaclab.utils.math import quat_apply

import isaac_lab
from isaac_lab.checkpoint_state import FlatEnvState
from isaac_lab.flat_env_cfg import YertleFlatEnvCfg
from isaac_lab.linked_motor_training import configure_linked_motor_actuator
from isaac_lab.signed_joint_reference import configure_signed_joint_reference
from isaac_lab.sim2real_contact import configure_sim2real_contact

cfg = YertleFlatEnvCfg()
cfg.scene.num_envs = 2
cfg.sim.device = args.device
configure_signed_joint_reference(cfg)
configure_linked_motor_actuator(cfg)
configure_sim2real_contact(cfg)

env = gym.make("Isaac-Velocity-Flat-Yertle-v0", cfg=cfg)
try:
    env.reset()
    base = env.unwrapped
    robot = base.scene["robot"]
    view = robot.root_physx_view
    body = robot.body_names.index("base_link")
    nominal = view.get_coms().clone()
    inertias = view.get_inertias().clone()
    pose = robot.data.root_link_pose_w.clone()
    angles = pose.new_tensor([2.0, 30.0]).deg2rad() / 2.0
    pose[:, 3:] = 0.0
    pose[:, 3] = angles.cos()
    pose[:, 4] = angles.sin()
    robot.write_root_link_pose_to_sim(pose)
    velocity = robot.data.root_com_vel_w.clone()
    velocity[:, 5] = 1.0
    robot.write_root_com_velocity_to_sim(velocity)

    base.event_manager.apply(mode="interval", dt=base.step_dt)
    offset = view.get_coms()[:, body, :3] - nominal[:, body, :3]
    expected = offset.new_tensor([[0, 0, 0], [0, -0.015 * (1 - math.exp(-0.02 / 0.1)), 0]])
    torch.testing.assert_close(offset, expected, atol=1e-6, rtol=0)
    torch.testing.assert_close(view.get_inertias(), inertias)
    expected_velocity = velocity.clone()
    expected_velocity[:, :3] += torch.linalg.cross(velocity[:, 3:], quat_apply(pose[:, 3:], offset.to(base.device)))
    torch.testing.assert_close(robot.data.root_com_vel_w, expected_velocity)

    state = FlatEnvState(base)
    saved = state.capture()
    shifted = view.get_coms().clone()
    base.event_manager.reset(env_ids=torch.tensor([1], device=base.device))
    torch.testing.assert_close(view.get_coms(), nominal)
    state.restore(saved)
    torch.testing.assert_close(view.get_coms(), shifted)
    base.event_manager.reset(env_ids=torch.tensor([0], device=base.device))
    torch.testing.assert_close(view.get_coms(), shifted)
    base.event_manager.reset(env_ids=torch.tensor([1], device=base.device))
    torch.testing.assert_close(view.get_coms(), nominal)
    print("SIM2REAL_CONTACT_CHECK_OK", flush=True)
finally:
    env.close()
    app.close()
