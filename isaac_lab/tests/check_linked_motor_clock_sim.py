"""Run inside Isaac Sim: verify held PWM across real physics steps."""
import argparse
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
import isaac_lab
from isaac_lab.flat_env_cfg import YertleFlatEnvCfg
from isaac_lab.linked_motor_training import configure_linked_motor_actuator
from isaac_lab.signed_joint_reference import configure_signed_joint_reference

cfg = YertleFlatEnvCfg()
cfg.scene.num_envs = 2
cfg.sim.device = args.device
cfg.terminations.base_contact = None
cfg.episode_length_s = 1000.0
configure_signed_joint_reference(cfg)
configure_linked_motor_actuator(cfg)
env = gym.make('Isaac-Velocity-Flat-Yertle-v0', cfg=cfg)
env.reset()
drive = env.unwrapped.scene['robot'].actuators['legs']
calls = []
original = drive.compute
def record(*values, **kwargs):
    index = drive._sim.current_time_step_index
    previous = drive.motor_pwm.clone()
    pending = drive._refresh_pending
    result = original(*values, **kwargs)
    changed = bool(torch.any(drive.motor_pwm != previous))
    calls.append((index, pending, changed))
    return result
drive.compute = record
with torch.inference_mode():
    env.step(torch.zeros(2, 12, device=cfg.sim.device))
    env.step(torch.full((2, 12), 0.5, device=cfg.sim.device))
assert len(calls) == 40, calls
assert all(b[0] == a[0] + 1 for a, b in zip(calls, calls[1:])), calls
origin = calls[0][0]
assert all(not changed for index, pending, changed in calls if (index - origin) % 20 != 0 and not pending), calls
assert all(calls[i][2] for i in (0, 20)), calls
print(f'LINKED_CLOCK_OK calls={len(calls)} first={origin} last={calls[-1][0]} changed={[i for i, _, c in calls if c]}', flush=True)
env.close()
app.close()
