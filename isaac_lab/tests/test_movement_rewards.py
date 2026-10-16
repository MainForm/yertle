"""Production reward tensor/configuration tests without launching Isaac Sim."""
import ast
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import torch

ROOT = Path(__file__).resolve().parents[1]
TREE = ast.parse((ROOT / 'flat_env_cfg_reward.py').read_text(encoding='utf-8'))
spec = importlib.util.spec_from_file_location('movement_test_phase', ROOT / 'phase_generator.py')
phase = importlib.util.module_from_spec(spec)
stub = ModuleType('isaaclab.managers')
stub.SceneEntityCfg = object
with patch.dict(sys.modules, {'isaaclab.managers': stub}):
    spec.loader.exec_module(phase)
names = {'feet_air_time_trot', '_command_relative_cauchy', 'track_lin_vel_xy_cauchy', 'track_ang_vel_z_cauchy'}
namespace = {'torch': torch, 'SceneEntityCfg': object, 'phase_generator': phase}
exec(compile(ast.Module(body=[n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name in names], type_ignores=[]), str(ROOT / 'flat_env_cfg_reward.py'), 'exec'), namespace)
linear = namespace['track_lin_vel_xy_cauchy']
air_time = namespace['feet_air_time_trot']


def env_for(commands, velocities=None, touches=None, air=None):
    commands = torch.tensor(commands, dtype=torch.float64)
    count = len(commands)
    velocities = torch.zeros_like(commands) if velocities is None else torch.tensor(velocities, dtype=torch.float64)
    sensor = NS(data=NS(last_air_time=torch.tensor(air if air is not None else [[.227] * 4] * count, dtype=torch.float64)),
                compute_first_contact=lambda dt: torch.tensor(touches if touches is not None else [[True]*4]*count),
                body_names=list(phase._FOOT_NAMES))
    sensor.data.net_forces_w_history = torch.zeros(count, 1, 4, 3, dtype=torch.float64)
    scene = {'robot': NS(data=NS(root_lin_vel_b=velocities))}
    class Scene(dict):
        pass
    scene = Scene(scene)
    scene.sensors = {'contact_forces': sensor}
    return NS(num_envs=count, step_dt=.02, episode_length_buf=torch.zeros(count, dtype=torch.long),
              command_manager=NS(get_command=lambda name: commands), scene=scene)


class MovementRewardTests(unittest.TestCase):
    def test_body_height_barrier_is_removed_from_registration_and_implementation(self):
        self.assertFalse(any(isinstance(n,ast.FunctionDef) and n.name=='body_height_barrier' for n in TREE.body))
        configure=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='configure_rewards')
        self.assertNotIn('body_height_barrier',ast.unparse(configure))

    def test_vx_only_standing_does_not_get_free_vy_score(self):
        env = env_for([[.1, 0, 0]])
        torch.testing.assert_close(2*linear(env, components=(0,)), torch.tensor([1.0], dtype=torch.float64))
        torch.testing.assert_close(2*linear(env), torch.tensor([1.5], dtype=torch.float64))

    def test_exact_vx_tracking_has_same_maximum_and_ignores_vy(self):
        env = env_for([[.1, 0, 0]], [[.1, 4, 0]])
        torch.testing.assert_close(linear(env, components=(0,)), torch.ones(1, dtype=torch.float64))
        self.assertLess(linear(env).item(), 1)

    def test_full_xy_behavior_and_cauchy_scaling_are_preserved(self):
        env = env_for([[.1, .2, 0], [-.1, -.2, 0], [0, 0, 0]])
        expected = torch.tensor([(.5+1/(1+(.2/.15)**2))/2]*2+[1], dtype=torch.float64)
        torch.testing.assert_close(linear(env), expected)

    def test_component_validation(self):
        for components in ((), (0,0), (2,), (-1,)):
            with self.subTest(components=components), self.assertRaises(ValueError):
                linear(env_for([[.1,0,0]]), components=components)

    def test_air_time_normal_trot_touchdown_is_positive(self):
        actual = air_time(env_for([[.1,0,0]], touches=[[True,False,False,True]]), 'base_velocity', NS(name='contact_forces',body_ids=slice(None)), .15)
        torch.testing.assert_close(actual, torch.tensor([.154],dtype=torch.float64))

    def test_air_time_requires_touchdown_and_selected_bodies(self):
        env = env_for([[.1,0,0]], touches=[[False,True,True,False]])
        actual = air_time(env,'base_velocity', NS(name='contact_forces',body_ids=[0,3]), .15)
        torch.testing.assert_close(actual, torch.zeros(1,dtype=torch.float64))

    def test_air_time_short_touchdowns_remain_negative(self):
        actual = air_time(env_for([[.1,0,0]], air=[[.1]*4]),'base_velocity',NS(name='contact_forces',body_ids=slice(None)),.15)
        torch.testing.assert_close(actual,torch.tensor([-.2],dtype=torch.float64))

    def test_air_time_command_gate_matches_phase_including_zero_and_turns(self):
        commands = [[0,0,0],[.05,0,0],[.051,0,0],[-.1,0,0],[0,0,.1],[0,0,.101],[0,.06,0]]
        env=env_for(commands)
        actual=air_time(env,'base_velocity',NS(name='contact_forces',body_ids=slice(None)),.15)
        expected=torch.tensor([0,0,.308,.308,0,.308,.308],dtype=torch.float64)
        torch.testing.assert_close(actual,expected)

    def test_continuous_contact_is_net_negative_in_both_half_cycles(self):
        env=env_for([[.1,0,0]])
        env.scene.sensors['contact_forces'].data.net_forces_w_history[:,:,:,2]=2
        sensor_cfg=NS(name='contact_forces')
        for step in (0,12):
            env.episode_length_buf.fill_(step)
            score=.2*phase.phase_contact_schedule(env,sensor_cfg)-.4*phase.phase_unscheduled_contact_penalty(env,sensor_cfg)
            torch.testing.assert_close(score,torch.tensor([-.1]))

    def test_correct_phase_contacts_avoid_unscheduled_penalty(self):
        env=env_for([[.1,0,0]])
        env.scene.sensors['contact_forces'].data.net_forces_w_history[:,:, [0,3],2]=2
        torch.testing.assert_close(phase.phase_unscheduled_contact_penalty(env,NS(name='contact_forces')), torch.zeros(1))
        torch.testing.assert_close(phase.phase_contact_schedule(env,NS(name='contact_forces')), torch.ones(1))

    def test_real_registration(self):
        configure=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='configure_rewards')
        selected=[n for n in configure.body if isinstance(n,ast.Assign) and any('rewards.feet_air_time' in ast.unparse(t) or ast.unparse(t) in ('rewards.track_lin_vel_xy_cauchy','rewards.track_ang_vel_z_cauchy','rewards.phase_unscheduled_contact_penalty') for t in n.targets)]
        rewards=NS(feet_air_time=NS(params={'sensor_cfg':NS()}))
        ns=dict(namespace,rewards=rewards,FEET='.*_shin',CONTACT_SENSOR=NS(),RewTerm=lambda **kw:NS(**kw))
        exec(compile(ast.Module(body=selected,type_ignores=[]),'<registration>','exec'),ns)
        self.assertIs(rewards.feet_air_time.func,air_time)
        self.assertEqual(rewards.feet_air_time.params['threshold'],.15)
        self.assertEqual(rewards.feet_air_time.weight,1.0)
        self.assertEqual(rewards.track_lin_vel_xy_cauchy.params['components'],(0,1))
        self.assertEqual(rewards.track_lin_vel_xy_cauchy.weight,2.0)
        self.assertEqual(rewards.track_ang_vel_z_cauchy.weight,.75)
        clearance=next(n for n in configure.body if isinstance(n,ast.Assign) and any(ast.unparse(t)=='rewards.phase_swing_clearance' for t in n.targets))
        self.assertEqual(ast.literal_eval(next(k.value for k in clearance.value.keywords if k.arg=='weight')),-.08)
        self.assertEqual(rewards.phase_unscheduled_contact_penalty.weight,-.4)

    def test_train_vx_only_selects_vx_only_for_flat_not_rough(self):
        tree=ast.parse((ROOT/'train.py').read_text(encoding='utf-8'))
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        block=next(n for n in main.body if isinstance(n,ast.If) and ast.unparse(n.test)=='args_cli.vx_only')
        for task,enabled,expected in [('flat',True,(0,)),('flat',False,(0,1)),('rough',True,(0,1))]:
            cfg=NS(commands=NS(base_velocity=NS(ranges=NS())),rewards=NS(track_lin_vel_xy_cauchy=NS(params={'components':(0,1)})))
            exec(compile(ast.Module(body=[block],type_ignores=[]),'<train vx-only>','exec'),{'env_cfg':cfg,'args_cli':NS(task=task,vx_only=enabled)})
            self.assertEqual(cfg.rewards.track_lin_vel_xy_cauchy.params['components'],expected)


if __name__ == '__main__':
    unittest.main()
