"""Best-effort flat-task continuation state; never a PhysX solver snapshot."""
import hashlib
import json
import random
from collections.abc import Mapping

import numpy as np
import torch

SCHEMA = 1
EPHEMERAL = {'extras', '_yertle_sole_vertex_mask'}


def cpu_tree(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().clone()
    if isinstance(value, Mapping):
        return {k: cpu_tree(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return type(value)(cpu_tree(v) for v in value)
    return value


def rng_state():
    return dict(python=random.getstate(), numpy=np.random.get_state(),
                torch=torch.get_rng_state(),
                cuda=torch.cuda.get_rng_state_all() if torch.cuda.is_available() else [])


def restore_rng(state):
    random.setstate(state['python'])
    np.random.set_state(state['numpy'])
    torch.set_rng_state(state['torch'].cpu())
    if state['cuda']:
        if len(state['cuda']) != torch.cuda.device_count():
            raise ValueError('Checkpoint CUDA device count differs')
        torch.cuda.set_rng_state_all([v.cpu() for v in state['cuda']])


def normalize_config(value):
    if callable(value):
        return value.__module__ + '.' + value.__qualname__
    if isinstance(value, dict):
        return {str(k): normalize_config(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [normalize_config(v) for v in value]
    if isinstance(value, slice):
        return ['slice', value.start, value.stop, value.step]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().tolist()
    raise TypeError(f'Unsupported checkpoint config value: {type(value)}')


def config_digest(cfg):
    return hashlib.sha256(json.dumps(normalize_config(cfg.to_dict()), sort_keys=True).encode()).hexdigest()


def tensor_container(value):
    if isinstance(value, torch.Tensor):
        return True
    return isinstance(value, (list, tuple, dict)) and bool(value) and all(
        tensor_container(v) for v in (value.values() if isinstance(value, dict) else value))


def capture_attrs(obj):
    return {name: cpu_tree(value) for name, value in vars(obj).items()
            if name not in EPHEMERAL and tensor_container(value)}


def restore_value(target, saved, path):
    if isinstance(saved, torch.Tensor):
        if not isinstance(target, torch.Tensor) or target.shape != saved.shape or target.dtype != saved.dtype:
            raise ValueError(f'Checkpoint tensor layout differs: {path}')
        target.copy_(saved.to(target.device))
    elif isinstance(saved, dict):
        if set(target) != set(saved):
            raise ValueError(f'Checkpoint dictionary layout differs: {path}')
        for key in saved:
            restore_value(target[key], saved[key], f'{path}.{key}')
    else:
        if len(target) != len(saved):
            raise ValueError(f'Checkpoint sequence layout differs: {path}')
        for i, value in enumerate(saved):
            restore_value(target[i], value, f'{path}.{i}')


def restore_attrs(obj, state):
    for key, saved in state.items():
        if key in EPHEMERAL:
            continue  # per-step log output, not persistent dynamics/history
        if not hasattr(obj, key):
            if key in ('reset_buf', 'reset_terminated', 'reset_time_outs', 'reward_buf') and isinstance(saved, torch.Tensor):
                setattr(obj, key, saved.to(obj.device).clone())
                continue
            raise ValueError(f'Missing checkpoint state attribute: {key}')
        restore_value(getattr(obj, key), saved, key)


class FlatEnvState:
    """Versioned adapter for the current non-recurrent flat Yertle task."""
    def __init__(self, env):
        self.env = env
        if env.cfg.scene.terrain.terrain_type != 'plane' or set(env.scene.articulations) != {'robot'}:
            raise ValueError('Environment continuation currently supports the single-robot flat task only')
        if getattr(env.cfg.observations.policy, 'history_length', 0):
            raise ValueError('Observation history continuation is not supported')
        if getattr(env.cfg.events, 'planar_force_impulse', None) is not None:
            raise ValueError('Environment continuation with active external force pulses is not supported yet')
        self.digest = config_digest(env.cfg)

    def objects(self):
        e = self.env
        result = {'env': e, 'actions': e.action_manager, 'rewards': e.reward_manager,
                  'events': e.event_manager, 'terminations': e.termination_manager}
        for name, term in e.command_manager._terms.items():
            result['command/' + name] = term
        for name, term in e.action_manager._terms.items():
            result['action/' + name] = term
        for name, cfg in zip(e.reward_manager._term_names, e.reward_manager._term_cfgs):
            if not isinstance(cfg.func, type) and hasattr(cfg.func, '__dict__'):
                result['reward/' + name] = cfg.func
        for mode, configs in e.event_manager._mode_term_cfgs.items():
            for i, cfg in enumerate(configs):
                if not isinstance(cfg.func, type) and hasattr(cfg.func, '__dict__'):
                    result[f'event/{mode}/{i}'] = cfg.func
        for name, sensor in e.scene.sensors.items():
            # Materialize latest sensor data before saving timestamps/history.
            result['sensor/' + name] = sensor
            result['sensor_data/' + name] = sensor.data
        return result

    def capture(self):
        e = self.env
        robot = e.scene['robot']
        view = robot.root_physx_view
        physics = {name: getattr(view, 'get_' + name)().cpu().clone()
                   for name in ('masses', 'inertias', 'coms', 'material_properties')}
        return dict(schema=SCHEMA, config_digest=self.digest, num_envs=e.num_envs,
                    joint_names=list(robot.joint_names), body_names=list(robot.body_names),
                    scene=cpu_tree(e.scene.get_state()), physics=physics,
                    objects={k: capture_attrs(v) for k, v in self.objects().items()},
                    common_step_counter=e.common_step_counter,
                    sim_step_counter=e._sim_step_counter,
                    robot_previous_joint_vel=cpu_tree(robot.data._previous_joint_vel),
                    joint_targets={k: cpu_tree(getattr(robot.data, k)) for k in
                                   ('joint_pos_target', 'joint_vel_target', 'joint_effort_target')})

    def restore(self, state):
        e = self.env
        robot = e.scene['robot']
        if (state['schema'] != SCHEMA or state['config_digest'] != self.digest or
            state['num_envs'] != e.num_envs or state['joint_names'] != list(robot.joint_names) or
            state['body_names'] != list(robot.body_names)):
            raise ValueError('Environment continuation configuration/model/layout mismatch; use --resume-env-state off for an intentional reset')
        objects = self.objects()
        if objects.keys() != state['objects'].keys():
            raise ValueError('Environment continuation term layout differs')
        # restore_to writes the saved poses/velocities, without simulating a step.
        scene = torch.utils._pytree.tree_map(lambda v: v.to(e.device) if isinstance(v, torch.Tensor) else v, state['scene'])
        e.scene.reset_to(scene)
        view = robot.root_physx_view
        ids = torch.arange(e.num_envs, dtype=torch.int32, device='cpu')
        for name, values in state['physics'].items():
            getattr(view, 'set_' + name)(values.cpu(), ids)
        robot.set_joint_position_target(state['joint_targets']['joint_pos_target'].to(e.device))
        robot.set_joint_velocity_target(state['joint_targets']['joint_vel_target'].to(e.device))
        robot.set_joint_effort_target(state['joint_targets']['joint_effort_target'].to(e.device))
        e.scene.write_data_to_sim()
        for key, saved in state['objects'].items():
            restore_attrs(objects[key], saved)
        robot.data._previous_joint_vel.copy_(state['robot_previous_joint_vel'].to(e.device))
        e.common_step_counter = state['common_step_counter']
        e._sim_step_counter = state['sim_step_counter']
        # Cached raw contact points belong to the new PhysX scene, not the old one.
        if hasattr(e, '_yertle_ground_contacts'):
            e._yertle_ground_contacts.last_step = None
        return e
