"""PPO runner with atomic, iteration-boundary environment checkpoints.

The rollout/update equations follow the installed RSL-RL OnPolicyRunner.
Scoped to single-process PPO without RND/recurrent policy.
"""
import os
import hashlib
import json
import signal
import time
from collections import deque

import torch
from rsl_rl.runners import OnPolicyRunner
from rsl_rl.utils import store_code_state

from .checkpoint_state import FlatEnvState, rng_state, restore_rng, normalize_config
from .attitude_metrics import AttitudeMetrics


class StatefulOnPolicyRunner(OnPolicyRunner):
    def __init__(self, *args, resume_env_state='auto', **kwargs):
        super().__init__(*args, **kwargs)
        if self.is_distributed or self.alg.rnd or getattr(self.alg.policy, 'is_recurrent', False):
            raise ValueError('Environment checkpoints support single-process non-recurrent PPO without RND')
        if self.cfg.get('logger', 'tensorboard') != 'tensorboard':
            raise ValueError('Atomic environment checkpoints currently require tensorboard logging')
        self.state_adapter = FlatEnvState(self.env.unwrapped)
        settings = {k: self.cfg[k] for k in ('policy', 'algorithm', 'num_steps_per_env')}
        self.runner_digest = hashlib.sha256(json.dumps(normalize_config(settings), sort_keys=True).encode()).hexdigest()
        self.resume_env_state = resume_env_state
        self._continuation = None
        self._boundary = None
        self._stop_requested = False

    def load(self, path, load_optimizer=True, map_location=None):
        infos = super().load(path, load_optimizer=load_optimizer, map_location=map_location)
        state = infos.get('yertle_continuation') if isinstance(infos, dict) else None
        if self.resume_env_state == 'off':
            print('RESUME_ENV_STATE disabled: environment reset explicitly requested', flush=True)
        elif state is None:
            if self.resume_env_state == 'required':
                raise ValueError('Checkpoint has no environment continuation state')
            print('RESUME_ENV_STATE legacy_checkpoint: environment initialized; no saved state available', flush=True)
        else:
            if not load_optimizer:
                raise ValueError('Environment continuation requires optimizer restoration')
            if state.get('runner_digest') != self.runner_digest:
                raise ValueError('PPO/model settings differ; use --resume-env-state off for an intentional reset')
            if state['next_iteration'] != self.current_learning_iteration + 1:
                raise ValueError('Model and environment checkpoint iteration mismatch')
            self.state_adapter.restore(state['environment'])
            self._continuation = state
            # A boundary checkpoint includes update k; the next update is k+1.
            self.current_learning_iteration = state['next_iteration']
            self.tot_timesteps = state['tot_timesteps']
            self.tot_time = state['tot_time']
            self.alg.learning_rate = state['learning_rate']
            print(f"RESUME_ENV_STATE restored next_iteration={self.current_learning_iteration} solver_exact=False", flush=True)
        return infos

    def save(self, path, infos=None):
        if self._boundary is None:
            raise RuntimeError('Environment checkpoint is only valid after a complete PPO update')
        # Capture before writing so a failed snapshot cannot leave a model-only checkpoint.
        state = dict(self._boundary, environment=self.state_adapter.capture(),
                     runner_digest=self.runner_digest,
                     next_iteration=self.current_learning_iteration + 1,
                     tot_timesteps=self.tot_timesteps, tot_time=self.tot_time,
                     learning_rate=self.alg.learning_rate)
        for key in ('obs', 'cur_reward_sum', 'cur_episode_length'):
            state[key] = state[key].detach().cpu().clone()
        state['rng'] = rng_state()
        metadata = dict(infos or {}, yertle_continuation=state)
        temporary = str(path) + '.tmp'
        try:
            super().save(temporary, infos=metadata)
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        print(f'CHECKPOINT_ENV_STATE saved={path} next_iteration={state["next_iteration"]} solver_exact=False', flush=True)

    def _request_stop(self, signum, frame):
        self._stop_requested = True
        print('TRAIN_STOP_REQUESTED: completing current PPO iteration before saving', flush=True)

    def learn(self, num_learning_iterations, init_at_random_ep_len=False):
        self._prepare_logging_writer()
        state = self._continuation
        if state is not None and init_at_random_ep_len:
            raise ValueError('Cannot randomize episode lengths after restoring environment state')
        if init_at_random_ep_len:
            self.env.episode_length_buf = torch.randint_like(self.env.episode_length_buf, high=int(self.env.max_episode_length))
        obs = state['obs'].to(self.device) if state else self.env.get_observations().to(self.device)
        self.train_mode()
        ep_infos = []
        rewbuffer = deque(state['rewbuffer'] if state else [], maxlen=100)
        lenbuffer = deque(state['lenbuffer'] if state else [], maxlen=100)
        cur_reward_sum = state['cur_reward_sum'].to(self.device) if state else torch.zeros(self.env.num_envs, device=self.device)
        cur_episode_length = state['cur_episode_length'].to(self.device) if state else torch.zeros(self.env.num_envs, device=self.device)
        if state:
            # Last operation before action sampling; setup must not consume saved RNG.
            restore_rng(state['rng'])
        old_handlers = {s: signal.signal(s, self._request_stop) for s in (signal.SIGINT, signal.SIGTERM)}
        start_iter = self.current_learning_iteration
        tot_iter = start_iter + num_learning_iterations
        attitude_metrics = AttitudeMetrics(self.env.unwrapped)
        attitude_metrics.install()
        try:
            for it in range(start_iter, tot_iter):
                attitude_metrics.reset()
                start = time.time()
                with torch.inference_mode():
                    for _ in range(self.num_steps_per_env):
                        actions = self.alg.act(obs)
                        obs, rewards, dones, extras = self.env.step(actions.to(self.env.device))
                        obs, rewards, dones = obs.to(self.device), rewards.to(self.device), dones.to(self.device)
                        self.alg.process_env_step(obs, rewards, dones, extras)
                        if self.log_dir is not None:
                            if 'episode' in extras: ep_infos.append(extras['episode'])
                            elif 'log' in extras: ep_infos.append(extras['log'])
                            cur_reward_sum += rewards
                            cur_episode_length += 1
                            ids = (dones > 0).nonzero(as_tuple=False)
                            rewbuffer.extend(cur_reward_sum[ids][:, 0].cpu().tolist())
                            lenbuffer.extend(cur_episode_length[ids][:, 0].cpu().tolist())
                            cur_reward_sum[ids] = 0
                            cur_episode_length[ids] = 0
                    collection_time = time.time() - start
                    self.alg.compute_returns(obs)
                start = time.time()
                loss_dict = self.alg.update()
                learn_time = time.time() - start
                self.current_learning_iteration = it
                self._boundary = dict(obs=obs,
                                      cur_reward_sum=cur_reward_sum,
                                      cur_episode_length=cur_episode_length,
                                      rewbuffer=list(rewbuffer), lenbuffer=list(lenbuffer))
                if self.log_dir is not None and not self.disable_logs:
                    self.log(locals())
                    for name, value in attitude_metrics.scalars().items():
                        self.writer.add_scalar(name, value, it)
                    if it % self.save_interval == 0 or self._stop_requested:
                        self.save(os.path.join(self.log_dir, f'model_{it}.pt'))
                ep_infos.clear()
                if it == start_iter:
                    store_code_state(self.log_dir, self.git_status_repos)
                if self._stop_requested:
                    break
            if self.log_dir is not None and not self.disable_logs:
                self.save(os.path.join(self.log_dir, f'model_{self.current_learning_iteration}.pt'))
        finally:
            attitude_metrics.close()
            for s, handler in old_handlers.items(): signal.signal(s, handler)
