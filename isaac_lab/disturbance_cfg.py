"""Stateful disturbance events for Yertle robustness training."""

import math
from collections.abc import Sequence

import torch

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ManagerTermBase, SceneEntityCfg


class TimedPlanarForceImpulse(ManagerTermBase):
    """Apply randomized horizontal force pulses for an exact control-step duration."""

    DEFAULT_ENABLED = False
    DEFAULT_FORCE_BODY_WEIGHT = 1.0
    DEFAULT_DURATION_S = 0.10
    DEFAULT_INTERVAL_S = (3.0, 6.0)
    DEFAULT_PERTURBED_ENV_FRACTION = 0.30
    DEFAULT_WARMUP_S = 1.0
    DEFAULT_RECOVERY_WINDOW_S = 2.0
    DEFAULT_RECOVERY_TILT_THRESHOLD_RAD = 0.25
    DEFAULT_RECOVERY_VELOCITY_ERROR_THRESHOLD_MPS = 0.20
    DEFAULT_RECOVERY_STABLE_STEPS = 4

    @classmethod
    def event_cfg(
        cls,
        control_dt: float,
        enabled: bool = DEFAULT_ENABLED,
        force_body_weight: float = DEFAULT_FORCE_BODY_WEIGHT,
        duration_s: float = DEFAULT_DURATION_S,
        interval_s: tuple[float, float] = DEFAULT_INTERVAL_S,
        perturbed_env_fraction: float = DEFAULT_PERTURBED_ENV_FRACTION,
        terrain_type_names: tuple[str, ...] = (),
    ) -> EventTerm | None:
        """Build the Isaac Lab event configuration using this term's defaults."""
        if not enabled:
            return None
        return EventTerm(
            func=cls,
            mode="interval",
            interval_range_s=(control_dt, control_dt),
            is_global_time=False,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=["base_link"]),
                "force_fraction_of_weight": force_body_weight,
                "duration_s": duration_s,
                "push_interval_s": interval_s,
                "warmup_s": cls.DEFAULT_WARMUP_S,
                "perturbed_env_fraction": perturbed_env_fraction,
                "recovery_window_s": cls.DEFAULT_RECOVERY_WINDOW_S,
                "recovery_tilt_threshold_rad": cls.DEFAULT_RECOVERY_TILT_THRESHOLD_RAD,
                "recovery_velocity_error_threshold_mps": cls.DEFAULT_RECOVERY_VELOCITY_ERROR_THRESHOLD_MPS,
                "recovery_stable_steps": cls.DEFAULT_RECOVERY_STABLE_STEPS,
                "terrain_type_names": terrain_type_names,
            },
        )

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._active_force_w = torch.zeros(env.num_envs, 3, device=env.device)
        self._remaining_steps = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self._cooldown_steps = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self._perturbed_env = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

        # Diagnostics are deliberately separate from reward terms. They
        # summarize the state after each force pulse at episode reset.
        self._recovery_active = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        self._recovery_age_steps = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self._recovery_stable_steps = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self._recovery_peak_tilt = torch.zeros(env.num_envs, device=env.device)
        self._episode_recovery_attempts = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self._episode_recovery_successes = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self._episode_recovery_time_sum = torch.zeros(env.num_envs, device=env.device)
        self._episode_peak_tilt_sum = torch.zeros(env.num_envs, device=env.device)
        self._terrain_type_names = tuple(cfg.params.get("terrain_type_names", ()))

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        else:
            env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        if env_ids.numel() == 0:
            return

        # An episode ending during a pending recovery counts as a failure.
        pending_ids = env_ids[self._recovery_active[env_ids]]
        if pending_ids.numel() > 0:
            self._finish_recovery(pending_ids, success=False)

        # EventManager.reset() runs after extras["log"] is recreated, so these
        # become regular per-iteration Episode_Disturbance metrics.
        attempts = self._episode_recovery_attempts[env_ids]
        attempted_ids = env_ids[attempts > 0]
        if attempted_ids.numel() > 0:
            total_attempts = self._episode_recovery_attempts[attempted_ids].sum().float()
            total_successes = self._episode_recovery_successes[attempted_ids].sum().float()
            self._env.extras["log"]["Episode_Disturbance/recovery_success_rate"] = total_successes / total_attempts
            self._env.extras["log"]["Episode_Disturbance/recovery_time_s"] = (
                self._episode_recovery_time_sum[attempted_ids].sum() / total_successes.clamp_min(1.0)
            )
            self._env.extras["log"]["Episode_Disturbance/peak_tilt_rad"] = (
                self._episode_peak_tilt_sum[attempted_ids].sum() / total_attempts
            )
            self._log_terrain_recovery_metrics(attempted_ids)

        fraction = self.cfg.params["perturbed_env_fraction"]
        self._perturbed_env[env_ids] = torch.rand(env_ids.numel(), device=self.device) < fraction
        self._active_force_w[env_ids] = 0.0
        self._remaining_steps[env_ids] = 0
        self._cooldown_steps[env_ids] = self._sample_steps(env_ids.numel(), self.cfg.params["push_interval_s"])
        self._recovery_active[env_ids] = False
        self._recovery_age_steps[env_ids] = 0
        self._recovery_stable_steps[env_ids] = 0
        self._recovery_peak_tilt[env_ids] = 0.0
        self._episode_recovery_attempts[env_ids] = 0
        self._episode_recovery_successes[env_ids] = 0
        self._episode_recovery_time_sum[env_ids] = 0.0
        self._episode_peak_tilt_sum[env_ids] = 0.0

    def _log_terrain_recovery_metrics(self, attempted_ids: torch.Tensor) -> None:
        """Emit recovery scalars by terrain type at reset, not each control step."""
        if not self._terrain_type_names or not hasattr(self._env.scene.terrain, "terrain_types"):
            return
        terrain_ids = self._env.scene.terrain.terrain_types[attempted_ids]
        count = len(self._terrain_type_names)
        attempts = self._episode_recovery_attempts[attempted_ids].float()
        successes = self._episode_recovery_successes[attempted_ids].float()
        recovery_time = self._episode_recovery_time_sum[attempted_ids]
        peak_tilt = self._episode_peak_tilt_sum[attempted_ids]
        attempts_by_type = torch.zeros(count, device=self.device).scatter_add_(0, terrain_ids, attempts)
        successes_by_type = torch.zeros(count, device=self.device).scatter_add_(0, terrain_ids, successes)
        time_by_type = torch.zeros(count, device=self.device).scatter_add_(0, terrain_ids, recovery_time)
        tilt_by_type = torch.zeros(count, device=self.device).scatter_add_(0, terrain_ids, peak_tilt)
        for index in (attempts_by_type > 0).nonzero(as_tuple=False).flatten().tolist():
            attempts = attempts_by_type[index]
            successes = successes_by_type[index]
            prefix = f"Terrain/{self._terrain_type_names[index]}"
            self._env.extras["log"][f"{prefix}/recovery_success_rate"] = successes / attempts
            self._env.extras["log"][f"{prefix}/recovery_time_s"] = time_by_type[index] / successes.clamp_min(1.0)
            self._env.extras["log"][f"{prefix}/peak_tilt_rad"] = tilt_by_type[index] / attempts

    def __call__(
        self,
        env,
        env_ids: torch.Tensor,
        asset_cfg: SceneEntityCfg,
        force_fraction_of_weight: float,
        duration_s: float,
        push_interval_s: tuple[float, float],
        warmup_s: float,
        perturbed_env_fraction: float,
        recovery_window_s: float,
        recovery_tilt_threshold_rad: float,
        recovery_velocity_error_threshold_mps: float,
        recovery_stable_steps: int,
        terrain_type_names: tuple[str, ...] = (),
    ) -> None:
        del (
            perturbed_env_fraction,
            recovery_window_s,
            recovery_tilt_threshold_rad,
            recovery_velocity_error_threshold_mps,
            recovery_stable_steps,
            terrain_type_names,
        )  # Stored in cfg.params; sampling remains episode-local.
        if env_ids.numel() == 0:
            return

        duration_steps = max(1, round(duration_s / env.step_dt))
        warmup_steps = math.ceil(warmup_s / env.step_dt)
        self._update_recovery(env, env_ids, asset_cfg)

        # The force set at this post-step event is applied during the next control step.
        active = self._remaining_steps[env_ids] > 0
        active_ids = env_ids[active]
        if active_ids.numel() > 0:
            self._remaining_steps[active_ids] -= 1
            ended_ids = active_ids[self._remaining_steps[active_ids] == 0]
            if ended_ids.numel() > 0:
                self._active_force_w[ended_ids] = 0.0
                self._cooldown_steps[ended_ids] = self._sample_steps(ended_ids.numel(), push_interval_s)

        idle_ids = env_ids[self._remaining_steps[env_ids] == 0]
        if idle_ids.numel() > 0:
            self._cooldown_steps[idle_ids] -= 1

        eligible = (
            self._perturbed_env[env_ids]
            & (self._remaining_steps[env_ids] == 0)
            & (self._cooldown_steps[env_ids] <= 0)
            & (env.episode_length_buf[env_ids] >= warmup_steps)
        )
        trigger_ids = env_ids[eligible]
        if trigger_ids.numel() > 0:
            asset = env.scene[asset_cfg.name]
            # PhysX exposes masses on CPU while event indices live on the
            # simulation device, so index on CPU and move only the total back.
            total_mass = asset.root_physx_view.get_masses()[trigger_ids.cpu()].sum(dim=1).to(self.device)
            magnitude = force_fraction_of_weight * total_mass * 9.81
            direction = torch.rand(trigger_ids.numel(), device=self.device) * (2.0 * math.pi)
            self._active_force_w[trigger_ids, 0] = magnitude * torch.cos(direction)
            self._active_force_w[trigger_ids, 1] = magnitude * torch.sin(direction)
            self._active_force_w[trigger_ids, 2] = 0.0
            self._remaining_steps[trigger_ids] = duration_steps
            self._recovery_active[trigger_ids] = True
            self._recovery_age_steps[trigger_ids] = 0
            self._recovery_stable_steps[trigger_ids] = 0
            self._recovery_peak_tilt[trigger_ids] = 0.0

        asset = env.scene[asset_cfg.name]
        forces = self._active_force_w[env_ids].unsqueeze(1)
        torques = torch.zeros_like(forces)
        asset.permanent_wrench_composer.set_forces_and_torques(
            forces=forces,
            torques=torques,
            body_ids=asset_cfg.body_ids,
            env_ids=env_ids,
            is_global=True,
        )

    def _sample_steps(self, count: int, interval_s: tuple[float, float]) -> torch.Tensor:
        durations = torch.empty(count, device=self.device).uniform_(*interval_s)
        return torch.clamp(torch.round(durations / self._env.step_dt), min=1).to(torch.long)

    def _update_recovery(self, env, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg) -> None:
        """Track recovery after a pulse ends, without changing reward."""
        candidate_ids = env_ids[self._recovery_active[env_ids] & (self._remaining_steps[env_ids] == 0)]
        if candidate_ids.numel() == 0:
            return

        asset = env.scene[asset_cfg.name]
        gravity_xy = torch.linalg.vector_norm(asset.data.projected_gravity_b[candidate_ids, :2], dim=1)
        tilt_rad = torch.asin(gravity_xy.clamp(max=1.0))
        self._recovery_peak_tilt[candidate_ids] = torch.maximum(self._recovery_peak_tilt[candidate_ids], tilt_rad)
        self._recovery_age_steps[candidate_ids] += 1

        command = env.command_manager.get_command("base_velocity")[candidate_ids]
        velocity_error = torch.linalg.vector_norm(asset.data.root_lin_vel_b[candidate_ids, :2] - command[:, :2], dim=1)
        stable = (tilt_rad <= self.cfg.params["recovery_tilt_threshold_rad"]) & (
            velocity_error <= self.cfg.params["recovery_velocity_error_threshold_mps"]
        )
        self._recovery_stable_steps[candidate_ids] = torch.where(
            stable,
            self._recovery_stable_steps[candidate_ids] + 1,
            torch.zeros_like(self._recovery_stable_steps[candidate_ids]),
        )

        success_ids = candidate_ids[
            self._recovery_stable_steps[candidate_ids] >= self.cfg.params["recovery_stable_steps"]
        ]
        if success_ids.numel() > 0:
            self._finish_recovery(success_ids, success=True)

        timeout_steps = math.ceil(self.cfg.params["recovery_window_s"] / env.step_dt)
        timeout_ids = candidate_ids[
            self._recovery_active[candidate_ids] & (self._recovery_age_steps[candidate_ids] >= timeout_steps)
        ]
        if timeout_ids.numel() > 0:
            self._finish_recovery(timeout_ids, success=False)

    def _finish_recovery(self, env_ids: torch.Tensor, success: bool) -> None:
        """Commit one recovery attempt to the current episode."""
        self._episode_recovery_attempts[env_ids] += 1
        self._episode_peak_tilt_sum[env_ids] += self._recovery_peak_tilt[env_ids]
        if success:
            self._episode_recovery_successes[env_ids] += 1
            self._episode_recovery_time_sum[env_ids] += self._recovery_age_steps[env_ids] * self._env.step_dt
        self._recovery_active[env_ids] = False
