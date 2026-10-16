"""Read-only, pre-auto-reset roll/pitch statistics for each PPO iteration."""
import torch


def roll_pitch_degrees(gravity):
    """Right-handed body Euler roll/pitch from projected unit gravity.

    Roll spans +/-180 deg, pitch +/-90 deg. This does not alter observations
    or the termination formula (which uses absolute angles).
    """
    roll=torch.atan2(-gravity[:,1],-gravity[:,2])
    pitch=torch.atan2(gravity[:,0],torch.linalg.vector_norm(gravity[:,1:],dim=1))
    return torch.rad2deg(torch.stack((roll,pitch),dim=-1))


class AttitudeMetrics:
    """Sample all envs after termination computation, BEFORE automatic resets.

    Running sums remain on-device; only iteration-end logging synchronizes.
    Fall-only means are omitted when no fall occurred (not invented zeroes).
    Signed means can cancel, so absolute means/maxima are logged separately.
    No RNG, reward, termination flags, or environment state are modified.
    """
    def __init__(self,env):
        self.env=env
        self.manager=env.termination_manager
        self.original_compute=None
        self.reset()

    def reset(self):
        device=self.env.scene['robot'].data.projected_gravity_b.device
        self.sums=torch.zeros(4,device=device)
        self.max_abs=torch.zeros(2,device=device)
        self.fall_sums=torch.zeros(4,device=device)
        self.fall_count=torch.zeros((),device=device)
        self.sample_count=0

    def install(self):
        if self.original_compute is not None:raise RuntimeError('Attitude metrics already installed')
        self.had_override='compute' in vars(self.manager)
        self.original_compute=self.manager.compute
        def compute():
            result=self.original_compute()
            angles=roll_pitch_degrees(self.env.scene['robot'].data.projected_gravity_b)
            values=torch.cat((angles,angles.abs()),dim=1)
            self.sums.add_(values.sum(0));self.sample_count+=angles.shape[0]
            self.max_abs=torch.maximum(self.max_abs,angles.abs().amax(0))
            name='body_tilt_and_nonsole_contact'
            if name not in self.manager.active_terms and 'base_contact' in self.manager.active_terms:
                name='base_contact'
            if name in self.manager.active_terms:
                falls=self.manager.get_term(name)
                self.fall_sums.add_((values*falls[:,None]).sum(0))
                self.fall_count.add_(falls.sum())
            return result
        self.manager.compute=compute

    def close(self):
        if self.original_compute is not None:
            if self.had_override:self.manager.compute=self.original_compute
            else:delattr(self.manager,'compute')
            self.original_compute=None

    def scalars(self):
        if not self.sample_count:return {}
        means=(self.sums/self.sample_count).tolist()
        maximum=self.max_abs.tolist();count=int(self.fall_count.item())
        out={'Metrics/body_attitude/'+key:value for key,value in zip(
            ('roll_mean_deg','pitch_mean_deg','roll_abs_mean_deg','pitch_abs_mean_deg'),means)}
        out.update({'Metrics/body_attitude/roll_abs_max_deg':maximum[0],
                    'Metrics/body_attitude/pitch_abs_max_deg':maximum[1],
                    'Metrics/body_attitude/fall_count':count})
        if count:
            out.update({'Metrics/body_attitude/'+key:value for key,value in zip(
                ('fall_roll_mean_deg','fall_pitch_mean_deg','fall_roll_abs_mean_deg','fall_pitch_abs_mean_deg'),
                (self.fall_sums/count).tolist())})
        return out
