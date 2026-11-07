"""Freeze committed source and a deliberate PPO transfer; never rent a Pod."""
from pathlib import Path
import ast
import json
import tarfile
import time
import uuid
import run


def prepare():
    here = Path(__file__).resolve().parent
    request_path = here / '.state/v62212_committed_resume_request.json'
    if request_path.exists():
        raise RuntimeError('Request already exists; inspect it before preparing another job')
    checkpoint = run.REPO / 'isaac_lab/runs/yertle_flat/2026-11-04_10-10-11/model_22780.pt'
    expected_hash = '8d97849f3deae582e19f6cda045101bf70defbe7b616834c3db0d13e58adfa68'
    assert run.digest(checkpoint) == expected_hash, 'v6.22.12 checkpoint changed'
    head = run.output(['git', '-C', run.REPO, 'rev-parse', 'HEAD'])
    config = json.loads((here / 'config.json').read_text(encoding='utf-8'))
    config.update(iterations=3000, num_envs=1024, profile='isaac_lab/configs/runpod_v62212_transfer.json',
                  transfer_entry='isaac_lab/runpod_committed_transfer.py',
                  source_iteration=22780, next_iteration=22781, target_last_iteration=25780,
                  max_gpu_hourly_usd=.25, max_planned_usd=1.70, max_seconds=36000,
                  persistent_disk_gb=10, cloud='COMMUNITY')
    tag = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '-' + uuid.uuid4().hex[:6]
    job = here / '.state' / tag
    job.mkdir(parents=True)
    clean = job / 'committed-source'
    run.execute(['git', 'clone', '--local', '--no-hardlinks', '--no-checkout', str(run.REPO), str(clean)])
    run.execute(['git', '-C', clean, 'checkout', '--detach', head])
    original = run.REPO
    try:
        run.REPO = clean
        spec, payload = run.snapshot(config, job, include_dirty=False, resume=checkpoint)
    finally:
        run.REPO = original
    # Use committed explicit motor implementation, excluding working-tree experiments.
    args = json.loads((original / 'isaac_lab/runs/v6_22_12_current3x_height03_resume19911/run_profile.json').read_text())['train_args']
    for flag in ('--velocity-limit-sim', '--effort-limit-sim'):
        idx = args.index(flag)
        del args[idx:idx+2]
    args[args.index('--max_iterations') + 1] = '3000'
    args.append('--explicit-linked-motor')
    profile = dict(source_version='v6.22.12', source_iteration=22780, fresh=False,
                   environment_reset=True, committed_source=head, train_args=args)
    entry = '''"""Intentional model/optimizer transfer to changed committed physics/rewards."""
from pathlib import Path
import json
import runpy
import sys

root = Path('/workspace/yertle-job')
spec = json.loads((root / 'resolved.json').read_text())
source = Path(spec['source'])
profile = json.loads((source / spec['profile']).read_text())
sys.path[:0] = [str(source), str(source / 'isaac_lab')]
sys.argv = [str(source / 'isaac_lab/train.py')] + profile['train_args'] + [
    '--checkpoint', str(root / 'resume.pt'), '--resume-env-state', 'off']
ns = runpy.run_path(sys.argv[0], run_name='runpod_committed_transfer_entry')
Original = ns['StatefulOnPolicyRunner']

class TransferRunner(Original):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        cfg = self.env.unwrapped.cfg
        drive = cfg.scene.robot.actuators['legs']
        assert cfg.sim.dt == .001 and cfg.decimation == 20
        assert cfg.scene.contact_forces.history_length == 11
        assert drive.motor_torque_limit == 1.024 and drive.motor_speed_limit == 4.6
        assert drive.kp_pwm == 7.0 and drive.ki_pwm == 3.0 and drive.kd_pwm == .13
        assert cfg.rewards.estimated_total_current_excess.weight == -.3
        assert cfg.rewards.estimated_total_current_excess.params['threshold_a'] == 8.
        assert cfg.rewards.estimated_leg_current_excess.weight == -.3
        assert cfg.rewards.estimated_leg_current_excess.params['threshold_a'] == 3.
        assert cfg.rewards.lin_vel_xy_tracking_error_cauchy.weight == -1.
        assert cfg.rewards.ang_vel_z_tracking_error_cauchy.weight == -.1
        print('COMMITTED_SETTINGS_VERIFIED ' + spec['commit'], flush=True)

    def load(self, path, load_optimizer=True, map_location=None):
        assert load_optimizer and self.resume_env_state == 'off'
        infos = super().load(path, load_optimizer=load_optimizer, map_location=map_location)
        state = infos['yertle_continuation']
        assert self.current_learning_iteration == spec['source_iteration']
        assert state['next_iteration'] == spec['next_iteration']
        self.current_learning_iteration = state['next_iteration']
        self.tot_timesteps = state['tot_timesteps']
        self.tot_time = state['tot_time']
        self.alg.learning_rate = state['learning_rate']
        for group in self.alg.optimizer.param_groups:
            group['lr'] = state['learning_rate']
        print('TRANSFER_STATE policy_critic_optimizer_lr=restored next_iteration=' +
              str(self.current_learning_iteration) + ' environment_rng_reset=True', flush=True)
        return infos

ns['main'].__globals__['StatefulOnPolicyRunner'] = TransferRunner
ns['main']()
'''
    ast.parse(entry)
    extras = {config['profile']: json.dumps(profile, indent=2), config['transfer_entry']: entry}
    for name, body in extras.items():
        assert not (clean / name).exists(), 'Never overwrite committed source'
        target = job / 'extra' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(body, encoding='utf-8', newline='\n')
        spec['extra_files'][name] = run.digest(target)
    run.save(job / 'job.json', spec)
    remote_path = job / 'remote_job.py'
    remote = remote_path.read_text(encoding='utf-8')
    old = "    profile = json.loads((source / spec['profile']).read_text())\n"
    replacement = "    if spec.get('transfer_entry'):\n        runpy.run_path(str(source / spec['transfer_entry']), run_name='__main__')\n        return\n" + old
    assert remote.count(old) == 1
    remote = remote.replace(old, replacement)
    old_validation = "    assert spec.get('resume_checkpoint') or steps[0] == 0, 'Fresh run did not start at zero'"
    assert remote.count(old_validation) == 1
    remote = remote.replace(old_validation, old_validation + "\n    assert steps[0] == spec['next_iteration'] and steps[-1] == spec['target_last_iteration'], 'Transfer global iteration boundary mismatch'")
    ast.parse(remote)
    remote_path.write_text(remote, encoding='utf-8', newline='\n')
    replay = run.verify_snapshot(job)
    for file in replay.rglob('*.py'):
        if '.git' not in file.parts:
            ast.parse(file.read_text(encoding='utf-8-sig'), filename=str(file))
    with tarfile.open(payload, 'w:gz') as archive:
        for name in ('source.bundle', 'working.patch', 'extra', 'job.json', 'resume.pt', 'remote_job.py', 'bootstrap.sh'):
            archive.add(job / name, arcname=name)
    state = dict(phase='waiting_capacity', job_dir=str(job), commit=head,
                 pod_name='yertle-resume-' + tag, storage='host-persistent', created_at=time.time())
    run.save(job / 'state.json', state)
    request = dict(job_id=tag, job_dir=str(job), payload=str(payload), payload_sha256=run.digest(payload),
                   commit=head, checkpoint=str(checkpoint), checkpoint_sha256=expected_hash,
                   source_version='v6.22.12', next_iteration=22781, target_last_iteration=25780,
                   iterations=3000, num_envs=1024, max_gpu_hourly_usd=.25, max_planned_usd=1.70,
                   min_cuda='12.8', min_vram_gb=16, cloud='COMMUNITY', poll_seconds=60,
                   pod_name=state['pod_name'], status='waiting_capacity', environment_reset=True)
    run.save(request_path, request)
    print(json.dumps(request, indent=2))


if __name__ == '__main__':
    prepare()
