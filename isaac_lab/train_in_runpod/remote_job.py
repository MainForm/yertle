"""Pod-side setup/training supervisor. No account-wide API key is sent here."""
from pathlib import Path
import hashlib
import json
import os
import runpy
import signal
import subprocess
import sys
import tarfile
import time
import urllib.request

ROOT = Path('/workspace/yertle-job')


def runtime_environment(spec):
    # sshd need not forward Docker environment variables to remote sessions.
    # Apply the non-secret runtime contract explicitly to EVERY subprocess.
    return dict(os.environ, ACCEPT_EULA='Y', PRIVACY_CONSENT='Y',
                OMNI_KIT_ACCEPT_EULA='YES', OMNI_KIT_ALLOW_ROOT='1',
                PYTHONUNBUFFERED='1', YERTLE_SIM_VERSION=spec['isaac_sim_version'],
                YERTLE_LAB_VERSION=spec['isaac_lab_version'],
                YERTLE_TORCH_VERSION=spec['torch_version'], YERTLE_RSL_VERSION=spec['rsl_rl_version'])


def command(args, **kwargs):
    print('EXEC', args, flush=True)
    return subprocess.run(args, check=True, **kwargs)


def save_status(**fields):
    target = ROOT / 'status.json'
    tmp = target.with_suffix('.tmp')
    tmp.write_text(json.dumps(fields, indent=2))
    tmp.replace(target)


def prepare():
    spec = json.loads((ROOT / 'job.json').read_text())
    source = ROOT / 'source'
    command(['git', 'clone', '--filter=blob:none', '--branch', spec['branch'], spec['repository'], str(source)])
    command(['git', '-C', str(source), 'fetch', str(ROOT / 'source.bundle'), 'HEAD'])
    command(['git', '-C', str(source), 'checkout', '--detach', 'FETCH_HEAD'])
    head = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    assert head == spec['commit'], (head, spec['commit'])
    patch = ROOT / 'working.patch'
    if patch.stat().st_size:
        command(['git', '-C', str(source), 'apply', '--check', str(patch)])
        command(['git', '-C', str(source), 'apply', str(patch)])
    # Only explicitly inventoried, safe untracked source files, never credentials/runs.
    for relative, expected in spec['extra_files'].items():
        src = ROOT / 'extra' / relative
        assert hashlib.sha256(src.read_bytes()).hexdigest() == expected
        dst = source / relative
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(src.read_bytes())
    for relative, expected in spec['source_hashes'].items():
        raw = (source / relative).read_bytes()
        raw = raw if b'\0' in raw else raw.replace(b'\r\n', b'\n')
        assert hashlib.sha256(raw).hexdigest() == expected, relative
    env = runtime_environment(spec)
    command(['bash', str(ROOT / 'bootstrap.sh')], env=env)
    python = '/opt/yertle-venv/bin/python'
    usd = source / 'simulation/usd/yertle.usd'
    usd.parent.mkdir(parents=True, exist_ok=True)
    command([python, '/opt/IsaacLab/scripts/tools/convert_urdf.py', str(source / 'simulation/yertle.urdf'),
             str(usd), '--merge-joints', '--headless'], cwd=source, env=env)
    spec['source'] = str(source)
    (ROOT / 'resolved.json').write_text(json.dumps(spec, indent=2))


def train():
    spec = json.loads((ROOT / 'resolved.json').read_text())
    source = Path(spec['source'])
    os.chdir(source)
    sys.path[:0] = [str(source), str(source / 'isaac_lab')]
    profile = json.loads((source / spec['profile']).read_text())
    args = profile['train_args'][:]
    assert '--checkpoint' not in args, 'Profile must be fresh; resume uses explicit --checkpoint on controller.'
    for flag, value in [('--max_iterations', spec['iterations']), ('--num_envs', spec['num_envs'])]:
        args[args.index(flag) + 1] = str(value)
    if spec.get('resume_checkpoint'):
        args += ['--checkpoint', str(ROOT / 'resume.pt')]
    print('YERTLE_REMOTE_TRAIN', json.dumps({'commit': spec['commit'], 'args': args}), flush=True)
    sys.argv = [str(source / 'isaac_lab/train.py')] + args
    runpy.run_path(sys.argv[0], run_name='__main__')


def validate_training():
    """Do not equate an exit code or a lone checkpoint with completed PPO updates."""
    import torch
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    spec = json.loads((ROOT / 'resolved.json').read_text())
    folders = list((Path(spec['source']) / 'isaac_lab/runs/yertle_flat').glob('*'))
    folder = max((p for p in folders if p.is_dir()), key=lambda p: p.name)
    checkpoints = list(folder.glob('model_*.pt'))
    latest = max(checkpoints, key=lambda p: int(p.stem.split('_')[1]))
    state = torch.load(latest, map_location='cpu', weights_only=False)
    assert 'model_state_dict' in state
    event = EventAccumulator(str(folder), size_guidance={'scalars': 0}).Reload()
    tags = event.Tags()['scalars']
    reward_tag = next(t for t in tags if t == 'Train/mean_reward')
    recorded = event.Scalars(reward_tag)
    steps = sorted({v.step for v in recorded})
    assert len(steps) == spec['iterations'], (len(steps), spec['iterations'])
    assert steps == list(range(steps[0], steps[-1] + 1)), 'Non-contiguous PPO updates'
    assert int(latest.stem.split('_')[1]) == steps[-1], 'Final checkpoint/event boundary mismatch'
    assert spec.get('resume_checkpoint') or steps[0] == 0, 'Fresh run did not start at zero'
    save_status(phase='validated', first_iteration=steps[0], latest_iteration=steps[-1])
    print('YERTLE_TRAINING_VERIFIED', steps[0], steps[-1], latest.name, flush=True)


def manifest():
    files = {}
    for path in ROOT.rglob('*'):
        if not path.is_file() or '.git' in path.parts:
            continue
        rel = path.relative_to(ROOT).as_posix()
        if rel.startswith('source/') and not rel.startswith('source/isaac_lab/runs/'):
            continue
        if rel in ('artifact-manifest.json', 'status.json') or rel.endswith(('.log', '.pid')):
            continue
        files[rel] = {'size': path.stat().st_size, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    (ROOT / 'artifact-manifest.json').write_text(json.dumps(files, indent=2))


def archive():
    """Also works after setup failure: only archive paths which actually exist."""
    manifest()
    files = json.loads((ROOT / 'artifact-manifest.json').read_text())
    names = set(files) | {'artifact-manifest.json', 'train.log', 'status.json'}
    # Outside ROOT so the archive never includes itself; persistent across Pod deletion.
    target = ROOT.parent / 'yertle-result.tgz'
    with tarfile.open(target, 'w:gz') as tar:
        for name in sorted(names):
            path = ROOT / name
            if path.is_file():
                tar.add(path, arcname=name, recursive=False)
    print(hashlib.sha256(target.read_bytes()).hexdigest(), flush=True)


def supervise():
    save_status(phase='setup', started=time.time())
    final = None
    try:
        prepare()
        save_status(phase='training', started=time.time())
        spec = json.loads((ROOT / 'job.json').read_text())
        env = runtime_environment(spec)
        child = subprocess.Popen(['/opt/yertle-venv/bin/python', __file__, 'train'], start_new_session=True, env=env)

        def interrupt(signum, frame):
            os.killpg(child.pid, signal.SIGINT)
        signal.signal(signal.SIGTERM, interrupt)
        signal.signal(signal.SIGINT, interrupt)
        result = child.wait()
        if result == 0:
            command(['/opt/yertle-venv/bin/python', __file__, 'validate'], env=env)
        final = dict(phase='finished' if result == 0 else 'failed', exit_code=result, ended=time.time())
    except Exception as exc:
        final = dict(phase='failed', error=str(exc), ended=time.time())
        raise
    finally:
        manifest()
        if final:
            save_status(**final)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'train':
        train()
    elif len(sys.argv) > 1 and sys.argv[1] == 'archive':
        archive()
    elif len(sys.argv) > 1 and sys.argv[1] == 'validate':
        validate_training()
    else:
        supervise()
