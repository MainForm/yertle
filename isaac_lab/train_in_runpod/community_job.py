"""Prepare a frozen job, then attach to ONE explicitly created Community Pod.

No Pod creation here: the availability heartbeat uses Runpod MCP to create it.
On backup failure, STOP (do not delete) the host-persistent result disk's Pod.
"""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys
import time
import uuid
import run

HERE = Path(__file__).resolve().parent
EXPECTED_COMMIT = '6432f7d934904c12a0fdf99315dc484c9e0387e3'
KEY_FILE = Path('C:/Users/ik/Desktop/runpod_issac_sim/runpod_api_key2.txt')
REQUEST = HERE / '.state/low_cost_wait_request.json'


def prepare():
    if REQUEST.exists():
        print(REQUEST.read_text(encoding='utf-8'))
        return
    config = json.loads((HERE / 'config.json').read_text(encoding='utf-8'))
    config.update(profile='isaac_lab/configs/v6_22_2_training.json', iterations=3000,
                  num_envs=1024, max_gpu_hourly_usd=0.25, max_planned_usd=1.70,
                  max_seconds=36000, persistent_disk_gb=10, cloud='COMMUNITY')
    tag = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '-' + uuid.uuid4().hex[:6]
    job = HERE / '.state' / tag
    job.mkdir(parents=True)
    clean = job / 'committed-source'
    run.execute(['git', 'clone', '--local', '--no-hardlinks', '--no-checkout', str(run.REPO), str(clean)])
    run.execute(['git', '-C', str(clean), 'checkout', '--detach', EXPECTED_COMMIT])
    original_repo = run.REPO
    try:
        run.REPO = clean
        spec, payload = run.snapshot(config, job, include_dirty=False)
    finally:
        run.REPO = original_repo
    assert spec['commit'] == EXPECTED_COMMIT and not spec['dirty_snapshot']
    assert json.loads((clean / spec['profile']).read_text())['fresh'] is True
    state = dict(phase='waiting_capacity', job_dir=str(job), commit=EXPECTED_COMMIT,
                 pod_name='yertle-cheap-' + tag, storage='host-persistent',
                 created_at=time.time())
    run.save(job / 'state.json', state)
    request = dict(job_id=tag, job_dir=str(job), payload=str(payload), commit=EXPECTED_COMMIT,
                   profile=spec['profile'], iterations=3000, num_envs=1024,
                   max_gpu_hourly_usd=0.25, max_planned_usd=1.70, account_budget_usd=2.0,
                   min_cuda='12.8', min_vram_gb=16, cloud='COMMUNITY', pod_name=state['pod_name'],
                   status='waiting_capacity', api_key_file=str(KEY_FILE))
    run.save(REQUEST, request)
    print(json.dumps(request, indent=2))


def deadline(start, config, gpu_price):
    if not 0 < gpu_price <= config['max_gpu_hourly_usd']:
        raise ValueError('Pod price exceeds the approved hourly cap')
    hourly = gpu_price + (config['container_disk_gb'] + config['persistent_disk_gb']) * config['disk_monthly_usd_per_gb'] / 730
    return start + min(config['max_seconds'], config['max_planned_usd'] / hourly * 3600)


def stop_preserving_disk(state, path, error):
    state.update(phase='stop_requested', last_error=str(error))
    run.save(path, state)
    run.api('POST', '/pods/' + state['pod_id'] + '/action', {'action': 'stop'})
    for attempt in range(5):
        pod = run.api('GET', '/pods/' + state['pod_id'])
        state['stop_readback'] = {k: pod.get(k) for k in ('status', 'cost')}
        if pod.get('status') in ('EXITED', 'STOPPED'):
            state['phase'] = 'stopped_persistent_disk_retained'
            run.save(path, state)
            return
        run.save(path, state)
        if attempt < 4:
            time.sleep(5)
    raise RuntimeError('Stop requested but not confirmed; inspect the recorded Pod urgently')


def supervise(job_id, pod_id):
    path, state = run.job_state(job_id)
    config = json.loads((path.parent / 'job.json').read_text(encoding='utf-8'))
    with run.job_lock(path.parent):
        if state.get('pod_id') not in (None, pod_id) or state['phase'] not in ('waiting_capacity', 'creating', 'booting', 'working'):
            raise RuntimeError('Job already bound/finished; never create duplicate resources')
        # ID must come from this conversation's successful MCP create response.
        state.update(pod_id=pod_id, controller_pid=os.getpid(), phase='booting',
                     billing_start=state.get('billing_start', time.time()))
        run.save(path, state)
        remote = None
        ownership_verified = False
        training_launched = bool(state.get('training_launched'))
        try:
            pod = run.api('GET', '/pods/' + pod_id)
            if pod.get('name') != state['pod_name'] or pod.get('image') != config['image']:
                raise RuntimeError('Created Pod identity mismatch')
            mount = pod.get('mounts', {}).get('persistent') or {}
            if mount.get('path') != '/workspace' or mount.get('size') != config['persistent_disk_gb']:
                raise RuntimeError('Persistent result disk mismatch')
            ownership_verified = True
            state.update(deadline=deadline(state['billing_start'], config, pod['cost']),
                         pod_readback={k: pod.get(k) for k in ('name', 'status', 'cost', 'gpu', 'image', 'mounts')})
            run.save(path, state)
            boot_until = min(state['deadline'], time.time() + 900)
            while time.time() < boot_until:
                try:
                    candidate = run.get_remote(pod_id, config, path.parent)
                    candidate.run('printf READY')
                    remote = candidate
                    break
                except (RuntimeError, subprocess.SubprocessError):
                    time.sleep(10)
            if remote is None:
                raise RuntimeError('SSH boot deadline exceeded; do not rent another Pod')
            if not training_launched:
                remote.run('mkdir -p /workspace/yertle-job')
                remote.copy(path.parent / 'payload.tgz', '/workspace/payload.tgz')
                if remote.run('sha256sum /workspace/payload.tgz').split()[0] != run.digest(path.parent / 'payload.tgz'):
                    raise RuntimeError('Upload hash mismatch')
                remote.run('tar -xzf /workspace/payload.tgz -C /workspace/yertle-job')
                # Persist intent BEFORE launch: uncertain SSH outcomes must not relaunch training.
                state['training_launched'] = True
                training_launched = True
                run.save(path, state)
                remote.run("bash -lc 'cd /workspace/yertle-job; setsid python3 remote_job.py > train.log 2>&1 < /dev/null & echo YERTLE_LAUNCHED'")
            state['phase'] = 'working'
            run.save(path, state)
            status = None
            while time.time() < state['deadline']:
                try:
                    observed = json.loads(remote.run('cat /workspace/yertle-job/status.json'))
                    if observed != status:
                        print('REMOTE_STATUS', json.dumps(observed), flush=True)
                    status = observed
                    state['controller_last_seen'] = time.time()
                    run.save(path, state)
                    if status['phase'] in ('finished', 'failed'):
                        break
                except (subprocess.SubprocessError, json.JSONDecodeError):
                    pass
                time.sleep(15)
            else:
                state['budget_deadline_reached'] = True
                run.save(path, state)
                remote.run("pkill -INT -f '^/opt/yertle-venv/bin/python .*/remote_job.py train$' || true")
                # Let the supervisor close files; never archive concurrently changing events.
                until = time.time() + 45
                while time.time() < until:
                    status = json.loads(remote.run('cat /workspace/yertle-job/status.json'))
                    if status['phase'] in ('finished', 'failed'):
                        break
                    time.sleep(3)
                else:
                    raise RuntimeError('Budget deadline: training did not stop within backup grace')
            state['remote_status'] = status
            run.save(path, state)
            run.download_results(remote, config, state, path)
            run.cleanup(state, path, delete_volume=False)
            state['phase'] = 'complete' if status['phase'] == 'finished' else 'partial_or_failed_backed_up'
            run.save(path, state)
            print('JOB_RESULT', json.dumps(state, indent=2), flush=True)
        except BaseException as exc:
            if state.get('pod_id') and ownership_verified:
                if state.get('backup_verified') or not training_launched:
                    # No unique results exist before launch, or results are verified locally.
                    run.cleanup(state, path, delete_volume=False)
                    state.update(phase='failed_cleaned', last_error=str(exc))
                    run.save(path, state)
                else:
                    stop_preserving_disk(state, path, exc)
            raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['prepare', 'supervise'])
    parser.add_argument('--job-id')
    parser.add_argument('--pod-id')
    parser.add_argument('--background', action='store_true')
    args = parser.parse_args()
    if args.mode == 'prepare':
        prepare()
        return
    if not args.job_id or not args.pod_id:
        parser.error('supervise requires --job-id and --pod-id from the successful create receipt')
    if args.background:
        logfile = HERE / '.state' / args.job_id / 'community-controller.log'
        options = {'creationflags': subprocess.DETACHED_PROCESS | subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW} if os.name == 'nt' else {'start_new_session': True}
        with logfile.open('ab') as stream:
            process = subprocess.Popen([sys.executable, '-u', str(Path(__file__).resolve()), *[a for a in sys.argv[1:] if a != '--background']],
                                       stdin=subprocess.DEVNULL, stdout=stream, stderr=stream, **options)
        print(json.dumps({'controller_pid': process.pid, 'log': str(logfile)}))
        return
    key = KEY_FILE.read_text(encoding='utf-8-sig').strip()
    if not key or any(c.isspace() for c in key):
        raise ValueError('Invalid local credential format')
    os.environ['RUNPOD_API_KEY'] = key
    supervise(args.job_id, args.pod_id)


if __name__ == '__main__':
    main()
