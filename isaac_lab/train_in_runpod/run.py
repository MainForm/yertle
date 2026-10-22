"""Independent, cost-capped Yertle jobs. Python 3.11+ standard library only.

Credentials: RUNPOD_API_KEY in the local process environment, NEVER copied to pod.
All deletes are restricted to IDs recorded by this controller's own create calls.
"""
from pathlib import Path, PurePosixPath
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from urllib.parse import urlencode
import argparse
import contextlib
import hashlib
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import time
import uuid

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
BASE = 'https://api.runpod.io/v2'


class ApiError(RuntimeError):
    def __init__(self, status, body):
        self.status, self.body = status, body
        super().__init__(f'Runpod HTTP {status}: {body}')


def api(method, path, body=None):
    key = os.environ.get('RUNPOD_API_KEY')
    if not key:
        raise RuntimeError('Set RUNPOD_API_KEY in the local environment; do not put it in config.json.')
    data = json.dumps(body).encode() if body is not None else None
    req = Request(BASE + path, data=data, method=method,
                  headers={'Authorization': 'Bearer ' + key, 'Content-Type': 'application/json'})
    try:
        with urlopen(req, timeout=40) as response:
            raw = response.read()
            return json.loads(raw) if raw else None
    except HTTPError as exc:
        # Do not retry ambiguous POSTs: a lost response can still have created a pod.
        raise ApiError(exc.code, exc.read().decode(errors='replace').replace(key, '[REDACTED]')) from None


def hidden_process_options(kwargs):
    """Detached controllers must also hide their noninteractive console children."""
    options = dict(kwargs)
    if os.name == 'nt':
        options['creationflags'] = options.get('creationflags', 0) | getattr(subprocess, 'CREATE_NO_WINDOW', 0x08000000)
    return options


def execute(args, **kwargs):
    return subprocess.run([str(a) for a in args], check=True, **hidden_process_options(kwargs))


def output(args, **kwargs):
    return subprocess.check_output([str(a) for a in args], text=True, **hidden_process_options(kwargs)).strip()


def save(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=2), encoding='utf-8')
    tmp.replace(path)


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def source_digest(path):
    # Git autocrlf differs between Windows and Linux. Compare semantic file bytes.
    raw = path.read_bytes()
    return hashlib.sha256(raw if b'\0' in raw else raw.replace(b'\r\n', b'\n')).hexdigest()


def list_all(path, field):
    results, cursor = [], None
    while True:
        sep = '&' if '?' in path else '?'
        data = api('GET', path + (sep + urlencode({'cursor': cursor}) if cursor else ''))
        results.extend(data[field])
        page = data.get('pagination', {})
        if not page.get('hasNextPage'):
            return results
        cursor = page['nextCursor']


def candidates(config, gpus, centers):
    """RTX-capable allowlist + live stock + STANDARD volume, cheapest first."""
    standard = {d['id'] for d in centers if 'STANDARD' in d.get('networkVolumeTypes', [])}
    result = []
    for gpu in gpus:
        price = gpu.get('price', {}).get('secure')
        if gpu['id'] not in config['gpu_ids'] or not price or price > config['max_gpu_hourly_usd']:
            continue
        if gpu.get('availability') in (None, 'NONE'):
            continue
        for dc in gpu.get('dataCenters', []):
            if dc['id'] in standard and dc.get('availability') not in (None, 'NONE'):
                result.append({'gpu': gpu['id'], 'dc': dc['id'], 'gpu_hourly': price,
                               'stock': dc['availability']})
    return sorted(result, key=lambda c: (c['gpu_hourly'], c['dc']))


def quote(config, item):
    storage = (config['container_disk_gb'] * config['disk_monthly_usd_per_gb'] +
               config['network_volume_gb'] * config['volume_monthly_usd_per_gb']) / 730
    return item['gpu_hourly'] + storage


def placement_failure(exc):
    # Authentication, account quota, invalid image/schema and unknown failures are fatal.
    if exc.status not in (400, 409, 422, 503):
        return False
    message = exc.body.lower()
    if any(s in message for s in ('balance', 'unauthorized', 'permission', 'invalid gpu')):
        return False
    return any(s in message for s in ('no instances available', 'insufficient capacity',
                                      'no available', 'out of stock', 'no capacity', 'datacenter limit',
                                      'data center limit', 'region limit', 'regional limit'))


def snapshot(config, jobdir, include_dirty, resume=None):
    head = output(['git', '-C', REPO, 'rev-parse', 'HEAD'])
    dirty = output(['git', '-C', REPO, 'status', '--porcelain'])
    if dirty and not include_dirty:
        raise RuntimeError('Working tree has changes. Commit them, or explicitly pass --include-dirty.')
    bundle = jobdir / 'source.bundle'
    execute(['git', '-C', REPO, 'bundle', 'create', bundle, 'HEAD'])
    patch = subprocess.check_output(['git', '-C', REPO, 'diff', '--binary', 'HEAD']) if include_dirty else b''
    (jobdir / 'working.patch').write_bytes(patch)
    extra = {}
    if include_dirty:
        names = subprocess.check_output(['git', '-C', REPO, 'ls-files', '--others', '--exclude-standard', '-z']).decode().split('\0')
        for name in filter(None, names):
            relative = PurePosixPath(name)
            if (not name.startswith('isaac_lab/') or relative.suffix not in ('.py', '.json', '.md', '.sh', '.ps1')
                    and relative.name != '.gitignore'):
                raise RuntimeError('Untracked file not in source allowlist; commit/exclude it first: ' + name)
            if any(word in name.lower() for word in ('secret', 'password', '.env', 'private_key', '.state/')):
                raise RuntimeError('Refusing sensitive-looking untracked file: ' + name)
            src = REPO / name
            dst = jobdir / 'extra' / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_bytes(src.read_bytes())
            extra[name] = digest(src)
    tracked = subprocess.check_output(['git', '-C', REPO, 'ls-files', '-z']).decode().split('\0')
    hashes = {name: source_digest(REPO / name) for name in filter(None, tracked) if (REPO / name).is_file()}
    spec = dict(config, commit=head, extra_files=extra, source_hashes=hashes, dirty_snapshot=bool(dirty))
    if resume:
        src = Path(resume).resolve()
        (jobdir / 'resume.pt').write_bytes(src.read_bytes())
        spec['resume_checkpoint'] = {'name': src.name, 'sha256': digest(src)}
    save(jobdir / 'job.json', spec)
    for name in ('remote_job.py', 'bootstrap.sh'):
        (jobdir / name).write_text((HERE / name).read_text(encoding='utf-8'), encoding='utf-8', newline='\n')
    payload = jobdir / 'payload.tgz'
    with tarfile.open(payload, 'w:gz') as tar:
        for path in jobdir.iterdir():
            if path.name in ('source.bundle', 'working.patch', 'extra', 'job.json', 'resume.pt', 'remote_job.py', 'bootstrap.sh'):
                tar.add(path, arcname=path.name)
    return spec, payload


def verify_snapshot(jobdir):
    """Offline replay of the exact bundle + patch + extras, in a new checkout."""
    spec = json.loads((jobdir / 'job.json').read_text(encoding='utf-8'))
    source = jobdir / ('replay-' + uuid.uuid4().hex[:8])
    execute(['git', 'clone', str(jobdir / 'source.bundle'), str(source)])
    execute(['git', '-C', source, 'checkout', '--detach', spec['commit']])
    if (jobdir / 'working.patch').stat().st_size:
        execute(['git', '-C', source, 'apply', '--check', str(jobdir / 'working.patch')])
        execute(['git', '-C', source, 'apply', str(jobdir / 'working.patch')])
    for name, expected in spec['extra_files'].items():
        src = jobdir / 'extra' / name
        assert digest(src) == expected, name
        dst = source / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(src.read_bytes())
    for name, expected in spec['source_hashes'].items():
        assert source_digest(source / name) == expected, name
    print('SNAPSHOT_REPLAY_VERIFIED', spec['commit'], len(spec['source_hashes']), len(spec['extra_files']), flush=True)
    return source


class Remote:
    def __init__(self, pod, key, known_hosts):
        ssh = pod.get('ssh') or {}
        direct = ssh.get('direct')
        if isinstance(direct, dict):
            self.args = ['-p', str(direct['port']), direct['username'] + '@' + direct['host']]
        else:
            ports = pod.get('runtime', {}).get('ports', []) if pod.get('runtime') else []
            mapping = next((p for p in ports if p.get('private') == 22 and p.get('public') and p.get('ip')), None)
            if not mapping:
                raise RuntimeError('Direct SSH mapping not ready')
            self.args = ['-p', str(mapping['public']), 'root@' + mapping['ip']]
        self.common = ['-i', str(key), '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=12',
                       '-o', 'StrictHostKeyChecking=accept-new', '-o', 'UserKnownHostsFile=' + str(known_hosts)]

    def run(self, command, timeout=60):
        return output(['ssh', *self.common, *self.args, command], timeout=timeout, stderr=subprocess.PIPE)

    def copy(self, source, target, download=False):
        args = self.args[:]
        destination = args.pop()
        if len(args) != 2 or args[0] != '-p':
            raise RuntimeError('Unsupported SSH direct shape; use explicit API runtime port mapping.')
        remote = destination + ':' + target
        cmd = ['scp', *self.common, '-P', args[1]]
        execute(cmd + ([remote, str(source)] if download else [str(source), remote]), timeout=300)


def get_remote(podid, config, state_dir):
    pod = api('GET', '/pods/' + podid)
    return Remote(pod, config['ssh_key'], state_dir / 'known_hosts')


def extract_verified(archive, target):
    target.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive, 'r:gz') as tar:
        for member in tar.getmembers():
            relative = PurePosixPath(member.name)
            if relative.is_absolute() or '..' in relative.parts or member.issym() or member.islnk():
                raise RuntimeError('Unsafe result archive member: ' + member.name)
        tar.extractall(target, filter='data')
    manifest = json.loads((target / 'artifact-manifest.json').read_text())
    for name, expected in manifest.items():
        path = target / name
        if path.stat().st_size != expected['size'] or digest(path) != expected['sha256']:
            raise RuntimeError('Backup verification failed: ' + name)
    return manifest


def organize_backup(backup):
    """Provide the project's checkpoint/training_data layout, retaining raw evidence."""
    raw = backup / 'files'
    runs = raw / 'source/isaac_lab/runs'
    copied = []
    for src in runs.rglob('*') if runs.exists() else []:
        if not src.is_file():
            continue
        if src.name.startswith('model_') and src.suffix == '.pt':
            dst = backup / 'checkpoints' / src.name
            if dst.exists() and digest(dst) != digest(src):
                raise RuntimeError('Ambiguous checkpoint basename: ' + src.name)
        else:
            dst = backup / 'training_data' / src.relative_to(runs)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        if digest(src) != digest(dst):
            raise RuntimeError('Organized backup mismatch: ' + str(dst))
        copied.append(dst.relative_to(backup).as_posix())
    for name in ('job.json', 'resolved.json', 'status.json', 'train.log', 'interventions.json'):
        src = raw / name
        if src.is_file():
            dst = backup / 'training_data' / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            assert digest(src) == digest(dst)
    save(backup / 'organized-backup.json', {'verified_files': copied})
    return copied


def delete_confirm(path):
    try:
        api('DELETE', path)
    except ApiError as exc:
        if exc.status != 404:
            raise
    try:
        api('GET', path)
    except ApiError as exc:
        if exc.status == 404:
            return
        raise
    raise RuntimeError('Deletion not confirmed: ' + path)


def cleanup(state, path, delete_volume=False):
    if state.get('pod_id'):
        delete_confirm('/pods/' + state['pod_id'])
        state['deleted_pod_id'] = state.pop('pod_id')
        save(path, state)
    if delete_volume and state.get('volume_id'):
        delete_confirm('/network-volumes/' + state['volume_id'])
        state['deleted_volume_id'] = state.pop('volume_id')
        save(path, state)


def retain_volume(state, path, error):
    """Delete owned compute, never the persistent result volume on failure."""
    state.update(last_error=str(error), phase='cleanup_pending')
    save(path, state)
    cleanup(state, path, delete_volume=False)
    state['phase'] = 'volume_retained' if state.get('volume_id') else 'failed_cleaned'
    save(path, state)


def download_results(remote, config, state, statefile):
    archive_hash = remote.run('python3 /workspace/yertle-job/remote_job.py archive', timeout=240).splitlines()[-1]
    backup = Path(config['backup_dir'])
    if not backup.is_absolute():
        backup = HERE / backup
    backup = backup / statefile.parent.name
    backup.mkdir(parents=True, exist_ok=True)
    archive = backup / 'result.tgz'
    remote.copy(archive, '/workspace/yertle-result.tgz', download=True)
    if digest(archive) != archive_hash:
        raise RuntimeError('Transferred archive hash mismatch')
    files = extract_verified(archive, backup / 'files')
    organized = organize_backup(backup)
    checkpoints = list((backup / 'files/source/isaac_lab/runs').rglob('model_*.pt'))
    state.update(backup=str(backup), backup_verified=True, artifact_count=len(files),
                 checkpoint_names=[p.name for p in checkpoints], organized_files=len(organized))
    save(statefile, state)


@contextlib.contextmanager
def job_lock(jobdir):
    """Only recovery of the same job is exclusive; separate jobs may run together."""
    lock = jobdir / 'controller.lock'
    with lock.open('x') as stream:
        stream.write(str(os.getpid()))
    try:
        yield
    finally:
        lock.unlink(missing_ok=True)


def job_state(job_id):
    # Never accept arbitrary state files/resource IDs for destructive operations.
    if not re.fullmatch(r'\d{8}T\d{6}Z-[a-f0-9]{6}', job_id):
        raise ValueError('Use the exact locally recorded job ID.')
    path = HERE / '.state' / job_id / 'state.json'
    return path, json.loads(path.read_text(encoding='utf-8'))


def list_jobs():
    rows = []
    for path in sorted((HERE / '.state').glob('*/state.json')):
        state = json.loads(path.read_text(encoding='utf-8'))
        rows.append({k: state.get(k) for k in ('phase', 'pod_id', 'volume_id', 'backup_verified', 'last_error')}
                    | {'job_id': path.parent.name})
    print(json.dumps(rows, indent=2), flush=True)


def recover_job(job_id, config):
    path, state = job_state(job_id)
    with job_lock(path.parent):
        if state.get('phase') not in ('volume_retained', 'cleanup_pending'):
            raise RuntimeError('Recovery accepts retained/cleanup-pending jobs only; active jobs are left untouched.')
        spec = json.loads((path.parent / 'job.json').read_text(encoding='utf-8'))
        # Credentials remain local. Keep the original job's SSH key and runtime image.
        spec['backup_dir'] = config['backup_dir']
        recovery_created = False
        try:
            if state.get('pod_id') and state.get('phase') == 'cleanup_pending':
                retain_volume(state, path, state.get('last_error', 'Retrying failed compute cleanup'))
            if not state.get('pod_id'):
                if not state.get('volume_id'):
                    raise RuntimeError('Job has no retained volume or active pod.')
                body = state.get('pod_template')
                if not body:
                    raise RuntimeError('Legacy job has no recovery Pod template; manual recovery required.')
                api('GET', '/network-volumes/' + state['volume_id'])
                catalog = list_all('/catalog/gpus?include=AVAILABILITY&product=POD&cloud=SECURE&minCudaVersion=' + spec['min_cuda'], 'gpus')
                centers = api('GET', '/catalog/datacenters')['dataCenters']
                matches = [item for item in candidates(spec, catalog, centers)
                           if item['dc'] == state['placement']['dc'] and item['gpu'] == body['gpu']['id']]
                if not matches:
                    raise RuntimeError('Retained volume region currently unavailable/over price cap. Volume kept; retry later.')
                item = matches[0]
                print('RECOVERY_CREATES_BILLABLE_POD', json.dumps(dict(item, total_planning_hourly=quote(spec, item))), flush=True)
                pod = api('POST', '/pods', body)
                recovery_created = True
                state.update(pod_id=pod['id'], phase='recovering')
                save(path, state)
                readback = api('GET', '/pods/' + pod['id'])
                if readback['mounts']['network'][0]['volumeId'] != state['volume_id']:
                    raise RuntimeError('Recovery mount mismatch')
            deadline = time.time() + min(900, spec['max_seconds'])
            remote = None
            while time.time() < deadline:
                try:
                    candidate = get_remote(state['pod_id'], spec, path.parent)
                    candidate.run('printf YERTLE_RECOVERY_READY')
                    remote = candidate
                    break
                except ApiError:
                    raise
                except (RuntimeError, subprocess.SubprocessError):
                    time.sleep(10)
            if remote is None:
                raise RuntimeError('Recovery SSH deadline exceeded')
            status = json.loads(remote.run("cat /workspace/yertle-job/status.json 2>/dev/null || printf '{\"phase\":\"failed\"}'"))
            if not recovery_created and status['phase'] not in ('finished', 'failed'):
                raise RuntimeError('Job is not finished. Recovery never interrupts an active training job.')
            # Install the current archiver (stdlib only) for legacy /tmp archives too.
            remote.copy(HERE / 'remote_job.py', '/workspace/yertle-job/remote_job.py')
            download_results(remote, spec, state, path)
            cleanup(state, path, delete_volume=True)
            state['phase'] = 'recovered'
            save(path, state)
        except BaseException as exc:
            # A recovery-created Pod is disposable; do not interrupt a pre-existing job.
            if recovery_created:
                retain_volume(state, path, exc)
            raise


def monitor_job(config, state, statefile, remote):
    """Observe the existing detached training; never create or relaunch a Pod."""
    state['controller_pid'] = os.getpid()
    save(statefile, state)
    last_phase = None
    while time.time() < state['deadline']:
        try:
            status = json.loads(remote.run('cat /workspace/yertle-job/status.json'))
            if status != last_phase:
                print('REMOTE_STATUS', json.dumps(status), flush=True)
                last_phase = status
            state['controller_last_seen'] = time.time()
            save(statefile, state)
            if status['phase'] in ('finished', 'failed'):
                break
        except subprocess.SubprocessError:
            pass
        time.sleep(15)
    else:
        remote.run("pkill -INT -f '^/opt/yertle-venv/bin/python .*/remote_job.py train$' || true")
        raise RuntimeError('Time/budget planning deadline reached; stopping compute and retaining the volume for recovery.')
    state['remote_status'] = status
    save(statefile, state)
    download_results(remote, config, state, statefile)
    cleanup(state, statefile, delete_volume=True)
    state['phase'] = 'complete' if status['phase'] == 'finished' and state['checkpoint_names'] else 'failed_cleaned'
    save(statefile, state)
    print('JOB_RESULT', json.dumps(state, indent=2), flush=True)
    if state['phase'] != 'complete':
        raise RuntimeError('Remote job failed; verified logs were downloaded and owned resources deleted.')


def attach_monitor(job_id, config):
    """Explicit local-controller replacement; original deadline is never extended."""
    path, state = job_state(job_id)
    with job_lock(path.parent):
        if state.get('phase') != 'working' or not state.get('pod_id'):
            raise RuntimeError('Only an existing working job can be monitored; no resources created.')
        spec = json.loads((path.parent / 'job.json').read_text(encoding='utf-8'))
        spec['backup_dir'] = config['backup_dir']
        remote = get_remote(state['pod_id'], spec, path.parent)
        try:
            monitor_job(spec, state, path, remote)
        except BaseException as error:
            if state.get('pod_id'):
                retain_volume(state, path, error)
            raise


def run_job(config, args):
    statedir = HERE / '.state'
    statedir.mkdir(exist_ok=True)
    state, statefile = {}, None
    try:
        catalog = list_all('/catalog/gpus?include=AVAILABILITY&product=POD&cloud=SECURE&minCudaVersion=' + config['min_cuda'], 'gpus')
        centers = api('GET', '/catalog/datacenters')['dataCenters']
        choices = candidates(config, catalog, centers)
        choices = [c for c in choices if config['max_planned_usd'] / quote(config, c) * 3600 >= 600]
        if not choices:
            raise RuntimeError('No live RTX capacity with standard storage inside the configured price/time cap.')
        print(json.dumps({'choices': choices, 'max_seconds': config['max_seconds'],
                          'max_planned_usd': config['max_planned_usd']}, indent=2), flush=True)
        if args.dry_run:
            return
        if not Path(config['ssh_key']).is_file():
            raise RuntimeError('SSH private key file not found; set ssh_key in config.json.')
        public_key = Path(config['ssh_key'] + '.pub').read_text().strip()
        jobdir = statedir / (time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '-' + uuid.uuid4().hex[:6])
        jobdir.mkdir()
        spec, payload = snapshot(config, jobdir, args.include_dirty, args.resume_checkpoint)
        statefile = jobdir / 'state.json'
        state = {'created_at': time.time(), 'job_dir': str(jobdir), 'phase': 'placing', 'commit': spec['commit']}
        save(statefile, state)
        for item in choices:
            print('BILLING_QUOTE', json.dumps(dict(item, total_planning_hourly=quote(config, item))), flush=True)
            volume = api('POST', '/network-volumes', {'name': 'yertle-test-' + jobdir.name,
                         'dataCenter': item['dc'], 'size': config['network_volume_gb'], 'type': 'STANDARD'})
            state['volume_id'] = volume['id']
            save(statefile, state)
            verified_volume = api('GET', '/network-volumes/' + volume['id'])
            assert verified_volume['id'] == volume['id']
            body = {'name': 'yertle-v620-test-' + jobdir.name, 'image': config['image'],
                    'gpu': {'id': item['gpu'], 'count': 1, 'minCudaVersion': config['min_cuda'],
                            'minRamPerGpu': 24, 'minVcpuCountPerGpu': 4},
                    'cloud': 'SECURE', 'dataCenterIds': [item['dc']],
                    'disk': config['container_disk_gb'], 'mounts': {'network': [{'volumeId': volume['id'], 'path': '/workspace'}]},
                    'ports': ['22/tcp'], 'startSsh': True, 'startJupyter': False,
                    'env': {'PUBLIC_KEY': public_key, 'ACCEPT_EULA': 'Y', 'PRIVACY_CONSENT': 'Y',
                            'OMNI_KIT_ACCEPT_EULA': 'YES', 'PYTHONUNBUFFERED': '1'}}
            state['pod_template'] = body
            state['placement'] = item
            save(statefile, state)
            try:
                pod = api('POST', '/pods', body)
            except ApiError as exc:
                cleanup(state, statefile, delete_volume=True)
                if placement_failure(exc):
                    print('PLACEMENT_UNAVAILABLE_NEXT_REGION', item['dc'], str(exc), flush=True)
                    continue
                raise
            seconds = min(config['max_seconds'], config['max_planned_usd'] / quote(config, item) * 3600)
            state.update(pod_id=pod['id'], placement=item, phase='booting', deadline=time.time() + seconds)
            save(statefile, state)
            readback = api('GET', '/pods/' + pod['id'])
            assert readback['gpu']['id'] == item['gpu'] and readback['gpu']['count'] == 1
            assert readback['mounts']['network'][0]['volumeId'] == volume['id']
            assert readback['image'] == config['image']
            state['pod_readback'] = {k: readback.get(k) for k in ('status', 'cost', 'dataCenterId', 'image')}
            save(statefile, state)
            break
        else:
            raise RuntimeError('All currently available placements rejected capacity; nothing left running.')
        print('CREATED', state['pod_id'], flush=True)
        # A successful POST is never retried, even if boot/image pull takes minutes.
        remote = None
        boot_deadline = min(state['deadline'], time.time() + 900)
        while time.time() < boot_deadline:
            try:
                remote = get_remote(state['pod_id'], config, jobdir)
                if remote.run('printf YERTLE_SSH_READY') == 'YERTLE_SSH_READY':
                    break
            except (RuntimeError, subprocess.SubprocessError, ApiError):
                remote = None
            time.sleep(10)
        if remote is None:
            raise RuntimeError('SSH boot deadline exceeded; no second pod was created.')
        remote.run('mkdir -p /workspace/yertle-job')
        remote.copy(payload, '/workspace/payload.tgz')
        assert remote.run('sha256sum /workspace/payload.tgz').split()[0] == digest(payload)
        remote.run('tar -xzf /workspace/payload.tgz -C /workspace/yertle-job')
        remote.run("bash -lc 'cd /workspace/yertle-job; setsid python3 remote_job.py > train.log 2>&1 < /dev/null & echo YERTLE_LAUNCHED'")
        state['phase'] = 'working'
        save(statefile, state)
        with job_lock(jobdir):
            monitor_job(config, state, statefile, remote)
    except BaseException as error:
        if statefile and state.get('pod_id'):
            # Network volume survives Pod deletion; ephemeral disk does not.
            try:
                retain_volume(state, statefile, error)
                print('RECOVERY_REQUIRED: Pod deleted; network volume still bills. State:', statefile, flush=True)
            except Exception as exc:
                print('URGENT: could not delete owned pod:', state.get('pod_id'), str(exc), flush=True)
        elif statefile and state.get('volume_id'):
            cleanup(state, statefile, delete_volume=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE / 'config.json')
    parser.add_argument('--dry-run', action='store_true', help='Read prices/capacity only; never create resources.')
    parser.add_argument('--include-dirty', action='store_true', help='Explicitly snapshot tracked diff and safe untracked source.')
    parser.add_argument('--iterations', type=int)
    parser.add_argument('--num-envs', type=int)
    parser.add_argument('--resume-checkpoint', type=Path)
    parser.add_argument('--api-key-file', type=Path, help='Read the explicitly selected local key file; never upload it.')
    parser.add_argument('--backup-dir', type=Path, help='Local verified artifact destination.')
    parser.add_argument('--profile', help='Training profile relative to the source checkout.')
    parser.add_argument('--verify-package', type=Path, help='Offline bundle replay; never calls Runpod.')
    parser.add_argument('--list-jobs', action='store_true', help='List local job/resource records without API calls.')
    parser.add_argument('--recover', metavar='JOB_ID', help='Rent a temporary Pod in the retained volume region, retrieve, verify and delete.')
    parser.add_argument('--delete-retained', metavar='JOB_ID', help='Permanently delete a retained result volume.')
    parser.add_argument('--confirm-data-loss', action='store_true', help='Explicitly authorize deletion of an unbacked retained volume.')
    parser.add_argument('--background', action='store_true', help='Start an independent local controller; PC/network must stay on.')
    parser.add_argument('--monitor', metavar='JOB_ID', help='Attach to a recorded working job after its previous local controller has exited; never start training again.')
    args = parser.parse_args()
    if args.background:
        logs = HERE / '.state' / 'logs'
        logs.mkdir(parents=True, exist_ok=True)
        log = logs / (time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '-' + uuid.uuid4().hex[:6] + '.log')
        command = [sys.executable, '-u', str(Path(__file__).resolve()),
                   *(arg for arg in sys.argv[1:] if arg != '--background')]
        options = {'creationflags': subprocess.DETACHED_PROCESS | subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW} if os.name == 'nt' else {'start_new_session': True}
        with log.open('ab') as stream:
            child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stream, stderr=stream, **options)
        print(json.dumps({'controller_pid': child.pid, 'log': str(log), 'pc_must_stay_on': True}))
        return
    if args.list_jobs:
        list_jobs()
        return
    if args.verify_package:
        verify_snapshot(args.verify_package.resolve())
        return
    if args.api_key_file:
        key = args.api_key_file.read_text(encoding='utf-8-sig').strip()
        if not key or any(c.isspace() or ord(c) < 32 for c in key):
            raise ValueError('Selected key file is empty or contains internal whitespace/control characters.')
        os.environ['RUNPOD_API_KEY'] = key
    config = json.loads(args.config.read_text(encoding='utf-8'))
    if args.backup_dir:
        config['backup_dir'] = str(args.backup_dir.resolve())
    if args.profile:
        config['profile'] = args.profile
    if args.iterations is not None:
        config['iterations'] = args.iterations
    if args.num_envs is not None:
        config['num_envs'] = args.num_envs
    if config['iterations'] < 1 or config['max_seconds'] <= 0 or config['max_planned_usd'] <= 0:
        raise ValueError('Iterations, time and budget must be positive.')
    if args.monitor:
        attach_monitor(args.monitor, config)
        return
    if args.recover:
        recover_job(args.recover, config)
        return
    if args.delete_retained:
        path, state = job_state(args.delete_retained)
        with job_lock(path.parent):
            if state.get('pod_id') or state.get('phase') != 'volume_retained':
                raise RuntimeError('Only a retained volume with no attached Pod can be deleted here.')
            if not state.get('backup_verified') and not args.confirm_data_loss:
                raise RuntimeError('No verified local backup. Use --confirm-data-loss only to intentionally discard results.')
            cleanup(state, path, delete_volume=True)
            state['phase'] = 'discarded' if not state.get('backup_verified') else 'complete'
            save(path, state)
        return
    run_job(config, args)


if __name__ == '__main__':
    main()
