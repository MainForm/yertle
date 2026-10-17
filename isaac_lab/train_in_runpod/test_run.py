"""Offline controller tests: never rent a GPU or contact Runpod."""
from pathlib import Path
import importlib.util
import io
import json
import tarfile
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('yertle_runpod', Path(__file__).with_name('run.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(Path(__file__).with_name('config.json').read_text())
        self.centers = [{'id': 'one', 'networkVolumeTypes': ['STANDARD']},
                        {'id': 'two', 'networkVolumeTypes': ['STANDARD']},
                        {'id': 'no-storage', 'networkVolumeTypes': []}]
        self.gpu = {'id': self.config['gpu_ids'][0], 'availability': 'LOW', 'price': {'secure': .5},
                    'dataCenters': [{'id': 'one', 'availability': 'LOW'}, {'id': 'two', 'availability': 'LOW'},
                                    {'id': 'no-storage', 'availability': 'LOW'}]}

    def test_region_selection_requires_storage(self):
        result = runner.candidates(self.config, [self.gpu], self.centers)
        self.assertEqual([r['dc'] for r in result], ['one', 'two'])

    def test_out_of_stock_excluded(self):
        self.gpu['availability'] = 'NONE'
        self.assertEqual(runner.candidates(self.config, [self.gpu], self.centers), [])

    def test_account_gpu_price_cap(self):
        self.gpu['price']['secure'] = 10
        self.assertEqual(runner.candidates(self.config, [self.gpu], self.centers), [])

    def test_non_rtx_gpu_excluded(self):
        self.gpu['id'] = 'NVIDIA A100-SXM4-80GB'
        self.assertEqual(runner.candidates(self.config, [self.gpu], self.centers), [])

    def test_cost_includes_both_disks(self):
        self.assertAlmostEqual(runner.quote(self.config, {'gpu_hourly': .5}), .5 + 8.7 / 730)

    def test_capacity_error_allows_other_region(self):
        self.assertTrue(runner.placement_failure(runner.ApiError(400, 'No instances available')))

    def test_auth_error_never_retried(self):
        self.assertFalse(runner.placement_failure(runner.ApiError(401, 'invalid token')))

    def test_balance_error_never_retried(self):
        self.assertFalse(runner.placement_failure(runner.ApiError(400, 'Insufficient balance, no available')))

    def test_unknown_400_not_capacity(self):
        self.assertFalse(runner.placement_failure(runner.ApiError(400, 'invalid image field')))

    def test_windows_linux_text_equivalence(self):
        with tempfile.TemporaryDirectory() as folder:
            a, b = Path(folder) / 'a', Path(folder) / 'b'
            a.write_bytes(b'print(1)\r\n'); b.write_bytes(b'print(1)\n')
            self.assertEqual(runner.source_digest(a), runner.source_digest(b))
            self.assertNotEqual(runner.digest(a), runner.digest(b))

    def test_remote_eula_not_dependent_on_ssh_forwarding(self):
        module_spec = importlib.util.spec_from_file_location('yertle_remote', Path(__file__).with_name('remote_job.py'))
        remote = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(remote)
        with patch.dict(remote.os.environ, {}, clear=True):
            env = remote.runtime_environment(self.config)
        self.assertEqual(env['OMNI_KIT_ACCEPT_EULA'], 'YES')
        self.assertEqual(env['OMNI_KIT_ALLOW_ROOT'], '1')
        self.assertNotIn('RUNPOD_API_KEY', env)

    def test_binary_not_normalized(self):
        with tempfile.TemporaryDirectory() as folder:
            a, b = Path(folder) / 'a', Path(folder) / 'b'
            a.write_bytes(b'\0\r\n'); b.write_bytes(b'\0\n')
            self.assertNotEqual(runner.source_digest(a), runner.source_digest(b))

    def test_current_v2_ssh_shape(self):
        remote = runner.Remote({'ssh': {'direct': {'host': '192.0.2.1', 'port': 2233, 'username': 'root'}}},
                               Path('key'), Path('known_hosts'))
        self.assertEqual(remote.args, ['-p', '2233', 'root@192.0.2.1'])
        self.assertIn('StrictHostKeyChecking=accept-new', remote.common)

    def test_missing_ssh_does_not_mean_new_pod(self):
        with self.assertRaises(RuntimeError):
            runner.Remote({'ssh': {'direct': None}, 'runtime': None}, Path('key'), Path('hosts'))

    def test_backup_hash_checked(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); data = root / 'model.pt'; data.write_bytes(b'test-checkpoint')
            manifest = {'model.pt': {'size': data.stat().st_size, 'sha256': runner.digest(data)}}
            (root / 'artifact-manifest.json').write_text(json.dumps(manifest))
            archive = root / 'results.tgz'
            with tarfile.open(archive, 'w:gz') as tar:
                tar.add(data, arcname='model.pt')
                tar.add(root / 'artifact-manifest.json', arcname='artifact-manifest.json')
            self.assertEqual(runner.extract_verified(archive, root / 'out'), manifest)

    def test_backup_corruption_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); data = root / 'model.pt'; data.write_bytes(b'test')
            (root / 'artifact-manifest.json').write_text(json.dumps({'model.pt': {'size': 4, 'sha256': 'wrong'}}))
            archive = root / 'results.tgz'
            with tarfile.open(archive, 'w:gz') as tar:
                tar.add(data, arcname='model.pt')
                tar.add(root / 'artifact-manifest.json', arcname='artifact-manifest.json')
            with self.assertRaisesRegex(RuntimeError, 'verification failed'):
                runner.extract_verified(archive, root / 'out')

    def test_backup_separates_checkpoints_and_metrics(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / 'files/source/isaac_lab/runs/yertle_flat/test-run'
            source.mkdir(parents=True)
            (source / 'model_299.pt').write_bytes(b'checkpoint')
            (source / 'events.out').write_bytes(b'events')
            copied = runner.organize_backup(root)
            self.assertIn('checkpoints/model_299.pt', copied)
            self.assertEqual((root / 'checkpoints/model_299.pt').read_bytes(), b'checkpoint')
            self.assertEqual((root / 'training_data/yertle_flat/test-run/events.out').read_bytes(), b'events')
            self.assertTrue((source / 'model_299.pt').exists())

    def test_archive_traversal_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); archive = root / 'results.tgz'
            with tarfile.open(archive, 'w:gz') as tar:
                info = tarfile.TarInfo('../outside'); info.size = 1
                tar.addfile(info, io.BytesIO(b'x'))
            with self.assertRaisesRegex(RuntimeError, 'Unsafe'):
                runner.extract_verified(archive, root / 'out')

    def test_cleanup_only_recorded_ids_and_checks_readback(self):
        calls = []
        def fake_api(method, path, body=None):
            calls.append((method, path))
            if method == 'GET':
                raise runner.ApiError(404, 'not found')
        with tempfile.TemporaryDirectory() as folder, patch.object(runner, 'api', side_effect=fake_api):
            state = {'pod_id': 'created-pod', 'volume_id': 'created-volume'}
            runner.cleanup(state, Path(folder) / 'state.json', delete_volume=True)
            self.assertEqual(calls, [('DELETE', '/pods/created-pod'), ('GET', '/pods/created-pod'),
                                     ('DELETE', '/network-volumes/created-volume'), ('GET', '/network-volumes/created-volume')])
            self.assertNotIn('pod_id', state)
            self.assertNotIn('volume_id', state)

    def test_unconfirmed_delete_is_failure(self):
        with patch.object(runner, 'api', return_value={'id': 'still-there'}):
            with self.assertRaisesRegex(RuntimeError, 'Deletion not confirmed'):
                runner.delete_confirm('/pods/created-pod')

    def test_list_pagination(self):
        pages = [{'pods': [{'id': 'a'}], 'pagination': {'hasNextPage': True, 'nextCursor': 'c'}},
                 {'pods': [{'id': 'b'}], 'pagination': {'hasNextPage': False}}]
        with patch.object(runner, 'api', side_effect=pages) as mocked:
            self.assertEqual(runner.list_all('/pods', 'pods'), [{'id': 'a'}, {'id': 'b'}])
            self.assertEqual(mocked.call_args_list[1].args[1], '/pods?cursor=c')

    def test_independent_jobs_can_hold_locks_at_once(self):
        with tempfile.TemporaryDirectory() as folder:
            first, second = Path(folder) / 'one', Path(folder) / 'two'
            first.mkdir(); second.mkdir()
            with runner.job_lock(first), runner.job_lock(second):
                self.assertTrue((first / 'controller.lock').exists())
                with self.assertRaises(FileExistsError):
                    with runner.job_lock(first):
                        pass
            self.assertFalse((first / 'controller.lock').exists())

    def test_copy_failure_deletes_only_gpu_not_volume(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(runner, 'delete_confirm') as delete:
            state = {'pod_id': 'own-pod', 'volume_id': 'own-volume', 'phase': 'working'}
            path = Path(folder) / 'state.json'
            runner.retain_volume(state, path, OSError('local disk unavailable'))
            delete.assert_called_once_with('/pods/own-pod')
            stored = json.loads(path.read_text())
            self.assertEqual(stored['volume_id'], 'own-volume')
            self.assertEqual(stored['phase'], 'volume_retained')
            self.assertNotIn('pod_id', stored)
            self.assertNotIn('backup_verified', stored)

    def test_failed_gpu_delete_remains_actionable(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(runner, 'delete_confirm', side_effect=RuntimeError('offline')):
            state = {'pod_id': 'own-pod', 'volume_id': 'own-volume'}
            path = Path(folder) / 'state.json'
            with self.assertRaisesRegex(RuntimeError, 'offline'):
                runner.retain_volume(state, path, OSError('copy failed'))
            self.assertEqual(json.loads(path.read_text())['phase'], 'cleanup_pending')
            self.assertEqual(state['pod_id'], 'own-pod')
            self.assertEqual(state['volume_id'], 'own-volume')

    def test_download_failure_never_marks_backup_verified(self):
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as folder:
            remote = Mock()
            remote.run.return_value = 'expected-hash'
            remote.copy.side_effect = OSError('disk full')
            config = dict(self.config, backup_dir=folder)
            state = {}
            with self.assertRaisesRegex(OSError, 'disk full'):
                runner.download_results(remote, config, state, Path(folder) / 'state.json')
            self.assertNotIn('backup_verified', state)

    def test_recovery_rejects_active_job_before_api(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'state.json'
            with patch.object(runner, 'job_state', return_value=(path, {'phase': 'working'})), patch.object(runner, 'api') as api:
                with self.assertRaisesRegex(RuntimeError, 'active jobs'):
                    runner.recover_job('ignored-by-mock', self.config)
                api.assert_not_called()

    def test_job_id_cannot_escape_owned_state_directory(self):
        with self.assertRaises(ValueError):
            runner.job_state('../../foreign-resource')

    def test_archive_is_on_persistent_volume_not_tmp(self):
        module_spec = importlib.util.spec_from_file_location('remote_persistent_test', Path(__file__).with_name('remote_job.py'))
        remote = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(remote)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / 'yertle-job'
            root.mkdir()
            (root / 'job.json').write_text('{}')
            with patch.object(remote, 'ROOT', root):
                remote.archive()
            archive = root.parent / 'yertle-result.tgz'
            self.assertTrue(archive.is_file())
            with tarfile.open(archive) as tar:
                self.assertNotIn('yertle-result.tgz', tar.getnames())

    def test_controller_completion_and_copy_failure_cleanup_paths(self):
        """Exercise full run_job orchestration, not just the cleanup helper."""
        from types import SimpleNamespace
        from unittest.mock import Mock
        for copy_fails in (False, True):
            with self.subTest(copy_fails=copy_fails), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                key = root / 'key'; key.write_text('test-only')
                (root / 'key.pub').write_text('ssh-ed25519 TEST_ONLY')
                config = dict(self.config, ssh_key=str(key), backup_dir=str(root / 'backup'))
                args = SimpleNamespace(dry_run=False, include_dirty=True, resume_checkpoint=None)
                item = {'gpu': config['gpu_ids'][0], 'dc': 'one', 'gpu_hourly': .5, 'stock': 'LOW'}
                calls, deleted = [], set()
                def fake_api(method, path, body=None):
                    calls.append((method, path))
                    if path == '/catalog/datacenters':
                        return {'dataCenters': self.centers}
                    if method == 'POST' and path == '/network-volumes':
                        return {'id': 'test-volume'}
                    if method == 'POST' and path == '/pods':
                        return {'id': 'test-pod'}
                    if method == 'DELETE':
                        deleted.add(path); return
                    if path in deleted:
                        raise runner.ApiError(404, 'deleted')
                    if path == '/network-volumes/test-volume':
                        return {'id': 'test-volume'}
                    if path == '/pods/test-pod':
                        return {'gpu': {'id': item['gpu'], 'count': 1}, 'image': config['image'],
                                'mounts': {'network': [{'volumeId': 'test-volume'}]}}
                    raise AssertionError((method, path))
                def fake_snapshot(config, jobdir, dirty, resume):
                    payload = jobdir / 'payload.tgz'; payload.write_bytes(b'payload')
                    return {'commit': 'test-commit'}, payload
                remote = Mock()
                def remote_run(command, **kwargs):
                    if command == 'printf YERTLE_SSH_READY': return 'YERTLE_SSH_READY'
                    if command.startswith('sha256sum'): return runner.hashlib.sha256(b'payload').hexdigest()
                    if command.startswith('cat '): return '{"phase":"finished"}'
                    return ''
                remote.run.side_effect = remote_run
                def fake_download(remote, config, state, path):
                    if copy_fails: raise OSError('simulated local disk failure')
                    state.update(backup_verified=True, checkpoint_names=['model_299.pt'])
                    runner.save(path, state)
                with patch.object(runner, 'HERE', root), patch.object(runner, 'api', side_effect=fake_api), \
                     patch.object(runner, 'list_all', return_value=[self.gpu]), \
                     patch.object(runner, 'snapshot', side_effect=fake_snapshot), \
                     patch.object(runner, 'get_remote', return_value=remote), \
                     patch.object(runner, 'download_results', side_effect=fake_download), \
                     patch('builtins.print'):
                    if copy_fails:
                        with self.assertRaises(OSError): runner.run_job(config, args)
                    else:
                        runner.run_job(config, args)
                state = json.loads(next((root / '.state').glob('*/state.json')).read_text())
                self.assertIn('/pods/test-pod', deleted)
                if copy_fails:
                    self.assertNotIn('/network-volumes/test-volume', deleted)
                    self.assertEqual(state['phase'], 'volume_retained')
                else:
                    self.assertIn('/network-volumes/test-volume', deleted)
                    self.assertEqual(state['phase'], 'complete')

    def test_retained_volume_recovery_success_failure_and_unavailable_region(self):
        from unittest.mock import Mock
        for outcome in ('success', 'copy_failed', 'no_capacity'):
            with self.subTest(outcome=outcome), tempfile.TemporaryDirectory() as folder:
                root = Path(folder); job_id = '20261017T000000Z-abcdef'
                jobdir = root / '.state' / job_id; jobdir.mkdir(parents=True)
                statefile = jobdir / 'state.json'
                state = {'phase': 'volume_retained', 'volume_id': 'retained',
                         'placement': {'gpu': self.config['gpu_ids'][0], 'dc': 'one', 'gpu_hourly': .5},
                         'pod_template': {'gpu': {'id': self.config['gpu_ids'][0]},
                                          'mounts': {'network': [{'volumeId': 'retained', 'path': '/workspace'}]}}}
                runner.save(statefile, state); runner.save(jobdir / 'job.json', self.config)
                deleted, posts = set(), []
                def fake_api(method, path, body=None):
                    if method == 'DELETE': deleted.add(path); return
                    if path in deleted: raise runner.ApiError(404, 'deleted')
                    if method == 'POST': posts.append(path); return {'id': 'recovery-pod'}
                    if path == '/catalog/datacenters': return {'dataCenters': self.centers}
                    if path == '/network-volumes/retained': return {'id': 'retained'}
                    if path == '/pods/recovery-pod': return {'mounts': {'network': [{'volumeId': 'retained'}]}}
                    raise AssertionError(path)
                remote = Mock()
                # Stale status from interrupted training is recoverable on a NEW Pod.
                remote.run.side_effect = lambda cmd, **kwargs: '{"phase":"training"}' if cmd.startswith('cat ') else 'READY'
                def fake_download(remote, config, state, path):
                    if outcome == 'copy_failed': raise OSError('disk offline')
                    state['backup_verified'] = True; runner.save(path, state)
                with patch.object(runner, 'HERE', root), patch.object(runner, 'api', side_effect=fake_api), \
                     patch.object(runner, 'list_all', return_value=[] if outcome == 'no_capacity' else [self.gpu]), \
                     patch.object(runner, 'get_remote', return_value=remote), \
                     patch.object(runner, 'download_results', side_effect=fake_download), patch('builtins.print'):
                    if outcome == 'success': runner.recover_job(job_id, self.config)
                    else:
                        with self.assertRaises((OSError, RuntimeError)): runner.recover_job(job_id, self.config)
                state = json.loads(statefile.read_text())
                if outcome == 'no_capacity':
                    self.assertEqual(posts, [])
                    self.assertEqual(deleted, set())
                else:
                    self.assertIn('/pods/recovery-pod', deleted)
                if outcome == 'success':
                    self.assertIn('/network-volumes/retained', deleted)
                    self.assertEqual(state['phase'], 'recovered')
                else:
                    self.assertNotIn('/network-volumes/retained', deleted)
                    self.assertEqual(state['phase'], 'volume_retained')


if __name__ == '__main__':
    unittest.main()
