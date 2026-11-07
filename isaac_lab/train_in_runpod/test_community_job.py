"""Offline-only checks; no GPU is rented."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import community_job as community


class CommunityJobTests(unittest.TestCase):
    def config(self):
        return dict(max_gpu_hourly_usd=.25, max_planned_usd=1.7, max_seconds=36000,
                    container_disk_gb=80, persistent_disk_gb=10, disk_monthly_usd_per_gb=.1)

    def test_planning_budget_includes_both_disks(self):
        config = self.config()
        end = community.deadline(100, config, .19)
        hourly = .19 + 90 * .1 / 730
        self.assertAlmostEqual((end - 100) / 3600 * hourly, 1.7)

    def test_time_cap_is_not_ignored(self):
        config = self.config(); config['max_seconds'] = 600
        self.assertEqual(community.deadline(100, config, .19), 700)

    def test_expensive_gpu_not_silently_accepted(self):
        with self.assertRaises(ValueError): community.deadline(0, self.config(), .89)

    def test_failed_backup_stops_instead_of_deleting_host_disk(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'state.json'
            state = {'pod_id': 'own-created-pod'}
            with patch.object(community.run, 'api', side_effect=[None, {'status': 'EXITED', 'cost': 0}]) as api:
                community.stop_preserving_disk(state, path, OSError('local disk offline'))
            self.assertEqual(state['phase'], 'stopped_persistent_disk_retained')
            self.assertEqual(api.call_args_list[0].args,
                             ('POST', '/pods/own-created-pod/action', {'action': 'stop'}))
            self.assertTrue(all(call.args[0] != 'DELETE' for call in api.call_args_list))


if __name__ == '__main__':
    unittest.main()
