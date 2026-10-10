import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('signed_joint_reference',Path(__file__).parents[1]/'signed_joint_reference.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class ReferenceTests(unittest.TestCase):
    def test_shared_reference_and_legacy_preservation(self):
        robot=SimpleNamespace(spawn=SimpleNamespace(usd_path='legacy.usd'),init_state=SimpleNamespace(joint_pos={'old':123}))
        action=SimpleNamespace(use_default_offset=False,scale=.25)
        cfg=SimpleNamespace(scene=SimpleNamespace(robot=robot),actions=SimpleNamespace(joint_pos=action))
        with patch.object(module,'create_signed_usd',return_value='signed.usda') as creator:
            module.configure_signed_joint_reference(cfg)
        creator.assert_called_once_with('legacy.usd')
        self.assertEqual(robot.spawn.usd_path,'signed.usda')
        self.assertEqual(robot.init_state.joint_pos,{'.*_shoulder':0.,'.*_thigh':math.radians(34.4),'.*_shin':math.radians(-51.6)})
        self.assertTrue(action.use_default_offset)
        self.assertEqual(action.scale,.25)


if __name__=='__main__':unittest.main()
