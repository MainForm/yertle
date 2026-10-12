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
        self.assertEqual(robot.init_state.joint_pos,{'.*_shoulder':0.,'.*_thigh':math.radians(34),'.*_shin':math.radians(-85)})
        self.assertTrue(action.use_default_offset)
        self.assertEqual(action.scale,.25)

    def test_standing_json_matches_all_reference_joints(self):
        import json
        data=json.loads((Path(__file__).parents[1]/'configs/standing_api_reference.json').read_text(encoding='utf-8'))
        for name,radians,degrees in zip(data['joint_names'],data['angles_rad'],data['angles_deg']):
            kind=name.rsplit('_',1)[1]
            self.assertEqual(degrees,module.REFERENCE_DEG[kind])
            self.assertAlmostEqual(radians,math.radians(module.REFERENCE_DEG[kind]),places=12)

    def test_new_compose_template_aligns_train_and_play_references(self):
        text=(Path(__file__).parents[1]/'train_in_docker_container/.env.example').read_text(encoding='utf-8')
        for key in ('TRAIN_EXTRA_ARGS','PLAY_EXTRA_ARGS'):
            value=next(line for line in text.splitlines() if line.startswith(key+'='))
            self.assertIn('--signed-joint-reference',value)


if __name__=='__main__':unittest.main()
