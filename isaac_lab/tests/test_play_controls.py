"""Exercise production play-only controls without launching Isaac Sim."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import torch

spec = importlib.util.spec_from_file_location("controls", Path(__file__).resolve().parents[1] / "play_controls.py")
controls = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controls)


class PlayControlsTests(unittest.TestCase):
    def test_base_contact_only_is_disabled(self):
        timeout, other = object(), object()
        cfg = SimpleNamespace(terminations=SimpleNamespace(base_contact=object(),time_out=timeout,other=other))
        controls.configure_play_terminations(cfg)
        self.assertIsNone(cfg.terminations.base_contact)
        self.assertIs(cfg.terminations.time_out, timeout)
        self.assertIs(cfg.terminations.other, other)

    def test_reset_key_is_distinct_from_stop_and_quit(self):
        command = [0.2, 0.1, 0.3]
        for key in ("r", "R"):
            self.assertEqual(controls.handle_key(key,command,.05,.1),(False,False,None,None,True))
        self.assertEqual(command,[0.2,0.1,0.3])
        self.assertEqual(controls.handle_key(" ",command,.05,.1),(True,False,None,None,False))
        self.assertEqual(command,[0,0,0])
        for key in ("x","\x1b"):
            self.assertEqual(controls.handle_key(key,command,.05,.1),(False,True,None,None,False))

    def test_motion_push_strength_and_unknown_keys_preserved(self):
        for key, index, value in (("w",0,.05),("s",0,-.05),("q",1,.05),("e",1,-.05),("a",2,.1),("d",2,-.1)):
            command=[0.,0.,0.]
            self.assertEqual(controls.handle_key(key,command,.05,.1),(True,False,None,None,False))
            self.assertAlmostEqual(command[index],value)
        command=[.3,.2,1.]
        for key in ("w","q","a"): controls.handle_key(key,command,.05,.1)
        self.assertEqual(command,[.3,.2,1.])
        for key, push in controls.PUSH_KEYS.items():
            self.assertEqual(controls.handle_key(key,command,.05,.1),(False,False,push,None,False))
        for key, delta in (("[",-.1),("]",.1)):
            self.assertEqual(controls.handle_key(key,command,.05,.1),(False,False,None,delta,False))
        for key in (None,"?"):
            self.assertEqual(controls.handle_key(key,command,.05,.1),(False,False,None,None,False))

    def test_reset_clears_force_commands_and_returns_fresh_observations(self):
        term=SimpleNamespace(vel_command_b=torch.ones(2,3), is_standing_env=torch.ones(2,dtype=torch.bool),
                             is_heading_env=torch.ones(2,dtype=torch.bool),time_left=torch.zeros(2))
        base=SimpleNamespace(command_manager=SimpleNamespace(get_term=lambda name:term))
        command=[.2,.1,.5]
        tensor=torch.ones(2,3)
        pulse=SimpleNamespace(remaining=5,force_w=torch.ones(2,3),apply=Mock())
        def reset():
            self.assertEqual(pulse.remaining,0)
            self.assertTrue(torch.equal(pulse.force_w,torch.zeros(2,3)))
            term.vel_command_b.fill_(.3)
        def observe():
            torch.testing.assert_close(term.vel_command_b,torch.zeros(2,3))
            return term.vel_command_b.clone()
        env=SimpleNamespace(unwrapped=base,reset=Mock(side_effect=reset),get_observations=Mock(side_effect=observe))
        obs=controls.reset_play_environment(env,pulse,command,tensor)
        env.reset.assert_called_once()
        env.get_observations.assert_called_once()
        self.assertEqual(pulse.apply.call_count,2)
        self.assertEqual(command,[0,0,0])
        torch.testing.assert_close(obs,torch.zeros(2,3))
        self.assertFalse(term.is_heading_env.any())
        self.assertFalse(term.is_standing_env.any())
        torch.testing.assert_close(term.time_left,torch.full((2,),1e6))

    def test_cleanup_of_inference_force_tensor_outside_inference_mode(self):
        with torch.inference_mode():
            force = torch.ones(1, 3)
        pulse = SimpleNamespace(remaining=5, force_w=force, apply=Mock())
        controls.clear_manual_push(pulse)
        self.assertEqual(pulse.remaining, 0)
        torch.testing.assert_close(pulse.force_w, torch.zeros(1, 3))
        pulse.apply.assert_called_once()


if __name__ == "__main__":
    unittest.main()
