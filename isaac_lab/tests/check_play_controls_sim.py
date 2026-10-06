"""Run the real native player with bounded, injected terminal control events.

Use --checkpoint and --pose-json; verifies contact persistence and two R resets.
No hardware is controlled and no actual user terminal keypress is generated.
"""
import argparse
from pathlib import Path
import runpy
import sys
import traceback

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", required=True)
parser.add_argument("--pose-json", required=True)
parser.add_argument("--experience", default="isaaclab.python.yertle_min_d3d12.kit")
args = parser.parse_args()
source = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(source))
import play_controls as controls
from play_start_pose import load_pose_json

pose = load_pose_json(args.pose_json)
original_reset = controls.reset_play_environment
original_key = controls.handle_key
original_config = controls.configure_play_terminations
state = {"calls": 0, "resets": 0, "base": None, "contact": False, "nonterminated_steps": 0}


def configure(cfg):
    timeout = cfg.terminations.time_out
    original_config(cfg)
    assert cfg.terminations.base_contact is None
    assert cfg.terminations.time_out is timeout


def reset(env, pulse, command, tensor):
    import torch
    base = env.unwrapped
    robot = base.scene["robot"]
    defaults = robot.data.default_joint_pos.clone()
    pulse.trigger(1.0, 0.0, "verification")
    pulse.apply()
    command[:] = [0.2, 0.1, 0.3]
    tensor[:] = tensor.new_tensor(command)
    obs = original_reset(env, pulse, command, tensor)
    assert command == [0.0, 0.0, 0.0] and pulse.remaining == 0
    assert torch.count_nonzero(pulse.force_w).item() == 0
    torch.testing.assert_close(base.command_manager.get_command("base_velocity"), torch.zeros_like(tensor))
    torch.testing.assert_close(obs["policy"][:, 9:12], torch.zeros_like(tensor))
    assert torch.count_nonzero(base.episode_length_buf).item() == 0
    assert torch.count_nonzero(base.action_manager.action).item() == 0
    expected = robot.data.joint_pos.new_tensor([pose[name] for name in robot.joint_names]).unsqueeze(0)
    torch.testing.assert_close(robot.data.joint_pos, expected, atol=1e-5, rtol=0)
    torch.testing.assert_close(robot.data.default_joint_pos, defaults)
    assert "base_contact" not in base.termination_manager.active_terms
    state["base"] = base
    state["resets"] += 1
    # Put the body against the ground to exercise the formerly terminating contact.
    root_pose = robot.data.root_state_w[:, :7].clone()
    root_pose[:, 2] = 0.005
    robot.write_root_pose_to_sim(root_pose)
    robot.write_root_velocity_to_sim(torch.zeros_like(robot.data.root_state_w[:, 7:13]))
    print(f"PLAY_CONTROLS_SIM_PASS reset={state['resets']} JSON_pose/commands/forces/observations", flush=True)
    return obs


def key(_key, command, step_v, step_yaw):
    import torch
    state["calls"] += 1
    base = state["base"]
    if base is not None:
        assert "base_contact" not in base.termination_manager.active_terms
        assert not base.termination_manager.terminated.any()
        sensor = base.scene.sensors["contact_forces"]
        index = sensor.body_names.index("base_link")
        force = torch.linalg.vector_norm(sensor.data.net_forces_w_history[:, :, index, :], dim=-1).max().item()
        if force > 1.0:
            state["contact"] = True
        if base.episode_length_buf[0].item() > 0:
            state["nonterminated_steps"] += 1
    # Kit may terminate Python during app.close(); assert BEFORE player shutdown.
    if state["calls"] == 40:
        assert state["resets"] == 2, state
        assert state["contact"], "No measured base contact during bounded test"
        assert state["nonterminated_steps"] >= 20, state
        print("PLAY_CONTROLS_SIM_PASS measured_base_contact_without_termination; real_play_loop; two_R_resets", flush=True)
    simulated = {3:"r", 6:"w", 15:"r"}.get(state["calls"])
    return original_key(simulated, command, step_v, step_yaw)


controls.configure_play_terminations = configure
controls.reset_play_environment = reset
controls.handle_key = key
sys.argv = [str(source / "play.py"), "--checkpoint", args.checkpoint,
            "--pose-json", args.pose_json, "--extend-start-pose-limits",
            "--task", "flat", "--num_envs", "1", "--steps", "40", "--keyboard",
            "--headless", "--rendering_mode", "performance", "--experience", args.experience]
try:
    runpy.run_path(str(source / "play.py"), run_name="__main__")
except BaseException:
    traceback.print_exc()
    print("PLAY_CONTROLS_SIM_FAIL", flush=True)
    raise
