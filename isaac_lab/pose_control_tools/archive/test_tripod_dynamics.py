"""Replay actual CLI raw targets in free-base physics; never connects hardware.

No changes to calibrated axes, mapping, source USD, joint bounds, gains or
training configuration. Incompatible joint bounds fail preflight explicitly.
No root pose/state is rewritten after initialization; all later motion is PhysX.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import h5py  # noqa: F401 -- native Isaac DLL load ordering
import numpy as np
import torch
from isaaclab.app import AppLauncher

SOURCE = Path(__file__).resolve().parents[2]
ROOT = SOURCE.parents[1]
TASK = ROOT / "measurements/tripod_dynamic_20261020"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source-log", type=Path, default=ROOT / "measurements/tripod_high_cli_20261020/deployed-complete.jsonl")
parser.add_argument("--inverse-exe", type=Path, default=TASK / "build/Release/tripod_raw_inverse.exe")
parser.add_argument("--output-dir", type=Path, default=TASK / "physics_original")
parser.add_argument("--inspect-only", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.output_dir.exists():
    parser.error("Refusing to overwrite diagnostics directory")
args.output_dir.mkdir(parents=True)

records = [json.loads(line) for line in args.source_log.read_text(encoding="utf-8-sig").splitlines()]
plans = [r for r in records if r.get("kind") == "tripod_lift_plan"]
frames = [r for r in records if r.get("kind") == "tripod_lift_step"]
if len(plans) != 1 or plans[0].get("profile") != "half_body" or len(frames) != 951:
    raise ValueError("Expected exactly one complete 951-frame high-tripod CLI log")
for index, frame in enumerate(frames):
    if frame["index"] != index or abs(frame["nominal_time_s"] - index * .02) > 1e-5:
        raise ValueError("CLI trajectory has a missing/rebased/duplicate frame")
payload = "\n".join(" ".join(map(str, f["target_raw"])) for f in frames) + "\n"
conversion = subprocess.run([str(args.inverse_exe)], input=payload, text=True,
                            capture_output=True, check=True, timeout=20)
lines = conversion.stdout.splitlines()
names = json.loads(lines[0])["joint_names"]
targets = np.asarray([json.loads(line) for line in lines[1:]], dtype=float)
if targets.shape != (951, 12) or not np.isfinite(targets).all():
    raise ValueError("Frozen actual API conversion failed")
report = {
    "source_log": str(args.source_log),
    "source_log_sha256": hashlib.sha256(args.source_log.read_bytes()).hexdigest(),
    "raw_inverse_exe": str(args.inverse_exe),
    "conversion": "Frozen deployed C++ PolicyRawConverter, all951 raw roundtrips verified",
    "frames": 951, "control_dt_s": .02, "trajectory_duration_s": 19,
    "root_fixed": None, "source_or_calibration_modified": False,
    "motor_model_caveat": "Existing implicit PhysX drive, not a calibrated STS3215 firmware/Speed/ACC/PID model",
}


def save_report():
    (args.output_dir / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("TRIPOD_DYNAMICS_REPORT " + json.dumps(report), flush=True)


launcher = AppLauncher(args)
app = launcher.app
sim = None
try:
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sensors import ContactSensor, ContactSensorCfg
    sys.path.insert(0, str(SOURCE))
    from isaac_lab.yertle_cfg import YERTLE_CFG

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=.005, device=args.device))
    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/Ground", ground)
    light = sim_utils.DomeLightCfg(intensity=3000)
    light.func("/World/Light", light)
    sim.set_camera_view(eye=(.6, -.7, .35), target=(0, 0, .12))
    robot = Articulation(YERTLE_CFG.replace(prim_path="/World/Yertle"))
    contacts = ContactSensor(ContactSensorCfg(prim_path="/World/Yertle/.*", update_period=0, history_length=1))
    sim.reset()
    order = [names.index(n) for n in robot.joint_names]
    targets = targets[:, order]
    limits = robot.root_physx_view.get_dof_limits()[0].cpu().numpy()
    report.update({
        "joint_names": robot.joint_names,
        "joint_limits_rad": limits.tolist(),
        "start_angles_deg": np.degrees(targets[0]).tolist(),
        "target_min_deg": np.degrees(targets.min(axis=0)).tolist(),
        "target_max_deg": np.degrees(targets.max(axis=0)).tolist(),
        "effort_limit_sim_nm": YERTLE_CFG.actuators["legs"].effort_limit_sim,
        "velocity_limit_sim_rad_s": YERTLE_CFG.actuators["legs"].velocity_limit_sim,
        "stiffness": YERTLE_CFG.actuators["legs"].stiffness,
        "damping": YERTLE_CFG.actuators["legs"].damping,
        "root_fixed": bool(robot.is_fixed_base),
        "self_collisions_enabled": YERTLE_CFG.spawn.articulation_props.enabled_self_collisions,
        "usd_path": YERTLE_CFG.spawn.usd_path,
    })
    violations = []
    for j, name in enumerate(robot.joint_names):
        bad = np.where((targets[:, j] < limits[j, 0] - 1e-5) | (targets[:, j] > limits[j, 1] + 1e-5))[0]
        if len(bad):
            violations.append({"joint": name, "invalid_frames": len(bad), "first_frame": int(bad[0]),
                               "limit_deg": np.degrees(limits[j]).tolist(),
                               "requested_start_deg": math.degrees(targets[0, j]),
                               "requested_min_max_deg": np.degrees([targets[:, j].min(), targets[:, j].max()]).tolist()})
    report["joint_limit_violations"] = violations
    if violations or robot.is_fixed_base:
        report["status"] = "incompatible_joint_limits" if violations else "fixed_base_cannot_test_balance"
        report["physics_trajectory_executed"] = False
        report["conclusion"] = "Cannot honestly test this CLI posture with the current unchanged model; no joint limits silently widened."
        save_report()
    elif args.inspect_only:
        report["status"] = "preflight_passed_only"
        report["physics_trajectory_executed"] = False
        save_report()
    else:
        q0 = torch.tensor(targets[0:1], dtype=torch.float32, device=robot.device)
        root = robot.data.default_root_state.clone()
        root[:, :3] = torch.tensor([0, 0, plans[0]["body_reference_height_m"] + .003], device=robot.device)
        root[:, 3:7] = torch.tensor([1, 0, 0, 0], device=robot.device)
        root[:, 7:] = 0
        robot.write_root_state_to_sim(root)
        robot.write_joint_state_to_sim(q0, torch.zeros_like(q0))
        robot.set_joint_position_target(q0)
        robot.write_data_to_sim()
        # No further state writes, root fixing, gravity cancellation or assistance.
        timeline = []
        base_index = contacts.body_names.index("base_link")
        foot_indices = [contacts.body_names.index(p + "_shin") for p in ("rf", "lf", "rb", "lb")]
        settle_s, tail_s = 3.0, 2.0
        first_guard, first_fall, reference_gravity = None, None, None
        total_steps = int((settle_s + 19 + tail_s) / .005)
        for step in range(total_steps):
            time_s = step * .005
            motion_s = time_s - settle_s
            frame_index = min(950, max(0, int((motion_s + 1e-8) / .02)))
            target = torch.tensor(targets[frame_index:frame_index+1], dtype=torch.float32, device=robot.device)
            robot.set_joint_position_target(target)
            robot.write_data_to_sim()
            sim.step(render=not args.headless and step % 4 == 0)
            robot.update(.005)
            contacts.update(.005)
            if step % 4:
                continue
            gravity = robot.data.projected_gravity_b[0].cpu().numpy()
            tilt = math.degrees(math.acos(float(np.clip(-gravity[2], -1, 1))))
            if reference_gravity is None and motion_s >= 0:
                reference_gravity = gravity.copy()
                report["settled_start_tilt_deg"] = tilt
            relative = 0 if reference_gravity is None else math.degrees(math.acos(float(np.clip(np.dot(reference_gravity, gravity), -1, 1))))
            forces = torch.linalg.vector_norm(contacts.data.net_forces_w[0], dim=-1).cpu().tolist()
            position = robot.data.root_pos_w[0].cpu().tolist()
            row = {"time_s": time_s, "motion_s": motion_s, "frame": frame_index,
                   "phase": "settle" if motion_s < 0 else frames[frame_index]["phase"],
                   "root_pos_m": position, "tilt_deg": tilt, "relative_tilt_deg": relative,
                   "base_contact_n": forces[base_index], "shin_contact_n": [forces[i] for i in foot_indices],
                   "max_joint_error_rad": float(torch.max(torch.abs(robot.data.joint_pos-target)).item()),
                   "max_joint_speed_rad_s": float(torch.max(torch.abs(robot.data.joint_vel)).item())}
            timeline.append(row)
            if motion_s >= 0 and relative > 8 and first_guard is None:
                first_guard = row
            if tilt > 45 or position[2] < .08:
                if first_fall is None:
                    first_fall = row
                if motion_s > 0:
                    break
        (args.output_dir / "timeline.json").write_text(json.dumps(timeline), encoding="utf-8")
        report.update({"status": "fell" if first_fall else "completed_no_fall_observed",
                       "physics_trajectory_executed": True, "samples": len(timeline),
                       "max_tilt_deg": max(r["tilt_deg"] for r in timeline),
                       "max_joint_error_rad": max(r["max_joint_error_rad"] for r in timeline),
                       "max_base_contact_n": max(r["base_contact_n"] for r in timeline),
                       "first_cli_8deg_guard": first_guard, "first_fall": first_fall,
                       "all_951_frames_reached": timeline[-1]["frame"] == 950,
                       "replay_mode": "unassisted open-loop diagnostic; CLI guard crossings logged, not suppressed"})
        save_report()
finally:
    if sim is not None:
        sim.clear_instance()
    app.close(wait_for_replicator=False)
