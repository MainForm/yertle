"""One simulation-only pose editor and unassisted tripod replay entry point.

Reuse play.py's committed reference/position-limit/reset helpers. No policy,
serial connection, hardware API mutation or automatic fall reset. Static mode
holds the body; physics mode never writes root state after an explicit reset.
"""
import argparse
import asyncio
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import time
import traceback
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
DATA_PATH = HERE / "pose_control_tools/data/pose_targets.json"
PRESETS = ("standing", "seated", "tripod-ready", "tripod-low", "tripod-high")


def load_targets(path=DATA_PATH):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    required = {p + "_" + k for p in ("rf", "lf", "rb", "lb") for k in ("shoulder", "thigh", "shin")}
    names = data.get("joint_names", [])
    if data.get("schema_version") != 1 or len(names) != 12 or set(names) != required:
        raise ValueError("Invalid named-joint pose targets")
    for preset in PRESETS:
        validate_angles(data["presets"][preset]["angles_rad"])
    for key, count in (("tripod-low", 651), ("tripod-high", 951)):
        trajectory = data["trajectories"][key]
        if trajectory["period_s"] != .02 or len(trajectory["frames"]) != count:
            raise ValueError("Not the complete CLI trajectory: " + key)
        for frame in trajectory["frames"]:
            validate_angles(frame["angles_rad"])
        if trajectory["frames"][0]["angles_rad"] != trajectory["frames"][-1]["angles_rad"]:
            raise ValueError("CLI trajectory does not return exactly")
    return data


def validate_angles(angles):
    if len(angles) != 12 or any(type(x) not in (int, float) or not math.isfinite(x) or abs(x) > math.pi for x in angles):
        raise ValueError("Expected 12 finite simulation angles within +/-180 degrees")


def select_frame(trajectory, elapsed_s, settle_s=3.):
    if elapsed_s < settle_s:
        return 0, "settle", False
    frame = min(len(trajectory["frames"])-1, int((elapsed_s-settle_s+1e-8)/trajectory["period_s"]))
    complete = elapsed_s >= settle_s + (len(trajectory["frames"])-1)*trajectory["period_s"] + 2.
    return frame, trajectory["frames"][frame]["phase"], complete


def foot_world(robot):
    from pxr import Gf
    links = robot.root_physx_view.get_link_transforms()[0].cpu().tolist()
    feet = {}
    for prefix in ("rf", "lf", "rb", "lb"):
        link = links[robot.body_names.index(prefix + "_shin")]
        point = Gf.Vec3d(-.0008145190142713589, -.0018949970081173842 + (-.007 if prefix in ("rf", "rb") else .008), -.13366482367025265)
        feet[prefix] = list(Gf.Vec3d(*link[:3]) + Gf.Rotation(Gf.Quatd(link[6], Gf.Vec3d(*link[3:6]))).TransformDir(point))
    return feet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("static", "physics"), default="static")
    parser.add_argument("--pose", choices=(*PRESETS, "json"), default=None)
    parser.add_argument("--pose-json", type=Path)
    parser.add_argument("--trajectory", choices=("tripod-low", "tripod-high"))
    parser.add_argument("--settle-seconds", type=float, default=3.)
    parser.add_argument("--exit-on-complete", action="store_true")
    parser.add_argument("--seconds", type=float, default=0, help="0: keep GUI open")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--targets", type=Path, default=DATA_PATH)
    parser.add_argument("--verify-view", action="store_true", help="GUI static comparison capture and verification")
    import h5py  # noqa: F401 -- same native DLL ordering as play.py
    import torch
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.pose = args.pose or ("json" if args.pose_json else "standing")
    if (args.pose == "json") != bool(args.pose_json):
        parser.error("Use --pose-json with --pose json (or omit --pose)")
    if args.trajectory and (args.mode != "physics" or args.pose != "standing"):
        parser.error("--trajectory requires --mode physics and standing start")
    if args.exit_on_complete and not args.trajectory:
        parser.error("--exit-on-complete requires --trajectory")
    if args.seconds < 0 or not math.isfinite(args.seconds) or args.frames < 0 or not 0 <= args.settle_seconds <= 30:
        parser.error("Invalid run duration")
    if args.verify_view and (args.headless or args.mode != "static"):
        parser.error("--verify-view requires GUI static mode")
    if args.headless and not (args.seconds or args.frames or args.exit_on_complete):
        parser.error("Bound headless runs using --seconds/--frames/--exit-on-complete")
    data = load_targets(args.targets)
    output = args.output_dir or HERE / "runs/pose_control" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    if output.exists():
        parser.error("Refusing to overwrite existing output directory")
    output.mkdir(parents=True)
    launcher = AppLauncher(args)
    app = launcher.app
    sim = None
    try:
        sys.path.insert(0, str(HERE.parent))
        import omni.ui as ui
        import omni.kit.app
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation
        from isaaclab.sensors import ContactSensor, ContactSensorCfg
        from isaacsim.core.prims import XFormPrim
        from isaac_lab.flat_env_cfg import YertleFlatEnvCfg_PLAY
        from isaac_lab.signed_joint_reference import configure_signed_joint_reference
        from isaac_lab.joint_position_limits import release_joint_position_limits
        from isaac_lab.play_start_pose import load_pose_json, reset_selected_pose

        cfg = YertleFlatEnvCfg_PLAY()
        cfg.scene.num_envs = 1
        cfg.sim.device = args.device
        cfg.sim.use_fabric = args.mode == "physics"
        configure_signed_joint_reference(cfg)  # Same committed path as play.py's flag.
        sim = sim_utils.SimulationContext(cfg.sim)
        ground = sim_utils.GroundPlaneCfg(physics_material=cfg.scene.terrain.physics_material)
        ground.func("/World/Ground", ground)
        light = sim_utils.DomeLightCfg(intensity=3000)
        light.func("/World/Light", light)
        robot = Articulation(cfg.scene.robot.replace(prim_path="/World/Yertle"))
        contacts = ContactSensor(ContactSensorCfg(prim_path="/World/Yertle/.*", update_period=0., history_length=1))
        sim.reset()
        facade = SimpleNamespace(scene={"robot": robot}, device=robot.device, num_envs=1)
        release_joint_position_limits(facade)
        limits = robot.root_physx_view.get_dof_limits()[0]
        if not ((limits[:, 0] < -1e6) & (limits[:, 1] > 1e6)).all():
            raise RuntimeError("Committed PhysX position-limit release did not apply")
        if robot.is_fixed_base:
            raise RuntimeError("Model has a fixed base; cannot test free-body physics")
        order = [data["joint_names"].index(n) for n in robot.joint_names]
        static_links = XFormPrim(prim_paths_expr=["/World/Yertle/"+n for n in robot.body_names], reset_xform_properties=False) if args.mode == "static" else None
        render_order = [robot.body_names.index(Path(p).name) for p in static_links.prim_paths] if static_links else []
        requests = deque()
        models = {}
        syncing = False
        selected = args.pose
        goal = robot.data.default_joint_pos.clone()
        command = goal.clone()
        preset_shift = [0., 0., 0.]
        trajectory_name = args.trajectory
        elapsed = 0.
        phase = "hold"
        current_index = 0
        dirty = True
        quit_requested = False
        paused = False
        root_writes = 0
        reference_gravity = None
        timeline = []
        dt = sim.get_physics_dt()
        substeps = round(.02/dt)
        if abs(dt*substeps-.02) > 1e-8:
            raise RuntimeError("Physics dt is incompatible with the CLI20ms period")
        report = {"mode": args.mode, "pose": args.pose, "trajectory": args.trajectory,
                  "joint_names": list(robot.joint_names),
                  "trajectory_sources": {name: {"source": value["source"], "source_sha256": value["source_sha256"]}
                                         for name, value in data["trajectories"].items()},
                  "shared_play_helpers": ["configure_signed_joint_reference", "release_joint_position_limits", "load_pose_json", "reset_selected_pose"],
                  "physics_position_stops_released": 12, "torque_limit_nm": cfg.scene.robot.actuators["legs"].effort_limit_sim,
                  "speed_limit_rad_s": cfg.scene.robot.actuators["legs"].velocity_limit_sim,
                  "stiffness": cfg.scene.robot.actuators["legs"].stiffness, "damping": cfg.scene.robot.actuators["legs"].damping,
                  "device": args.device, "physics_dt_s": dt, "control_dt_s": .02,
                  "usd_path": cfg.scene.robot.spawn.usd_path, "original_axes_mapping_api_unchanged": True,
                  "self_collisions_enabled": cfg.scene.robot.spawn.articulation_props.enabled_self_collisions,
                  "policy_executed": False, "hardware_connected": False,
                  "automatic_resets": False, "root_fixed": False,
                  "root_held_kinematically": args.mode == "static",
                  "motor_model_caveat": "Uncalibrated implicit drive; not STS3215 firmware PID/Speed/ACC emulation"}

        window = ui.Window("Yertle pose control", width=570, height=820)
        with window.frame:
            with ui.VStack(spacing=5):
                ui.Label("SIMULATION ONLY | NO policy / NO hardware", height=24)
                ui.Label("Mode: " + ("STATIC - body fixed" if args.mode == "static" else "PHYSICS - free base, gravity/contact ON"), height=24)
                ui.Label("Original axes | reference0/+34/-85 | position stops released", height=24)
                with ui.HStack(height=32):
                    for label, name in (("Standing", "standing"), ("Seated", "seated"), ("Body shift", "tripod-ready")):
                        ui.Button(label, clicked_fn=lambda n=name: requests.append(("pose", n)))
                with ui.HStack(height=32):
                    ui.Button("RF25mm", clicked_fn=lambda: requests.append(("pose", "tripod-low")))
                    ui.Button("RF half body", clicked_fn=lambda: requests.append(("pose", "tripod-high")))
                if args.mode == "physics":
                    with ui.HStack(height=34):
                        ui.Button("Reset + Run25mm", clicked_fn=lambda: requests.append(("run", "tripod-low")))
                        ui.Button("Reset + Run half body", clicked_fn=lambda: requests.append(("run", "tripod-high")))
                with ui.ScrollingFrame():
                    with ui.VStack(spacing=4):
                        for name in robot.joint_names:
                            model = ui.SimpleFloatModel(0., min=-180., max=180.)
                            models[name] = model
                            with ui.HStack(height=30):
                                ui.Label(name, width=125)
                                ui.FloatSlider(model, min=-180., max=180., step=.1, format="%.2f deg")
                                ui.FloatField(model, width=72, precision=2)
                            def change(m, n=name):
                                if not syncing:
                                    requests.append(("joint", (n, m.as_float)))
                            model.add_value_changed_fn(change)
                path_model = ui.SimpleStringModel(str(args.pose_json) if args.pose_json else "")
                ui.StringField(path_model, height=28)
                with ui.HStack(height=30):
                    ui.Button("Load JSON", clicked_fn=lambda: requests.append(("load", path_model.as_string)))
                    ui.Button("Save target JSON", clicked_fn=lambda: requests.append(("save", None)))
                with ui.HStack(height=30):
                    ui.Button("R: Reset", clicked_fn=lambda: requests.append(("reset", None)))
                    ui.Button("Pause/resume", clicked_fn=lambda: requests.append(("pause", None)))
                    ui.Button("X: Exit", clicked_fn=lambda: requests.append(("quit", None)))
                status = ui.Label("Loading...", height=115, word_wrap=True)
                ui.Label("Sliders: deg; JSON: rad; UI +/-180 is NOT a motor safety bound.", height=28)

        def sync_models():
            nonlocal syncing
            syncing = True
            try:
                for i, name in enumerate(robot.joint_names):
                    models[name].set_value(math.degrees(goal[0, i].item()))
            finally:
                syncing = False

        def choose(name=None, path=None):
            nonlocal selected, goal, preset_shift, dirty, trajectory_name
            trajectory_name = None
            if path is not None:
                mapping = load_pose_json(path)  # Same schema validation as play.py.
                angles = [mapping[n] for n in robot.joint_names]
                preset_shift = [0., 0., 0.]
                selected = "json"
            else:
                preset = data["presets"][name]
                angles = [preset["angles_rad"][i] for i in order]
                preset_shift = preset.get("body_shift_m", [0., 0., 0.])
                selected = name
            validate_angles(angles)
            goal[:] = goal.new_tensor(angles)
            sync_models()
            dirty = True

        def reset():
            nonlocal command, dirty, elapsed, reference_gravity, root_writes
            command = goal.clone()
            root = robot.data.default_root_state.clone()
            root[:, :3] = root.new_tensor([0., 0., .21])
            root[:, 3:7] = root.new_tensor([1., 0., 0., 0.])
            root[:, 7:] = 0
            robot.write_root_state_to_sim(root)
            reset_selected_pose(facade, None, {n: goal[0, i].item() for i, n in enumerate(robot.joint_names)})
            robot.write_data_to_sim()
            robot.reset()
            contacts.reset()
            elapsed = 0.
            reference_gravity = None
            root_writes += 1
            dirty = True

        def static_apply():
            nonlocal dirty, root_writes
            root = robot.data.default_root_state.clone()
            root[:, :3] = root.new_tensor([preset_shift[0], preset_shift[1], .25])
            root[:, 7:] = 0
            for ground_align in (False, True):
                if ground_align:
                    feet = foot_world(robot)
                    root[:, 2] -= min(feet[p][2] for p in ("lf", "rb", "lb"))
                robot.write_root_state_to_sim(root)
                robot.write_joint_state_to_sim(goal, torch.zeros_like(goal))
                robot.set_joint_position_target(goal)
                robot.write_data_to_sim()
                sim.physics_sim_view.update_articulations_kinematic()
                root_writes += 1
            links = robot.root_physx_view.get_link_transforms()[0, render_order]
            static_links.set_world_poses(positions=links[:, :3], orientations=links[:, [6, 3, 4, 5]], usd=False)
            dirty = False

        choose(args.pose, args.pose_json)
        reset()
        trajectory_name = args.trajectory
        if args.mode == "static":
            sim.pause()
            static_apply()
        sim.set_camera_view(eye=(.42, -.52, .29), target=(.015, -.015, .11))

        async def arrange_and_verify():
            from omni.kit.viewport.utility import get_active_viewport_window, capture_viewport_to_file
            kit_app = omni.kit.app.get_app()
            for _ in range(12):
                await kit_app.next_update_async()
            viewport_window = get_active_viewport_window()
            for title in ("Stage", "Property", "Content", "Console", "Layer", "Render Settings", "Semantics Schema Editor"):
                existing = ui.Workspace.get_window(title)
                if existing:
                    existing.visible = False
            viewport = ui.Workspace.get_window(viewport_window.title)
            viewport.visible = True
            window.dock_in(viewport, ui.DockPosition.RIGHT, .44)
            print("POSE_CONTROL_READY mode="+args.mode, flush=True)
            if not args.verify_view:
                return
            from PIL import Image, ImageChops, ImageStat
            temporary = Path(tempfile.mkdtemp(prefix="pose_control_capture_"))
            captures = {}
            for name in ("standing", "tripod-high"):
                requests.append(("pose", name))
                for _ in range(25):
                    await kit_app.next_update_async()
                actual = robot.root_physx_view.get_dof_positions()[0]
                if torch.max(torch.abs(actual-goal[0])).item() > 1e-5:
                    raise AssertionError("Static pose not applied")
                feet = foot_world(robot)
                captures[name] = {"feet_world_m": feet, "root_z_m": robot.root_physx_view.get_root_transforms()[0, 2].item()}
                image_path = temporary/(name+".png")
                capture = capture_viewport_to_file(viewport_window.viewport_api, str(image_path))
                await capture.wait_for_result()
                for _ in range(180):
                    try:
                        with Image.open(image_path) as im:
                            im.load()
                        break
                    except OSError:
                        await kit_app.next_update_async()
                else:
                    raise AssertionError("PNG capture failed")
                shutil.copyfile(image_path, output/image_path.name)
            raised = captures["tripod-high"]
            ratio = raised["feet_world_m"]["rf"][2]/raised["root_z_m"]
            assert abs(ratio-.5) < .002, ratio
            with Image.open(temporary/"standing.png") as a, Image.open(temporary/"tripod-high.png") as b:
                delta = sum(ImageStat.Stat(ImageChops.difference(a.convert("RGB"), b.convert("RGB"))).mean)/3
            assert delta > .1, delta
            saved_goal = goal.clone()
            requests.append(("save", None))
            for _ in range(5):
                await kit_app.next_update_async()
            saved_path = path_model.as_string
            mapping = load_pose_json(saved_path)
            assert all(abs(mapping[n]-saved_goal[0, i].item()) < 1e-7 for i, n in enumerate(robot.joint_names))
            name = "rf_thigh"
            models[name].set_value(models[name].as_float + 1.)
            for _ in range(5):
                await kit_app.next_update_async()
            index = robot.joint_names.index(name)
            assert abs(goal[0, index].item()-saved_goal[0, index].item()-math.radians(1)) < 1e-5
            requests.append(("load", saved_path))
            for _ in range(5):
                await kit_app.next_update_async()
            assert torch.max(torch.abs(robot.root_physx_view.get_dof_positions()[0]-saved_goal[0])).item() < 1e-5
            report["static_verification"] = {"ratio": ratio, "pixel_difference": delta, "captures": captures,
                                             "live_slider_verified": True, "save_load_verified": saved_path}
            print("POSE_CONTROL_STATIC_TEST_PASS "+json.dumps(report["static_verification"]), flush=True)

        layout_task = asyncio.ensure_future(arrange_and_verify()) if not args.headless else None
        iteration = 0
        base_id = contacts.body_names.index("base_link")
        foot_ids = [contacts.body_names.index(p+"_shin") for p in ("rf", "lf", "rb", "lb")]
        try:
            import msvcrt
        except ImportError:
            msvcrt = None
        while app.is_running() and not quit_requested:
            tick_start = time.perf_counter()
            if msvcrt and not args.headless and msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                if key in ("x", "\x1b"):
                    requests.append(("quit", None))
                elif key == "r":
                    requests.append(("reset", None))
                elif key == " ":
                    requests.append(("pause", None))
            while requests:
                operation, value = requests.popleft()
                try:
                    if operation == "pose":
                        choose(value)
                    elif operation == "load":
                        choose(path=value)
                    elif operation == "joint":
                        name, degrees = value
                        if not math.isfinite(degrees) or abs(degrees) > 180:
                            raise ValueError("UI angle must be finite within +/-180 degrees")
                        goal[0, robot.joint_names.index(name)] = math.radians(degrees)
                        trajectory_name = None
                        selected = "manual"
                        dirty = True
                    elif operation == "reset":
                        trajectory_name = None
                        reset()
                    elif operation == "run":
                        if args.mode != "physics":
                            raise ValueError("Trajectory replay needs physics mode")
                        choose("standing")
                        reset()
                        trajectory_name = value
                        paused = False
                        sim.play()
                    elif operation == "pause":
                        paused = not paused
                        if args.mode == "physics":
                            sim.pause() if paused else sim.play()
                    elif operation == "quit":
                        quit_requested = True
                    elif operation == "save":
                        angles = goal[0].cpu().tolist()
                        filename = output/("pose_"+datetime.now().strftime("%H%M%S_%f")+".json")
                        filename.write_text(json.dumps({"schema_version": 1, "purpose": "simulation target, not hardware calibration",
                            "motor_frame": "sim", "joint_names": robot.joint_names, "angles_rad": angles,
                            "angles_deg": [math.degrees(x) for x in angles]}, indent=2), encoding="utf-8")
                        path_model.set_value(str(filename))
                        print("POSE_CONTROL_SAVED "+str(filename), flush=True)
                except (ValueError, OSError) as exc:
                    status.text = str(exc)
                    print("POSE_CONTROL_REQUEST_ERROR "+str(exc), flush=True)
            if args.mode == "static":
                if sim.is_playing():
                    sim.pause()
                if dirty:
                    static_apply()
                if args.headless:
                    app.update()
                else:
                    sim.render()
            elif not paused:
                if trajectory_name:
                    frame, phase, complete = select_frame(data["trajectories"][trajectory_name], elapsed, args.settle_seconds)
                    current_index = frame
                    q = data["trajectories"][trajectory_name]["frames"][frame]["angles_rad"]
                    goal[:] = goal.new_tensor([q[i] for i in order])
                    command = goal.clone()  # Exact CLI target, no extra interpolation.
                else:
                    complete = False
                    phase = "manual-hold"
                    command += torch.clamp(goal-command, -.02, .02)
                for substep in range(substeps):
                    robot.set_joint_position_target(command)
                    robot.write_data_to_sim()
                    sim.step(render=not args.headless and substep == substeps-1)
                    robot.update(dt)
                    contacts.update(dt)
                gravity = robot.data.projected_gravity_b[0]
                tilt = math.degrees(math.acos(max(-1., min(1., -gravity[2].item()))))
                if reference_gravity is None and elapsed >= args.settle_seconds:
                    reference_gravity = gravity.clone()
                relative = 0. if reference_gravity is None else math.degrees(math.acos(max(-1., min(1., torch.dot(reference_gravity, gravity).item()))))
                forces = torch.linalg.vector_norm(contacts.data.net_forces_w[0], dim=-1).cpu().tolist()
                position = robot.data.root_pos_w[0].cpu().tolist()
                feet = foot_world(robot)
                row = {"time_s": elapsed + .02, "target_time_s": elapsed, "frame": current_index, "phase": phase, "tilt_deg": tilt,
                       "relative_tilt_deg": relative, "root_pos_m": position, "feet_world_m": feet,
                       "base_contact_n": forces[base_id], "shin_contact_n": [forces[i] for i in foot_ids],
                       "max_joint_error_rad": float(torch.max(torch.abs(robot.data.joint_pos-command)).item()),
                       "joint_pos_rad": robot.data.joint_pos[0].cpu().tolist(), "target_pos_rad": command[0].cpu().tolist()}
                timeline.append(row)
                if not all(math.isfinite(x) for x in position) or max(abs(x) for x in position) > 5:
                    raise RuntimeError("Physics state escaped diagnostic bounds")
                elapsed += .02
                if complete and args.exit_on_complete:
                    break
            else:
                app.update()
            if iteration % 10 == 0:
                if args.mode == "static":
                    feet = foot_world(robot)
                    height = (feet["rf"][2]-sum(feet[p][2] for p in ("lf", "rb", "lb"))/3)*1000
                    status.text = f"STATIC | {selected}\nRF above support proxies: {height:.1f}mm\nBody fixed; not a balance test. R reset / X quit."
                elif timeline:
                    row = timeline[-1]
                    sync_models()
                    status.text = f"PHYSICS | {trajectory_name or selected} | {phase}\nt={elapsed:.2f}s tilt={row['tilt_deg']:.2f}deg z={row['root_pos_m'][2]:.3f}m\nBase contact={row['base_contact_n']:.2f}N\nJoint error max={math.degrees(row['max_joint_error_rad']):.2f}deg | R reset / Space pause / X exit"
            iteration += 1
            if layout_task and layout_task.done():
                layout_task.result()
                if args.verify_view:
                    break
            if args.frames and iteration >= args.frames:
                break
            if args.seconds and ((elapsed if args.mode == "physics" else iteration*.02) >= args.seconds):
                break
            if not args.headless:
                time.sleep(max(0, .02-(time.perf_counter()-tick_start)))
        if layout_task and not layout_task.done():
            layout_task.cancel()
        report["root_state_write_count"] = root_writes
        report["physics_samples"] = len(timeline)
        if timeline:
            report["max_tilt_deg"] = max(r["tilt_deg"] for r in timeline)
            report["max_relative_tilt_deg"] = max(r["relative_tilt_deg"] for r in timeline)
            report["max_base_contact_n"] = max(r["base_contact_n"] for r in timeline)
            report["max_joint_error_rad"] = max(r["max_joint_error_rad"] for r in timeline)
            report["first_fall"] = next((r for r in timeline if r["tilt_deg"] > 45 or r["root_pos_m"][2] < .08), None)
            report["first_cli_tilt_guard"] = next((r for r in timeline if r["relative_tilt_deg"] > 8), None)
            report["last_frame"] = timeline[-1]["frame"]
            report["status"] = "fall_observed" if report["first_fall"] else "no_fall_observed"
            (output/"physics_timeline.json").write_text(json.dumps(timeline, separators=(",", ":")), encoding="utf-8")
        else:
            report["status"] = "static_only_no_balance_claim"
        (output/"report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print("POSE_CONTROL_REPORT "+json.dumps(report), flush=True)
        print("POSE_CONTROL_OUTPUT "+str(output), flush=True)
    except Exception as exc:
        failure = {"status": "error", "error": type(exc).__name__ + ": " + str(exc),
                   "mode": args.mode, "hardware_connected": False}
        if "report" in locals():
            failure = {**report, **failure}
        (output/"report.json").write_text(json.dumps(failure, indent=2), encoding="utf-8")
        print("POSE_CONTROL_ERROR " + failure["error"], flush=True)
        traceback.print_exc()
        raise
    finally:
        if sim is not None:
            sim.clear_instance()
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
