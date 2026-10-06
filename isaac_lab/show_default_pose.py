"""Adjust and save 12 Yertle joint angles without a policy or hardware commands.

The UI displays degrees; JSON records radians and motor-ID mapping. Physics
stays paused and the body is held at its initial transform. This is a static
geometry reference, not a test of balance, torque, contact, or collision.
"""

import argparse
import asyncio
from collections import deque
import faulthandler
import json
import math
from pathlib import Path
import sys
import time
import traceback

# Match the native player's DLL import ordering.
import h5py  # noqa: F401
import torch
from isaaclab.app import AppLauncher

# These helpers have no Isaac dependencies and can be unit-tested separately.
from pose_calibration import CalibrationPose, MOTOR_FRAMES, FRAME_DESCRIPTIONS


class PoseEditor:
    def __init__(self, sim, robot, ui, output_dir, initial_pose=None, motor_frame="reverse-front"):
        self.sim, self.robot, self.ui = sim, robot, ui
        self.output_dir = Path(output_dir)
        self.root = robot.data.default_root_state.clone()
        from isaacsim.core.prims import XFormPrim
        self.render_links = XFormPrim(
            prim_paths_expr=[f"/World/Yertle/{name}" for name in robot.body_names],
            reset_xform_properties=False,
        )
        self.render_indices = [robot.body_names.index(Path(path).name)
                               for path in self.render_links.prim_paths]
        self.pose = CalibrationPose(robot.joint_names,
                                    robot.data.default_joint_pos[0].cpu().tolist(),
                                    robot.data.joint_pos_limits[0].cpu().tolist(),
                                    manual_limit_degrees=180, motor_frame=motor_frame)
        # Change only this tool's live articulation; never the USD/URDF or
        # train/play configuration. Paused kinematics may explore impossible poses.
        robot.write_joint_position_limit_to_sim(torch.tensor(
            [self.pose.limits], dtype=robot.data.joint_pos_limits.dtype, device=robot.device))
        self.models, self.rad_labels = {}, {}
        self.dirty = True
        self._updating_models = False
        self.last_saved = None
        self.last_loaded = None
        self.last_error = None
        self.requests = deque()
        self.file_picker = None
        self.path_model = ui.SimpleStringModel(str(initial_pose) if initial_pose else "")
        self.window = ui.Window("Yertle pose calibration", width=640, height=780)
        with self.window.frame:
            with ui.VStack(spacing=6):
                ui.Label("Static pose only | Body fixed | NO policy / NO motor commands", height=24)
                ui.Label("Manual range: -180 to +180 deg. NOT hardware/training limits. JSON: rad.", height=24)
                ui.Label("Motor frame: " + FRAME_DESCRIPTIONS[self.pose.motor_frame], height=24)
                ui.Label("No balance/contact/collision check. Close this window to exit.", height=24)
                with ui.ScrollingFrame():
                    with ui.VStack(spacing=6):
                        for prefix, label in self.pose.leg_labels:
                            ui.Label(f"{label} | sim {prefix}", height=25)
                            for kind in ("shoulder", "thigh", "shin"):
                                self._joint_row(prefix + "_" + kind)
                            ui.Spacer(height=7)
                with ui.HStack(height=32, spacing=8):
                    self.reset_button = ui.Button("Restore default standing pose", clicked_fn=self.request_reset)
                    self.save_button = ui.Button("Save pose JSON", clicked_fn=self.request_save)
                ui.Label("Pose JSON path (or use Browse)", height=22)
                with ui.HStack(height=28, spacing=6):
                    ui.StringField(self.path_model)
                    self.browse_button = ui.Button("Browse...", width=85, clicked_fn=self.request_browse)
                    self.load_button = ui.Button("Load pose JSON", width=120, clicked_fn=self.request_load)
                self.status = ui.Label("Ready. Saved files are simulation references, not raw motor targets.",
                                       height=48, word_wrap=True)

    async def arrange_windows(self):
        """Keep the live viewport and controls visible in the same Kit window."""
        import omni.kit.app
        from omni.kit.viewport.utility import get_active_viewport_window

        # Kit's startup layout also docks windows asynchronously. Let it finish
        # first so it cannot subsequently cover or replace our viewport layout.
        for _ in range(12):
            await omni.kit.app.get_app().next_update_async()
        self.viewport_window = get_active_viewport_window()
        if self.viewport_window is None:
            raise RuntimeError("No viewport window is available in this experience")
        for title in ("Stage", "Property", "Content", "Console", "Layer", "Render Settings", "Semantics Schema Editor"):
            window = self.ui.Workspace.get_window(title)
            if window is not None:
                window.visible = False
        viewport = self.ui.Workspace.get_window(self.viewport_window.title)
        viewport.visible = True
        dockspace = self.ui.Workspace.get_window("DockSpace")
        if dockspace is not None:
            viewport.dock_in(dockspace, self.ui.DockPosition.SAME)
        self.window.dock_in(viewport, self.ui.DockPosition.RIGHT, 0.50)
        self.viewport_window.viewport_api.updates_enabled = True
        self.viewport_window.viewport_api.fill_frame = True
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        print("POSE_LAYOUT_READY: live viewport left, joint controls right", flush=True)

    def _joint_row(self, name):
        ui = self.ui
        index = self.pose.indices[name]
        lower, upper = (math.degrees(q) for q in self.pose.limits[index])
        model = ui.SimpleFloatModel(math.degrees(self.pose.angles[index]), min=lower, max=upper)
        self.models[name] = model
        motor_id = self.pose.motor_joint_names.index(name) + 1
        with ui.HStack(height=30, spacing=5):
            ui.Label(f"ID {motor_id:2d}  {name}", width=170)
            ui.FloatSlider(model, min=lower, max=upper, step=0.1, format="%.2f deg")
            ui.FloatField(model, width=80, precision=2)
            self.rad_labels[name] = ui.Label(f"{self.pose.angles[index]:+.4f} rad", width=94)
        model.add_value_changed_fn(lambda changed, name=name: self._changed(name, changed))

    def _changed(self, name, model):
        if self._updating_models:
            return
        try:
            clamped = self.pose.set_degrees(name, model.as_float)
        except ValueError as error:
            self._updating_models = True
            try:
                model.set_value(math.degrees(self.pose.angles[self.pose.indices[name]]))
            finally:
                self._updating_models = False
            self.status.text = str(error)
            return
        if abs(model.as_float - clamped) > 1e-5:
            self._updating_models = True
            try:
                model.set_value(clamped)
            finally:
                self._updating_models = False
        self.rad_labels[name].text = f"{self.pose.angles[self.pose.indices[name]]:+.4f} rad"
        self.dirty = True
        self.status.text = "Pose changed. Save JSON to keep this reference. No hardware commands sent."

    def apply(self):
        """Refresh kinematics and mirror link poses to the paused viewport."""
        # Only the outer loop calls this. A UI callback must never call pause()
        # or apply(): pause() itself invokes app.update() in Isaac Sim 5.1.
        angles = torch.tensor([self.pose.angles], dtype=self.robot.data.default_joint_pos.dtype,
                              device=self.robot.device)
        self.robot.write_root_pose_to_sim(self.root[:, :7])
        self.robot.write_root_velocity_to_sim(torch.zeros_like(self.root[:, 7:]))
        self.robot.write_joint_state_to_sim(angles, torch.zeros_like(angles))
        self.robot.set_joint_position_target(angles)
        self.robot.write_data_to_sim()
        # This computes correct tensor link poses while paused, but does not
        # flush them through the usual simulate/fetch render synchronization.
        self.sim.physics_sim_view.update_articulations_kinematic()
        links = self.robot.root_physx_view.get_link_transforms()[0, self.render_indices]
        # The public transform API updates Fabric's hierarchy (including visual
        # children). PhysX tensor quaternions are xyzw; this API expects wxyz.
        # Do not advance physics or modify the source USD asset to refresh a pose.
        self.render_links.set_world_poses(
            positions=links[:, :3], orientations=links[:, [6, 3, 4, 5]], usd=False)
        self.dirty = False

    def _sync_models(self):
        self._updating_models = True
        try:
            for name, model in self.models.items():
                value = self.pose.angles[self.pose.indices[name]]
                model.set_value(math.degrees(value))
                self.rad_labels[name].text = f"{value:+.4f} rad"
        finally:
            self._updating_models = False
        self.dirty = True

    # Button/file-picker callbacks only enqueue requests; no simulation or IO.
    def request_reset(self):
        self.requests.append(("reset", None))

    def request_save(self):
        self.requests.append(("save", None))

    def request_load(self):
        self.requests.append(("load", self.path_model.as_string.strip()))

    def request_browse(self):
        self.requests.append(("browse", None))

    def _file_picked(self, filename, dirname):
        self.path_model.set_value(str(Path(dirname) / filename))
        self.requests.append(("hide_picker", None))
        self.request_load()

    def _file_cancelled(self, *_):
        self.requests.append(("hide_picker", None))

    def process_requests(self):
        """Called between UI frames, never inside a rendering callback."""
        while self.requests:
            operation, argument = self.requests.popleft()
            try:
                if operation == "reset":
                    self.pose.reset()
                    self._sync_models()
                    self.status.text = "Default standing pose restored."
                elif operation == "save":
                    self.last_saved = self.pose.save(self.output_dir)
                    self.status.text = f"Saved: {self.last_saved.name}"
                    print(f"POSE_SAVED {self.last_saved}", flush=True)
                elif operation == "load":
                    if not argument:
                        raise ValueError("Enter a JSON path or use Browse first.")
                    self.last_loaded = self.pose.load(argument)
                    self._sync_models()
                    self.path_model.set_value(str(self.last_loaded))
                    self.status.text = f"Loaded: {self.last_loaded.name}"
                    if self.pose.loaded_motor_frame != self.pose.motor_frame:
                        self.status.text += " | File motor mapping differs; angles loaded by SIM joint name."
                    print(f"POSE_LOADED {self.last_loaded}", flush=True)
                elif operation == "browse":
                    if self.file_picker is None:
                        from omni.kit.window.filepicker import FilePickerDialog
                        self.file_picker = FilePickerDialog(
                            "Load Yertle pose JSON", width=800, height=500,
                            apply_button_label="Load",
                            click_apply_handler=self._file_picked,
                            click_cancel_handler=self._file_cancelled,
                            item_filter_options=["Pose JSON (*.json)"],
                            item_filter_fn=lambda item: not item or item.is_folder or item.path.lower().endswith(".json"),
                        )
                    start = str(self.output_dir.resolve()).replace("\\", "/")
                    self.file_picker.show(path=start)
                elif operation == "hide_picker" and self.file_picker is not None:
                    self.file_picker.hide()
                self.last_error = None
            except (OSError, ValueError, TypeError, ImportError) as error:
                self.last_error = str(error)
                self.status.text = f"{operation.title()} failed: {error}"
                print(f"POSE_{operation.upper()}_ERROR {error}", flush=True)

    def self_test(self):
        """Exercise actual UI callbacks, PhysX kinematics, reset, and JSON IO."""
        faulthandler.dump_traceback_later(30, repeat=True)
        time_before = self.sim.current_time
        defaults = self.pose.default_angles.copy()
        before = self.robot.root_physx_view.get_link_transforms().clone()
        for index, name in enumerate(self.pose.joint_names):
            self.models[name].set_value(math.degrees(defaults[index]) + 1.0)
        self.apply()
        actual = self.robot.root_physx_view.get_dof_positions()[0].cpu().tolist()
        assert all(abs(a - q) < 1e-5 for a, q in zip(actual, self.pose.angles))
        after = self.robot.root_physx_view.get_link_transforms().clone()
        assert not torch.allclose(before[:, 1:], after[:, 1:], atol=1e-6), "Computed link poses did not change"
        torch.testing.assert_close(before[:, 0], after[:, 0], atol=1e-6, rtol=0)
        self.request_save()
        assert self.last_saved is None, "Save ran inside the callback"
        self.process_requests()
        assert self.last_saved is not None, "JSON save failed"
        saved = json.loads(self.last_saved.read_text(encoding="utf-8"))
        assert saved["angles_rad"] == self.pose.angles
        assert [m["joint"] for m in saved["motors"]] == list(self.pose.motor_joint_names)
        print("POSE_TEST: save verified; restoring default", flush=True)
        self.request_reset()
        self.process_requests()
        print("POSE_TEST: default UI values restored; applying articulation", flush=True)
        self.apply()
        print("POSE_TEST: checking restored joint positions", flush=True)
        actual = self.robot.root_physx_view.get_dof_positions()[0].cpu().tolist()
        assert all(abs(a - q) < 1e-6 for a, q in zip(actual, defaults)), (actual, defaults)
        self.path_model.set_value(str(self.last_saved))
        self.request_load()
        self.process_requests()
        self.apply()
        actual = self.robot.root_physx_view.get_dof_positions()[0].cpu().tolist()
        assert all(abs(a - q) < 1e-6 for a, q in zip(actual, saved["angles_rad"]))
        self.request_reset()
        self.process_requests()
        self.apply()
        # Exercise both extended endpoints on every real joint/model/PhysX view.
        for degrees in (-180.0, 180.0):
            for name in self.pose.joint_names:
                self.models[name].set_value(degrees)
            self.apply()
            actual = self.robot.root_physx_view.get_dof_positions()[0].cpu().tolist()
            assert all(abs(q - math.radians(degrees)) < 1e-5 for q in actual), actual
        self.request_reset()
        self.process_requests()
        self.apply()
        assert self.sim.current_time == time_before, "Unexpected physics time advancement"
        faulthandler.cancel_dump_traceback_later()
        print("POSE_EDITOR_TEST_PASS: 12 joints at +/-180 deg, joint/link transforms, fixed body, save/load/reset", flush=True)

    async def ui_self_test(self):
        """Use Kit's mouse events to exercise buttons inside real UI frames."""
        import omni.kit.ui_test as ui_test
        from omni.kit.viewport.utility import capture_viewport_to_file
        from PIL import Image, ImageChops, ImageStat
        import omni.renderer_capture

        await ui_test.wait_n_updates(12)
        window = self.ui.Workspace.get_window("Yertle pose calibration")
        window.focus()
        await ui_test.wait_n_updates(5)
        async def click(widget):
            # Avoid WidgetRef.click() repeatedly undocking the window: that
            # can transiently hide it and end the editor's close-to-exit loop.
            point = ui_test.Vec2(widget.screen_position_x + widget.computed_content_width / 2,
                                widget.screen_position_y + widget.computed_content_height / 2)
            await ui_test.emulate_mouse_move_and_click(point, human_delay_speed=1)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        def verify_render_transforms():
            import omni.usd
            from usdrt import Usd, Rt
            stage = Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
            actual = self.robot.root_physx_view.get_link_transforms()[0].cpu().tolist()
            for name, transform in zip(self.robot.body_names, actual):
                prim = stage.GetPrimAtPath(f"/World/Yertle/{name}")
                if prim.HasAttribute("omni:fabric:worldMatrix"):
                    matrix = prim.GetAttribute("omni:fabric:worldMatrix").Get()
                    rendered = matrix.ExtractTranslation()
                    rotation = matrix.ExtractRotationQuat()
                else:
                    attr = Rt.Xformable(prim).GetWorldPositionAttr()
                    rendered = attr.Get() if attr.IsValid() else None
                    rotation = Rt.Xformable(prim).GetWorldOrientationAttr().Get()
                assert rendered is not None, f"Missing rendered transform for {name}"
                assert all(abs(a - b) < 1e-5 for a, b in zip(rendered, transform[:3])), (
                    f"Viewport/PhysX transform mismatch for {name}", rendered, transform[:3])
                rendered_q = [rotation.GetReal(), *rotation.GetImaginary()]
                expected_q = [transform[6], *transform[3:6]]
                error = min(math.dist(rendered_q, expected_q),
                            math.dist(rendered_q, [-q for q in expected_q]))
                assert error < 1e-5, f"Viewport/PhysX rotation mismatch for {name}: {error}"
        async def capture_pose(label):
            await ui_test.wait_n_updates(20)
            assert self.window.visible and self.viewport_window.visible, "Viewport/editor hidden"
            verify_render_transforms()
            path = self.output_dir / f"viewport_{label}_{time.time_ns()}.png"
            capture = capture_viewport_to_file(self.viewport_window.viewport_api, str(path))
            await capture.wait_for_result()
            # GPU capture completion precedes the background PNG disk write.
            for _ in range(120):
                if path.is_file():
                    try:
                        with Image.open(path) as captured:
                            captured.load()
                        break
                    except OSError:
                        pass
                await ui_test.wait_n_updates(1)
            else:
                raise AssertionError("Viewport PNG write did not finish")
            assert path.is_file(), "Viewport capture failed"
            return path
        self.request_reset()
        await ui_test.wait_n_updates(5)
        standing = await capture_pose("standing")
        selected_joint = self.pose.motor_joint_names[1]  # Physical right-front knee, ID 2.
        self.models[selected_joint].set_value(180.0)
        await ui_test.wait_n_updates(5)
        assert abs(self.models[selected_joint].as_float - 180.0) < 1e-4
        assert abs(self.robot.root_physx_view.get_dof_positions()[0, self.pose.indices[selected_joint]].item()
                   - math.pi) < 1e-5
        await capture_pose("motor2_180deg")
        print(f"POSE_RANGE_MAPPING_TEST_PASS: ID 2 -> {selected_joint}, UI/PhysX/viewport at 180 deg", flush=True)
        self.request_reset()
        await ui_test.wait_n_updates(5)
        for cycle in range(3):
            for name in self.pose.joint_names:
                kind = name.rsplit("_", 1)[-1]
                self.models[name].set_value({"shoulder": 0.0, "thigh": -57.3, "shin": 80.2}[kind])
            await ui_test.wait_n_updates(3)
            expected = self.pose.angles.copy()
            if cycle == 0:
                folded = await capture_pose("folded")
                with Image.open(standing) as a, Image.open(folded) as b:
                    # A viewport-only image excludes the changing UI labels.
                    delta = ImageChops.difference(a.convert("RGB"), b.convert("RGB"))
                    mean_delta = sum(ImageStat.Stat(delta).mean) / 3
                    assert mean_delta > 0.5, f"Rendered pose did not change: {mean_delta}"
                capture_api = omni.renderer_capture.acquire_renderer_capture_interface()
                capture_api.capture_next_frame_swapchain(str(self.output_dir / "editor_live.png"))
                await ui_test.wait_n_updates(2)
                capture_api.wait_async_capture()
                print(f"POSE_VIEWPORT_TEST_PASS: controls open, rendered pose changed; mean pixel delta {mean_delta:.3f}", flush=True)
            previous = self.last_saved
            await click(self.save_button)
            await ui_test.wait_n_updates(5)
            assert self.last_saved is not None and self.last_saved != previous, "Save button did not save"
            assert json.loads(self.last_saved.read_text(encoding="utf-8"))["angles_rad"] == expected
            await click(self.reset_button)
            await ui_test.wait_n_updates(5)
            assert self.pose.angles == self.pose.default_angles, "Reset button did not reset"
            self.path_model.set_value(str(self.last_saved))
            await click(self.load_button)
            await ui_test.wait_n_updates(5)
            assert self.pose.angles == expected, "Load button did not restore the saved pose"
            actual = self.robot.root_physx_view.get_dof_positions()[0].cpu().tolist()
            assert all(abs(a - q) < 1e-6 for a, q in zip(actual, expected))
            print(f"POSE_UI_TEST_CYCLE_PASS {cycle + 1}: real save/reset/load clicks", flush=True)
        await click(self.browse_button)
        await ui_test.wait_n_updates(5)
        assert self.file_picker is not None, "Browse button did not open the file picker"
        self._file_cancelled()
        await ui_test.wait_n_updates(5)
        print("POSE_UI_TEST_PASS: 3 real save/reset/load click cycles, file-picker open/cancel", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=0, help="0: run until the editor/GUI closes.")
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parent / "runs" / "pose_calibration",
                        help="Directory for new timestamped pose JSON files.")
    parser.add_argument("--self-test", action="store_true", help="Bounded UI/PhysX/JSON verification.")
    parser.add_argument("--ui-self-test", action="store_true", help="Bounded native GUI mouse-click regression test.")
    parser.add_argument("--pose-json", type=Path, help="Start with a previously saved pose JSON.")
    parser.add_argument("--motor-frame", choices=tuple(MOTOR_FRAMES), default="reverse-front",
                        help="Physical motor reference frame; default front=-X, right=+Y (180-degree yaw).")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.frames < 0:
        parser.error("--frames must be non-negative")
    if args.pose_json is not None and not args.pose_json.is_file():
        parser.error("--pose-json must point to an existing JSON file")
    if args.ui_self_test and args.headless:
        parser.error("--ui-self-test requires a GUI (omit --headless)")
    if args.self_test and not args.ui_self_test and args.frames == 0:
        args.frames = 3
    launcher = AppLauncher(args)
    simulation_app = launcher.app
    editor = None
    sim = None
    try:
        import omni.ui as ui
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation

        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from isaac_lab.yertle_cfg import YERTLE_CFG

        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
            dt=0.005, device=args.device, use_fabric=False))
        # Keep the physics-to-Fabric writer disabled: the paused pose editor
        # explicitly mirrors computed link transforms, without a physics step.
        sim.set_camera_view(eye=(0.7, 0.7, 0.45), target=(0.0, 0.0, 0.15))
        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/Ground", ground)
        light = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.85, 0.85, 0.85))
        light.func("/World/Light", light)
        robot = Articulation(YERTLE_CFG.replace(prim_path="/World/Yertle"))
        sim.reset()
        sim.pause()
        editor = PoseEditor(sim, robot, ui, args.output_dir, args.pose_json, args.motor_frame)
        if args.pose_json is not None:
            editor.request_load()
            editor.process_requests()
            if editor.last_error is not None:
                raise ValueError(editor.last_error)
        editor.apply()
        print("INITIAL_POSE " + json.dumps({"joint_names": robot.joint_names,
                                             "angles_rad": editor.pose.angles,
                                             "physics": "paused; manual static reference"}), flush=True)
        print(f"POSE_OUTPUT_DIR {args.output_dir.resolve()}", flush=True)
        print("MOTOR_MAPPING " + json.dumps(editor.pose.as_dict()["motors"]), flush=True)
        if args.self_test:
            editor.self_test()
        layout_task = asyncio.ensure_future(editor.arrange_windows()) if not args.headless else None
        async def run_ui_test():
            await layout_task
            await editor.ui_self_test()
        ui_task = asyncio.ensure_future(run_ui_test()) if args.ui_self_test else None
        ui_deadline = time.monotonic() + 120.0
        frame = 0
        while simulation_app.is_running() and (args.frames == 0 or frame < args.frames):
            # Prevent the timeline Play button from turning this into a rollout.
            # pause() also pumps a UI frame, so do not call it redundantly.
            if sim.is_playing():
                sim.pause()
            editor.process_requests()
            if editor.dirty:
                editor.apply()
            if args.headless:
                simulation_app.update()
            else:
                sim.render()
            time.sleep(1.0 / 60.0)
            frame += 1
            if layout_task is not None and layout_task.done():
                layout_task.result()
            if not editor.window.visible and (layout_task is None or layout_task.done()):
                break
            if ui_task is not None:
                if ui_task.done():
                    ui_task.result()
                    break
                if time.monotonic() > ui_deadline:
                    ui_task.cancel()
                    raise TimeoutError("Native UI click verification exceeded 120 seconds")
        if ui_task is not None and not ui_task.done():
            ui_task.cancel()
            raise RuntimeError("GUI closed or frame limit reached before UI verification finished")
        if layout_task is not None and not layout_task.done():
            layout_task.cancel()
    except BaseException:
        # Print diagnostics before Kit teardown, which can itself be slow.
        traceback.print_exc()
        raise
    finally:
        faulthandler.cancel_dump_traceback_later()
        if args.self_test or args.ui_self_test:
            print("POSE_TEST: closing simulation app", flush=True)
            faulthandler.dump_traceback_later(30, repeat=True)
        if sim is not None:
            # Unsubscribe Isaac Lab's STOP callback before close_stage(). In
            # Lab 2.3.2 it otherwise renders forever on our paused timeline.
            sim.clear_instance()
        # This tool has no Replicator workflow to drain during shutdown.
        simulation_app.close(wait_for_replicator=False)
        faulthandler.cancel_dump_traceback_later()


if __name__ == "__main__":
    main()
