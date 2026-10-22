"""Static tripod comparison viewer. No policy, serial connection or physics steps.

Uses the existing joint-name pose loader unchanged. Only this viewer's root
placement, camera and visual markers differ: support-foot proxy points are put
at ground height, and the planned body translation is displayed. This is NOT
a validation of real contact, center of mass, torque or dynamic balance.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile

import show_default_pose as base
from tripod_visual_pose import make_half_body_pose

PREFIXES = ("rf", "lf", "rb", "lb")
DEFAULT_POSES = (Path.home() / "Desktop/yertle-RTOS/policy_onnx/dist/"
                 "v6.20.3_model_2200_cpu/poses/tripod_rf_20261020")


def foot_points(robot):
    """The same provisional shin-local distal-cap points as the current API."""
    from pxr import Gf
    links = robot.root_physx_view.get_link_transforms()[0].cpu().tolist()
    result = {}
    for prefix in PREFIXES:
        pose = links[robot.body_names.index(prefix + "_shin")]
        point = Gf.Vec3d(-0.0008145190142713589,
                        -0.0018949970081173842 + (-.007 if prefix in ("rf", "rb") else .008),
                        -0.13366482367025265)
        rotation = Gf.Rotation(Gf.Quatd(pose[6], Gf.Vec3d(*pose[3:6])))
        result[prefix] = list(Gf.Vec3d(*pose[:3]) + rotation.TransformDir(point))
    return result


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--pose-dir", type=Path, default=DEFAULT_POSES)
    parser.add_argument("--verify-tripod", action="store_true")
    parser.add_argument("--capture-dir", type=Path)
    parser.add_argument("--rf-height-ratio", type=float, default=.5,
                        help="SIMULATION ONLY: foot ground height / body-reference height (default 0.5).")
    options, remaining = parser.parse_known_args()
    for name in ("standing", "tripod_ready", "tripod_rf_up25"):
        if not (options.pose_dir / (name + ".json")).is_file():
            parser.error("Missing tripod pose: " + name)
    if options.verify_tripod and options.capture_dir is None:
        parser.error("--verify-tripod requires --capture-dir")
    ready = json.loads((options.pose_dir / "tripod_ready.json").read_text(encoding="utf-8"))
    visual_pose = make_half_body_pose(ready, options.rf_height_ratio)
    visual_dir = Path(__file__).resolve().parent / "runs/pose_calibration/tripod_visual"
    visual_dir.mkdir(parents=True, exist_ok=True)
    generated = Path(tempfile.mkdtemp(prefix="half_body_", dir=str(visual_dir))) / "tripod_rf_half_body.json"
    generated.write_text(json.dumps(visual_pose, indent=2), encoding="utf-8")

    def pose_path(name):
        return generated if name == "tripod_rf_half_body" else options.pose_dir / (name + ".json")

    if "--pose-json" not in remaining:
        remaining += ["--pose-json", str(generated)]
    if "--motor-frame" not in remaining:
        remaining += ["--motor-frame", "sim"]
    sys.argv = [sys.argv[0], *remaining]

    class TripodViewer(base.PoseEditor):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            import isaaclab.sim as sim_utils
            self.base_root = self.root.clone()
            self.marker_paths = {}
            for prefix in PREFIXES:
                path = "/World/TripodFoot_" + prefix
                sphere = sim_utils.SphereCfg(
                    radius=.003,
                    visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(1.0, .08, .05) if prefix == "rf" else (.02, .65, .9)))
                sphere.func(path, sphere)
                self.marker_paths[prefix] = path
            self.comparison = self.ui.Window("Tripod pose comparison", width=620, height=220)
            with self.comparison.frame:
                with self.ui.VStack(spacing=7):
                    self.ui.Label("STATIC SIM ONLY | Red = RF | Blue = support-foot proxy", height=23)
                    with self.ui.HStack(height=35, spacing=5):
                        self.ui.Button("1: Standing", clicked_fn=lambda: self.select("standing"))
                        self.ui.Button("2: Body shift", clicked_fn=lambda: self.select("tripod_ready"))
                        self.ui.Button("3: RF up 25 mm", clicked_fn=lambda: self.select("tripod_rf_up25"))
                        self.ui.Button("4: RF half body", clicked_fn=lambda: self.select("tripod_rf_half_body"))
                    self.measurement = self.ui.Label("Loading...", height=70, word_wrap=True)
                    self.ui.Label("Half-body pose is SIM ONLY; physical CLI remains capped at 25 mm.", height=24)
                    self.ui.Label("Legacy ID labels above are NOT the calibrated hardware mapping.", height=24)

        def select(self, name):
            self.path_model.set_value(str(pose_path(name)))
            self.request_load()

        def apply(self):
            from pxr import Gf, UsdGeom
            import omni.usd
            # This changes only this tool's static root pose, not the asset or API.
            shift = [0.0, 0.0, 0.0]
            if self.last_loaded is not None:
                data = json.loads(Path(self.last_loaded).read_text(encoding="utf-8"))
                shift = data.get("body_shift_m", shift)
            if len(shift) != 3 or not all(math.isfinite(x) for x in shift):
                raise ValueError("Invalid viewer body_shift_m")
            self.root[:, :3] = self.base_root[:, :3]
            self.root[:, 0] += shift[0]
            self.root[:, 1] += shift[1]
            super().apply()
            feet = foot_points(self.robot)
            self.root[:, 2] -= min(feet[prefix][2] for prefix in ("lf", "rb", "lb"))
            super().apply()
            feet = foot_points(self.robot)
            actual = self.robot.root_physx_view.get_dof_positions()[0].cpu().tolist()
            error = max(abs(a - b) for a, b in zip(actual, self.pose.angles))
            if error > 1e-5:
                raise RuntimeError("Requested/PhysX joint angles differ: " + str(error))
            stage = omni.usd.get_context().get_stage()
            for prefix, position in feet.items():
                # SphereCfg uses a non-common op stack. Replace only the visual
                # marker's transform stack instead of silently failing SetTranslate.
                UsdGeom.Xformable(stage.GetPrimAtPath(self.marker_paths[prefix])).MakeMatrixXform().Set(
                    Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(*position)))
                marker = UsdGeom.Xformable(stage.GetPrimAtPath(self.marker_paths[prefix]))
                rendered_point = marker.ComputeLocalToWorldTransform(0).ExtractTranslation()
                if math.dist(rendered_point, position) > 1e-6:
                    raise RuntimeError("Visual foot marker position mismatch: " + prefix)
            support_z = sum(feet[p][2] for p in ("lf", "rb", "lb")) / 3
            clearance = (feet["rf"][2] - support_z) * 1000
            self.measurement.text = (
                f"{Path(self.last_loaded).name if self.last_loaded else 'Manual pose'}\n"
                f"RF height above 3 support-foot proxies: {clearance:.2f} mm\n"
                f"Body rear/left shift: {-shift[0]*1000:.1f} / {shift[1]*1000:.1f} mm")
            self.diagnostic = {"pose": str(self.last_loaded), "feet_world_m": feet,
                               "rf_clearance_mm": clearance,
                               "rf_ground_height_mm": feet["rf"][2] * 1000,
                               "rf_body_height_ratio": feet["rf"][2] / self.root[0, 2].item(),
                               "body_position_m": self.root[0, :3].cpu().tolist(),
                               "joint_angle_error_max_rad": error}
            print("TRIPOD_VIEW " + json.dumps(self.diagnostic), flush=True)

        async def arrange_windows(self):
            await super().arrange_windows()
            import omni.kit.app
            self.comparison.dock_in(self.window, self.ui.DockPosition.BOTTOM, .30)
            self.sim.set_camera_view(eye=(.42, -.52, .29), target=(.015, -.015, .11))
            if options.verify_tripod:
                from omni.kit.viewport.utility import capture_viewport_to_file
                from PIL import Image, ImageChops, ImageStat
                app = omni.kit.app.get_app()
                options.capture_dir.mkdir(parents=True, exist_ok=True)
                # Kit image writer cannot save to a non-ASCII Windows path.
                temporary = Path(tempfile.mkdtemp(prefix="yertle_tripod_capture_"))
                reports = {}
                for name in ("standing", "tripod_ready", "tripod_rf_up25", "tripod_rf_half_body"):
                    self.select(name)
                    for _ in range(25):
                        await app.next_update_async()
                    if self.last_error:
                        raise RuntimeError(self.last_error)
                    reports[name] = self.diagnostic.copy()
                    source = json.loads(pose_path(name).read_text(encoding="utf-8"))
                    for prefix, expected in zip(PREFIXES, source["expected_feet_body_m"]):
                        actual = [x - r for x, r in zip(self.diagnostic["feet_world_m"][prefix],
                                                       self.diagnostic["body_position_m"])]
                        assert math.dist(actual, expected) < 1e-5, (prefix, actual, expected)
                    image_path = temporary / (name + ".png")
                    capture = capture_viewport_to_file(self.viewport_window.viewport_api, str(image_path))
                    await capture.wait_for_result()
                    for _ in range(180):
                        try:
                            with Image.open(image_path) as image:
                                image.load()
                            break
                        except OSError:
                            await app.next_update_async()
                    else:
                        raise RuntimeError("Viewport capture did not finish: " + str(image_path))
                    shutil.copyfile(image_path, options.capture_dir / image_path.name)
                assert 24 < reports["tripod_rf_up25"]["rf_clearance_mm"] < 26
                raised = reports["tripod_rf_half_body"]
                assert abs(raised["rf_body_height_ratio"] - options.rf_height_ratio) < .001, raised
                assert max(abs(raised["feet_world_m"][p][2]) for p in ("lf", "rb", "lb")) < .0001
                assert abs(reports["standing"]["rf_clearance_mm"]) < .2
                for prefix in ("lf", "rb", "lb"):
                    assert math.dist(reports["standing"]["feet_world_m"][prefix],
                                     raised["feet_world_m"][prefix]) < .0002
                with Image.open(temporary / "standing.png") as a, Image.open(temporary / "tripod_rf_half_body.png") as b:
                    difference = ImageChops.difference(a.convert("RGB"), b.convert("RGB"))
                    delta = sum(ImageStat.Stat(difference).mean) / 3
                assert delta > .1, "Standing and raised viewport captures are identical"
                reports["viewport_mean_pixel_difference"] = delta
                (options.capture_dir / "verification.json").write_text(
                    json.dumps(reports, indent=2), encoding="utf-8")
                print("TRIPOD_VIEW_TEST_PASS " + json.dumps({"rf_clearance_mm": raised["rf_clearance_mm"],
                      "height_ratio": raised["rf_body_height_ratio"],
                      "pixel_delta": delta, "captures": str(options.capture_dir)}), flush=True)

    base.PoseEditor = TripodViewer
    base.main()


if __name__ == "__main__":
    main()
