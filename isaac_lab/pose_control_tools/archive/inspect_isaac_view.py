"""Capture and compare the existing pose viewer; no hardware or mapping changes."""
import asyncio
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "yertle-isaaclab-host/source/isaac_lab"))
import show_default_pose as viewer

OUT = Path(__file__).resolve().parent / "isaac_view_diagnostic"
OUT.mkdir(exist_ok=True)
original_arrange = viewer.PoseEditor.arrange_windows


async def inspect(self):
    await original_arrange(self)
    import omni.kit.app
    from omni.kit.viewport.utility import capture_viewport_to_file
    from pxr import Gf

    app = omni.kit.app.get_app()
    selected = self.pose.angles.copy()
    self.sim.set_camera_view(eye=(0.55, -0.65, 0.35), target=(0.01, 0, 0.17))
    reports = {}
    for label, angles in (("standing", [0.0 if n.endswith("shoulder") else
                                         math.radians(34 if n.endswith("thigh") else -85)
                                         for n in self.pose.joint_names]),
                          ("rf_up25", selected)):
        self.pose.angles[:] = angles
        self._sync_models()
        for _ in range(35):
            await app.next_update_async()
        dof = self.robot.root_physx_view.get_dof_positions()[0].cpu().tolist()
        links = self.robot.root_physx_view.get_link_transforms()[0].cpu().tolist()
        root = self.robot.root_physx_view.get_root_transforms()[0].cpu().tolist()
        feet = {}
        for prefix in ("rf", "lf", "rb", "lb"):
            link = links[self.robot.body_names.index(prefix + "_shin")]
            point = Gf.Vec3d(-0.0008145190142713589,
                            -0.0018949970081173842 + (-.007 if prefix in ("rf", "rb") else .008),
                            -0.13366482367025265)
            rotation = Gf.Rotation(Gf.Quatd(link[6], Gf.Vec3d(*link[3:6])))
            world = Gf.Vec3d(*link[:3]) + rotation.TransformDir(point)
            feet[prefix] = list(world)
        reports[label] = {"joint_names": self.pose.joint_names, "requested": angles,
                          "actual_dof": dof, "root": root, "feet_world_m": feet,
                          "links": dict(zip(self.robot.body_names, links))}
        capture = capture_viewport_to_file(self.viewport_window.viewport_api,
                                          str(OUT / (label + ".png")))
        await capture.wait_for_result()
        for _ in range(20):
            await app.next_update_async()
        print("POSE_DIAGNOSTIC " + json.dumps({label: reports[label]}), flush=True)
    (OUT / "report.json").write_text(json.dumps(reports, indent=2), encoding="utf-8")
    print("POSE_DIAGNOSTIC_COMPLETE " + str(OUT), flush=True)


viewer.PoseEditor.arrange_windows = inspect
viewer.main()
