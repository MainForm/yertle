"""Simulator-independent pose data for manual calibration, not servo control."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path


MOTOR_JOINT_NAMES = (
    "rf_thigh", "rf_shin", "rf_shoulder",
    "lf_thigh", "lf_shin", "lf_shoulder",
    "rb_thigh", "rb_shin", "rb_shoulder",
    "lb_thigh", "lb_shin", "lb_shoulder",
)
LEG_LABELS = (
    ("rf", "Right front"), ("lf", "Left front"),
    ("rb", "Right rear"), ("lb", "Left rear"),
)
MOTOR_FRAMES = {
    "sim": ("rf", "lf", "rb", "lb"),
    # Physical forward=-sim X, physical right=+sim Y (a 180-degree yaw).
    "reverse-front": ("lb", "rb", "lf", "rf"),
    # Only use if the user's left/right labels intentionally remain unchanged.
    "swap-front-rear": ("rb", "lb", "rf", "lf"),
}
FRAME_DESCRIPTIONS = {
    "sim": "front=+sim X, right=-sim Y",
    "reverse-front": "front=-sim X, right=+sim Y (180-degree yaw)",
    "swap-front-rear": "front/rear labels swapped only; left/right labels unchanged",
}


class CalibrationPose:
    """Named joint angles and the loaded articulation's hard limits, in rad."""

    def __init__(self, joint_names, default_angles, limits, *, manual_limit_degrees=None, motor_frame="sim"):
        self.joint_names = list(joint_names)
        if len(self.joint_names) != 12 or set(self.joint_names) != set(MOTOR_JOINT_NAMES):
            raise ValueError("Expected exactly the 12 named Yertle joints.")
        if len(default_angles) != 12 or len(limits) != 12:
            raise ValueError("Expected one default angle and limit pair per joint.")
        self.indices = {name: index for index, name in enumerate(self.joint_names)}
        self.limits = [tuple(map(float, pair)) for pair in limits]
        self.default_angles = list(map(float, default_angles))
        for angle, (lower, upper) in zip(self.default_angles, self.limits):
            if not all(math.isfinite(v) for v in (angle, lower, upper)) or not lower <= angle <= upper:
                raise ValueError("Default angle or joint limits are invalid.")
        self.angles = self.default_angles.copy()
        self.source_limits = self.limits.copy()
        if manual_limit_degrees is not None:
            maximum = math.radians(float(manual_limit_degrees))
            if not math.isfinite(maximum) or maximum <= 0 or any(abs(q) > maximum for q in self.default_angles):
                raise ValueError("Invalid manual editor angle range.")
            self.limits = [(-maximum, maximum)] * 12
        if motor_frame not in MOTOR_FRAMES:
            raise ValueError("Unknown motor reference frame.")
        self.motor_frame = motor_frame
        self.leg_labels = tuple(zip(MOTOR_FRAMES[motor_frame],
                                    ("Robot right front", "Robot left front", "Robot right rear", "Robot left rear")))
        self.motor_joint_names = tuple(f"{leg}_{kind}" for leg in MOTOR_FRAMES[motor_frame]
                                       for kind in ("thigh", "shin", "shoulder"))
        self.loaded_motor_frame = None

    def set_degrees(self, name, value):
        """Clamp UI entry to the active editor range; reject NaN/infinity."""
        radians = math.radians(float(value))
        if not math.isfinite(radians):
            raise ValueError("Angle must be finite.")
        index = self.indices[name]
        lower, upper = self.limits[index]
        self.angles[index] = min(upper, max(lower, radians))
        return math.degrees(self.angles[index])

    def reset(self):
        self.angles = self.default_angles.copy()

    def load(self, path):
        """Load by joint name, atomically; invalid data never changes the pose.

        Rad values are authoritative. Optional redundant deg/named fields are
        checked for consistency. Hardware raw/sign metadata is not imported.
        """
        path = Path(path).expanduser().resolve()
        with path.open(encoding="utf-8-sig") as stream:
            data = json.load(stream)
        if not isinstance(data, dict) or type(data.get("schema_version")) is not int or data["schema_version"] != 1:
            raise ValueError("Expected a pose JSON with schema_version=1.")
        names, radians = data.get("joint_names"), data.get("angles_rad")
        if (not isinstance(names, list) or len(names) != 12
                or any(not isinstance(name, str) for name in names)
                or set(names) != set(self.joint_names)):
            raise ValueError("Pose must contain exactly the 12 unique Yertle joint names.")
        if not isinstance(radians, list) or len(radians) != 12:
            raise ValueError("Pose must contain 12 angles_rad values.")
        candidate = self.angles.copy()
        for name, value in zip(names, radians):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"Invalid rad angle for {name}.")
            index = self.indices[name]
            lower, upper = self.limits[index]
            if not lower - 1e-6 <= value <= upper + 1e-6:
                raise ValueError(f"{name} is outside the active pose joint limits.")
            candidate[index] = min(upper, max(lower, float(value)))
        if "angles_deg" in data:
            degrees = data["angles_deg"]
            if not isinstance(degrees, list) or len(degrees) != 12:
                raise ValueError("Invalid angles_deg array.")
            for q, degrees_value in zip(radians, degrees):
                if (type(degrees_value) not in (int, float) or not math.isfinite(degrees_value)
                        or abs(math.radians(degrees_value) - q) > 1e-6):
                    raise ValueError("angles_deg and angles_rad disagree.")
        if "joints" in data:
            joints = data["joints"]
            if not isinstance(joints, dict) or set(joints) != set(self.joint_names):
                raise ValueError("Named joint data does not match joint_names.")
            for name, q in zip(names, radians):
                record = joints[name]
                value = record.get("angle_rad") if isinstance(record, dict) else None
                if type(value) not in (int, float) or not math.isfinite(value) or abs(value - q) > 1e-6:
                    raise ValueError(f"Named rad angle disagrees for {name}.")
        self.angles = candidate
        # Legacy JSON has the original sim-front mapping. Joint-named angles
        # must not be permuted just because motor IDs now use another frame.
        self.loaded_motor_frame = data.get("motor_frame", "sim")
        return path

    def as_dict(self):
        return {
            "schema_version": 1,
            "saved_at_utc": datetime.now(timezone.utc).isoformat(),
            "purpose": "manual simulation pose reference; not a hardware calibration",
            "physics": "paused, body fixed, no policy or motor commands",
            "motor_mapping_version": 2,
            "motor_frame": self.motor_frame,
            "motor_frame_description": FRAME_DESCRIPTIONS[self.motor_frame],
            "angle_range_scope": "manual pose editor only; not hardware or training limits",
            "joint_names": self.joint_names.copy(),
            "angles_rad": self.angles.copy(),
            "angles_deg": [math.degrees(q) for q in self.angles],
            "joints": {name: {"angle_rad": self.angles[i],
                              "angle_deg": math.degrees(self.angles[i]),
                              "limits_rad": list(self.limits[i]),
                              "source_usd_limits_rad": list(self.source_limits[i])}
                       for i, name in enumerate(self.joint_names)},
            "motors": [{"motor_id": motor_id, "joint": name,
                        "simulation_joint_index": self.indices[name],
                        "angle_rad": self.angles[self.indices[name]],
                        "raw_reference": None, "raw_direction_sign": None}
                       for motor_id, name in enumerate(self.motor_joint_names, start=1)],
        }

    def save(self, output_dir):
        """Create a new timestamped JSON; never overwrite an existing pose."""
        output_dir = Path(output_dir).expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        name = "pose_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%fZ") + ".json"
        path = output_dir / name
        with path.open("x", encoding="utf-8") as stream:
            json.dump(self.as_dict(), stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        return path
