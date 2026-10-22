"""Simulation-only high RF pose. Never computes or emits motor raw targets.

FK uses the unchanged bundled URDF chain and the current API's provisional
shin-local distal-cap points. An actual Isaac/PhysX check remains necessary.
"""
import math
import numpy as np

PREFIXES = ("rf", "lf", "rb", "lb")


def foot_body(prefix, angles):
    """Return foot proxy in base_link coordinates; angles are shoulder/thigh/shin."""
    if prefix not in PREFIXES:
        raise ValueError("Unknown leg")
    q = np.asarray(angles, dtype=float)
    if q.shape != (3,) or not np.isfinite(q).all():
        raise ValueError("Expected three finite radian angles")
    shoulder, thigh, shin = q
    cs, ss = math.cos(shoulder), math.sin(shoulder)
    rx = np.array(((1, 0, 0), (0, cs, -ss), (0, ss, cs)))

    def ry(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))

    front, right = prefix in ("rf", "lf"), prefix in ("rf", "rb")
    origin = np.array((.090 if front else -.081, -.040 if right else (.037 if front else .038), 0))
    thigh_origin = np.array((.025 if front else -.043, -.027 if right else .027, 0))
    point = np.array((-.0008145190142713589,
                      -.0018949970081173842 + (-.007 if right else .008),
                      -.13366482367025265))
    return origin + rx @ (thigh_origin + ry(thigh) @ (np.array((0, 0, -.130)) + ry(shin) @ point))


def make_half_body_pose(source, ratio=.5):
    """Keep support angles and RF XY; solve RF Z to ratio * fixed body height."""
    if not math.isfinite(ratio) or not .05 <= ratio <= .75:
        raise ValueError("Display height ratio must be between 0.05 and 0.75")
    names = source["joint_names"]
    q = np.asarray(source["angles_rad"], dtype=float).copy()
    required = {p + "_" + k for p in PREFIXES for k in ("shoulder", "thigh", "shin")}
    if len(names) != 12 or len(set(names)) != 12 or set(names) != required or q.shape != (12,) or not np.isfinite(q).all():
        raise ValueError("Expected the 12 unique Yertle joints with finite angles")
    indices = {p: [names.index(p + "_" + k) for k in ("shoulder", "thigh", "shin")] for p in PREFIXES}
    feet = {p: foot_body(p, q[indices[p]]) for p in PREFIXES}
    # Viewer places the lowest of the three support proxy points at world Z=0.
    height = -min(feet[p][2] for p in ("lf", "rb", "lb"))
    if not .05 < height < .4:
        raise ValueError("Implausible body reference height")
    target = feet["rf"].copy()
    target[2] = -height + ratio * height
    angles = q[indices["rf"]].copy()
    for _ in range(150):
        position = foot_body("rf", angles)
        error = target - position
        if np.linalg.norm(error) < 1e-9:
            break
        jacobian = np.column_stack([(foot_body("rf", angles + np.eye(3)[j] * 1e-6) - position) / 1e-6
                                    for j in range(3)])
        delta = np.linalg.lstsq(jacobian, error, rcond=None)[0]
        delta *= min(1.0, .15 / max(np.max(np.abs(delta)), 1e-12))
        angles += delta
        if not np.isfinite(angles).all() or np.max(np.abs(angles)) > math.pi:
            raise ValueError("Visual IK exceeded the editor's +/-180 degree range")
    else:
        raise ValueError("Visual RF inverse kinematics did not converge")
    q[indices["rf"]] = angles
    # Deliberately no raw_positions_by_motor_id, hardware claims or stale reward metadata.
    return {
        "schema_version": 1,
        "purpose": "SIMULATION DISPLAY ONLY: RF at half-body height; NOT hardware-safe or balance-validated",
        "motor_frame": "sim",
        "joint_names": list(names),
        "angles_rad": q.tolist(),
        "angles_deg": [math.degrees(x) for x in q],
        "body_shift_m": list(source.get("body_shift_m", [0, 0, 0])),
        "feet_order": ["right_front", "left_front", "right_rear", "left_rear"],
        "expected_feet_body_m": [foot_body(p, q[indices[p]]).tolist() for p in PREFIXES],
        "display_height_ratio": ratio,
        "display_body_height_m": height,
        "display_rf_ground_clearance_m": ratio * height,
    }
