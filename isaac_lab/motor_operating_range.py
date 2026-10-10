"""Hardware raw-coordinate range cost, without Isaac/Kit dependencies.

Calibration matches pc-connect-api/src/policy_raw.cpp (2026-10-10 native-front restore).
Never wrap or clamp requested raw positions to the encoder range.
"""

import csv
import math
from pathlib import Path

import torch


RANGE_CSV = Path(__file__).resolve().parent / "configs" / "contact_operating_ranges.csv"
MOTOR_JOINT_NAMES = tuple(f"{leg}_{kind}" for leg in ("rf", "lf", "rb", "lb")
                          for kind in ("shin", "thigh", "shoulder"))
MOTOR_SIGNS = (1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1)
REFERENCE_RAD = tuple(math.radians(d) for d in (-160, 75, 0) * 4)
EXPECTED_KEYS = {(0, 3), (0, 6), (0, 9), (0, 12), (1, 2), (4, 5), (7, 8), (10, 11)}


def load_operating_ranges(csv_path=RANGE_CSV, command_speed_steps_per_s=200):
    """Read the measured safe bounds; fail rather than silently lose a leg.

    Only the explicitly selected measured speed is used. There is no evidence
    for extrapolating this position envelope to unmeasured command speeds.
    """
    if command_speed_steps_per_s <= 0:
        raise ValueError("command_speed_steps_per_s must be positive")
    groups = {}
    required = {"anchor_id", "anchor_position_steps", "sweep_id",
                "command_speed_steps_per_s", "safe_min_steps", "safe_max_steps"}
    with Path(csv_path).open(encoding="utf-8-sig", newline="") as stream:
        rows = csv.DictReader(line for line in stream if line.strip() and not line.lstrip().startswith("#"))
        if not required.issubset(rows.fieldnames or ()):
            raise ValueError("Operating-range CSV is missing required columns")
        for row in rows:
            anchor, sweep, position, speed, low, high = (int(row[name]) for name in (
                "anchor_id", "sweep_id", "anchor_position_steps",
                "command_speed_steps_per_s", "safe_min_steps", "safe_max_steps"))
            key = (anchor, sweep)
            if key not in EXPECTED_KEYS or not 0 <= position <= 4095 or not 0 <= low < high <= 4095 or speed <= 0:
                raise ValueError(f"Invalid operating-range CSV row: {row}")
            if speed == command_speed_steps_per_s:
                groups.setdefault(key, []).append((position, low, high))
    if set(groups) != EXPECTED_KEYS:
        raise ValueError(f"Incomplete operating ranges at speed {command_speed_steps_per_s}: {set(groups)}")
    for (anchor, sweep), points in groups.items():
        points.sort()
        if len({p[0] for p in points}) != len(points):
            raise ValueError(f"Duplicate anchor positions for {(anchor, sweep)}")
        if (anchor == 0 and (len(points) != 1 or points[0][0] != 0)) or (anchor != 0 and len(points) < 2):
            raise ValueError(f"Invalid anchor sampling for {(anchor, sweep)}")
    return groups


class MotorRawTransform:
    """Vectorized absolute q [rad] -> continuous raw in motor ID order 1..12."""

    def __init__(self, joint_names, *, device="cpu", dtype=torch.float32):
        if len(joint_names) != 12 or set(joint_names) != set(MOTOR_JOINT_NAMES):
            raise ValueError("Expected exactly the 12 unique Yertle joint names")
        self.indices = [joint_names.index(name) for name in MOTOR_JOINT_NAMES]
        self.signs = torch.tensor(MOTOR_SIGNS, device=device, dtype=dtype)
        self.reference = torch.tensor(REFERENCE_RAD, device=device, dtype=dtype)
        # Shin servo drives linkage angle, not shin joint angle alone.

    def __call__(self, q):
        if q.ndim != 2 or q.shape[1] != 12:
            raise ValueError("Joint positions must have shape (num_envs, 12)")
        delta = q[:, self.indices] - self.reference
        # Direct addition also avoids TF32 matrix-multiply precision changes.
        delta[:, (0, 3, 6, 9)] += delta[:, (1, 4, 7, 10)]
        return 2048 + delta * self.signs * (4096 / (2 * math.pi))


class OperatingRangeCost:
    """Per-motor excess [raw] and strong dimensionless cost.

    Anchor positions outside measured coverage are penalized, NOT extrapolated
    or accepted. Endpoint bounds are used only to calculate an additional
    sweep violation while that anchor is already marked out of coverage.
    """

    def __init__(self, csv_path=RANGE_CSV, command_speed_steps_per_s=200,
                 excess_scale_raw=100.0, *, device="cpu", dtype=torch.float32):
        if not math.isfinite(excess_scale_raw) or excess_scale_raw <= 0:
            raise ValueError("excess_scale_raw must be finite and positive")
        self.scale = excess_scale_raw
        self.groups = {key: torch.tensor(points, device=device, dtype=dtype)
                       for key, points in load_operating_ranges(csv_path, command_speed_steps_per_s).items()}

    def excess(self, raw):
        if raw.ndim != 2 or raw.shape[1] != 12:
            raise ValueError("Raw positions must have shape (num_envs, 12)")
        # All axes retain their physical encoder bounds as well as measured bounds.
        excess = torch.maximum((-raw).clamp_min(0), (raw - 4095).clamp_min(0))
        for (anchor, sweep), points in self.groups.items():
            if anchor == 0:
                low, high = points[0, 1], points[0, 2]
            else:
                x = raw[:, anchor - 1]
                coverage = torch.maximum((points[0, 0] - x).clamp_min(0), (x - points[-1, 0]).clamp_min(0))
                excess[:, anchor - 1] = torch.maximum(excess[:, anchor - 1], coverage)
                x = x.clamp(points[0, 0], points[-1, 0]).contiguous()
                upper = torch.searchsorted(points[:, 0].contiguous(), x).clamp(1, len(points) - 1)
                a, b = points[upper - 1], points[upper]
                fraction = (x - a[:, 0]) / (b[:, 0] - a[:, 0])
                bounds = a[:, 1:] + fraction[:, None] * (b[:, 1:] - a[:, 1:])
                low, high = bounds.unbind(dim=1)
            value = raw[:, sweep - 1]
            violation = torch.maximum((low - value).clamp_min(0), (value - high).clamp_min(0))
            excess[:, sweep - 1] = torch.maximum(excess[:, sweep - 1], violation)
        # Float32 rad/raw roundtrips can overshoot an exact endpoint by <.001
        # raw. Do not turn that numerical noise into the unit violation cost.
        return torch.where(excess > 0.001, excess, torch.zeros_like(excess))

    def __call__(self, target_raw, measured_raw=None):
        excess = self.excess(target_raw)
        if measured_raw is not None:
            # Count each motor once, using its worse target/measured violation.
            excess = torch.maximum(excess, self.excess(measured_raw))
        return ((excess > 0).to(excess.dtype) + (excess / self.scale).square()).sum(dim=1)
