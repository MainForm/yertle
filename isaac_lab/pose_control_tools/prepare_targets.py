"""Prepare immutable pose/trajectory data using the exact deployed C++ API.

Offline only. No serial. Does not change the API calibration or CLI source.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_INVERSE = ROOT / "measurements/tripod_dynamic_20261020/build/Release/tripod_raw_inverse.exe"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inverse-exe", type=Path, default=DEFAULT_INVERSE)
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "data/pose_targets.json")
    args = parser.parse_args()
    sources = {"tripod-low": ROOT / "measurements/tripod_rf_pose_20261020/deployed-complete.jsonl",
               "tripod-high": ROOT / "measurements/tripod_high_cli_20261020/deployed-complete.jsonl"}
    result = {"schema_version": 1, "kind": "simulation_pose_targets_not_hardware_commands",
              "api_conversion": "deployed frozen C++ PolicyRawConverter; every raw roundtrip checked",
              "inverse_exe_sha256": hashlib.sha256(args.inverse_exe.read_bytes()).hexdigest(),
              "trajectories": {}, "presets": {}}
    for name, path in sources.items():
        records = [json.loads(line) for line in path.read_text(encoding="utf-8-sig").splitlines()]
        plans = [r for r in records if r.get("kind") == "tripod_lift_plan"]
        steps = [r for r in records if r.get("kind") == "tripod_lift_step"]
        expected = 951 if name == "tripod-high" else 651
        if len(plans) != 1 or len(steps) != expected:
            raise ValueError("Missing complete exact CLI trajectory: " + name)
        for i, step in enumerate(steps):
            if step["index"] != i or abs(step["nominal_time_s"] - i * .02) > 1e-6:
                raise ValueError("Missing/duplicate/global-time error in source trajectory")
        payload = "\n".join(" ".join(map(str, s["target_raw"])) for s in steps) + "\n"
        proc = subprocess.run([str(args.inverse_exe)], input=payload, capture_output=True, text=True,
                              check=True, timeout=20)
        converted = proc.stdout.splitlines()
        names = json.loads(converted[0])["joint_names"]
        if len(converted) != expected + 1:
            raise ValueError("Actual API did not convert every frame")
        if "joint_names" in result and names != result["joint_names"]:
            raise ValueError("API joint order changed between conversions")
        result["joint_names"] = names
        frames = [{"angles_rad": json.loads(q), "phase": s["phase"],
                   "body_shift_m": s["body_shift_m"], "expected_feet_body_m": s["expected_feet_body_m"]}
                  for s, q in zip(steps, converted[1:])]
        result["trajectories"][name] = {"period_s": .02, "frames": frames,
                                       "source": str(path), "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        apex = 350 if name == "tripod-high" else 200
        result["presets"][name] = frames[apex]
        if name == "tripod-high":
            result["presets"]["standing"] = frames[0]
            result["presets"]["tripod-ready"] = frames[100]
    import math
    result["presets"]["seated"] = {"angles_rad": [math.radians(0 if n.endswith("shoulder") else 75 if n.endswith("thigh") else -160)
                                                  for n in result["joint_names"]], "body_shift_m": [0, 0, 0]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    print("PREPARED_CLI_TARGETS " + str(args.output))
    print("low=651 high=951; actual API inverse/roundtrip; no hardware IO")


if __name__ == "__main__":
    main()
