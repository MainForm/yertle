"""Pure tensor regression check, without launching Isaac Sim."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import torch

spec = importlib.util.spec_from_file_location("train_terminations", Path(__file__).parents[1]/"train_terminations.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
poses = [(0,0), (44.9,0), (45,0), (45.1,0), (-45,0), (0,44.9), (0,45), (0,-45), (0,45.1), (180,0), (30,30)]
gravity = []
for roll,pitch in poses:
    r,p = math.radians(roll), math.radians(pitch)
    gravity.append([math.sin(p), -math.cos(p)*math.sin(r), -math.cos(p)*math.cos(r)])
env = SimpleNamespace(scene={"robot":SimpleNamespace(data=SimpleNamespace(projected_gravity_b=torch.tensor(gravity,dtype=torch.float64)))})
result = module.body_roll_pitch_limit(env, math.pi/4, SimpleNamespace(name="robot")).tolist()
assert result == [False,False,True,True,True,False,True,True,True,True,False], result
print("BODY_TILT_PASS roll/pitch +/-45 boundaries, upright, inverted, combined sub-threshold tilt")
