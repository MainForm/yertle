# Hardware operating-range reward (2026-10-10)

`flat_env_cfg_reward.py` registers `motor_operating_range_penalty`, weight **-10.0**,
for the flat environment and its derived play/distillation configurations.
Other reward weights, reset poses, observations (55), actions (12), actuator
limits and command ranges are unchanged. An already-running process keeps its
loaded reward definitions; use a new process for this term to take effect.

## Measured source

`contact_operating_ranges.csv` here is an unchanged-data snapshot of:

`C:\Users\ik\Desktop\yertle-SCServo_Linux\window\SCServo_Linux\examples\SMS_STS\estimate_load\leg_range_scan\build\contact_operating_ranges.csv`

The original collision detector's `GroundRangeRegistry` references that file.
54 rows cover four standalone shoulders and four shin-anchor/thigh-sweep curves.
Use **safe_min_steps / safe_max_steps**, not contact endpoints: the table already
includes its 64-raw safety margin; do not apply it twice.
All measurements were made at **200 raw/s**. This implementation explicitly
selects that measured envelope; it does not claim validation at 1500 raw/s or
infer speed-dependent limits from a single measured speed. Missing, malformed,
duplicate or incomplete selected-speed data raise an error during environment
construction, rather than silently dropping the constraint.

## Coordinates and range checks

Absolute simulation joint targets are converted using the same seated reference,
motor ID mapping, signs and shin/thigh linkage as `pc-connect-api/policy_raw.cpp`:

- Motor IDs 1/2/3 = rf shin/thigh/shoulder; 4/5/6 = lf;
  7/8/9 = rb; 10/11/12 = lb (native front; 180-degree reverse-front restored).
- Raw reference 2048; joint reference shoulder 0°, thigh +75°, shin -160°.
- Direction signs by ID: +,+,+ / -,-,+ / +,+,- / -,-,-.
- Raw scale 4096/(2*pi); shin servo displacement uses **delta shin + delta thigh**.
- Continuous raw positions are NOT rounded, wrapped or clipped. This is slightly
  stricter than the integer-rounded transmission API at fractional boundaries.

All 12 motors are checked against encoder 0..4095. Shoulder IDs 3/6/9/12 use
their standalone safe intervals. Thigh IDs 2/5/8/11 use linear interpolation of
the safe interval at their corresponding shin raw position. Shin IDs 1/4/7/10
outside the measured anchor domain incur their own excess cost: unmeasured
coverage is not assumed safe. Endpoint interpolation for the thigh outside
coverage supplies an additional cost only, not permission for that anchor.
The shoulder table was measured with the other two motors at 2048; it is not a
complete 3-D self-collision envelope at all leg poses.

## Cost

Check both `joint_pos.processed_actions` (requested absolute targets) and the
measured `joint_pos`. Name-based transforms handle different joint orders.
For each motor use the larger target/measured range excess `e_i` [raw], without
double-counting the same motor. Cost is:

`sum_i(1[e_i > 0] + (e_i / 100)^2)`

Excess up to 0.001 raw is treated as zero to avoid a unit penalty caused by
float32 angle/raw rounding noise at an exact boundary.

The reward manager multiplies this by **-10.0** and the control timestep.
At 50 Hz: one violating motor costs at least -0.2 per step, 100 raw excess costs
-0.4, and 200 raw costs -1.0. XY tracking can contribute at most +0.04 per step
with its unchanged weight 2.0. Thus this is intentionally a strong starting
penalty; tune it from actual learning results, not an asserted safety guarantee.
It is a soft reward, not a hard target clamp or replacement for runtime safety
checks. Existing checkpoints are unchanged and do not acquire this constraint
without further training. Existing default/reset poses may already violate the
measured envelope; this change deliberately does not alter them.

## Reproducible verification

From the repository root, using a Python environment with PyTorch:

`python isaac_lab/tests/test_motor_operating_range.py`

Tests cover CSV completeness/invalid rows, all signs and linkage coupling,
permuted joint orders, every measured endpoint, interpolation, unmeasured anchor
coverage, encoder overflow, target-only and measured-only costs, and the prior
actor target that was motor 7 = 4403 under reverse-front (now motor 4 = -307).
No COM commands or robot movement occur.
