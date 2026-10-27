# Production PID / real-error-level contact experiment

`diagnose_error_contact.py` is an isolated simulation experiment. Its default
`full-pid` backend runs the production `position_pid.c` through the prepared C
bridge, including the real P5/I3/D0.10, I-output limit100, PWM limit800,
20ms controller update, Speed200..3000 and ACC100. There is no D filter or
PWM slew limiter. The twelve physical PWM polarities are read from that C
controller; angle/raw calibration and motor IDs are preserved.

## Reproduce

The prepared data/model/API/C bridge live in
`/workspace/yertle-artifacts/real_sim_tracking_20261027`.
Results for this experiment are under
`/workspace/yertle-artifacts/error_contact_sweep_20261027`.

```powershell
docker exec yertle-isaaclab-container-1 /isaac-sim/python.sh -u /workspace/yertle/isaac_lab/diagnose_error_contact.py --data-dir /workspace/yertle-artifacts/real_sim_tracking_20261027 --out-dir /workspace/yertle-artifacts/error_contact_sweep_20261027 --backend full-pid --kp 5 --ki 3 --kd .1 --plant dc --motor-gain .32 --physics-dt .0005 --seconds 3 --initial-integral log --sync-start --repeats 3 --label repeat_match --headless --device cuda:0
```

Use a new label to preserve the original traces. New runs write `_SUCCESS`
only after the summary and traces have been saved. Isaac's app shutdown can
mask an exception in the process exit status; check the marker and summary,
not only exit0. Exceptions are also printed before app shutdown.

Outputs per label:

- `_config.json`, `_script.py`: exact arguments/cases and source snapshot.
- `_trace.npz`: calibrated raw goal/reference/measured positions, original
  actor actions, P/I/D terms, sent PWM, base forces/history and root state.
- `_summary.json`: first-contact/first-PID-guard times and censored errors,
  gain/limit metadata, code/model/bridge hashes and source checks.
- `_SUCCESS`: all output and runtime checks completed.

For numerical sensitivity use `.001`, `.0005` and `.00025` physics intervals.
The policy/PID interval remains20ms; contact sensor interval remains5ms with
history3, matching the checkpoint configuration.

## Experimental definition

Model: v6.22.2/model_6999.pt,55D input/12D output, fixed trot1.7Hz/duty0.6,
reference0/+34/-85deg, scale0.25, command vx=vy=yaw=0. Observed linear XYZ
velocity is forced to zero, as in the real CLI test; body angular velocity,
gravity, joints and original last actor action remain available. There are
no added pushes/reset noise. Saved checkpoint physics limits are1.024Nm and
2.3rad/s; these are checkpoint settings, not newly measured physical specs.

Before the full-PID test, a1s physics-only native standing warmup establishes
ground support without advancing the policy phase/action history. `--sync-start`
copies env0's root/joint state to the clones, adjusted for environment origins.
`--initial-integral log` primes integrals from the last logged pre-policy
target sequence. It does not copy the real robot's unknown hand-support force.

The unchanged signed individual-joint torque ablation from the previous tests
is used; calibrated position conversion still retains thigh/shin coupling.
The plant is an ASSUMPTION:

```text
drive = sent_PWM * physical_PWM_polarity /1000
ideal: tau_motor = clamp(1.28 * motor_gain * drive, +/-1.024 Nm)
dc:    tau_motor = clamp(1.28 * motor_gain * (drive - motor_velocity/2.3), +/-1.024 Nm)
```

The motor velocity and torque are mapped with the existing signed individual
joint permutation. `motor_gain` changes the simulated motor response, not the
P/I/D coefficients. The plant has not been identified from real PWM/current/
torque measurements and is not an exact electrical or linkage model.
The optional `implicit-pd` backend has I=0 and joint-unit P/D, so it is only
a separate reference and must not be described as the real raw/PWM PID.

## Termination and statistics

Use the checkpoint's original `mdp.illegal_contact` predicate: base_link force
history >1N. Automatic reset is suppressed to preserve the trajectory, but
first true is the case's terminal event. PID512raw/raw-range guards are
separate competing events. Post-terminal samples never enter the statistics.

The real target is mean absolute FINAL GOAL minus measured position132.890raw,
averaged across twelve motors. Internal REFERENCE minus measured position is
reported independently; the real mean is42.426raw. A10% final-error band is
declared in advance. Error is measured before each20ms output interval. A
one-second rolling-error window is also recorded where enough samples exist.
Contact is localized to a20ms interval, not an exact substep timestamp.

Matching an overall error averaged over a short failing run is not equivalent
to matching the real12.58s trajectory. In particular, the pre-contact mean,
last1s mean, per-motor errors, internal error and profile gap can differ.

## Validation / interpretation

Runtime checks verify actual PhysX gain readback, C state/config sizes,
action-to-angle and absolute-angle-to-raw independently, and fixed model
configuration. The artifact's `analyze.py` independently recomputes censored
means, verifies force >1N at first contact, checks P=Kp*error, I/PWM caps and
the signed P+I+D output against recorded PWM.

The near-real gain grid was P4.5/5/5.5, I2.5/3/3.5, D0.08/0.10/0.12. Final
response searches retain EXACT P5/I3/D0.10. Read `REPORT.md` and `analysis.json`
in the result directory for outcomes, including conflicting physics-interval
results; do not select only the runs that fall. These are mechanical contact
experiments, not predictions of firmware current/temperature/UART shutdowns.

No firmware, physical motor registers, deployment mapping or training defaults
are modified. Headless computation is tested; container Vulkan GUI/probe errors
remain outside this experiment's validation scope.
