# F103 position PID trial: P5 / I3 / D0.10

User request: reduce P from10 to5 for the next H/seated comparison. This is
not the earlier suggested I0 experiment: I remains3, D remains0.10 and the
integral-output clamp remains+/-100 PWM raw. PWM ceiling remains+/-800.
All12 physical motor polarities remain-1. No D filter or PWM slew is added.
Only the default proportional gain changes in production code.

Baseline: robot_20261026_221328_9596.jsonl. After H reached a fixed2048 target,
IDs4 and10 oscillated at about4.5Hz. In the9.6..14.0836s observed window,
ID10 moved1987..2107raw, with max absolute P610, I11.83, D283.33 and PWM725.
ID4 had P400, I6.27, D183.33 and PWM502. These are measured controller terms,
not proof that P alone caused the real motion or that P5 will stabilize it.

Use the same hardware setup and CLI options for an H-only comparison after
flashing the standard yertle_f103rb.bin and resetting: F3, F5 ready, H. Compare
motor4/10 position oscillation, current, P/I/D and PWM after the seated target
is fixed. Do not treat an ACK as measured arrival. The user performs the real
motor trial; host numerical/integration checks do not validate physical gains.

Calibration, API mapping, policy/model, reference speed/ACC, direct PWM and
native-position behavior remain unchanged. PC2A per motor /6A sum protection,
F103 following/feedback/temperature/voltage/target-liveness guards remain.
The baseline ended with firmware thermal/voltage reason11 despite normal PC
received values; this P-only trial does not fix or diagnose that separate issue.
