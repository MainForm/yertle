# F103 position PID gain trial: P10 / I3 / D0.10

Historical trial. Superseded by [P5 / I3 / D0.10](PID_GAIN_TRIAL_5_3.md).
The polarity notes below describe the earlier trial, not the current table.

User-requested settings: P10, I3, integral output limited to +/-100 PWM raw.
Select D0.10 as an initial measured-position damping trial, with no D filter or
PWM slew limit. PWM remains capped at +/-800. Per-motor polarity is unchanged:
all IDs except3/9 use-1;3/9 retain+1 pending a physical direction check.

Evidence: robot_20261026_204417_16084.jsonl, U at38.7054s, PC collision release
at41.1552s. Motor7 held+101 raw error with I+20/PWM-222; motor10 held-132raw
with I-20/PWM+284. The old integral-output cap was already binding. Raising Ki
alone would not raise its steady-state contribution, so raise the cap to100
as requested. Conditional integration anti-windup remains active at duty800.

On the67 received U PID sets, measured derivative magnitude was at most450
steps/s for ID7 and500steps/s for ID10 (from old D/0.05). D0.10 would contribute
up to45/50raw on that historical trajectory; this is not a closed-loop forecast
after changing gains. Actual PID dt in these records ranged12..30ms. A1raw
change over20ms produces5raw of derivative output. Derivative remains on measured
position, so target steps alone do not create a derivative kick.

This firmware changes gains and the I-output cap only. Native servo EEPROM PID,
raw calibration, API mapping, signed following error512, PC current protection,
collision stop and firmware temperature/voltage limits are retained. No hardware
motion or successful physical gain validation is claimed by numerical tests.
Flash the standard yertle_f103rb.bin, reset and use the existing CLI F3/F5/H/U
manual sequence to gather a new real-world response before trying policy P.
