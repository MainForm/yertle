# Estimated total current excess reward

Status: **registered in the flat environment** as
`estimated_total_current_excess`. The user selected the provisional
specification-based coefficient after inspecting the real logs. Neither the
current/torque coefficient nor the reward weight has been calibrated.

The Isaac Lab implicit actuator does not model electrical current. This term
is an **uncalibrated torque-derived proxy**, not an actual battery current or
an enforced 7 A limit. The estimate sums the absolute motor-torque proxy for
all twelve motors and multiplies by a provisional 1.3073 A/Nm. This ratio is
2.5 A / (19.5 kgf cm * 0.0980665 Nm/kgf cm), from ST3215 7.4 V locked-rotor
specifications: https://www.waveshare.com/product/modules/st3215-servo.htm .
Those single-point specifications do not identify current at 6 V, at speed,
or battery input current. No motor no-load current, losses, PWM efficiency,
mechanical friction, regeneration, or voltage variation is represented.
Calibrate against simultaneous real motor load/velocity/current and, for supply
current, independently measured battery current before quantitative use.

The ideal coupled-leg assumption is motor thigh torque = joint thigh torque
minus joint shin torque, and motor shin torque = joint shin torque. Joint
efforts come from implicit-actuator `applied_torque` telemetry, which is an
estimate and need not equal actual solved drive torque. Neither the motor
mapping nor current estimation is calibrated by the spec above.

The reward is `-0.1 * max(I_proxy - 7 A, 0)^2 / (1 A)^2` before the
RewardManager's control-step dt multiplication. For example 7 A yields zero,
8 A yields -0.1 per second of sustained excess, and 9 A yields -0.4 per second.
The weight and one-ampere scale are initial choices to tune, not measured
constants. No clipping/hardware protection is implied. This changes reward
configuration; environments already running must be restarted and old saved
environment-state digests must not be restored as if the reward were unchanged.
