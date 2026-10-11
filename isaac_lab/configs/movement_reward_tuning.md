# Movement reward tuning (after v6.4)

This reward-only experiment does not change joint axes/frames, action offsets
or scale, observations, hardware raw mapping, joint limits, motor envelope,
torque/speed limits, gait frequency, start poses, or termination settings.

1. `feet_air_time`: threshold 0.50 -> 0.15 seconds; weight +0.50 -> +1.00
   (doubled in the subsequent requested adjustment).
   A fixed 2.2 Hz / 50% duty trot has ~0.227 seconds of scheduled swing.
   Touchdowns after a normal swing now score positively. The signed
   `(last_air_time - threshold)` touchdown-only formula is preserved.
   The gate now matches the phase rewards: commanded XY norm >0.05 m/s OR
   commanded absolute yaw >0.10 rad/s (not measured velocity). No-command
   environments receive zero air-time score.
2. Cauchy linear tracking accepts `components`: default `(0, 1)` retains
   the previous full-command XY mean. `train.py --task flat --vx-only`
   explicitly selects `(0,)`, so vy=0 cannot give an unearned half score.
   `min_scale=0.05`, `relative_scale=0.50`, linear weight +2.0 and yaw
   Cauchy weight +0.75 are unchanged. The historical term/log name remains
   `track_lin_vel_xy_cauchy`; inspect the saved `components` to distinguish
   VX-only and XY scoring. At commanded vx=0.1 with measured vx=vy=0,
   the weighted linear score drops from 1.5 to 1.0; exact vx tracking gives 2.0.
   Yaw=0 still rewards heading stability; this change does not remove it.
3. `phase_unscheduled_contact_penalty`: weight -0.20 -> -0.40; its sensor,
   1 N contact threshold, phase timing and command gate are unchanged.
   Four continuously contacting feet on an active trot command have half
   the feet violating swing: phase schedule +0.1, unscheduled contact -0.2,
   total -0.1 (before dt), rather than cancellation to zero.

Existing running processes retain their loaded reward functions/configuration.
These changes apply only to newly created environments, including a future
resumed training run. Existing checkpoints remain structurally compatible
(55 observation dimensions, 12 joint actions), but the reward objective differs.
No new training or interruption is performed by this code change.
