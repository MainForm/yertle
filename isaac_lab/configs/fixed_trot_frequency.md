# Fixed trot clock profiles

Train and play accept `--trot-frequency-hz` to override the shared fixed gait
clock. The default remains 2.2 Hz. v6_21_training.json selects 1.0 Hz and
v6_21_2_training.json selects 1.7 Hz. Pass the same frequency during play.
No observation layout, action reference, joint axes or calibrated API mapping
is changed by the override.

The experimental thigh/shin motor travel matching reward was removed on
2026-10-19, including its implementation, registration and CLI switch. It is
not merely assigned a zero weight. Other reward terms and weights, actuator
limits, commands, reset pose and termination conditions are unchanged.

Historical v6.21/v6.21.2 checkpoints, saved params and events retain their
original travel reward history. They were not edited or deleted. To restore
their full environment state exactly, use the original source commit with
the original reward term. Removing a stateful reward changes the environment
digest/layout; loading only compatible actor weights for play is different
from restoring the complete training environment.

Removal validation: 172 standalone regression tests passed. The reward
configuration module is identical to v6.20.3 (b9c68d6). Only the experimental
travel term/integration was removed; frequency override support remains.
No new simulation training was launched for this removal.
