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

## Fresh comparison without travel matching

`v6_21_3_training.json` selects 1.7 Hz and `v6_21_4_training.json` selects
1.2 Hz. Both profiles start fresh for 7000 updates with 1024 environments and
seed 0, without a checkpoint or the removed travel matching term. Their
training arguments differ ONLY in the fixed frequency. Both retain the
v6.20.3 reward configuration, 1.9 Nm / 4.6 rad/s actuator caps and 2.3 rad/s
soft speed penalty threshold. No existing profile/global default is changed.
Each process loads its own frequency into env_cfg before environment creation;
parallel runs do not share a mutable gait clock or checkpoint state.

Validation: 173 standalone tests passed. Each profile completed an actual
8-env/2-update Docker GPU test and saved full environment checkpoints. The
two saved environment configs differ only in trot_frequency_hz; comparison
against the v6.20.3 test config likewise changes only frequency. The removed
travel matching reward is absent from saved configs, event tags and state.
