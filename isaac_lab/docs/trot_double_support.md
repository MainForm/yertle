# Four-foot support between diagonal swings

`--trot-duty-factor 0.6` in train and play opts into a stance fraction of 60%.
The default remains 50%, including the original phase and transition behavior.
Offsets in LF/RF/LB/RB order remain `(0, 0.5, 0.5, 0)`.

| Global phase | Desired foot support |
|---|---|
| [0, 0.1) | all four |
| [0.1, 0.5) | LF + RB |
| [0.5, 0.6) | all four |
| [0.6, 1) | RF + LB |

At 1.7 Hz each overlap lasts 0.1/1.7 = 58.82 ms; each foot swings
0.4/1.7 = 235.29 ms. The reward guide does not force physical contact.
The existing all-feet-airborne cost remains active even during transition grace.
Grace follows both touchdown and liftoff edges when duty exceeds 50%.

The duty observation changes from 0.5 to 0.6 when enabled; observation shape
(55), action shape (12), phase offsets, motor IDs/signs/raw calibration, action
references and joint axes remain unchanged. Match train/play settings. Do not
use 60% for an old checkpoint without a deliberate evaluation or retraining.
New flags omitted do not add a config field, preserving old resume digests.

The v6.21.6 fresh profile also uses the separately committed 1.5 cm phase swing
height and removal of the redundant 3.5 cm clearance cost. Thus it is not a
frequency/duty-only comparison against older v6.21.3 training.

Run from the repository root with `/isaac-sim/python.sh isaac_lab/train.py`
and the `train_args` listed in `configs/v6_21_6_training.json`.
