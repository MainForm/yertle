# Sole-only ground-contact penalties

## Before / after

| Item | Before | After |
| --- | --- | --- |
| Base penalty | Net force history >1 N, weight -1.0 | Any qualifying ground contact, weight -1.0 |
| Shoulder/thigh penalty | Net force history >1 N, weight -0.5 | Any qualifying ground contact, weight -0.5 |
| Shin nonsole contact | Whole shin treated as foot; no separate penalty | New `nonsole_foot_contact_penalty`, weight -0.5 per offending shin |
| Geometry / API / policy inputs | Existing model / 55-D obs / 12-D actions | Unchanged |

Individual `RigidContactView.get_contact_data` points/counts/separations are
used without force magnitude. The exact ground **collider** path is resolved
from the flat terrain root; filtering the root Xform can report no contacts.
One body per view avoids multi-body filter/average-contact limitations. Contact
points are not averaged: an allowed sole point cannot hide a simultaneous shin
contact. A body incurs one count, not one penalty per contact point.

Reported separation must be <=1 mm (including penetration). This tolerance is
explicit because PhysX can generate contacts before visible surfaces touch.
For shins, transform each point to the body frame and calculate exact nearest
triangle-surface distance to the USD **Rubber material collision surface**.
Distances <=2 mm are allowed. This surface is the authored rubber foot-end
region, including its curved material surface; it is not a new calibrated
hardware boundary or a hand-selected anatomical tread-only subset. Cooking
and geometry mismatch remain possible and should be checked after model edits.

The ground snapshot is sampled at reward computation (last physics tick of
the control step), not an exhaustive continuous-time collision guarantee.
Default contact-buffer capacity is 64 points per env/body; saturation fails
explicitly instead of silently classifying incomplete data. Support is scoped
to the existing flat task with one ground collider, not general obstacles or
rough terrain. No collision meshes, materials, axes, masses or joints change.

Existing phase/contact/air-time and feet_slide gates remain unchanged; their
1 N threshold is NOT globally replaced. Only forbidden-contact penalties use
the new force-independent detector. Other reward formulas/weights are retained.
Existing training retains its loaded config until a new env/run is created.

Verification: CPU triangle-interior/edge/vertex distances, empty data, zero-force
contact existence, positive separation, independent environments, mixed sole
and nonsole contacts and rigid transforms; real two-env PhysX contact buffers.
At the current standing reference, the real LF shin has a rubber contact near
the sole surface plus knee-end contacts ~11.8 cm from it. The new penalty
correctly counts those nonsole contacts; do not broaden the region to hide them.
