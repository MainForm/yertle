# Action second-difference penalty

Term `action_second_difference_l2`, current weight **-0.05** (strengthened 5x
from the initial -0.01 at the user's request):

`cost = sum_j (a[t,j] - 2*a[t-1,j] + a[t-2,j])^2`.

Uses the `joint_pos` action term's dimensionless raw actions before its 0.25 rad
scale, not measured joint acceleration or degrees. Existing `action_rate_l2`
weight -0.04 and all other reward settings remain unchanged. No dt^-4 scaling;
RewardManager applies its normal control-dt multiplication once (50 Hz here).
No velocity-command/phase gate. Constant actions and constant action increments
have zero second difference; alternating targets have a larger cost. Real gait
curvature is also penalized, so this is a training experiment, not a perfect
frequency filter or a guarantee of walking.

At alternating amplitude A, each joint's second difference has magnitude 4A.
With weight -0.01, its cost equals the existing -0.04 first-difference cost
for this alternating sequence; the current -0.05 makes it five times that cost.
At 2.2 Hz the second-difference contribution is
much smaller than at 25 Hz for the same amplitude and 50 Hz control rate.

History is per environment; RewardManager reset clears only selected envs.
The first two action samples after reset are unpenalized. Repeated reward
evaluation within one common control step returns the cached result without
advancing history. The action tensors themselves are never modified.

No joint axes, motor/raw API mappings, pose, action/observation semantics,
actuator limits or termination changes. Checkpoints remain structurally
compatible; the training objective changes only for newly created environments.
Already-running training keeps its loaded configuration. This commit does not
stop/restart training or designate a new training version.
