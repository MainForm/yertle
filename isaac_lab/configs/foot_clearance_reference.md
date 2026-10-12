# Foot clearance reference

`phase_swing_clearance` and `swing_foot_clearance_barrier` now measure
the lowest vertex on the authored rubber sole collision surface, rather than
the `*_shin` body origin. The shin origin is near the knee, not the ground
contact point. Thresholds (4.5 cm and 3.5 cm), weights and gates are unchanged.

`foot_geometry.py` reads the active articulation's USD once per environment,
selects Rubber material face subsets only inside CollisionAPI hierarchies, applies all
mesh-to-link transforms, then rotates those vertices using the live shin
world quaternion (w,x,y,z). Taking minimum world Z handles changes of leg
orientation; a fixed vertical subtraction would not. This is the nominal
authored collision surface, not a measured contact point or a claim about
PhysX collision margin/convex-cooking accuracy.

Flat-ground height is zero in this task. Heights are currently world-Z meters,
not distance above a queried rough terrain surface. Contact sensing still uses
the existing shin collision bodies; stance/workspace rewards are not changed.
The model USD, joint axes/frames/limits, observations, actions, and calibrated
hardware API mapping are unchanged. Model changes without an identifiable
rubber sole fail explicitly instead of silently reverting to the link origin.

Already-running training keeps its loaded reward code and is not affected.
Apply this change by creating a new environment/run; historical clearance
values measured at the link origin are not directly comparable to new ones.

Verification: CPU geometry/clearance regression tests and
`tests/check_foot_geometry_sim.py --headless` on the actual restored-axis USD.
