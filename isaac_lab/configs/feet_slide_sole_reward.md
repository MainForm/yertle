# Sole-point sliding penalty (v6.7)

Only `feet_slide` changes; its weight remains -0.10. Previously the inherited
Isaac Lab reward used shin **COM** planar speed (`body_lin_vel_w`), not actual
sole point speed. The new reward sums contacting feet's planar speeds at an
estimated lowest rubber-surface patch. The >1 N / max over three sensor-history
frames contact gate is unchanged. No command/phase gate or new reward term.

The patch is the vertex centroid within 1 mm of each foot's lowest world-Z
rubber collision surface. Vertex padding is excluded from the centroid. The
cached mesh is rotated with live body quaternions, and speed is calculated as
`v_COM + omega_world x (point_world - COM_world)`. COM offsets and sensor-body
ordering are explicitly handled; vertical speed is excluded from the cost.

This is a flat-ground geometry proxy, **not** a measured PhysX contact patch.
Collision cooking/margins and a sloping surface can make true contacts differ.
Missing rubber geometry fails explicitly; no link-origin fallback is used.

Axes, model files, action reference/scale, observation dimensions, hardware API
calibration, torque/speed caps and all other reward weights/parameters are
unchanged. Historical feet_slide values are not directly equivalent to this
new sole-point metric. This does not guarantee walking or prevent every form
of vibratory/rolling locomotion; compare policy rollouts after training.

Verification: CPU stationary-contact, translating/rotating body, COM offset,
quaternion/lowest-patch, padding, contact threshold/history/body-order tests;
actual Isaac Lab configuration and independent simulated point-velocity check.
