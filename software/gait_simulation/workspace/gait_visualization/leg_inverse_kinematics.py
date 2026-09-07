'''Explain planar leg inverse kinematics with six Matplotlib plots in two windows.

Edit the initial link polar pairs (length in meters, joint angle in degrees)
and target coordinates in main.
The target is (-0.05, -0.15) m to show a clearly bent knee.

X describes forward/backward foot movement and Z describes foot height,
measured in meters relative to the shoulder. Y stays zero because the
shoulder angle is fixed. Positive joint rotation turns from -Z toward -X.
The knee angle q2 is relative to the thigh, so the shin direction is q1 + q2.

The first window shows the initial pose, target, and cosine rule for q2.
The second shows the beta components, q1 angle relationship, and IK solution. The solution keeps the initial knee bend direction. Unreachable
targets raise ValueError rather than being silently moved into reach.
'''

import math
from collections.abc import Sequence

import matplotlib.pyplot as plt
from matplotlib.axes import Axes


def convert_polar_to_cartesian(length: float, angle: float) -> tuple[float, float]:
    '''Convert length (m) and angle (rad) to (x, z), rotating from -Z toward -X.'''
    return (-length * math.sin(angle), -length * math.cos(angle))


def add_formulas(ax: Axes, lines: Sequence[str]) -> None:
    '''Join the supplied strings with line breaks and display them below the plot.'''
    ax.text(0.5, -0.2, "\n".join(lines), transform=ax.transAxes,
            ha="center", va="top", fontsize=11, linespacing=1.8)


def calculate_inverse_kinematics(
    target: tuple[float, float],
    initial_thigh_polar: tuple[float, float],
    initial_shin_polar: tuple[float, float],
) -> tuple[float, float]:
    '''Return (thigh, shin) joint angles in degrees for a target (x, z) in meters.

    Each initial polar pair is (link length in meters, joint angle in degrees).
    The thigh angle starts at -Z and turns toward -X when positive. The shin
    angle is relative to the thigh, so its absolute direction is their sum.
    Initial angles select the knee bend branch and nearby equivalent angles;
    link lengths determine the reachable range and the IK geometry.
    '''
    thigh_length, initial_thigh_degrees = initial_thigh_polar
    shin_length, initial_shin_degrees = initial_shin_polar
    x, z = target
    if not all(math.isfinite(value) for value in (
        x, z, thigh_length, shin_length, initial_thigh_degrees, initial_shin_degrees
    )):
        raise ValueError('Target coordinates, link lengths, and initial angles must be finite.')
    if thigh_length <= 0.0 or shin_length <= 0.0:
        raise ValueError('Link lengths must be positive.')

    # The foot must lie within the annulus reachable by the two links.
    distance = math.hypot(x, z)
    if distance > thigh_length + shin_length + 1e-12 or distance < abs(thigh_length - shin_length) - 1e-12:
        raise ValueError('Target is outside the reachable range of the leg.')

    # The cosine rule gives the relative knee angle, not the triangle's interior angle.
    cos_q2 = (distance**2 - thigh_length**2 - shin_length**2) / (2 * thigh_length * shin_length)
    cos_q2 = max(-1.0, min(1.0, cos_q2))
    initial_q2 = math.radians(initial_shin_degrees)
    bend_sign = -1.0 if math.sin(initial_q2) < 0.0 else 1.0
    q2 = bend_sign * math.acos(cos_q2)

    if distance <= 1e-12 and abs(thigh_length - shin_length) <= 1e-12:
        # At the shoulder, equal links fold fully and any thigh angle is valid.
        q1 = math.radians(initial_thigh_degrees)
    else:
        # alpha points toward the target; beta is the offset from thigh to target.
        alpha = math.atan2(-x, -z)
        # Resolve the target vector along and perpendicular to the thigh.
        beta = math.atan2(
            shin_length * math.sin(q2),
            thigh_length + shin_length * math.cos(q2),
        )
        q1 = alpha - beta

    # Choose equivalent angles near the initial pose to avoid full-turn jumps.
    thigh_degrees = initial_thigh_degrees + (math.degrees(q1) - initial_thigh_degrees + 180.0) % 360.0 - 180.0
    shin_degrees = initial_shin_degrees + (math.degrees(q2) - initial_shin_degrees + 180.0) % 360.0 - 180.0
    return thigh_degrees, shin_degrees


def setup_axes(ax: Axes, thigh_length: float, shin_length: float) -> None:
    '''Use a shared workspace, equal scaling, and shoulder-centered X-Z axes.'''
    reach = thigh_length + shin_length
    ax.set_xlim(-reach - 0.04, reach + 0.04)
    ax.set_ylim(-reach - 0.05, reach + 0.04)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X: forward / backward (m)')
    ax.set_ylabel('Z: foot height (m)')
    ax.grid(alpha=0.2)
    ax.plot(0.0, 0.0, 'ks', markersize=6)
    ax.annotate('Shoulder', (0.0, 0.0), xytext=(8, 8), textcoords='offset points')


def plot_leg(
    ax: Axes,
    knee: tuple[float, float],
    foot: tuple[float, float],
    faded: bool = False,
) -> None:
    '''Draw the thigh first, then the shin; optionally show a faded initial pose.'''
    alpha = 0.3 if faded else 1.0
    style = '--' if faded else '-'

    # Draw the thigh from the shoulder to the knee.
    ax.plot([0.0, knee[0]], [0.0, knee[1]], style, color='#2563eb',
            linewidth=3, alpha=alpha, label='Thigh')
    ax.plot(*knee, 'o', color='#2563eb', alpha=alpha)

    # Draw the shin from the knee to the foot.
    ax.plot([knee[0], foot[0]], [knee[1], foot[1]], style, color='#e67700',
            linewidth=3, alpha=alpha, label='Shin')
    ax.plot(*foot, 'o', color='#e67700', alpha=alpha)


def draw_angle(ax: Axes, start: float, angle: float, radius: float,
               label: str, color: str,
               center: tuple[float, float] = (0.0, 0.0)) -> None:
    '''Draw a directed angle around center, rotating from -Z toward -X.'''
    offsets = [convert_polar_to_cartesian(radius, start + angle * i / 60)
               for i in range(61)]
    points = [(center[0] + x, center[1] + z) for x, z in offsets]
    ax.plot([p[0] for p in points], [p[1] for p in points], color=color, lw=2)
    if abs(angle) > 1e-9:
        ax.annotate('', xy=points[-1], xytext=points[-5],
                    arrowprops={'arrowstyle': '->', 'color': color, 'lw': 2})
    label_offset = convert_polar_to_cartesian(radius + 0.012, start + angle / 2)
    ax.text(center[0] + label_offset[0], center[1] + label_offset[1],
            label, color=color, fontsize=12, ha='center', va='center')


def main() -> None:
    '''Show the q2 calculation first, then beta, q1, and the verified result.'''
    # Polar pairs: (length in meters, joint angle in degrees).
    # The shin angle is relative to the thigh, as in the FK example.
    initial_thigh_polar = (0.13, -25.0)
    initial_shin_polar = (0.13, 50.0)
    thigh_length, initial_thigh_degrees = initial_thigh_polar
    shin_length, initial_shin_degrees = initial_shin_polar

    # Convert the thigh direction to radians; its endpoint is the knee.
    initial_thigh_angle = math.radians(initial_thigh_degrees)
    initial_knee = convert_polar_to_cartesian(thigh_length, initial_thigh_angle)

    # The shin direction includes the knee angle relative to the thigh.
    initial_shin_angle = math.radians(initial_thigh_degrees + initial_shin_degrees)
    initial_shin_vector = convert_polar_to_cartesian(shin_length, initial_shin_angle)

    # Add the shin vector to the knee position to locate the initial foot.
    initial_foot = (
        initial_knee[0] + initial_shin_vector[0],
        initial_knee[1] + initial_shin_vector[1],
    )

    # Place the target closer to the shoulder to make the knee bend clear.
    target = (-0.05, -0.15)
    gait_x_displacement = target[0] - initial_foot[0]
    gait_z_displacement = target[1] - initial_foot[1]

    # Solve IK and verify the resulting foot position through forward kinematics.
    thigh_degrees, shin_degrees = calculate_inverse_kinematics(
        target, initial_thigh_polar, initial_shin_polar
    )

    # Rebuild the knee position from the solved thigh angle.
    knee = convert_polar_to_cartesian(thigh_length, math.radians(thigh_degrees))

    # Add the solved shin vector to the knee to verify the final foot position.
    shin_vector = convert_polar_to_cartesian(
        shin_length, math.radians(thigh_degrees + shin_degrees)
    )
    foot = (knee[0] + shin_vector[0], knee[1] + shin_vector[1])
    distance = math.hypot(*target)
    error = math.hypot(foot[0] - target[0], foot[1] - target[1])

    fig, first_axes = plt.subplots(1, 3, figsize=(18, 9))
    detail_fig, detail_axes = plt.subplots(1, 3, figsize=(18, 9))
    axes = [*first_axes, *detail_axes]
    for window in (fig, detail_fig):
        window.subplots_adjust(left=0.08, right=0.97, top=0.88, bottom=0.36,
                               wspace=0.4)
    fig.suptitle('Window 1: find q2 from the target', fontsize=18)
    detail_fig.suptitle('Window 2: find q1 using q2', fontsize=18)

    #---------------------------------------------
    # First plot: the initial leg pose and foot coordinates.
    initial_axis : Axes = axes[0]
    setup_axes(initial_axis, thigh_length, shin_length)
    initial_axis.set_title('1. Initial pose', fontsize=16)
    plot_leg(initial_axis, initial_knee, initial_foot)
    initial_axis.annotate('Initial foot', initial_foot, xytext=(10, -15), textcoords='offset points')
    initial_axis.legend(loc='upper right')
    add_formulas(initial_axis, (
        rf'$q_1 = {initial_thigh_degrees:g}^\circ,\quad q_2 = {initial_shin_degrees:g}^\circ$',
        r'$\theta_1=q_1,\quad \theta_2=q_1+q_2$',
        rf'Foot = ({initial_foot[0]:.4f}, {initial_foot[1]:.4f}) m',
        rf'$L_1={thigh_length:g}$ m, $L_2={shin_length:g}$ m',
    ))

    #---------------------------------------------
    # Second plot: the target and the requested movement from the initial foot.
    target_axis : Axes = axes[1]
    setup_axes(target_axis, thigh_length, shin_length)
    target_axis.set_title('2. Target position', fontsize=16)
    plot_leg(target_axis, initial_knee, initial_foot, faded=True)
    target_axis.plot(*target, '*', color='#be123c', markersize=14, label='Target')
    target_axis.annotate('Target', target, xytext=(10, 5), textcoords='offset points')
    target_axis.annotate('', xy=target, xytext=initial_foot,
                         arrowprops={'arrowstyle': '->', 'color': '#be123c', 'lw': 2})
    target_axis.plot([0.0, target[0]], [0.0, target[1]], ':', color='#64748b')
    target_axis.legend(loc='upper right')
    add_formulas(target_axis, (
        rf'$\Delta x={gait_x_displacement:.4f}$ m, $\Delta z={gait_z_displacement:.4f}$ m',
        rf'Target = ({target[0]:.4f}, {target[1]:.4f}) m',
        rf'$r=\sqrt{{x^2+z^2}}={distance:.4f}$ m',
        r'Reachable when $|L_1-L_2|\leq r\leq L_1+L_2$',
    ))

    #---------------------------------------------
    # Third plot: the knee interior angle gamma is supplementary to |q2|.
    cosine_axis : Axes = axes[2]
    setup_axes(cosine_axis, thigh_length, shin_length)
    cosine_axis.set_title('3. Find q2 with cosine rule', fontsize=16)
    plot_leg(cosine_axis, knee, target)
    cosine_axis.fill([0.0, knee[0], target[0]], [0.0, knee[1], target[1]],
                     color='#16a34a', alpha=0.08)
    cosine_axis.plot([0.0, target[0]], [0.0, target[1]], '--', color='#16a34a')
    cosine_axis.plot(*target, '*', color='#be123c', markersize=14)
    cosine_axis.annotate('Target', target, xytext=(-50, -18), textcoords='offset points')
    for label, point, offset in (
        (r'$L_1$', (knee[0] / 2, knee[1] / 2), (10, 0)),
        (r'$L_2$', ((knee[0] + target[0]) / 2, (knee[1] + target[1]) / 2), (-22, -5)),
        (r'$r$', (target[0] / 2, target[1] / 2), (-18, 0)),
    ):
        cosine_axis.annotate(label, point, xytext=offset,
                             textcoords='offset points', fontsize=13)
    # Convert the solved thigh angle to radians for the angle arcs.
    q1 = math.radians(thigh_degrees)
    # Select the knee bend direction from the initial shin joint angle.
    bend_sign = -1.0 if math.sin(math.radians(initial_shin_degrees)) < 0.0 else 1.0
    # Use the cosine rule to recover the relative knee angle.
    cos_q2 = (distance**2 - thigh_length**2 - shin_length**2) / (2 * thigh_length * shin_length)
    # Clamp rounding errors to the acos domain, then apply the bend direction.
    q2 = bend_sign * math.acos(max(-1.0, min(1.0, cos_q2)))
    # The interior knee angle is supplementary to the magnitude of q2.
    gamma = math.pi - abs(q2)

    # Extend the thigh 0.075 m beyond the knee as the reference ray for q2.
    extension = convert_polar_to_cartesian(0.075, q1)
    cosine_axis.plot([knee[0], knee[0] + extension[0]],
                     [knee[1], knee[1] + extension[1]], ':', color='#64748b')
    # Draw q2 from the thigh continuation toward the shin.
    draw_angle(cosine_axis, q1, q2, 0.055, r'$q_2$', '#e67700', center=knee)
    # Skip the interior-angle arc when the target coincides with the shoulder.
    if distance > 1e-12:
        # Start toward the shoulder and sweep toward the shin inside the triangle.
        draw_angle(cosine_axis, q1 + bend_sign * math.pi, -bend_sign * gamma,
                   0.03, r'$\gamma$', '#9333ea', center=knee)
    add_formulas(cosine_axis, (
        r'$r^2=L_1^2+L_2^2-2L_1L_2\cos\gamma$',
        r'$\gamma=\pi-|q_2|\quad\Rightarrow\quad\cos\gamma=-\cos q_2$',
        r'$c=\frac{r^2-L_1^2-L_2^2}{2L_1L_2},\quad c=\mathrm{clip}(c,-1,1)$',
        r'$s=-1$ if $\sin(q_{2,\mathrm{initial}})<0$, else $s=+1$',
        rf'$q_2=s\,\arccos(c)={math.degrees(q2):.2f}^\circ$',
        r'Dotted ray: thigh continuation; triangle drawn in solved pose.',
    ))

    #---------------------------------------------
    # Fourth plot: resolve the target in a coordinate frame along the thigh.
    triangle_axis : Axes = axes[3]
    setup_axes(triangle_axis, thigh_length, shin_length)
    triangle_axis.set_title('4. Find beta with atan2', fontsize=16)
    plot_leg(triangle_axis, knee, target)
    triangle_axis.plot([0.0, target[0]], [0.0, target[1]], '--', color='#16a34a')
    triangle_axis.plot(*target, '*', color='#be123c', markersize=14)
    triangle_axis.annotate('Target', target, xytext=(-50, -18), textcoords='offset points')
    triangle_axis.annotate('Knee', knee, xytext=(10, 0), textcoords='offset points')

    # Convert the solved joint angles to radians for trigonometry and plotting.
    q1 = math.radians(thigh_degrees)
    q2 = math.radians(shin_degrees)
    # A target away from the shoulder defines a direction and a triangle.
    if distance > 1e-12:
        # Measure the target direction from -Z, with positive rotation toward -X.
        alpha = math.atan2(-target[0], -target[1])
        # Resolve the shoulder-to-target vector into signed thigh-frame components.
        # A includes the thigh length; B comes only from the shin.
        along = thigh_length + shin_length * math.cos(q2)
        perpendicular = shin_length * math.sin(q2)
        # atan2 preserves the quadrant of the signed angle from thigh to target.
        beta = math.atan2(perpendicular, along)
        # P is the target's projection onto the thigh line. Positive normal
        # points in the direction of increasing joint angle (-Z toward -X).
        tangent = convert_polar_to_cartesian(1.0, q1)
        normal = (-math.cos(q1), math.sin(q1))
        # Convert the along-thigh component back to world coordinates to locate P.
        projection = (along * tangent[0], along * tangent[1])
        # Shade the right triangle formed by the shoulder, P, and the target.
        triangle_axis.fill([0.0, projection[0], target[0]],
                           [0.0, projection[1], target[1]],
                           color='#9333ea', alpha=0.08)
        # Draw the A and B component arrows and label their midpoints.
        for start, end, color, label, offset in (
            ((0.0, 0.0), projection, '#0891b2', 'A', (12, 0)),
            (projection, target, '#9333ea', 'B', (-5, 12)),
        ):
            triangle_axis.annotate('', xy=end, xytext=start,
                                   arrowprops={'arrowstyle': '->', 'color': color,
                                               'lw': 2, 'linestyle': '--'})
            midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
            triangle_axis.annotate(label, midpoint, xytext=offset,
                                   textcoords='offset points', color=color, fontsize=13)
        # Mark the foot of the perpendicular on the thigh line.
        triangle_axis.plot(*projection, 'o', color='#0891b2', markersize=4)
        triangle_axis.annotate('P', projection, xytext=(8, 8), textcoords='offset points')
        # Mark the right angle at P, directed toward the shoulder and target.
        # Both components must be nonzero for a visible right-angle marker.
        if abs(along) > 1e-12 and abs(perpendicular) > 1e-12:
            # Keep the marker small enough to fit within both triangle legs.
            size = min(0.01, abs(along) / 4, abs(perpendicular) / 4)
            # Orient unit vectors from P toward the shoulder (u) and target (v).
            u = tuple(-math.copysign(1.0, along) * v for v in tangent)
            v = tuple(math.copysign(1.0, perpendicular) * n for n in normal)
            # Trace three corners of a square to form the right-angle marker.
            corner = [(projection[0] + size * dx, projection[1] + size * dz)
                      for dx, dz in (u, (u[0] + v[0], u[1] + v[1]), v)]
            triangle_axis.plot([p[0] for p in corner], [p[1] for p in corner],
                               color='#64748b', lw=1)
        # Draw beta at the shoulder, limiting the arc radius for nearby targets.
        draw_angle(triangle_axis, q1, beta, min(0.04, distance / 3),
                   r'$\beta$', '#9333ea')
        # Display the component values and the resulting signed angle.
        add_formulas(triangle_axis, (
            rf'Use solved $q_2={shin_degrees:.2f}^\circ$ from plot 3.',
            rf'$A=L_1+L_2\cos q_2={along:.4f}$ m (along thigh)',
            rf'$B=L_2\sin q_2={perpendicular:.4f}$ m (perpendicular)',
            rf'$\beta=\mathrm{{atan2}}(B,A)={math.degrees(beta):.2f}^\circ$',
            r'P: target projection onto the thigh line; A and B are signed.',
        ))
    # At the shoulder, the target ray collapses and its direction is undefined.
    else:
        add_formulas(triangle_axis, (
            r'$r=0$: the triangle collapses.',
            r'Target direction and $\beta$ are undefined.',
            r'$q_1$ keeps its initial value.',
        ))
        
    #---------------------------------------------
    # Fifth plot: directed angles share the shoulder as their vertex.
    angle_axis : Axes = axes[4]
    setup_axes(angle_axis, thigh_length, shin_length)
    angle_axis.set_title('5. Find q1 = alpha - beta', fontsize=16)
    plot_leg(angle_axis, knee, target)
    angle_axis.plot([0.0, 0.0], [0.0, -0.28], ':', color='#64748b')
    angle_axis.annotate('-Z reference', (0.0, -0.28), xytext=(8, 0),
                        textcoords='offset points')
    angle_axis.plot([0.0, target[0]], [0.0, target[1]], '--', color='#16a34a')
    angle_axis.plot(*target, '*', color='#be123c', markersize=14)
    angle_axis.annotate('Target', target, xytext=(-50, -18), textcoords='offset points')
    # Use the equivalent q1 consistent with alpha - beta to avoid full-turn arcs.
    if distance > 1e-12:
        draw_angle(angle_axis, 0.0, alpha - beta, 0.055, r'$q_1$', '#2563eb')
        draw_angle(angle_axis, alpha - beta, beta, 0.10, r'$\beta$', '#9333ea')
        draw_angle(angle_axis, 0.0, alpha, 0.185, r'$\alpha$', '#16a34a')
        add_formulas(angle_axis, (
            rf'$\alpha=\mathrm{{atan2}}(-x,-z)={math.degrees(alpha):.2f}^\circ$',
            r'$q_1$: -Z reference $\rightarrow$ thigh',
            r'$\beta$: thigh $\rightarrow$ target ray (signed)',
            r'$\alpha=q_1+\beta\quad\Longrightarrow\quad q_1=\alpha-\beta$',
            r'Positive rotation: -Z toward -X; angles modulo $360^\circ$.',
        ))
    else:
        add_formulas(angle_axis, ('Target is at the shoulder: no target ray.',
                                  r'$\alpha,\beta$ undefined; keep initial $q_1$.'))

    #---------------------------------------------
    # Sixth plot: the solved leg overlaid on the initial pose for comparison.
    result_axis : Axes = axes[5]
    setup_axes(result_axis, thigh_length, shin_length)
    result_axis.set_title('6. IK result', fontsize=16)
    plot_leg(result_axis, initial_knee, initial_foot, faded=True)
    plot_leg(result_axis, knee, foot)
    result_axis.plot(*target, '*', color='#be123c', markersize=14)
    result_axis.annotate('Foot at target', foot, xytext=(10, -15), textcoords='offset points')
    result_axis.annotate('New knee', knee, xytext=(10, 5), textcoords='offset points')
    add_formulas(result_axis, (
        r'$q_2=s\,\arccos\left(\frac{r^2-L_1^2-L_2^2}{2L_1L_2}\right)$',
        r'$\beta=\mathrm{atan2}(L_2\sin q_2, L_1+L_2\cos q_2)$',
        r'$\alpha=\mathrm{atan2}(-x,-z),\quad q_1=\alpha-\beta$',
        rf'$q_1={thigh_degrees:.2f}^\circ,\quad q_2={shin_degrees:.2f}^\circ$',
        rf'Foot = ({foot[0]:.4f}, {foot[1]:.4f}) m; error = {error:.1e} m',
    ))

    plt.show()
    plt.close(fig)
    plt.close(detail_fig)


if __name__ == '__main__':
    main()
