'''Visualize planar forward kinematics of an ideal two-link left front leg.

Coordinates are (x, z) in meters relative to the shoulder:
    X describes forward/backward foot movement.
    Z describes foot height; positions below the shoulder have negative Z.
The shoulder angle is fixed at zero, so there is no sideways motion and Y
stays zero. The shoulder and thigh joint share the origin in this model.
This is a geometric linkage model, not a mesh-derived foot-contact calculation.

Both links point along -Z at zero joint angles. Positive rotation about +Y
turns them toward -X (clockwise when X is right and Z is up). The thigh
joint angle is q1, and the knee angle q2 is relative to the thigh, giving
link directions theta1 = q1 and theta2 = q1 + q2.

For a link of length L, the Cartesian components are:
    x = -L * sin(theta)
    z = -L * cos(theta)
Equivalently, the usual angle measured counterclockwise from +X is
phi = -90 degrees - theta. The inverse direction is atan2(-x, -z).
Joint angles in main are in degrees; trigonometric functions use radians.

Three plots share the same axis limits and scale:
    1. The thigh vector v1, drawn from the origin.
    2. The shin vector v2, drawn from the origin for comparison.
    3. The links drawn head-to-tail, with v1 + v2 locating the foot.
Each plot includes length and angle labels and component equations.
Edit the joint angles in main to explore different poses.
'''

import math
from collections.abc import Sequence

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.patches import Arc, FancyArrowPatch

THIGH_LENGTH = 0.13     # 13 cm
SHIN_LENGTH = 0.13      # 13 cm

def convert_polar_to_cartesian(length: float, angle: float) -> tuple[float, float]:
    '''Convert length (m) and angle (rad) to an (x, z) vector. Angles start at -Z and turn toward -X when positive.'''
    return (-length * math.sin(angle), -length * math.cos(angle))

def arrow(
    ax: Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    color: str,
    dashed: bool = False,
) -> None:
    '''Draw an arrow from start to end using the given color and optional dashed style.'''
    ax.add_patch(FancyArrowPatch(
        start, end, arrowstyle="-|>", mutation_scale=17,
        linewidth=2.7, color=color, linestyle="--" if dashed else "-",
        shrinkA=0, shrinkB=0, zorder=3,
    ))

def length_label(
    ax: Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    text: str,
    color: str,
    offset: tuple[float, float],
) -> None:
    '''Place a label connected to the segment midpoint. The offset specifies the text displacement in points.'''
    midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
    ax.annotate(
        text, xy=midpoint, xytext=offset, textcoords="offset points",
        color=color, fontsize=11,
        ha="left" if offset[0] >= 0 else "right", va="center",
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.9, "pad": 2},
        arrowprops={"arrowstyle": "-", "color": color, "linewidth": 0.8},
    )

def direction_angle(
    ax: Axes,
    start: tuple[float, float],
    degrees: float,
    symbol: str,
    color: str,
    offset: tuple[float, float],
) -> None:
    '''Draw the -Z reference line, angle arc, and label at start. The angle is in degrees and the text offset is in points.'''
    # Plot angles from -Z; positive rotation about +Y leans toward -X.
    wrapped = (degrees + 180.0) % 360.0 - 180.0
    ax.plot([start[0], start[0]], [start[1], start[1] - 0.06],
            ":", color="#94a3b8", linewidth=1.2)
    low, high = sorted((-90.0, -90.0 - wrapped))
    ax.add_patch(Arc(start, 0.06, 0.06, theta1=low, theta2=high,
                     color=color, linewidth=1.7))
    midpoint = math.radians(-90.0 - wrapped / 2)
    arc_point = (start[0] + 0.03 * math.cos(midpoint),
                 start[1] + 0.03 * math.sin(midpoint))
    ax.annotate(
        rf"${symbol} = {degrees:g}^\circ$", xy=arc_point,
        xytext=offset, textcoords="offset points", fontsize=11, color=color,
        ha="left" if offset[0] >= 0 else "right",
        arrowprops={"arrowstyle": "-", "color": color, "linewidth": 0.8},
    )

def setup_axes(
    ax: Axes,
    x_limits: tuple[float, float],
    z_limits: tuple[float, float],
) -> None:
    '''Set the X and Z limits in meters, equal axis scaling, axis labels, grid, and origin marker.'''
    ax.set_xlim(x_limits)
    ax.set_ylim(z_limits)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (m)", fontsize=11)
    ax.set_ylabel("Z (m)", fontsize=11)
    ax.grid(alpha=0.2)
    ax.plot(0, 0, "o", color="#334155", markersize=5, zorder=4)
    ax.annotate("O", (0.0, 0.0), xytext=(-12, 8), textcoords="offset points")


def add_formulas(ax: Axes, lines: Sequence[str]) -> None:
    '''Join the supplied strings with line breaks and display them below the plot.'''
    ax.text(0.5, -0.2, "\n".join(lines), transform=ax.transAxes,
            ha="center", va="top", fontsize=11, linespacing=1.8)


def plot_vector(
    ax: Axes,
    vector: tuple[float, float],
    color: str,
) -> None:
    '''Draw an (x, z) vector from the origin and its component guides in the given color.'''
    # Use the vector already calculated in main as the arrow endpoint.
    arrow(ax, (0.0, 0.0), vector, color)

    # Show the horizontal component and the vertical distance from the origin.
    ax.plot([0, vector[0]], [vector[1], vector[1]], "--",
            color=color, alpha=0.35, linewidth=1)
    ax.plot([0, 0], [0, vector[1]], "--",
            color=color, alpha=0.35, linewidth=1)


def plot_vector_sum(
    ax: Axes,
    v1: tuple[float, float],
    v2: tuple[float, float],
    theta1: float,
    theta2: float,
) -> None:
    '''Draw two (x, z) vectors head-to-tail, their sum, knee and foot positions for plot 3. Both angles are in degrees.'''
    # Add the vectors to locate the foot relative to the shoulder.
    origin = (0.0, 0.0)
    tip = (v1[0] + v2[0], v1[1] + v2[1])
    blue, orange, green = "#2563eb", "#e67700", "#15803d"

    #---------------------------------------------
    # Draw v1 from the shoulder to the knee.
    arrow(ax, origin, v1, blue)
    ax.plot(*v1, "o", color="#334155", markersize=5, zorder=4)
    ax.annotate("Knee", v1, xytext=(10, 6), textcoords="offset points", fontsize=10)

    # Label the thigh length and its direction from the -Z axis.
    length_label(ax, origin, v1, rf"$|\vec{{v}}_1| = {THIGH_LENGTH:g}$ m", blue, (26, 0))
    direction_angle(ax, origin, theta1, r"\theta_1", blue, (-32, 8))
    #---------------------------------------------

    #---------------------------------------------
    # Draw v2 from the knee to the foot, keeping its original direction.
    arrow(ax, v1, tip, orange)
    ax.annotate("Foot", tip, xytext=(7, -15), textcoords="offset points", fontsize=10)

    # Measure the shin direction from a -Z reference at the knee.
    length_label(ax, v1, tip, rf"$|\vec{{v}}_2| = {SHIN_LENGTH:g}$ m", orange, (26, -12))
    direction_angle(ax, v1, theta2, r"\theta_2", orange, (35, 0))
    #---------------------------------------------

    #---------------------------------------------
    # Draw the resultant directly from the shoulder to the foot.
    arrow(ax, origin, tip, green, dashed=True)

    # Unpack tip into (x, z) and calculate the length: sqrt(x**2 + z**2).
    resultant_length = math.hypot(*tip)

    # Treat lengths at or below 1e-12 m as zero to allow for floating-point error.
    if resultant_length > 1e-12:
        # Recover the angle from x = -L*sin(angle) and z = -L*cos(angle).
        # atan2(-x, -z) measures from -Z; degrees() converts radians to degrees.
        resultant_angle = math.degrees(math.atan2(-tip[0], -tip[1]))

        # Show length to four decimal places and angle to one on separate lines.
        resultant_label = (
            rf"$|\vec{{v}}_1+\vec{{v}}_2| = {resultant_length:.4f}$ m"
            "\n" + rf"$\theta_{{sum}} = {resultant_angle:.1f}^\circ$"
        )

        # Place the label 23 points left and 28 points below the vector midpoint.
        length_label(ax, origin, tip, resultant_label, green, (-23, -28))
    else:
        # A zero-length vector has no defined direction.
        # Axes coordinates place the message 4% from the left and 95% from the bottom.
        ax.text(0.04, 0.95, "Sum = 0 m; direction undefined",
                transform=ax.transAxes, va="top", color=green)


def main() -> None:
    '''Set joint angles, calculate link vectors, and display the three plots side by side.'''
    # Use (x, z) coordinates because this model moves only in the X-Z plane.
    # With the shoulder angle fixed at zero, there is no sideways motion,
    # so the Y coordinate stays zero and is omitted.
    # X: forward/backward foot movement relative to the shoulder.
    # Z: foot height relative to the shoulder (negative below the shoulder).

    # Set the thigh angle and the knee angle relative to the thigh (degrees).
    thigh_degrees = -25.0
    shin_degrees = 50.0

    # Calculate each link's direction relative to the -Z axis.
    theta1 = thigh_degrees
    theta2 = thigh_degrees + shin_degrees

    # Convert each link's length and direction to Cartesian (x, z) components.
    v1 = convert_polar_to_cartesian(THIGH_LENGTH, math.radians(theta1))
    v2 = convert_polar_to_cartesian(SHIN_LENGTH, math.radians(theta2))

    # Add the link vectors to get the foot position relative to the shoulder.
    tip = (v1[0] + v2[0], v1[1] + v2[1])

    # Arrange three plots side by side, leaving room below for equations.
    fig, axes = plt.subplots(1, 3, figsize=(18, 9))
    fig.subplots_adjust(left=0.06, right=0.97, top=0.9, bottom=0.34, wspace=0.35)

    # Use shared axis limits with enough padding for arrows and labels.
    points = ((0.0, 0.0), v1, v2, tip)
    x_limits = (min(p[0] for p in points) - 0.13,
                max(p[0] for p in points) + 0.17)
    z_limits = (min(p[1] for p in points) - 0.05,
                max(p[1] for p in points) + 0.055)

    origin = (0.0, 0.0)
    blue, orange = "#2563eb", "#e67700"

    #---------------------------------------------
    # First plot: thigh vector v1 from the origin.
    thigh_axis : Axes = axes[0]
    setup_axes(thigh_axis, x_limits, z_limits)
    thigh_axis.set_title(r"1. $\vec{v}_1$", fontsize=19, pad=16)

    # Draw the thigh vector and label its length and direction.
    plot_vector(thigh_axis, v1, blue)
    length_label(thigh_axis, origin, v1, rf"$|\vec{{v}}_1| = {THIGH_LENGTH:g}$ m",
                 blue, (25, -10))
    direction_angle(thigh_axis, origin, theta1, r"\theta_1", blue, (-32, 8))

    # The thigh direction is the thigh joint angle q1.
    add_formulas(thigh_axis, (
        rf"$\theta_1 = q_1 = {thigh_degrees:g}^\circ$",
        r"$\vec{v}_1 = (-L_1\sin\theta_1,\ -L_1\cos\theta_1)$",
        rf"$= ({v1[0]:+.5f},\ {v1[1]:+.5f})$ m",
        r"$\theta_1$: direction measured from $-Z$",
    ))
    #---------------------------------------------

    #---------------------------------------------
    # Second plot: shin vector v2 from the origin.
    shin_axis : Axes = axes[1]
    setup_axes(shin_axis, x_limits, z_limits)
    shin_axis.set_title(r"2. $\vec{v}_2$", fontsize=19, pad=16)

    # Draw the shin vector with labels placed on the opposite side.
    plot_vector(shin_axis, v2, orange)
    length_label(shin_axis, origin, v2, rf"$|\vec{{v}}_2| = {SHIN_LENGTH:g}$ m",
                 orange, (-25, -10))
    direction_angle(shin_axis, origin, theta2, r"\theta_2", orange, (32, 8))

    # The shin direction includes the knee angle q2 relative to the thigh.
    add_formulas(shin_axis, (
        rf"$\theta_2 = q_1 + q_2 = ({thigh_degrees:g}^\circ) + ({shin_degrees:g}^\circ)"
        rf" = {theta2:g}^\circ$",
        r"$\vec{v}_2 = (-L_2\sin\theta_2,\ -L_2\cos\theta_2)$",
        rf"$= ({v2[0]:+.5f},\ {v2[1]:+.5f})$ m",
        rf"$q_2 = {shin_degrees:g}^\circ$: relative knee angle",
    ))
    #---------------------------------------------

    #---------------------------------------------
    # Third plot: head-to-tail vector sum and foot position.
    sum_axis : Axes = axes[2]
    setup_axes(sum_axis, x_limits, z_limits)
    sum_axis.set_title(r"3. $\vec{v}_1 + \vec{v}_2$", fontsize=19, pad=16)

    # Translate v2 to the tip of v1 and draw the resultant vector.
    plot_vector_sum(sum_axis, v1, v2, theta1, theta2)

    # Add the vector components to explain the final foot position.
    add_formulas(sum_axis, (
        r"$\vec{v}_1+\vec{v}_2 = (v_{1x}+v_{2x},\ v_{1z}+v_{2z})$",
        rf"$= ({v1[0]:+.5f} {v2[0]:+.5f},$",
        rf"$\qquad {v1[1]:+.5f} {v2[1]:+.5f})$ m",
        rf"$= ({tip[0]:+.5f},\ {tip[1]:+.5f})$ m",
        r"$\vec{v}_2$ keeps its direction when translated.",
    ))
    #---------------------------------------------

    # Display the figure and release it after the window closes.
    plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
