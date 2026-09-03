import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


FOOT_ELEVATION = 4.0
X_STRIDE = 7.0

def calculate_forward_gait(
    phase_degrees: np.ndarray,
    foot_elevation: float = FOOT_ELEVATION,
    x_stride: float = X_STRIDE,
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Calculate one cycle of Yertle's forward trot gait.

    The returned tuple for each foot is ``(x_offset, y_offset)`` and follows
    the same sign convention as ``YertlePid.gaitCalc`` in ``yertle_ui.py``.
    """

    phase = np.radians(phase_degrees)
    sin_phase = np.sin(phase)
    cos_phase = np.cos(phase)

    # The RF/LB pair is 180 degrees out of phase with the LF/RB pair.
    lf_rb_x = x_stride * cos_phase
    rf_lb_x = -x_stride * cos_phase

    # LF/RB lift during 0-180 degrees; RF/LB lift during 180-360 degrees.
    # Clamp values to the range [-foot_elevation, 0].
    lf_rb_y = np.clip(
        -foot_elevation * sin_phase, -foot_elevation, 0.0
    )
    rf_lb_y = np.clip(
        foot_elevation * sin_phase, -foot_elevation, 0.0
    )

    return {
        "LF": (lf_rb_x, lf_rb_y),
        "RF": (rf_lb_x, rf_lb_y),
        "LB": (rf_lb_x, rf_lb_y),
        "RB": (lf_rb_x, lf_rb_y),
    }


def animate_diagonal_groups(
    phase_degrees: np.ndarray,
    gait: dict[str, tuple[np.ndarray, np.ndarray]],
    foot_elevation: float,
    x_stride: float,
    colors: dict[str, str],
) -> FuncAnimation:
    """Animate representative feet from the two diagonal gait groups."""

    figure, axis = plt.subplots(figsize=(10, 5.5))
    lf_x, lf_y = gait["LF"]
    rf_x, rf_y = gait["RF"]
    lf_lift = -lf_y
    rf_lift = -rf_y

    # The geometric path is shared; the two groups enter it 180 degrees apart.
    axis.plot(lf_x, lf_lift, color="0.75", linewidth=2, label="Foot path")
    axis.axhline(0.0, color="black", linewidth=2, label="Ground")

    # These are schematic representative legs, not a linkage/IK model. Sharing
    # one reference hip makes the phase crossing especially easy to compare.
    hip_x = 0.0
    hip_y = 1.2 * foot_elevation
    axis.plot(hip_x, hip_y, "ks", markersize=7, label="Reference hip")
    lf_leg, = axis.plot([], [], color=colors["LF + RB"], linewidth=3, alpha=0.75)
    rf_leg, = axis.plot([], [], color=colors["RF + LB"], linewidth=3, alpha=0.75)

    lf_point, = axis.plot(
        [], [], "o", color=colors["LF + RB"], markersize=12, label="LF + RB"
    )
    rf_point, = axis.plot(
        [], [], "o", color=colors["RF + LB"], markersize=12, label="RF + LB"
    )
    lf_trail, = axis.plot([], [], color=colors["LF + RB"], linewidth=2, alpha=0.45)
    rf_trail, = axis.plot([], [], color=colors["RF + LB"], linewidth=2, alpha=0.45)

    phase_text = axis.text(
        0.02,
        0.95,
        "",
        transform=axis.transAxes,
        va="top",
        fontsize=11,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )
    state_text = axis.text(
        0.5,
        0.95,
        "",
        transform=axis.transAxes,
        ha="center",
        va="top",
        fontsize=11,
    )

    axis.set_title("Diagonal gait animation (representative feet)")
    axis.set_xlabel("X offset (forward/backward)")
    axis.set_ylabel("Lift height (-Y offset)")
    axis.set_xlim(-1.25 * abs(x_stride), 1.25 * abs(x_stride))
    axis.set_ylim(-0.2 * foot_elevation, 1.3 * foot_elevation)
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper right")
    axis.set_aspect("equal", adjustable="box")

    # A short trail makes the local direction visible without covering the
    # entire shared path. Frames advance by two degrees for a smooth loop.
    frame_indices = np.arange(0, len(phase_degrees), 4)
    trail_length = 35

    def update(frame_index):
        lf_leg.set_data([hip_x, lf_x[frame_index]], [hip_y, lf_lift[frame_index]])
        rf_leg.set_data([hip_x, rf_x[frame_index]], [hip_y, rf_lift[frame_index]])
        lf_point.set_data([lf_x[frame_index]], [lf_lift[frame_index]])
        rf_point.set_data([rf_x[frame_index]], [rf_lift[frame_index]])

        trail_start = max(0, frame_index - trail_length)
        lf_trail.set_data(
            lf_x[trail_start : frame_index + 1],
            lf_lift[trail_start : frame_index + 1],
        )
        rf_trail.set_data(
            rf_x[trail_start : frame_index + 1],
            rf_lift[trail_start : frame_index + 1],
        )

        phase = phase_degrees[frame_index]
        phase_text.set_text(f"Phase: {phase:5.1f}°")
        if lf_lift[frame_index] > 1e-6:
            state_text.set_text("LF + RB: swing     RF + LB: stance")
        elif rf_lift[frame_index] > 1e-6:
            state_text.set_text("LF + RB: stance     RF + LB: swing")
        else:
            state_text.set_text("Gait transition")

        return (
            lf_leg,
            rf_leg,
            lf_point,
            rf_point,
            lf_trail,
            rf_trail,
            phase_text,
            state_text,
        )

    return FuncAnimation(
        figure,
        update,
        frames=frame_indices,
        interval=30,
        blit=True,
        repeat=True,
    )

if __name__ == "__main__":
    # Phase angles from 0° to 360° in 0.5° increments
    phase_degrees = np.linspace(0.0, 360.0, 721)

    gait = calculate_forward_gait(
        phase_degrees,
        foot_elevation=FOOT_ELEVATION,
        x_stride=X_STRIDE,
    )

    legs_groups = {
        "LF + RB": gait["LF"],
        "RF + LB": gait["RF"],
    }

    # Define the line color for each diagonal leg pair.
    colors = {"LF + RB": "tab:blue", "RF + LB": "tab:orange"}

    # Create a figure containing four plots arranged in a 2×2 grid.
    figure, axes = plt.subplots(2, 2, figsize=(13, 9))

    #---------------------------------------------
    # First plot: X-axis movement of the LF/RB and RF/LB diagonal leg pairs
    legs_x_axis = axes[0][0]

    # Plot the X offset of each diagonal leg pair over the gait cycle.
    for leg_name, (x_offset, _) in legs_groups.items():
        legs_x_axis.plot(phase_degrees, x_offset, color=colors[leg_name], label=leg_name)

    # Set the plot title and Y-axis label.
    legs_x_axis.set_title("Forward/backward foot offset")
    legs_x_axis.set_ylabel("X offset")

    # Configure the X-axis, grid, and legend.
    legs_x_axis.set_xlabel("Gait phase (degrees)")   # Set the x-axis label.
    legs_x_axis.set_xlim(0, 360)                     # Display one complete gait cycle.
    legs_x_axis.set_xticks(np.arange(0, 361, 90))    # Place ticks at 90-degree intervals.
    legs_x_axis.grid(True, alpha=0.3)                # Show a lightly transparent grid.
    legs_x_axis.legend(ncols=2)                      # Arrange legend entries in two columns.
    #---------------------------------------------

    #---------------------------------------------
    # Second plot: Y-axis movement of the LF/RB and RF/LB diagonal leg pairs
    legs_y_axis = axes[0][1]

    # Plot the X offset of each diagonal leg pair over the gait cycle.
    for leg_name, (_, y_offset) in legs_groups.items():
        legs_y_axis.plot(phase_degrees, -y_offset, color=colors[leg_name], label=leg_name)

    # Set the plot title and Y-axis label.
    legs_y_axis.set_title("Foot lift")
    legs_y_axis.set_ylabel("Lift height (-Y offset)")

    # Configure the X-axis, grid, and legend.
    legs_y_axis.set_xlabel("Gait phase (degrees)")   # Set the x-axis label.
    legs_y_axis.set_xlim(0, 360)                     # Display one complete gait cycle.
    legs_y_axis.set_xticks(np.arange(0, 361, 90))    # Place ticks at 90-degree intervals.
    legs_y_axis.grid(True, alpha=0.3)                # Show a lightly transparent grid.
    legs_y_axis.legend(ncols=2)                      # Arrange legend entries in two columns.
    #---------------------------------------------


    #---------------------------------------------
    # Third plot: foot-tip trajectory of the LF/RB diagonal leg pair.
    lf_rb_trajectory_axis = axes[1][0]

    # Retrieve the horizontal and vertical offsets for this synchronized pair.
    x_offset, y_offset = legs_groups["LF + RB"]

    # Plot lift height against the forward/backward foot displacement.
    lf_rb_trajectory_axis.plot(x_offset, -y_offset, color=colors["LF + RB"], linewidth=2.5)
    # Mark the first sample so the beginning of the gait cycle is easy to identify.
    lf_rb_trajectory_axis.scatter(
        [x_offset[0]],
        [-y_offset[0]],
        s=100,
        color=colors["LF + RB"],
        edgecolor="black",
        linewidth=1.0,
        zorder=4,
        label="Start (0°)",
    )
    # Place a short text label beside the start marker.
    lf_rb_trajectory_axis.annotate(
        "start",
        xy=(x_offset[0], -y_offset[0]),
        xytext=(8, 12),
        textcoords="offset points",
        ha="left",
        fontsize=9,
    )

    # Each arrow connects nearby samples on the actual path, so its angle
    # is tangent to the trajectory and follows increasing gait phase.
    for start_phase, end_phase in ((45, 70), (225, 250)):
        # The trajectory contains two samples per degree of gait phase.
        start_index = int(start_phase * 2)
        end_index = int(end_phase * 2)

        # Draw an arrow from the earlier sample to the later sample.
        lf_rb_trajectory_axis.annotate(
            "",
            xy=(x_offset[end_index], -y_offset[end_index]),
            xytext=(x_offset[start_index], -y_offset[start_index]),
            arrowprops={
                "arrowstyle": "-|>",
                "color": "black",
                "lw": 1.8,
                "mutation_scale": 14,
                "shrinkA": 0,
                "shrinkB": 0,
            },
            zorder=5,
        )

    # Label the subplot and add a zero-height reference line.
    lf_rb_trajectory_axis.set_title("LF + RB foot-tip trajectory")
    lf_rb_trajectory_axis.set_xlabel("X offset (forward/backward)")
    lf_rb_trajectory_axis.set_ylabel("Lift height (-Y offset)")
    lf_rb_trajectory_axis.axhline(0.0, color="black", linewidth=1, alpha=0.5)
    # Add 20% padding around the expected stride and lift ranges.
    lf_rb_trajectory_axis.set_xlim(-1.2 * abs(X_STRIDE), 1.2 * abs(X_STRIDE))
    lf_rb_trajectory_axis.set_ylim(-0.2 * FOOT_ELEVATION, 1.2 * FOOT_ELEVATION)
    lf_rb_trajectory_axis.grid(True, alpha=0.3)
    lf_rb_trajectory_axis.legend(loc="upper right")
    # Use equal scaling so the plotted trajectory preserves its geometry.
    lf_rb_trajectory_axis.set_aspect("equal", adjustable="box")

    #---------------------------------------------

    #---------------------------------------------
    # Fourth plot: foot-tip trajectory of the RF/LB diagonal leg pair.
    rf_lb_trajectory_axis = axes[1][1]

    # Retrieve the horizontal and vertical offsets for this synchronized pair.
    x_offset, y_offset = legs_groups["RF + LB"]

    # Plot lift height against the forward/backward foot displacement.
    rf_lb_trajectory_axis.plot(x_offset, -y_offset, color=colors["RF + LB"], linewidth=2.5)
    # Mark the first sample so the beginning of the gait cycle is easy to identify.
    rf_lb_trajectory_axis.scatter(
        [x_offset[0]],
        [-y_offset[0]],
        s=100,
        color=colors["RF + LB"],
        edgecolor="black",
        linewidth=1.0,
        zorder=4,
        label="Start (0°)",
    )
    # Place a short text label beside the start marker.
    rf_lb_trajectory_axis.annotate(
        "start",
        xy=(x_offset[0], -y_offset[0]),
        xytext=(8, 12),
        textcoords="offset points",
        ha="left",
        fontsize=9,
    )

    # Each arrow connects nearby samples on the actual path, so its angle
    # is tangent to the trajectory and follows increasing gait phase.
    for start_phase, end_phase in ((45, 70), (225, 250)):
        # The trajectory contains two samples per degree of gait phase.
        start_index = int(start_phase * 2)
        end_index = int(end_phase * 2)

        # Draw an arrow from the earlier sample to the later sample.
        rf_lb_trajectory_axis.annotate(
            "",
            xy=(x_offset[end_index], -y_offset[end_index]),
            xytext=(x_offset[start_index], -y_offset[start_index]),
            arrowprops={
                "arrowstyle": "-|>",
                "color": "black",
                "lw": 1.8,
                "mutation_scale": 14,
                "shrinkA": 0,
                "shrinkB": 0,
            },
            zorder=5,
        )

    # Label the subplot and add a zero-height reference line.
    rf_lb_trajectory_axis.set_title("RF + LB foot-tip trajectory")
    rf_lb_trajectory_axis.set_xlabel("X offset (forward/backward)")
    rf_lb_trajectory_axis.set_ylabel("Lift height (-Y offset)")
    rf_lb_trajectory_axis.axhline(0.0, color="black", linewidth=1, alpha=0.5)
    # Add 20% padding around the expected stride and lift ranges.
    rf_lb_trajectory_axis.set_xlim(-1.2 * abs(X_STRIDE), 1.2 * abs(X_STRIDE))
    rf_lb_trajectory_axis.set_ylim(-0.2 * FOOT_ELEVATION, 1.2 * FOOT_ELEVATION)
    rf_lb_trajectory_axis.grid(True, alpha=0.3)
    rf_lb_trajectory_axis.legend(loc="upper right")
    # Use equal scaling so the plotted trajectory preserves its geometry.
    rf_lb_trajectory_axis.set_aspect("equal", adjustable="box")

    #---------------------------------------------


    figure.suptitle(
        f"Yertle forward gait (X stride={X_STRIDE:g}, foot elevation={FOOT_ELEVATION:g})"
    )
    figure.tight_layout()


    # Keep the animation object alive until plt.show() closes. Without this
    # reference Matplotlib may garbage-collect it before drawing any frames.
    gait_animation = animate_diagonal_groups(
        phase_degrees,
        gait,
        foot_elevation=FOOT_ELEVATION,
        x_stride=X_STRIDE,
        colors=colors,
    )
    plt.show()

    plt.show(block=True)
