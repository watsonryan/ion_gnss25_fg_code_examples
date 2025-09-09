import gtsam
import numpy as np
from graphviz import Source
from examples.helpers.earth_utils import Earth
import matplotlib.pyplot as plt
from pathlib import Path

# def visualize_factor_graph(graph, values, output_path):
#     dot_str = graph.dot(values)  # Pass the Values object
#     src = Source(dot_str)
#     # Save as PNG in the plots directory
#     src.render(output_path, format='png')

def visualize_factor_graph(graph, values, output_path, fmt="svg"):
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    dot_str = graph.dot(values)
    # Visualize using graphviz and save it to the plots directory
    src = Source(dot_str)
    src.render(output_path, format=fmt, cleanup=True)

def plot_trajectory_and_errors(epochs, true_positions, estimated_positions, output_path, reference_epoch=0):
    true_positions_eci = []
    estimated_positions_eci = []
    position_errors = []

    for epoch, true_pos, est_pos in zip(epochs, true_positions, estimated_positions):
        # Convert ECEF to ECI
        C = Earth.earth_to_inertial(epoch, reference_epoch)
        true_eci = np.dot(C, true_pos)
        est_eci = np.dot(C, est_pos)

        true_positions_eci.append(true_eci)
        estimated_positions_eci.append(est_eci)

        error = np.linalg.norm(true_eci - est_eci)
        position_errors.append(error)

    true_x_eci = [pos[0] for pos in true_positions_eci]
    true_y_eci = [pos[1] for pos in true_positions_eci]
    estimated_x_eci = [pos[0] for pos in estimated_positions_eci]
    estimated_y_eci = [pos[1] for pos in estimated_positions_eci]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    ax1.plot(true_x_eci, true_y_eci, 'b-o', label='True Trajectory (ECI)', markersize=3)
    ax1.plot(estimated_x_eci, estimated_y_eci, 'r--x', label='Estimated Trajectory (ECI)', markersize=3)
    ax1.set_xlabel('X Position (ECI, m)')
    ax1.set_ylabel('Y Position (ECI, m)')
    ax1.set_title('Satellite X-Y Trajectory in ECI Frame')
    ax1.legend()
    ax1.grid(True)
    ax1.set_aspect('equal', adjustable='box')

    # Plot the position errors over time
    ax2.plot(epochs, position_errors, 'g-o', markersize=3)
    ax2.set_xlabel('Time (seconds since start)')
    ax2.set_ylabel('Norm Position Error (meters)')
    ax2.set_title('Norm Position vs Time')
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()

def plot_position_error_cdf(
    init_position_errors,
    final_position_errors,
    plot_output_path
):
    """
    Plot the CDF of initial and final position errors.

    Parameters:
    init_position_errors (list): List of initial position errors.
    final_position_errors (list): List of final position errors.
    plot_output_path (str): Path to save the plot.
    """
    init_position_errors = np.sort(init_position_errors)
    final_position_errors = np.sort(final_position_errors)

    cdf_final_position = np.arange(1, len(final_position_errors) + 1) / len(final_position_errors)

    # Calculate medians
    median_final_position_error = np.median(final_position_errors)

    # Plot the CDFs for initial and final position errors
    plt.figure(figsize=(8, 6))
    plt.plot(final_position_errors, cdf_final_position, label="Final Position Error CDF", color='red')

    # Add median lines
    plt.axvline(median_final_position_error, color='red', linestyle='--')

    # Annotate median values
    plt.text(
        median_final_position_error, 0.5,
        f'Median Final: {median_final_position_error:.2f} m',
        color='red', verticalalignment='center', horizontalalignment='center',
        bbox=dict(boxstyle="round,pad=0.3", edgecolor="red", facecolor="white")
    )

    plt.xscale('log')
    plt.xlabel("Position Error (meters)")
    plt.ylabel("CDF")
    plt.title("CDF of Final Position Errors")
    plt.legend()

    # Save the plot
    plt.savefig(plot_output_path)
    plt.close()