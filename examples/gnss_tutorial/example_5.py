'''
This script estimates the position of a satellite over multiple epochs using pseudorange data
and constructs a single factor graph for all the epochs.

The link between states for different epochs is modeled using a BetweenFactor, which captures the relationship between 
successive states via a simulated propagation model.

We use pseudorange data (P1, P2 combined to form the IF combination) with light-time iteration accounted for. 
This data is used within a factor graph to estimate the simplified state space of the satellite at each epoch.
'''

import numpy as np
from functools import partial
import matplotlib.pyplot as plt
import gtsam
from gtsam import noiseModel, Point3

from examples.helpers.earth_utils import Earth
from examples.helpers.sp3 import SP3Reader
from examples.helpers.rinex import RinexReader
from examples.helpers.plotter import visualize_factor_graph, plot_trajectory_and_errors, plot_position_error_cdf
from examples.helpers.pseudorange_factor import pseudorange_error
from examples.helpers.observations import ionosphere_free_combination, elevation_dependent_scaling, simulate_pseudorange

# Constants
DT                 = 30
POS_NOISE_STDDEV   = 1.0e2
CLK_NOISE_STDDEV   = 1.0e4
RANGE_NOISE_STDDEV = 1.5
POS_BETWEEN_STDDEV = 0.25 
CLK_BETWEEN_STDDEV = 1.0e-3
USE_ROBUST         = True 
PERCENT_FAULTY     = 0.15



def propagate(sp3_data, prev_epoch, curr_epoch):
    ##
    ## Fake propagation
    ##
    ## In reality, this module would have your force model mechanization 
    ## ( SPH Grav + Tidal + 3rd Body + Drag + SRP + .. ) & a numerical integrator
    curr_data = sp3_data.get_satellite_data(curr_epoch)
    prev_data = sp3_data.get_satellite_data(prev_epoch)
    diff = curr_data['L06']['position']['position'] - prev_data['L06']['position']['position'] 
    diff += np.random.normal(0, POS_BETWEEN_STDDEV, 3)
    return diff

def build_and_optimize_isam(
    champ_sp3,  # SP3Reader for CHAMP satellite
    gnss_sp3,  # SP3Reader for GNSS satellites
    rnx_data,  # RINEX data reader
    position_noise_model,  # Noise model for receiver position
    clock_bias_noise_model,  # Noise model for receiver clock bias
    range_noise_model,  # Noise model for GNSS pseudorange data
    start_epoch,  # Start time (GPS time in seconds)
    stop_epoch,  # Stop time (GPS time in seconds)
    position_between_factor_noise_model,  # Noise model for position BetweenFactor
    clock_between_factor_noise_model,  # Noise model for clock BetweenFactor
    print_results=False  # Whether to print results for each epoch
):
    """
    Build a GNSS factor graph using incremental iSAM2 optimization.

    Parameters:
    Same as the original batch factor graph, except using iSAM2 for incremental optimization.

    Returns:
    isam (gtsam.ISAM2), final_result (gtsam.Values)
    """
    # Initialize iSAM2 and parameters
    isam_params = gtsam.ISAM2Params()
    # isam_params.relinearizeSkip = 1  # Re-linearize every iteration
    # isam_params.enableRelinearization = True  # Ensure re-linearization is enabled
    isam = gtsam.ISAM2(isam_params)

    # Initialize the factor graph and initial estimate
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # Get initial position
    init_pos_dict = champ_sp3.get_satellite_data(start_epoch)
    initial_pos_est = init_pos_dict["L06"]["position"]["position"] + np.random.normal(0, POS_NOISE_STDDEV, 3)

    prev_epoch = 0.0
    previous_pose_key = None
    previous_clock_bias_key = None
    initial_clk_est = 0.0
    true_receiver_clk_bias = np.abs(np.random.normal(0, 1e4))

    # Loop through epochs and incrementally update the factor graph using iSAM2
    for curr_epoch in range(start_epoch, stop_epoch + 1, DT): 
        # Get RINEX data for the current epoch
        rnx_epoch_data = rnx_data.get_epoch_data(curr_epoch)
        if not rnx_epoch_data:
            print(f"No RINEX data available for epoch {curr_epoch}")
            continue

        # Generate keys for this epoch
        receiver_pose_key = gtsam.symbol('x', curr_epoch)
        clock_bias_key = gtsam.symbol('b', curr_epoch)

        # Insert the initial estimates (use the last estimate from the iSAM result if available)
        if curr_epoch != start_epoch:
            initial_pos_est = result.atPoint3(previous_pose_key) + propagate(champ_sp3, prev_epoch, curr_epoch)
            initial_clk_est = result.atDouble(previous_clock_bias_key)

        initial_estimate.insert(receiver_pose_key, initial_pos_est)
        initial_estimate.insert(clock_bias_key, initial_clk_est)

        # Add prior factors for the first epoch
        if curr_epoch == start_epoch:
            prior_position_factor = gtsam.PriorFactorPoint3(
                receiver_pose_key, Point3(*initial_pos_est), position_noise_model
            )
            graph.add(prior_position_factor)

            prior_clock_bias_factor = gtsam.PriorFactorDouble(
                clock_bias_key, initial_clk_est, clock_bias_noise_model
            )
            graph.add(prior_clock_bias_factor)

        # Process each satellite in the RINEX observation data for this epoch
        sat_data = gnss_sp3.get_satellite_data(curr_epoch)
        sp3_dict = champ_sp3.get_satellite_data(curr_epoch)
        curr_true_position = sp3_dict["L06"]["position"]["position"]
        for prn, _ in rnx_epoch_data.items():
            prn_key = 'G' + prn.zfill(2)  # Ensure PRN is two digits
            try:
                gnss_sv_pos = np.array(sat_data[prn_key]['position']['position'])
            except:
                continue

            p1 = simulate_pseudorange(curr_true_position, gnss_sv_pos, true_receiver_clk_bias, RANGE_NOISE_STDDEV, PERCENT_FAULTY)
            p2 = simulate_pseudorange(curr_true_position, gnss_sv_pos, true_receiver_clk_bias, RANGE_NOISE_STDDEV, PERCENT_FAULTY)
            if p1 is not None and p2 is not None:
                # Compute the ionosphere-free combination
                if_value = ionosphere_free_combination(p1, p2)
                if sat_data is None or prn_key not in sat_data:
                    print(f"Satellite data not available for {prn_key}")
                    continue
                # Create the factor
                factor = gtsam.CustomFactor(
                    range_noise_model,
                    [receiver_pose_key, clock_bias_key],
                    partial(pseudorange_error, if_value, gnss_sv_pos)
                )
                graph.add(factor)

        # Add BetweenFactors to link consecutive states
        if previous_pose_key is not None and previous_clock_bias_key is not None:
            delta_state = propagate(champ_sp3, prev_epoch, curr_epoch)
            between_pos_factor = gtsam.BetweenFactorPoint3(
                previous_pose_key, receiver_pose_key, Point3(delta_state), position_between_factor_noise_model
            )
            graph.add(between_pos_factor)

            between_clock_bias_factor = gtsam.BetweenFactorDouble(
                previous_clock_bias_key, clock_bias_key, 0.0, clock_between_factor_noise_model
            )
            graph.add(between_clock_bias_factor)

        # Update previous keys for the next iteration
        previous_pose_key = receiver_pose_key
        previous_clock_bias_key = clock_bias_key

        #
        # Update ISAM -- I.e., add new factors, updating the solution and relinearizing as needed. 
        #
        # newFactors:	 The new factors to be added to the system
        # newInitValues: Initialization points for new variables to be added to the system. 
        #                You must include here all new variables occuring in newFactors 
        #                (which were not already in the system). There must not be any variables
        #                here that do not occur in newFactors, and additionally, variables that were
        #                already in the system must not be included here. 
        isam.update(graph, initial_estimate)
        result = isam.calculateEstimate()

        # Clear the graph for the next iteration, but don't clear the estimates (we reuse them)
        graph            = gtsam.NonlinearFactorGraph()
        initial_estimate = gtsam.Values()
        prev_epoch       = curr_epoch

    # Optimize after all epochs and return the final result
    final_result = isam.calculateEstimate()

    return isam, final_result

def run_simulations_for_multiple_epochs(
    champ_sp3_path,
    gnss_sp3_path,
    rinex_path,
    start_epoch,
    stop_epoch,
    plot_cdf_output_path=None,
    plot_trajectory_output_path=None,
    print_results=False
):
    """
    Run GNSS simulation for multiple epochs using iSAM2 for incremental optimization.

    Parameters:
    Same as the original.

    Returns:
    None
    """
    # Read the data files
    champ_sp3 = SP3Reader(champ_sp3_path)
    gnss_sp3 = SP3Reader(gnss_sp3_path)
    rnx_data = RinexReader(rinex_path)

    # Define noise models
    position_noise_model = gtsam.noiseModel.Isotropic.Sigma(3, POS_NOISE_STDDEV)
    clock_bias_noise_model = gtsam.noiseModel.Isotropic.Sigma(1, CLK_NOISE_STDDEV)
    range_noise_model = gtsam.noiseModel.Isotropic.Sigma(1, RANGE_NOISE_STDDEV)
    if (USE_ROBUST):
        robust_range_noise_model = gtsam.noiseModel.Robust.Create(
        gtsam.noiseModel.mEstimator.Huber(k=RANGE_NOISE_STDDEV),
        range_noise_model
        )
        range_noise_model = robust_range_noise_model
        
    position_between_factor_noise_model = gtsam.noiseModel.Isotropic.Sigma(3, POS_BETWEEN_STDDEV)
    clock_between_factor_noise_model = gtsam.noiseModel.Isotropic.Sigma(1, CLK_BETWEEN_STDDEV)

    # Build the factor graph incrementally with iSAM
    isam, result = build_and_optimize_isam(
        champ_sp3,
        gnss_sp3,
        rnx_data,
        position_noise_model,
        clock_bias_noise_model,
        range_noise_model,
        start_epoch,
        stop_epoch,
        position_between_factor_noise_model,
        clock_between_factor_noise_model,
        print_results
    )

    # Extract and print results
    epochs = []
    final_errors = []
    true_positions = []
    estimated_positions = []
    for curr_epoch in range(start_epoch, stop_epoch + 1, DT):
        epochs.append(curr_epoch)
        receiver_pose_key = gtsam.symbol('x', curr_epoch)
        clock_bias_key = gtsam.symbol('b', curr_epoch)

        if result.exists(receiver_pose_key):
            final_position = np.array(result.atPoint3(receiver_pose_key))
            estimated_positions.append(final_position)
            final_clock_bias = result.atDouble(clock_bias_key)

            # Get the true receiver position from the CHAMP SP3 file for this epoch
            champ_data = champ_sp3.get_satellite_data(curr_epoch)
            if champ_data is not None and 'L06' in champ_data:
                true_position = champ_data['L06']['position']['position']
                true_positions.append(true_position)

                # Calculate the position errors using CHAMP SP3 true position
                final_position_error = np.linalg.norm(true_position - final_position)
                final_errors.append(final_position_error)
                
                if print_results:
                    print(f"\nEpoch {curr_epoch} Results:")
                    print(f"True Position (SP3): X={true_position[0]:.4f}, Y={true_position[1]:.4f}, Z={true_position[2]:.4f}")
                    print(f"Final Receiver Position: X={final_position[0]:.4f}, Y={final_position[1]:.4f}, Z={final_position[2]:.4f}")
                    print(f"Final Clock Bias: {final_clock_bias:.4e} meters")
                    print(f"Final Position Error (meters): {final_position_error:.4f}")

    if plot_trajectory_output_path:
        plot_trajectory_and_errors(
            epochs,
            true_positions,
            estimated_positions,
            plot_trajectory_output_path
        )

    if plot_cdf_output_path:
        plot_position_error_cdf(final_errors, final_errors, plot_cdf_output_path) 

    return isam, result


if __name__ == "__main__":
    # Example usage
    factor_graph, initial_estimate = run_simulations_for_multiple_epochs(
        champ_sp3_path='data/champ.sp3',
        gnss_sp3_path='data/gnss.sp3',
        rinex_path='data/champ.rnx',
        start_epoch=946360815,  # Start epoch (GPS time in seconds)
        # stop_epoch=946360935,   # Stop epoch (GPS time in seconds)
        stop_epoch=946366215,   # Stop epoch (GPS time in seconds)
        plot_cdf_output_path='plots/ex_5_position_error_cdf.png',
        plot_trajectory_output_path='plots/ex_5_trajectory_and_errors.png',
        print_results=True
    )
