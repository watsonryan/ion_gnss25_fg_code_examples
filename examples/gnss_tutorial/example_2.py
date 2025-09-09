import numpy as np
import gtsam
import matplotlib.pyplot as plt
from examples.helpers.sp3 import SP3Reader
from examples.helpers.rinex import RinexReader
from examples.helpers.plotter import plot_trajectory_and_errors, plot_position_error_cdf, visualize_factor_graph
from examples.helpers.pseudorange_factor import pseudorange_error
from examples.helpers.observations import ionosphere_free_combination, simulate_pseudorange
from functools import partial

# Constants
DT = 30
POS_NOISE_STDDEV = 1.0e2
CLK_NOISE_STDDEV = 1.0e4
RANGE_NOISE_STDDEV = 1.5
POS_BETWEEN_STDDEV = 5.0e5
CLK_BETWEEN_STDDEV = 1.0e-3

def build_and_optimize_factor_graph(
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
    print_results=False
):
    """
    Build a GNSS factor graph for multiple epochs with BetweenFactors.
    """
    first_iteration = True
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    init_pos_dict = champ_sp3.get_satellite_data(start_epoch)
    initial_pos_est = init_pos_dict["L06"]["position"]["position"] + np.random.normal(0, POS_NOISE_STDDEV, 3)

    previous_pose_key = None
    previous_clock_bias_key = None
    initial_clk_est = 0.0
    true_receiver_clk_bias = np.abs(np.random.normal(0, CLK_NOISE_STDDEV))

    
    #
    # Iterate over the requested epoch window
    #
    for epoch_time in range(start_epoch, stop_epoch + 1, DT): 
        rnx_epoch_data = rnx_data.get_epoch_data(epoch_time)
        if not rnx_epoch_data:
            print(f"No RINEX data available for epoch {epoch_time}")
            continue

        #
        # Initialize the states.
        #
        receiver_pose_key = gtsam.symbol('x', epoch_time)
        clock_bias_key = gtsam.symbol('b', epoch_time)
        initial_estimate.insert(receiver_pose_key, gtsam.Point3(*initial_pos_est))
        initial_estimate.insert(clock_bias_key, initial_clk_est)

        #
        # Add a prior constraint to the position and clock states on the first epoch only 
        #
        if first_iteration:
            prior_position_factor = gtsam.PriorFactorPoint3(
                receiver_pose_key, gtsam.Point3(*initial_pos_est), position_noise_model
            )
            graph.add(prior_position_factor)

            prior_clock_bias_factor = gtsam.PriorFactorDouble(
                clock_bias_key, initial_clk_est, clock_bias_noise_model
            )
            graph.add(prior_clock_bias_factor)
            first_iteration = False

        sat_data = gnss_sp3.get_satellite_data(epoch_time)
        sp3_dict = champ_sp3.get_satellite_data(epoch_time)
        curr_true_position = sp3_dict["L06"]["position"]["position"]
        
        #
        # Iterate over each satellite in the RINEX observation data and add a constraint to the graph
        #
        for prn, _ in rnx_epoch_data.items():
            prn_key = 'G' + prn.zfill(2)
            try:
                gnss_sv_pos = np.array(sat_data[prn_key]['position']['position'])
            except:
                continue
            p1 = simulate_pseudorange(curr_true_position, gnss_sv_pos, true_receiver_clk_bias, RANGE_NOISE_STDDEV, 0.0)
            p2 = simulate_pseudorange(curr_true_position, gnss_sv_pos, true_receiver_clk_bias, RANGE_NOISE_STDDEV, 0.0)
            if p1 is not None and p2 is not None:
                if_value = ionosphere_free_combination(p1, p2)
                if sat_data is None or prn_key not in sat_data:
                    print(f"Satellite data not available for {prn_key}")
                    continue
                factor = gtsam.CustomFactor(
                    range_noise_model,
                    [receiver_pose_key, clock_bias_key],
                    partial(pseudorange_error, if_value, gnss_sv_pos)
                )
                graph.add(factor)

        #
        # Add between factor to capture temporal relationship within the graph.
        # In this case, we've implemented a white-noise update ( i.e., we only have approximate delta between consecutive epochs )
        #
        if previous_pose_key is not None and previous_clock_bias_key is not None:
            between_pos_factor = gtsam.BetweenFactorPoint3(
                previous_pose_key, receiver_pose_key, gtsam.Point3(0, 0, 0), position_between_factor_noise_model
            )
            graph.add(between_pos_factor)

            between_clock_bias_factor = gtsam.BetweenFactorDouble(
                previous_clock_bias_key, clock_bias_key, 0.0, clock_between_factor_noise_model
            )
            graph.add(between_clock_bias_factor)

        previous_pose_key = receiver_pose_key
        previous_clock_bias_key = clock_bias_key

    #
    # -------------------------------------------------
    # Now, let's solve the graph 
    # -------------------------------------------------
    #
    params = gtsam.LevenbergMarquardtParams()
    params.setAbsoluteErrorTol(1e-9)
    params.setRelativeErrorTol(1e-9)
    params.setMaxIterations(100)
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
    result = optimizer.optimize()

    return graph, initial_estimate, result

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
    Run GNSS simulation for multiple epochs between the start and stop time.
    """
    champ_sp3 = SP3Reader(champ_sp3_path)
    gnss_sp3 = SP3Reader(gnss_sp3_path)
    rnx_data = RinexReader(rinex_path)

    position_noise_model = gtsam.noiseModel.Isotropic.Sigma(3, POS_NOISE_STDDEV)
    clock_bias_noise_model = gtsam.noiseModel.Isotropic.Sigma(1, CLK_NOISE_STDDEV)
    range_noise_model = gtsam.noiseModel.Isotropic.Sigma(1, RANGE_NOISE_STDDEV)
    position_between_factor_noise_model = gtsam.noiseModel.Isotropic.Sigma(3, POS_BETWEEN_STDDEV)
    clock_between_factor_noise_model = gtsam.noiseModel.Isotropic.Sigma(1, CLK_BETWEEN_STDDEV)

    graph, initial_estimate, result = build_and_optimize_factor_graph(
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

    true_positions = []
    estimated_positions = []
    position_errors = []
    epochs = []

    for epoch_time in range(start_epoch, stop_epoch + 1, DT):
        receiver_pose_key = gtsam.symbol('x', epoch_time)
        if result.exists(receiver_pose_key):
            final_position = np.array(result.atPoint3(receiver_pose_key))

            champ_data = champ_sp3.get_satellite_data(epoch_time)
            if champ_data is not None and 'L06' in champ_data:
                true_position = champ_data['L06']['position']['position']
                position_error = np.linalg.norm(true_position - final_position)
                position_errors.append(position_error)
                true_positions.append(true_position)
                estimated_positions.append(final_position)
                epochs.append(epoch_time)

    if plot_trajectory_output_path:
        plot_trajectory_and_errors(
            epochs,
            true_positions,
            estimated_positions,
            plot_trajectory_output_path
        )

    if plot_cdf_output_path:
        plot_position_error_cdf([], position_errors, plot_cdf_output_path)

    return graph, initial_estimate

if __name__ == "__main__":
    factor_graph, initial_estimate = run_simulations_for_multiple_epochs(
        champ_sp3_path='data/champ.sp3',
        gnss_sp3_path='data/gnss.sp3',
        rinex_path='data/champ.rnx',
        start_epoch=946360815,  # Start epoch (GPS time in seconds)
        stop_epoch=946366215,   # Stop epoch (GPS time in seconds)
        # stop_epoch=946360915,   # Stop epoch (GPS time in seconds)
        plot_cdf_output_path='plots/ex_2_position_error_cdf.png',
        plot_trajectory_output_path='plots/ex_2_trajectory_and_errors.png',
        print_results=True
    )
    visualize_factor_graph(factor_graph, initial_estimate, "plots/ex_2_factor_graph")
