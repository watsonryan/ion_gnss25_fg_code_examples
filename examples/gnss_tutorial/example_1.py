'''
As a first step, let's try to estimate the position of a satellite at a single
epoch given a noisy initial guess of the state + pseudorange data.

To do this, we generally conduct the following steps: 
-----------------------------------------------------
    1. Initialization of the hidden variables
    2. Construction of the factor graph
    3. “solve” the graph ( using one of the built in optimizers )
    4. Pulling values out of the graph

For this example, the data comes from the CHAMP (CHAllenging Minisatellite Payload)
satellite on 2010-01-01T04:00:00Z

Data reference: https://isdc.gfz-potsdam.de/champ-isdc/access-to-the-champ-data/

'''

import numpy as np
from functools import partial
import matplotlib.pyplot as plt
import gtsam
from gtsam import noiseModel, Point3

from examples.helpers.earth_utils import Earth
from examples.helpers.sp3 import SP3Reader
from examples.helpers.rinex import RinexReader
from examples.helpers.plotter import visualize_factor_graph
from examples.helpers.pseudorange_factor import pseudorange_error
from examples.helpers.observations import ionosphere_free_combination, elevation_dependent_scaling, simulate_pseudorange

# Constants
POS_NOISE_STDDEV = 1.0e2
CLOCK_NOISE_STDDEV = 1e4 
RANGE_NOISE_STDDEV = 1.5

def build_and_optimize_factor_graph(
    true_receiver_position,
    true_receiver_clk_bias,
    gnss_sp3,
    rnx_data,
    position_noise_model,
    clock_bias_noise_model,
    range_noise_model,
    start_epoch,
    print_results=False 
):
    """
    Run a single GNSS simulation.

    Parameters:
    true_receiver_position (np.array): True receiver position.
    gnss_sp3 (SP3Reader): SP3Reader object for GNSS satellites.
    rnx_data (RinexReader): RINEX reader object.
    position_noise_model: Noise model for the receiver position.
    clock_bias_noise_model: Noise model for the clock bias.
    range_noise_model (noiseModel): Noise model for the range data.
    start_epoch (int): GPS time in seconds.
    print_results (bool): Whether to print the sim resutls to stdout.

    Returns:
    tuple: (init_position_error, final_position_error, factor_graph, initial_estimate)
    """
    # Add random noise to the true position to get initial estimate
    noisy_initial_position = true_receiver_position + np.random.normal(0, POS_NOISE_STDDEV, 3)
    initial_clock_bias = 0.0  # Can add noise if desired

    ##
    #
    # ---------------------------------------
    # Step 1) Initialize the hidden variables
    # ---------------------------------------
    #
    # Some notes on hidden varaibles in GTSAM:
    #       * Implicitly declared
    #       * Referenced by a Key
    #       * Often created using a “symbol” command combining a character and number
    #       * They're set and retrieve using the same keys 
    #
    ##
    # Based on our assumed state space, let's create the state variables: receiver position and clock bias
    #
    initial_estimate = gtsam.Values()
    receiver_pose_key = gtsam.symbol('x', start_epoch)
    clock_bias_key = gtsam.symbol('b', start_epoch)
    #
    # Insert initial estimate
    #
    initial_estimate.insert(receiver_pose_key, Point3(*noisy_initial_position))
    initial_estimate.insert(clock_bias_key, initial_clock_bias)

    ##
    #
    # --------------------------------------------------------------
    # Step 2) Initialize the factor graph and add all the factors/constraints
    # --------------------------------------------------------------
    #
    ##

    # 
    # the factor graph is the instantiation in code of the joint probability distribution 𝑃(𝑋|𝑍) (posterior density)
    # over the entire time series of 𝑋={𝑥1,𝑥2,𝑥3} of the platform.
    #
    # This in conjunction with our initial_estimates allows us to calcualte the maximum a-posteriori (MAP) estimate  
    #
    graph = gtsam.NonlinearFactorGraph()
    
    #
    # A factor consists of three main items:
    #   1. An observation ( constraint ) 
    #   2. A observation function that relates hidden variables to the observation
    #   3. A PDF that describes the distribution of the observation -- p(z|x)
    # 
    # See pseudorange_factor.py for a concrete example.
    #
    #
    # Given, this, what factors ( constaints ) can we add on our state space: 
    #   1. prior constraints on the states ( e.g., we know the position given some coarse approximation )
    #   2. GNSS observation constraints ( in this case -- P1/P2 (in IF form) )
    #
    
    #
    # Add prior factors for the receiver position and clock bias
    #
    prior_position_factor = gtsam.PriorFactorPoint3(
        receiver_pose_key, Point3(*noisy_initial_position), position_noise_model
    )
    graph.add(prior_position_factor)

    prior_clock_bias_factor = gtsam.PriorFactorDouble(
        clock_bias_key, initial_clock_bias, clock_bias_noise_model
    )
    graph.add(prior_clock_bias_factor)

    # Store mapping from factor index to PRN for residuals
    factor_indices = {}

    #
    # Iterate over each satellite in the RINEX observation data and add a constraint to the graph
    #
    rnx_epoch_data = rnx_data.get_epoch_data(start_epoch)
    factor_index = 2  # Starting index after the two prior factors
    sat_data = gnss_sp3.get_satellite_data(start_epoch)
    for prn, observations in rnx_epoch_data.items():
        prn_key = 'G' + prn.zfill(2)  # Ensure PRN is two digits
        gnss_sv_pos = np.array(sat_data[prn_key]['position']['position'])
        p1 = simulate_pseudorange(true_receiver_position, gnss_sv_pos, true_receiver_clk_bias, 0.75, 0.0)
        p2 = simulate_pseudorange(true_receiver_position, gnss_sv_pos, true_receiver_clk_bias, 0.75, 0.0)
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
            # Store the factor index and PRN for residuals
            factor_indices[factor_index] = prn_key  # Use prn_key for consistency (e.g., 'G01')
            factor_index += 1
        else:
            print(f"PRN {prn} missing P1 or P2 data.")

    ##
    #
    # -------------------------------------------------
    # Step 3) solve the graph
    # -------------------------------------------------
    #
    # GTSAM has several optimizers and options for each optimizer. Choose
    # your optimizer, its parameters, and the intial values for the hidden variables and
    # run the optimizer.
    #
    # In this example, we'll use the Levenberg-Marquardt optimizer ( with a few specific parameters )
    #
    ##
    params = gtsam.LevenbergMarquardtParams()
    params.setAbsoluteErrorTol(1e-8)
    params.setRelativeErrorTol(1e-8)
    params.setMaxIterations(100)
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
    
    
    ##
    #
    # ------------------------------------------------------------------------------
    # Step 4) pull the values of interest out of the factor graph 
    # ------------------------------------------------------------------------------
    #
    ##
    result = optimizer.optimize()
    init_position_error = np.linalg.norm(true_receiver_position - noisy_initial_position)
    #
    # Hidden variables are referenced by “keys”
    #
    final_position       = np.array(result.atPoint3(receiver_pose_key))
    final_position_error = np.linalg.norm(true_receiver_position - final_position)
    final_clock_bias     = result.atDouble(clock_bias_key)
    if print_results:
        # Calculate the position errors
        print("\nInitial Receiver Position (ECEF in meters):")
        print(f"X: {noisy_initial_position[0]:.4f}, Y: {noisy_initial_position[1]:.4f}, Z: {noisy_initial_position[2]:.4f}")
        print(f"Initial Clock Bias (meters): {initial_clock_bias:.4e}")
        print(f"Initial Position Error (meters): {init_position_error:.4f}")

        print("\nFinal Receiver Position (ECEF in meters):")
        print(f"X: {final_position[0]:.4f}, Y: {final_position[1]:.4f}, Z: {final_position[2]:.4f}")
        print(f"Final Clock Bias (meters): {final_clock_bias:.4e}")
        print(f"Final Position Error (meters): {final_position_error:.4f}")

        ##
        # Compute marginal covariances
        #
        # The factor graph encodes the posterior density 𝑃(X|𝑍). Thus, we can pull out the covariance
        # Σ for each state estimate 𝑥 in X.
        #
        # Note:: even if your graph is non-linear, this is only an approximation to the true covariance
        # because GTSAM only computes a Gaussian approximation
        ##
        marginals = gtsam.Marginals(graph, result)

        # Get the covariance matrix for the receiver position
        position_covariance = marginals.marginalCovariance(receiver_pose_key)
        # Get the standard deviations (square roots of diagonal elements)
        position_std_dev = np.sqrt(np.diag(position_covariance))

        print("\nReceiver Position Covariance Matrix (in meters^2):")
        print(position_covariance)
        print("\nReceiver Position Standard Deviations (in meters):")
        print(f"X: {position_std_dev[0]:.4f}, Y: {position_std_dev[1]:.4f}, Z: {position_std_dev[2]:.4f}")

        # Get the covariance for the clock bias
        clock_bias_covariance = marginals.marginalCovariance(clock_bias_key)
        # Extract the scalar variance value
        clock_bias_variance = clock_bias_covariance[0, 0]
        # Compute the standard deviation
        clock_bias_std_dev = np.sqrt(clock_bias_variance)

        print("\nReceiver Clock Bias Covariance (in meters^2):")
        print(clock_bias_covariance)
        print("\nReceiver Clock Bias Standard Deviation (in meters):")
        print(f"{clock_bias_std_dev:.4e}")

        # Compute and print residuals
        total_error = 0.0
        print("\nResiduals for each factor:")
        for i in range(graph.size()):
            factor = graph.at(i)
            error = factor.error(result)
            total_error += error
            if i == 0:
                factor_type = "Prior Position Factor"
            elif i == 1:
                factor_type = "Prior Clock Bias Factor"
            else:
                prn = factor_indices.get(i, "Unknown")
                factor_type = f"Pseudorange Factor (PRN {prn})"
            print(f"Factor {i}: {factor_type}, Residual Error: {error:.6f}")

    return init_position_error, final_position_error, graph, initial_estimate

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

    cdf_init_position = np.arange(1, len(init_position_errors) + 1) / len(init_position_errors)
    cdf_final_position = np.arange(1, len(final_position_errors) + 1) / len(final_position_errors)

    # Calculate medians
    median_init_position_error = np.median(init_position_errors)
    median_final_position_error = np.median(final_position_errors)

    # Plot the CDFs for initial and final position errors
    plt.figure(figsize=(8, 6))
    plt.plot(init_position_errors, cdf_init_position, label="Initial Position Error CDF", color='blue')
    plt.plot(final_position_errors, cdf_final_position, label="Final Position Error CDF", color='red')

    # Add median lines
    plt.axvline(median_init_position_error, color='blue', linestyle='--')
    plt.axvline(median_final_position_error, color='red', linestyle='--')

    y_median_init = np.interp(median_init_position_error, init_position_errors, cdf_init_position)
    y_median_final = np.interp(median_final_position_error, final_position_errors, cdf_final_position)

    # Place text boxes directly at the intersection points
    plt.text(
        median_init_position_error, y_median_init,
        f'Median Initial: {median_init_position_error:.2f} m',
        color='blue', verticalalignment='center', horizontalalignment='center',
        bbox=dict(boxstyle="round,pad=0.3", edgecolor="blue", facecolor="white")
    )

    plt.text(
        median_final_position_error, y_median_final,
        f'Median Final: {median_final_position_error:.2f} m',
        color='red', verticalalignment='center', horizontalalignment='center',
        bbox=dict(boxstyle="round,pad=0.3", edgecolor="red", facecolor="white")
    )

    plt.xscale('log')
    plt.xlabel("Position Error (meters)")
    plt.ylabel("CDF")
    plt.title("CDF of Position Errors (Initial vs Final)")
    plt.legend()

    plt.savefig(plot_output_path)
    plt.close()  # Close the figure to free up memory

def run_simulations(
    n_simulations=1,
    plot_cdf=True,
    champ_sp3_path='data/champ.sp3',
    gnss_sp3_path='data/gnss.sp3',
    rinex_path='data/champ.rnx',
    start_epoch=946360815,
    plot_output_path='plots/ex_1_position_error_cdf.png',
    factor_graph_output_path='plots/ex_1_factor_graph',
    print_results=False 
):
    """
    Run GNSS simulation and optionally plot CDF of position errors.

    Parameters:
    n_simulations (int): Number of simulations to run.
    plot_cdf (bool): Whether to plot the CDF of position errors.
    champ_sp3_path (str): Path to the SP3 file for the receiver.
    gnss_sp3_path (str): Path to the SP3 file for GNSS satellites.
    rinex_path (str): Path to the RINEX observation file.
    start_epoch (int): Epoch time to start the simulation.
    plot_output_path (str): File path to save the position error CDF plot.
    factor_graph_output_path (str): File path to save the factor graph visualization.
    print_results (bool): Whether to print sims results to stdout .

    Returns:
    tuple: (factor_graph, last_initial_estimate)
    """
    # Read the data files
    champ_sp3 = SP3Reader(champ_sp3_path)
    gnss_sp3 = SP3Reader(gnss_sp3_path)
    rnx_data = RinexReader(rinex_path)

    # Get data for the start epoch
    champ_data = champ_sp3.get_satellite_data(start_epoch)
    if champ_data is None or 'L06' not in champ_data:
        raise ValueError("CHAMP satellite data not available for the given epoch.")
    true_receiver_position = champ_data['L06']['position']['position']
    true_receiver_clk_bias = np.abs(np.random.normal(0, 1e4))

    # Initialize lists to store errors
    init_position_errors = []
    final_position_errors = []
    factor_graph = None
    last_initial_estimate = None

    position_noise_model = noiseModel.Isotropic.Sigma(3, POS_NOISE_STDDEV)
    clock_bias_noise_model = noiseModel.Isotropic.Sigma(1, CLOCK_NOISE_STDDEV)
    range_noise_model = noiseModel.Isotropic.Sigma(1, RANGE_NOISE_STDDEV)

    for _ in range(n_simulations):
        init_err, final_err, factor_graph, last_initial_estimate = build_and_optimize_factor_graph(
            true_receiver_position,
            true_receiver_clk_bias,
            gnss_sp3,
            rnx_data,
            position_noise_model,
            clock_bias_noise_model,
            range_noise_model,
            start_epoch,
            print_results
        )
        init_position_errors.append(init_err)
        final_position_errors.append(final_err)

    # If multiple simulations were run, generate and plot the CDF
    if plot_cdf and n_simulations > 1:
        plot_position_error_cdf(
            init_position_errors,
            final_position_errors,
            plot_output_path
        )

    return factor_graph, last_initial_estimate

if __name__ == "__main__":

    # Run the simulations and visualize the factor graph
    factor_graph, initial_estimate = run_simulations(
        n_simulations=1,
        print_results=True
    )
    visualize_factor_graph(factor_graph, initial_estimate, "plots/ex_1_factor_graph")
