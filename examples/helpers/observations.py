import numpy as np

# Constants for GPS frequencies
FREQUENCY_L1 = 1575.42e6  # L1 frequency in Hz
FREQUENCY_L2 = 1227.60e6  # L2 frequency in Hz

def simulate_pseudorange(receiver_position, satellite_position, receiver_clock_bias, noise_stddev=1.0, probability_of_fault=0.0):
    """
    Simulate pseudorange measurement with added noise.

    Parameters:
    receiver_position (np.array): Receiver position in ECEF.
    satellite_position (np.array): Satellite position in ECEF.
    receiver_clock_bias (float): Receiver clock bias in meters.
    noise_stddev (float): Standard deviation of pseudorange noise.

    Returns:
    float: Simulated pseudorange measurement.
    """
    # Compute geometric distance between satellite and receiver
    distance = np.linalg.norm(satellite_position - receiver_position)
    # Add clock bias and some noise
    pseudorange = distance + receiver_clock_bias + np.random.normal(0, noise_stddev)
    test_num = np.random.uniform(0, 1)
    if (test_num > (1.0 - probability_of_fault)):
        pseudorange += np.random.normal(50, 10, 1)
    return pseudorange

def ionosphere_free_combination(P1: float, P2: float) -> float:
    """
    Calculate the ionosphere-free (IF) combination of P1 and P2 pseudorange data.

    Parameters:
    P1 (float): Pseudorange on L1 frequency (in meters).
    P2 (float): Pseudorange on L2 frequency (in meters).

    Returns:
    float: Ionosphere-free pseudorange.
    """
    # Calculate the coefficients based on frequencies
    f1 = FREQUENCY_L1
    f2 = FREQUENCY_L2
    c1 = (f1 ** 2) / (f1 ** 2 - f2 ** 2)
    c2 = (-f2 ** 2) / (f1 ** 2 - f2 ** 2)
    # Ionosphere-free combination formula
    IF_P = c1 * P1 + c2 * P2
    return IF_P

def elevation_dependent_scaling(elevation: float) -> float:
    """
    Adjust the pseudorange measurement noise based on the elevation angle.

    Parameters:
    elevation (float): Elevation angle in radians.

    Returns:
    float: Scaling factor for the measurement noise.
    """
    # Minimum elevation angle for reliable GNSS signals (5 degrees in radians)
    min_elevation = np.deg2rad(5.0)

    # Cap the elevation to the minimum elevation angle
    elevation = max(elevation, min_elevation)

    # Noise increases as the satellite approaches the horizon
    scaling_factor = 1 / np.sin(elevation)  # Weight increases as elevation decreases
    return scaling_factor