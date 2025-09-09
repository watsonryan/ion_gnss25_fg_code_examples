import numpy as np
import gtsam


'''
* What is a factor in GTSAM
    *  it's a class (object) that defines the residual function (i.e., what we want to minimize)
    
    * the constructor takes in:
        * the hidden variables connections
        * the observation
        * the noise model (likelihood function)
        * and other parameters as needed (e.g., Satellite ephemeris)

    * evaluateError is a function in class that:
        * takes in current hidden variable values
        * returns the error (residual) (function - measurement)
            * Measurement is value passed in when constructor is called
            * Function is why you write your own factor code...
        * Also returns derivative of residual w.r.t. hidden variables
'''

def pseudorange_error(measured_pseudorange, gnss_sv_pos, this_factor, values, jacobians):
    """
    Error function for the pseudorange factor.

    Args:
        prn_key (string): ID for the current GNSS SV 
        epoch (float): time-stamp of GPS reception 
        measured_pseudorange (float): Measured pseudorange in meters.
        gnss_sv_pos (np.array): GNSS SP3 position
        this_factor (gtsam.CustomFactor): The CustomFactor instance.
        values (gtsam.Values): The Values object containing variable estimates.
        jacobians (list of numpy.ndarray): Optional Jacobians with respect to position and clock bias.

    Returns:
        numpy.ndarray: Residual error for the factor.
    """

    # Retrieve the keys for position and clock bias
    position_key = this_factor.keys()[0]
    clock_bias_key = this_factor.keys()[1]

    # Retrieve the receiver position and clock bias estimates
    receiver_pos = np.array(values.atPoint3(position_key))
    receiver_clock_bias = values.atDouble(clock_bias_key)

    geometric_range = np.linalg.norm(gnss_sv_pos - receiver_pos)

    # Predicted pseudorange includes geometric range and clock bias
    predicted_pseudorange = geometric_range + receiver_clock_bias

    # Compute residual (error between measured and predicted pseudorange)
    residual = measured_pseudorange - predicted_pseudorange

    if jacobians is not None:
        # Compute the unit line-of-sight vector from receiver to satellite
        los_vector = (gnss_sv_pos - receiver_pos) / geometric_range  # Note the sign

        # Jacobian with respect to receiver position (partial derivative of residual w.r.t position)
        jacobian_pos = los_vector.reshape(1, -1)  # Shape (1, 3)

        # Jacobian with respect to clock bias (partial derivative of residual w.r.t clock bias)
        jacobian_clock = np.array([[-1.0]])  # Shape (1, 1)

        # Assign the computed Jacobians
        jacobians[0] = jacobian_pos  # Jacobian w.r.t position
        jacobians[1] = jacobian_clock  # Jacobian w.r.t clock bias

    return np.array([residual])