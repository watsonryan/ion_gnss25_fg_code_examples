import numpy as np
from pathlib import Path
from datetime import datetime, timedelta
from typing import List, Dict, Optional, Tuple
from examples.helpers.gpstime import GPSTimeConverter

# Speed of light in meters per microsecond (used for converting clock bias)
SPEED_OF_LIGHT_METERS_PER_MICROSECOND: float = 299.792458
SPEED_OF_LIGHT_METERS_PER_SECOND: float = 299792458.0  # Speed of light in m/s

class SP3Reader:
    def __init__(self, file_path: str) -> None:
        self.file_path = Path(file_path)
        self.data: Dict[datetime, Dict[str, Dict[str, Dict[str, float]]]] = {}

        self._parse_sp3_file()

    def _parse_sp3_file(self) -> None:
        """Parse the SP3 file and store the data."""
        with self.file_path.open('r') as file:
            current_epoch: Optional[datetime] = None

            for line in file:
                if line.startswith('*'):
                    current_epoch = self._parse_time_epoch(line)
                elif line.startswith(('P', 'V')):
                    sat_id, data_type, sat_info = self._parse_satellite_data(line)
                    if current_epoch is None:
                        continue
                    if current_epoch not in self.data:
                        self.data[current_epoch] = {}
                    if sat_id not in self.data[current_epoch]:
                        self.data[current_epoch][sat_id] = {}
                    self.data[current_epoch][sat_id][data_type] = sat_info

    def _parse_time_epoch(self, line: str) -> datetime:
        """Parse time epoch line from SP3."""
        year = int(line[3:7])
        month = int(line[8:10])
        day = int(line[11:13])
        hour = int(line[14:16])
        minute = int(line[17:19])
        second = float(line[20:31])
    
        return datetime(year, month, day, hour, minute, int(second))

    def _parse_satellite_data(self, line: str) -> Tuple[str, str, Dict[str, np.ndarray]]:
        """Parse satellite position (P) and velocity (V) data."""
        sat_id = line[1:4].strip()  # Satellite ID (e.g., L06)
        data_type = 'position' if line[0] == 'P' else 'velocity'
        
        # Split the line into parts and only take the first 4 (x, y, z, clock)
        parts = line[5:].split()
        if len(parts) < 4:
            raise ValueError(f"Unexpected line format: {line}")
        
        x, y, z, clock = map(float, parts[:4])  # Extract only the first 4 values

        if data_type == 'position':  # Positions are in kilometers, convert to meters
            return sat_id, data_type, {
                'position': np.array([x * 1000, y * 1000, z * 1000]),  # Convert to meters
                'clock': clock * SPEED_OF_LIGHT_METERS_PER_MICROSECOND  # Convert clock bias (μs) to meters
            }
        elif data_type == 'velocity':  # Velocities are in decimeters/second, convert to meters/second
            return sat_id, data_type, {
                'velocity': np.array([x / 10, y / 10, z / 10]),  # Convert from decimeters/second to meters/second
                'clock_rate_change': clock * 1e-4 * SPEED_OF_LIGHT_METERS_PER_MICROSECOND  # Convert rate of change (10^-4 μs/s) to meters/second
            }

    def get_satellite_data(self, gps_time: int, sat_id: Optional[str] = None, receiver_position: Optional[np.ndarray] = None, max_iterations: int = 10, tolerance: float = 1e-8) -> Optional[Dict[str, Dict[str, np.ndarray]]]:
        """
        Retrieve interpolated satellite data for the given GPS time.

        If 'sat_id' and 'receiver_position' are provided, the method will perform light-time iteration to estimate
        the transmit time and retrieve the satellite data at that time.

        Parameters:
        - gps_time: GPS time in seconds
        - sat_id: Satellite identifier (e.g., 'G01')
        - receiver_position: Receiver's approximate position in ECEF coordinates (meters)
        - max_iterations: Maximum number of iterations for light-time iteration
        - tolerance: Convergence tolerance in seconds for light-time iteration

        Returns:
        - Dictionary containing interpolated satellite data, or None if data is unavailable
        """
        c = SPEED_OF_LIGHT_METERS_PER_SECOND  # Speed of light in m/s

        if sat_id is not None and receiver_position is not None:
            # Perform light-time iteration to estimate transmit time
            received_time_gps = gps_time
            transmit_time_gps = received_time_gps

            for iteration in range(max_iterations):
                # Retrieve satellite data at the current transmit time estimate
                sat_data = self._interpolated_satellite_data(transmit_time_gps)
                if sat_data is None or sat_id not in sat_data:
                    print(f"Satellite data not available for {sat_id} at time {transmit_time_gps}")
                    return None

                sat_position = sat_data[sat_id]['position']['position']
                sat_clock_bias = sat_data[sat_id]['position']['clock'] / c  # Convert clock bias from meters to seconds

                # Correct transmit time for satellite clock bias
                corrected_transmit_time_gps = transmit_time_gps - sat_clock_bias

                # Compute geometric range
                range_vector = sat_position - receiver_position
                geometric_range = np.linalg.norm(range_vector)

                # Compute signal travel time
                signal_travel_time = geometric_range / c

                # Update transmit time estimate
                new_transmit_time_gps = received_time_gps - signal_travel_time - sat_clock_bias

                # Check for convergence
                if abs(new_transmit_time_gps - transmit_time_gps) < tolerance:
                    transmit_time_gps = new_transmit_time_gps
                    break

                transmit_time_gps = new_transmit_time_gps

            else:
                print("Light-time iteration did not converge within the maximum number of iterations.")
                return None

            # Retrieve satellite data at the estimated transmit time
            interpolated_data = self._interpolated_satellite_data(transmit_time_gps)
            if interpolated_data is not None:
                return {sat_id: interpolated_data.get(sat_id)}
            else:
                return None

        else:
            # No light-time iteration; retrieve satellite data at the given GPS time
            return self._interpolated_satellite_data(gps_time)

    def _interpolated_satellite_data(self, gps_time: int) -> Optional[Dict[str, Dict[str, np.ndarray]]]:
        """Helper method to interpolate satellite data at a given GPS time."""
        utc_time = GPSTimeConverter.gps_time_to_utc(gps_time)

        # Get a sorted list of epochs
        epochs = sorted(self.data.keys())

        # Convert epochs to seconds since the first epoch for numerical stability
        epoch_times = np.array([(t - epochs[0]).total_seconds() for t in epochs])

        # Check if the requested time is within the data range
        if utc_time < epochs[0] or utc_time > epochs[-1]:
            print("Requested time is outside the range of the SP3 data.")
            return None

        # Convert requested time to seconds since the first epoch
        request_time = (utc_time - epochs[0]).total_seconds()

        # Determine the number of points to use for interpolation
        num_points = 4  # For cubic interpolation, we need at least 4 points

        # Find the indices of the epochs surrounding the requested time
        idx = np.searchsorted(epoch_times, request_time)

        # Select indices for interpolation
        idx_start = max(0, idx - num_points // 2)
        idx_end = idx_start + num_points
        if idx_end > len(epoch_times):
            idx_end = len(epoch_times)
            idx_start = idx_end - num_points

        # Ensure indices are within bounds
        idx_start = max(0, idx_start)
        idx_end = min(len(epoch_times), idx_end)

        # Extract the times and data for interpolation
        interp_times = epoch_times[idx_start:idx_end]

        interpolated_data = {}

        # Collect all satellites that are present in the selected epochs
        all_sats = set()
        for epoch in epochs[idx_start:idx_end]:
            all_sats.update(self.data[epoch].keys())

        for sat_id in all_sats:
            positions = []
            clocks = []
            times = []
            velocities = []
            clock_rates = []

            # Collect data for this satellite
            for epoch_time in interp_times:
                epoch = epochs[0] + timedelta(seconds=epoch_time)
                if sat_id in self.data[epoch]:
                    sat_data = self.data[epoch][sat_id]
                    if 'position' in sat_data:
                        positions.append(sat_data['position']['position'])
                        clocks.append(sat_data['position']['clock'])
                        times.append(epoch_time)
                    if 'velocity' in sat_data:
                        velocities.append(sat_data['velocity']['velocity'])
                        clock_rates.append(sat_data['velocity']['clock_rate_change'])

            if len(times) < num_points:
                continue  # Not enough data points to interpolate

            times = np.array(times)
            positions = np.array(positions)
            clocks = np.array(clocks)

            # Perform cubic interpolation for each coordinate
            interp_pos = []
            for dim in range(3):
                coeffs = np.polyfit(times, positions[:, dim], 3)
                interp_pos.append(np.polyval(coeffs, request_time))
            interp_pos = np.array(interp_pos)

            # Interpolate clock bias
            clock_coeffs = np.polyfit(times, clocks, 3)
            interp_clock = np.polyval(clock_coeffs, request_time)

            interpolated_data[sat_id] = {
                'position': {
                    'position': interp_pos,
                    'clock': interp_clock
                }
            }

            # Interpolate velocities if available
            if len(velocities) >= num_points:
                velocities = np.array(velocities)
                clock_rates = np.array(clock_rates)

                # Interpolate velocity for each coordinate
                interp_vel = []
                for dim in range(3):
                    vel_coeffs = np.polyfit(times, velocities[:, dim], 3)
                    interp_vel.append(np.polyval(vel_coeffs, request_time))
                interp_vel = np.array(interp_vel)

                # Interpolate clock rate change
                clock_rate_coeffs = np.polyfit(times, clock_rates, 3)
                interp_clock_rate = np.polyval(clock_rate_coeffs, request_time)

                interpolated_data[sat_id]['velocity'] = {
                    'velocity': interp_vel,
                    'clock_rate_change': interp_clock_rate
                }

            else:
                # Compute velocity by differentiating position polynomials
                vel_coeffs = []
                for dim in range(3):
                    coeffs = np.polyfit(times, positions[:, dim], 3)
                    deriv_coeffs = np.polyder(coeffs)
                    interp_vel_dim = np.polyval(deriv_coeffs, request_time)
                    vel_coeffs.append(interp_vel_dim)
                interp_vel = np.array(vel_coeffs)

                # Compute derivative for clock rate change
                clock_deriv_coeffs = np.polyder(clock_coeffs)
                interp_clock_rate = np.polyval(clock_deriv_coeffs, request_time)

                interpolated_data[sat_id]['velocity'] = {
                    'velocity': interp_vel,
                    'clock_rate_change': interp_clock_rate
                }

        return interpolated_data

    # You can include the estimate_transmit_time method separately if you wish to use it independently
    def estimate_transmit_time(self, sat_id: str, received_time_gps: int, receiver_position: np.ndarray, max_iterations: int = 10, tolerance: float = 1e-8) -> Optional[float]:
        """
        Estimate the transmit time of a satellite signal using light-time iteration.

        Parameters:
        - sat_id: Satellite identifier (e.g., 'G01')
        - received_time_gps: Received time in GPS seconds
        - receiver_position: Receiver's approximate position in ECEF coordinates (meters)
        - max_iterations: Maximum number of iterations
        - tolerance: Convergence tolerance in seconds

        Returns:
        - Estimated transmit time in GPS seconds, or None if estimation fails
        """
        c = SPEED_OF_LIGHT_METERS_PER_SECOND  # Speed of light in m/s

        # Initial estimate of transmit time (received time minus an initial guess)
        transmit_time_gps = received_time_gps

        for iteration in range(max_iterations):
            # Get satellite data at the estimated transmit time
            sat_data = self._interpolated_satellite_data(transmit_time_gps)
            if sat_data is None or sat_id not in sat_data:
                print(f"Satellite data not available for {sat_id} at time {transmit_time_gps}")
                return None

            sat_position = sat_data[sat_id]['position']['position']
            sat_clock_bias = sat_data[sat_id]['position']['clock'] / c  # Convert clock bias from meters to seconds

            # Correct transmit time for satellite clock bias
            corrected_transmit_time_gps = transmit_time_gps - sat_clock_bias

            # Compute geometric range
            range_vector = sat_position - receiver_position
            geometric_range = np.linalg.norm(range_vector)

            # Compute signal travel time
            signal_travel_time = geometric_range / c

            # Update transmit time estimate
            new_transmit_time_gps = received_time_gps - signal_travel_time - sat_clock_bias

            # Check for convergence
            if abs(new_transmit_time_gps - transmit_time_gps) < tolerance:
                transmit_time_gps = new_transmit_time_gps
                print(f"Converged after {iteration + 1} iterations.")
                break

            transmit_time_gps = new_transmit_time_gps

        else:
            print("Light-time iteration did not converge within the maximum number of iterations.")
            return None

        return transmit_time_gps

