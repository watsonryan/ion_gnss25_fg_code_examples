import numpy as np
from math import sqrt, atan2, sin, cos
from datetime import datetime, timedelta

class Earth:
    # Class variables (constants)
    a = 6378137.0  # semi-major axis (meters)
    b = 6356752.3142  # semi-minor axis (meters)
    f = 1 / 298.257223560  # flattening
    e_sqr = 1 - (b ** 2) / (a ** 2)  # eccentricity squared
    ep_sqr = (a ** 2) / (b ** 2) - 1  # second eccentricity squared
    rotation_rate = 7.292115e-5  # Earth's rotation rate (rad/sec)

    @staticmethod
    def from_xyz_to_llh(xyz: np.ndarray) -> list:
        """
        Converts ECEF XYZ coordinates to geodetic coordinates (latitude, longitude, height).
        Returns latitude and longitude in radians.
        """
        x, y, z = xyz
        a = Earth.a
        e_sqr = Earth.e_sqr

        longitude = atan2(y, x)
        p = sqrt(x**2 + y**2)

        # Initial approximation of latitude
        latitude = atan2(z, p * (1 - e_sqr))

        # Iterative computation
        for _ in range(5):
            N = a / sqrt(1 - e_sqr * sin(latitude)**2)
            height = p / cos(latitude) - N
            latitude = atan2(z, p * (1 - e_sqr * N / (N + height)))

        return [latitude, longitude, height]

    @staticmethod
    def from_xyz_to_enu(sat_xyz: np.ndarray, sta_xyz: np.ndarray) -> np.ndarray:
        """
        Converts satellite ECEF XYZ coordinates to ENU coordinates relative to a station's position.
        """
        if sat_xyz.shape != (3,) or sta_xyz.shape != (3,):
            raise ValueError("Input coordinates must be 1D numpy arrays with shape (3,)")

        # Convert station ECEF coordinates to geodetic coordinates (LLH)
        sta_llh = Earth.from_xyz_to_llh(sta_xyz)
        lat, lon = sta_llh[0], sta_llh[1]
        sin_lat, cos_lat = sin(lat), cos(lat)
        sin_lon, cos_lon = sin(lon), cos(lon)

        # Difference between satellite and station position in ECEF
        diff_xyz = sat_xyz - sta_xyz

        # Rotation matrix from ECEF to ENU
        R = np.array([
            [-sin_lon, cos_lon, 0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat]
        ])

        # Convert to ENU coordinates
        enu = R @ diff_xyz
        return enu

    @staticmethod
    def calculate_elevation_azimuth(sat_xyz: np.ndarray, sta_xyz: np.ndarray) -> tuple:
        """
        Calculate the elevation and azimuth of a satellite from a station's position.
        Returns elevation and azimuth in radians.
        """
        enu = Earth.from_xyz_to_enu(sat_xyz, sta_xyz)
        east, north, up = enu

        # Compute horizontal distance
        horizontal_dist = sqrt(east**2 + north**2)

        # Elevation angle in radians
        elevation = atan2(up, horizontal_dist)

        # Azimuth angle in radians, measured clockwise from the North
        azimuth = atan2(east, north)
        if azimuth < 0:
            azimuth += 2 * np.pi  # Ensure azimuth is between 0 and 2π

        return elevation, azimuth

    @staticmethod
    def earth_to_inertial(t, to=0, omega=7.2921150e-5):
        """
        Earth to Inertial Transformation Matrix

        Inputs:
        - t [sec]: Time of transformation
        - to [sec]: Epoch at which ECI and ECEF are coincident, default is 0
        - omega [rad/sec]: Earth's rotation rate, defaults to 7.2921150e-5 rad/sec (IAU value)

        Returns:
        - C [3x3 numpy matrix]: Rotation matrix from ECEF to ECI coordinates
        """
        theta = omega * (t - to)
        c = np.cos(theta)
        s = np.sin(theta)

        # Corrected rotation matrix about the z-axis
        C = np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])
        return C