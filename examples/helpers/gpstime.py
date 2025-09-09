from datetime import datetime, timedelta
from typing import List, Tuple

class GPSTimeConverter:
    """A class to handle GPS to UTC time conversion and leap second management."""

    # Table of leap seconds with their respective UTC introduction dates
    LEAP_SECONDS_TABLE: List[Tuple[datetime, int]] = [
        (datetime(1981, 6, 30, 23, 59, 59), 1),
        (datetime(1982, 6, 30, 23, 59, 59), 2),
        (datetime(1983, 6, 30, 23, 59, 59), 3),
        (datetime(1985, 6, 30, 23, 59, 59), 4),
        (datetime(1987, 12, 31, 23, 59, 59), 5),
        (datetime(1989, 12, 31, 23, 59, 59), 6),
        (datetime(1990, 12, 31, 23, 59, 59), 7),
        (datetime(1992, 6, 30, 23, 59, 59), 8),
        (datetime(1993, 6, 30, 23, 59, 59), 9),
        (datetime(1994, 6, 30, 23, 59, 59), 10),
        (datetime(1995, 12, 31, 23, 59, 59), 11),
        (datetime(1997, 6, 30, 23, 59, 59), 12),
        (datetime(1998, 12, 31, 23, 59, 59), 13),
        (datetime(2005, 12, 31, 23, 59, 59), 14),
        (datetime(2008, 12, 31, 23, 59, 59), 15),
        (datetime(2012, 6, 30, 23, 59, 59), 16),
        (datetime(2015, 6, 30, 23, 59, 59), 17),
        (datetime(2016, 12, 31, 23, 59, 59), 18),
        # Update this table if new leap seconds are announced
    ]

    @staticmethod
    def gps_time_to_utc(gps_time: float) -> datetime:
        """Convert GPS time (seconds) to UTC, accounting for leap seconds dynamically."""
        gps_epoch = datetime(1980, 1, 6)  # GPS epoch start date

        # Calculate the base UTC time without leap second correction
        utc_time = gps_epoch + timedelta(seconds=gps_time)

        # Correct for leap seconds based on the time
        leap_seconds = GPSTimeConverter.get_leap_seconds(utc_time)
        corrected_utc_time = utc_time - timedelta(seconds=leap_seconds)

        return corrected_utc_time

    @staticmethod
    def utc_to_gps_time(utc_time: datetime) -> float:
        """Convert UTC datetime to GPS time (seconds since GPS epoch), accounting for leap seconds."""
        gps_epoch = datetime(1980, 1, 6)  # GPS epoch start date

        # Calculate the total number of leap seconds up to the given UTC time
        leap_seconds = GPSTimeConverter.get_leap_seconds(utc_time)

        # GPS time is ahead of UTC by the number of leap seconds
        gps_time = (utc_time - gps_epoch).total_seconds() + leap_seconds

        return gps_time

    @staticmethod
    def calculate_gmst(gps_seconds):
        """
        Calculate the Greenwich Mean Sidereal Time (GMST) in radians for a given epoch in GPS seconds.
        """
        # Convert GPS time to UTC time
        utc_time = GPSTimeConverter.gps_to_utc(gps_seconds)
    
        # Reference epoch J2000.0
        j2000_epoch = datetime(2000, 1, 1, 12, 0, 0)
        seconds_per_day = 86400

        # Julian date calculation from UTC time
        jd = (utc_time - j2000_epoch).total_seconds() / seconds_per_day + 2451545.0

        # Calculate the number of Julian centuries since J2000.0
        t = (jd - 2451545.0) / 36525.0

        # GMST calculation (in degrees)
        gmst_deg = (280.46061837 + 360.98564736629 * (jd - 2451545.0) +
                    0.000387933 * t**2 - t**3 / 38710000.0) % 360.0

        # Convert GMST from degrees to radians
        gmst_rad = np.deg2rad(gmst_deg)

        return gmst_rad

    @staticmethod
    def get_leap_seconds(utc_time: datetime) -> int:
        """Get the correct number of leap seconds for a given UTC time."""
        leap_seconds = 0
        for leap_time, ls in GPSTimeConverter.LEAP_SECONDS_TABLE:
            if utc_time > leap_time:
                leap_seconds = ls
            else:
                break
        return leap_seconds
