import numpy as np
from datetime import datetime
from typing import List, Dict, Optional
from examples.helpers.gpstime import GPSTimeConverter

class RinexReader:
    def __init__(self, file_path: str) -> None:
        self.file_path = file_path
        self.header = {}
        self.observation_types = []
        self.data = []

        self._parse_rinex_file()

    def _parse_rinex_file(self) -> None:
        """Parse the RINEX file and extract header and observation data."""
        with open(self.file_path, 'r') as file:
            lines = file.readlines()

        header_lines, data_lines = self._split_header_and_data(lines)
        self._parse_header(header_lines)
        self._parse_data(data_lines)

    def _split_header_and_data(self, lines: List[str]) -> (List[str], List[str]):
        """Split the header and data sections."""
        header_lines = []
        data_lines = []
        in_header = True

        for line in lines:
            if in_header:
                header_lines.append(line)
                if 'END OF HEADER' in line:
                    in_header = False
            else:
                data_lines.append(line)

        return header_lines, data_lines

    def _parse_header(self, header_lines: List[str]) -> None:
        """Parse the header of the RINEX file."""
        i = 0
        while i < len(header_lines):
            line = header_lines[i]
            if "TYPES OF OBSERV" in line:
                parts = line[:60].split()
                num_obs = int(parts[0])
                self.observation_types.extend(parts[1:])
                while len(self.observation_types) < num_obs:
                    i += 1
                    line = header_lines[i]
                    parts = line[:60].split()
                    self.observation_types.extend(parts)
            elif "TIME OF FIRST OBS" in line:
                self.header['time_of_first_obs'] = self._parse_time_of_first_obs(line)
            i += 1

    def _parse_time_of_first_obs(self, line: str) -> float:
        """Parse the TIME OF FIRST OBS line in the header and return GPS time."""
        fields = line[:60].split()
        time_info = list(map(int, fields[:5]))  # Year, month, day, hour, minute
        seconds = float(fields[5])  # Seconds with potential decimal places
        utc_time = datetime(*time_info, int(seconds), int((seconds % 1) * 1e6))
        gps_time = GPSTimeConverter.utc_to_gps_time(utc_time)
        return gps_time

    def _is_epoch_line(self, line: str) -> bool:
        """Check if a line is an epoch line."""
        tokens = line.strip().split()
        if len(tokens) >= 8:
            try:
                # Try parsing date and time tokens
                int(tokens[0])  # Year
                int(tokens[1])  # Month
                int(tokens[2])  # Day
                int(tokens[3])  # Hour
                int(tokens[4])  # Minute
                float(tokens[5])  # Second
                int(tokens[6])  # Epoch flag
                int(tokens[7])  # Number of satellites
                return True
            except ValueError:
                return False
        return False

    def _parse_epoch_line(self, line: str) -> Optional[Dict]:
        """Parse the epoch line to extract the timestamp and satellite IDs."""
        tokens = line.strip().split()
        try:
            year = int(tokens[0])
            if year < 80:
                year += 2000
            elif year < 100:
                year += 1900
            month = int(tokens[1])
            day = int(tokens[2])
            hour = int(tokens[3])
            minute = int(tokens[4])
            second = float(tokens[5])
            epoch_flag = int(tokens[6])
            num_satellites = int(tokens[7])
            # Satellite IDs may span multiple lines if there are many satellites
            sat_ids = []
            index = 8
            while len(sat_ids) < num_satellites:
                if index >= len(tokens):
                    # Need to read the next line(s)
                    self._current_line_index += 1
                    if self._current_line_index >= len(self._data_lines):
                        break
                    next_line = self._data_lines[self._current_line_index].strip()
                    tokens.extend(next_line.split())
                    continue
                sat_id = tokens[index]
                sat_ids.append(sat_id)
                index += 1
            utc_time = datetime(year, month, day, hour, minute, int(second), int((second % 1) * 1e6))
            gps_time = GPSTimeConverter.utc_to_gps_time(utc_time)
            return {
                'time': gps_time,
                'satellite_ids': sat_ids
            }
        except (ValueError, IndexError) as e:
            print(f"Error parsing epoch line: {line}")
            return None

    def _parse_data(self, data_lines: List[str]) -> None:
        """Parse the data lines in the RINEX file."""
        self._data_lines = data_lines
        self._current_line_index = 0
        while self._current_line_index < len(self._data_lines):
            line = self._data_lines[self._current_line_index].strip()
            if self._is_epoch_line(line):
                # Parse the epoch line
                epoch_data = self._parse_epoch_line(line)
                self._current_line_index += 1
                if epoch_data is None:
                    continue
                # Collect observation data for each satellite
                satellite_observations = {}
                for sat_id in epoch_data['satellite_ids']:
                    # Each satellite may have multiple observation lines
                    obs_lines = []
                    # Read lines until we have all observation data
                    num_obs_lines = (len(self.observation_types) + 4) // 5  # 5 observations per line
                    for _ in range(num_obs_lines):
                        if self._current_line_index < len(self._data_lines):
                            obs_lines.append(self._data_lines[self._current_line_index].rstrip('\n'))
                            self._current_line_index += 1
                    # Combine the lines and parse
                    obs_line = ' '.join(obs_lines)
                    satellite_observations[sat_id] = self._parse_observation_line(obs_line)
                self.data.append({
                    'epoch': {'time': epoch_data['time']},
                    'satellites': satellite_observations
                })
            else:
                self._current_line_index += 1

    def _parse_observation_line(self, line: str) -> Dict[str, Optional[float]]:
        """Parse a single observation line for a satellite."""
        values = line.split()
        if len(values) < len(self.observation_types):
            print(f"Skipping line due to insufficient values: {line}")
            return {}
        obs_values = []
        for val in values:
            try:
                obs_values.append(float(val))
            except ValueError:
                obs_values.append(None)
        return {obs_type: obs_values[i] for i, obs_type in enumerate(self.observation_types)}

    def get_data(self) -> List[Dict[str, Dict[str, np.ndarray]]]:
        """Return the parsed RINEX data."""
        return self.data

    def get_epoch_data(self, gps_time: float) -> Optional[Dict[str, Dict[str, np.ndarray]]]:
        """Retrieve data for a specific epoch."""
        for epoch in self.data:
            epoch_time = epoch['epoch']['time']
            if epoch_time == gps_time:
                return epoch['satellites']
        return None