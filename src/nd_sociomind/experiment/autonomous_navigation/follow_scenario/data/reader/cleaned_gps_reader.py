import csv
import math
import re
from pathlib import Path

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.data_source_settings import DataSourceSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory import Trajectory
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.lap_detector import LapDetector


class CleanedGpsReader:
    _HEADER_NORMALIZATION_PATTERN = re.compile(r"[^a-z0-9]+")

    def __init__(self, data_source_settings: DataSourceSettings, scenario_settings: ScenarioSettings):
        self._data_source_settings = data_source_settings
        self._scenario_settings = scenario_settings

    def read_first_lap(self, leader_path: Path, follower_path: Path) -> TrajectoryPair:
        leader_trajectory = self._read_first_lap_from_file(leader_path)
        follower_trajectory = self._read_first_lap_from_file(follower_path)

        if leader_trajectory.length() < 20 or follower_trajectory.length() < 20:
            raise ValueError("Not enough GPS samples were found in the cleaned GPS files.")

        return TrajectoryPair(leader_trajectory, follower_trajectory)

    def _read_first_lap_from_file(self, file_path: Path) -> Trajectory:
        trajectory = Trajectory()
        lap_detector = LapDetector(self._scenario_settings)
        header_fields: list[str] | None = None
        x_index: int | None = None
        y_index: int | None = None
        timestamp_index: int | None = None
        numeric_line_index = 0

        with file_path.open("r", encoding="utf-8", errors="ignore", buffering=1024 * 1024) as gps_file:
            for raw_line in gps_file:
                stripped_line = raw_line.strip()
                if not stripped_line:
                    continue

                fields = self._split_line(stripped_line)
                if len(fields) < 2:
                    continue

                if header_fields is None and not self._all_fields_are_numeric(fields):
                    header_fields = fields
                    x_index, y_index, timestamp_index = self._resolve_header_indexes(header_fields)
                    continue

                numeric_values = self._parse_numeric_fields(fields)
                if numeric_values is None:
                    continue

                if x_index is None or y_index is None:
                    x_index, y_index, timestamp_index = self._resolve_numeric_indexes(numeric_values)

                if x_index >= len(numeric_values) or y_index >= len(numeric_values):
                    continue

                x_meters = numeric_values[x_index]
                y_meters = numeric_values[y_index]
                if not math.isfinite(x_meters) or not math.isfinite(y_meters):
                    continue

                timestamp_seconds = self._fallback_timestamp(numeric_line_index)
                if timestamp_index is not None and timestamp_index < len(numeric_values):
                    timestamp_seconds = self._normalize_timestamp(numeric_values[timestamp_index], numeric_line_index)
                numeric_line_index += 1

                sample = PositionSample(timestamp_seconds, x_meters, y_meters)
                if not self._should_append(trajectory, sample):
                    continue

                trajectory.append(sample)
                if lap_detector.observe(sample):
                    break

        if trajectory.length() < 20:
            raise ValueError(f"Could not read a usable GPS trajectory from {file_path}")

        if not self._trajectory_is_closed(trajectory):
            raise ValueError(
                f"A complete lap could not be detected in {file_path}. "
                "The reader stops after one closed lap and never needs the 3.3 GB YAML file."
            )

        return trajectory

    def _split_line(self, line: str) -> list[str]:
        if "," in line:
            return next(csv.reader([line], delimiter=","))
        if "\t" in line:
            return [field.strip() for field in line.split("\t")]
        if ";" in line:
            return next(csv.reader([line], delimiter=";"))
        return line.split()

    def _all_fields_are_numeric(self, fields: list[str]) -> bool:
        for field in fields:
            try:
                float(field.strip().strip("\"'"))
            except ValueError:
                return False
        return True

    def _parse_numeric_fields(self, fields: list[str]) -> list[float] | None:
        numeric_values: list[float] = []
        for field in fields:
            try:
                numeric_values.append(float(field.strip().strip("\"'")))
            except ValueError:
                return None
        return numeric_values

    def _resolve_header_indexes(self, header_fields: list[str]) -> tuple[int, int, int | None]:
        normalized_headers = [self._normalize_header(field) for field in header_fields]
        x_index = self._best_position_column(normalized_headers, "x")
        y_index = self._best_position_column(normalized_headers, "y")
        timestamp_index = self._best_timestamp_column(normalized_headers)

        if x_index is None or y_index is None:
            raise ValueError(
                "The cleaned GPS header does not contain recognizable X/Y position columns. "
                f"Header: {header_fields}"
            )

        return x_index, y_index, timestamp_index

    def _normalize_header(self, field: str) -> str:
        normalized = field.strip().strip("\"'").lower().replace("%", "percent")
        return self._HEADER_NORMALIZATION_PATTERN.sub("_", normalized).strip("_")

    def _best_position_column(self, headers: list[str], axis: str) -> int | None:
        best_index: int | None = None
        best_score = -1
        for index, header in enumerate(headers):
            score = self._position_column_score(header, axis)
            if score > best_score:
                best_score = score
                best_index = index
        if best_score <= 0:
            return None
        return best_index

    def _position_column_score(self, header: str, axis: str) -> int:
        if "velocity" in header or "twist" in header or "linear" in header:
            return -1
        if header.endswith(f"pose_pose_position_{axis}"):
            return 100
        if "position" in header and header.endswith(f"_{axis}"):
            return 90
        if header in {f"position_{axis}", f"pos_{axis}", f"p{axis}"}:
            return 80
        if header == axis:
            return 70
        if header.endswith(f"_{axis}") and "pos" in header:
            return 60
        return 0

    def _best_timestamp_column(self, headers: list[str]) -> int | None:
        preferred_names = {
            "percenttime",
            "time",
            "timestamp",
            "timestamp_seconds",
            "header_stamp",
            "stamp",
        }
        for index, header in enumerate(headers):
            if header in preferred_names:
                return index
        for index, header in enumerate(headers):
            if "timestamp" in header or header.endswith("_time"):
                return index
        return None

    def _resolve_numeric_indexes(self, numeric_values: list[float]) -> tuple[int, int, int | None]:
        if len(numeric_values) < 2:
            raise ValueError("A cleaned GPS row must contain at least X and Y values.")

        timestamp_index: int | None = None
        x_index = 0
        y_index = 1
        if len(numeric_values) >= 3 and abs(numeric_values[0]) > 1_000_000.0:
            timestamp_index = 0
            x_index = 1
            y_index = 2
        return x_index, y_index, timestamp_index

    def _normalize_timestamp(self, timestamp_value: float, numeric_line_index: int) -> float:
        if not math.isfinite(timestamp_value):
            return self._fallback_timestamp(numeric_line_index)
        absolute_value = abs(timestamp_value)
        # Follow-scenario cleaned GPS first column uses relative nanoseconds
        # (e.g. 420044000000 means 420.044 s), not Unix milliseconds.
        if 1_000_000_000_000_000_000 <= absolute_value:
            return timestamp_value / 1_000_000_000.0  # Epoch ns
        if 1_000_000_000_000_000 <= absolute_value:
            return timestamp_value / 1_000_000.0  # Epoch microseconds
        if 1_000_000_000_000 <= absolute_value:
            return timestamp_value / 1_000.0  # Epoch milliseconds
        if 1_000_000_000 <= absolute_value:
            return timestamp_value / 1_000_000_000.0  # Relative ns
        return timestamp_value

    def _fallback_timestamp(self, numeric_line_index: int) -> float:
        return float(numeric_line_index) * self._data_source_settings.fallback_sample_period_seconds

    def _should_append(self, trajectory: Trajectory, sample: PositionSample) -> bool:
        if trajectory.is_empty():
            return True
        return sample.distance_to(trajectory.samples[-1]) >= self._scenario_settings.minimum_sample_spacing_meters

    def _trajectory_is_closed(self, trajectory: Trajectory) -> bool:
        if trajectory.length() < 2:
            return False
        return trajectory.samples[0].distance_to(trajectory.samples[-1]) <= self._scenario_settings.lap_closure_radius_meters
