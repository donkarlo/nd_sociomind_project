import math
from pathlib import Path
import re
from typing import Callable

from anomaly_graphing.config.anomaly_graph_settings import AnomalyGraphSettings
from anomaly_graphing.data.model.lidar_sample import LidarSample
from anomaly_graphing.data.model.position_sample import PositionSample
from anomaly_graphing.data.model.scenario_record import ScenarioRecord
from anomaly_graphing.data.processing.lap_detector import LapDetector


class RosYamlScenarioReader:
    _NUMBER_PATTERN = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")

    def __init__(self, settings: AnomalyGraphSettings):
        self._settings = settings

    def read_first_lap(self, yaml_path: Path, progress_callback: Callable[[float], None] | None = None) -> ScenarioRecord:
        leader_positions: list[PositionSample] = []
        follower_positions: list[PositionSample] = []
        leader_lidar: list[LidarSample] = []
        follower_lidar: list[LidarSample] = []
        lap_detector = LapDetector(self._settings)
        file_size = max(1, yaml_path.stat().st_size)
        bytes_read = 0
        last_progress_report = -1
        synthetic_timestamp = 0.0
        lap_start_timestamp: float | None = None
        lap_end_timestamp: float | None = None
        follower_reached_end = False
        leader_lidar_reached_end = False
        follower_lidar_reached_end = False

        message_topic = ""
        seconds_value: float | None = None
        nanoseconds_value: float | None = None
        position_indent = -1
        position_x: float | None = None
        position_y: float | None = None
        ranges_indent = -1
        lidar_ranges: list[float] = []
        lidar_range_max: float | None = None

        def timestamp_value() -> float:
            nonlocal synthetic_timestamp
            if seconds_value is not None:
                value = seconds_value
                if nanoseconds_value is not None:
                    value += nanoseconds_value / 1_000_000_000.0
                return value
            synthetic_timestamp += 0.05
            return synthetic_timestamp

        def append_position(target: list[PositionSample], sample: PositionSample) -> bool:
            if target and sample.distance_to(target[-1]) < self._settings.minimum_gps_sample_spacing_meters:
                return False
            target.append(sample)
            return True

        def finalize_message() -> None:
            nonlocal lap_start_timestamp
            nonlocal lap_end_timestamp
            nonlocal follower_reached_end
            nonlocal leader_lidar_reached_end
            nonlocal follower_lidar_reached_end

            if not message_topic:
                return

            current_timestamp = timestamp_value()

            if self._is_gps_topic(message_topic) and position_x is not None and position_y is not None:
                sample = PositionSample(current_timestamp, position_x, position_y)
                if self._is_leader_topic(message_topic):
                    if lap_end_timestamp is None and append_position(leader_positions, sample):
                        if lap_start_timestamp is None:
                            lap_start_timestamp = current_timestamp
                        if lap_detector.observe(sample):
                            lap_end_timestamp = current_timestamp
                elif self._is_follower_topic(message_topic):
                    if lap_start_timestamp is not None and current_timestamp >= lap_start_timestamp:
                        if lap_end_timestamp is None or current_timestamp <= lap_end_timestamp + self._settings.lidar_finish_grace_seconds:
                            append_position(follower_positions, sample)
                    if lap_end_timestamp is not None and current_timestamp >= lap_end_timestamp:
                        follower_reached_end = True

            if self._is_lidar_topic(message_topic) and lidar_ranges:
                if lap_start_timestamp is not None:
                    if lap_end_timestamp is None or current_timestamp <= lap_end_timestamp + self._settings.lidar_finish_grace_seconds:
                        sector_ranges = self._build_lidar_sector_features(lidar_ranges, lidar_range_max)
                        lidar_sample = LidarSample(current_timestamp, sector_ranges)
                        if self._is_leader_topic(message_topic):
                            leader_lidar.append(lidar_sample)
                        elif self._is_follower_topic(message_topic):
                            follower_lidar.append(lidar_sample)

                if lap_end_timestamp is not None and current_timestamp >= lap_end_timestamp:
                    if self._is_leader_topic(message_topic):
                        leader_lidar_reached_end = True
                    elif self._is_follower_topic(message_topic):
                        follower_lidar_reached_end = True

        def reset_message(topic: str) -> None:
            nonlocal message_topic
            nonlocal seconds_value
            nonlocal nanoseconds_value
            nonlocal position_indent
            nonlocal position_x
            nonlocal position_y
            nonlocal ranges_indent
            nonlocal lidar_ranges
            nonlocal lidar_range_max
            message_topic = topic
            seconds_value = None
            nanoseconds_value = None
            position_indent = -1
            position_x = None
            position_y = None
            ranges_indent = -1
            lidar_ranges = []
            lidar_range_max = None

        with yaml_path.open("rb", buffering=1024 * 1024) as yaml_file:
            for raw_bytes in yaml_file:
                bytes_read += len(raw_bytes)
                raw_line = raw_bytes.decode("utf-8", errors="ignore")
                stripped_line = raw_line.strip()
                indentation = len(raw_line) - len(raw_line.lstrip(" "))

                detected_topic = self._extract_topic(stripped_line)
                if detected_topic is not None:
                    finalize_message()
                    reset_message(detected_topic)
                    if self._can_stop(lap_end_timestamp, follower_reached_end, leader_lidar_reached_end, follower_lidar_reached_end):
                        break
                    continue

                if not message_topic:
                    continue

                if ranges_indent >= 0 and indentation > ranges_indent and stripped_line.startswith("-"):
                    value = self._extract_list_number(stripped_line)
                    if value is not None:
                        lidar_ranges.append(value)
                    continue

                if ranges_indent >= 0 and stripped_line and indentation <= ranges_indent:
                    ranges_indent = -1

                if stripped_line.startswith("secs:") or stripped_line.startswith("sec:"):
                    seconds_value = self._extract_value_after_colon(stripped_line)
                    continue

                if stripped_line.startswith("nsecs:") or stripped_line.startswith("nanosec:"):
                    nanoseconds_value = self._extract_value_after_colon(stripped_line)
                    continue

                if stripped_line == "position:" or stripped_line.endswith(" position:"):
                    position_indent = indentation
                    position_x = None
                    position_y = None
                    continue

                if position_indent >= 0 and indentation > position_indent:
                    if stripped_line.startswith("x:"):
                        position_x = self._extract_value_after_colon(stripped_line)
                        continue
                    if stripped_line.startswith("y:"):
                        position_y = self._extract_value_after_colon(stripped_line)
                        continue

                if stripped_line.startswith("range_max:"):
                    lidar_range_max = self._extract_value_after_colon(stripped_line)
                    continue

                if stripped_line.startswith("ranges:"):
                    ranges_indent = indentation
                    inline_values = self._extract_inline_list(stripped_line)
                    if inline_values:
                        lidar_ranges.extend(inline_values)
                    continue

                if progress_callback is not None:
                    progress_percent = int(100.0 * bytes_read / file_size)
                    if progress_percent != last_progress_report:
                        last_progress_report = progress_percent
                        progress_callback(min(1.0, bytes_read / file_size))

            finalize_message()

        self._validate_record(leader_positions, follower_positions, leader_lidar, follower_lidar, yaml_path)
        return ScenarioRecord(tuple(leader_positions), tuple(follower_positions), tuple(leader_lidar), tuple(follower_lidar))

    def _can_stop(self, lap_end_timestamp: float | None, follower_reached_end: bool, leader_lidar_reached_end: bool, follower_lidar_reached_end: bool) -> bool:
        if lap_end_timestamp is None:
            return False
        return follower_reached_end and leader_lidar_reached_end and follower_lidar_reached_end

    def _extract_topic(self, stripped_line: str) -> str | None:
        normalized_line = stripped_line
        if normalized_line.startswith("-"):
            normalized_line = normalized_line[1:].strip()

        if normalized_line.startswith("topic:"):
            return normalized_line.split(":", 1)[1].strip().strip("\"'")

        if normalized_line.startswith("/") and normalized_line.endswith(":"):
            return normalized_line[:-1]

        return None

    def _is_leader_topic(self, topic: str) -> bool:
        return "/uav1/" in topic or topic.startswith("uav1/")

    def _is_follower_topic(self, topic: str) -> bool:
        return "/uav2/" in topic or topic.startswith("uav2/")

    def _is_gps_topic(self, topic: str) -> bool:
        return "odom_gps" in topic or "gps" in topic and "rplidar" not in topic and "scan" not in topic

    def _is_lidar_topic(self, topic: str) -> bool:
        return "rplidar" in topic or topic.endswith("/scan")

    def _extract_value_after_colon(self, text: str) -> float | None:
        value_text = text.split(":", 1)[1]
        match = self._NUMBER_PATTERN.search(value_text)
        if match is None:
            return None
        try:
            return float(match.group(0))
        except ValueError:
            return None

    def _extract_list_number(self, text: str) -> float | None:
        value_text = text[1:].strip()
        if value_text.lower() in {"inf", ".inf", "+.inf", "infinity"}:
            return math.inf
        if value_text.lower() in {"-inf", "-.inf", "-infinity"}:
            return -math.inf
        match = self._NUMBER_PATTERN.search(value_text)
        if match is None:
            return None
        try:
            return float(match.group(0))
        except ValueError:
            return None

    def _extract_inline_list(self, text: str) -> list[float]:
        if "[" not in text or "]" not in text:
            return []
        content = text[text.find("[") + 1:text.rfind("]")]
        values: list[float] = []
        for part in content.split(","):
            value = self._extract_list_number("- " + part.strip())
            if value is not None:
                values.append(value)
        return values

    def _build_lidar_sector_features(self, ranges: list[float], range_max: float | None) -> tuple[float, ...]:
        maximum_range = range_max
        if maximum_range is None or not math.isfinite(maximum_range) or maximum_range <= 0.0:
            maximum_range = self._settings.default_lidar_max_range_meters

        sector_count = self._settings.lidar_sector_count
        sector_ranges: list[float] = []
        for sector_index in range(sector_count):
            start_index = int(sector_index * len(ranges) / sector_count)
            end_index = int((sector_index + 1) * len(ranges) / sector_count)
            if end_index <= start_index:
                end_index = min(len(ranges), start_index + 1)
            valid_values = [
                min(maximum_range, value)
                for value in ranges[start_index:end_index]
                if math.isfinite(value) and value > 0.0
            ]
            if not valid_values:
                sector_ranges.append(maximum_range)
                continue
            valid_values.sort()
            percentile_index = int(0.20 * (len(valid_values) - 1))
            sector_ranges.append(valid_values[percentile_index])
        return tuple(sector_ranges)

    def _validate_record(self, leader_positions: list[PositionSample], follower_positions: list[PositionSample], leader_lidar: list[LidarSample], follower_lidar: list[LidarSample], yaml_path: Path) -> None:
        if len(leader_positions) < 20 or len(follower_positions) < 20:
            raise ValueError(f"Not enough GPS samples were found in the first lap of {yaml_path}.")
        if len(leader_lidar) < 5 or len(follower_lidar) < 5:
            raise ValueError(f"Not enough LiDAR scans were found in the first lap of {yaml_path}.")
