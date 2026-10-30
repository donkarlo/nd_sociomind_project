import re
from pathlib import Path


class RosYamlTimeReader:
    _timestamp_pattern = re.compile(r"#\s*timestamp\s*\(sec\):\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))")

    def read_duration_seconds(self, yaml_path: Path) -> float:
        first_timestamp: float | None = None
        last_timestamp: float | None = None

        with yaml_path.open("r", encoding="utf-8", errors="ignore") as yaml_file:
            for line in yaml_file:
                match = self._timestamp_pattern.search(line)
                if match is None:
                    continue

                timestamp = float(match.group(1))
                if first_timestamp is None:
                    first_timestamp = timestamp
                last_timestamp = timestamp

        if first_timestamp is None or last_timestamp is None:
            raise ValueError(f"No ROS timestamps were found in {yaml_path}.")

        duration_seconds = last_timestamp - first_timestamp
        if duration_seconds <= 0.0:
            raise ValueError(f"ROS YAML duration is not positive in {yaml_path}.")

        return duration_seconds
