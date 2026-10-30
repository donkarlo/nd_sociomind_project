from dataclasses import dataclass


@dataclass(frozen=True)
class LidarSample:
    timestamp_seconds: float
    sector_ranges_meters: tuple[float, ...]
