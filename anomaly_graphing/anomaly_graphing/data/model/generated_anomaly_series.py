from dataclasses import dataclass


@dataclass(frozen=True)
class GeneratedAnomalySeries:
    time_seconds: tuple[float, ...]
    gps_wasserstein_distance: tuple[float, ...]
    lidar_wasserstein_distance: tuple[float, ...]
    fused_wasserstein_distance: tuple[float, ...]
