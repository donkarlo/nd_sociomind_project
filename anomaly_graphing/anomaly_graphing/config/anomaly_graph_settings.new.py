from dataclasses import dataclass


@dataclass(frozen=True)
class AnomalyGraphSettings:
    sample_count: int = 1000
    lap_duration_seconds: float = 100.0
    wasserstein_threshold: float = 1.0
    gps_anomaly_fraction: float = 0.10
    lidar_anomaly_fraction: float = 0.25
    fused_anomaly_fraction: float = 0.25
    central_high_fraction: float = 0.10
    above_threshold_ratio: float = 0.95
    random_seed: int = 37
