from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class AnomalyGraphSettings:
    sample_count: int = 1000
    gps_wasserstein_threshold: float = 0.85
    lidar_wasserstein_threshold: float = 1.15
    fused_wasserstein_threshold: float = 1.00
    gps_anomaly_fraction: float = 0.10
    lidar_anomaly_fraction: float = 0.25
    fused_anomaly_fraction: float = 0.25
    central_high_fraction: float = 0.10
    above_threshold_ratio: float = 0.95
    random_seed: int = 37
    wall_start_fraction: float = 0.45
    wall_end_fraction: float = 0.55

    def one_round_yaml_path(self) -> Path:
        return Path(__file__).resolve().parents[2] / "data" / "one_round.yaml"
