from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class AnomalyGraphSettings:
    sample_count: int = 1000
    threshold_center: float = 0.125
    threshold_random_half_range: float = 0.05
    gps_anomaly_fraction: float = 0.10
    gps_transition_fraction: float = 0.01
    lidar_anomaly_fraction: float = 0.25
    fused_anomaly_fraction: float = 0.25
    central_high_fraction: float = 0.10
    lidar_transition_fraction: float = 1.0 / 3.0
    positive_class_error_probability: float = 0.03
    negative_class_error_probability: float = 0.01
    random_seed: int = 37
    wall_start_fraction: float = 0.45
    wall_end_fraction: float = 0.55

    def lidar_visibility_start_fraction(self) -> float:
        return 0.5 - self.lidar_anomaly_fraction / 2.0

    def lidar_visibility_end_fraction(self) -> float:
        return 0.5 + self.lidar_anomaly_fraction / 2.0

    def one_round_yaml_path(self) -> Path:
        return Path(__file__).resolve().parents[2] / "data" / "one_round.yaml"
