from dataclasses import dataclass


@dataclass(frozen=True)
class AnomalyThresholds:
    gps: float
    lidar: float
    fused: float
