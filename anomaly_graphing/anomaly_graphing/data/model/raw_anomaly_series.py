from dataclasses import dataclass


@dataclass(frozen=True)
class RawAnomalySeries:
    sample_positions: tuple[float, ...]
    anomaly_values: tuple[float, ...]
