from dataclasses import dataclass

from anomaly_graphing.data.model.anomaly_thresholds import AnomalyThresholds
from anomaly_graphing.data.model.binary_classification_metrics import BinaryClassificationMetrics


@dataclass(frozen=True)
class AnomalySeriesBundle:
    time_seconds: tuple[float, ...]
    gps_wasserstein_distance: tuple[float, ...]
    lidar_wasserstein_distance: tuple[float, ...]
    fused_wasserstein_distance: tuple[float, ...]
    thresholds: AnomalyThresholds
    gps_metrics: BinaryClassificationMetrics
    lidar_metrics: BinaryClassificationMetrics
    fused_metrics: BinaryClassificationMetrics
