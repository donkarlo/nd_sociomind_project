import random

from anomaly_graphing.config.anomaly_graph_settings import AnomalyGraphSettings
from anomaly_graphing.data.model.anomaly_thresholds import AnomalyThresholds


class RandomThresholdGenerator:
    def __init__(self, settings: AnomalyGraphSettings):
        self._settings = settings
        self._random = random.SystemRandom()

    def generate(self) -> AnomalyThresholds:
        lower_bound = self._settings.threshold_center - self._settings.threshold_random_half_range
        upper_bound = self._settings.threshold_center + self._settings.threshold_random_half_range

        return AnomalyThresholds(
            self._random.uniform(lower_bound, upper_bound),
            self._random.uniform(lower_bound, upper_bound),
            self._random.uniform(lower_bound, upper_bound),
        )
