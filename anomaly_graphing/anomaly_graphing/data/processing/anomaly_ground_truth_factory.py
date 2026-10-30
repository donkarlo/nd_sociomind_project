from anomaly_graphing.config.anomaly_graph_settings import AnomalyGraphSettings


class AnomalyGroundTruthFactory:
    def __init__(self, settings: AnomalyGraphSettings):
        self._settings = settings

    def gps(self) -> tuple[bool, ...]:
        return tuple(
            self._settings.wall_start_fraction <= self._fraction(index) <= self._settings.wall_end_fraction
            for index in range(self._settings.sample_count)
        )

    def lidar(self) -> tuple[bool, ...]:
        start_fraction = 0.5 - self._settings.lidar_anomaly_fraction / 2.0
        end_fraction = 0.5 + self._settings.lidar_anomaly_fraction / 2.0
        return tuple(
            start_fraction <= self._fraction(index) <= end_fraction
            for index in range(self._settings.sample_count)
        )

    def fused(self) -> tuple[bool, ...]:
        gps_labels = self.gps()
        lidar_labels = self.lidar()
        return tuple(
            gps_positive or lidar_positive
            for gps_positive, lidar_positive in zip(gps_labels, lidar_labels)
        )

    def _fraction(self, index: int) -> float:
        return index / max(1, self._settings.sample_count - 1)
