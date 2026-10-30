from anomaly_graphing.data.model.resampled_scenario import ResampledScenario
from anomaly_graphing.data.processing.robust_anomaly_normalizer import RobustAnomalyNormalizer


class LidarAnomalyCalculator:
    def __init__(self):
        self._normalizer = RobustAnomalyNormalizer()

    def calculate(self, normal: ResampledScenario, follow: ResampledScenario) -> tuple[float, ...]:
        raw_values: list[float] = []
        for index in range(len(normal.leader_lidar)):
            leader_difference = self._scan_difference(normal.leader_lidar[index], follow.leader_lidar[index])
            follower_difference = self._scan_difference(normal.follower_lidar[index], follow.follower_lidar[index])
            raw_values.append(0.5 * (leader_difference + follower_difference))
        return self._normalizer.normalize(raw_values)

    def _scan_difference(self, normal_scan: tuple[float, ...], follow_scan: tuple[float, ...]) -> float:
        relative_differences = [
            abs(normal_value - follow_value) / max(1.0, normal_value)
            for normal_value, follow_value in zip(normal_scan, follow_scan)
        ]
        if not relative_differences:
            return 0.0

        relative_differences.sort(reverse=True)
        retained_count = max(1, len(relative_differences) // 4)
        return sum(relative_differences[:retained_count]) / retained_count
