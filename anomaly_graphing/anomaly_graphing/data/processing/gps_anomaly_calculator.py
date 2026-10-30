import math

from anomaly_graphing.data.model.resampled_scenario import ResampledScenario
from anomaly_graphing.data.processing.robust_anomaly_normalizer import RobustAnomalyNormalizer


class GpsAnomalyCalculator:
    def __init__(self):
        self._normalizer = RobustAnomalyNormalizer()

    def calculate(self, normal: ResampledScenario, follow: ResampledScenario) -> tuple[float, ...]:
        raw_values: list[float] = []
        for index in range(len(normal.leader_positions)):
            normal_leader = normal.leader_positions[index]
            follow_leader = follow.leader_positions[index]
            normal_follower = normal.follower_positions[index]
            follow_follower = follow.follower_positions[index]

            leader_distance = self._distance(normal_leader, follow_leader)
            follower_distance = self._distance(normal_follower, follow_follower)
            normal_separation = self._distance(normal_leader, normal_follower)
            follow_separation = self._distance(follow_leader, follow_follower)
            formation_change = abs(normal_separation - follow_separation)

            trajectory_difference = math.sqrt(0.5 * (leader_distance * leader_distance + follower_distance * follower_distance))
            raw_values.append(trajectory_difference + 0.35 * formation_change)

        return self._normalizer.normalize(raw_values)

    def _distance(self, first: tuple[float, float], second: tuple[float, float]) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])
