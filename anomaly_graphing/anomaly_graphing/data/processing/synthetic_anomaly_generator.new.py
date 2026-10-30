import math
import random

from anomaly_graphing.config.anomaly_graph_settings import AnomalyGraphSettings
from anomaly_graphing.data.model.anomaly_series_bundle import AnomalySeriesBundle
from anomaly_graphing.data.processing.gaussian_wasserstein_distance_calculator import GaussianWassersteinDistanceCalculator


class SyntheticAnomalyGenerator:
    def __init__(self, settings: AnomalyGraphSettings):
        self._settings = settings
        self._wasserstein_calculator = GaussianWassersteinDistanceCalculator()
        self._random = random.Random(settings.random_seed)

    def generate(self) -> AnomalySeriesBundle:
        time_seconds = tuple(
            self._settings.lap_duration_seconds * index / (self._settings.sample_count - 1)
            for index in range(self._settings.sample_count)
        )

        gps_target = self._build_modality_profile(
            self._settings.gps_anomaly_fraction,
            quiet_floor=0.18,
            quiet_noise_scale=0.10,
            anomaly_floor=1.20,
            anomaly_noise_scale=0.26,
        )
        lidar_target = self._build_modality_profile(
            self._settings.lidar_anomaly_fraction,
            quiet_floor=0.22,
            quiet_noise_scale=0.13,
            anomaly_floor=1.18,
            anomaly_noise_scale=0.30,
        )
        fused_target = self._build_fused_profile(gps_target, lidar_target)

        gps_distance = tuple(self._distance_from_target(value) for value in gps_target)
        lidar_distance = tuple(self._distance_from_target(value) for value in lidar_target)
        fused_distance = tuple(self._distance_from_target(value) for value in fused_target)

        return AnomalySeriesBundle(time_seconds, gps_distance, lidar_distance, fused_distance)

    def _build_modality_profile(self, anomaly_fraction: float, quiet_floor: float, quiet_noise_scale: float, anomaly_floor: float, anomaly_noise_scale: float) -> tuple[float, ...]:
        start_index, end_index = self._centered_interval_indices(anomaly_fraction)
        interval_count = end_index - start_index + 1
        below_threshold_count = max(1, round((1.0 - self._settings.above_threshold_ratio) * interval_count))
        below_threshold_indices = self._edge_indices(start_index, end_index, below_threshold_count)

        quiet_noise = self._correlated_noise(self._settings.sample_count, quiet_noise_scale, 0.82)
        anomaly_noise = self._correlated_noise(self._settings.sample_count, anomaly_noise_scale, 0.64)
        values: list[float] = []

        for index in range(self._settings.sample_count):
            fraction = index / (self._settings.sample_count - 1)
            slow_component = 0.04 * math.sin(7.0 * math.pi * fraction + 0.6)
            medium_component = 0.03 * math.sin(29.0 * math.pi * fraction + 1.7)
            quiet_value = max(0.03, quiet_floor + quiet_noise[index] + slow_component + medium_component)

            if index < start_index or index > end_index:
                random_spike = 0.0
                if self._random.random() < 0.015:
                    random_spike = self._random.uniform(0.04, 0.18)
                values.append(min(self._settings.wasserstein_threshold * 0.82, quiet_value + random_spike))
                continue

            if index in below_threshold_indices:
                values.append(self._random.uniform(0.72, 0.96) * self._settings.wasserstein_threshold)
                continue

            local_fraction = (index - start_index) / max(1, interval_count - 1)
            broad_shape = 0.12 * math.sin(math.pi * local_fraction) ** 2
            random_peak = 0.0
            if self._random.random() < 0.08:
                random_peak = self._random.uniform(0.08, 0.34)

            anomaly_value = anomaly_floor + broad_shape + abs(anomaly_noise[index]) + random_peak
            values.append(max(self._settings.wasserstein_threshold + 0.04, anomaly_value))

        return tuple(values)

    def _build_fused_profile(self, gps_target: tuple[float, ...], lidar_target: tuple[float, ...]) -> tuple[float, ...]:
        start_index, end_index = self._centered_interval_indices(self._settings.fused_anomaly_fraction)
        central_start_index, central_end_index = self._centered_interval_indices(self._settings.central_high_fraction)
        interval_count = end_index - start_index + 1
        below_threshold_count = max(1, round((1.0 - self._settings.above_threshold_ratio) * interval_count))
        below_threshold_indices = self._edge_indices(start_index, end_index, below_threshold_count)
        fused_noise = self._correlated_noise(self._settings.sample_count, 0.18, 0.70)
        values: list[float] = []

        for index in range(self._settings.sample_count):
            if index < start_index or index > end_index:
                quiet_value = 0.16 + abs(fused_noise[index]) * 0.55
                values.append(min(self._settings.wasserstein_threshold * 0.80, quiet_value))
                continue

            if index in below_threshold_indices:
                values.append(self._random.uniform(0.78, 0.97) * self._settings.wasserstein_threshold)
                continue

            base_fusion = 0.52 * lidar_target[index] + 0.48 * gps_target[index]
            random_component = abs(fused_noise[index]) + self._random.uniform(0.02, 0.16)
            fused_value = max(self._settings.wasserstein_threshold + 0.08, base_fusion + random_component)

            if central_start_index <= index <= central_end_index:
                central_fraction = (index - central_start_index) / max(1, central_end_index - central_start_index)
                central_boost = 0.55 + 0.70 * math.sin(math.pi * central_fraction) ** 2
                central_boost += self._random.uniform(0.08, 0.30)
                fused_value += central_boost

            values.append(fused_value)

        return tuple(values)

    def _correlated_noise(self, count: int, scale: float, persistence: float) -> list[float]:
        values: list[float] = []
        previous_value = 0.0

        for _ in range(count):
            innovation = self._random.gauss(0.0, scale)
            current_value = persistence * previous_value + (1.0 - persistence) * innovation
            values.append(current_value)
            previous_value = current_value

        return values

    def _edge_indices(self, start_index: int, end_index: int, count: int) -> set[int]:
        left_count = count // 2
        right_count = count - left_count
        indices = set(range(start_index, start_index + left_count))
        indices.update(range(end_index - right_count + 1, end_index + 1))
        return indices

    def _centered_interval_indices(self, interval_fraction: float) -> tuple[int, int]:
        interval_count = max(1, round(self._settings.sample_count * interval_fraction))
        start_index = (self._settings.sample_count - interval_count) // 2
        end_index = start_index + interval_count - 1
        return start_index, end_index

    def _distance_from_target(self, target_distance: float) -> float:
        mean_component = 0.94 * target_distance
        standard_deviation_component = math.sqrt(max(0.0, target_distance * target_distance - mean_component * mean_component))

        first_mean = (0.0, 0.0)
        second_mean = (mean_component, 0.0)
        first_standard_deviation = (1.0, 1.0)
        second_standard_deviation = (1.0 + standard_deviation_component, 1.0)

        return self._wasserstein_calculator.calculate_diagonal(
            first_mean,
            first_standard_deviation,
            second_mean,
            second_standard_deviation,
        )
