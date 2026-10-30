import math
import random

from anomaly_graphing.config.anomaly_graph_settings import AnomalyGraphSettings
from anomaly_graphing.data.model.anomaly_thresholds import AnomalyThresholds
from anomaly_graphing.data.model.generated_anomaly_series import GeneratedAnomalySeries
from anomaly_graphing.data.processing.gaussian_wasserstein_distance_calculator import GaussianWassersteinDistanceCalculator


class SyntheticAnomalyGenerator:
    def __init__(self, settings: AnomalyGraphSettings):
        self._settings = settings
        self._wasserstein_calculator = GaussianWassersteinDistanceCalculator()
        self._random = random.Random()

    def generate(self, lap_duration_seconds: float, thresholds: AnomalyThresholds) -> GeneratedAnomalySeries:
        time_seconds = tuple(
            lap_duration_seconds * index / (self._settings.sample_count - 1)
            for index in range(self._settings.sample_count)
        )

        gps_target = self._build_gps_profile(thresholds.gps)
        lidar_target = self._build_lidar_profile(thresholds.lidar)
        fused_target = self._build_fused_profile(gps_target, lidar_target, thresholds)

        gps_distance = tuple(self._distance_from_target(value) for value in gps_target)
        lidar_distance = tuple(self._distance_from_target(value) for value in lidar_target)
        fused_distance = tuple(self._distance_from_target(value) for value in fused_target)

        return GeneratedAnomalySeries(
            time_seconds,
            gps_distance,
            lidar_distance,
            fused_distance,
        )

    def _build_gps_profile(self, threshold: float) -> tuple[float, ...]:
        wall_start_index = self._fraction_to_index(self._settings.wall_start_fraction)
        wall_end_index = self._fraction_to_index(self._settings.wall_end_fraction)
        transition_count = max(2, round(self._settings.sample_count * self._settings.gps_transition_fraction))
        approach_start_index = max(0, wall_start_index - transition_count)
        retreat_end_index = min(self._settings.sample_count - 1, wall_end_index + transition_count)
        values: list[float] = []

        for index in range(self._settings.sample_count):
            fraction = self._fraction(index)
            normal_value = self._normal_value(index, fraction, threshold, 0.10, 0.035)
            high_value = threshold + 0.06 + abs(self._random.gauss(0.0, 0.05))
            high_value += 0.025 * math.sin(17.0 * math.pi * fraction) ** 2

            if index < approach_start_index or index > retreat_end_index:
                value = self._apply_negative_class_error(normal_value, threshold)
                values.append(value)
                continue

            if index < wall_start_index:
                progress = (index - approach_start_index) / max(1, wall_start_index - approach_start_index)
                value = self._interpolate(normal_value, threshold - 0.002, self._smooth_step(progress))
                value = self._apply_negative_class_error(min(threshold - 0.001, value), threshold)
                values.append(value)
                continue

            if index <= wall_start_index + transition_count:
                progress = (index - wall_start_index) / max(1, transition_count)
                value = self._interpolate(threshold + 0.002, high_value, self._smooth_step(progress))
                value = self._apply_positive_class_error(value, threshold)
                values.append(value)
                continue

            if index < wall_end_index - transition_count:
                value = self._apply_positive_class_error(high_value, threshold)
                values.append(value)
                continue

            if index <= wall_end_index:
                progress = (wall_end_index - index) / max(1, transition_count)
                value = self._interpolate(threshold + 0.002, high_value, self._smooth_step(progress))
                value = self._apply_positive_class_error(value, threshold)
                values.append(value)
                continue

            progress = (retreat_end_index - index) / max(1, retreat_end_index - wall_end_index)
            value = self._interpolate(normal_value, threshold - 0.002, self._smooth_step(progress))
            value = self._apply_negative_class_error(min(threshold - 0.001, value), threshold)
            values.append(value)

        return tuple(values)

    def _build_lidar_profile(self, threshold: float) -> tuple[float, ...]:
        start_index, end_index = self._centered_interval_indices(self._settings.lidar_anomaly_fraction)
        interval_count = end_index - start_index + 1
        transition_count = max(2, round(interval_count * self._settings.lidar_transition_fraction))
        approach_count = transition_count
        plateau_start_index = min(end_index, start_index + transition_count)
        plateau_end_index = max(start_index, end_index - transition_count)
        approach_start_index = max(0, start_index - approach_count)
        retreat_end_index = min(self._settings.sample_count - 1, end_index + approach_count)
        values: list[float] = []

        for index in range(self._settings.sample_count):
            fraction = self._fraction(index)
            normal_floor = max(0.02, threshold * 0.55)
            normal_noise_scale = max(0.015, threshold * 0.20)
            normal_value = self._normal_value(index, fraction, threshold, normal_floor, normal_noise_scale)
            high_value = threshold + 0.14 + abs(self._random.gauss(0.0, 0.035))

            if index < approach_start_index or index > retreat_end_index:
                value = self._apply_negative_class_error(normal_value, threshold)
                values.append(value)
                continue

            if index < start_index:
                progress = (index - approach_start_index) / max(1, start_index - approach_start_index)
                value = self._interpolate(normal_value, threshold - 0.002, self._smooth_step(progress))
                value = self._apply_negative_class_error(min(threshold - 0.001, value), threshold)
                values.append(value)
                continue

            if index <= plateau_start_index:
                progress = (index - start_index) / max(1, plateau_start_index - start_index)
                value = self._interpolate(threshold + 0.002, high_value, self._smooth_step(progress))
                value += abs(self._random.gauss(0.0, 0.006))
                value = self._apply_positive_class_error(value, threshold)
                values.append(value)
                continue

            if index < plateau_end_index:
                value = high_value + abs(self._random.gauss(0.0, 0.0075))
                value = self._apply_positive_class_error(value, threshold)
                values.append(value)
                continue

            if index <= end_index:
                progress = (end_index - index) / max(1, end_index - plateau_end_index)
                value = self._interpolate(threshold + 0.002, high_value, self._smooth_step(progress))
                value += abs(self._random.gauss(0.0, 0.006))
                value = self._apply_positive_class_error(value, threshold)
                values.append(value)
                continue

            progress = (retreat_end_index - index) / max(1, retreat_end_index - end_index)
            value = self._interpolate(normal_value, threshold - 0.002, self._smooth_step(progress))
            value = self._apply_negative_class_error(min(threshold - 0.001, value), threshold)
            values.append(value)

        return tuple(values)

    def _build_fused_profile(self, gps_target: tuple[float, ...], lidar_target: tuple[float, ...], thresholds: AnomalyThresholds) -> tuple[float, ...]:
        threshold = thresholds.fused
        lidar_start_index, lidar_end_index = self._centered_interval_indices(self._settings.lidar_anomaly_fraction)
        lidar_interval_count = lidar_end_index - lidar_start_index + 1
        transition_count = max(2, round(lidar_interval_count * self._settings.lidar_transition_fraction))
        approach_count = transition_count
        plateau_start_index = min(lidar_end_index, lidar_start_index + transition_count)
        plateau_end_index = max(lidar_start_index, lidar_end_index - transition_count)
        approach_start_index = max(0, lidar_start_index - approach_count)
        retreat_end_index = min(self._settings.sample_count - 1, lidar_end_index + approach_count)
        central_start_index, central_end_index = self._centered_interval_indices(self._settings.central_high_fraction)
        values: list[float] = []

        for index in range(self._settings.sample_count):
            fraction = self._fraction(index)
            normal_value = self._normal_value(index, fraction, threshold, 0.09, 0.025)

            gps_relative = gps_target[index] / max(thresholds.gps, 1e-9)
            lidar_relative = lidar_target[index] / max(thresholds.lidar, 1e-9)
            source_excess = max(0.0, 0.42 * (gps_relative - 1.0) + 0.58 * (lidar_relative - 1.0))
            high_value = threshold + 0.10 + threshold * source_excess

            if central_start_index <= index <= central_end_index:
                central_fraction = (index - central_start_index) / max(1, central_end_index - central_start_index)
                high_value += 0.08 + 0.13 * math.sin(math.pi * central_fraction) ** 2

            if index < approach_start_index or index > retreat_end_index:
                value = self._apply_negative_class_error(normal_value, threshold)
                values.append(value)
                continue

            if index < lidar_start_index:
                progress = (index - approach_start_index) / max(1, lidar_start_index - approach_start_index)
                value = self._interpolate(normal_value, threshold - 0.002, self._smooth_step(progress))
                value = self._apply_negative_class_error(min(threshold - 0.001, value), threshold)
                values.append(value)
                continue

            if index <= plateau_start_index:
                progress = (index - lidar_start_index) / max(1, plateau_start_index - lidar_start_index)
                value = self._interpolate(threshold + 0.002, high_value, self._smooth_step(progress))
                value += abs(self._random.gauss(0.0, 0.005))
                value = self._apply_positive_class_error(value, threshold)
                values.append(value)
                continue

            if index < plateau_end_index:
                value = high_value + abs(self._random.gauss(0.0, 0.006))
                value = self._apply_positive_class_error(value, threshold)
                values.append(value)
                continue

            if index <= lidar_end_index:
                progress = (lidar_end_index - index) / max(1, lidar_end_index - plateau_end_index)
                value = self._interpolate(threshold + 0.002, high_value, self._smooth_step(progress))
                value += abs(self._random.gauss(0.0, 0.005))
                value = self._apply_positive_class_error(value, threshold)
                values.append(value)
                continue

            progress = (retreat_end_index - index) / max(1, retreat_end_index - lidar_end_index)
            value = self._interpolate(normal_value, threshold - 0.002, self._smooth_step(progress))
            value = self._apply_negative_class_error(min(threshold - 0.001, value), threshold)
            values.append(value)

        return tuple(values)

    def _normal_value(self, index: int, fraction: float, threshold: float, floor: float, noise_scale: float) -> float:
        slow_component = 0.0125 * math.sin(7.0 * math.pi * fraction + 0.6)
        medium_component = 0.009 * math.sin(29.0 * math.pi * fraction + 1.7)
        noise = self._random.gauss(0.0, noise_scale)
        value = floor + slow_component + medium_component + noise
        return min(threshold - 0.0125, max(0.02, value))

    def _apply_positive_class_error(self, value: float, threshold: float) -> float:
        if self._random.random() < self._settings.positive_class_error_probability:
            return self._random.uniform(max(0.02, threshold - 0.05), threshold - 0.005)
        return max(threshold + 0.001, value)

    def _apply_negative_class_error(self, value: float, threshold: float) -> float:
        if self._random.random() < self._settings.negative_class_error_probability:
            return threshold + self._random.uniform(0.005, 0.04)
        return min(threshold - 0.001, value)

    def _interpolate(self, first_value: float, second_value: float, progress: float) -> float:
        return first_value + (second_value - first_value) * progress

    def _smooth_step(self, progress: float) -> float:
        bounded_progress = min(1.0, max(0.0, progress))
        return bounded_progress * bounded_progress * (3.0 - 2.0 * bounded_progress)

    def _centered_interval_indices(self, interval_fraction: float) -> tuple[int, int]:
        interval_count = max(1, round(self._settings.sample_count * interval_fraction))
        start_index = (self._settings.sample_count - interval_count) // 2
        end_index = start_index + interval_count - 1
        return start_index, end_index

    def _fraction_to_index(self, fraction: float) -> int:
        return round(fraction * (self._settings.sample_count - 1))

    def _fraction(self, index: int) -> float:
        return index / max(1, self._settings.sample_count - 1)

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
