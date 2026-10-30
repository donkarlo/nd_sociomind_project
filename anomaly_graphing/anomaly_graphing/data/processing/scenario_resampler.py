from bisect import bisect_left
import math

from anomaly_graphing.config.anomaly_graph_settings import AnomalyGraphSettings
from anomaly_graphing.data.model.lidar_sample import LidarSample
from anomaly_graphing.data.model.position_sample import PositionSample
from anomaly_graphing.data.model.resampled_scenario import ResampledScenario
from anomaly_graphing.data.model.scenario_record import ScenarioRecord


class ScenarioResampler:
    def __init__(self, settings: AnomalyGraphSettings):
        self._settings = settings

    def resample(self, record: ScenarioRecord) -> ResampledScenario:
        leader_positions = self._resample_positions(record.leader_positions)
        follower_positions = self._resample_positions(record.follower_positions)
        leader_lidar = self._resample_lidar(record.leader_lidar, record.leader_positions)
        follower_lidar = self._resample_lidar(record.follower_lidar, record.leader_positions)
        return ResampledScenario(leader_positions, follower_positions, leader_lidar, follower_lidar)

    def _resample_positions(self, samples: tuple[PositionSample, ...]) -> tuple[tuple[float, float], ...]:
        cumulative_distance = self._cumulative_distance(samples)
        total_distance = cumulative_distance[-1]
        if total_distance <= 0.0:
            raise ValueError("A trajectory with zero traveled distance cannot be resampled.")

        result: list[tuple[float, float]] = []
        point_count = self._settings.resampled_point_count
        for point_index in range(point_count):
            target_distance = total_distance * point_index / max(1, point_count - 1)
            upper_index = bisect_left(cumulative_distance, target_distance)
            if upper_index <= 0:
                sample = samples[0]
                result.append((sample.x_meters, sample.y_meters))
                continue
            if upper_index >= len(samples):
                sample = samples[-1]
                result.append((sample.x_meters, sample.y_meters))
                continue

            lower_index = upper_index - 1
            lower_distance = cumulative_distance[lower_index]
            upper_distance = cumulative_distance[upper_index]
            span = max(1e-12, upper_distance - lower_distance)
            fraction = (target_distance - lower_distance) / span
            lower_sample = samples[lower_index]
            upper_sample = samples[upper_index]
            x_value = lower_sample.x_meters + (upper_sample.x_meters - lower_sample.x_meters) * fraction
            y_value = lower_sample.y_meters + (upper_sample.y_meters - lower_sample.y_meters) * fraction
            result.append((x_value, y_value))
        return tuple(result)

    def _resample_lidar(self, lidar_samples: tuple[LidarSample, ...], leader_positions: tuple[PositionSample, ...]) -> tuple[tuple[float, ...], ...]:
        point_count = self._settings.resampled_point_count
        sector_count = self._settings.lidar_sector_count
        bins: list[list[tuple[float, ...]]] = [[] for _ in range(point_count)]
        leader_timestamps = [sample.timestamp_seconds for sample in leader_positions]
        cumulative_distance = self._cumulative_distance(leader_positions)
        total_distance = max(1e-12, cumulative_distance[-1])

        for lidar_sample in lidar_samples:
            progress = self._progress_at_timestamp(lidar_sample.timestamp_seconds, leader_positions, leader_timestamps, cumulative_distance, total_distance)
            bin_index = min(point_count - 1, max(0, int(round(progress * (point_count - 1)))))
            bins[bin_index].append(lidar_sample.sector_ranges_meters)

        averaged: list[tuple[float, ...] | None] = []
        for bin_samples in bins:
            if not bin_samples:
                averaged.append(None)
                continue
            sector_values: list[float] = []
            for sector_index in range(sector_count):
                values = [sample[sector_index] for sample in bin_samples if sector_index < len(sample)]
                sector_values.append(sum(values) / len(values) if values else self._settings.default_lidar_max_range_meters)
            averaged.append(tuple(sector_values))

        return tuple(self._fill_missing_lidar(averaged))

    def _progress_at_timestamp(self, timestamp: float, samples: tuple[PositionSample, ...], timestamps: list[float], cumulative_distance: list[float], total_distance: float) -> float:
        upper_index = bisect_left(timestamps, timestamp)
        if upper_index <= 0:
            return 0.0
        if upper_index >= len(samples):
            return 1.0

        lower_index = upper_index - 1
        lower_timestamp = timestamps[lower_index]
        upper_timestamp = timestamps[upper_index]
        time_span = max(1e-12, upper_timestamp - lower_timestamp)
        fraction = min(1.0, max(0.0, (timestamp - lower_timestamp) / time_span))
        distance = cumulative_distance[lower_index] + (cumulative_distance[upper_index] - cumulative_distance[lower_index]) * fraction
        return min(1.0, max(0.0, distance / total_distance))

    def _fill_missing_lidar(self, values: list[tuple[float, ...] | None]) -> list[tuple[float, ...]]:
        available_indices = [index for index, value in enumerate(values) if value is not None]
        if not available_indices:
            raise ValueError("No LiDAR features are available for resampling.")

        result: list[tuple[float, ...]] = []
        for index, value in enumerate(values):
            if value is not None:
                result.append(value)
                continue

            insertion = bisect_left(available_indices, index)
            if insertion <= 0:
                nearest = values[available_indices[0]]
                result.append(nearest if nearest is not None else tuple())
                continue
            if insertion >= len(available_indices):
                nearest = values[available_indices[-1]]
                result.append(nearest if nearest is not None else tuple())
                continue

            left_index = available_indices[insertion - 1]
            right_index = available_indices[insertion]
            left_value = values[left_index]
            right_value = values[right_index]
            if left_value is None or right_value is None:
                result.append(left_value or right_value or tuple())
                continue
            fraction = (index - left_index) / max(1, right_index - left_index)
            interpolated = tuple(
                left_component + (right_component - left_component) * fraction
                for left_component, right_component in zip(left_value, right_value)
            )
            result.append(interpolated)
        return result

    def _cumulative_distance(self, samples: tuple[PositionSample, ...]) -> list[float]:
        cumulative = [0.0]
        for index in range(1, len(samples)):
            cumulative.append(cumulative[-1] + samples[index].distance_to(samples[index - 1]))
        return cumulative
