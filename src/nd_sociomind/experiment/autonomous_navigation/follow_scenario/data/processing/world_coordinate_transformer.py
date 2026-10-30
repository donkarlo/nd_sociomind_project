import math

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory import Trajectory
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.corridor_side import CorridorSide
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.narrow_section_estimator import NarrowSectionEstimator


class WorldCoordinateTransformer:
    def __init__(self, settings: ScenarioSettings):
        self._settings = settings
        self._narrow_section_estimator = NarrowSectionEstimator(settings)
        self.rotation_radians = 0.0

    def transform(self, trajectory_pair: TrajectoryPair) -> TrajectoryPair:
        source_center_x, source_center_y = self._robust_center(trajectory_pair.leader)
        source_axis_angle = self._estimate_axis_angle(trajectory_pair.leader)
        self.rotation_radians = -source_axis_angle

        rotated_leader = self._rotate_trajectory(trajectory_pair.leader, source_center_x, source_center_y, -source_axis_angle)
        rotated_follower = self._rotate_trajectory(trajectory_pair.follower, source_center_x, source_center_y, -source_axis_angle)

        rotated_center_x, rotated_center_y = self._robust_center(rotated_leader)
        target_center_x = self._settings.outer_width_meters * 0.5
        target_center_y = self._settings.outer_height_meters * 0.5
        offset_x = target_center_x - rotated_center_x
        offset_y = target_center_y - rotated_center_y

        translated_leader = self._translate_trajectory(rotated_leader, offset_x, offset_y)
        translated_follower = self._translate_trajectory(rotated_follower, offset_x, offset_y)
        translated_pair = TrajectoryPair(translated_leader, translated_follower)

        narrow_section = self._narrow_section_estimator.estimate(translated_pair, target_center_x, target_center_y)
        quarter_turn_angle = self._rotation_to_right_side(narrow_section.side)
        self.rotation_radians += quarter_turn_angle
        if abs(quarter_turn_angle) < 1e-12:
            return translated_pair

        aligned_leader = self._rotate_trajectory(translated_leader, target_center_x, target_center_y, quarter_turn_angle)
        aligned_follower = self._rotate_trajectory(translated_follower, target_center_x, target_center_y, quarter_turn_angle)
        return TrajectoryPair(aligned_leader, aligned_follower)

    def _rotation_to_right_side(self, side: CorridorSide) -> float:
        if side == CorridorSide.RIGHT:
            return 0.0
        if side == CorridorSide.LEFT:
            return math.pi
        if side == CorridorSide.TOP:
            return math.pi * 0.5
        return -math.pi * 0.5

    def _estimate_axis_angle(self, trajectory: Trajectory) -> float:
        samples = trajectory.samples
        if len(samples) < 4:
            return 0.0

        stride = max(1, len(samples) // 2000)
        weighted_sine = 0.0
        weighted_cosine = 0.0
        total_weight = 0.0

        for sample_index in range(stride, len(samples), stride):
            previous_sample = samples[sample_index - stride]
            current_sample = samples[sample_index]
            delta_x = current_sample.x_meters - previous_sample.x_meters
            delta_y = current_sample.y_meters - previous_sample.y_meters
            segment_length = math.hypot(delta_x, delta_y)
            if segment_length < self._settings.minimum_sample_spacing_meters * 2.0:
                continue

            angle = math.atan2(delta_y, delta_x)
            weighted_sine += math.sin(4.0 * angle) * segment_length
            weighted_cosine += math.cos(4.0 * angle) * segment_length
            total_weight += segment_length

        if total_weight <= 0.0:
            return 0.0

        concentration = math.hypot(weighted_sine, weighted_cosine) / total_weight
        if concentration < 0.20:
            return 0.0

        return math.atan2(weighted_sine, weighted_cosine) / 4.0

    def _robust_center(self, trajectory: Trajectory) -> tuple[float, float]:
        x_values = sorted(sample.x_meters for sample in trajectory.samples)
        y_values = sorted(sample.y_meters for sample in trajectory.samples)
        if not x_values or not y_values:
            return 0.0, 0.0

        lower_x = self._percentile(x_values, 0.03)
        upper_x = self._percentile(x_values, 0.97)
        lower_y = self._percentile(y_values, 0.03)
        upper_y = self._percentile(y_values, 0.97)
        center_x = (lower_x + upper_x) * 0.5
        center_y = (lower_y + upper_y) * 0.5
        return center_x, center_y

    def _percentile(self, sorted_values: list[float], fraction: float) -> float:
        if len(sorted_values) == 1:
            return sorted_values[0]

        position = fraction * float(len(sorted_values) - 1)
        lower_index = int(math.floor(position))
        upper_index = int(math.ceil(position))
        if lower_index == upper_index:
            return sorted_values[lower_index]

        interpolation = position - float(lower_index)
        lower_value = sorted_values[lower_index]
        upper_value = sorted_values[upper_index]
        return lower_value + (upper_value - lower_value) * interpolation

    def _rotate_trajectory(self, trajectory: Trajectory, center_x: float, center_y: float, angle_radians: float) -> Trajectory:
        transformed = Trajectory()
        cosine = math.cos(angle_radians)
        sine = math.sin(angle_radians)

        for sample in trajectory.samples:
            delta_x = sample.x_meters - center_x
            delta_y = sample.y_meters - center_y
            rotated_x = center_x + delta_x * cosine - delta_y * sine
            rotated_y = center_y + delta_x * sine + delta_y * cosine
            transformed.append(PositionSample(sample.timestamp_seconds, rotated_x, rotated_y))

        return transformed

    def _translate_trajectory(self, trajectory: Trajectory, offset_x: float, offset_y: float) -> Trajectory:
        transformed = Trajectory()

        for sample in trajectory.samples:
            transformed.append(PositionSample(sample.timestamp_seconds, sample.x_meters + offset_x, sample.y_meters + offset_y))

        return transformed
