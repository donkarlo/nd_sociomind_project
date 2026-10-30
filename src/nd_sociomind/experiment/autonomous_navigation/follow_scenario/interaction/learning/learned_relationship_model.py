import math

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory import Trajectory
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.drone_role import DroneRole


class LearnedRelationshipModel:
    def __init__(self, trajectory_pair: TrajectoryPair, settings: ScenarioSettings):
        self._trajectory_pair = trajectory_pair
        self._settings = settings

    def coordinated_target(self, frame_index: int, dragged_role: DroneRole, dragged_x_meters: float, dragged_y_meters: float) -> tuple[float, float]:
        offset_x, offset_y = self._smoothed_relative_offset(frame_index, dragged_role)
        return dragged_x_meters + offset_x, dragged_y_meters + offset_y

    def learned_response_speed(self, frame_index: int, moving_role: DroneRole, driver_role: DroneRole) -> float:
        local_speed = self._smoothed_local_speed(frame_index, moving_role)
        coupling_gain = self._smoothed_motion_coupling(frame_index, driver_role)
        response_speed = local_speed * coupling_gain
        response_speed = max(response_speed, self._settings.interaction_minimum_response_speed_meters_per_second)
        response_speed = min(response_speed, self._settings.interaction_maximum_response_speed_meters_per_second)
        return response_speed

    def nominal_position(self, role: DroneRole, frame_index: int) -> tuple[float, float]:
        trajectory = self._trajectory(role)
        safe_index = self._safe_frame_index(frame_index)
        sample = trajectory.samples[safe_index]
        return sample.x_meters, sample.y_meters

    def recovery_frame(self, dragged_role: DroneRole, dragged_x_meters: float, dragged_y_meters: float, other_x_meters: float, other_y_meters: float) -> int:
        dragged_trajectory = self._trajectory(dragged_role)
        other_trajectory = self._trajectory(dragged_role.other())
        frame_count = min(dragged_trajectory.length(), other_trajectory.length())
        if frame_count <= 0:
            return 0

        best_index = 0
        best_score = float("inf")
        for frame_index in range(frame_count):
            dragged_sample = dragged_trajectory.samples[frame_index]
            other_sample = other_trajectory.samples[frame_index]
            dragged_distance_squared = self._distance_squared(dragged_sample.x_meters, dragged_sample.y_meters, dragged_x_meters, dragged_y_meters)
            other_distance_squared = self._distance_squared(other_sample.x_meters, other_sample.y_meters, other_x_meters, other_y_meters)
            score = dragged_distance_squared + other_distance_squared * 0.35
            if score < best_score:
                best_score = score
                best_index = frame_index
        return best_index

    def _smoothed_relative_offset(self, frame_index: int, dragged_role: DroneRole) -> tuple[float, float]:
        dragged_trajectory = self._trajectory(dragged_role)
        other_trajectory = self._trajectory(dragged_role.other())
        start_index, end_index = self._window_bounds(frame_index)
        weighted_offset_x = 0.0
        weighted_offset_y = 0.0
        total_weight = 0.0

        for sample_index in range(start_index, end_index + 1):
            dragged_sample = dragged_trajectory.samples[sample_index]
            other_sample = other_trajectory.samples[sample_index]
            weight = self._window_weight(frame_index, sample_index)
            weighted_offset_x += (other_sample.x_meters - dragged_sample.x_meters) * weight
            weighted_offset_y += (other_sample.y_meters - dragged_sample.y_meters) * weight
            total_weight += weight

        if total_weight <= 0.0:
            dragged_sample = dragged_trajectory.samples[self._safe_frame_index(frame_index)]
            other_sample = other_trajectory.samples[self._safe_frame_index(frame_index)]
            return other_sample.x_meters - dragged_sample.x_meters, other_sample.y_meters - dragged_sample.y_meters

        return weighted_offset_x / total_weight, weighted_offset_y / total_weight

    def _smoothed_local_speed(self, frame_index: int, role: DroneRole) -> float:
        trajectory = self._trajectory(role)
        start_index, end_index = self._window_bounds(frame_index)
        weighted_speed = 0.0
        total_weight = 0.0

        for sample_index in range(max(start_index + 1, 1), end_index + 1):
            previous_sample = trajectory.samples[sample_index - 1]
            current_sample = trajectory.samples[sample_index]
            delta_time = current_sample.timestamp_seconds - previous_sample.timestamp_seconds
            if delta_time <= 0.0001 or delta_time > 1.0:
                delta_time = 0.02
            distance = current_sample.distance_to(previous_sample)
            speed = distance / delta_time
            weight = self._window_weight(frame_index, sample_index)
            weighted_speed += speed * weight
            total_weight += weight

        if total_weight <= 0.0:
            return self._settings.interaction_minimum_response_speed_meters_per_second
        return weighted_speed / total_weight

    def _smoothed_motion_coupling(self, frame_index: int, driver_role: DroneRole) -> float:
        driver_trajectory = self._trajectory(driver_role)
        other_trajectory = self._trajectory(driver_role.other())
        start_index, end_index = self._window_bounds(frame_index)
        weighted_gain = 0.0
        total_weight = 0.0

        for sample_index in range(max(start_index + 1, 1), end_index + 1):
            driver_previous = driver_trajectory.samples[sample_index - 1]
            driver_current = driver_trajectory.samples[sample_index]
            other_previous = other_trajectory.samples[sample_index - 1]
            other_current = other_trajectory.samples[sample_index]

            driver_delta_x = driver_current.x_meters - driver_previous.x_meters
            driver_delta_y = driver_current.y_meters - driver_previous.y_meters
            other_delta_x = other_current.x_meters - other_previous.x_meters
            other_delta_y = other_current.y_meters - other_previous.y_meters
            driver_norm_squared = driver_delta_x * driver_delta_x + driver_delta_y * driver_delta_y
            if driver_norm_squared < 1e-8:
                continue

            gain = (driver_delta_x * other_delta_x + driver_delta_y * other_delta_y) / driver_norm_squared
            gain = max(0.35, min(gain, 1.35))
            weight = self._window_weight(frame_index, sample_index)
            weighted_gain += gain * weight
            total_weight += weight

        if total_weight <= 0.0:
            return 1.0
        return weighted_gain / total_weight

    def _window_bounds(self, frame_index: int) -> tuple[int, int]:
        frame_count = min(self._trajectory_pair.leader.length(), self._trajectory_pair.follower.length())
        if frame_count <= 0:
            return 0, 0
        radius = self._settings.interaction_learning_window_frames
        safe_index = max(0, min(frame_index, frame_count - 1))
        start_index = max(0, safe_index - radius)
        end_index = min(frame_count - 1, safe_index + radius)
        return start_index, end_index

    def _window_weight(self, center_index: int, sample_index: int) -> float:
        distance = abs(sample_index - center_index)
        radius = max(1, self._settings.interaction_learning_window_frames)
        return 1.0 + float(radius - min(distance, radius))

    def _safe_frame_index(self, frame_index: int) -> int:
        frame_count = min(self._trajectory_pair.leader.length(), self._trajectory_pair.follower.length())
        if frame_count <= 0:
            return 0
        return max(0, min(frame_index, frame_count - 1))

    def _trajectory(self, role: DroneRole) -> Trajectory:
        if role == DroneRole.LEADER:
            return self._trajectory_pair.leader
        return self._trajectory_pair.follower

    def _distance_squared(self, first_x: float, first_y: float, second_x: float, second_y: float) -> float:
        delta_x = first_x - second_x
        delta_y = first_y - second_y
        return delta_x * delta_x + delta_y * delta_y
