import math

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.corridor_side import CorridorSide
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.narrow_section_placement import NarrowSectionPlacement


class NarrowSectionEstimator:
    def __init__(self, settings: ScenarioSettings):
        self._settings = settings

    def estimate(self, trajectory_pair: TrajectoryPair, center_x: float, center_y: float) -> NarrowSectionPlacement:
        frame_count = min(trajectory_pair.leader.length(), trajectory_pair.follower.length())
        if frame_count < 6:
            return self._centered_placement(CorridorSide.RIGHT)

        scores = {
            CorridorSide.LEFT: 0.0,
            CorridorSide.RIGHT: 0.0,
            CorridorSide.TOP: 0.0,
            CorridorSide.BOTTOM: 0.0,
        }

        for frame_index in range(2, frame_count - 2):
            leader_previous = trajectory_pair.leader.samples[frame_index - 1]
            leader_current = trajectory_pair.leader.samples[frame_index]
            leader_next = trajectory_pair.leader.samples[frame_index + 1]
            follower_current = trajectory_pair.follower.samples[frame_index]

            velocity_x = leader_next.x_meters - leader_previous.x_meters
            velocity_y = leader_next.y_meters - leader_previous.y_meters
            leader_from_follower_x = leader_current.x_meters - follower_current.x_meters
            leader_from_follower_y = leader_current.y_meters - follower_current.y_meters
            velocity_norm = math.hypot(velocity_x, velocity_y)
            separation = math.hypot(leader_from_follower_x, leader_from_follower_y)

            if velocity_norm < 0.01 or separation < 0.01:
                continue

            following_alignment = (velocity_x * leader_from_follower_x + velocity_y * leader_from_follower_y) / (velocity_norm * separation)
            if following_alignment < 0.68:
                continue

            if separation > self._settings.desired_follow_distance_meters * 1.8:
                continue

            heading_stability = self._heading_stability(trajectory_pair, frame_index)
            if heading_stability < 0.78:
                continue

            midpoint_x = (leader_current.x_meters + follower_current.x_meters) * 0.5
            midpoint_y = (leader_current.y_meters + follower_current.y_meters) * 0.5
            side = self._side_from_position(midpoint_x, midpoint_y, center_x, center_y)
            distance_error = abs(separation - self._settings.desired_follow_distance_meters)
            proximity_weight = 1.0 / (1.0 + distance_error)
            scores[side] += following_alignment * heading_stability * proximity_weight

        best_side = max(scores, key=scores.get)
        if scores[best_side] <= 0.0:
            return self._centered_placement(CorridorSide.RIGHT)
        return self._centered_placement(best_side)

    def _centered_placement(self, side: CorridorSide) -> NarrowSectionPlacement:
        if side in {CorridorSide.LEFT, CorridorSide.RIGHT}:
            center_meters = self._settings.outer_height_meters * 0.5
        else:
            center_meters = self._settings.outer_width_meters * 0.5
        return NarrowSectionPlacement(side, center_meters)

    def _heading_stability(self, trajectory_pair: TrajectoryPair, frame_index: int) -> float:
        first = trajectory_pair.leader.samples[frame_index - 1]
        second = trajectory_pair.leader.samples[frame_index]
        third = trajectory_pair.leader.samples[frame_index + 1]
        first_velocity_x = second.x_meters - first.x_meters
        first_velocity_y = second.y_meters - first.y_meters
        second_velocity_x = third.x_meters - second.x_meters
        second_velocity_y = third.y_meters - second.y_meters
        first_norm = math.hypot(first_velocity_x, first_velocity_y)
        second_norm = math.hypot(second_velocity_x, second_velocity_y)
        if first_norm < 0.01 or second_norm < 0.01:
            return 0.0
        return (first_velocity_x * second_velocity_x + first_velocity_y * second_velocity_y) / (first_norm * second_norm)

    def _side_from_position(self, x_meters: float, y_meters: float, center_x: float, center_y: float) -> CorridorSide:
        delta_x = x_meters - center_x
        delta_y = y_meters - center_y
        if abs(delta_x) >= abs(delta_y):
            if delta_x >= 0.0:
                return CorridorSide.RIGHT
            return CorridorSide.LEFT
        if delta_y >= 0.0:
            return CorridorSide.BOTTOM
        return CorridorSide.TOP
