import math

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings


class ArenaSafetyModel:
    def __init__(self, settings: ScenarioSettings):
        self._settings = settings

    def constrain_motion(self, start_x_meters: float, start_y_meters: float, desired_x_meters: float, desired_y_meters: float, other_x_meters: float, other_y_meters: float) -> tuple[float, float]:
        delta_x = desired_x_meters - start_x_meters
        delta_y = desired_y_meters - start_y_meters
        distance = math.hypot(delta_x, delta_y)
        if distance <= 1e-9:
            return start_x_meters, start_y_meters

        step_size = max(0.01, self._settings.interaction_motion_step_meters)
        step_count = max(1, min(500, int(math.ceil(distance / step_size))))
        current_x = start_x_meters
        current_y = start_y_meters

        for step_index in range(1, step_count + 1):
            fraction = float(step_index) / float(step_count)
            candidate_x = start_x_meters + delta_x * fraction
            candidate_y = start_y_meters + delta_y * fraction

            if self.is_safe_position(candidate_x, candidate_y, other_x_meters, other_y_meters):
                current_x = candidate_x
                current_y = candidate_y
                continue

            horizontal_safe = self.is_safe_position(candidate_x, current_y, other_x_meters, other_y_meters)
            vertical_safe = self.is_safe_position(current_x, candidate_y, other_x_meters, other_y_meters)

            if horizontal_safe and vertical_safe:
                horizontal_progress = abs(candidate_x - current_x)
                vertical_progress = abs(candidate_y - current_y)
                if horizontal_progress >= vertical_progress:
                    current_x = candidate_x
                else:
                    current_y = candidate_y
                continue

            if horizontal_safe:
                current_x = candidate_x
                continue

            if vertical_safe:
                current_y = candidate_y
                continue

            break

        return current_x, current_y

    def is_safe_position(self, x_meters: float, y_meters: float, other_x_meters: float, other_y_meters: float) -> bool:
        if not self._inside_outer_boundary(x_meters, y_meters):
            return False
        if self._inside_expanded_inner_square(x_meters, y_meters):
            return False
        if self._too_close_to_narrowing_walls(x_meters, y_meters):
            return False
        if self._too_close_to_other_drone(x_meters, y_meters, other_x_meters, other_y_meters):
            return False
        return True

    def _inside_outer_boundary(self, x_meters: float, y_meters: float) -> bool:
        center_clearance = self._wall_center_clearance()
        if x_meters < center_clearance:
            return False
        if y_meters < center_clearance:
            return False
        if x_meters > self._settings.outer_width_meters - center_clearance:
            return False
        if y_meters > self._settings.outer_height_meters - center_clearance:
            return False
        return True

    def _inside_expanded_inner_square(self, x_meters: float, y_meters: float) -> bool:
        center_x = self._settings.outer_width_meters * 0.5
        center_y = self._settings.outer_height_meters * 0.5
        half_width = self._settings.inner_width_meters * 0.5 + self._wall_center_clearance()
        half_height = self._settings.inner_height_meters * 0.5 + self._wall_center_clearance()
        return abs(x_meters - center_x) < half_width and abs(y_meters - center_y) < half_height

    def _too_close_to_narrowing_walls(self, x_meters: float, y_meters: float) -> bool:
        minimum_distance = self._wall_center_clearance()
        for start_x, start_y, end_x, end_y in self._narrowing_wall_segments():
            distance = self._point_to_segment_distance(x_meters, y_meters, start_x, start_y, end_x, end_y)
            if distance < minimum_distance:
                return True
        return False

    def _too_close_to_other_drone(self, x_meters: float, y_meters: float, other_x_meters: float, other_y_meters: float) -> bool:
        minimum_distance = self._settings.drone_outer_radius_meters * 2.0 + self._settings.minimum_inter_drone_clearance_meters
        return math.hypot(x_meters - other_x_meters, y_meters - other_y_meters) < minimum_distance

    def _wall_center_clearance(self) -> float:
        return self._settings.drone_outer_radius_meters + self._settings.minimum_wall_clearance_meters + self._settings.wall_thickness_meters * 0.5

    def _narrowing_wall_segments(self) -> list[tuple[float, float, float, float]]:
        center_y = self._settings.outer_height_meters * 0.5
        half_span = self._settings.narrow_obstacle_length_meters * 0.5
        top_y = center_y - half_span
        bottom_y = center_y + half_span
        inner_right_x = self._settings.outer_width_meters * 0.5 + self._settings.inner_width_meters * 0.5
        inner_cap_x = inner_right_x + self._settings.narrow_obstacle_depth_meters
        outer_right_x = self._settings.outer_width_meters
        outer_cap_x = outer_right_x - self._settings.narrow_obstacle_depth_meters

        return [
            (inner_right_x, top_y, inner_cap_x, top_y),
            (inner_right_x, bottom_y, inner_cap_x, bottom_y),
            (inner_cap_x, top_y, inner_cap_x, bottom_y),
            (outer_cap_x, top_y, outer_right_x, top_y),
            (outer_cap_x, bottom_y, outer_right_x, bottom_y),
            (outer_cap_x, top_y, outer_cap_x, bottom_y),
        ]

    def _point_to_segment_distance(self, point_x: float, point_y: float, start_x: float, start_y: float, end_x: float, end_y: float) -> float:
        segment_x = end_x - start_x
        segment_y = end_y - start_y
        segment_length_squared = segment_x * segment_x + segment_y * segment_y
        if segment_length_squared <= 1e-12:
            return math.hypot(point_x - start_x, point_y - start_y)

        projection = ((point_x - start_x) * segment_x + (point_y - start_y) * segment_y) / segment_length_squared
        projection = max(0.0, min(1.0, projection))
        closest_x = start_x + projection * segment_x
        closest_y = start_y + projection * segment_y
        return math.hypot(point_x - closest_x, point_y - closest_y)
