import math

from anomaly_graphing.data.model.resampled_scenario import ResampledScenario


class ScenarioAligner:
    def align_follow_to_normal(self, normal: ResampledScenario, follow: ResampledScenario) -> tuple[ResampledScenario, int]:
        normal_center = self._center(normal)
        follow_center = self._center(follow)
        centered_normal = self._translate(normal, -normal_center[0], -normal_center[1])
        centered_follow = self._translate(follow, -follow_center[0], -follow_center[1])

        best_error = math.inf
        best_rotation = 0.0
        best_shift = 0
        point_count = len(centered_normal.leader_positions)
        shift_step = max(1, point_count // 300)

        for quarter_turn in range(4):
            rotation = quarter_turn * math.pi * 0.5
            rotated_follow = self._rotate(centered_follow, rotation)
            for shift in range(0, point_count, shift_step):
                error = self._alignment_error(centered_normal, rotated_follow, shift)
                if error < best_error:
                    best_error = error
                    best_rotation = rotation
                    best_shift = shift

        rotated_follow = self._rotate(centered_follow, best_rotation)
        refined_shift = best_shift
        for shift_offset in range(-shift_step, shift_step + 1):
            candidate_shift = (best_shift + shift_offset) % point_count
            error = self._alignment_error(centered_normal, rotated_follow, candidate_shift)
            if error < best_error:
                best_error = error
                refined_shift = candidate_shift

        aligned_follow = self._circular_shift(rotated_follow, refined_shift)
        return aligned_follow, refined_shift

    def _center(self, scenario: ResampledScenario) -> tuple[float, float]:
        points = list(scenario.leader_positions) + list(scenario.follower_positions)
        center_x = sum(point[0] for point in points) / len(points)
        center_y = sum(point[1] for point in points) / len(points)
        return center_x, center_y

    def _translate(self, scenario: ResampledScenario, offset_x: float, offset_y: float) -> ResampledScenario:
        leader = tuple((x_value + offset_x, y_value + offset_y) for x_value, y_value in scenario.leader_positions)
        follower = tuple((x_value + offset_x, y_value + offset_y) for x_value, y_value in scenario.follower_positions)
        return ResampledScenario(leader, follower, scenario.leader_lidar, scenario.follower_lidar)

    def _rotate(self, scenario: ResampledScenario, angle_radians: float) -> ResampledScenario:
        cosine = math.cos(angle_radians)
        sine = math.sin(angle_radians)

        def rotate_points(points: tuple[tuple[float, float], ...]) -> tuple[tuple[float, float], ...]:
            return tuple(
                (x_value * cosine - y_value * sine, x_value * sine + y_value * cosine)
                for x_value, y_value in points
            )

        return ResampledScenario(
            rotate_points(scenario.leader_positions),
            rotate_points(scenario.follower_positions),
            scenario.leader_lidar,
            scenario.follower_lidar,
        )

    def _circular_shift(self, scenario: ResampledScenario, shift: int) -> ResampledScenario:
        return ResampledScenario(
            self._shift_tuple(scenario.leader_positions, shift),
            self._shift_tuple(scenario.follower_positions, shift),
            self._shift_tuple(scenario.leader_lidar, shift),
            self._shift_tuple(scenario.follower_lidar, shift),
        )

    def _shift_tuple(self, values: tuple, shift: int) -> tuple:
        if not values:
            return values
        normalized_shift = shift % len(values)
        return values[normalized_shift:] + values[:normalized_shift]

    def _alignment_error(self, normal: ResampledScenario, follow: ResampledScenario, shift: int) -> float:
        point_count = len(normal.leader_positions)
        errors: list[float] = []
        sampling_step = max(1, point_count // 120)
        for index in range(0, point_count, sampling_step):
            shifted_index = (index + shift) % point_count
            leader_error = self._distance(normal.leader_positions[index], follow.leader_positions[shifted_index])
            follower_error = self._distance(normal.follower_positions[index], follow.follower_positions[shifted_index])
            errors.append(leader_error + follower_error)

        errors.sort()
        retained_count = max(1, int(len(errors) * 0.75))
        return sum(errors[:retained_count]) / retained_count

    def _distance(self, first: tuple[float, float], second: tuple[float, float]) -> float:
        delta_x = first[0] - second[0]
        delta_y = first[1] - second[1]
        return math.hypot(delta_x, delta_y)
