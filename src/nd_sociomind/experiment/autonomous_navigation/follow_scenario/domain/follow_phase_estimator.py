import math

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.flight_phase import FlightPhase


class FollowPhaseEstimator:
    def estimate(self, previous_leader: PositionSample, leader: PositionSample, follower: PositionSample) -> FlightPhase:
        velocity_x = leader.x_meters - previous_leader.x_meters
        velocity_y = leader.y_meters - previous_leader.y_meters
        relative_x = follower.x_meters - leader.x_meters
        relative_y = follower.y_meters - leader.y_meters
        velocity_norm = math.hypot(velocity_x, velocity_y)
        relative_norm = math.hypot(relative_x, relative_y)

        if velocity_norm < 0.001 or relative_norm < 0.001:
            return FlightPhase.NORMAL_SIDE_BY_SIDE

        alignment = abs((velocity_x * relative_x + velocity_y * relative_y) / (velocity_norm * relative_norm))
        if alignment >= 0.72:
            return FlightPhase.NARROW_FOLLOWING
        return FlightPhase.NORMAL_SIDE_BY_SIDE
