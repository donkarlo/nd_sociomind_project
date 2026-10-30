from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.corridor_geometry import CorridorGeometry
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.corridor_side import CorridorSide
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.narrow_section_placement import NarrowSectionPlacement


class CorridorGeometryEstimator:
    def __init__(self, settings: ScenarioSettings):
        self._settings = settings

    def estimate(self, trajectory_pair: TrajectoryPair) -> CorridorGeometry:
        center_x = self._settings.outer_width_meters * 0.5
        center_y = self._settings.outer_height_meters * 0.5
        narrow_section = NarrowSectionPlacement(CorridorSide.RIGHT, center_y)
        return CorridorGeometry(center_x, center_y, narrow_section)
