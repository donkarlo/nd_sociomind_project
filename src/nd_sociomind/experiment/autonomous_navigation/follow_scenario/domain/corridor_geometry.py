from dataclasses import dataclass

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.narrow_section_placement import NarrowSectionPlacement


@dataclass(frozen=True)
class CorridorGeometry:
    inner_center_x_meters: float
    inner_center_y_meters: float
    narrow_section: NarrowSectionPlacement
