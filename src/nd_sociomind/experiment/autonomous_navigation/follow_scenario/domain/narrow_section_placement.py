from dataclasses import dataclass

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.corridor_side import CorridorSide


@dataclass(frozen=True)
class NarrowSectionPlacement:
    side: CorridorSide
    center_meters: float
