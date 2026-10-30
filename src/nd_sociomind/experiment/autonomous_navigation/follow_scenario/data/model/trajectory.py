from dataclasses import dataclass, field

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample


@dataclass
class Trajectory:
    samples: list[PositionSample] = field(default_factory=list)

    def append(self, sample: PositionSample) -> None:
        self.samples.append(sample)

    def length(self) -> int:
        return len(self.samples)

    def is_empty(self) -> bool:
        return len(self.samples) == 0
