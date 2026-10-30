from dataclasses import dataclass


@dataclass(frozen=True)
class PositionSample:
    timestamp_seconds: float
    x_meters: float
    y_meters: float

    def distance_to(self, other: "PositionSample") -> float:
        delta_x = self.x_meters - other.x_meters
        delta_y = self.y_meters - other.y_meters
        return (delta_x * delta_x + delta_y * delta_y) ** 0.5
