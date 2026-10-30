from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample


class LapDetector:
    def __init__(self, settings: ScenarioSettings):
        self._settings = settings
        self._start_sample: PositionSample | None = None
        self._previous_sample: PositionSample | None = None
        self._distance_traveled_meters = 0.0
        self._departed_start_area = False

    def reset(self) -> None:
        self._start_sample = None
        self._previous_sample = None
        self._distance_traveled_meters = 0.0
        self._departed_start_area = False

    def observe(self, sample: PositionSample) -> bool:
        if self._start_sample is None:
            self._start_sample = sample
            self._previous_sample = sample
            return False

        if self._previous_sample is None:
            self._previous_sample = sample
            return False

        step_distance = sample.distance_to(self._previous_sample)
        if step_distance < self._settings.minimum_sample_spacing_meters:
            return False

        self._distance_traveled_meters += step_distance
        self._previous_sample = sample
        distance_from_start = sample.distance_to(self._start_sample)

        if distance_from_start >= self._settings.departure_radius_meters:
            self._departed_start_area = True

        if not self._departed_start_area:
            return False

        if self._distance_traveled_meters < self._settings.minimum_lap_distance_meters:
            return False

        return distance_from_start <= self._settings.lap_closure_radius_meters
