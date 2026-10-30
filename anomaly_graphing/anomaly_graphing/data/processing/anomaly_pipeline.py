from typing import Callable

from anomaly_graphing.config.anomaly_graph_settings import AnomalyGraphSettings
from anomaly_graphing.data.model.anomaly_series_bundle import AnomalySeriesBundle
from anomaly_graphing.data.processing.anomaly_ground_truth_factory import AnomalyGroundTruthFactory
from anomaly_graphing.data.processing.binary_classification_evaluator import BinaryClassificationEvaluator
from anomaly_graphing.data.processing.random_threshold_generator import RandomThresholdGenerator
from anomaly_graphing.data.processing.synthetic_anomaly_generator import SyntheticAnomalyGenerator
from anomaly_graphing.data.reader.ros_yaml_time_reader import RosYamlTimeReader


class AnomalyPipeline:
    def __init__(self, settings: AnomalyGraphSettings):
        self._settings = settings
        self._generator = SyntheticAnomalyGenerator(settings)
        self._time_reader = RosYamlTimeReader()
        self._threshold_generator = RandomThresholdGenerator(settings)
        self._ground_truth_factory = AnomalyGroundTruthFactory(settings)
        self._classification_evaluator = BinaryClassificationEvaluator()

    def calculate(self, progress_callback: Callable[[str, int], None] | None = None) -> AnomalySeriesBundle:
        self._report(progress_callback, "Reading one-lap time range from ROS YAML", 10)
        lap_duration_seconds = self._time_reader.read_duration_seconds(self._settings.one_round_yaml_path())

        self._report(progress_callback, "Choosing thresholds", 25)
        thresholds = self._threshold_generator.generate()

        self._report(progress_callback, "Generating Gaussian Wasserstein anomaly curves", 45)
        generated = self._generator.generate(lap_duration_seconds, thresholds)

        self._report(progress_callback, "Evaluating binary anomaly classification", 75)
        gps_metrics = self._classification_evaluator.evaluate(
            generated.gps_wasserstein_distance,
            thresholds.gps,
            self._ground_truth_factory.gps(),
        )
        lidar_metrics = self._classification_evaluator.evaluate(
            generated.lidar_wasserstein_distance,
            thresholds.lidar,
            self._ground_truth_factory.lidar(),
        )
        fused_metrics = self._classification_evaluator.evaluate(
            generated.fused_wasserstein_distance,
            thresholds.fused,
            self._ground_truth_factory.fused(),
        )

        self._report(progress_callback, "Ready", 100)
        return AnomalySeriesBundle(
            generated.time_seconds,
            generated.gps_wasserstein_distance,
            generated.lidar_wasserstein_distance,
            generated.fused_wasserstein_distance,
            thresholds,
            gps_metrics,
            lidar_metrics,
            fused_metrics,
        )

    def _report(self, progress_callback: Callable[[str, int], None] | None, message: str, percent: int) -> None:
        if progress_callback is not None:
            progress_callback(message, percent)
