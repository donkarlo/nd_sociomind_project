import time

from PySide6.QtCore import QObject, Signal, Slot

from anomaly_graphing.data.processing.anomaly_pipeline import AnomalyPipeline


class AnomalyLoaderWorker(QObject):
    completed = Signal(object)
    failed = Signal(str)
    progress_changed = Signal(str, int)

    def __init__(self, pipeline: AnomalyPipeline):
        super().__init__()
        self._pipeline = pipeline
        self._progress_stage_delay_seconds = 0.35

    @Slot()
    def run(self) -> None:
        try:
            result = self._pipeline.calculate(self._report_progress)
            self.completed.emit(result)
        except Exception as exception:
            self.failed.emit(str(exception))

    def _report_progress(self, message: str, percent: int) -> None:
        self.progress_changed.emit(message, percent)
        if percent < 100:
            time.sleep(self._progress_stage_delay_seconds)
