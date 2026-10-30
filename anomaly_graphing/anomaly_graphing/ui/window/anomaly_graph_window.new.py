from PySide6.QtCore import QThread
from PySide6.QtWidgets import QLabel, QMainWindow, QMessageBox, QProgressBar, QVBoxLayout, QWidget

from anomaly_graphing.application.anomaly_loader_worker import AnomalyLoaderWorker
from anomaly_graphing.config.anomaly_graph_settings import AnomalyGraphSettings
from anomaly_graphing.data.model.anomaly_series_bundle import AnomalySeriesBundle
from anomaly_graphing.data.processing.anomaly_pipeline import AnomalyPipeline
from anomaly_graphing.ui.widget.anomaly_plot_widget import AnomalyPlotWidget


class AnomalyGraphWindow(QMainWindow):
    def __init__(self, settings: AnomalyGraphSettings):
        super().__init__()
        self._settings = settings
        self._thread: QThread | None = None
        self._worker: AnomalyLoaderWorker | None = None
        self._status_label = QLabel(
            "Schematic one-lap comparison: normal vs follow | synthetic Gaussian distributions"
        )
        self._progress_bar = QProgressBar()
        self._gps_plot = AnomalyPlotWidget(
            "GPS — centered 10% anomaly interval",
            settings.wasserstein_threshold,
            settings.gps_anomaly_fraction,
        )
        self._lidar_plot = AnomalyPlotWidget(
            "LiDAR — centered 25% anomaly interval",
            settings.wasserstein_threshold,
            settings.lidar_anomaly_fraction,
        )
        self._fused_plot = AnomalyPlotWidget(
            "Fused GPS + LiDAR — centered 25% anomaly interval; central 10% amplified",
            settings.wasserstein_threshold,
            settings.fused_anomaly_fraction,
        )
        self._build_ui()

    def start_loading(self) -> None:
        pipeline = AnomalyPipeline(self._settings)
        self._thread = QThread(self)
        self._worker = AnomalyLoaderWorker(pipeline)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.progress_changed.connect(self._on_progress_changed)
        self._worker.completed.connect(self._on_completed)
        self._worker.failed.connect(self._on_failed)
        self._worker.completed.connect(self._thread.quit)
        self._worker.failed.connect(self._thread.quit)
        self._thread.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.start()

    def _build_ui(self) -> None:
        self.setWindowTitle("Schematic Gaussian Wasserstein Anomaly — Normal vs Follow")
        self.resize(1240, 920)
        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        layout.addWidget(self._status_label)
        layout.addWidget(self._progress_bar)
        layout.addWidget(self._gps_plot, 1)
        layout.addWidget(self._lidar_plot, 1)
        layout.addWidget(self._fused_plot, 1)
        self.setCentralWidget(central_widget)

    def _on_progress_changed(self, message: str, percent: int) -> None:
        self._status_label.setText(message)
        self._progress_bar.setValue(percent)

    def _on_completed(self, result: AnomalySeriesBundle) -> None:
        self._gps_plot.set_series(result.time_seconds, result.gps_wasserstein_distance)
        self._lidar_plot.set_series(result.time_seconds, result.lidar_wasserstein_distance)
        self._fused_plot.set_series(result.time_seconds, result.fused_wasserstein_distance)
        self._status_label.setText(
            "Synthetic one-lap result: shaded area marks the intended anomaly interval"
        )
        self._progress_bar.setValue(100)

    def _on_failed(self, message: str) -> None:
        self._status_label.setText("Failed to generate schematic anomaly graphs")
        QMessageBox.critical(self, "Anomaly graphing error", message)
