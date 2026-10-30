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
        self._status_label = QLabel("Normal vs Follow")
        self._progress_bar = QProgressBar()

        self._gps_plot = self._create_plot("GPS", settings.gps_anomaly_fraction)
        self._lidar_plot = self._create_plot("LiDAR", settings.lidar_anomaly_fraction, True)
        self._fused_plot = self._create_plot("Fused GPS + LiDAR", settings.fused_anomaly_fraction, True)

        self._build_ui()

    def _create_plot(self, title: str, anomaly_fraction: float, show_lidar_visibility: bool = False) -> AnomalyPlotWidget:
        lidar_start = self._settings.lidar_visibility_start_fraction() if show_lidar_visibility else None
        lidar_end = self._settings.lidar_visibility_end_fraction() if show_lidar_visibility else None

        return AnomalyPlotWidget(
            title,
            anomaly_fraction,
            self._settings.wall_start_fraction,
            self._settings.wall_end_fraction,
            lidar_start,
            lidar_end,
        )

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
        self.setWindowTitle("Gaussian Wasserstein Anomaly — Normal vs Follow")
        self.resize(1420, 980)

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
        self._gps_plot.set_series(
            result.time_seconds,
            result.gps_wasserstein_distance,
            result.thresholds.gps,
            result.gps_metrics,
        )
        self._lidar_plot.set_series(
            result.time_seconds,
            result.lidar_wasserstein_distance,
            result.thresholds.lidar,
            result.lidar_metrics,
        )
        self._fused_plot.set_series(
            result.time_seconds,
            result.fused_wasserstein_distance,
            result.thresholds.fused,
            result.fused_metrics,
        )
        self._status_label.setText(
            f"One lap 0–{result.time_seconds[-1]:.1f} s | "
            f"threshold: GPS={result.thresholds.gps:.2f}, "
            f"LiDAR={result.thresholds.lidar:.2f}, "
            f"Fused={result.thresholds.fused:.2f}"
        )
        self._progress_bar.setValue(100)

    def _on_failed(self, message: str) -> None:
        self._status_label.setText("Failed to generate anomaly graphs")
        QMessageBox.critical(self, "Anomaly graphing error", message)
