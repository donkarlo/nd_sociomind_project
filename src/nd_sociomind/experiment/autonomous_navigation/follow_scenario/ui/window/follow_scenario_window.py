from pathlib import Path

from PySide6.QtCore import QThread
from PySide6.QtWidgets import QHBoxLayout, QLabel, QMainWindow, QMessageBox, QVBoxLayout, QWidget

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.application.trajectory_loader_worker import TrajectoryLoaderWorker
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.data_source_settings import DataSourceSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.trajectory_synchronizer import TrajectorySynchronizer
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.world_coordinate_transformer import WorldCoordinateTransformer
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.cleaned_gps_reader import CleanedGpsReader
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.interaction.controller.interactive_flight_controller import InteractiveFlightController
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.playback.playback_controller import PlaybackController
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.scene.follow_scenario_scene import FollowScenarioScene
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.view.scenario_graphics_view import ScenarioGraphicsView
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.widget.playback_control_bar import PlaybackControlBar


class FollowScenarioWindow(QMainWindow):
    def __init__(self, leader_path: Path, follower_path: Path, scenario_settings: ScenarioSettings, data_source_settings: DataSourceSettings):
        super().__init__()
        self._leader_path = leader_path
        self._follower_path = follower_path
        self._scenario_settings = scenario_settings
        self._data_source_settings = data_source_settings
        self._scene = FollowScenarioScene(scenario_settings)
        self._view = ScenarioGraphicsView()
        self._controls = PlaybackControlBar(scenario_settings)
        self._playback = PlaybackController(scenario_settings.target_frames_per_second, self)
        self._interactive_controller: InteractiveFlightController | None = None
        self._loader_thread: QThread | None = None
        self._loader_worker: TrajectoryLoaderWorker | None = None
        self._build_ui()
        self._connect_signals()
        self._start_loading()

    def _build_ui(self) -> None:
        self.setWindowTitle("Sociomind · Leader–Follower Corridor Replay")
        self.resize(1180, 900)
        self.setMinimumSize(900, 700)
        self.setStyleSheet("QMainWindow { background: #07101d; }")

        central_widget = QWidget()
        root_layout = QVBoxLayout(central_widget)
        root_layout.setContentsMargins(18, 16, 18, 18)
        root_layout.setSpacing(14)

        header_layout = QHBoxLayout()
        title_label = QLabel("Leader–Follower Corridor Replay")
        title_label.setStyleSheet("color: #f4f8ff; font-size: 24px; font-weight: 800;")
        source_label = QLabel(f"{self._leader_path.name} + {self._follower_path.name}")
        self._source_label = source_label
        source_label.setStyleSheet("color: #7892ba; font-size: 12px;")
        header_layout.addWidget(title_label)
        header_layout.addStretch(1)
        header_layout.addWidget(source_label)
        root_layout.addLayout(header_layout)

        subtitle_label = QLabel("Top view · recorded LiDAR contours (when available) · drag either drone · Gazebo safety")
        subtitle_label.setStyleSheet("color: #9bb0d0; font-size: 13px;")
        root_layout.addWidget(subtitle_label)

        self._view.setScene(self._scene)
        root_layout.addWidget(self._view, 1)
        root_layout.addWidget(self._controls)
        self.setCentralWidget(central_widget)

    def _connect_signals(self) -> None:
        self._controls.play_pause_requested.connect(self._playback.toggle)
        self._controls.restart_requested.connect(self._playback.restart)
        self._controls.speed_changed.connect(self._playback.set_speed_multiplier)
        self._controls.timeline_changed.connect(self._playback.set_frame)
        self._playback.frame_changed.connect(self._on_frame_changed)
        self._playback.playing_changed.connect(self._controls.set_playing)

    def _start_loading(self) -> None:
        self._controls.set_loading_status("Loading models ...")
        reader = CleanedGpsReader(self._data_source_settings, self._scenario_settings)
        synchronizer = TrajectorySynchronizer()
        transformer = WorldCoordinateTransformer(self._scenario_settings)
        self._loader_thread = QThread(self)
        self._loader_worker = TrajectoryLoaderWorker(
            self._leader_path, self._follower_path, reader, synchronizer, transformer,
            lidar_path=self._data_source_settings.find_lidar_yaml(),
        )
        self._loader_worker.moveToThread(self._loader_thread)
        self._loader_thread.started.connect(self._loader_worker.load)
        self._loader_worker.loaded.connect(self._on_loaded)
        self._loader_worker.failed.connect(self._on_loading_failed)
        self._loader_worker.finished.connect(self._loader_thread.quit)
        self._loader_worker.finished.connect(self._loader_worker.deleteLater)
        self._loader_thread.finished.connect(self._loader_thread.deleteLater)
        self._loader_thread.start()

    def _on_loaded(self, payload) -> None:
        trajectory_pair, lidar_frames, clusters, lidar_note = payload
        self._scene.set_lidar_frames(lidar_frames)
        self._scene.set_lidar_clusters(clusters)
        if self._data_source_settings.find_lidar_yaml() is not None:
            self._source_label.setText(self._data_source_settings.find_lidar_yaml().name + " · ROS GPS + LiDAR")
        self._scene.set_trajectories(trajectory_pair)
        frame_count = self._scene.frame_count()
        source_frames_per_second = self._source_frames_per_second()
        self._playback.configure(frame_count, source_frames_per_second)
        self._controls.configure_timeline(frame_count)
        self._interactive_controller = InteractiveFlightController(self._scene, self._playback, trajectory_pair, self._scenario_settings, self)
        self._interactive_controller.status_changed.connect(self._controls.set_loading_status)
        self._view.fit_scene()
        self._controls.set_loading_status(f"Loaded {frame_count:,} synchronized GPS frames · {lidar_note} · drag either drone")
        self._playback.play()

    def _source_frames_per_second(self) -> float:
        sample_period_seconds = self._data_source_settings.fallback_sample_period_seconds
        if sample_period_seconds <= 0.0:
            return 50.0
        return 1.0 / sample_period_seconds

    def _on_loading_failed(self, message: str) -> None:
        self._controls.set_loading_status("Could not load GPS trajectory")
        QMessageBox.critical(self, "GPS data loading failed", message)

    def _on_frame_changed(self, frame_index: int) -> None:
        phase, separation_meters = self._scene.set_frame(frame_index)
        self._controls.set_timeline_frame(frame_index)
        self._controls.set_status(phase.value, separation_meters)
