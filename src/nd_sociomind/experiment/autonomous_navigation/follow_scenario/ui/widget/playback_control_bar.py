import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings


class PlaybackControlBar(QFrame):
    play_pause_requested = Signal()
    restart_requested = Signal()
    speed_changed = Signal(float)
    timeline_changed = Signal(int)

    def __init__(self, settings: ScenarioSettings, parent: QWidget | None = None):
        super().__init__(parent)
        self._settings = settings
        self._frame_count = 0
        self._play_button = QPushButton("▶  Play")
        self._restart_button = QPushButton("↺")
        self._speed_slider = QSlider(Qt.Horizontal)
        self._timeline_slider = QSlider(Qt.Horizontal)
        self._speed_value_label = QLabel(f"{settings.default_speed_multiplier:0.2f}×")
        self._phase_label = QLabel("Waiting for GPS data…")
        self._distance_label = QLabel("UAV separation  —")
        self._build_ui()
        self._connect_signals()

    def _build_ui(self) -> None:
        self.setObjectName("controlCard")
        self.setStyleSheet("""
            QFrame#controlCard { background: #101a2b; border: 1px solid #2a3955; border-radius: 16px; }
            QLabel { color: #dbe7ff; }
            QPushButton { background: #1d2b45; color: #eef5ff; border: 1px solid #365078; border-radius: 9px; padding: 8px 14px; font-weight: 600; }
            QPushButton:hover { background: #263b5f; }
            QSlider::groove:horizontal { height: 6px; background: #263650; border-radius: 3px; }
            QSlider::sub-page:horizontal { background: #4f8cff; border-radius: 3px; }
            QSlider::handle:horizontal { width: 16px; margin: -5px 0; background: #eef5ff; border: 2px solid #4f8cff; border-radius: 8px; }
        """)
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(16, 14, 16, 14)
        outer_layout.setSpacing(10)

        top_row = QHBoxLayout()
        top_row.setSpacing(10)
        self._restart_button.setFixedWidth(44)
        top_row.addWidget(self._play_button)
        top_row.addWidget(self._restart_button)
        top_row.addSpacing(8)
        top_row.addWidget(QLabel("Speed"))
        self._speed_slider.setRange(0, 1000)
        self._speed_slider.setValue(self._speed_to_slider(self._settings.default_speed_multiplier))
        top_row.addWidget(self._speed_slider, 1)
        self._speed_value_label.setMinimumWidth(72)
        self._speed_value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        top_row.addWidget(self._speed_value_label)
        outer_layout.addLayout(top_row)

        timeline_row = QHBoxLayout()
        timeline_row.addWidget(QLabel("Lap"))
        self._timeline_slider.setRange(0, 0)
        timeline_row.addWidget(self._timeline_slider, 1)
        outer_layout.addLayout(timeline_row)

        status_row = QHBoxLayout()
        self._phase_label.setStyleSheet("font-weight: 700; color: #f0f5ff;")
        self._distance_label.setStyleSheet("color: #9db1d2;")
        status_row.addWidget(self._phase_label)
        status_row.addStretch(1)
        status_row.addWidget(self._distance_label)
        outer_layout.addLayout(status_row)

    def _connect_signals(self) -> None:
        self._play_button.clicked.connect(self.play_pause_requested.emit)
        self._restart_button.clicked.connect(self.restart_requested.emit)
        self._speed_slider.valueChanged.connect(self._on_speed_slider_changed)
        self._timeline_slider.valueChanged.connect(self.timeline_changed.emit)

    def configure_timeline(self, frame_count: int) -> None:
        self._frame_count = frame_count
        maximum_frame = max(0, frame_count - 1)
        self._timeline_slider.setRange(0, maximum_frame)

    def set_timeline_frame(self, frame_index: int) -> None:
        self._timeline_slider.blockSignals(True)
        self._timeline_slider.setValue(frame_index)
        self._timeline_slider.blockSignals(False)

    def set_playing(self, is_playing: bool) -> None:
        if is_playing:
            self._play_button.setText("Ⅱ  Pause")
        else:
            self._play_button.setText("▶  Play")

    def set_status(self, phase_text: str, separation_meters: float) -> None:
        self._phase_label.setText(phase_text)
        self._distance_label.setText(f"UAV separation  {separation_meters:0.2f} m · narrow target {self._settings.desired_follow_distance_meters:0.1f} m")

    def set_loading_status(self, text: str) -> None:
        self._phase_label.setText(text)
        self._distance_label.setText("Using only the two cleaned GPS files")

    def _on_speed_slider_changed(self, slider_value: int) -> None:
        speed_multiplier = self._slider_to_speed(slider_value)
        self._speed_value_label.setText(f"{speed_multiplier:0.2f}×")
        self.speed_changed.emit(speed_multiplier)

    def _slider_to_speed(self, slider_value: int) -> float:
        minimum_speed = self._settings.minimum_speed_multiplier
        maximum_speed = self._settings.maximum_speed_multiplier
        normalized_value = slider_value / 1000.0
        return minimum_speed * ((maximum_speed / minimum_speed) ** normalized_value)

    def _speed_to_slider(self, speed_multiplier: float) -> int:
        minimum_speed = self._settings.minimum_speed_multiplier
        maximum_speed = self._settings.maximum_speed_multiplier
        normalized_value = math.log(speed_multiplier / minimum_speed) / math.log(maximum_speed / minimum_speed)
        return int(normalized_value * 1000.0)
