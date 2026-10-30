from PySide6.QtCore import QObject, QTimer, Signal


class PlaybackController(QObject):
    frame_changed = Signal(int)
    playing_changed = Signal(bool)

    def __init__(self, frames_per_second: int, parent: QObject | None = None):
        super().__init__(parent)
        self._frames_per_second = frames_per_second
        self._frame_count = 0
        self._source_frames_per_second = 10.0
        self._current_frame = 0
        self._speed_multiplier = 1.0
        self._fractional_frames = 0.0
        self._timer = QTimer(self)
        self._timer.setInterval(max(1, int(1000 / frames_per_second)))
        self._timer.timeout.connect(self._advance)

    def configure(self, frame_count: int, source_frames_per_second: float) -> None:
        self._frame_count = frame_count
        self._source_frames_per_second = max(0.1, source_frames_per_second)
        self._current_frame = 0
        self._fractional_frames = 0.0
        self.frame_changed.emit(self._current_frame)

    def play(self) -> None:
        if self._frame_count <= 1:
            return
        self._timer.start()
        self.playing_changed.emit(True)

    def pause(self) -> None:
        self._timer.stop()
        self.playing_changed.emit(False)

    def toggle(self) -> None:
        if self._timer.isActive():
            self.pause()
        else:
            self.play()

    def restart(self) -> None:
        self._current_frame = 0
        self._fractional_frames = 0.0
        self.frame_changed.emit(self._current_frame)

    def set_speed_multiplier(self, speed_multiplier: float) -> None:
        self._speed_multiplier = max(0.01, speed_multiplier)

    def set_frame(self, frame_index: int) -> None:
        if self._frame_count <= 0:
            return
        self._current_frame = max(0, min(frame_index, self._frame_count - 1))
        self._fractional_frames = 0.0
        self.frame_changed.emit(self._current_frame)

    def is_playing(self) -> bool:
        return self._timer.isActive()

    def current_frame(self) -> int:
        return self._current_frame

    def _advance(self) -> None:
        if self._frame_count <= 1:
            return

        self._fractional_frames += self._speed_multiplier * self._source_frames_per_second / self._frames_per_second
        whole_frames = int(self._fractional_frames)
        if whole_frames <= 0:
            return

        self._fractional_frames -= whole_frames
        self._current_frame += whole_frames
        if self._current_frame >= self._frame_count:
            self._current_frame = 0
        self.frame_changed.emit(self._current_frame)
