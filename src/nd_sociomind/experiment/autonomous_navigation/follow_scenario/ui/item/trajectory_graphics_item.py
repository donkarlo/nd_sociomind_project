from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QGraphicsItem, QStyleOptionGraphicsItem, QWidget

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory import Trajectory


class TrajectoryGraphicsItem(QGraphicsItem):
    def __init__(self, trajectory: Trajectory, pixels_per_meter: float, color: QColor, close_loop: bool = False):
        super().__init__()
        self._trajectory = trajectory
        self._pixels_per_meter = pixels_per_meter
        self._color = color
        self._close_loop = close_loop
        self.setZValue(-5.0)

    def boundingRect(self) -> QRectF:
        if self._trajectory.is_empty():
            return QRectF()
        x_values = [sample.x_meters * self._pixels_per_meter for sample in self._trajectory.samples]
        y_values = [sample.y_meters * self._pixels_per_meter for sample in self._trajectory.samples]
        minimum_x = min(x_values)
        maximum_x = max(x_values)
        minimum_y = min(y_values)
        maximum_y = max(y_values)
        return QRectF(minimum_x - 10.0, minimum_y - 10.0, maximum_x - minimum_x + 20.0, maximum_y - minimum_y + 20.0)

    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget: QWidget | None = None) -> None:
        if self._trajectory.length() < 2:
            return

        path = QPainterPath()
        first_sample = self._trajectory.samples[0]
        path.moveTo(first_sample.x_meters * self._pixels_per_meter, first_sample.y_meters * self._pixels_per_meter)
        for sample in self._trajectory.samples[1:]:
            path.lineTo(sample.x_meters * self._pixels_per_meter, sample.y_meters * self._pixels_per_meter)
        if self._close_loop:
            path.lineTo(first_sample.x_meters * self._pixels_per_meter, first_sample.y_meters * self._pixels_per_meter)

        route_color = QColor(self._color)
        route_color.setAlpha(92)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(route_color, 2.0, Qt.DashLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawPath(path)
