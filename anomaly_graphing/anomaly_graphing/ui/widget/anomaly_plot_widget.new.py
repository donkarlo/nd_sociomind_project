from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QFontMetrics, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget


class AnomalyPlotWidget(QWidget):
    def __init__(self, title: str, threshold: float, anomaly_fraction: float, parent=None):
        super().__init__(parent)
        self._title = title
        self._threshold = threshold
        self._anomaly_fraction = anomaly_fraction
        self._time_seconds: tuple[float, ...] = tuple()
        self._values: tuple[float, ...] = tuple()
        self.setMinimumHeight(220)

    def set_series(self, time_seconds: tuple[float, ...], values: tuple[float, ...]) -> None:
        self._time_seconds = time_seconds
        self._values = values
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        palette = self.palette()
        painter.fillRect(self.rect(), palette.base())

        left_margin = 88.0
        right_margin = 22.0
        top_margin = 34.0
        bottom_margin = 52.0
        plot_rectangle = QRectF(
            left_margin,
            top_margin,
            max(1.0, self.width() - left_margin - right_margin),
            max(1.0, self.height() - top_margin - bottom_margin),
        )

        painter.setPen(palette.text().color())
        painter.drawText(QPointF(left_margin, 21.0), self._title)

        maximum_value = max(self._threshold * 1.35, max(self._values, default=self._threshold))
        maximum_value *= 1.08

        grid_pen = QPen(palette.mid().color())
        grid_pen.setWidthF(0.7)
        painter.setPen(grid_pen)
        for fraction in (0.0, 0.25, 0.50, 0.75, 1.0):
            y_value = plot_rectangle.bottom() - fraction * plot_rectangle.height()
            painter.drawLine(QPointF(plot_rectangle.left(), y_value), QPointF(plot_rectangle.right(), y_value))
            x_value = plot_rectangle.left() + fraction * plot_rectangle.width()
            painter.drawLine(QPointF(x_value, plot_rectangle.top()), QPointF(x_value, plot_rectangle.bottom()))

        axis_pen = QPen(palette.text().color())
        axis_pen.setWidthF(1.2)
        painter.setPen(axis_pen)
        painter.drawLine(plot_rectangle.bottomLeft(), plot_rectangle.bottomRight())
        painter.drawLine(plot_rectangle.bottomLeft(), plot_rectangle.topLeft())

        font_metrics = QFontMetrics(painter.font())
        painter.setPen(palette.text().color())

        for fraction in (0.0, 0.25, 0.50, 0.75, 1.0):
            value = maximum_value * fraction
            y_value = plot_rectangle.bottom() - fraction * plot_rectangle.height()
            label = f"{value:.2f}"
            painter.drawText(QPointF(26.0, y_value + font_metrics.ascent() * 0.35), label)

        if self._time_seconds:
            minimum_time = self._time_seconds[0]
            maximum_time = self._time_seconds[-1]
        else:
            minimum_time = 0.0
            maximum_time = 100.0

        for fraction in (0.0, 0.25, 0.50, 0.75, 1.0):
            time_value = minimum_time + (maximum_time - minimum_time) * fraction
            x_value = plot_rectangle.left() + fraction * plot_rectangle.width()
            label = f"{time_value:.0f}"
            label_width = font_metrics.horizontalAdvance(label)
            painter.drawText(QPointF(x_value - label_width * 0.5, plot_rectangle.bottom() + 19.0), label)

        x_label = "Time (s)"
        x_label_width = font_metrics.horizontalAdvance(x_label)
        painter.drawText(
            QPointF(plot_rectangle.center().x() - x_label_width * 0.5, self.height() - 7.0),
            x_label,
        )

        painter.save()
        painter.translate(17.0, plot_rectangle.center().y())
        painter.rotate(-90.0)
        y_label = "Anomaly (Gaussian Wasserstein distance)"
        painter.drawText(QPointF(-font_metrics.horizontalAdvance(y_label) * 0.5, 0.0), y_label)
        painter.restore()

        threshold_pen = QPen(palette.text().color())
        threshold_pen.setStyle(Qt.DashLine)
        threshold_pen.setWidthF(1.0)
        painter.setPen(threshold_pen)
        threshold_fraction = min(1.0, max(0.0, self._threshold / maximum_value))
        threshold_y = plot_rectangle.bottom() - threshold_fraction * plot_rectangle.height()
        painter.drawLine(QPointF(plot_rectangle.left(), threshold_y), QPointF(plot_rectangle.right(), threshold_y))
        painter.drawText(QPointF(plot_rectangle.right() - 112.0, threshold_y - 5.0), f"threshold = {self._threshold:.2f}")

        if len(self._values) < 2:
            return

        points = self._build_curve_points(plot_rectangle, maximum_value)
        self._draw_anomaly_fill(painter, plot_rectangle, points)

        line_pen = QPen(palette.highlight().color())
        line_pen.setWidthF(2.1)
        painter.setPen(line_pen)
        path = QPainterPath()
        for index, point in enumerate(points):
            if index == 0:
                path.moveTo(point)
            else:
                path.lineTo(point)
        painter.drawPath(path)

    def _build_curve_points(self, plot_rectangle: QRectF, maximum_value: float) -> list[QPointF]:
        points: list[QPointF] = []
        for index, value in enumerate(self._values):
            x_fraction = index / max(1, len(self._values) - 1)
            x_value = plot_rectangle.left() + x_fraction * plot_rectangle.width()
            y_fraction = min(1.0, max(0.0, value / maximum_value))
            y_value = plot_rectangle.bottom() - y_fraction * plot_rectangle.height()
            points.append(QPointF(x_value, y_value))
        return points

    def _draw_anomaly_fill(self, painter: QPainter, plot_rectangle: QRectF, points: list[QPointF]) -> None:
        point_count = len(points)
        anomaly_count = max(2, round(point_count * self._anomaly_fraction))
        start_index = max(0, (point_count - anomaly_count) // 2)
        end_index = min(point_count - 1, start_index + anomaly_count - 1)

        fill_path = QPainterPath()
        fill_path.moveTo(points[start_index].x(), plot_rectangle.bottom())
        fill_path.lineTo(points[start_index])

        for index in range(start_index + 1, end_index + 1):
            fill_path.lineTo(points[index])

        fill_path.lineTo(points[end_index].x(), plot_rectangle.bottom())
        fill_path.closeSubpath()

        fill_color = self.palette().highlight().color()
        fill_color.setAlpha(42)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(fill_color))
        painter.drawPath(fill_path)
        painter.setBrush(Qt.NoBrush)
