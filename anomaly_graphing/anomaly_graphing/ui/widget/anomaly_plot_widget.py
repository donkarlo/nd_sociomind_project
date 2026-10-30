from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFontMetrics, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QWidget

from anomaly_graphing.data.model.binary_classification_metrics import BinaryClassificationMetrics


class AnomalyPlotWidget(QWidget):
    def __init__(self, title: str, anomaly_fraction: float, wall_start_fraction: float, wall_end_fraction: float, lidar_visibility_start_fraction: float | None = None, lidar_visibility_end_fraction: float | None = None, parent=None):
        super().__init__(parent)
        self._title = title
        self._threshold = 0.5
        self._metrics: BinaryClassificationMetrics | None = None
        self._anomaly_fraction = anomaly_fraction
        self._wall_start_fraction = wall_start_fraction
        self._wall_end_fraction = wall_end_fraction
        self._lidar_visibility_start_fraction = lidar_visibility_start_fraction
        self._lidar_visibility_end_fraction = lidar_visibility_end_fraction
        self._time_seconds: tuple[float, ...] = tuple()
        self._values: tuple[float, ...] = tuple()
        self.setMinimumHeight(255)

    def set_series(self, time_seconds: tuple[float, ...], values: tuple[float, ...], threshold: float, metrics: BinaryClassificationMetrics) -> None:
        self._time_seconds = time_seconds
        self._values = values
        self._threshold = threshold
        self._metrics = metrics
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        palette = self.palette()
        painter.fillRect(self.rect(), palette.base())

        left_margin = 88.0
        right_margin = 22.0
        top_margin = 58.0
        bottom_margin = 52.0
        plot_rectangle = QRectF(
            left_margin,
            top_margin,
            max(1.0, self.width() - left_margin - right_margin),
            max(1.0, self.height() - top_margin - bottom_margin),
        )

        painter.setPen(palette.text().color())
        painter.drawText(QPointF(left_margin, 19.0), self._title)
        self._draw_metrics(painter, left_margin)

        maximum_value = max(self._threshold * 1.35, max(self._values, default=self._threshold))
        maximum_value *= 1.08

        self._draw_grid_and_axes(painter, plot_rectangle, maximum_value)
        self._draw_threshold(painter, plot_rectangle, maximum_value)

        if len(self._values) < 2:
            return

        points = self._build_curve_points(plot_rectangle, maximum_value)
        self._draw_anomaly_fill(painter, plot_rectangle, points)
        self._draw_wall_markers(painter, plot_rectangle)
        self._draw_lidar_visibility_markers(painter, plot_rectangle)
        self._draw_curve(painter, points)

    def _draw_metrics(self, painter: QPainter, left_margin: float) -> None:
        if self._metrics is None:
            return

        metrics = self._metrics
        line = (
            f"Threshold={self._threshold:.2f} | "
            f"TP={metrics.true_positive}  FP={metrics.false_positive}  "
            f"TN={metrics.true_negative}  FN={metrics.false_negative} | "
            f"Sensitivity={metrics.sensitivity:.3f}  "
            f"Specificity={metrics.specificity:.3f}  "
            f"Precision={metrics.precision:.3f}  "
            f"Accuracy={metrics.accuracy:.3f}  "
            f"F1={metrics.f1_score:.3f}"
        )
        painter.drawText(QPointF(left_margin, 39.0), line)

    def _draw_grid_and_axes(self, painter: QPainter, plot_rectangle: QRectF, maximum_value: float) -> None:
        palette = self.palette()
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
            painter.drawText(QPointF(26.0, y_value + font_metrics.ascent() * 0.35), f"{value:.2f}")

        maximum_time = self._time_seconds[-1] if self._time_seconds else 0.0
        for fraction in (0.0, 0.25, 0.50, 0.75, 1.0):
            time_value = maximum_time * fraction
            x_value = plot_rectangle.left() + fraction * plot_rectangle.width()
            label = f"{time_value:.1f}"
            label_width = font_metrics.horizontalAdvance(label)
            painter.drawText(QPointF(x_value - label_width * 0.5, plot_rectangle.bottom() + 19.0), label)

        x_label = "Time (s)"
        x_label_width = font_metrics.horizontalAdvance(x_label)
        painter.drawText(QPointF(plot_rectangle.center().x() - x_label_width * 0.5, self.height() - 7.0), x_label)

        painter.save()
        painter.translate(17.0, plot_rectangle.center().y())
        painter.rotate(-90.0)
        y_label = "Anomaly (Gaussian Wasserstein distance)"
        painter.drawText(QPointF(-font_metrics.horizontalAdvance(y_label) * 0.5, 0.0), y_label)
        painter.restore()

    def _draw_threshold(self, painter: QPainter, plot_rectangle: QRectF, maximum_value: float) -> None:
        threshold_pen = QPen(QColor(190, 35, 35))
        threshold_pen.setStyle(Qt.DashLine)
        threshold_pen.setWidthF(1.0)
        painter.setPen(threshold_pen)

        threshold_fraction = min(1.0, max(0.0, self._threshold / maximum_value))
        threshold_y = plot_rectangle.bottom() - threshold_fraction * plot_rectangle.height()
        painter.drawLine(QPointF(plot_rectangle.left(), threshold_y), QPointF(plot_rectangle.right(), threshold_y))

    def _draw_wall_markers(self, painter: QPainter, plot_rectangle: QRectF) -> None:
        marker_pen = QPen(QColor(128, 70, 170))
        marker_pen.setStyle(Qt.DashLine)
        marker_pen.setWidthF(1.0)
        painter.setPen(marker_pen)

        start_x = plot_rectangle.left() + self._wall_start_fraction * plot_rectangle.width()
        end_x = plot_rectangle.left() + self._wall_end_fraction * plot_rectangle.width()
        painter.drawLine(QPointF(start_x, plot_rectangle.top()), QPointF(start_x, plot_rectangle.bottom()))
        painter.drawLine(QPointF(end_x, plot_rectangle.top()), QPointF(end_x, plot_rectangle.bottom()))

        legend_x = plot_rectangle.left() + 10.0
        legend_y = plot_rectangle.top() + 16.0
        line_length = 28.0
        painter.drawLine(QPointF(legend_x, legend_y), QPointF(legend_x + line_length, legend_y))
        painter.drawText(QPointF(legend_x + line_length + 7.0, legend_y + 4.0), "narrow corridor start and end")

    def _draw_lidar_visibility_markers(self, painter: QPainter, plot_rectangle: QRectF) -> None:
        if self._lidar_visibility_start_fraction is None or self._lidar_visibility_end_fraction is None:
            return

        marker_pen = QPen(QColor(220, 130, 25))
        marker_pen.setStyle(Qt.DashLine)
        marker_pen.setWidthF(1.0)
        painter.setPen(marker_pen)

        start_x = plot_rectangle.left() + self._lidar_visibility_start_fraction * plot_rectangle.width()
        end_x = plot_rectangle.left() + self._lidar_visibility_end_fraction * plot_rectangle.width()
        painter.drawLine(QPointF(start_x, plot_rectangle.top()), QPointF(start_x, plot_rectangle.bottom()))
        painter.drawLine(QPointF(end_x, plot_rectangle.top()), QPointF(end_x, plot_rectangle.bottom()))

        legend_x = plot_rectangle.left() + 10.0
        legend_y = plot_rectangle.top() + 34.0
        line_length = 28.0
        painter.drawLine(QPointF(legend_x, legend_y), QPointF(legend_x + line_length, legend_y))
        painter.drawText(QPointF(legend_x + line_length + 7.0, legend_y + 4.0), "LIDAR starts and ends to see narrow corridor")

    def _draw_curve(self, painter: QPainter, points: list[QPointF]) -> None:
        line_pen = QPen(self.palette().highlight().color())
        line_pen.setWidthF(1.5)
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

        if self._lidar_visibility_start_fraction is not None and self._lidar_visibility_end_fraction is not None:
            start_index = round((point_count - 1) * self._lidar_visibility_start_fraction)
            end_index = round((point_count - 1) * self._lidar_visibility_end_fraction)
        else:
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
