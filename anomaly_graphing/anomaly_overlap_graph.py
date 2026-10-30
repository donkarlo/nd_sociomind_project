from __future__ import annotations

import math
import random
import sys

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFontMetrics, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget


SAMPLE_COUNT = 1000

# Each anomaly occupies 20% of the timeline.
# Their overlap is exactly 1% of the complete timeline.
SELF_START = 0.40
SELF_END = 0.60
LEADER_PHONEME_START = 0.59
LEADER_PHONEME_END = 0.79

SELF_LABEL = "Anomalies on normal scenario compared with self sensory data on follower"
LEADER_LABEL = "Anomaly using sensory data from follower but phonemes of leader robot"


def smooth_step(x: float, edge0: float, edge1: float) -> float:
    if edge0 == edge1:
        return 1.0 if x >= edge1 else 0.0
    t = max(0.0, min(1.0, (x - edge0) / (edge1 - edge0)))
    return t * t * (3.0 - 2.0 * t)


def anomaly_envelope(x: float, start: float, end: float, transition: float = 0.015) -> float:
    rise = smooth_step(x, start - transition, start + transition)
    fall = 1.0 - smooth_step(x, end - transition, end + transition)
    return max(0.0, min(1.0, rise * fall))


def generate_series(
    seed: int,
    center: float,
    drift_sigma: float,
    noise_sigma: float,
    kick_sigma: float,
) -> tuple[float, ...]:
    rng = random.Random(seed)
    values: list[float] = []

    # Random/noisy series with a stable range and no special deterministic segment.
    level = center
    for _ in range(SAMPLE_COUNT):
        level += rng.gauss(0.0, drift_sigma)
        level += 0.10 * (center - level)

        local_noise = rng.gauss(0.0, noise_sigma)
        occasional_kick = rng.gauss(0.0, kick_sigma) if rng.random() < 0.02 else 0.0
        values.append(max(0.0, level + local_noise + occasional_kick))

    return tuple(values)


def percentile(values: tuple[float, ...], p: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = round((len(ordered) - 1) * p)
    index = max(0, min(index, len(ordered) - 1))
    return ordered[index]


def add_distributed_one_percent_crossings(
    lower: tuple[float, ...],
    upper: tuple[float, ...],
    threshold: float,
) -> tuple[float, ...]:
    upper_values = list(upper)
    rng = random.Random(311)

    anchors: list[int] = []
    while len(anchors) < 10:
        candidate = rng.randint(25, SAMPLE_COUNT - 26)
        if all(abs(candidate - existing) >= 55 for existing in anchors):
            anchors.append(candidate)

    # Ten distributed center samples (=1% of 1000) become false negatives:
    # the orange curve briefly crosses below both the blue curve and threshold.
    # Neighboring points approach/recover smoothly but remain above threshold.
    for anchor in sorted(anchors):
        for offset in (-3, -2, -1, 1, 2, 3):
            index = anchor + offset
            distance = abs(offset)
            margin = {1: 0.003, 2: 0.009, 3: 0.018}[distance]
            upper_values[index] = max(
                lower[index] + margin,
                threshold + 0.002 + 0.003 * (distance - 1),
            )

        upper_values[anchor] = min(
            lower[anchor] - 0.002,
            threshold - 0.0015,
        )

    return tuple(upper_values)


def compute_binary_metrics(
    negative_values: tuple[float, ...],
    positive_values: tuple[float, ...],
    threshold: float,
) -> dict[str, float]:
    tn = sum(value <= threshold for value in negative_values)
    fp = sum(value > threshold for value in negative_values)
    tp = sum(value > threshold for value in positive_values)
    fn = sum(value <= threshold for value in positive_values)

    sensitivity = tp / (tp + fn) if tp + fn else 0.0
    specificity = tn / (tn + fp) if tn + fp else 0.0
    precision = tp / (tp + fp) if tp + fp else 0.0
    accuracy = (tp + tn) / (tp + tn + fp + fn) if tp + tn + fp + fn else 0.0
    f1 = (
        2.0 * precision * sensitivity / (precision + sensitivity)
        if precision + sensitivity
        else 0.0
    )

    return {
        "TP": tp,
        "FP": fp,
        "TN": tn,
        "FN": fn,
        "Sensitivity": sensitivity,
        "Specificity": specificity,
        "Precision": precision,
        "Accuracy": accuracy,
        "F1": f1,
    }


class CombinedAnomalyPlot(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        # Give the two curves clearly separated, publication-friendly
        # vertical ranges while keeping their fluctuations fully random.
        # Lower series: roughly 0.10--0.18.
        # Upper series: roughly 0.30--0.40.
        self._self_values = generate_series(
            seed=37,
            center=0.098,
            drift_sigma=0.00154,
            noise_sigma=0.00420,
            kick_sigma=0.00770,
        )
        self._leader_values = generate_series(
            seed=73,
            center=0.132,
            drift_sigma=0.00150,
            noise_sigma=0.00430,
            kick_sigma=0.00720,
        )

        # The threshold is the 99th percentile of the lower/blue series,
        # so about 1% of blue samples are false positives.
        self._threshold = percentile(self._self_values, 0.99)

        # Make exactly 1% of the orange samples distributed false negatives.
        self._leader_values = add_distributed_one_percent_crossings(
            self._self_values,
            self._leader_values,
            self._threshold,
        )

        # Shift both curves and the threshold downward by exactly the minimum
        # of the blue curve. This preserves all vertical distances/shapes while
        # forcing the blue minimum to be exactly zero on every run.
        vertical_shift = min(self._self_values)
        self._self_values = tuple(value - vertical_shift for value in self._self_values)
        self._leader_values = tuple(value - vertical_shift for value in self._leader_values)
        self._threshold -= vertical_shift

        self._metrics = compute_binary_metrics(
            self._self_values,
            self._leader_values,
            self._threshold,
        )

        # Publication-friendly blue/orange line colors.
        self._self_color = QColor("#4C78A8")
        self._leader_color = QColor("#F58518")
        self.setMinimumSize(1100, 560)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        palette = self.palette()
        painter.fillRect(self.rect(), palette.base())

        self._draw_metrics(painter)

        left_margin = 95.0
        right_margin = 30.0
        top_margin = 78.0
        bottom_margin = 115.0

        rect = QRectF(
            left_margin,
            top_margin,
            max(1.0, self.width() - left_margin - right_margin),
            max(1.0, self.height() - top_margin - bottom_margin),
        )

        # Blue is shifted so its minimum is exactly zero. Keep the same
        # relative scale and a small amount of headroom above the orange curve.
        minimum_value = 0.0
        maximum_value = 0.16

        self._draw_grid_and_axes(painter, rect, minimum_value, maximum_value)
        self._draw_threshold(painter, rect, minimum_value, maximum_value)

        self_points = self._build_points(
            rect, minimum_value, maximum_value, self._self_values
        )
        leader_points = self._build_points(
            rect, minimum_value, maximum_value, self._leader_values
        )

        self._draw_curve(painter, self_points, self._self_color)
        self._draw_curve(painter, leader_points, self._leader_color)
        self._draw_legend(painter, rect)

    def _draw_metrics(self, painter: QPainter) -> None:
        painter.setPen(self.palette().text().color())
        metrics = self._metrics

        painter.drawText(
            QPointF(16.0, 24.0),
            "Anomaly comparison",
        )

        text = (
            f"Threshold={self._threshold:.3f}  "
            f"TP={int(metrics['TP'])}  FP={int(metrics['FP'])}  "
            f"TN={int(metrics['TN'])}  FN={int(metrics['FN'])} | "
            f"Sensitivity={metrics['Sensitivity']:.3f}  "
            f"Specificity={metrics['Specificity']:.3f}  "
            f"Precision={metrics['Precision']:.3f}  "
            f"Accuracy={metrics['Accuracy']:.3f}  "
            f"F1={metrics['F1']:.3f}"
        )
        painter.drawText(QPointF(16.0, 47.0), text)

    def _draw_grid_and_axes(
        self,
        painter: QPainter,
        rect: QRectF,
        minimum_value: float,
        maximum_value: float,
    ) -> None:
        palette = self.palette()

        grid_pen = QPen(palette.mid().color())
        grid_pen.setWidthF(0.7)
        painter.setPen(grid_pen)

        for fraction in (0.0, 0.25, 0.50, 0.75, 1.0):
            y = rect.bottom() - fraction * rect.height()
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))

            x = rect.left() + fraction * rect.width()
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))

        axis_pen = QPen(palette.text().color())
        axis_pen.setWidthF(1.2)
        painter.setPen(axis_pen)
        painter.drawLine(rect.bottomLeft(), rect.bottomRight())
        painter.drawLine(rect.bottomLeft(), rect.topLeft())

        metrics = QFontMetrics(painter.font())

        for fraction in (0.0, 0.25, 0.50, 0.75, 1.0):
            y_value = minimum_value + (maximum_value - minimum_value) * fraction
            y = rect.bottom() - fraction * rect.height()
            painter.drawText(QPointF(35.0, y + metrics.ascent() * 0.35), f"{y_value:.2f}")

            x = rect.left() + fraction * rect.width()
            label = f"{fraction * 218:.1f}".rstrip("0").rstrip(".")
            width = metrics.horizontalAdvance(label)
            painter.drawText(QPointF(x - width / 2.0, rect.bottom() + 20.0), label)

        x_label = "Time"
        painter.drawText(
            QPointF(rect.center().x() - metrics.horizontalAdvance(x_label) / 2.0, rect.bottom() + 43.0),
            x_label,
        )

        painter.save()
        painter.translate(19.0, rect.center().y())
        painter.rotate(-90.0)
        y_label = "Anomaly"
        painter.drawText(QPointF(-metrics.horizontalAdvance(y_label) / 2.0, 0.0), y_label)
        painter.restore()

    def _draw_threshold(
        self,
        painter: QPainter,
        rect: QRectF,
        minimum_value: float,
        maximum_value: float,
    ) -> None:
        threshold_fraction = (self._threshold - minimum_value) / (maximum_value - minimum_value)
        threshold_y = rect.bottom() - threshold_fraction * rect.height()
        pen = QPen(QColor("#D62728"))
        pen.setStyle(Qt.DashLine)
        pen.setWidthF(1.0)
        painter.setPen(pen)
        painter.drawLine(QPointF(rect.left(), threshold_y), QPointF(rect.right(), threshold_y))
        painter.drawText(QPointF(rect.right() - 90.0, threshold_y - 6.0), "threshold")

    def _build_points(
        self,
        rect: QRectF,
        minimum_value: float,
        maximum_value: float,
        values: tuple[float, ...],
    ) -> list[QPointF]:
        points: list[QPointF] = []
        span = maximum_value - minimum_value
        for index, value in enumerate(values):
            fraction = index / (len(values) - 1)
            x = rect.left() + fraction * rect.width()
            y_fraction = (value - minimum_value) / span
            y = rect.bottom() - y_fraction * rect.height()
            points.append(QPointF(x, y))
        return points

    def _draw_curve(self, painter: QPainter, points: list[QPointF], color: QColor) -> None:
        pen = QPen(color)
        pen.setWidthF(1.8)
        painter.setPen(pen)

        path = QPainterPath()
        for index, point in enumerate(points):
            if index == 0:
                path.moveTo(point)
            else:
                path.lineTo(point)
        painter.drawPath(path)

    def _draw_legend(self, painter: QPainter, rect: QRectF) -> None:
        metrics = QFontMetrics(painter.font())
        y1 = rect.bottom() + 68.0
        y2 = rect.bottom() + 91.0
        x = rect.left()

        painter.setPen(QPen(self._self_color, 2.5))
        painter.drawLine(QPointF(x, y1), QPointF(x + 34.0, y1))
        painter.setPen(self.palette().text().color())
        painter.drawText(QPointF(x + 45.0, y1 + metrics.ascent() * 0.35), SELF_LABEL)

        painter.setPen(QPen(self._leader_color, 2.5))
        painter.drawLine(QPointF(x, y2), QPointF(x + 34.0, y2))
        painter.setPen(self.palette().text().color())
        painter.drawText(QPointF(x + 45.0, y2 + metrics.ascent() * 0.35), LEADER_LABEL)


class AnomalyOverlapWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Anomaly comparison")
        self.resize(1450, 720)
        self.setCentralWidget(CombinedAnomalyPlot())


def main() -> int:
    app = QApplication(sys.argv)
    window = AnomalyOverlapWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
