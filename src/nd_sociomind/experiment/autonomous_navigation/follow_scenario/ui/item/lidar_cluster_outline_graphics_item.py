"""A 720-vertex cluster medoid outline, attached to the recorded drone pose."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainterPath, QPen
from PySide6.QtWidgets import QGraphicsPathItem


class LidarClusterOutlineGraphicsItem(QGraphicsPathItem):
    def __init__(self, color: QColor, pixels_per_meter: float, cluster_style=Qt.SolidLine) -> None:
        super().__init__()
        self._scale = pixels_per_meter
        self._cluster_style = cluster_style
        self._last_points = None
        self._noise = False
        pen = QPen(color)
        pen.setWidthF(2.0)
        pen.setCosmetic(True)
        self.setPen(pen)
        self.setBrush(Qt.NoBrush)
        self.setZValue(15.0)
        self.setAcceptedMouseButtons(Qt.NoButton)

    def set_contour(self, points, *, noise: bool = False) -> None:
        if points is self._last_points and noise == self._noise:
            return
        self._last_points = points
        self._noise = noise
        if points is None:
            self.hide()
            self.setPath(QPainterPath())
            return
        self.show()
        pen = self.pen()
        pen.setStyle(Qt.DotLine if noise else self._cluster_style)
        self.setPen(pen)
        path = QPainterPath()
        previous_valid = False
        fully_valid = all(point is not None for point in points)
        for point in points:
            if point is None:
                previous_valid = False
                continue
            x, y = point[0] * self._scale, point[1] * self._scale
            if previous_valid:
                path.lineTo(x, y)
            else:
                path.moveTo(x, y)
            previous_valid = True
        if fully_valid:
            path.closeSubpath()
        self.setPath(path)
