"""A small upright CG cross and label attached to the LiDAR area centroid."""
from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QGraphicsItem, QStyleOptionGraphicsItem, QWidget


class LidarCgGraphicsItem(QGraphicsItem):
    def __init__(self, color: QColor, parent: QGraphicsItem):
        super().__init__(parent)
        self._color = color
        # The position follows the parent LiDAR's ROS heading and location,
        # but the tiny cross/label remain upright and screen-sized.
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self.setAcceptedMouseButtons(Qt.NoButton)
        self.setZValue(2.0)
        self.setVisible(False)

    def boundingRect(self) -> QRectF:
        return QRectF(-5.0, -7.0, 38.0, 16.0)

    def paint(
        self,
        painter: QPainter,
        option: QStyleOptionGraphicsItem,
        widget: QWidget | None = None,
    ) -> None:
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(self._color, 1.2))
        painter.drawLine(QPointF(-3.0, -3.0), QPointF(3.0, 3.0))
        painter.drawLine(QPointF(-3.0, 3.0), QPointF(3.0, -3.0))
        font = QFont()
        font.setPixelSize(10)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(QPointF(6.0, 3.5), "CG")
