import math

from PySide6.QtCore import QPointF, QRectF, QLineF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QPainter, QPen, QPolygonF, QRadialGradient
from PySide6.QtWidgets import QGraphicsObject, QStyleOptionGraphicsItem, QWidget

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.drone_role import DroneRole


class DroneGraphicsItem(QGraphicsObject):
    drag_started = Signal(object, float, float)
    drag_moved = Signal(object, float, float)
    drag_finished = Signal(object, float, float)

    def __init__(self, role: DroneRole, accent_color: QColor, visual_scale: float):
        super().__init__()
        self._role = role
        self._accent_color = accent_color
        self._drag_offset = QPointF(0.0, 0.0)
        self.setZValue(20.0)
        self.setTransformOriginPoint(0.0, 0.0)
        self.setScale(visual_scale)
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setAcceptHoverEvents(True)
        self.setCursor(Qt.OpenHandCursor)

    def boundingRect(self) -> QRectF:
        return QRectF(-34.0, -34.0, 68.0, 68.0)

    def set_pose(self, x_pixels: float, y_pixels: float, heading_radians: float) -> None:
        self.setPos(x_pixels, y_pixels)
        self.setRotation(math.degrees(heading_radians) + 90.0)

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.LeftButton:
            event.ignore()
            return
        self._drag_offset = self.pos() - event.scenePos()
        self.setCursor(Qt.ClosedHandCursor)
        self.drag_started.emit(self._role, self.pos().x(), self.pos().y())
        event.accept()

    def mouseMoveEvent(self, event) -> None:
        desired_position = event.scenePos() + self._drag_offset
        self.drag_moved.emit(self._role, desired_position.x(), desired_position.y())
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() != Qt.LeftButton:
            event.ignore()
            return
        self.setCursor(Qt.OpenHandCursor)
        self.drag_finished.emit(self._role, self.pos().x(), self.pos().y())
        event.accept()

    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget: QWidget | None = None) -> None:
        painter.setRenderHint(QPainter.Antialiasing, True)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 75)))
        painter.drawEllipse(QRectF(-27.0, -21.0, 58.0, 58.0))

        arm_pen = QPen(self._accent_color.lighter(125), 6.0, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(arm_pen)
        painter.drawLine(QLineF(-21.0, -21.0, 21.0, 21.0))
        painter.drawLine(QLineF(21.0, -21.0, -21.0, 21.0))

        for center_x, center_y in [(-23.0, -23.0), (23.0, -23.0), (-23.0, 23.0), (23.0, 23.0)]:
            rotor_gradient = QRadialGradient(center_x, center_y, 13.0)
            rotor_gradient.setColorAt(0.0, QColor(self._accent_color.red(), self._accent_color.green(), self._accent_color.blue(), 65))
            rotor_gradient.setColorAt(1.0, QColor(self._accent_color.red(), self._accent_color.green(), self._accent_color.blue(), 8))
            painter.setBrush(QBrush(rotor_gradient))
            painter.setPen(QPen(self._accent_color.lighter(140), 1.4))
            painter.drawEllipse(QRectF(center_x - 12.0, center_y - 12.0, 24.0, 24.0))
            painter.setBrush(QBrush(QColor("#dce7ff")))
            painter.drawEllipse(QRectF(center_x - 3.2, center_y - 3.2, 6.4, 6.4))

        body_gradient = QRadialGradient(0.0, 0.0, 18.0)
        body_gradient.setColorAt(0.0, self._accent_color.lighter(145))
        body_gradient.setColorAt(1.0, self._accent_color.darker(145))
        painter.setBrush(QBrush(body_gradient))
        painter.setPen(QPen(QColor("#f4f7ff"), 1.6))
        painter.drawRoundedRect(QRectF(-13.0, -16.0, 26.0, 32.0), 8.0, 8.0)

        painter.setBrush(QBrush(QColor("#ffffff")))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(QPolygonF([QPointF(-5.0, -20.0), QPointF(5.0, -20.0), QPointF(0.0, -29.0)]))
