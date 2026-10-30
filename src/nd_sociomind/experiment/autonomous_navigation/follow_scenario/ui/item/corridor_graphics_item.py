from PySide6.QtCore import QLineF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QLinearGradient, QPainter, QPen
from PySide6.QtWidgets import QGraphicsItem, QStyleOptionGraphicsItem, QWidget

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.corridor_geometry import CorridorGeometry
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.corridor_side import CorridorSide
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.narrow_section_placement import NarrowSectionPlacement


class CorridorGraphicsItem(QGraphicsItem):
    def __init__(self, settings: ScenarioSettings):
        super().__init__()
        self._settings = settings
        self._scale = settings.world_pixels_per_meter
        center_x = settings.outer_width_meters * 0.5
        center_y = settings.outer_height_meters * 0.5
        narrow_section = NarrowSectionPlacement(CorridorSide.RIGHT, center_y)
        self._geometry = CorridorGeometry(center_x, center_y, narrow_section)
        self.setZValue(-20.0)

    def boundingRect(self) -> QRectF:
        return QRectF(0.0, 0.0, self._settings.outer_width_meters * self._scale, self._settings.outer_height_meters * self._scale)

    def set_geometry(self, geometry: CorridorGeometry) -> None:
        self._geometry = geometry
        self.update()

    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget: QWidget | None = None) -> None:
        painter.setRenderHint(QPainter.Antialiasing, True)
        outer_rectangle = self.boundingRect()

        background_gradient = QLinearGradient(outer_rectangle.topLeft(), outer_rectangle.bottomRight())
        background_gradient.setColorAt(0.0, QColor("#0b1220"))
        background_gradient.setColorAt(1.0, QColor("#121a2c"))
        painter.fillRect(outer_rectangle, QBrush(background_gradient))

        self._draw_grid(painter, outer_rectangle)
        self._draw_outer_walls(painter)
        self._draw_inner_obstacle(painter)
        self._draw_gazebo_narrowing_walls(painter)

    def _draw_grid(self, painter: QPainter, outer_rectangle: QRectF) -> None:
        grid_pen = QPen(QColor(255, 255, 255, 16), 1.0)
        painter.setPen(grid_pen)
        for meter_x in range(0, int(self._settings.outer_width_meters) + 1, 2):
            pixel_x = meter_x * self._scale
            painter.drawLine(QLineF(pixel_x, 0.0, pixel_x, outer_rectangle.height()))
        for meter_y in range(0, int(self._settings.outer_height_meters) + 1, 2):
            pixel_y = meter_y * self._scale
            painter.drawLine(QLineF(0.0, pixel_y, outer_rectangle.width(), pixel_y))

    def _draw_outer_walls(self, painter: QPainter) -> None:
        wall_pen = self._wall_pen(QColor("#4f8cff"))
        painter.setPen(wall_pen)
        painter.setBrush(Qt.NoBrush)

        width_pixels = self._settings.outer_width_meters * self._scale
        height_pixels = self._settings.outer_height_meters * self._scale
        half_wall_pixels = self._settings.wall_thickness_meters * self._scale * 0.5

        painter.drawLine(QLineF(half_wall_pixels, half_wall_pixels, width_pixels - half_wall_pixels, half_wall_pixels))
        painter.drawLine(QLineF(width_pixels - half_wall_pixels, half_wall_pixels, width_pixels - half_wall_pixels, height_pixels - half_wall_pixels))
        painter.drawLine(QLineF(width_pixels - half_wall_pixels, height_pixels - half_wall_pixels, half_wall_pixels, height_pixels - half_wall_pixels))
        painter.drawLine(QLineF(half_wall_pixels, height_pixels - half_wall_pixels, half_wall_pixels, half_wall_pixels))

    def _draw_inner_obstacle(self, painter: QPainter) -> None:
        inner_rectangle = self._inner_rectangle()
        inner_gradient = QLinearGradient(inner_rectangle.topLeft(), inner_rectangle.bottomRight())
        inner_gradient.setColorAt(0.0, QColor("#23314e"))
        inner_gradient.setColorAt(1.0, QColor("#152033"))
        painter.fillRect(inner_rectangle, QBrush(inner_gradient))
        painter.setPen(self._wall_pen(QColor("#65a5ff")))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(inner_rectangle)

    def _inner_rectangle(self) -> QRectF:
        inner_left_meters = self._geometry.inner_center_x_meters - self._settings.inner_width_meters * 0.5
        inner_top_meters = self._geometry.inner_center_y_meters - self._settings.inner_height_meters * 0.5
        return QRectF(
            inner_left_meters * self._scale,
            inner_top_meters * self._scale,
            self._settings.inner_width_meters * self._scale,
            self._settings.inner_height_meters * self._scale,
        )

    def _draw_gazebo_narrowing_walls(self, painter: QPainter) -> None:
        center_y = self._settings.outer_height_meters * 0.5
        obstacle_height = self._settings.narrow_obstacle_length_meters
        obstacle_half_height = obstacle_height * 0.5
        obstacle_top = center_y - obstacle_half_height
        obstacle_bottom = center_y + obstacle_half_height

        inner_right = self._settings.outer_width_meters * 0.5 + self._settings.inner_width_meters * 0.5
        inner_cap_x = inner_right + self._settings.narrow_obstacle_depth_meters
        outer_cap_x = self._settings.outer_width_meters - self._settings.narrow_obstacle_depth_meters
        outer_right = self._settings.outer_width_meters

        obstacle_pen = self._wall_pen(QColor("#c96dec"))
        painter.setPen(obstacle_pen)
        painter.setBrush(Qt.NoBrush)

        self._draw_u_wall(painter, inner_right, inner_cap_x, obstacle_top, obstacle_bottom, False)
        self._draw_u_wall(painter, outer_cap_x, outer_right, obstacle_top, obstacle_bottom, True)

        lane_rectangle = QRectF(
            inner_cap_x * self._scale,
            obstacle_top * self._scale,
            (outer_cap_x - inner_cap_x) * self._scale,
            obstacle_height * self._scale,
        )
        painter.fillRect(lane_rectangle, QBrush(QColor(255, 193, 79, 16)))
        painter.setPen(QPen(QColor(255, 193, 79, 100), 2.0, Qt.DashLine))
        painter.drawRect(lane_rectangle)

    def _draw_u_wall(self, painter: QPainter, attached_x_meters: float, cap_x_meters: float, top_meters: float, bottom_meters: float, attached_to_outer_wall: bool) -> None:
        attached_x_pixels = attached_x_meters * self._scale
        cap_x_pixels = cap_x_meters * self._scale
        top_pixels = top_meters * self._scale
        bottom_pixels = bottom_meters * self._scale

        painter.drawLine(QLineF(cap_x_pixels, top_pixels, cap_x_pixels, bottom_pixels))
        painter.drawLine(QLineF(attached_x_pixels, top_pixels, cap_x_pixels, top_pixels))
        painter.drawLine(QLineF(attached_x_pixels, bottom_pixels, cap_x_pixels, bottom_pixels))

    def _wall_pen(self, color: QColor) -> QPen:
        # Double only the rendered outlines (blue and purple walls).
        # The Gazebo wall thickness and collision geometry stay unchanged.
        wall_width_pixels = self._settings.wall_thickness_meters * self._scale
        wall_width_pixels = 2.0 * max(1.0, wall_width_pixels)
        return QPen(color, wall_width_pixels, Qt.SolidLine, Qt.SquareCap, Qt.MiterJoin)
