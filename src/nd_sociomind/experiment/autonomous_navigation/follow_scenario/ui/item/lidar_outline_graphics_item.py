"""Polygonal outlines from actual recorded LiDAR returns (not simulated rays)."""
import math

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainterPath, QPen
from PySide6.QtWidgets import QGraphicsPathItem

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.polygon_area_centroid import polygon_area_centroid
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_yaml_lidar_reader import LidarScan
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.item.lidar_cg_graphics_item import LidarCgGraphicsItem


class LidarOutlineGraphicsItem(QGraphicsPathItem):
    def __init__(self, color: QColor, pixels_per_meter: float):
        super().__init__()
        self._scale = pixels_per_meter
        self._last_scan: LidarScan | None = None
        pen = QPen(color)
        pen.setWidthF(1.6)
        pen.setCosmetic(True)
        self.setPen(pen)
        self.setBrush(Qt.NoBrush)
        self.setZValue(14.0)  # Arena below; draggable drone above.
        self.setAcceptedMouseButtons(Qt.NoButton)
        self._cg_item = LidarCgGraphicsItem(color, self)
        self._centroid_meters: tuple[float, float] | None = None

    def set_pose(self, x_pixels: float, y_pixels: float, heading_radians: float) -> None:
        self.setPos(x_pixels, y_pixels)
        self.setRotation(math.degrees(heading_radians))

    def set_scan(self, scan: LidarScan | None) -> None:
        if scan is self._last_scan:
            return
        self._last_scan = scan
        path = QPainterPath()
        previous_is_valid = False
        fully_valid = bool(scan and scan.points_meters and all(p is not None for p in scan.points_meters))
        # Only the genuinely closed rendered polygon encloses mass.
        # Partially missing/degenerate scans have no definable area centroid.
        centroid = None
        if scan is not None and scan.covers_full_circle and fully_valid:
            centroid = polygon_area_centroid(scan.points_meters)
        self._centroid_meters = centroid
        if centroid is None:
            self._cg_item.hide()
        else:
            self._cg_item.setPos(centroid[0] * self._scale, centroid[1] * self._scale)
            self._cg_item.show()
        if scan is not None:
            for point in scan.points_meters:
                if point is None:
                    previous_is_valid = False
                    continue  # Never join points across a missing LiDAR return.
                x, y = point[0] * self._scale, point[1] * self._scale
                if previous_is_valid:
                    path.lineTo(x, y)
                else:
                    path.moveTo(x, y)
                previous_is_valid = True
            if fully_valid and scan.covers_full_circle:
                path.closeSubpath()
        self.setPath(path)
