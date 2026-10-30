import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import QGraphicsItem, QGraphicsScene, QGraphicsSimpleTextItem

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.corridor_geometry_estimator import CorridorGeometryEstimator
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.drone_role import DroneRole
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.follow_phase_estimator import FollowPhaseEstimator
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.flight_phase import FlightPhase
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.item.corridor_graphics_item import CorridorGraphicsItem
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.item.drone_graphics_item import DroneGraphicsItem
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.item.drone_label_graphics_item import DroneLabelGraphicsItem
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.item.lidar_outline_graphics_item import LidarOutlineGraphicsItem
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.item.lidar_cluster_outline_graphics_item import LidarClusterOutlineGraphicsItem
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.item.trajectory_graphics_item import TrajectoryGraphicsItem


class FollowScenarioScene(QGraphicsScene):
    drone_drag_started = Signal(object, float, float)
    drone_drag_moved = Signal(object, float, float)
    drone_drag_finished = Signal(object, float, float)

    def __init__(self, settings: ScenarioSettings, parent=None):
        super().__init__(parent)
        self._settings = settings
        self._trajectory_pair: TrajectoryPair | None = None
        self._interaction_active = False
        self._phase_estimator = FollowPhaseEstimator()
        self._corridor_geometry_estimator = CorridorGeometryEstimator(settings)
        self._corridor_item = CorridorGraphicsItem(settings)
        self._leader_item = DroneGraphicsItem(DroneRole.LEADER, QColor("#50a7ff"), settings.drone_visual_scale)
        self._follower_item = DroneGraphicsItem(DroneRole.FOLLOWER, QColor("#ffb84d"), settings.drone_visual_scale)
        self._leader_label_item = DroneLabelGraphicsItem("Leader", QColor("#d9ecff"))
        self._follower_label_item = DroneLabelGraphicsItem("Follower", QColor("#ffe6b9"))
        self._lidar_frames = {"leader": [], "follower": []}
        self._lidar_items = {
            DroneRole.LEADER: LidarOutlineGraphicsItem(QColor("#4ddcff"), settings.world_pixels_per_meter),
            DroneRole.FOLLOWER: LidarOutlineGraphicsItem(QColor("#ffc868"), settings.world_pixels_per_meter),
        }
        self._lidar_clusters = {}
        # Green: zeroth derivative; magenta: temporal first derivative.
        self._cluster_items = {
            (role, order): LidarClusterOutlineGraphicsItem(
                QColor("#50e58b" if order == 0 else "#ff58aa"),
                settings.world_pixels_per_meter,
                cluster_style=Qt.SolidLine if order == 0 else Qt.DashLine)
            for role in (DroneRole.LEADER, DroneRole.FOLLOWER)
            for order in (0, 1)
        }
        self._headings = {
            DroneRole.LEADER: 0.0,
            DroneRole.FOLLOWER: 0.0,
        }
        self.addItem(self._corridor_item)
        self._cg_legend_item = QGraphicsSimpleTextItem(
            "× CG = center of gravity (uniform mass density)"
        )
        font = QFont()
        font.setPixelSize(11)
        self._cg_legend_item.setFont(font)
        self._cg_legend_item.setBrush(QColor("#c5d6ed"))
        self._cg_legend_item.setZValue(50.0)
        self._cg_legend_item.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self._cg_legend_item.setAcceptedMouseButtons(Qt.NoButton)
        self.addItem(self._cg_legend_item)
        self._cluster_legend_item = QGraphicsSimpleTextItem(
            "DBSCAN: green = LiDAR ranges (0th), pink = temporal range derivative (1st); dotted = unclustered scan"
        )
        self._cluster_legend_item.setFont(font)
        self._cluster_legend_item.setBrush(QColor("#d4ddec"))
        self._cluster_legend_item.setZValue(50.0)
        self._cluster_legend_item.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self._cluster_legend_item.setAcceptedMouseButtons(Qt.NoButton)
        self.addItem(self._cluster_legend_item)
        for lidar_item in self._lidar_items.values():
            self.addItem(lidar_item)
        for cluster_item in self._cluster_items.values():
            self.addItem(cluster_item)
            cluster_item.hide()
        self.addItem(self._leader_item)
        self.addItem(self._follower_item)
        self.addItem(self._leader_label_item)
        self.addItem(self._follower_label_item)
        self._leader_item.drag_started.connect(self._forward_drag_started)
        self._leader_item.drag_moved.connect(self._forward_drag_moved)
        self._leader_item.drag_finished.connect(self._forward_drag_finished)
        self._follower_item.drag_started.connect(self._forward_drag_started)
        self._follower_item.drag_moved.connect(self._forward_drag_moved)
        self._follower_item.drag_finished.connect(self._forward_drag_finished)
        self._configure_scene_rectangle()

    def _configure_scene_rectangle(self) -> None:
        scale = self._settings.world_pixels_per_meter
        width_pixels = self._settings.outer_width_meters * scale
        height_pixels = self._settings.outer_height_meters * scale
        margin_pixels = self._settings.scene_visual_margin_meters * scale
        self.setSceneRect(-margin_pixels, -margin_pixels, width_pixels + 2.0 * margin_pixels, height_pixels + 2.0 * margin_pixels)
        # Scene legend sits above the arena, inside the existing visual margin.
        self._cg_legend_item.setPos(3.0, -margin_pixels + 2.0)
        self._cluster_legend_item.setPos(3.0, -margin_pixels + 15.0)

    def set_lidar_frames(self, frames: dict) -> None:
        self._lidar_frames = frames

    def set_lidar_clusters(self, profiles: dict) -> None:
        self._lidar_clusters = profiles

    def _update_lidar_scan(self, role: DroneRole, frame_index: int) -> None:
        role_key = "leader" if role == DroneRole.LEADER else "follower"
        role_frames = self._lidar_frames.get(role_key, [])
        scan = role_frames[frame_index] if frame_index < len(role_frames) else None
        self._lidar_items[role].set_scan(scan)
        cluster_profiles = self._lidar_clusters.get(role_key)
        for order in (0, 1):
            item = self._cluster_items[(role, order)]
            if cluster_profiles is None:
                item.set_contour(None)
                continue
            profiles = cluster_profiles.zeroth if order == 0 else cluster_profiles.first
            contour, label = profiles.outline_for(frame_index)
            # Noise (-1) and the first undefined derivative remain visible as
            # dashed recorded outlines, never misleadingly as learned clusters.
            unclustered = contour is None
            if unclustered and scan is not None and len(scan.points_meters) == 720:
                contour = scan.points_meters
            item.set_contour(contour, noise=unclustered)

    def set_trajectories(self, trajectory_pair: TrajectoryPair) -> None:
        self._trajectory_pair = trajectory_pair
        corridor_geometry = self._corridor_geometry_estimator.estimate(trajectory_pair)
        self._corridor_item.set_geometry(corridor_geometry)
        self.addItem(TrajectoryGraphicsItem(trajectory_pair.leader, self._settings.world_pixels_per_meter, QColor("#50a7ff"), close_loop=True))
        # The follower finishes its own closed GPS circuit a few seconds after
        # the leader's replay ends. Draw the full *recorded* route, but keep
        # synchronized positions/time for both moving drone sprites.
        follower_route = trajectory_pair.follower_full_route or trajectory_pair.follower
        self.addItem(TrajectoryGraphicsItem(
            follower_route, self._settings.world_pixels_per_meter,
            QColor("#ffb84d"), close_loop=trajectory_pair.follower_full_route is not None,
        ))
        self.set_frame(0)

    def frame_count(self) -> int:
        if self._trajectory_pair is None:
            return 0
        return min(self._trajectory_pair.leader.length(), self._trajectory_pair.follower.length())

    def begin_interaction(self) -> None:
        self._interaction_active = True
        # Manually dragged poses were not observed by the recorded LiDAR.
        for item in (*self._lidar_items.values(), *self._cluster_items.values()):
            item.hide()

    def end_interaction(self) -> None:
        self._interaction_active = False
        for item in self._lidar_items.values():
            item.show()
        # Cluster contours resume only for actual recorded playback frames.
        for item in self._cluster_items.values():
            if item.path().elementCount():
                item.show()

    def position_meters(self, role: DroneRole) -> tuple[float, float]:
        item = self._drone_item(role)
        scale = self._settings.world_pixels_per_meter
        return item.pos().x() / scale, item.pos().y() / scale

    def heading_radians(self, role: DroneRole) -> float:
        return self._headings[role]

    def set_interactive_pose(self, role: DroneRole, x_meters: float, y_meters: float, heading_radians: float) -> None:
        scale = self._settings.world_pixels_per_meter
        x_pixels = x_meters * scale
        y_pixels = y_meters * scale
        item = self._drone_item(role)
        label = self._label_item(role)
        item.set_pose(x_pixels, y_pixels, heading_radians)
        self._lidar_items[role].set_pose(x_pixels, y_pixels, heading_radians)
        for order in (0, 1):
            cluster_item = self._cluster_items[(role, order)]
            cluster_item.setPos(x_pixels, y_pixels)
            cluster_item.setRotation(math.degrees(heading_radians))
        label.set_anchor_position(x_pixels, y_pixels)
        self._headings[role] = heading_radians

    def set_frame(self, frame_index: int) -> tuple[FlightPhase, float]:
        if self._trajectory_pair is None or self.frame_count() == 0:
            return FlightPhase.NORMAL_SIDE_BY_SIDE, 0.0

        if self._interaction_active:
            leader_x, leader_y = self.position_meters(DroneRole.LEADER)
            follower_x, follower_y = self.position_meters(DroneRole.FOLLOWER)
            separation = math.hypot(leader_x - follower_x, leader_y - follower_y)
            return FlightPhase.NORMAL_SIDE_BY_SIDE, separation

        frame_index = max(0, min(frame_index, self.frame_count() - 1))
        leader = self._trajectory_pair.leader.samples[frame_index]
        follower = self._trajectory_pair.follower.samples[frame_index]
        previous_index = max(0, frame_index - 1)
        next_index = min(self.frame_count() - 1, frame_index + 1)
        previous_leader = self._trajectory_pair.leader.samples[previous_index]
        next_leader = self._trajectory_pair.leader.samples[next_index]
        previous_follower = self._trajectory_pair.follower.samples[previous_index]
        next_follower = self._trajectory_pair.follower.samples[next_index]

        leader_heading = math.atan2(next_leader.y_meters - previous_leader.y_meters, next_leader.x_meters - previous_leader.x_meters)
        follower_heading = math.atan2(next_follower.y_meters - previous_follower.y_meters, next_follower.x_meters - previous_follower.x_meters)
        # ROS quaternion orientation is the actual body/sensor yaw. Velocity
        # direction is only a fallback when recorded orientation is absent.
        for role in (DroneRole.LEADER, DroneRole.FOLLOWER):
            key = "leader" if role == DroneRole.LEADER else "follower"
            scans = self._lidar_frames.get(key, [])
            if frame_index < len(scans) and scans[frame_index] is not None:
                recorded = scans[frame_index].heading_radians
                if recorded is not None:
                    if role == DroneRole.LEADER:
                        leader_heading = recorded
                    else:
                        follower_heading = recorded
        self.set_interactive_pose(DroneRole.LEADER, leader.x_meters, leader.y_meters, leader_heading)
        self.set_interactive_pose(DroneRole.FOLLOWER, follower.x_meters, follower.y_meters, follower_heading)
        self._update_lidar_scan(DroneRole.LEADER, frame_index)
        self._update_lidar_scan(DroneRole.FOLLOWER, frame_index)

        phase = self._phase_estimator.estimate(previous_leader, leader, follower)
        separation = leader.distance_to(follower)
        return phase, separation

    def current_separation_meters(self) -> float:
        leader_x, leader_y = self.position_meters(DroneRole.LEADER)
        follower_x, follower_y = self.position_meters(DroneRole.FOLLOWER)
        return math.hypot(leader_x - follower_x, leader_y - follower_y)

    def _drone_item(self, role: DroneRole) -> DroneGraphicsItem:
        if role == DroneRole.LEADER:
            return self._leader_item
        return self._follower_item

    def _label_item(self, role: DroneRole) -> DroneLabelGraphicsItem:
        if role == DroneRole.LEADER:
            return self._leader_label_item
        return self._follower_label_item

    def _forward_drag_started(self, role: DroneRole, x_pixels: float, y_pixels: float) -> None:
        self.drone_drag_started.emit(role, x_pixels, y_pixels)

    def _forward_drag_moved(self, role: DroneRole, x_pixels: float, y_pixels: float) -> None:
        self.drone_drag_moved.emit(role, x_pixels, y_pixels)

    def _forward_drag_finished(self, role: DroneRole, x_pixels: float, y_pixels: float) -> None:
        self.drone_drag_finished.emit(role, x_pixels, y_pixels)
