import math

from PySide6.QtCore import QObject, QTimer, Signal

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.domain.drone_role import DroneRole
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.interaction.learning.learned_relationship_model import LearnedRelationshipModel
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.interaction.safety.arena_safety_model import ArenaSafetyModel
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.playback.playback_controller import PlaybackController
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.ui.scene.follow_scenario_scene import FollowScenarioScene


class InteractiveFlightController(QObject):
    status_changed = Signal(str)

    def __init__(self, scene: FollowScenarioScene, playback: PlaybackController, trajectory_pair: TrajectoryPair, settings: ScenarioSettings, parent: QObject | None = None):
        super().__init__(parent)
        self._scene = scene
        self._playback = playback
        self._settings = settings
        self._relationship_model = LearnedRelationshipModel(trajectory_pair, settings)
        self._safety_model = ArenaSafetyModel(settings)
        self._mode = "idle"
        self._dragged_role = DroneRole.LEADER
        self._interaction_frame_index = 0
        self._recovery_frame_index = 0
        self._was_playing_before_drag = False
        self._positions = {
            DroneRole.LEADER: self._scene.position_meters(DroneRole.LEADER),
            DroneRole.FOLLOWER: self._scene.position_meters(DroneRole.FOLLOWER),
        }
        self._headings = {
            DroneRole.LEADER: self._scene.heading_radians(DroneRole.LEADER),
            DroneRole.FOLLOWER: self._scene.heading_radians(DroneRole.FOLLOWER),
        }
        self._timer = QTimer(self)
        self._timer.setInterval(max(1, int(1000 / self._settings.target_frames_per_second)))
        self._timer.timeout.connect(self._tick)
        self._scene.drone_drag_started.connect(self._start_drag)
        self._scene.drone_drag_moved.connect(self._move_drag)
        self._scene.drone_drag_finished.connect(self._finish_drag)

    def _start_drag(self, role: DroneRole, x_pixels: float, y_pixels: float) -> None:
        self._was_playing_before_drag = self._playback.is_playing()
        self._playback.pause()
        self._interaction_frame_index = self._playback.current_frame()
        self._dragged_role = role
        self._positions[DroneRole.LEADER] = self._scene.position_meters(DroneRole.LEADER)
        self._positions[DroneRole.FOLLOWER] = self._scene.position_meters(DroneRole.FOLLOWER)
        self._headings[DroneRole.LEADER] = self._scene.heading_radians(DroneRole.LEADER)
        self._headings[DroneRole.FOLLOWER] = self._scene.heading_radians(DroneRole.FOLLOWER)
        self._scene.begin_interaction()
        self._mode = "dragging"
        self._timer.start()
        self.status_changed.emit("Interactive control · the other drone is coordinating from the recorded GPS relationship")

    def _move_drag(self, role: DroneRole, x_pixels: float, y_pixels: float) -> None:
        if self._mode != "dragging":
            return
        if role != self._dragged_role:
            return

        scale = self._settings.world_pixels_per_meter
        desired_x = x_pixels / scale
        desired_y = y_pixels / scale
        current_x, current_y = self._positions[role]
        other_x, other_y = self._positions[role.other()]
        constrained_x, constrained_y = self._safety_model.constrain_motion(current_x, current_y, desired_x, desired_y, other_x, other_y)
        heading = self._heading_from_motion(current_x, current_y, constrained_x, constrained_y, self._headings[role])
        self._positions[role] = (constrained_x, constrained_y)
        self._headings[role] = heading
        self._scene.set_interactive_pose(role, constrained_x, constrained_y, heading)

    def _finish_drag(self, role: DroneRole, x_pixels: float, y_pixels: float) -> None:
        if self._mode != "dragging":
            return
        if role != self._dragged_role:
            return

        dragged_x, dragged_y = self._positions[self._dragged_role]
        other_role = self._dragged_role.other()
        other_x, other_y = self._positions[other_role]
        self._recovery_frame_index = self._relationship_model.recovery_frame(self._dragged_role, dragged_x, dragged_y, other_x, other_y)
        self._mode = "recovering"
        self.status_changed.emit("Returning gradually to the learned GPS path…")

    def _tick(self) -> None:
        if self._mode == "dragging":
            self._tick_dragging()
            return
        if self._mode == "recovering":
            self._tick_recovering()
            return
        self._timer.stop()

    def _tick_dragging(self) -> None:
        other_role = self._dragged_role.other()
        dragged_x, dragged_y = self._positions[self._dragged_role]
        current_other_x, current_other_y = self._positions[other_role]
        target_other_x, target_other_y = self._relationship_model.coordinated_target(self._interaction_frame_index, self._dragged_role, dragged_x, dragged_y)
        response_speed = self._relationship_model.learned_response_speed(self._interaction_frame_index, other_role, self._dragged_role)
        next_other_x, next_other_y = self._move_toward(current_other_x, current_other_y, target_other_x, target_other_y, response_speed)
        safe_other_x, safe_other_y = self._safety_model.constrain_motion(current_other_x, current_other_y, next_other_x, next_other_y, dragged_x, dragged_y)
        heading = self._heading_from_motion(current_other_x, current_other_y, safe_other_x, safe_other_y, self._headings[other_role])
        self._positions[other_role] = (safe_other_x, safe_other_y)
        self._headings[other_role] = heading
        self._scene.set_interactive_pose(other_role, safe_other_x, safe_other_y, heading)

    def _tick_recovering(self) -> None:
        leader_target_x, leader_target_y = self._relationship_model.nominal_position(DroneRole.LEADER, self._recovery_frame_index)
        follower_target_x, follower_target_y = self._relationship_model.nominal_position(DroneRole.FOLLOWER, self._recovery_frame_index)
        leader_x, leader_y = self._positions[DroneRole.LEADER]
        follower_x, follower_y = self._positions[DroneRole.FOLLOWER]

        leader_speed = self._relationship_model.learned_response_speed(self._recovery_frame_index, DroneRole.LEADER, DroneRole.FOLLOWER) * self._settings.interaction_recovery_speed_scale
        follower_speed = self._relationship_model.learned_response_speed(self._recovery_frame_index, DroneRole.FOLLOWER, DroneRole.LEADER) * self._settings.interaction_recovery_speed_scale

        next_leader_x, next_leader_y = self._move_toward(leader_x, leader_y, leader_target_x, leader_target_y, leader_speed)
        safe_leader_x, safe_leader_y = self._safety_model.constrain_motion(leader_x, leader_y, next_leader_x, next_leader_y, follower_x, follower_y)
        next_follower_x, next_follower_y = self._move_toward(follower_x, follower_y, follower_target_x, follower_target_y, follower_speed)
        safe_follower_x, safe_follower_y = self._safety_model.constrain_motion(follower_x, follower_y, next_follower_x, next_follower_y, safe_leader_x, safe_leader_y)

        leader_heading = self._heading_from_motion(leader_x, leader_y, safe_leader_x, safe_leader_y, self._headings[DroneRole.LEADER])
        follower_heading = self._heading_from_motion(follower_x, follower_y, safe_follower_x, safe_follower_y, self._headings[DroneRole.FOLLOWER])

        self._positions[DroneRole.LEADER] = (safe_leader_x, safe_leader_y)
        self._positions[DroneRole.FOLLOWER] = (safe_follower_x, safe_follower_y)
        self._headings[DroneRole.LEADER] = leader_heading
        self._headings[DroneRole.FOLLOWER] = follower_heading
        self._scene.set_interactive_pose(DroneRole.LEADER, safe_leader_x, safe_leader_y, leader_heading)
        self._scene.set_interactive_pose(DroneRole.FOLLOWER, safe_follower_x, safe_follower_y, follower_heading)

        leader_error = math.hypot(safe_leader_x - leader_target_x, safe_leader_y - leader_target_y)
        follower_error = math.hypot(safe_follower_x - follower_target_x, safe_follower_y - follower_target_y)
        if leader_error <= self._settings.interaction_completion_distance_meters and follower_error <= self._settings.interaction_completion_distance_meters:
            self._complete_recovery()

    def _complete_recovery(self) -> None:
        self._timer.stop()
        self._mode = "idle"
        self._scene.end_interaction()
        self._playback.set_frame(self._recovery_frame_index)
        if self._was_playing_before_drag:
            self._playback.play()
        self.status_changed.emit("Interactive replay ready · drag either drone with the mouse")

    def _move_toward(self, start_x: float, start_y: float, target_x: float, target_y: float, speed_meters_per_second: float) -> tuple[float, float]:
        delta_x = target_x - start_x
        delta_y = target_y - start_y
        distance = math.hypot(delta_x, delta_y)
        if distance <= 1e-9:
            return start_x, start_y

        seconds_per_tick = 1.0 / float(self._settings.target_frames_per_second)
        maximum_step = max(0.001, speed_meters_per_second * seconds_per_tick)
        if distance <= maximum_step:
            return target_x, target_y

        scale = maximum_step / distance
        return start_x + delta_x * scale, start_y + delta_y * scale

    def _heading_from_motion(self, start_x: float, start_y: float, end_x: float, end_y: float, fallback_heading: float) -> float:
        delta_x = end_x - start_x
        delta_y = end_y - start_y
        if math.hypot(delta_x, delta_y) < 1e-5:
            return fallback_heading
        return math.atan2(delta_y, delta_x)
