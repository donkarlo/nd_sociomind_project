from dataclasses import dataclass


@dataclass(frozen=True)
class ScenarioSettings:
    outer_width_meters: float = 32.0
    outer_height_meters: float = 32.0
    inner_width_meters: float = 8.0
    inner_height_meters: float = 8.0
    wall_thickness_meters: float = 0.05
    narrow_lane_width_meters: float = 4.0
    narrow_obstacle_depth_meters: float = 4.0
    narrow_obstacle_length_meters: float = 3.99
    desired_follow_distance_meters: float = 5.0
    minimum_lap_distance_meters: float = 55.0
    departure_radius_meters: float = 8.0
    lap_closure_radius_meters: float = 2.5
    minimum_sample_spacing_meters: float = 0.08
    default_speed_multiplier: float = 1.0
    minimum_speed_multiplier: float = 0.1
    maximum_speed_multiplier: float = 80.0
    target_frames_per_second: int = 60
    world_pixels_per_meter: float = 28.0
    drone_visual_scale: float = 0.408
    drone_outer_radius_meters: float = 0.495
    minimum_wall_clearance_meters: float = 0.20
    minimum_inter_drone_clearance_meters: float = 0.20
    interaction_motion_step_meters: float = 0.04
    interaction_learning_window_frames: int = 24
    interaction_minimum_response_speed_meters_per_second: float = 0.55
    interaction_maximum_response_speed_meters_per_second: float = 4.0
    interaction_recovery_speed_scale: float = 0.85
    interaction_completion_distance_meters: float = 0.06
    scene_visual_margin_meters: float = 1.5
