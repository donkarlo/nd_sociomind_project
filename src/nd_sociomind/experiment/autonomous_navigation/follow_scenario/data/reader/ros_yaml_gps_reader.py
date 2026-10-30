"""GPS trajectories from the same ROS timestamp stream as the LiDAR scans."""
from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory import Trajectory
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.lap_detector import LapDetector
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_yaml_lidar_reader import RosYamlLidarReader


class RosYamlGpsReader:
    """Read GPS odometry for both robots on one common recording clock."""

    def __init__(self, settings: ScenarioSettings):
        self._settings = settings
        self.leader_lap_complete = False
        self.follower_lap_complete = False

    def read_first_lap(self, path: Path) -> TrajectoryPair:
        leader = Trajectory()
        follower = Trajectory()
        trajectories = {"leader": leader, "follower": follower}
        lap_detector = LapDetector(self._settings)
        # The follower's recorded inner circuit is shorter than the leader's
        # outer circuit (~46 m vs. >55 m). Detect its *own* closure using GPS.
        follower_detector = LapDetector(replace(
            self._settings,
            minimum_lap_distance_meters=self._settings.minimum_lap_distance_meters * 0.65,
            lap_closure_radius_meters=max(0.12, self._settings.minimum_sample_spacing_meters * 1.5),
        ))
        leader_lap_complete = False
        follower_lap_complete = False

        for topic, stamp, fields in RosYamlLidarReader()._messages(path):
            if topic not in RosYamlLidarReader.GPS_TOPICS or stamp is None:
                continue
            role = RosYamlLidarReader.GPS_TOPICS[topic]
            if (role == "leader" and leader_lap_complete) or (
                role == "follower" and follower_lap_complete
            ):
                continue
            position = fields.get("position") or {}
            if not all(k in position for k in ("x", "y")):
                continue
            sample = PositionSample(stamp, position["x"], position["y"])
            trajectory = trajectories[role]
            if trajectory.samples and (
                sample.distance_to(trajectory.samples[-1]) <
                self._settings.minimum_sample_spacing_meters
            ):
                continue
            trajectory.append(sample)
            if role == "leader" and lap_detector.observe(sample):
                leader_lap_complete = True
            elif role == "follower" and follower_detector.observe(sample):
                follower_lap_complete = True
            if leader_lap_complete and follower_lap_complete:
                break

        if leader.length() < 20 or follower.length() < 20:
            raise ValueError("Not enough recorded ROS odometry for both UAVs.")
        # The compact recording is time-bounded. Never claim a closed lap if
        # its last sample precedes the leader's true return to its start.
        self.leader_lap_complete = leader_lap_complete
        self.follower_lap_complete = follower_lap_complete
        return TrajectoryPair(leader, follower)
