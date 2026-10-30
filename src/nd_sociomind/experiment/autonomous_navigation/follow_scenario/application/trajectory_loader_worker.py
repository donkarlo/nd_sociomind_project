from pathlib import Path

from PySide6.QtCore import QObject, Signal, Slot

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.lidar_scan_clusterer import LidarScanClusterer
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.lidar_cluster_cache import LidarClusterCache
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.trajectory_synchronizer import TrajectorySynchronizer
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.world_coordinate_transformer import WorldCoordinateTransformer
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.cleaned_gps_reader import CleanedGpsReader
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_yaml_gps_reader import RosYamlGpsReader
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_yaml_lidar_reader import RosYamlLidarReader
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_replay_cache import RosReplayCache


class TrajectoryLoaderWorker(QObject):
    loaded = Signal(object)
    failed = Signal(str)
    finished = Signal()

    def __init__(self, leader_path: Path, follower_path: Path, reader: CleanedGpsReader, synchronizer: TrajectorySynchronizer, transformer: WorldCoordinateTransformer, lidar_path: Path | None = None):
        super().__init__()
        self._leader_path = leader_path
        self._follower_path = follower_path
        self._reader = reader
        self._synchronizer = synchronizer
        self._transformer = transformer
        self._lidar_path = lidar_path

    @Slot()
    def load(self) -> None:
        try:
            lidar_frames = {"leader": [], "follower": []}
            lidar_note = "ROS YAML unavailable (cleaned GPS replay only)"
            using_ros = self._lidar_path is not None
            leader_lap_complete = False
            replay_cache = None
            if using_ros:
                replay_cache = RosReplayCache(self._lidar_path, self._reader._scenario_settings)
                cached = replay_cache.load()
                if cached is not None:
                    trajectory_pair, lidar_frames, lidar_note = cached
                    clusters = self._cluster_frames(lidar_frames, replay_cache)
                    self.loaded.emit((trajectory_pair, lidar_frames, clusters, lidar_note + " · cached"))
                    return
                # Preserve the follower's entire recorded circuit for the route,
                # independently of the overlapping leader/follower replay time.
                ros_gps_reader = RosYamlGpsReader(self._reader._scenario_settings)
                recorded_pair = ros_gps_reader.read_first_lap(self._lidar_path)
                leader_lap_complete = ros_gps_reader.leader_lap_complete
                follower_lap_complete = ros_gps_reader.follower_lap_complete
                transformed_pair = self._transformer.transform(recorded_pair)
                trajectory_pair = self._synchronizer.synchronize(transformed_pair)
                if follower_lap_complete:
                    trajectory_pair.follower_full_route = transformed_pair.follower
            else:
                trajectory_pair = self._reader.read_first_lap(self._leader_path, self._follower_path)
                trajectory_pair = self._synchronizer.synchronize(trajectory_pair)
                trajectory_pair = self._transformer.transform(trajectory_pair)
            if using_ros:
                lidar_frames = RosYamlLidarReader().read_for_frames(
                    self._lidar_path, trajectory_pair,
                    scene_rotation_radians=self._transformer.rotation_radians,
                    same_ros_clock=True,
                )
                counts = {role: sum(scan is not None for scan in frames) for role, frames in lidar_frames.items()}
                lidar_note = f"Same-source ROS GPS + quaternion + LiDAR: UAV1 {counts['leader']:,}, UAV2 {counts['follower']:,}"
                if not all(counts.values()):
                    raise ValueError(f"No time-aligned LiDAR scan found for both UAVs: {counts}")
                if not leader_lap_complete:
                    lidar_note += " · leader recording ends before full lap closure"
                if not follower_lap_complete:
                    lidar_note += " · follower route is incomplete in the source"
                # Failure to write the cache must never prevent normal replay.
                try:
                    replay_cache.save(trajectory_pair, lidar_frames, lidar_note)
                except (OSError, ValueError, TypeError, OverflowError):
                    pass
            clusters = self._cluster_frames(lidar_frames, replay_cache)
            self.loaded.emit((trajectory_pair, lidar_frames, clusters, lidar_note))
        except Exception as exception:
            self.failed.emit(str(exception))
        finally:
            self.finished.emit()

    @staticmethod
    def _cluster_frames(lidar_frames: dict, replay_cache: RosReplayCache | None = None) -> dict:
        cached_result = LidarClusterCache(replay_cache) if replay_cache is not None else None
        if cached_result is not None:
            stored = cached_result.load(lidar_frames)
            if stored is not None:
                return stored
        clusterer = LidarScanClusterer()
        profiles = {role: clusterer.fit(lidar_frames.get(role, []))
                    for role in ("leader", "follower")}
        if cached_result is not None:
            try:
                cached_result.save(profiles)
            except (OSError, ValueError, TypeError, OverflowError):
                pass
        return profiles
