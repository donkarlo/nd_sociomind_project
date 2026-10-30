from dataclasses import dataclass
import os
from pathlib import Path


@dataclass(frozen=True)
class DataSourceSettings:
    leader_file_name: str = "uav-1-cleaned-gps-pos-vel.txt"
    follower_file_name: str = "uav-2-cleaned-gps-pos-vel.txt"
    fallback_sample_period_seconds: float = 0.02

    def candidate_directories(self) -> list[Path]:
        project_root = Path(__file__).resolve().parents[6]
        bundled_data_directory = project_root / "data" / "autonomous_navigation" / "follow_scenario"
        home_directory = Path.home()
        legacy_relative_directory = Path("phd/data/old_experiment_not_synced_dropbox/follow-scenario/gps")

        return [
            bundled_data_directory,
            home_directory / "Dropbox" / legacy_relative_directory,
            home_directory / legacy_relative_directory,
            Path("/") / legacy_relative_directory,
        ]

    def find_lidar_yaml(self) -> Path | None:
        # Follow-scenario ROS dump holds both UAVs' LaserScan topics.
        override = os.environ.get("ND_FOLLOW_ROS_YAML")
        relative = Path("phd/data/old_experiment_not_synced_dropbox/follow-scenario/uav1-gps-lidar-uav2-gps-lidar.yaml")
        bundled_first_lap = (Path(__file__).resolve().parents[6]
                             / "data/autonomous_navigation/follow_scenario/first_lap_ros_gps_lidar.yaml")
        candidates = ([Path(override).expanduser()] if override else []) + [
            bundled_first_lap,
            Path("/") / relative,
            Path.home() / relative,
            Path.home() / "Dropbox" / relative,
        ]
        return next((path for path in candidates if path.is_file()), None)

    def candidate_path_pairs(self) -> list[tuple[Path, Path]]:
        path_pairs: list[tuple[Path, Path]] = []
        for directory in self.candidate_directories():
            leader_path = directory / self.leader_file_name
            follower_path = directory / self.follower_file_name
            path_pairs.append((leader_path, follower_path))
        return path_pairs
