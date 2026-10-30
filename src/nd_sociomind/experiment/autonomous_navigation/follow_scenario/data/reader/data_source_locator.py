from pathlib import Path

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.data_source_settings import DataSourceSettings


class DataSourceLocator:
    def __init__(self, settings: DataSourceSettings):
        self._settings = settings

    def locate(self) -> tuple[Path, Path]:
        for leader_path, follower_path in self._settings.candidate_path_pairs():
            if leader_path.exists() and follower_path.exists():
                return leader_path, follower_path

        checked_paths: list[str] = []
        for leader_path, follower_path in self._settings.candidate_path_pairs():
            checked_paths.append(str(leader_path))
            checked_paths.append(str(follower_path))
        paths_text = "\n".join(checked_paths)
        raise FileNotFoundError(
            "The two cleaned GPS files were not found locally. Only these small files are required; "
            "the 3.3 GB ROS YAML file is not used. Checked:\n"
            f"{paths_text}"
        )
