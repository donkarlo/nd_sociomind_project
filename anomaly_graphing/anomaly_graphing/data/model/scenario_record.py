from dataclasses import dataclass

from anomaly_graphing.data.model.lidar_sample import LidarSample
from anomaly_graphing.data.model.position_sample import PositionSample


@dataclass(frozen=True)
class ScenarioRecord:
    leader_positions: tuple[PositionSample, ...]
    follower_positions: tuple[PositionSample, ...]
    leader_lidar: tuple[LidarSample, ...]
    follower_lidar: tuple[LidarSample, ...]
