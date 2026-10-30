from dataclasses import dataclass


@dataclass(frozen=True)
class ResampledScenario:
    leader_positions: tuple[tuple[float, float], ...]
    follower_positions: tuple[tuple[float, float], ...]
    leader_lidar: tuple[tuple[float, ...], ...]
    follower_lidar: tuple[tuple[float, ...], ...]
