import math

from anomaly_graphing.data.model.resampled_scenario import ResampledScenario
from anomaly_graphing.data.processing.fused_anomaly_calculator import FusedAnomalyCalculator
from anomaly_graphing.data.processing.gps_anomaly_calculator import GpsAnomalyCalculator
from anomaly_graphing.data.processing.lidar_anomaly_calculator import LidarAnomalyCalculator


def _build_scenario(with_narrowing: bool) -> ResampledScenario:
    point_count = 160
    leader_positions = []
    follower_positions = []
    leader_lidar = []
    follower_lidar = []

    for index in range(point_count):
        angle = 2.0 * math.pi * index / point_count
        x_value = 12.0 * math.cos(angle)
        y_value = 12.0 * math.sin(angle)
        follower_x = 10.2 * math.cos(angle)
        follower_y = 10.2 * math.sin(angle)

        lidar = [8.0] * 36
        follower_scan = [8.2] * 36

        if with_narrowing and 42 <= index <= 66:
            x_value -= 2.0
            follower_x -= 2.8
            for sector_index in range(8, 18):
                lidar[sector_index] = 2.2
                follower_scan[sector_index] = 1.8

        leader_positions.append((x_value, y_value))
        follower_positions.append((follower_x, follower_y))
        leader_lidar.append(tuple(lidar))
        follower_lidar.append(tuple(follower_scan))

    return ResampledScenario(
        tuple(leader_positions),
        tuple(follower_positions),
        tuple(leader_lidar),
        tuple(follower_lidar),
    )


def test_narrow_section_creates_high_modality_and_fused_anomaly():
    normal = _build_scenario(False)
    follow = _build_scenario(True)

    gps = GpsAnomalyCalculator().calculate(normal, follow)
    lidar = LidarAnomalyCalculator().calculate(normal, follow)
    fused = FusedAnomalyCalculator().calculate(gps, lidar)

    narrow_slice = slice(42, 67)
    outside_indices = list(range(0, 35)) + list(range(75, 150))

    gps_inside = sum(gps[narrow_slice]) / len(gps[narrow_slice])
    lidar_inside = sum(lidar[narrow_slice]) / len(lidar[narrow_slice])
    fused_inside = sum(fused[narrow_slice]) / len(fused[narrow_slice])
    gps_outside = sum(gps[index] for index in outside_indices) / len(outside_indices)
    lidar_outside = sum(lidar[index] for index in outside_indices) / len(outside_indices)

    assert gps_inside > gps_outside + 0.45
    assert lidar_inside > lidar_outside + 0.45
    assert fused_inside > 0.55
