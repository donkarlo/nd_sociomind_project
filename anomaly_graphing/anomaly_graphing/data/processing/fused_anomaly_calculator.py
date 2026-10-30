class FusedAnomalyCalculator:
    def calculate(self, gps_anomaly: tuple[float, ...], lidar_anomaly: tuple[float, ...]) -> tuple[float, ...]:
        point_count = min(len(gps_anomaly), len(lidar_anomaly))
        return tuple(
            0.5 * gps_anomaly[index] + 0.5 * lidar_anomaly[index]
            for index in range(point_count)
        )
