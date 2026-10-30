"""720-beam ROS LiDAR clipping and per-UAV temporal DBSCAN tests."""
import math
from pathlib import Path
import tempfile
import unittest

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.lidar_cluster_cache import LidarClusterCache
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_replay_cache import RosReplayCache
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.lidar_scan_clusterer import LidarScanClusterer
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_yaml_lidar_reader import LidarScan, RosYamlLidarReader


def scan_at(timestamp: float, radius: float) -> LidarScan:
    points = tuple(
        (radius * math.cos(i * 2 * math.pi / 720),
         radius * math.sin(i * 2 * math.pi / 720))
        for i in range(720)
    )
    return LidarScan(timestamp, points, True, 0.0)


class Lidar720ClusteringTest(unittest.TestCase):
    def test_original_720_beams_and_range_clipping(self):
        distances = [2.5] * 720
        distances[0] = float("inf")
        distances[1] = 25.0
        distances[2] = float("nan")
        reader = RosYamlLidarReader()
        result = reader._scan(0.0, {
            "ranges": distances, "angle_min": 0.0,
            "angle_increment": 2 * math.pi / 720,
            "range_min": 0.1, "range_max": 15.0,
        })
        self.assertEqual(len(result.points_meters), 720)
        self.assertAlmostEqual(math.hypot(*result.points_meters[0]), 5.0)
        self.assertAlmostEqual(math.hypot(*result.points_meters[1]), 5.0)
        self.assertIsNone(result.points_meters[2])

    def test_two_orders_and_associated_medoid_polygons(self):
        scans = [scan_at(i * 0.1, 2.0 + i * 0.05) for i in range(8)]
        frames = [scans[0], *scans, scans[-1]]
        profiles = LidarScanClusterer().fit(frames)
        self.assertEqual(len(profiles.zeroth.labels_by_frame), len(frames))
        self.assertEqual(len(profiles.first.labels_by_frame), len(frames))
        self.assertIsNone(profiles.first.labels_by_frame[0])
        self.assertIsNone(profiles.first.labels_by_frame[1])
        self.assertTrue(profiles.zeroth.outlines_by_label)
        self.assertTrue(profiles.first.outlines_by_label)
        for order in (profiles.zeroth, profiles.first):
            self.assertTrue(all(len(points) == 720 for points in order.outlines_by_label.values()))
        self.assertEqual(profiles.zeroth.labels_by_frame[0], profiles.zeroth.labels_by_frame[1])

    def test_missing_scans_are_not_clusters(self):
        profiles = LidarScanClusterer().fit([None, scan_at(0.0, 2.0), None])
        self.assertEqual(profiles.zeroth.labels_by_frame, (None, -1, None))
        self.assertEqual(profiles.first.labels_by_frame, (None, None, None))


class LidarClusterCacheTest(unittest.TestCase):
    def test_round_trip_and_source_invalidation(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "source.yaml"
            source.write_text("source 1")
            cache = LidarClusterCache(RosReplayCache(source, ScenarioSettings(), Path(folder)))
            scans = [scan_at(i * 0.1, 2.0 + 0.05 * i) for i in range(6)]
            frames = {"leader": scans, "follower": scans}
            profiles = {role: LidarScanClusterer().fit(scans) for role in frames}
            self.assertIsNone(cache.load(frames))
            cache.save(profiles)
            self.assertEqual(cache.load(frames), profiles)
            source.write_text("source changed")
            self.assertIsNone(cache.load(frames))


if __name__ == "__main__":
    unittest.main()
