"""ROS replay cache: exact GPS, quaternion/LiDAR, closure, invalidation."""
from __future__ import annotations

import math
from pathlib import Path
import tempfile
import unittest

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory import Trajectory
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_replay_cache import RosReplayCache
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_yaml_lidar_reader import LidarScan


class RosReplayCacheTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.yaml = root / "recording.yaml"
        self.yaml.write_text("fake yaml source for cache invalidation\n")
        self.cache = RosReplayCache(self.yaml, ScenarioSettings(), root / "cache")

    def _payload(self):
        leader = Trajectory([PositionSample(2.0 + i, i * 0.2, 1.0) for i in range(3)])
        follower = Trajectory([PositionSample(2.0 + i, 5.0, i * 0.3) for i in range(3)])
        full = Trajectory(follower.samples + [PositionSample(6.0, 5.0, 0.0)])
        pair = TrajectoryPair(leader, follower, full)
        scan_a = LidarScan(2.01, ((0.0, 0.0), (5.0, 0.0), None, (2.0, 3.0)),
                           True, -math.pi / 2)
        scan_b = LidarScan(3.02, ((0.2, 0.1), (2.3, 1.1)), True, math.pi / 3)
        frames = {
            "leader": [scan_a, scan_a, scan_b],
            "follower": [None, scan_b, scan_a],
        }
        return pair, frames, "Real GPS + LiDAR"

    def test_round_trip_and_shared_scan_identity(self):
        pair, frames, note = self._payload()
        self.assertIsNone(self.cache.load())
        self.cache.save(pair, frames, note)
        self.assertTrue(self.cache.cache_path.exists())
        actual_pair, actual_frames, actual_note = self.cache.load()
        self.assertEqual(actual_pair, pair)
        self.assertEqual(actual_note, note)
        self.assertEqual(actual_frames, frames)
        self.assertIs(actual_frames["leader"][0], actual_frames["leader"][1])
        self.assertIsNone(actual_frames["follower"][0])
        self.assertEqual(actual_frames["leader"][0].heading_radians, -math.pi / 2)

    def test_source_and_settings_changes_invalidate(self):
        self.cache.save(*self._payload())
        self.assertIsNotNone(self.cache.load())
        self.yaml.write_text("replaced yaml content\n")
        self.assertIsNone(self.cache.load())
        changed_settings = ScenarioSettings(world_pixels_per_meter=99.0)
        self.assertIsNone(RosReplayCache(self.yaml, changed_settings,
                                         self.cache.cache_path.parent).load())

    def test_corrupt_cache_is_ignored(self):
        self.cache.save(*self._payload())
        self.cache.cache_path.write_bytes(b"corrupted")
        self.assertIsNone(self.cache.load())


if __name__ == "__main__":
    unittest.main()
