"""Safe, compressed, automatically invalidated cache for Follow ROS replay.

The source YAML is only parsed when its file metadata, relevant processing code,
or settings change. Cache lives outside Dropbox and holds no executable pickle.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import tempfile
import zlib

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory import Trajectory
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_yaml_lidar_reader import LidarScan, RosYamlLidarReader


class RosReplayCache:
    """Cache transformed, synchronized GPS and recorded LiDAR in a local file."""

    VERSION = 1
    # Hard upper bounds for a corrupted cache, without loading the ROS YAML.
    MAX_COMPRESSED_BYTES = 100 * 1024 * 1024
    MAX_DECOMPRESSED_BYTES = 256 * 1024 * 1024
    RELEVANT_MODULES = (
        "ros_replay_cache.py",
        "ros_yaml_gps_reader.py",
        "ros_yaml_lidar_reader.py",
        "../processing/trajectory_synchronizer.py",
        "../processing/world_coordinate_transformer.py",
        "../processing/lap_detector.py",
        "../model/position_sample.py",
        "../model/trajectory_pair.py",
        "../../config/scenario_settings.py",
        "../../application/trajectory_loader_worker.py",
    )

    def __init__(
        self,
        source: Path,
        settings: ScenarioSettings,
        cache_directory: Path | None = None,
    ) -> None:
        self.source = source.resolve()
        self.settings = settings
        directory = cache_directory or (
            Path.home() / ".cache" / "nd_sociomind" / "follow_scenario"
        )
        name = hashlib.sha256(str(self.source).encode("utf-8")).hexdigest()[:24]
        self.cache_path = directory / f"{name}.json.zlib"

    def _fingerprint(self) -> dict:
        stat = self.source.stat()
        relevant_code = {}
        for module in self.RELEVANT_MODULES:
            location = (Path(__file__).parent / module).resolve()
            info = location.stat()
            relevant_code[module] = [info.st_size, info.st_mtime_ns]
        reader = RosYamlLidarReader()
        return {
            "schema": self.VERSION,
            "source": str(self.source),
            "source_size": stat.st_size,
            "source_mtime_ns": stat.st_mtime_ns,
            "settings": asdict(self.settings),
            "lidar_maximum_points": reader.maximum_points,
            "lidar_maximum_age_seconds": reader.maximum_age_seconds,
            "lidar_clip_meters": reader.MAXIMUM_DISPLAY_DISTANCE_METERS,
            "implementation": relevant_code,
        }

    @staticmethod
    def _encode_trajectory(trajectory: Trajectory | None):
        if trajectory is None:
            return None
        return [
            [p.timestamp_seconds, p.x_meters, p.y_meters]
            for p in trajectory.samples
        ]

    @staticmethod
    def _decode_trajectory(data) -> Trajectory | None:
        if data is None:
            return None
        if not isinstance(data, list) or len(data) > 2_000_000:
            raise ValueError("Invalid cached trajectory")
        return Trajectory([PositionSample(*sample) for sample in data])

    @staticmethod
    def _encode_scans(frames: list[LidarScan | None]) -> dict:
        scans = []
        indices = []
        identity_index = {}
        for scan in frames:
            if scan is None:
                indices.append(-1)
                continue
            key = id(scan)
            if key not in identity_index:
                identity_index[key] = len(scans)
                scans.append([
                    scan.timestamp_seconds,
                    scan.points_meters,
                    scan.covers_full_circle,
                    scan.heading_radians,
                ])
            indices.append(identity_index[key])
        return {"scans": scans, "indices": indices}

    @staticmethod
    def _decode_scans(encoded: dict) -> list[LidarScan | None]:
        raw_scans = encoded["scans"]
        indices = encoded["indices"]
        if len(raw_scans) > 2_000_000 or len(indices) > 2_000_000:
            raise ValueError("Excessive cached LiDAR frame count")
        scans = [
            LidarScan(
                float(entry[0]),
                tuple(None if pt is None else (float(pt[0]), float(pt[1]))
                      for pt in entry[1]),
                bool(entry[2]),
                None if entry[3] is None else float(entry[3]),
            )
            for entry in raw_scans
        ]
        return [None if index == -1 else scans[index] for index in indices]

    def load(self) -> tuple[TrajectoryPair, dict, str] | None:
        """Return None when missing, stale, corrupted or incompatible."""
        try:
            fingerprint = self._fingerprint()
            if self.cache_path.stat().st_size > self.MAX_COMPRESSED_BYTES:
                return None
            compressed = self.cache_path.read_bytes()
            decompressor = zlib.decompressobj()
            content = decompressor.decompress(compressed, self.MAX_DECOMPRESSED_BYTES + 1)
            if len(content) > self.MAX_DECOMPRESSED_BYTES or not decompressor.eof or decompressor.unused_data:
                return None
            payload = json.loads(content)
            if payload["fingerprint"] != fingerprint:
                return None
            data = payload["data"]
            trajectories = TrajectoryPair(
                self._decode_trajectory(data["leader"]),
                self._decode_trajectory(data["follower"]),
                self._decode_trajectory(data["follower_full_route"]),
            )
            scans = {
                role: self._decode_scans(data["lidar_frames"][role])
                for role in ("leader", "follower")
            }
            if trajectories.leader.length() < 2 or (
                trajectories.leader.length() != trajectories.follower.length()
            ) or any(len(scans[role]) != trajectories.leader.length()
                     for role in ("leader", "follower")):
                return None
            # The source could have changed while decoding the cache.
            if self._fingerprint() != fingerprint:
                return None
            return trajectories, scans, data["note"]
        except (OSError, ValueError, TypeError, KeyError, IndexError,
                AttributeError, UnicodeError, zlib.error):
            return None

    def save(self, trajectories: TrajectoryPair, scans: dict, note: str) -> None:
        """Write atomically; retain the original YAML unmodified."""
        fingerprint = self._fingerprint()
        body = {
            "leader": self._encode_trajectory(trajectories.leader),
            "follower": self._encode_trajectory(trajectories.follower),
            "follower_full_route": self._encode_trajectory(
                trajectories.follower_full_route
            ),
            "lidar_frames": {
                role: self._encode_scans(scans[role])
                for role in ("leader", "follower")
            },
            "note": note,
        }
        raw = json.dumps(
            {"fingerprint": fingerprint, "data": body},
            separators=(",", ":"), ensure_ascii=False, allow_nan=False,
        ).encode("utf-8")
        compressed = zlib.compress(raw, level=2)
        if self._fingerprint() != fingerprint:
            return  # Source changed during cache construction; don't save.
        self.cache_path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb", dir=self.cache_path.parent,
                prefix=self.cache_path.stem + ".", suffix=".tmp",
                delete=False,
            ) as tmp:
                temp_path = Path(tmp.name)
                tmp.write(compressed)
            if self._fingerprint() == fingerprint:
                os.replace(temp_path, self.cache_path)
                temp_path = None
        finally:
            if temp_path is not None:
                temp_path.unlink(missing_ok=True)
