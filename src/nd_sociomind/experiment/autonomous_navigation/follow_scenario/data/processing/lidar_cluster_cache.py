"""Versioned, non-executable cache for expensive 720-dimensional DBSCAN results."""
from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import zlib

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing import lidar_scan_clusterer
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.processing.lidar_scan_clusterer import (
    DerivativeClusterProfiles, LidarClusterProfiles,
)
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_replay_cache import RosReplayCache


class LidarClusterCache:
    VERSION = 1
    MAX_COMPRESSED_BYTES = 32 * 1024 * 1024
    MAX_DECOMPRESSED_BYTES = 100 * 1024 * 1024

    def __init__(self, replay_cache: RosReplayCache):
        self.replay_cache = replay_cache
        self.path = replay_cache.cache_path.with_name(
            replay_cache.cache_path.stem + ".dbscan.json.zlib"
        )

    def _fingerprint(self):
        module_info = Path(lidar_scan_clusterer.__file__).stat()
        return {
            "version": self.VERSION,
            "recording": self.replay_cache._fingerprint(),
            "clusterer_source_size": module_info.st_size,
            "clusterer_source_mtime": module_info.st_mtime_ns,
        }

    @staticmethod
    def _encode_order(order):
        return {
            "labels": list(order.labels_by_frame),
            "outlines": {str(key): points for key, points in order.outlines_by_label.items()},
        }

    @staticmethod
    def _decode_order(data, frame_count):
        labels = tuple(data["labels"])
        if len(labels) != frame_count or any(
            value is not None and (type(value) is not int or value < -1)
            for value in labels
        ):
            raise ValueError("Invalid DBSCAN label series")
        outlines = {}
        for key, points in data["outlines"].items():
            label = int(key)
            if label < 0 or len(points) != 720:
                raise ValueError("Invalid recorded cluster medoid")
            outlines[label] = tuple(
                None if point is None else (float(point[0]), float(point[1]))
                for point in points
            )
        return DerivativeClusterProfiles(labels, outlines)

    def load(self, frames: dict) -> dict | None:
        try:
            if self.path.stat().st_size > self.MAX_COMPRESSED_BYTES:
                return None
            compressed = self.path.read_bytes()
            decoder = zlib.decompressobj()
            raw = decoder.decompress(compressed, self.MAX_DECOMPRESSED_BYTES + 1)
            if len(raw) > self.MAX_DECOMPRESSED_BYTES or not decoder.eof or decoder.unused_data:
                return None
            payload = json.loads(raw)
            if payload["fingerprint"] != self._fingerprint():
                return None
            profiles = {}
            for role in ("leader", "follower"):
                record = payload["clusters"][role]
                profiles[role] = LidarClusterProfiles(
                    self._decode_order(record["zeroth"], len(frames[role])),
                    self._decode_order(record["first"], len(frames[role])),
                )
            return profiles
        except (OSError, ValueError, TypeError, KeyError, IndexError,
                AttributeError, UnicodeError, zlib.error):
            return None

    def save(self, clusters: dict) -> None:
        fingerprint = self._fingerprint()
        body = {
            role: {
                "zeroth": self._encode_order(clusters[role].zeroth),
                "first": self._encode_order(clusters[role].first),
            }
            for role in ("leader", "follower")
        }
        content = json.dumps(
            {"fingerprint": fingerprint, "clusters": body},
            separators=(",", ":"), allow_nan=False,
        ).encode("utf-8")
        compressed = zlib.compress(content, level=2)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb", dir=self.path.parent,
                prefix=self.path.stem + ".", suffix=".tmp", delete=False,
            ) as file:
                temp_path = Path(file.name)
                file.write(compressed)
            if self._fingerprint() == fingerprint:
                os.replace(temp_path, self.path)
                temp_path = None
        finally:
            if temp_path is not None:
                temp_path.unlink(missing_ok=True)
