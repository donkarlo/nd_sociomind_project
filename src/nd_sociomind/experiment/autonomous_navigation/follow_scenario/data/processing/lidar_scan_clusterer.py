"""DBSCAN over full 720-ray LiDAR scans and their temporal first derivatives.

Each feature vector is 720-dimensional, not a cloud of Cartesian vertices.
A cluster's displayed outline is its recorded medoid scan (720 vertices).
The first-derivative cluster uses temporal range change to assign membership,
but draws the associated *range* medoid, since signed m/s cannot be a radius.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from sklearn.cluster import DBSCAN

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.ros_yaml_lidar_reader import LidarScan


@dataclass(frozen=True)
class DerivativeClusterProfiles:
    labels_by_frame: tuple[int | None, ...]
    outlines_by_label: dict[int, tuple[tuple[float, float] | None, ...]]

    def outline_for(self, frame_index: int):
        label = self.labels_by_frame[frame_index] if frame_index < len(self.labels_by_frame) else None
        return self.outlines_by_label.get(label), label


@dataclass(frozen=True)
class LidarClusterProfiles:
    zeroth: DerivativeClusterProfiles
    first: DerivativeClusterProfiles


class LidarScanClusterer:
    """Cluster complete recorded scans by range and temporal range velocity."""

    RAY_COUNT = 720
    MAX_RANGE_METERS = 5.0
    MIN_SAMPLES = 3
    ZEROTH_EPS_RMS_METERS = 0.38
    FIRST_EPS_RMS_METERS_PER_SECOND = 0.85

    @classmethod
    def _ranges(cls, scan: LidarScan) -> np.ndarray:
        return np.array([
            cls.MAX_RANGE_METERS if point is None else math.hypot(*point)
            for point in scan.points_meters
        ], dtype=np.float32)

    def fit(self, frames: list[LidarScan | None]) -> LidarClusterProfiles:
        unique_scans = []
        identity_to_index = {}
        scan_indices = []
        for scan in frames:
            if scan is None or len(scan.points_meters) != self.RAY_COUNT or not scan.covers_full_circle:
                scan_indices.append(None)
                continue
            key = id(scan)
            if key not in identity_to_index:
                identity_to_index[key] = len(unique_scans)
                unique_scans.append(scan)
            scan_indices.append(identity_to_index[key])

        if unique_scans:
            ranges = np.stack([self._ranges(scan) for scan in unique_scans])
            timestamps = np.array([scan.timestamp_seconds for scan in unique_scans])
        else:
            ranges = np.empty((0, self.RAY_COUNT), dtype=np.float32)
            timestamps = np.empty(0)
        zeroth = self._fit_order(
            unique_scans, scan_indices, ranges, np.arange(len(unique_scans)),
            self.ZEROTH_EPS_RMS_METERS,
        )
        valid_pairs = np.flatnonzero(np.diff(timestamps) > 0.001) + 1
        if len(valid_pairs):
            deltas = np.diff(ranges, axis=0)[valid_pairs - 1]
            dt = np.diff(timestamps)[valid_pairs - 1].astype(np.float32)
            velocities = deltas / dt[:, None]
        else:
            velocities = np.empty((0, self.RAY_COUNT), dtype=np.float32)
        first = self._fit_order(
            unique_scans, scan_indices, velocities, valid_pairs,
            self.FIRST_EPS_RMS_METERS_PER_SECOND,
        )
        return LidarClusterProfiles(zeroth, first)

    def _fit_order(self, scans, scan_indices, features, unique_indices, eps_rms):
        labels = np.full(len(scans), -1, dtype=np.int32)
        if len(features) >= self.MIN_SAMPLES:
            clustering = DBSCAN(
                eps=eps_rms * math.sqrt(self.RAY_COUNT),
                min_samples=self.MIN_SAMPLES,
                metric="euclidean",
                algorithm="brute",
                n_jobs=1,
            )
            labels[unique_indices] = clustering.fit_predict(features)

        # Represent a cluster by the recorded 720-gon nearest its range mean.
        # This avoids drawing an unphysical polygon from signed derivatives.
        outlines = {}
        for label in sorted(set(labels.tolist()) - {-1}):
            members = unique_indices[labels[unique_indices] == label]
            member_ranges = np.stack([self._ranges(scans[i]) for i in members])
            center = member_ranges.mean(axis=0)
            medoid = members[np.argmin(np.sum((member_ranges - center) ** 2, axis=1))]
            outlines[int(label)] = scans[int(medoid)].points_meters

        valid_index_set = set(int(i) for i in unique_indices)
        labels_by_frame = tuple(
            None if index is None or index not in valid_index_set
            else int(labels[index])
            for index in scan_indices
        )
        return DerivativeClusterProfiles(labels_by_frame, outlines)
