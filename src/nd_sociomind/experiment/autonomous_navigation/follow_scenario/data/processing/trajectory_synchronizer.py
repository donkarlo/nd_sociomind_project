from bisect import bisect_left

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.position_sample import PositionSample
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory import Trajectory
from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair


class TrajectorySynchronizer:
    def synchronize(self, trajectory_pair: TrajectoryPair) -> TrajectoryPair:
        if self._timestamps_are_usable(trajectory_pair):
            synchronized = self._synchronize_by_timestamp(trajectory_pair)
            if synchronized.leader.length() >= 20 and synchronized.follower.length() >= 20:
                return synchronized
        return self._synchronize_by_index(trajectory_pair)

    def _timestamps_are_usable(self, trajectory_pair: TrajectoryPair) -> bool:
        if trajectory_pair.leader.length() < 2 or trajectory_pair.follower.length() < 2:
            return False
        leader_duration = trajectory_pair.leader.samples[-1].timestamp_seconds - trajectory_pair.leader.samples[0].timestamp_seconds
        follower_duration = trajectory_pair.follower.samples[-1].timestamp_seconds - trajectory_pair.follower.samples[0].timestamp_seconds
        return leader_duration > 0.0 and follower_duration > 0.0

    def _synchronize_by_timestamp(self, trajectory_pair: TrajectoryPair) -> TrajectoryPair:
        follower_timestamps = [sample.timestamp_seconds for sample in trajectory_pair.follower.samples]
        overlap_start = max(trajectory_pair.leader.samples[0].timestamp_seconds, follower_timestamps[0])
        overlap_end = min(trajectory_pair.leader.samples[-1].timestamp_seconds, follower_timestamps[-1])
        leader = Trajectory()
        follower = Trajectory()

        for leader_sample in trajectory_pair.leader.samples:
            timestamp = leader_sample.timestamp_seconds
            if timestamp < overlap_start or timestamp > overlap_end:
                continue
            interpolated_follower = self._interpolate_at_timestamp(trajectory_pair.follower, follower_timestamps, timestamp)
            if interpolated_follower is None:
                continue
            leader.append(leader_sample)
            follower.append(interpolated_follower)

        return TrajectoryPair(leader, follower)

    def _interpolate_at_timestamp(self, trajectory: Trajectory, timestamps: list[float], timestamp: float) -> PositionSample | None:
        insertion_index = bisect_left(timestamps, timestamp)
        if insertion_index <= 0:
            return trajectory.samples[0]
        if insertion_index >= len(timestamps):
            return trajectory.samples[-1]

        first = trajectory.samples[insertion_index - 1]
        second = trajectory.samples[insertion_index]
        duration = second.timestamp_seconds - first.timestamp_seconds
        if duration <= 0.0:
            return first
        interpolation = (timestamp - first.timestamp_seconds) / duration
        return self._interpolate(first, second, interpolation)

    def _synchronize_by_index(self, trajectory_pair: TrajectoryPair) -> TrajectoryPair:
        frame_count = min(trajectory_pair.leader.length(), trajectory_pair.follower.length())
        if frame_count < 2:
            raise ValueError("At least two synchronized samples are required.")
        leader = self._resample_by_index(trajectory_pair.leader, frame_count)
        follower = self._resample_by_index(trajectory_pair.follower, frame_count)
        return TrajectoryPair(leader, follower)

    def _resample_by_index(self, trajectory: Trajectory, target_count: int) -> Trajectory:
        if trajectory.length() == target_count:
            return trajectory

        resampled = Trajectory()
        maximum_source_index = trajectory.length() - 1
        maximum_target_index = target_count - 1

        for target_index in range(target_count):
            source_position = target_index * maximum_source_index / maximum_target_index
            lower_index = int(source_position)
            upper_index = min(lower_index + 1, maximum_source_index)
            interpolation = source_position - lower_index
            lower_sample = trajectory.samples[lower_index]
            upper_sample = trajectory.samples[upper_index]
            resampled.append(self._interpolate(lower_sample, upper_sample, interpolation))

        return resampled

    def _interpolate(self, first: PositionSample, second: PositionSample, interpolation: float) -> PositionSample:
        timestamp_seconds = first.timestamp_seconds + (second.timestamp_seconds - first.timestamp_seconds) * interpolation
        x_meters = first.x_meters + (second.x_meters - first.x_meters) * interpolation
        y_meters = first.y_meters + (second.y_meters - first.y_meters) * interpolation
        return PositionSample(timestamp_seconds, x_meters, y_meters)
