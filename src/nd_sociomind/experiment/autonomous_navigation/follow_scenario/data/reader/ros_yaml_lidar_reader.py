"""Stream ROS LaserScan messages from the follow-scenario YAML recording."""
from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math
from pathlib import Path
import re

from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.model.trajectory_pair import TrajectoryPair


@dataclass(frozen=True)
class LidarScan:
    timestamp_seconds: float
    points_meters: tuple[tuple[float, float] | None, ...]
    covers_full_circle: bool
    heading_radians: float | None = None


class RosYamlLidarReader:
    """Parse both UAV scan topics with bounded RAM; never invent missing returns."""

    TOPICS = {'/uav1/rplidar/scan': 'leader', '/uav2/rplidar/scan': 'follower'}
    GPS_TOPICS = {'/uav1/odometry/odom_gps': 'leader', '/uav2/odometry/odom_gps': 'follower'}
    MAXIMUM_DISPLAY_DISTANCE_METERS = 5.0
    NUMBER = re.compile(r'(?i)[+-]?(?:\.inf|\.nan|inf(?:inity)?|nan|(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)')

    def __init__(self, maximum_points: int = 720, maximum_age_seconds: float = 0.35):
        self.maximum_points = max(16, maximum_points)
        self.maximum_age_seconds = maximum_age_seconds

    @classmethod
    def _number(cls, text: str) -> float | None:
        match = cls.NUMBER.search(text)
        return float(match.group()) if match else None

    @classmethod
    def _topic(cls, text: str) -> str | None:
        # rosbag text dumps put the topic in a comment, e.g. '# topic: /uav1/rplidar/scan'.
        if text.startswith('# topic:'):
            text = text[1:].strip()
        if text.startswith('topic:'):
            return text.split(':', 1)[1].strip().strip(chr(34)).strip(chr(39))
        for topic in (*cls.TOPICS, *cls.GPS_TOPICS):
            if text.startswith(topic + ':'):
                return topic
        return None

    def _messages(self, path: Path):
        topic = ''
        fields = {}
        secs = None
        nsecs = 0.0
        collecting_ranges = False
        inline_ranges = False
        range_indent = -1
        gps_field_group = ''
        gps_field_indent = -1

        def message():
            timestamp = secs + nsecs / 1e9 if secs is not None else None
            return topic, timestamp, fields

        with path.open('r', encoding='utf-8', errors='replace', buffering=1024 * 1024) as source:
            for raw in source:
                text = raw.strip()
                indentation = len(raw) - len(raw.lstrip(' '))
                normalized = text[2:].strip() if text.startswith('- ') else text
                next_topic = self._topic(normalized)
                # Recorded rosbag YAML uses '# topic: ...' followed by a
                # document separator ('---') before the actual message body.
                # The separator must NOT reset the topic or discard its fields.
                if next_topic is not None:
                    if topic:
                        yield message()
                    topic = next_topic
                    fields, secs, nsecs = {}, None, 0.0
                    collecting_ranges, inline_ranges = False, False
                    gps_field_group, gps_field_indent = '', -1
                    continue
                if text == '---':
                    continue
                if not topic:
                    continue

                if collecting_ranges:
                    if inline_ranges:
                        fields['ranges'].extend(self._range_values(text))
                        if ']' in text:
                            collecting_ranges, inline_ranges = False, False
                        continue
                    if text.startswith('- ') and indentation >= range_indent:
                        fields['ranges'].extend(self._range_values(text[2:]))
                        continue
                    collecting_ranges = False

                if text.startswith('stamp:') and '{' in text:
                    msecs = re.search(r'\bsecs:\s*(\d+)', text)
                    mnsecs = re.search(r'\bnsecs:\s*(\d+)', text)
                    if msecs:
                        secs = float(msecs.group(1))
                        nsecs = float(mnsecs.group(1)) if mnsecs else 0
                elif text.startswith('secs:'):
                    secs = self._number(text.split(':', 1)[1])
                elif text.startswith('nsecs:'):
                    nsecs = self._number(text.split(':', 1)[1]) or 0.0
                elif topic in self.GPS_TOPICS:
                    if text in ('position:', 'orientation:'):
                        gps_field_group = text[:-1]
                        gps_field_indent = indentation
                        fields[gps_field_group] = {}
                    elif gps_field_group and indentation > gps_field_indent:
                        name, separator, value = text.partition(':')
                        if separator and name in ('x', 'y', 'z', 'w'):
                            number = self._number(value)
                            if number is not None:
                                fields[gps_field_group][name] = number
                    elif indentation <= gps_field_indent:
                        gps_field_group = ''
                elif topic in self.TOPICS:
                    for name in ('angle_min', 'angle_increment', 'range_min', 'range_max'):
                        if text.startswith(name + ':'):
                            fields[name] = self._number(text.split(':', 1)[1])
                            break
                    else:
                        if text.startswith('ranges:'):
                            fields['ranges'] = []
                            range_indent = indentation
                            rest = text.split(':', 1)[1].strip()
                            if rest:
                                fields['ranges'].extend(self._range_values(rest))
                                inline_ranges = '[' in rest and ']' not in rest
                                collecting_ranges = inline_ranges
                            else:
                                collecting_ranges = True
            if topic:
                yield message()

    @classmethod
    def _range_values(cls, text: str) -> list[float]:
        # ROS/PyYAML use .inf/.nan, which Python float() does not accept.
        return [float(m.group().lower().replace('.inf', 'inf').replace('.nan', 'nan'))
                for m in cls.NUMBER.finditer(text)]

    def _scan(self, timestamp: float, fields: dict) -> LidarScan | None:
        ranges = fields.get('ranges') or []
        angle_min = fields.get('angle_min')
        increment = fields.get('angle_increment')
        if not ranges or angle_min is None or increment is None or not math.isfinite(increment) or increment == 0:
            return None
        minimum = fields.get('range_min')
        maximum = fields.get('range_max')
        minimum = minimum if minimum is not None and math.isfinite(minimum) else 0.0
        maximum = maximum if maximum is not None and math.isfinite(maximum) else math.inf
        stride = max(1, math.ceil(len(ranges) / self.maximum_points))
        points = []
        for index in range(0, len(ranges), stride):
            distance = ranges[index]
            # Missing returns reported as +inf are displayed at the 5 m
            # limit. Only the in-memory visualization changes, not the ROS YAML.
            if distance == math.inf:
                distance = self.MAXIMUM_DISPLAY_DISTANCE_METERS
            if not math.isfinite(distance) or distance <= 0 or distance < minimum:
                points.append(None)
            else:
                # Clip all other valid distances longer than 5 m as well.
                distance = min(distance, self.MAXIMUM_DISPLAY_DISTANCE_METERS)
                theta = angle_min + index * increment
                points.append((distance * math.cos(theta), distance * math.sin(theta)))
        return LidarScan(timestamp, tuple(points), abs(len(ranges) * increment) > (2 * math.pi - 0.05))

    @staticmethod
    def _offset(first_gps_time: float, first_yaml_time: float) -> float:
        # Absolute UNIX stamps can be matched directly. Relative GPS times need
        # alignment to the first recorded odometry message (approximate).
        if first_gps_time > 100_000_000 and first_yaml_time > 100_000_000:
            return 0.0
        return first_gps_time - first_yaml_time

    def read_for_frames(self, path: Path, trajectories: TrajectoryPair, *, scene_rotation_radians: float = 0.0, same_ros_clock: bool = False) -> dict[str, list[LidarScan | None]]:
        """Keep only the closest scan for each GPS frame; do not retain the 3.3 GB stream."""
        timestamps = {
            'leader': [p.timestamp_seconds for p in trajectories.leader.samples],
            'follower': [p.timestamp_seconds for p in trajectories.follower.samples],
        }
        frames: dict[str, list[LidarScan | None]] = {
            role: [None] * len(times) for role, times in timestamps.items()
        }
        errors = {role: [math.inf] * len(times) for role, times in timestamps.items()}
        first_gps_stamp: dict[str, float] = {}
        odometry_heading: dict[str, float] = {}
        passed_end: set[str] = set()

        for topic, stamp, fields in self._messages(path):
            if stamp is None:
                continue
            if topic in self.GPS_TOPICS:
                role = self.GPS_TOPICS[topic]
                first_gps_stamp.setdefault(role, stamp)
                q = fields.get('orientation') or {}
                if all(axis in q for axis in ('x', 'y', 'z', 'w')):
                    x, y, z, w = (q[axis] for axis in ('x', 'y', 'z', 'w'))
                    if all(math.isfinite(v) for v in (x, y, z, w)):
                        yaw = math.atan2(2.0 * (w * z + x * y),
                                         1.0 - 2.0 * (y * y + z * z))
                        odometry_heading[role] = yaw + scene_rotation_radians
                continue
            if topic not in self.TOPICS:
                continue

            role = self.TOPICS[topic]
            if role not in first_gps_stamp or not timestamps[role]:
                continue
            offset = 0.0 if same_ros_clock else self._offset(timestamps[role][0], first_gps_stamp[role])
            aligned_time = stamp + offset
            frame_times = timestamps[role]
            if aligned_time > frame_times[-1] + self.maximum_age_seconds:
                passed_end.add(role)
                if len(passed_end) == 2:
                    break
                continue
            if aligned_time < frame_times[0] - self.maximum_age_seconds:
                continue

            first_index = bisect_left(frame_times, aligned_time - self.maximum_age_seconds)
            end_index = bisect_right(frame_times, aligned_time + self.maximum_age_seconds)
            eligible = [(index, abs(frame_times[index] - aligned_time))
                        for index in range(first_index, end_index)
                        if abs(frame_times[index] - aligned_time) < errors[role][index]]
            if not eligible:
                continue
            scan = self._scan(stamp, fields)
            if scan is None:
                continue
            # LiDAR frame is fixed to the drone body in this experiment.
            # The latest odometry quaternion supplies the actual recorded yaw.
            scan = LidarScan(scan.timestamp_seconds, scan.points_meters,
                             scan.covers_full_circle, odometry_heading.get(role))
            for index, error in eligible:
                frames[role][index] = scan
                errors[role][index] = error
        return frames
