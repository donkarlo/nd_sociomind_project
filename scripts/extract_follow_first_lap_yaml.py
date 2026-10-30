#!/usr/bin/env python3
"""Stream the original Dropbox Follow ROS YAML and save one GPS/LiDAR circuit per UAV."""
from __future__ import annotations

from collections import Counter
import io
import json
from pathlib import Path
import re
import sys
import time
from urllib.request import Request, urlopen

PROJECT = Path(__file__).resolve().parents[1]
DATA = PROJECT / "data/autonomous_navigation/follow_scenario"
OUTPUT = DATA / "first_lap_ros_gps_lidar.yaml"
SOURCE = "/phd/data/old_experiment_not_synced_dropbox/follow-scenario/uav1-gps-lidar-uav2-gps-lidar.yaml"
ROLES = {
    "/uav1/odometry/odom_gps": "uav1", "/uav1/rplidar/scan": "uav1",
    "/uav2/odometry/odom_gps": "uav2", "/uav2/rplidar/scan": "uav2",
}
TOPIC = re.compile(rb"^#\s*topic:\s*(/[^\s]+)")
COMMENT_STAMP = re.compile(rb"^#\s*timestamp\s*\(sec\):\s*([\d.]+)", re.M)
SEC = re.compile(rb"^\s+secs:\s*(\d+)", re.M)
NSEC = re.compile(rb"^\s+nsecs:\s*(\d+)", re.M)
MARGIN = 0.5


def durations():
    sys.path.insert(0, str(PROJECT / "src"))
    from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.data_source_settings import DataSourceSettings
    from nd_sociomind.experiment.autonomous_navigation.follow_scenario.config.scenario_settings import ScenarioSettings
    from nd_sociomind.experiment.autonomous_navigation.follow_scenario.data.reader.cleaned_gps_reader import CleanedGpsReader
    pair = CleanedGpsReader(DataSourceSettings(), ScenarioSettings()).read_first_lap(
        DATA / "uav-1-cleaned-gps-pos-vel.txt", DATA / "uav-2-cleaned-gps-pos-vel.txt")
    return {
        "uav1": pair.leader.samples[-1].timestamp_seconds - pair.leader.samples[0].timestamp_seconds,
        "uav2": pair.follower.samples[-1].timestamp_seconds - pair.follower.samples[0].timestamp_seconds,
    }


def messages(stream):
    prefix = bytearray()
    previous = bytearray()
    topic = None
    for line in stream:
        match = TOPIC.match(line)
        if match:
            yield topic, bytes(previous if topic is not None else prefix)
            topic = match.group(1).decode("ascii")
            previous = bytearray(line)
        elif topic is None:
            prefix.extend(line)
        else:
            previous.extend(line)
    if topic is not None:
        yield topic, bytes(previous)


def stamp_of(message):
    match = COMMENT_STAMP.search(message)
    if match:
        return float(match.group(1))
    seconds = SEC.search(message)
    if seconds:
        nanos = NSEC.search(message)
        return float(seconds.group(1)) + (float(nanos.group(1)) / 1e9 if nanos else 0.)
    return None


def extract(url):
    spans = durations()
    print("GPS first-lap duration (sec):", spans, flush=True)
    starts, lasts, counts = {}, {}, Counter()
    temp = OUTPUT.with_suffix(".yaml.partial")
    total = 0
    complete = False
    next_log = 32 * (2 ** 20)
    beginning = time.monotonic()
    with urlopen(Request(url, headers={"User-Agent": "nd-sociomind-ros-extractor"}), timeout=90) as response:
        with temp.open("wb", buffering=1024 * 1024) as output:
            for topic, message in messages(io.BufferedReader(response, 1024 * 1024)):
                if topic is None:
                    output.write(message)
                    continue
                role = ROLES.get(topic)
                if role is None:
                    continue
                stamp = stamp_of(message)
                if stamp is None:
                    continue
                starts.setdefault(role, stamp)
                if stamp <= starts[role] + spans[role] + MARGIN:
                    output.write(message)
                    total += len(message)
                    lasts[role] = stamp
                    counts[topic] += 1
                if total >= next_log:
                    print(f"extracted {total/2**20:.1f} MiB, {sum(counts.values())} messages, {time.monotonic()-beginning:.0f}s", flush=True)
                    next_log = total + 32 * (2 ** 20)
                if len(starts) == 2 and stamp > max(starts[r] + spans[r] + MARGIN for r in starts):
                    complete = True
                    break
    if not complete or len(counts) != 4:
        raise RuntimeError(f"Partial extract. Complete={complete}; topics={dict(counts)}; source starts={starts}")
    temp.replace(OUTPUT)
    metadata = {
        "source_dropbox_path": SOURCE,
        "file": str(OUTPUT.relative_to(PROJECT)),
        "range": "first GPS circuit of each UAV (plus 0.5 seconds)",
        "seconds_per_lap": spans,
        "first_ros_stamp": starts,
        "last_ros_stamp": lasts,
        "topic_counts": dict(counts),
        "size_bytes": OUTPUT.stat().st_size,
    }
    OUTPUT.with_suffix(".metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print("SUCCESS", json.dumps(metadata, indent=2), flush=True)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("Usage: extract_follow_first_lap_yaml.py TEMPORARY_DROPBOX_URL")
    extract(sys.argv[1])
