# Autonomous navigation visualizer

This feature adds a Qt/PySide6 top-view replay and interaction mode for the recorded leader–follower corridor experiment.

## What it shows

- UAV1 as `Leader` and UAV2 as `Follower`.
- ROS GPS odometry, quaternion body heading and LiDAR read from the **same ROS timebase** when the compact Follow recording is present. The cleaned GPS files are used only when ROS data is unavailable. The compact source may stop slightly before physical lap closure.
- The actual Gazebo follow-scenario arena dimensions: a 32 m × 32 m outer wall and a centered 8 m × 8 m inner wall.
- The actual follow-scenario narrowing arrangement reconstructed from `selfaware-drones/rosws/worlds/follow-scenario.world`: two opposing 4 m deep U-shaped wall protrusions on the right side, leaving a 4 m passage.
- The requested one-centimeter visual shortening of the narrowing span is retained: the 4.00 m Gazebo span is shown as 3.99 m, centered symmetrically.
- GPS trajectories are never scaled or warped. Both UAV trajectories receive exactly the same rigid rotation and translation, and ROS yaw is rotated into that same scene frame.
- The recorded abnormal/following segment is used only to resolve the 90-degree orientation ambiguity, so the abnormal segment is aligned with the right-side Gazebo narrowing without changing any recorded distance.
- Top-view Tarot T650 graphics use a visual scale corresponding to an approximately 0.495 m outer radius, based on the Gazebo motor offset and rotor collision radius.
- Automatic visual classification between side-by-side motion and leader-first/follower-behind motion.
- A logarithmic speed slider from 0.10× to 80.00×, play/pause, restart, and lap timeline scrubbing.
- Leader/Follower labels are separate non-rotating graphics items.
- A 1.5 m visual margin around the arena so the full drone and its label remain visible near the outer wall.

## Interactive learned coordination

Either drone can be grabbed with the left mouse button and dragged.

- Playback pauses automatically while a drone is being manipulated.
- The other drone responds gradually instead of teleporting.
- Its target relative position is learned from a local temporal window of the synchronized recorded GPS pair. This preserves the learned near/far relationship, side-by-side offsets, and leader-first/follower-behind relationship in the narrow section.
- The autonomous response speed and motion-coupling gain are also estimated from the recorded local GPS motion.
- When the mouse button is released, the pair gradually returns to the nearest recorded pair state and then resumes the original replay if playback was running before the drag.
- The recorded GPS data is never modified by interaction.

## Collision safety

Manual dragging and autonomous correction both use the same safety model.

- The Tarot T650 outer rotor footprint is modeled with a 0.495 m radius.
- A minimum 0.20 m blade-to-wall clearance is enforced.
- The user cannot drag a drone outside the outer square, through the inner square, or through the narrowing walls.
- Autonomous coordination is subject to the same wall constraints.
- A minimum inter-drone clearance is also enforced.
- Motion is checked continuously in small spatial steps, so a fast mouse movement cannot jump through a wall.

When the follow-scenario ROS YAML is available, the visualizer replays **recorded scans** from `/uav1/rplidar/scan` and `/uav2/rplidar/scan`, drawing timestamp-matched Cartesian outlines around each drone. For visualization, any valid LiDAR distance exceeding **5.0 m**, as well as positive infinity (`+inf`, representing a missing return), is displayed as **5.0 m** before Cartesian conversion. The original ROS measurements remain unchanged. `nan`, negative infinity, and invalid nonpositive ranges remain omitted. The `+inf` fallback is a display convention, not a measured obstacle. Every fully closed nondegenerate LiDAR polygon displays a small cross and `CG` at the **uniform-area-density center of mass**, calculated from the polygon area moments (shoelace formula), not the mean of ray endpoints. Open/incomplete polygons have no CG marker. The legend reads `CG = center of gravity`; the marker follows the recorded drone rotation and position. Outlines and CG markers are hidden during manual dragging, because a moved pose was never observed by the recorded sensors. Recorded scan outlines are visualization only, not a live collision oracle. For interactive positions, the safety controller computes the equivalent obstacle clearance directly from the exact Gazebo wall geometry, which remains valid at arbitrary user-selected positions.

## Gazebo reference

The visual arena is based on:

```text
/projs/research/artificial-intelligence/self-awareness/selfaware-drones/rosws/worlds/follow-scenario.world
```

The Gazebo wall model uses 32 m outer walls, 8 m inner walls, 0.05 m wall thickness, and 4 m narrowing-wall segments.

## Data source

When the compact ROS recording is present, the application uses GPS, body orientation and LiDAR from that one file (without guessing a timestamp offset). Otherwise, it falls back to these two cleaned GPS files:

```text
data/autonomous_navigation/follow_scenario/uav-1-cleaned-gps-pos-vel.txt
data/autonomous_navigation/follow_scenario/uav-2-cleaned-gps-pos-vel.txt
```

The locally extracted `data/autonomous_navigation/follow_scenario/first_lap_ros_gps_lidar.yaml` contains the recorded ROS odometry and LiDAR samples for both UAVs and is auto-discovered; its companion JSON documents the extraction. The extraction was initially bounded by the cleaned GPS lap durations, so the ROS segment may stop slightly **before** actual lap closure; the UI reports this explicitly instead of falsely claiming closure. Recorded GPS coordinates, odometry quaternion yaw and LiDAR scan timestamps are kept together in this mode. The dotted Follower route is now drawn from its own complete GPS circuit, even if it finishes a few seconds after Leader's recording ends. The moving drones and LiDAR frames still use the shared synchronized interval; no follower GPS points or closing path segments are invented. Loop closure is accepted only when the recorded follower returns within about 0.12 m of its recorded start, and its short final connector joins two real GPS samples. Robot and sensor contours rotate by quaternion yaw plus the common rigid scene rotation, **not** the path's instantaneous velocity tangent. The measured yaw is used assuming the LiDAR is aligned with the body frame; the first wall returns corroborate that mounting assumption (around 0.1 m error in the first scan). LiDAR clipping at 5 m and `+inf → 5 m` are visualization-only conventions, not physical hits at 5 m. For the original file elsewhere, set `ND_FOLLOW_ROS_YAML=/absolute/path/to/uav1-gps-lidar-uav2-gps-lidar.yaml`. When the file is absent, only cleaned GPS playback is used.

## LiDAR clustering overlays

For each UAV independently, each recorded LaserScan is retained as an ordered **720-ray** vector (not reduced to Cartesian point-to-point distances). Positive infinity and every finite distance greater than 5 m are mapped to 5 m for the visualization and clustering; the original ROS YAML remains unmodified. Invalid NaN or nonpositive returns remain missing in the outline and are represented by 5 m only when constructing the clustering feature vector.

Two separate DBSCAN models are fitted to the synchronized time series for each UAV:

- **Zeroth temporal derivative (green):** the full 720-dimensional range vector, with an RMS-per-ray Euclidean radius of 0.38 m.
- **First temporal derivative (pink):** the full 720-dimensional vector of range changes in meters per second, calculated from consecutive unique scans using their actual ROS timestamps; RMS-per-ray DBSCAN radius 0.85 m/s.

Both models use `min_samples=3`. Each identified cluster is represented by a **recorded 720-vertex medoid polygon**: the sample in the cluster closest to the mean of the corresponding raw range vectors. For derivative clusters, displaying the associated real range polygon avoids misrepresenting signed range velocities as physical distances. The respective green and pink outlines follow each UAV's recorded pose and body yaw. If DBSCAN calls a frame noise, or the first temporal derivative is unavailable at the beginning, its actual scan is shown as a dotted line instead of falsely claiming membership. Both overlays disappear during manual dragging, since those positions have no recorded LiDAR measurements.

A second automatically invalidated, compressed local cache stores DBSCAN labels and cluster medoids under `~/.cache/nd_sociomind/follow_scenario/`, so subsequent runs do not repeat clustering. Tests: `PYTHONPATH=src /home/donkarlo/phd-venv/bin/python -m unittest discover -s tests -p 'test_follow*.py'`.

## Fast local replay cache

On the first launch with a particular ROS YAML, the application parses its GPS positions, odometry quaternion headings, and synchronized LiDAR scans and automatically writes a **compressed, non-executable data cache** under `~/.cache/nd_sociomind/follow_scenario/`. The cache contains the processed leader/follower trajectories, the follower's complete recorded route, and timestamp-matched LiDAR geometry. Subsequent launches skip both full YAML parsing passes when the cache is valid. A missing, stale, or damaged cache is rebuilt automatically; it is also invalidated when the source YAML's size/mtime, scenario settings, or relevant processing code changes. Cache writes are atomic. The original YAML, recorded coordinates, and Gazebo world are not modified. To rebuild on demand, remove only the appropriate file in that cache directory.

## Run

From `/home/donkarlo/Dropbox/repo/nd_sociomind_project`:

```bash
source /home/donkarlo/phd-venv/bin/activate
PYTHONPATH=src python -m nd_sociomind.experiment.autonomous_navigation.follow_scenario
```

For ND Control Panel, select the repository-root launcher:

```text
/home/donkarlo/Dropbox/repo/nd_sociomind_project/follow_scenario.py
```

The control panel already runs Python programs with `/home/donkarlo/phd-venv/bin/python`.


## Implementation map

- Data discovery: follow_scenario/config/data_source_settings.py
- Arena and interaction parameters: follow_scenario/config/scenario_settings.py
- Data processing: follow_scenario/data/reader/ and follow_scenario/data/processing/
- Robot–robot relation learning: follow_scenario/interaction/learning/learned_relationship_model.py
- Collision constraints: follow_scenario/interaction/safety/arena_safety_model.py
- Interactive motion: follow_scenario/interaction/controller/interactive_flight_controller.py
- Qt playback and interaction: follow_scenario/playback/ and follow_scenario/ui/
- Module launcher: follow_scenario/__main__.py

## Scope and limits

This is a replay and research visualization based on synchronized recorded GPS observations, not a controller for physical UAVs. It does not reconstruct new LiDAR scans when drones are dragged, and it does not by itself validate autonomous swarm behavior. A locally extracted first-lap ROS recording or the original ROS recording must be available to see genuine recorded range polygons. Two cleaned GPS files are required; their search order is defined by DataSourceSettings.candidate_directories(). Qt/PySide6 and the relevant project dependencies must be available.
