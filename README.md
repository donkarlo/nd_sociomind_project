# ND Sociomind

## Purpose

ND Sociomind is a research and experimentation repository for interactions among robots, leader–follower coordination, multimodal recordings, and collective adaptation. The longer-term goal is a viable autonomous swarm using homeostatic regulation and active inference. That is the research direction, not a claim of complete swarm autonomy.

Python package: **nd_sociomind** (src layout).

## Main experiments and tools

### Autonomous navigation: leader–follower replay

See [Autonomous navigation documentation](src/nd_sociomind/experiment/autonomous_navigation/README.md).

- Implementation: src/nd_sociomind/experiment/autonomous_navigation/follow_scenario/
- Components: cleaned GPS reader, synchronized trajectories, lap detection, arena geometry, PySide6 playback UI, and interactive learned coordination.
- A dragged drone pauses playback; the other responds using local motion relationships derived from recorded GPS. Collision handling is based on the reconstructed arena, not hypothetical live LiDAR after dragging.
- Arena geometry and safety settings are defined in follow_scenario/config/scenario_settings.py (32 m outer square, centered 8 m inner square, narrowing walls, drone clearance).
- Data source discovery is defined in follow_scenario/config/data_source_settings.py.

From this repository root, after installing the required project packages and PySide6:

~~~bash
PYTHONPATH=src python -m nd_sociomind.experiment.autonomous_navigation.follow_scenario
# Or use the repository-root launcher
python follow_scenario.py
~~~

### Schematic anomaly graphs

See [Anomaly graphing documentation](anomaly_graphing/README.md).

- Primary entry: anomaly_graphing/anomaly_graph.py
- Three Gaussian Wasserstein-distance plots: GPS, LiDAR and fused GPS + LiDAR.
- **Anomaly magnitudes are synthetic**; the historical ROS YAML provides timing context, not the plotted distance values.
- A separate two-curve anomaly-overlap visualization is available at anomaly_graphing/anomaly_overlap_graph.py.

~~~bash
python anomaly_graphing/anomaly_graph.py
python anomaly_graphing/anomaly_overlap_graph.py
~~~

### Historical scenarios

src/nd_sociomind/experiment/part/oldest/ contains older normal/follow/next-to scenario definitions, multi-UAV composition, LiDAR utilities and GPS-related experiment code. Some files have unresolved dependencies or incomplete implementations; the old nd_world_project xpr command is not established as a standalone entry point for this repository.

## Structure and dependencies

- Package: src/nd_sociomind/
- Build metadata: pyproject.toml
- Cleaned replay data: data/autonomous_navigation/follow_scenario/ (also supports configured fallback paths)
- Anomaly timing source: anomaly_graphing/data/one_round.yaml
- Related repositories: nd_robotic_ai_project, nd_math_project, nd_physics_project, plus nd_utility
- Visualization and numerical dependencies include PySide6 and NumPy; these are not all declared in pyproject.toml.

## Development checks

~~~bash
python -m pip install -e .
python -m pytest anomaly_graphing/tests
~~~

The declared sociomind console entry imports historical experiment infrastructure and is not a replacement for the explicit visualization commands above.
