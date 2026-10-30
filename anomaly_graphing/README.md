# Anomaly graphing

This is a **schematic visualization**, not a reconstruction of anomaly scores measured in the historical ROS recording.

## Main three-plot application

Entry point: anomaly_graph.py

The PySide6 application builds three vertically stacked plots:

- GPS: an anomaly window covering approximately 10% of the lap.
- LiDAR: a centered anomaly window covering approximately 25% of the lap.
- Fused GPS + LiDAR: a centered 25% window, with an additional higher central 10% interval.

The displayed values are constructed from synthetic Gaussian-distribution pairs using Gaussian 2-Wasserstein distance. They are not calculated from the ROS GPS or LiDAR sensor values. The x-axis duration comes from the lap timing derived from data/one_round.yaml and is shifted to begin at zero seconds.

Thresholds are generated around a configured center of 0.125 with a half-range of 0.05, and may differ for GPS, LiDAR and fusion. The code does **not** assume a fixed threshold of 1.0. The default stochastic classification-error settings are 3% within an anomaly and 1% outside.

Two red dashed markers denote the nominal wall interval from 45% to 55% of the timeline. Settings are defined in anomaly_graphing/config/anomaly_graph_settings.py.

From the nd_sociomind_project root:

~~~bash
python anomaly_graphing/anomaly_graph.py
~~~

The script uses PySide6 and the local anomaly_graphing package. Timing input is expected at anomaly_graphing/data/one_round.yaml; consult the reader/settings for details if this file is absent.

## Separate two-curve anomaly-overlap figure

The second application, anomaly_overlap_graph.py, is separate from the three-plot GPS/LiDAR/fusion view. It draws two synthetic anomaly series:

- Each nominal anomaly interval occupies 20% of the 1000-sample timeline.
- The two nominal intervals overlap by 1% of the full timeline.
- Labels describe self-sensory anomalies versus anomalies from follower sensory data using leader phonemes.

~~~bash
python anomaly_graphing/anomaly_overlap_graph.py
~~~

Both plotting scripts are research visualizations; do not interpret generated curves as empirical detection performance.

## Implementation and testing

- anomaly_graphing/config/: visualization and threshold settings
- anomaly_graphing/data/reader/: historical timing input
- anomaly_graphing/data/processing/: Gaussian Wasserstein calculation and synthetic anomaly profiles
- anomaly_graphing/ui/: plot widgets and Qt window
- tests/test_synthetic_anomaly_pipeline.py: synthetic pipeline tests

~~~bash
python -m pytest anomaly_graphing/tests
~~~

Run commands require PySide6 and other relevant Python dependencies. A separate historical draft remains in README.new.md; this README documents the current default application.
