# Anomaly graphing (historical draft)

**Documentation status:** This is an older draft. Consult README.md in this directory for the current code settings (notably the variable thresholds near 0.125 rather than the fixed 1.0 described below) and the separate two-curve overlap application.

This application intentionally produces a schematic one-lap comparison of the historical normal and follow UAV scenarios.

The numerical anomaly values are synthetic. They are generated as correlated stochastic-looking Gaussian Wasserstein-distance sequences so the curves are irregular rather than smooth deterministic shapes.

- x-axis: Time (s).
- y-axis: Anomaly (Gaussian Wasserstein distance).
- threshold: 1.0.
- GPS: the centered 10% interval is anomalous and approximately 95% of that interval is above threshold.
- LiDAR: the centered 25% interval is anomalous and approximately 95% of that interval is above threshold.
- Fused GPS + LiDAR: the centered 25% interval is anomalous and the central 10% is amplified further.
- The area under each curve is lightly shaded only inside that plot's intended anomaly interval.

The generator constructs synthetic pairs of diagonal Gaussian distributions and evaluates their 2-Wasserstein distance.

Run:

```bash
/home/donkarlo/phd-venv/bin/python /home/donkarlo/Dropbox/repo/nd_sociomind_project/anomaly_graphing/anomaly_graph.py
```
