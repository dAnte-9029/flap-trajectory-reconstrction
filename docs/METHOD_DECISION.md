# Method selection record

## Stage 00 evidence update (2026-10-17)

The all-September raw audit parsed 91 unique ULogs; 90 have GNSS PVT and all 91 have main logged accel/gyro near 100 Hz. Logged delta integration covers only about 1% of elapsed time. GPTR remains the conditional first prototype choice for direct measurements and explicit SO(3)/R3 derivatives. gnssFGO loose coupling remains a conditional alternative and GNSS-factor reference; its Full WNOJ implementation does have acceleration states. Neither has been built or optimized here. See [observation audit](reports/00_observation_audit.md) and [paper/code comparison](reports/00_method_applicability.md).

## Candidate order (conditional; no reconstruction accuracy established)

1. **GPTR** (third-order continuous-time WNOJ, SO(3) / R3): best mathematical alignment with coherent acceleration/angular acceleration labels and asynchronously timed observations. Requires GNSS/ULog factor adaptation. The repository has Ceres/ROS sample pipelines; reuse its actual GP core.
2. **gnssFGO** (continuous-time GP factor graph, GNSS/multisensor system): strongest reference for loose-coupled GNSS observation handling; extensive ROS 2 and adapted-GTSAM dependencies; designed as online localization, not a ready offline label exporter.
3. B-splines / Kalibr as a published comparative family; Kalibr is NOT the released code of the specific B-spline paper discussed previously.
4. OB_GINS / GICI-LIB if actual source audit demonstrates continuous calibrated inertial data and a suitable GNSS interface; avoid unneeded satellite-raw pipeline complexity.
5. RTS smoother only as additional baseline, not a drop-in SO(3) label generator.

## Decision procedure

Before implementation, the current stage's report must answer:

- What exact usable observation sources exist for p, v, R, omega, a / alpha in all September raw ULogs authorized for Stage 00 (historical splits retained only as annotations)? Which are independent and which are fused/correlated?
- Are measurement-time stamps, uncertainty or outlier indicators available?
- Is reliable IMU preintegration supported by actual increment data, not nominal sensor configuration?
- Does candidate code represent translational and rotational derivatives on the correct Lie group? What is its interpolation/discretization error and GP prior's effect on 3–10 Hz motions?
- What upstream portions can be reused unmodified? Which necessary observation factors are missing?
- What are the license, build and runtime burdens, and what is the smallest upstream example that can be reproduced?

Issue a method decision with chosen method, rejected alternatives, observed evidence, external revision hashes and expected limitations. Do not implement a fresh custom GP optimizer merely because compilation/setup appears inconvenient.

## Hypotheses, not truths

A more coherent trajectory might reduce conflicting learning labels. It might also alter genuine dynamics or fail to improve input-conditioned prediction. The accepted method must be judged by independent observation support and dynamics preservation, not self-consistency alone.
