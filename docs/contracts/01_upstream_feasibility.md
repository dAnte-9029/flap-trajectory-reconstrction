# Stage 01: published-method reproduction and adaptation decision

**Not authorized by initial scaffold.** Run only after user explicitly approves Stage 01.

## Goal

Choose the minimum change needed to reuse an upstream published continuous-time trajectory estimator for the audited observations. Do not recreate the GP engine from scratch.

## Tasks

- Pin clone and submodule revisions; record fork/upstream URLs and licenses.
- Identify GPTR classes for SO(3)/R3 trajectories, acceleration and angular acceleration evaluation, Ceres parameterization and sensor-factor patterns. Investigate gnssFGO GNSS covariance, time interpolation and quality rejection implementations.
- List which factors already exist vs which must be adapted from measured ULog fields. Note if RTK solution is already fused or independent.
- Assess library/toolchain build constraints without silently installing ROS/Docker or replacing Ceres/GTSAM dependencies.
- Reproduce the smallest upstream synthetic/example result **only within authorized environment scope**; if missing dependencies, document blocker and stop.
- Produce a decision table: GPTR main, gnssFGO main, or alternative, with precise reasons and observed data compatibility.

## Deliverables and acceptance

- `docs/reports/01_method_feasibility.md`; upstream commit IDs and exact reusable files/functions; additional licensing questions; planned adaptation surface.
- Zero changes to upstream without explicit approval; no trajectory claims or label export yet.
- A bounded Stage 02 specification based on actual source evidence. Update STATUS and stop.
