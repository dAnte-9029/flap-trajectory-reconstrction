# Stage 01: published-method reproduction and adaptation decision

**Not authorized by initial scaffold.** Run only after user explicitly approves Stage 01.

**Current authorization (2026-10-17):** the user explicitly approved Stage 01 on
`reconstruction-audit`, including a minimal local synthetic build and numerical
checks. Stage 00 results must be reused without repeating raw-log statistics.
This does not authorize Stage 02. The user subsequently chose to create the
`flap-recon` Conda environment from `configs/environment.stage01.yml`. Recovery
of its interrupted download and routine builds/tests within that environment
are within the authorized Stage 01 work; no system-wide installation or upstream
fork modification is authorized. The original missing-dependency stop is no
longer a blocker once that environment is available.

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
