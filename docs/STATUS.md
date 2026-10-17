# Project status

## Stage 00 completed — 2026-10-17

Branch: `reconstruction-audit`. User authorized read-only observation auditing of **all September 2026 raw ULogs**, including historical test/excluded logs. No historical model test performance was read or used. No optimization, label generation, training, dependency installation or Stage 01 was executed.

Verified scope: recursively inspected QgcLogs across 17 September directory names plus nested archives; 116 actual ULog paths → **91 unique SHA-256 contents**, 25 duplicate paths; separately identified 92 AppleDouble files. Parsed **13,645 topic/instance records and 280,551 field instances**. All 90 historical inventory hashes were included, plus the formerly unlisted `2026.9.8/log_8_2026-9-8-07-00-42.ulg`. Historical roles/exposure remain annotations, not admission criteria.

## Verified findings

- 18,506.34 s of log duration; 14,274.18 s marked airborne by recorded landed/armed flags. These flags and unique-log counts do not prove independent flight experiments or ground truth.
- All 91 contain main accel/gyro near 100 Hz, corrected/filtered angular velocity near 100 Hz, attitude/local state near 50 Hz, encoder/frequency/phase messages near 100 Hz. Raw chip rates are higher and must not be confused with logging rate. No logged gyro/accel FIFO.
- 90 contain GNSS PVT near 5 Hz in `vehicle_gps_position`; it is selected GNSS output, not fused EKF position. Its position/velocity samples match the same receiver's sparse `sensor_gps` records. Joint airborne main-IMU/GNSS support is 13,859.19 s; one log has no GNSS and remains included.
- RTK fixed contributes 94.26% of valid airborne PVT support pooled across logs, but individual ratios range 1.33–100%. Accuracy fields are receiver reports, not measured error. In the logged CAN adapter `s_variance_m_s` carries a velocity variance value, despite generic message naming.
- `sensor_gps.timestamp_sample` is zero throughout; publication-time fallback is explicit. `vehicle_gps_position` sample/publication timestamps coincide; receiver latency is still unknown. Relative-GNSS sample time has an unverified remote-clock mapping.
- Logged `vehicle_imu` deltas recur around 2 Hz but each integrates only ~5 ms, covering ~1% of elapsed time. They cannot form a contiguous preintegration sequence. Main sampled IMU remains usable subject to calibration, timing, gaps and bandwidth assumptions.
- 71 logs have partial CAN relative-GNSS messages. Several validity/baseline fields are not populated by the adapter. GNSS yaw is not yet validated: PVT heading accuracy is zero/invalid, and heading contains extreme outliers. Do not interpret default flags as proof of actual receiver failure or use zero baseline components as measurements.
- Phase support is valid in 90 logs; 89 have sufficient continuous 8 s spectral windows. Window wing-frequency P5/median/P95 is 3.173/3.867/4.429 Hz; main IMU provides about 26 samples/cycle and strong phase-linked components. This supports the fundamental, not arbitrary harmonics or physically exact acceleration.
- 38 logs contain recorded attitude reset changes; logger dropouts, estimator/origin changes and time gaps are reported by raw hash and interval. Main sensor 0 is not always the selected EKF IMU. Lever arms, precise sensor noise, full delay chain and absolute heading remain unresolved.

## Method decision

**CONDITIONAL first choice: GPTR**, pinned to `2c0285a4f3141b131cc7b3aa77b3c6d979c0a2c2`, for its direct inertial observation factors, explicit p/v/a and R/omega/alpha states and closed-form SO(3) derivatives. Reuse `GaussianProcess.hpp`, GP motion prior and IMU factor; add repository-local ULog/calibration/time and GNSS PVT adapters. This is an applicability decision, not demonstrated reconstruction accuracy.

**CONDITIONAL alternative/reference: gnssFGO loose coupling**, pinned to `16030fb6a24263a4b927d2ec5d1da266340ddb9b`. Reuse GNSS PVT/lever-arm/time-association designs; the Full WNOJ implementation does have acceleration states. Continuous preintegration from the sparse logged deltas and tight coupling from missing per-satellite observations are blocked with current inputs. Numerical preintegration from calibrated sampled main IMU is a separate conditional possibility.

Module-level license statements differ from root licenses; reviewed and documented, no upstream source copied into tracked files. Neither upstream was built or modified.

## Evidence and checks

- [Observation audit and per-date results](reports/00_observation_audit.md)
- [Paper/code comparison and next-stage modules](reports/00_method_applicability.md)
- Configuration: `configs/stage00_audit.json`
- Final raw audit: `runs/stage00_20261017/audit_final/manifest.json`
- Independent raw diagnostics: `runs/stage00_20261017/diagnostics_final/manifest.json`
- PX4/upstream source provenance: `runs/stage00_20261017/provenance_final/manifest.json` (7 logged firmware versions; 229 reviewed-source entries available)
- Validation: `runs/stage00_20261017/validation/checks.json` — **18/18 passed**, including all raw path hashes unchanged and identical spectra across the two raw reads.
- Focused tests: **11 passed**; Python compilation and scoped `git diff --check` passed. No full optimization, build or physical-accuracy validation was attempted.
- Delivery index/hashes: `runs/stage00_20261017/delivery_manifest.json`. Large results remain ignored by Git. Exploratory/failed checks are retained separately and superseded by final outputs.

## Unresolved conditions for a separately authorized next stage

Choose fixed development-only prototype segments without model-test performance; pin their identities. Define device-specific calibration/bias, frame/gravity conventions, clock mapping/receiver delay and physical reference points/lever arms. Treat PX4 states as correlated priors/reference; verify candidate GNSS heading separately. Test continuous kinematic derivatives/Jacobians and downstream quadrature separately, then observation holdout, uncertainty sensitivity and measured dynamic retention. Preserve evaluation independence when fitting parameters.

Stage 01 is **not authorized or started**. No formal reconstruction labels or control-readiness claims exist.

## Verified work log

| Date | Stage | Result | Evidence | Decision |
|---|---|---|---|---|
| 2026-10-17 | 00 | All 91 unique September ULogs audited from raw bytes | Reports above; `stage00_20261017/delivery_manifest.json` | GPTR conditional priority; gnssFGO LC conditional reference; stop before Stage 01 |
