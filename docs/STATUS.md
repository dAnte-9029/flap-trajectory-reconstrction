# Project status

## Stage 02 completed — experimental reconstruction; Stage 03 not ready, 2026-10-17

The user explicitly authorized Stage 02. [Real-observation reconstruction report](reports/02_trajectory_reconstruction.md) records executed results and limitations. **No Stage 03, formal labels, neural training or controller changes were performed.**

- Reused Stage 00/01 evidence and fixed historical-train A/B/C SHA/HRT intervals. Adapted only these three raw ULogs, with 2 s context around each 16 s target; no full September audit or model-performance access.
- Implemented device-matched calibration, board/level rotation, native asynchronous IMU times, fixed-NED GNSS PVT/covariance/time/lever-arm handling, explicit validity/gap/reset checks and deterministic 20% observation holdout. EKF attitude is initialization only; no correlated EKF observation factors or imported biases.
- Built the pinned GPTR core with Stage 01 query/manifold/Jacobian/PSI adapters, plus a local Ceres PVT factor. **11 PVT synthetic checks, 6 independent geometry/calibration checks, CTest 1/1 passed.** Original raw files and upstream forks remain unchanged.
- Executed A short first, then 18 fixed 20/50 ms and decade-prior runs plus 12 delay/lever/initial-yaw sensitivities. Ten original 80-iteration runs exhausted their budget; identical-configuration 400-iteration reruns are separately preserved. **41 solves total, 31 final configurations converged.** This is numerical convergence, not physical acceptance.
- Exported experimental continuous p/v/a/R/omega/alpha, knots, native residuals, quality flags, independent derivative/integration checks, wing-phase harmonics/PSD/peaks and sensitivity envelopes. All large outputs remain local in `runs/stage02_20261017/`; final index is `delivery_manifest.json`.
- Baseline 20 ms held-out vector RMSE A/B/C: accel **12.389/13.007/15.546 m/s²** despite fitted RMSE near 0.1; gyro **0.247/0.262/0.634 rad/s**. GNSS held-out position **0.058/0.056/0.718 m**, velocity **0.494/0.321/0.732 m/s**. A/B windows are all fix 6; C all fix 4. These correlated holdouts are not independent truth.
- Main gyro-y wing fundamental gain is approximately 0.997/1.000/0.999, but stronger prior reduces its third harmonic to 0.439/0.408/0.273. Acceleration holdout versus harmonic retention is unresolved; no setting was promoted by lowest residual alone.
- Interior derivative checks and midpoint step refinement support numerical consistency at tested scales. Small-angle approximation remains: real local Jacobian errors reach ~7.51e-4; some B alpha knot limits retain ~1e-5–1e-4 rad/s² jumps. No new upstream derivative patch was made. Existing downstream 20 ms left-derivative integration has substantial discretization error even with exact reconstructed derivatives.
- GNSS delay, lever arms, rigid mounting versus structural vibration, instantaneous versus interval labels and calibrated uncertainty remain unresolved. Changing assumed delay by 110 ms shifts estimated position by ~0.68–1.14 m RMS and can alter attitude/acceleration materially. Saved envelopes are sensitivity diagnostics, not confidence intervals.

**Stage 03 decision: NOT READY for formal label production.** Before separate authorization, establish timing/extrinsic/reference-point evidence or restrict the label claim; resolve acceleration holdout and observed-harmonic tradeoffs; freeze validity/uncertainty and the continuous-to-discrete interface. GPTR remains an engineering-feasible prototype, not validated rigid-body ground truth. Reproduction: `experiments/stage02/README.md`; configuration: `configs/stage02_reconstruction.json`.

## Stage 01 completed — synthetic feasibility with limits, 2026-10-17

The user explicitly authorized Stage 01 and chose the `flap-recon` Conda environment. Its interrupted download was recovered and the core build/numerical work completed. [Method feasibility report](reports/01_method_feasibility.md) separates executed results, source analysis and scientific limitations. **At Stage 01 closure, Stage 02 was not authorized or executed; its subsequent authorization and results are recorded above.**

- Reused both Stage 00 reports and existing final audit CSVs. No raw ULogs or model performance were read, and no September statistics were rerun.
- Rechecked GPTR `2c0285a4f3141b131cc7b3aa77b3c6d979c0a2c2` and gnssFGO `16030fb6a24263a4b927d2ec5d1da266340ddb9b`, with upstream worktrees unchanged.
- **Built and executed:** repository-local CMake target against the pinned GPTR numerical sources, using a documented ROS/PCL utility bridge. Conda GCC 15.3.0, CMake 3.31.8, Eigen 3.4.0, Ceres 2.2.0 CPU and Sophus source 1.24.6; exact 90-package closure recorded. No full ROS application or gnssFGO build was performed.
- Reproduced wrong right-end alpha and near-knot snapping in `getStateAt`, and the nonstandard padded-tangent manifold convention. Tested `auditedQuery`, a true SO(3) manifold and explicit factor Jacobian conversion. Original API diagnostics remain separate from adapter checks.
- Reproduced the 1 ms `PSI` cutoff: a synthetic acceleration difference across 2 ns was 0.0014695 m/s². A one-line **build-local header patch**, guarded by the original SHA, reduced that difference to 2.94e−9 m/s². External GPTR source stayed unchanged; this is an explicit adapted build, not unmodified upstream behavior.
- Adapted C++ run: **47 PASS**, 21 measured interpolation/integration errors, 3 deliberately reproduced original-API issues, and 2 retained small-angle approximation limits; CTest **1/1 passed**. Quintic six-quantity errors ≤1.96e−14; IMU/prior local Jacobian errors ~5.83e−9/6.05e−8. Bias-only and joint-state synthetic Ceres solves passed. Independent midpoint integration exhibited second-order convergence.
- **Remaining numerical limit:** upstream small-angle truncation is not machine-precision differentiation. A tested small-angle state Jacobian error was 8.74e−5 normalized; the auxiliary derivative branch has a ~1.89e−4 jump. A requirement for strict global differentiability or 1e−6 Jacobians needs further stable-expansion validation before use. The PSI patch does not fix this limit.
- **Also executed:** 10/10 independent analytic convention checks in the new environment. Three fixed 16 s historical-train segment proposals plus 2 s context checked against existing Stage 00 coverage, boundary events and spectral-window evidence. A: good RTK/main IMU selected; B: good RTK/main IMU not selected; C: weak GNSS. Exact SHA/HRT intervals are in `configs/stage01_feasibility.json`.
- Defined the async accel/gyro, NED gravity/bias, GNSS PVT covariance/time/lever-arm adaptation plan and IMU-point versus rigid-body/center-of-mass limitations. The GNSS Ceres factor and real-log calibration adapter are not implemented.
- Final evidence: `runs/stage01_20261017/probe_{original,adapted}_final.csv`, `ctest_adapted.log`, `convention_checks_conda.json`, `prototype_segments.json`, `upstream_overlay.diff`, and `manifest_completed.json`. Initial blocked/earlier runs remain separately recorded. Reproduction: `experiments/stage01/README.md`; environment: `configs/environment.stage01.yml`. Large/third-party artifacts remain outside Git.

Next-stage conditions: separately authorize Stage 02; preserve tested numerical conventions and address small-angle accuracy requirements, device calibration/bias, frames, physical reference points/lever arms, timing and GNSS covariance. Implement/test the GNSS Ceres adapter before A/B/C prototypes. GPTR is the conditional first candidate; gnssFGO remains a GNSS design reference. Synthetic success is not real-flight accuracy, posterior uncertainty calibration, formal-label acceptance or control readiness.

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

## Stage 00 method decision (at closure)

**CONDITIONAL first choice: GPTR**, pinned to `2c0285a4f3141b131cc7b3aa77b3c6d979c0a2c2`, for its direct inertial observation factors, explicit p/v/a and R/omega/alpha states and closed-form SO(3) derivatives. Reuse `GaussianProcess.hpp`, GP motion prior and IMU factor; add repository-local ULog/calibration/time and GNSS PVT adapters. This is an applicability decision, not demonstrated reconstruction accuracy.

**CONDITIONAL alternative/reference: gnssFGO loose coupling**, pinned to `16030fb6a24263a4b927d2ec5d1da266340ddb9b`. Reuse GNSS PVT/lever-arm/time-association designs; the Full WNOJ implementation does have acceleration states. Continuous preintegration from the sparse logged deltas and tight coupling from missing per-satellite observations are blocked with current inputs. Numerical preintegration from calibrated sampled main IMU is a separate conditional possibility.

Module-level license statements differ from root licenses; reviewed and documented, no upstream source copied into tracked files. Neither upstream was built or modified during Stage 00; the subsequently executed Stage 01 core builds and local adaptation are recorded above.

## Stage 00 evidence and checks

- [Observation audit and per-date results](reports/00_observation_audit.md)
- [Paper/code comparison and next-stage modules](reports/00_method_applicability.md)
- Configuration: `configs/stage00_audit.json`
- Final raw audit: `runs/stage00_20261017/audit_final/manifest.json`
- Independent raw diagnostics: `runs/stage00_20261017/diagnostics_final/manifest.json`
- PX4/upstream source provenance: `runs/stage00_20261017/provenance_final/manifest.json` (7 logged firmware versions; 229 reviewed-source entries available)
- Validation: `runs/stage00_20261017/validation/checks.json` — **18/18 passed**, including all raw path hashes unchanged and identical spectra across the two raw reads.
- Focused tests: **11 passed**; Python compilation and scoped `git diff --check` passed. No full optimization, build or physical-accuracy validation was attempted.
- Delivery index/hashes: `runs/stage00_20261017/delivery_manifest.json`. Large results remain ignored by Git. Exploratory/failed checks are retained separately and superseded by final outputs.

## Stage 00 next-stage conditions (historical)

Choose fixed development-only prototype segments without model-test performance; pin their identities. Define device-specific calibration/bias, frame/gravity conventions, clock mapping/receiver delay and physical reference points/lever arms. Treat PX4 states as correlated priors/reference; verify candidate GNSS heading separately. Test continuous kinematic derivatives/Jacobians and downstream quadrature separately, then observation holdout, uncertainty sensitivity and measured dynamic retention. Preserve evaluation independence when fitting parameters.

At Stage 00 closure, Stage 01 was not authorized or started. Its subsequently authorized work is recorded above. No formal reconstruction labels or control-readiness claims exist.

## Verified work log

| Date | Stage | Result | Evidence | Decision |
|---|---|---|---|---|
| 2026-10-17 | 00 | All 91 unique September ULogs audited from raw bytes | Reports above; `stage00_20261017/delivery_manifest.json` | GPTR conditional priority; gnssFGO LC conditional reference; stop before Stage 01 |
| 2026-10-17 | 01 | GPTR core built; 47 adapted numerical checks and 10 analytic convention checks passed; three fixed segment proposals; implementation/approximation limits recorded | `reports/01_method_feasibility.md`; `stage01_20261017/manifest_completed.json` | Complete feasibility study; GPTR with explicit adapters conditionally preferred; stop before Stage 02 |
| 2026-10-17 | 02 | Three fixed real-log prototypes; 41 solves, 31 final configurations; synthetic/kinematic/dynamic/holdout/sensitivity evidence | `reports/02_trajectory_reconstruction.md`; `stage02_20261017/delivery_manifest.json` | Prototype completed with material acceleration/timing/rigidity limits; Stage 03 formal labels not ready |
