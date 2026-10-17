# Observation and data contract

## Source-of-truth hierarchy

1. Actual ULog bytes and hash; raw topic fields with real timestamps.
2. Pre-existing, scope-qualified audits with source hashes.
3. Model-ready exported datasets (derived outputs, NOT independent measurements).
4. Assumptions and explanations (must never masquerade as measured facts).

Expected source locations are recorded locally in `configs/paths.local.yaml` (untracked). September recordings are stored in `dAnte-9029/QgcLogs` using Git LFS. If the local clone only contains LFS pointers, audit cannot claim to have parsed ULog binary data. Do not pull all large files by default.

## Previously documented evidence (not an exhaustive 90-flight audit)

Existing reports in `flap-system-identification`:
- `docs/analysis/results/september_data_sufficiency_v1/review_report.md` on `control-dynamics-realdata-v1`: 9/6–9/19 inventory described 92 archived ULogs, 90 unique by hash; a split was already frozen. This is a report-level count, not a new raw-byte read in this repository.
- `docs/analysis/results/angular_rate_measurement_audit_v1/report.md` on the same branch: audited 17 validation flights and three 9/18 train flights; `vehicle_angular_velocity` / main `sensor_gyro` ~100 Hz; `vehicle_attitude` / `vehicle_local_position` ~50 Hz; `vehicle_imu` ~2 Hz. No `sensor_gyro_fifo` was found in that subset. Attitude and body-rate samples within a single export row were asynchronous.
- Earlier 9/6–9/8 audit reports variable RTK fixed quality, estimator resets and no validated relative-GNSS heading in the audited segments.

**Unverified / audit now:** coverage of `sensor_accel`, `vehicle_acceleration`, `sensor_combined`, gyro/accel integrals, per-sensor covariance, all September flights, and lever arms. Do not infer absence from old audit topic lists, nor assume 2 Hz `vehicle_imu` cannot contain meaningful accumulated deltas without inspecting its integration-time fields.

## Topic audit schema

For each ULog and topic instance: source hash, rows, measurement/publication time availability, observed rate distribution and gaps, fields, missing/finite ratio, units/frame, sensor source/selection, validity, reset counts and practical observation use. At minimum investigate:

- Raw/near-raw inertial: `sensor_gyro`, `sensor_accel`, `vehicle_imu`, `vehicle_acceleration`, `vehicle_angular_velocity`, `sensor_combined` if present.
- GNSS: `sensor_gps`, `vehicle_global_position`, `sensor_gnss_relative`, RTK fix/quality and all available position/velocity uncertainty fields.
- Fused state/reference: `vehicle_local_position`, `vehicle_attitude`, estimator selector/status, reset counters.
- Metadata (not direct motion truth): `wing_phase`, `encoder_count`, `flap_frequency`, actuators/commands.

Inspect firmware-specific UORB definitions if field semantics are uncertain. Do not invent topic meanings from names.

## Time and coordinate conventions

Use measurement-time alignment for OFFLINE smoothing when meaningful; keep publication time to enforce CAUSAL model inputs later. A hidden estimator filter delay is not necessarily resolved by `timestamp_sample`. Do not interpolate across estimator resets, long gaps, GNSS re-initialization, flight mode boundaries that change the reference, or changes in coordinate origin without explicit handling.

Prior model dataset: NED position/velocity, FRD body rates, quaternion body-to-NED. Verify actual source branch's attitude definition. `a_kin^B = (R_B^N)^T a_kin^N`. IMU-specific-force (after bias correction) satisfies `f^B = (R_B^N)^T(a_kin^N - g^N)` for a sensor colocated with the body reference. Include IMU/antenna lever-arm effects if material. `dot(v^B)` is **not** generally the same as `a_kin^B` because rotation adds `omega^B × v^B`.

## Split and privacy/data-integrity contract

Locate and hash the **authoritative currently approved** flight-level split in the modeling repository before use. Do not re-create splits via filenames, dates or random windows. Previously accessed test flights must not be called sealed/untouched. Default stage reads development flights only. No raw flight data should enter Git history; keep local pointers/symlinks and hashes only.
