# Stage 00: observation feasibility audit

**State: drafted, pending explicit user instruction.** Once authorized, this is a read-only inspection and reporting stage.

## Question

Do the **actual** 9/6–9/19 ULogs provide enough independent, timestamped observations to estimate p/v/R/omega and useful a/alpha labels without mislabeling unobservable high-frequency motion?

## Inputs and boundaries

- Resolve source paths from local config; verify every raw file is actual ULog, not a Git LFS pointer. Confirm and record source SHA.
- Discover currently authoritative approved flight list/splits in the modeling repo; preserve them unchanged. Audit DEVELOPMENT flights by default. The user must explicitly authorize opening additional reserved flights.
- Reuse existing audit code where practical, read-only, with source attribution. Do not copy unrelated training pipelines.
- No label regeneration, optimization, neural-network training, method installation, or test-performance checking.

## Required checks

1. Inventory hash de-duplication: ID/path/size/hash/date/split/flight. Distinguish old reports from actual new audits.
2. Every per-flight topic/instance: sample rate, P50/P95 gaps, duplicates, missing intervals, `timestamp`/`timestamp_sample`, fields and units.
3. Raw and corrected inertial: gyro/accel, deltas and integration dt if available; sensor selection and calibration provenance.
4. RTK/GNSS: RTK fix quality, independent position/velocity, uncertainty metadata, antenna offsets, valid time periods. Do not treat fused EKF and raw GNSS as independent without covariance model.
5. Position/orientation sources: GNSS/local, quaternion conventions, angular rates, estimator resets and origin changes.
6. Frequency/phase and control transition coverage to assess whether fast motion can be supported by observations.
7. Evidence-based method input compatibility matrix for GPTR / gnssFGO / GNSS-INS alternative. Separate support, absence and unknown.

## Deliverables

- `docs/reports/00_observation_audit.md` (short summary, with source provenance)
- `runs/<run_id>/inventory.csv`, `topic_coverage.csv`, `timing.csv`, `observability_matrix.csv`, `manifest.json` (machine-readable audit; all paths/hashes)
- List of verified usable and missing channels, with at least one flight-specific example per major failure mode
- Explicit PASS / CONDITIONAL / BLOCKED judgment for GPTR/gnssFGO/GNSS-INS. Never infer high-frequency acceleration accuracy from a small position residual.

## Done when

Every claim in the report is traceable to a specific raw file+topic or earlier scope-qualified audit. The current user's requested scope is satisfied without modifying any source, split, training code or original data. Update `docs/STATUS.md` with exact evidence and stop.
