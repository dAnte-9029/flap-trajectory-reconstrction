# Project status (update only after verified work)

## Proposed next stage: Stage 00 observation audit (awaiting explicit user instruction)

No reconstruction or label training has been run by this new repository. This folder is only the setup scaffold.

## Evidence inherited from prior project (not reverified here)

- September collection archive spans 9/6–9/19 with existing duplicate-file and flight split inventory; past report `september_data_sufficiency_v1/review_report.md` documents 92 files / 90 unique, but it does not attest a new byte-level audit here.
- Past `angular_rate_measurement_audit_v1/report.md` audited 20 specific development flights: ~100 Hz main body rate/gyro, ~50 Hz attitude/local position, ~2 Hz vehicle_imu, asynchronous sample times and no logged gyro FIFO in that subset.
- Some flights contain estimator resets and variable RTK fix quality. Do not generalize individual flight measurements to all September logs.

## Unknowns that gate method selection

- `sensor_accel` / `vehicle_acceleration` / `sensor_combined` presence, usable measured sample frequencies and continuity in the chosen data.
- Whether `vehicle_imu` records usable delta-angle/delta-velocity over verified integration intervals.
- Detailed GNSS position/velocity source, quality, update timing and antenna lever arm.
- Matching, currently authoritative split manifest, prior test exposure and exact model integrator definition.
- Installed toolchain and build feasibility for the selected upstream commit.

## Proposed next action (not automatically authorized)

Follow `docs/contracts/00_observation_audit.md`, read-only; produce a per-flight source/observation inventory and an explicit feasibility decision. Do not implement GPTR/gnssFGO integration, run training, or modify prior repositories at Stage 00.

## Verified work log

| Date | Stage | Result | Report / manifest | Decision |
|---|---|---|---|---|
| TBD | Stage 00 | Not executed | None | Pending source audit |
