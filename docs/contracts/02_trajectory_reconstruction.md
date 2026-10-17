# Stage 02: observation-constrained trajectory reconstruction

**Not authorized by initial scaffold.** Requires completed G0/G1 gates and separate user approval.

## Goal

Implement the selected **published** optimizer via adapters, not new solver internals. Produce p/v/R/omega and observation-supported acceleration/angular acceleration with transparent uncertainty and quality flags.

## Required behavior

- Use actual observation-time data, calibrations, coordinate frames and verified lever arms. Handle resets/gaps by segmentation or explicit model, not hidden interpolation.
- Use audited independent measurements with covariance/quality-aware factors. Treat derived EKF outputs as correlated or initialize only; no double counting.
- State continuous-time GP prior (WNOJ or chosen alternative), process noise, knot times, boundary conditions, outlier policy and optimization convergence diagnostics.
- Confirm Lie-group orientation derivatives and translational acceleration definitions. No Euler-angle component differencing.
- Choose regularization by observation evidence and fixed development protocol. Compare high-rate dynamics/wing-phase and maneuver transitions before declaring success.
- Emit per-segment status (success/insufficient observations/unstable/out-of-domain); never silently fabricate missing intervals.

## Deliverables

- Reusable project-owned ULog adapters and factor wrappers with unit tests.
- Immutable versioned result in `runs/<run_id>/`: reconstructed trajectory, quality mask, covariance or honest limitation, manifest with upstream refs, optimizer convergence, residual summaries and timing diagnostics.
- `docs/reports/02_trajectory_reconstruction.md` listing successes and failed flight/segment cases, including **negative** results.

## Acceptance

Pass `docs/VALIDATION.md` observation/kinematic/dynamic checks on development data. No neural training. Freeze the reconstruction spec before label export.
