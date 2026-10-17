# Project charter

## Research question

Can an offline, observation-constrained, kinematically consistent reconstruction of existing flapping-wing PX4 flights produce more credible state/derivative labels, while retaining observation-supported fast motions, and later improve the **same** dynamics model's free-running predictions?

## Target objects

A continuous-time or suitably discretized trajectory for position `p^N`, velocity `v^N`, orientation `R_B^N`, body rates `omega^B`, kinematic linear acceleration `a^N` (also exported as `a_kin^B`), and body angular acceleration `alpha^B`. The exact derivative and sampling definitions must be stated in every exported manifest.

The reconstruction is a **state-estimation / smoothing** problem conditioned on measured observations, NOT a neural model of how a new control command causes motion. Controls and wing phase are carried as metadata for later analysis, not a source of an invented trajectory prior.

## Preferred approach and non-goals

First evaluate reuse of GPTR's third-order WNOJ continuous-time representation with sensor observation factors. Use gnssFGO primarily for GNSS measurement conventions/factor design. Decision must follow Stage 00 audit, not paper popularity.

Out of scope without separate authorization: rewrites of PX4 or GPTR, new aircraft flights, controller design, MPC, RL, training runs, novelty claims, entire GNSS carrier-phase processing stacks, and proprietary/fullflight data upload.

## Gates

- **G0 Observation feasibility:** real raw channels, clocks, quality, covariance/extrinsics, resets and original split provenance audited per flight.
- **G1 Method selection:** verify upstream interfaces, reproducer feasibility, required observations and missing information; record decision and limits.
- **G2 Reconstruction validity:** coordinate/time/integration closure, normalized observation residuals, stable uncertainty, held-out observation predictions and fast-dynamics retention.
- **G3 Label export:** reproducible 50 Hz outputs and model integrator replay, with no use of future observations at deployment inference.
- **G4 Dynamics impact (separate authorization):** matched A/B/C training and free-running evaluation with unchanged architecture/splits/budget; report results against original and reconstructed references separately.

Success is not defined as smoother output looking visually smooth or having exactly zero integration residual alone. An inability to identify high-frequency motion from recorded sensors is a legitimate outcome.
