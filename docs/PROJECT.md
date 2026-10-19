# Project charter

## Research question

**Full-observation work branch (2026-10-19):** the current task is offline continuous 6-DoF reconstruction using all valid native asynchronous observations. Its primary engineering evaluation is full-observation consistency (`in_sample_observation_fit`), together with kinematic consistency, observed dynamics retention and bounded parameter sensitivity. Held-out observation prediction is an auxiliary model diagnostic, not the main acceptance gate for this task. Independent physical-truth validation is a separate, still unverified task. Historical Stage 02–02.6 outcomes and limitations remain unchanged; this branch does not continue Stage 02.7 or authorize Stage03 formal labels.

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
- **G2 Reconstruction validity:** coordinate/time/integration closure, full valid-observation physical and whitened residuals, explicit uncertainty limits and parameter sensitivity, and fast-dynamics retention. Held-out predictions provide auxiliary diagnostics; neither in-sample fit nor a held-out score proves physical truth.
- **G3 Label export:** reproducible 50 Hz outputs and model integrator replay, with no use of future observations at deployment inference.
- **G4 Dynamics impact (separate authorization):** matched A/B/C training and free-running evaluation with unchanged architecture/splits/budget; report results against original and reconstructed references separately.

Success is not defined as smoother output looking visually smooth or having exactly zero integration residual alone. An inability to identify high-frequency motion from recorded sensors is a legitimate outcome.
