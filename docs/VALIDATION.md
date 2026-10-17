# Reconstruction acceptance and evaluation protocol

All metrics must use fixed, fully specified input segments; report flight-wise distributions and explicit pooled vs equal-flight aggregation. A plot without quantitative validation is not a pass.

## V1 Observation agreement

- Compare reconstructed p/v/R/omega with each **actually observed** source on its native measurement-time grid, accounting for attitude convention and lever arms.
- Report signed/axis/vector residuals, missing/invalid percentage, time lag, GNSS fix class and normalized residuals when valid covariance exists.
- Withhold independent observations/selected timestamps during a declared diagnostic and evaluate them only AFTER fitting; do not claim already-fused PX4 outputs are independent ground truth.
- If measurement errors/covariance are unknown, label any weighting as heuristic and run sensitivity analysis; do not invent standard deviations.

## V2 Kinematic consistency and integrator validity

- Check p ↔ v, v ↔ a, R ↔ omega, omega ↔ alpha in continuous-time method evaluation.
- Separately replay using the actual **flight dynamics model's discrete integrator** at ~20 ms (confirm exact numerical scheme). Numerical quadrature error is not a sensor error; do not force labels to satisfy an inaccurate integrator by distorting observations.
- Evaluate 1 step, 100/200/500 ms, and a longer diagnostic if observation support allows. Report local and cumulative residual, not only an endpoint scalar.

## V3 Dynamic retention

- Compare before/after angular rate peak/phase/time lag, derivative distributions, PSD near measured wing frequency and harmonics, and control-transition intervals. Use actual logged wing frequency and phase validity; don't assume a fixed frequency.
- Report whether high-frequency components are supported by actual sensors. WNOJ is still a regularizing prior and may smooth away dynamics if poorly tuned.
- Ensure no unnatural jump is introduced at GP segment boundaries, estimator switches, or GNSS dropouts.

## V4 Leakage/reproducibility

- Freeze all method choice and weights based on TRAIN (and pre-authorized DEVELOPMENT calibration); do not tune on reserved test flights.
- Offline reconstruction can be acausal; all later model histories, initial conditions and feature extraction must remain causal. Distinguish `offline_label` from `online_available_state` in exports.
- Each run gets a unique immutable `runs/<run_id>/manifest.json` with hashes, source commits, dataset split, topic choices, covariance/prior, sampled times, output schema and tests.

## V5 Downstream model A/B/C (NOT authorized by initial scaffold)

- A: original labels. B: ordinary smoothing. C: observation-constrained coherent labels.
- Same network architecture/input history/splits/windows/seeds/train budget. Score predictions against both original measurements and reconstructed states, with sensor-held-out checks.
- Improvement on C labels alone is NOT evidence of better physical dynamics or changed-command response. Control perturbation validation is a separate task.

## Acceptance policy

No arbitrary numeric success threshold is predefined without source uncertainty information. Stage reports must define and justify tolerances before tuning and must disclose regression across channels/cohorts. A method that suppresses observed fast motions fails even if its integration residual is tiny.
