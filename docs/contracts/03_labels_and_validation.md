# Stage 03: label export and scientific validation

**Not authorized by initial scaffold.** Requires user approval and validated Stage 02 reconstruction.

## Purpose

Produce immutable state/derivative labels from one coherent continuous-time trajectory, and replay the downstream model integrator without using future observations in model inputs.

## Requirements

- Export time-aligned p^N, v^N, R_B^N (canonical quaternion order), omega^B, a_kin^B = R^T a^N, alpha^B, masks, uncertainty, method/source provenance, observed time vs publish time, and data split IDs.
- Explicitly record whether acceleration is kinematic or IMU specific force; do not confuse body-frame acceleration with derivative of body-frame velocity.
- Evaluate original vs reconstructed observation residuals; withhold selected independent observations in declared diagnostic; compare peaks, lag, PSD and phase-binned statistics.
- Replay exact downstream integrator (do not approximate its scheme) at the native nominal ~20 ms output grid; report p/v/R/omega closures at 1-step, 100/200/500 ms. If integration error is numerical, don't alter labels to hide it.
- Keep all raw sources, prior model datasets/splits and test gates unchanged. Avoid using future smoothed state as causal history or initial state.

## Outputs

- Versioned `labels.parquet` and `dataset_manifest.json` under `runs/<run_id>/` (or a separately approved export directory).
- `docs/reports/03_labels_and_validation.md`, including observation-fit vs motion-prior strength, dynamics-retention limitations and negative findings.
- A separate proposal for matched A/B/C downstream learning (no training without permission).

## Acceptance

Improvement must be justified by independent observation agreement and motion preservation, not a lower score against reconstructed labels alone.
