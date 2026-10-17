# Stage 02.2 phase-aware feasibility

Scope: original A/B sensor exports plus a targeted read of their mechanical phase/encoder/frequency topics. No September re-audit, GP solver reimplementation, real phase-trajectory joint optimization, or labels. See the Chinese report in `docs/reports/02_2_phase_aware_reconstruction.md`.

## Reproduction

Use existing Python with numpy, scipy, matplotlib and pyulog. No dependency installation or ROS build is needed. Scripts refuse to overwrite established output directories. To reproduce, choose a **fresh output root** in `configs/stage02_2_phase_feasibility.json` and copy that config to its `frozen_config_phase_proxy.json` before regression. Do not delete the existing run.

```bash
python scripts/stage02_2_extract_phase.py
python scripts/stage02_2_regression.py
python scripts/stage02_2_synthetic.py
python scripts/stage02_2_checks.py
python scripts/stage02_2_summarize.py
python -m py_compile scripts/stage02_2_*.py experiments/stage02_2/*.py
python scripts/stage02_2_seal.py
```

Run from repository root. Stage02 source data and Stage02.1 native observed spectra must already exist. The new real-data work is bounded to A/B and deterministic complete-cycle holdouts; no hyperparameter selection loop is present. The originally frozen plan preferred Hall events; their absence was discovered before regression and the phase-crossing proxy amendment is retained. Baseline row-weighting was corrected to the declared cycle weighting with original results/source retained in `initial_v1/`; principal Fourier/control outputs are identical.

`phase_models.py` contains finite Fourier weighted least squares and guarded phase interpolation. Per-cycle weights sum to one. The local affine model chooses eight training cycles by time, not by response or error. In the last-eight-cycle test there are no future IMU training cycles; measured phase of the query cycles remains available. Thus this is conditional observation prediction, not free flight prediction. All fit memberships and ranks are recorded. A constant phase shift is an invalid negative control because complete sine/cosine coefficients absorb it; use the predeclared per-cycle random offsets or constant-frequency clock phase.

`periodic_motion.py` contains only the chain-rule and SO(3) composition of a supplied slow trajectory and periodic position/rotation perturbations. The synthetic driver supplies analytic slow states and C2 coefficient splines; no real phase smoothing, GPTR joint factor, or physical allocation is claimed. The zero-angle right-Jacobian branch and time derivative are checked independently. Existing GPTR small-angle limitations remain.

## Reference snapshots

The user-supplied PDF in `docs/papers/Qian ...pdf` is read-only and stays untracked. Its SHA is in `reference/paper.json`. Public upstream commit: `3453d1898b08b864cc0012ee2b93e97ad025b3da` in `XJX-UAV/Flapping-wing-State-Estimation`. `reference/upstream/` holds the nine README/source/build/package files, downloaded with URLs of the form:

```
https://raw.githubusercontent.com/XJX-UAV/Flapping-wing-State-Estimation/3453d1898b08b864cc0012ee2b93e97ad025b3da/<path>
```

`reference/tree.json` records the commit tree, and `upstream_sources.json` records file hashes. No LICENSE was found and package.xml says TODO; source is not copied to tracked code or redistributed. Reused elements are mathematical ideas only. Matched PX4 source snapshots were obtained read-only with `git show <logged-firmware>:<path>` for WingPhase, messages and AS5600; they are not modified.

## Evidence limits

- phase_valid is software validity, not an independent Hall-event audit; no hall_event topic exists in A/B. phase_unwrapped_rad resets at Hall zero and cannot directly supply global C2 derivatives.
- flap_frequency is a filtered version of the same encoder source, not an independent frequency measurement.
- Sensor regression is not a six-DoF rigid-body reconstruction. Primary held cycles differ from Stage02 sample holdouts; their RMSE cannot be ranked directly against the old GPTR error.
- Whole-cycle harmonic fits are used instead of FFT of concatenated missing cycles. The full-native observed PSD is reused unchanged from Stage02.1.
- Synthetic motion/nuisance equivalence is a counterexample, not proof every actual mode is unobservable. Actual known-phase GNSS design rank is reported separately and is not a joint physical observability test.
- Local affine coefficients worsen last-eight-cycle extrapolation; peak/harmonic losses and non-benefiting axes are retained. No setting was promoted from minimum error alone.

After committing, refresh `delivery_manifest.json` to record the delivered commit; the seal checks previous Stage02 and Stage02.1 artifact hashes. Remote verification is saved separately and included in the final seal. Large outputs, upstream snapshots and binaries remain local.
