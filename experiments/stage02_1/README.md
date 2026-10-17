# Stage 02.1 diagnostics

This directory reuses the pinned GPTR query/IMU factor and saved Stage02 curves. It implements no GP solver and does not modify upstream or the dynamics project. See `docs/reports/02_1_reconstruction_diagnostics.md` for scientific limits and results.

Inputs are the immutable local `runs/stage02_20261017/` delivery, including its observations, final validation, saved knots and original solver binary. No ULog parsing or model evaluation is performed. Output is `runs/stage02_1_20261017/`, ignored by Git. Commands refuse to replace established analysis/output directories. For a reproduction use a fresh output directory consistently in the config and commands, copying its frozen configuration before running. Do not remove the established run to rerun it.

The experiment plan was fixed before results in `configs/stage02_1_diagnostics.json`. The later, separately frozen `configs/stage02_1_offgrid_check.json` checks the grid-alignment concern found in the initial replay. It does not add optimization or change weights.

## Build (existing environment only)

```bash
conda activate flap-recon
cmake -S experiments/stage02_1 -B runs/stage02_1_20261017/build -G Ninja \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG'
cmake --build runs/stage02_1_20261017/build --target stage02_1_diagnostics --parallel 2
```

CMake inherits Stage02/01 pinned-source checks, utility bridge and build-local guarded PSI overlay. It compiles original `SE3JQ.cpp` and includes original `GPIMUFactor`; it does not run or rebuild the whole upstream application. The external repositories remain read-only. License review remains as documented in Stage01; no upstream code or binary is redistributed.

## Execution order

1. Verify every `artifact_sha256` entry in the Stage02 delivery manifest; record the manifest SHA. Copy the plan to the new run's `frozen_config.json`.
2. Run `python scripts/stage02_1_residuals.py`. It analyzes the 18 final main-grid configurations, exact native joins/calibration/whitening, training-only interpolation, stratified errors and full-native spectra.
3. Run the built diagnostic for A/B/C baseline (`A_full_dt20_q1`, `B_full_dt20_q1`, `C_full_dt20_q1_iter400`), with respective observation directories:

```bash
runs/stage02_1_20261017/build/stage02_1_diagnostics \
  runs/stage02_20261017/A_full_dt20_q1 \
  runs/stage02_20261017/observations/A \
  runs/stage02_1_20261017/factor_audit/A_full_dt20_q1 .02 audit
```

4. Only after all three audits pass, `python scripts/stage02_1_run.py` executes the four predeclared acceleration-mask experiments with the **original Stage02 binary**, verified against its saved SHA. The runner preserves all other CSV field strings and all other sensor files. Then run `python scripts/stage02_1_residuals.py --new`. No iteration-budget retries or parameter-selection loop exist here.
5. For integration, use the same CLI with `integrate` in place of `audit` and output under `integration/<run>`. Selected names: A/B `full_dt20_q0.1`, `full_dt20_q1`, `full_dt20_q10`, plus `A_full_dt50_q1_iter400`, `B_full_dt50_q1`; C only `C_full_dt20_q1_iter400`. Pass each run's actual .02 or .05 knot spacing. All step sizes and horizons are fixed in the diagnostic and listed in the plan.
For the six selected curves other than the A/B/C baseline, also run mode `audit` into `scalar_psd_query_check/<run>`. Verify maximum `saved_minus_query` < 1e-7 and record the six `{run, max_saved_query_error}` records in that directory's `checks.json`. The query constructor uses baseline PSD: common scalar covariance multipliers cancel in WNOJ conditional interpolation; these checks confirm the saved curves are preserved (observed maximum 4.80e-10), not reoptimized.

6. Analytic controls:

```bash
runs/stage02_1_20261017/build/stage02_1_diagnostics --analytic \
  runs/stage02_1_20261017/integration/analytic
```

7. Freeze/copy the off-grid plan. Run A/B baseline with mode `integrate_offgrid` into `integration_offgrid/<run>`. Only the initial offset changes, from .1 to .1037 seconds; there is no optimization.
8. `python scripts/stage02_1_summarize.py` generates all tables/figures and six grouped assertions, including exact analytic controls and endpoint identities. Run `python -m py_compile scripts/stage02_1_*.py`, inspect figures and `git diff --check`. Seal with `python scripts/stage02_1_seal.py`; after the code/report commit, refresh the manifest once to record the delivered commit.

## Interpretation and implementation details

- All sensor residuals are evaluated at their own `timestamp_sample`; accel is specific force, while the integrator receives kinematic body acceleration `R(t)^T a_N(t)`.
- The replay reproduces MainV2's left derivative velocity/rate update, constant-acceleration position update and midpoint-rate exponential orientation update. RK4 uses queried body derivatives at substages and its own evolving orientation, rather than resetting to reference orientation at each step. It is an oracle curve replay, not a neural prediction test.
- `interval_average` supplies endpoint velocity/rate changes but retains the original position/orientation update. `endpoint_moments` additionally supplies the separate position moment and SO(3) increment; its near-zero error is an algebraic identity, never independent accuracy evidence.
- Native error spectra include the periodic fit/held mask's modulation. Sparse held-out phase regression is conditional and alias-ambiguous; it is not a 12 Hz spectral validation.
- The mask experiments remain conditional on original gyro/GNSS masks and initialization. They are development diagnostics. No new independent holdout set, calibrated physical noise, or rigid-body truth is claimed.
- The initial integration/audit executable preceded addition of the off-grid CLI option. The final CLI preserves the initial default numerical path; the initial source snapshot is retained in run provenance. The initial diagnostic binary hash was not captured before rebuilding; the final binary and unchanged original Stage02 solver hashes are recorded. This does not affect the input/curve hashes or saved results, but is an explicit provenance limit.
