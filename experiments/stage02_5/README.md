# Stage 02.5: residual ledger and slow-trajectory reliability

Read-only evaluation of Stage02.4 saved trajectories. **Zero new trajectory solves**,
no changes to historical outputs, no label export or network training. The exact
error identity is `Y-y = (T+P-y) + (S-T) + (Y-S-P)`. All components use the same
native timestamp, FRD sensor convention and fixed bias convention. It is a
prediction ledger, not a decomposition into physical causes.

## Inputs and execution

Uses existing Python NumPy/SciPy/pandas/Matplotlib and the previously built
`runs/stage02_3B_20261019/build/stage02_3B_query`. No build or dependency installation.
The query binary opens old trajectories read-only and writes only to the new run.

For the registered run, configuration is `configs/stage02_5_diagnostics.json`.
To reproduce, choose a fresh output directory in a copied configuration and set
`STAGE02_5_CONFIG` to that file. Other design fields must remain unchanged.

```bash
cd /home/zn/flap-trajectory-reconstrction
# Prepare a new config without touching the historical run.
/home/zn/anaconda3/bin/python - <<'PY'
import json
from pathlib import Path
c=json.loads(Path('configs/stage02_5_diagnostics.json').read_text())
c['output']='runs/stage02_5_reproduction'
Path('/tmp/stage02_5_reproduction.json').write_text(json.dumps(c,indent=2)+'\n')
PY
export STAGE02_5_CONFIG=/tmp/stage02_5_reproduction.json
/home/zn/anaconda3/bin/python scripts/stage02_5_finalize.py --init
/home/zn/anaconda3/bin/python scripts/stage02_5_residuals.py
/home/zn/anaconda3/bin/python scripts/stage02_5_slow.py
/home/zn/anaconda3/bin/python scripts/stage02_5_window_probe.py
/home/zn/anaconda3/bin/python scripts/stage02_5_summary.py
/home/zn/anaconda3/bin/python scripts/stage02_5_plots.py
/home/zn/anaconda3/bin/python scripts/stage02_5_finalize.py
```

The window probe was separately frozen before viewing its results. It changes only
the measurement operator on the *same saved full trajectories*: instantaneous
versus a nominal 2.5ms, five-interval trapezoid, either centred or trailing. It is
not exact FIFO replay, latency calibration, or refitting with averaged factors.

The registered run also verifies and stores 18 read-only firmware snapshots against
A/B metadata hashes. Reproduction initialization repeats that check with `git show`
on the pinned commits, without changing the PX4 checkout.

## Output interpretation

- `samples/A_accel.csv` etc.: y/T/P/S/Y, e1/e2/e3/total, remaining, same masks,
  mechanical phase, cycle and distance to nearest retained same-sensor observation.
- `ledger.csv`: vector and axis MSE/RMSE, all **signed** cross terms, each cycle,
  1s bins and fixed 4s windows. Overall metrics are sample weighted; cycle quantiles
  treat each cycle as one item. Never sum RMSE or call these terms noise fractions.
- `tails.csv`, `cycle_summary.csv`: quantiles, top 1/5/10% SSE contributions,
  trimmed descriptive errors. No samples were removed from the primary scores.
- `phase_projection.csv`: train-cycle-balanced H1–H3 residual projection transferred
  to held samples; held-only fitted projection is descriptive and not used to
  modify any model. `phase_windows.csv` records nonstationarity in 4s windows.
- `spectra.csv`: normalized Lomb power on original times, full and gapped held
  sets separately. No stitching, FFT of joined cycles or interpolation. Relative
  band areas in `spectral_bands.csv` are *not* calibrated PSD/Parseval MSE shares.
  Sampling-window plots expose spectral leakage; 30–45Hz is exploratory only.
- `*_weights.csv`, `whitening.csv`: residuals against **remaining** for slow GPTR,
  actual Huber rho-prime for training and counterfactual values for held samples.
  Physical GNSS position/velocity RMSE are separate; only whitened norms combine
  them. Group costs cover the entire context, not only the 16s target.
- `factor_costs.csv`: native IMU/GNSS cost recomputation, saved motion-prior cost
  and single bias priors; reconstructed solver cost agrees within old six-digit
  group-export rounding. Costs under different q have different objectives.
- `coupling.csv`: exact conditional difference of body-frame acceleration,
  orientation acting on base specific force, and bias. Allocation is order-dependent.
- `interval_sensitivity.csv`: differences between saved A variants for 20/50/100/200ms
  endpoint increments and duration-normalized quantities. These are model-sensitivity
  diagnostics, not labels or measured physical uncertainty. SO(3) increments use
  rotations; delta body-rate components are not an inertial angular momentum change.
- `identity_checks.json`, `query_checks.json`, `manifest.json`: algebra, saved-query
  parity, source/input/binary/output hashes, fixed configs and actual scope.

The precision guard accepts <1e-12 raw CSV decimal round-trip differences, while
requiring exact timestamp joins and identical masks; the ledger tolerance is1e-10.
The initial bitwise raw-value assertion failure and its 3.55e-15 cause are recorded
in `implementation_notes.json`. No historical data or reconstruction bug was fixed.

Large artifacts remain local. Read the Chinese report for VERIFIED / SUPPORTED /
UNRESOLVED conclusions. The recommended GNSS velocity-factor mechanism test is a
future bounded experiment, **not performed here** and not authorization for Stage03.
