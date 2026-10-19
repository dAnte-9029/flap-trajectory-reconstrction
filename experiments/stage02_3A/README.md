# Stage 02.3-A: continuous mechanical phase

Scope: two fixed A/B encoder/Hall-indexed contexts; no 6-DoF optimization or labels.
Read `docs/reports/02_3A_continuous_phase_reconstruction.md` for the scientific limits.

Existing base Python supplies NumPy, SciPy 1.11.4, Pandas, Matplotlib and pyulog.
No dependency installation is required. Repository root is the working directory.

```bash
python scripts/stage02_3A_prepare.py
python scripts/stage02_3A_reconstruct.py
python scripts/stage02_3A_validate.py
python scripts/stage02_3A_plot.py
python scripts/stage02_3A_integral_followup.py
python scripts/stage02_3A_checks.py
python scripts/stage02_3A_seal.py
```

Before a new reproduction, change only `output` in `configs/stage02_3A_phase.json`
to an unused run directory. Preparation refuses to overwrite an existing run;
it freezes that config before fitting. This first execution used the same
preparation operations interactively before the reusable prepare entry point
was saved. Parameter/source snapshots and raw SHA checks were actually run.

The count selection is one common scale for A/B over the three preregistered
scales. Short-block scores and IMU cycle scores do not participate in selection.
The reusable H3 implementation is `experiments/stage02_2/phase_models.py`;
exact saved cycle boundaries/memberships and old predictions are checked.

## Query the one selected curve for a segment

```python
from experiments.stage02_3A.continuous_phase import ContinuousPhase

model = ContinuousPhase.load('runs/phase_reconstruction_20261019/A_phase_model.json')
t = [102.0, 102.1234]  # seconds in PX4 HRT, not wall clock
phi = model.query(t, 0)       # unwrapped mechanical cycle coordinate, rad
phi_dot = model.query(t, 1)   # rad/s; divide by 2*pi for frequency
phi_ddot = model.query(t, 2)  # rad/s^2, low-confidence derivative candidate
flags = model.flags(t)
```

Model JSON stores cubic B-spline knots/coefficients, the exactly restored affine
trend, time origin, valid domain, target interior and gauge metadata. Derivatives
come from this same curve. `query` rejects extrapolation/invalid time; `fit`
rejects count reversals, invalid observations and native gaps, requiring explicit
segmentation before other data can be used. Withheld samples are masked only
after full input validity checks. No separate frequency or Hall factor is fitted.

All 20 s are queryable as a fitted curve. The central original 16 s receive the
`interior_derivative_support` flag; natural spline end conditions make the outer
context less trustworthy for derivatives. No sample is flagged as having a
physically validated phase acceleration or calibrated absolute zero. The
`*_smoothing_envelope.csv` min/max values describe parameter sensitivity, not
confidence bounds. Neither encoder speed nor phase acceleration is a true wing
joint angular velocity/acceleration without the linkage geometry.
