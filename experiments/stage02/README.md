# Stage 02 real-observation prototype

This adapter uses the pinned GPTR SO(3)xR3 WNOJ mixer, motion prior and IMU
factor through Stage 01's actual manifold, Jacobian lifting and PSI patch.
It adds a project-owned GNSS PVT factor and experiment/validation drivers.
It is not a replacement GP solver. External source repositories stay read-only.
Root and factor license statements reviewed in Stage 01 still apply; no upstream
source or linked binary is tracked or redistributed here.

## Reproduction

Run from the repository root. Use a new run directory: extraction, individual
optimizer outputs and evaluations reject an existing directory. The example
uses the same `runs/stage02_20261017` name as the recorded experiment; replace
it everywhere to replay. The base Python already has pyulog, NumPy, SciPy and
Matplotlib; the existing `flap-recon` environment provides the C++ toolchain.
No package installation is performed by these scripts.

```sh
/home/zn/anaconda3/bin/python scripts/stage02_adapter_checks.py
/home/zn/anaconda3/bin/python scripts/stage02_extract.py \
  --output runs/stage02_20261017/observations
conda activate flap-recon
cmake -S experiments/stage02 -B runs/stage02_20261017/build -G Ninja \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG'
cmake --build runs/stage02_20261017/build --parallel 2
runs/stage02_20261017/build/stage02_reconstruct --test
/home/zn/anaconda3/bin/python scripts/stage02_run.py --mode short
/home/zn/anaconda3/bin/python scripts/stage02_evaluate.py \
  --runs A_short_dt20_q1 --output runs/stage02_20261017/short_validation
```

Inspect the short run's solver, native held-out residuals, derivatives and
phase response before proceeding. A finite optimizer result alone is not a
scientific acceptance gate. For the recorded experiment, the A short engineering
path was usable but acceleration holdout exposed overfitting. It was retained
as a negative result, not hidden by changing weights or selecting another segment.

```sh
/home/zn/anaconda3/bin/python scripts/stage02_run.py --mode A
# Inspect A results before B/C.
/home/zn/anaconda3/bin/python scripts/stage02_run.py --mode BC
/home/zn/anaconda3/bin/python scripts/stage02_run.py --mode sensitivity
/home/zn/anaconda3/bin/python scripts/stage02_run.py --mode convergence
/home/zn/anaconda3/bin/python scripts/stage02_evaluate.py \
  --output runs/stage02_20261017/final_validation
```

`convergence` repeats only runs that exhausted the original 80-iteration
budget, using 400 iterations with identical observations, weights and initial
states. It writes a new `_iter400` run, keeps the original evidence, and does
not select by residual or downstream model performance. Both are evaluated;
primary comparisons use the later converged run where available.

The post-fit verifier reads `knots.csv` and calls the same GP query; it does not
reoptimize. Save its stdout in a separate new validation directory:

```sh
runs/stage02_20261017/build/stage02_verify \
  runs/stage02_20261017/A_full_dt20_q1
```

## Schema and conventions

- All times are PX4 HRT seconds. Accelerometer/gyro retain separate sample
  times. GNSS query time is receive time minus an explicit assumed delay.
- `trajectory.csv`: p/v/a in fixed local NED; quaternion xyzw, body FRD to NED;
  omega/alpha in body FRD. The reference point is the main IMU under a rigid
  mounting assumption, not a verified center of mass. Dense 5 ms queries do
  not create 200 Hz measurement information.
- `knots.csv`: sufficient to reproduce the continuous curve with the pinned
  mixer, corrected query and explicit PSI patch. There is no extrapolation.
- Native prediction CSVs include observations, predictions, whitened residuals,
  validity, fit/holdout membership and GNSS fix. Withheld samples are correlated
  neighbors; EKF attitude was used for initialization only and can also carry
  information from those samples. No claim of independent ground truth.
- Raw sensor calibration follows device-matched CAL parameters, board roll-180
  and level adjustment once. Driver rotation/scaling was already applied in
  `sensor_*`. Thermal compensation is disabled in these logs. No EKF bias or
  state factors are added. A constant residual bias has one heuristic prior.
- The factor predicts antenna p+R*l and v+R*(omega cross l), whitens by the
  Cholesky factor of a positive-definite 6x6 covariance, and uses original GP
  state Jacobians plus the true SO(3) ambient lift. A fixed delay is applied
  before finding the knot pair. Delay and lever are not estimated as truth.
- Receiver PVT diagonal covariance is a proxy: eph/epv squared; the matched
  CAN driver's `s_variance_m_s` is already variance. Floors are added in
  quadrature. Missing cross-correlations are not assumed physically absent.
- The 20/50 ms and decade-prior grid, zero/110/220 ms delay, one nonzero lever
  and +10 degree initialization tests are frozen diagnostics. They are not a
  calibrated uncertainty interval or an exhaustive extrinsic search.
- Gap, timestamp-order, calibration-change and EKF reset checks fail closed.
  Invalid GNSS epochs/quality and IMU clipping are excluded explicitly. No
  amplitude-based IMU spike rejection or interpolation across gaps is used.
- Solver termination and usability differ. Post-fit central differences,
  knot-boundary scaling, independent midpoint quadrature and downstream
  MainV2 integration are separate outputs. The latter reads only integrator
  source, never model weights, datasets or performance.

Output quality files explicitly mark experimental-only status and uncalibrated
posterior uncertainty. The sensitivity envelope is not a confidence interval.
Formal Stage 03 labels remain prohibited until the report's conditions are met.
