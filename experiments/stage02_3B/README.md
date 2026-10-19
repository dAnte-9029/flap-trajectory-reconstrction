# Stage 02.3-B: fixed-coefficient phase/GPTR composition

Scope: A/B development segments, no labels, no upstream edits. All local artifacts are under `runs/stage02_3B_20261019/` (ignored by Git). Frozen protocol: `configs/stage02_3B_joint.json`; Chinese findings: `docs/reports/02_3B_phase_aware_joint_reconstruction.md`.

## Representation and factor chain

`phase_motion.hpp` evaluates the exact cubic pieces exported from Stage02.3-A. No phase refit, resampling, time offset optimization or frequency pseudo-observation is performed. On each piece it returns phase and both derivatives. Outside support it fails.

For each harmonic h, fixed NED displacement and right-tangent FRD rotation coefficients multiply `[sin(h phi), cos(h phi)]`. Their derivatives are `[h phi_dot cos, -h phi_dot sin]` and `[h phi_ddot cos - (h phi_dot)^2 sin, -h phi_ddot sin - (h phi_dot)^2 cos]`. The translation and rotation coefficients have metres and radians respectively; IMU Fourier coefficients are never used as displacement initial values.

With Q=Exp(eta), omega_q=Jr(eta) eta_dot and alpha_q=Jr_dot eta_dot+Jr eta_ddot:

- p=p_s+p_osc, v=v_s+v_osc, a=a_s+a_osc.
- R=R_s Q, omega=Q^-1 omega_s+omega_q.
- alpha=Q^-1 alpha_s - omega_q cross (Q^-1 omega_s)+alpha_q.

The IMU factor predicts specific force `R^-1(a + [0,0,-9.80665]) + ba` and body rate `omega+bg` at their separate native timestamps. GNSS observes `p+R*l` and `v+R*(omega cross l)` at reception time minus assumed delay, whitened by the same Cholesky covariance as Stage02. No fused-state observation factors. All coefficient and bias Jacobians use Ceres Jets only through this local composition. GP endpoint Jacobians still use the pinned mixer and Stage01 ambient lift/PSI overlay.

There are 12H additional scalars. Gaussian coefficient priors are 0.05/h² metres and 0.05/h radians per sine/cosine axis. The WNOJ prior applies to the slow states, not the composed trajectory. This is an allocation assumption, not physical separation: a 50 ms GP still has enough bandwidth to carry periodic motion. `exchange.csv` measures the effect of moving 20% of the periodic coefficients into/out of slow knot states while preserving total knot states exactly, without reoptimizing.

## Reproduction

Use existing `flap-recon` C++ environment; Python base environment has SciPy 1.11.4. Do not install dependencies or edit upstream repositories.

```bash
PREFIX=/home/zn/anaconda3/envs/flap-recon
$PREFIX/bin/cmake -S experiments/stage02_3B -B runs/stage02_3B_20261019/build -G Ninja -DCMAKE_PREFIX_PATH=$PREFIX -DCMAKE_CXX_COMPILER=$PREFIX/bin/c++ -DCMAKE_C_COMPILER=$PREFIX/bin/cc -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
$PREFIX/bin/cmake --build runs/stage02_3B_20261019/build --target stage02_3B_synthetic stage02_3B_joint stage02_3B_query --parallel 2
python scripts/stage02_3B_prepare.py
python scripts/stage02_3B_run.py
python scripts/stage02_3B_analyze.py
python scripts/stage02_3B_diagnostics.py
python scripts/stage02_3B_finalize.py
```

For a fresh reproduction copy the config inside this repository, change only its `output` to a new directory, and set `STAGE02_3B_CONFIG` to the absolute copied-config path for every Python command. Create the new output directory and copy that config to its `frozen_config.json` before preparation; use its new build path in the CMake commands. The original run must remain intact. Existing observations/solve directories are deliberately refused. `jobs.json` and `execution.json` preserve exact commands. The run script stops on synthetic failure or unusable solve; an iteration-limited but usable result is preserved and must be reported as such.

The preparation script reuses the exact saved Stage02.2 complete-cycle mask, replaces both IMU row masks, and preserves GNSS masks. Entire held cycles are excluded from coefficient fitting and gyro initialization. EKF attitude initialization may still expose related IMU measurements; all claims remain conditional. Phase observations throughout are allowed exogenous information. Support clipping is common to every model, affects context edges only, and does not remove target cycles.

Query a stored solution at arbitrary supported times (CSV header `t`):

```bash
runs/stage02_3B_20261019/build/stage02_3B_query runs/stage02_3B_20261019/A_H3 times.csv new_query.csv
```

Output contains p/v/a, xyzw quaternion, body omega/alpha. Avoid extrapolation; target range is recorded in query settings. Knot coordinates are **slow** states; `trajectory.csv` is the composed trajectory. `slowpred` is the sensor prediction from the slow trajectory, so total-minus-slow is an observation effect with rotational coupling, not a separately identified physical accelerometer component.

Diagnostics use native-time predictions; spectra use full native streams, not low-rate held subseries. Independent finite differences, midpoint integration, knot limits, serialized query replay and a separate Python composition implementation check mathematical consistency. Integration closure does not certify physical truth. Instantaneous derivatives differ from interval effective increments. Parameter sensitivities are not calibrated posterior uncertainty.
