# Stage 02.4: train-only separation and fixed-slow physical recovery

Scope: A/B development windows only. No GP solver reimplementation, upstream edits,
formal labels, neural training or new flight selection. Report:
[`02_4_qian_style_separation.md`](../../docs/reports/02_4_qian_style_separation.md).

## Model and interfaces

1. `stage02_4_prepare.py` reuses Stage02.2 cycle-balanced affine-trend + fixed H3
   regression and Stage02.3-A continuous encoder phase. Only training complete
   cycles determine any regression statistics. **Subtract only periodic columns**;
   preserve the time trend and unexplained residual. Poisoning held IMU leaves
   learned coefficients exactly unchanged.
2. `stage02_4_slow.py` invokes the existing Stage02.3-B H0 executable, unchanged
   50ms GPTR, q=0.1, calibration, asynchronous times, masks, GNSS and initialization.
3. `recover.cpp` fixes every slow knot and bias, then projects the learned periodic
   sensor template into the existing physical composition `phase_motion.hpp`.
   Its IMU pseudo-target is `forward(slow)+bias+trained_periodic_sensor_template`.
   The 36 unknowns are 18 NED displacement coefficients (metres) and 18 right
   rotation-tangent coefficients (radians). Original training GNSS factors constrain
   the composed curve. Coefficient priors are 0.05/h² m and 0.05/h rad.
   This deterministic sequential projection is **not** two independent sensor
   likelihoods or a calibrated Bayesian posterior. No free nuisance is estimated.
4. `pred*` in native prediction CSVs is the actual physical forward prediction.
   `addback*` is only slow sensor prediction plus the learned template. These are
   intentionally different outputs. `slowpred*` includes the same fixed biases.
5. The published GPTR interpolation, local Stage01 PSI/query/manifold corrections,
   Stage02 PVT factor, and Stage02.3-B composition/Jacobians are reused. No Qian
   source is redistributed; the finite Fourier method is a mathematical adaptation.

The main reference point is the primary IMU, assuming rigid installation. Positions
are local NED; rates FRD; orientation maps body to NED. Use native clock seconds.
Phase is mechanical cycle coordinate, not wing joint angle. 110ms GNSS delay,
zero lever arm and no additional phase delay are assumptions, not calibrations.

## Reproduction in a fresh directory

Requires existing hashed Stage02.2 cycles, Stage02.3-A phase models and Stage02.3-B
observations/binaries/results. This deliberately avoids repeating historical stages.
Do not rerun into an established run directory. Only output relocation is supported
through `STAGE02_4_CONFIG`; the other frozen design fields must remain unchanged.

```bash
cd /home/zn/flap-trajectory-reconstrction
export STAGE02_4_CONFIG="$PWD/runs/stage02_4_reproduction/frozen_config.json"
/home/zn/anaconda3/bin/python - <<'PY'
import json, os
from pathlib import Path
p=Path(os.environ['STAGE02_4_CONFIG']);p.parent.mkdir(parents=True,exist_ok=False)
c=json.loads(Path('configs/stage02_4_separation.json').read_text())
c['output']=str(p.parent.relative_to(Path.cwd()))
p.write_text(json.dumps(c,indent=2)+'\n')
PY
export STAGE24_OUT="$PWD/runs/stage02_4_reproduction"
export STAGE24_ENV=/home/zn/anaconda3/envs/flap-recon
"$STAGE24_ENV/bin/cmake" -S experiments/stage02_4 -B "$STAGE24_OUT/build" -G Ninja \
  -DGPTR_ROOT=/home/zn/gptr -DCMAKE_PREFIX_PATH="$STAGE24_ENV" \
  -DCMAKE_C_COMPILER="$STAGE24_ENV/bin/cc" -DCMAKE_CXX_COMPILER="$STAGE24_ENV/bin/c++" \
  -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
"$STAGE24_ENV/bin/cmake" --build "$STAGE24_OUT/build" --target stage02_4_recover -j2
/home/zn/anaconda3/bin/python scripts/stage02_4_prepare.py
/home/zn/anaconda3/bin/python scripts/stage02_4_slow.py
/home/zn/anaconda3/bin/python scripts/stage02_4_projection.py
/home/zn/anaconda3/bin/python scripts/stage02_4_synthetic.py
/home/zn/anaconda3/bin/python scripts/stage02_4_run.py
/home/zn/anaconda3/bin/python scripts/stage02_4_analyze.py
/home/zn/anaconda3/bin/python scripts/stage02_4_diagnostics.py
/home/zn/anaconda3/bin/python scripts/stage02_4_summary.py
```

The real recovery runner checks both separation and synthetic gates before
executing its bounded plan. It performs two primary recoveries, four A-only
slow/recovery pairs, and one coefficient restart; together with the two primary
slow solves this is **13 real solves**. Synthetic recovery is a separate two solves.
No extra orthogonality penalty or unconstrained joint refinement was run.

The finalizer additionally needs `reference_review.json` in the fresh run; reuse the
review from `runs/stage02_4_20261019/` (same pinned paper/source, no new review claim),
then run `scripts/stage02_4_finalize.py`. It verifies inherited hashes, fixed slow
states/biases, same-mask inputs, convergence and finite states before sealing output
hashes. `git_delivery.json` is a separate post-commit record and is not self-hashed.

## Continuous query

`trajectory.csv` is only a convenient 5ms export, not the definition of the curve.
The existing exact query binary reads saved slow knots, phase pieces, coefficients
and settings, so arbitrary valid timestamps can be queried:

```bash
runs/stage02_3B_20261019/build/stage02_3B_query \
  runs/stage02_4_20261019/A_restore /path/to/times.csv /path/to/query.csv
```

Input is a CSV headed `t`; queries must stay inside the saved valid target domain.
Outputs p/v/a, quaternion R, body omega/alpha derive from the same composition.
`quality_flags.csv` marks research validity, **not physical truth**.
`uncertainty.json` and the A sensitivity table describe available limits; no calibrated
covariance or B sensitivity envelope is invented. Jacobian information is conditional
on the frozen slow estimate, template, phase, biases and mounting assumption.

## Evidence files

- `observations/*/*_decomposition.csv`: raw, periodic, remaining, train trend,
  unexplained, native time and cycle masks.
- `metrics.csv`, `cycle_metrics.csv`: physical versus signal-addback scores kept separate.
- `axis_separation_support.csv`, `held_projections.csv`, `slow_projections.csv`,
  `harmonics.csv`, `peaks.csv`: all axes and negative outcomes retained.
- `error_decomposition.csv`: exact error identity including the cross term.
- `synthetic/validation.json`: oracle recovery versus actual H3-template approximation.
- `*/kinematics.csv`, `*/integration.csv`, `phase_boundary_limits.csv`: independent
  differences/integration; numerical closure is not physical accuracy.
- `sensitivity.csv`, `fixed_slow_exchange.csv`, `periodic_amplitudes.csv`, `complexity.csv`:
  conditional allocation/stability, with no claim of full physical identifiability.
- `index.html`: native-time plots, whole-cycle errors, harmonics and full-stream
  Lomb power (not a calibrated PSD or a spectrum of the sparse held subsequence).
