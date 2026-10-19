# Stage 02.6: GNSS velocity-factor ablation

Six frozen slow-GPTR solves, no new periodic coefficients or physical-periodic restoration. See [Chinese report](../../docs/reports/02_6_gnss_velocity_ablation.md) and [protocol](../../configs/stage02_6_ablation.json).

## Implementation

- `pvt_blocks.hpp`: marginal position / conditional velocity whitening, SPD and Schur checks; composes the existing Stage02 physical PVT residual and analytic Jacobians. G1/G2 share exactly the same position factory, which has no observed-velocity argument.
- `initialization.hpp`: fixed slope of adjacent **retained GNSS positions**; endpoint interval extrapolation matches position initialization. No GNSS velocity input.
- `joint.cpp`: local Stage02.3-B driver reuse, restricted to H0; only adds G0/G1/G2 selection, position-only initialization, initial-state/count exports and correct empty-group cost reporting. Stage01 PSI adapter, upstream GPTR, Ceres manifold, IMU factors, prior, masks and queries remain unchanged.
- `synthetic.cpp`: correlated covariance norm parity; combined Huber cost/gradient parity; tangent-space FD Jacobians with nonzero lever; identical G1/G2 position residual/Jacobian/whitening/Huber checks; velocity-poison initialization check; invalid-covariance rejection. Records every checked Jacobian element.

G0 uses one joint Huber. G1 uses two separate Hubers and is generally a different robust objective. **Only G1/G2 is the main velocity-factor intervention.** G2 still exports original GNSS velocity predictions for evaluation. Native `white0..5` columns retain the original full-PVT whitening for comparison; actual split-factor residuals/weights/costs are recomputed in `*_gnss_weights.csv` and `factor_costs.csv`. Held weights are hypothetical, never optimization weights.

No upstream code was copied or modified here. Existing local Stage02/02.3-B adapters are reused; external source and license decisions remain those documented in `docs/UPSTREAM.md` and earlier stages. GPTR source `/home/zn/gptr` must remain at `2c0285a4f3141b131cc7b3aa77b3c6d979c0a2c2`.

## Reproduce in a fresh output directory

Existing artifacts are local-only. Do not overwrite the completed run. Use installed base Python for analysis and installed `flap-recon` C++ environment; no dependency installation is needed. Only output relocation is allowed by the Python protocol loader.

```bash
cd /home/zn/flap-trajectory-reconstrction
/home/zn/anaconda3/bin/python - <<'PY'
import json
from pathlib import Path
p=Path('configs/stage02_6_ablation.json')
c=json.loads(p.read_text()); c['output']='runs/stage02_6_replication'
Path('/tmp/stage02_6_replication.json').write_text(json.dumps(c,indent=2)+'\n')
PY
export STAGE02_6_CONFIG=/tmp/stage02_6_replication.json
STAGE26_RUN=runs/stage02_6_replication
/home/zn/anaconda3/bin/python scripts/stage02_6_finalize.py --init
/home/zn/anaconda3/envs/flap-recon/bin/cmake -S experiments/stage02_6 -B "$STAGE26_RUN/build" -G Ninja \
  -DGPTR_ROOT=/home/zn/gptr -DCMAKE_PREFIX_PATH=/home/zn/anaconda3/envs/flap-recon \
  -DCMAKE_C_COMPILER=/home/zn/anaconda3/envs/flap-recon/bin/cc \
  -DCMAKE_CXX_COMPILER=/home/zn/anaconda3/envs/flap-recon/bin/c++ \
  -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
/home/zn/anaconda3/envs/flap-recon/bin/cmake --build "$STAGE26_RUN/build" \
  --target stage02_6_joint stage02_6_synthetic -j2
"$STAGE26_RUN/build/stage02_6_synthetic" "$STAGE26_RUN/synthetic.csv"
/home/zn/anaconda3/bin/python scripts/stage02_6_run.py
"$STAGE26_RUN/build/stage02_6_synthetic" "$STAGE26_RUN/synthetic_with_jacobians.csv"
/home/zn/anaconda3/bin/python scripts/stage02_6_analyze.py
/home/zn/anaconda3/bin/python scripts/stage02_6_summary.py
/home/zn/anaconda3/bin/python scripts/stage02_6_plots.py
/home/zn/anaconda3/bin/python scripts/stage02_6_finalize.py
```

The runner checks already-completed executions and refuses to launch them again. A failed solve is not retried. It permits only the frozen six main cases and, if the declared A-G0 historical-initialization trigger fires, exactly the paired traditional-initialization sensitivity. The completed original run did not trigger that pair. The two synthetic executions in the original run differ only by adding per-element Jacobian exports, not numerical methods or thresholds.

The `--init` and final seal verify 409 Stage02.4 and 174 Stage02.5 historical artifacts and inherited input hashes. Files used from prior stages must be present; this is not a standalone packaged dataset. Query evaluation uses the pinned saved Stage02.3-B executable, whose hash is recorded. No raw ULog re-extraction, upstream rebuild/modification beyond local overlay compilation, model training or label production.

## Artifact interpretation

- `*_execution.json`, solver logs, `initial_knots.csv`, `gnss_components.csv`: exact command/input identity, convergence, initialized states and actual factor counts (A80/B79 including context).
- `ledger.csv`, `*_ledger_points.csv`, `cycles.csv`, `quarters.csv`: fixed T/P/y error algebra, fit/held/axes/cycles, cross terms and tail contributions. `S+P` is a **sensor prediction addback**, not a complete periodic physical trajectory.
- `*_weights.csv`, `whitening.csv`, `factor_costs.csv`: all native context and core factor evaluations. Cost exports from inherited driver use six significant digits; total reconstruction tolerance .06 covers serialization, not an optimization change.
- `state_differences.csv`, `coupling.csv`, `cycle_wiggles.csv`, `interval_differences.csv`: same-time comparisons; jerk/snap are shape diagnostics of the saved curve. Off-grid interval splits use start-cycle membership and report boundary crossing. No calibrated uncertainty or physical-truth error is claimed.
- `gnss_accel_correlations.csv`: descriptive association of GNSS velocity error and native accel error in ±100ms windows. Overlap/conditionality prevents causal interpretation.
- `synthetic*.csv`, `*.jacobians.csv`, `evaluation_checks.json`, `manifest.json`: mathematical gates, numerical checks and provenance.
- `figures/index.html`: local self-contained gallery index with local PNG/SVG plots. Full-window plots and every held cycle accompany descriptive worst-increase zooms.

Failures retained: initial runner assertion used A's count80 for B's true count79; verification was fixed and resumed without recomputing any solve. An initial summary-only pandas query failed and was corrected. Neither changed scientific parameters. Six successful solves, no scientific tuning or solver retries.
