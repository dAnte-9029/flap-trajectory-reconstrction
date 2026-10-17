# Stage 01 synthetic GPTR validation

This harness uses the pinned external GPTR checkout. It does not read flight
logs, optimize real trajectories, or produce training labels. Run from the
trajectory repository root after activating `flap-recon`.

```sh
cmake -S experiments/stage01 -B runs/stage01_replay/original -G Ninja \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG'
cmake --build runs/stage01_replay/original --parallel 2
runs/stage01_replay/original/stage01_probe

cmake -S experiments/stage01 -B runs/stage01_replay/adapted -G Ninja \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG' -DSTAGE01_FIX_PSI_CUTOFF=ON
cmake --build runs/stage01_replay/adapted --parallel 2
ctest --test-dir runs/stage01_replay/adapted --output-on-failure
```

Use a fresh output directory for a new recorded run. `GPTR_ROOT` defaults to
`/home/zn/gptr`; the checkout revision is checked by CMake. No dependency is
downloaded by the build. Enabling C as well as C++ is necessary because the
installed SuiteSparse configuration looks up OpenMP's C component.

## Explicit compatibility boundaries

- `utility_bridge.hpp` suppresses upstream `utility.h` through its existing
  guard, supplying only sparse-block assignment and standard numerical includes.
  It avoids ROS/PCL dependencies; this is not a full unmodified ROS build.
- `query_adapter.hpp` bypasses `getStateAt`'s wrong right-end alpha and time
  snapping. All GP interpolation still runs in the upstream mixer.
- `AuditedSO3Manifold` implements actual right-update quaternion derivatives.
  `AmbientFactorAdapter` converts upstream padded local Jacobians to ambient
  Jacobians. Do not apply that conversion to ordinary AutoDiff factors.
- The optional PSI repair changes **one condition** in a build-local header
  copy: `Dtau < DOUBLE_EPSILON` to `Dtau == 0.0`. It preserves the exact endpoint
  shortcut without dropping the right endpoint's influence for 1 ms. The
  original header SHA is checked; the external fork is never edited. Only
  enable this with the explicit recorded patch, not an unreported source copy.

## Reading the output

The executable prints CSV. `PASS` refers only to the stated finite synthetic
case and tolerance. `KNOWN_ISSUE` deliberately reproduces defects/interfaces in
the original API; a zero exit code does not make that original API suitable for
label export. `MEASURED_ERROR` reports interpolation or quadrature error without
pretending it is physical reconstruction accuracy. `APPROXIMATION_LIMIT`
quantifies upstream small-angle derivative truncation; the PSI repair does not
change it. The current checks cover SO(3)xR3, not the SE(3) representation.

The synthetic joint solve uses independent analytic state anchors, a motion
prior, IMU factors, and the true manifold. It checks Ceres composition and
convergence on a known problem, not observability from PX4 logs. Separate
midpoint integration checks demonstrate quadrature convergence.

Upstream license statements remain applicable: GPTR root CC BY-NC-SA 4.0 and
factor/utility GPLv3+ headers differ. No upstream source or binary is vendored
in tracked files. The local build copy and executables are for this experiment;
redistribution requires module-level license review.
