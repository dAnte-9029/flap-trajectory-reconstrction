# Upstream code provenance and usage

Keep these as **separate Git clones**, not nested copies or vendored packages inside this project's version control. Pin the exact commits used in every experimental manifest.

| Role | Official source | Expected default branch | License / note |
|---|---|---|---|
| Main GP trajectory library | https://github.com/brytsknguyen/gptr | `master` | Repository LICENSE: CC BY-NC-SA 4.0; non-commercial academic use |
| GNSS factor reference | https://github.com/hz658832/gnssFGO | `ros2` | BSD-3-Clause; ROS 2 and adapted GTSAM, with submodules |
| Historical gnssFGO location | https://github.com/rwth-irt/gnssFGO | `ros2` | Archived; not the preferred fork source |
| Existing dynamics/data/splits | https://github.com/dAnte-9029/flap-system-identification | user-approved pinned branch | Read only from this new project |
| Raw log archive | https://github.com/dAnte-9029/QgcLogs | `main` | Git LFS; read-only data |

## Provenance table to fill after clone

| Local clone (from `configs/paths.local.yaml`) | Fork remote URL | Upstream remote URL | Revision SHA | Local changes |
|---|---|---|---|---|
| GPTR | TODO | `https://github.com/brytsknguyen/gptr.git` | TODO | unknown until inspected |
| gnssFGO | TODO | `https://github.com/hz658832/gnssFGO.git` | TODO | unknown until inspected |
| modeling repo | TODO | `https://github.com/dAnte-9029/flap-system-identification.git` | TODO | read-only |
| log repo | TODO | `https://github.com/dAnte-9029/QgcLogs.git` | TODO | read-only |

## Entry points already verified

- GPTR: `gptr/include/GaussianProcess.hpp` declares the continuous trajectory representation; README mentions `GPUI.cpp` as UWB/inertial fusion example. Confirm exact file paths before referencing them in code.
- gnssFGO: `online_fgo/`, README describes ROS 2 continuous-time GP fusion and GNSS loose/tight coupling. It uses its own adapted GTSAM and other submodules; do not assume standard GTSAM is a drop-in substitute.

## Change policy

Start with source review and smallest upstream example. Keep any necessary adapter in this repo, not in the forks. If a fork patch is unavoidable, make the change explicitly authorized, isolated, and recorded by commit/diff. Ensure publication/reuse plans conform to each upstream license; GPTR's non-commercial ShareAlike terms deserve a separate check before redistribution.
