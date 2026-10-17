# Upstream code provenance and usage

Keep these as **separate Git clones**, not nested copies or vendored packages inside this project's version control. Pin the exact commits used in every experimental manifest.

| Role | Official source | Expected default branch | License / note |
|---|---|---|---|
| Main GP trajectory library | https://github.com/brytsknguyen/gptr | `master` | Repository LICENSE: CC BY-NC-SA 4.0; non-commercial academic use |
| GNSS factor reference | https://github.com/hz658832/gnssFGO | `ros2` | BSD-3-Clause; ROS 2 and adapted GTSAM, with submodules |
| Historical gnssFGO location | https://github.com/rwth-irt/gnssFGO | `ros2` | Archived; not the preferred fork source |
| Existing dynamics/data/splits | https://github.com/dAnte-9029/flap-system-identification | user-approved pinned branch | Read only from this new project |
| Raw log archive | https://github.com/dAnte-9029/QgcLogs | `main` | Git LFS; read-only data |

## Provenance verified in Stage 00 (2026-10-17)

| Verified local clone | Fork remote URL | Upstream remote URL | Revision SHA | Local changes |
|---|---|---|---|---|
| `/home/zn/gptr` | `https://github.com/dAnte-9029/gptr.git` | `https://github.com/brytsknguyen/gptr.git` (reference URL, not a configured remote) | `2c0285a4f3141b131cc7b3aa77b3c6d979c0a2c2` | clean |
| `/home/zn/gnssFGO` | `https://github.com/dAnte-9029/gnssFGO.git` | `https://github.com/hz658832/gnssFGO.git` (reference URL, not a configured remote) | `16030fb6a24263a4b927d2ec5d1da266340ddb9b` | clean |
| `/home/zn/flap-system-identification` | not needed for source audit | `https://github.com/dAnte-9029/flap-system-identification.git` | `2b561a09243baca3d95ae2982af1c6d9c4206620` | read-only; only historical identity CSV and discovery script read, no model performance |
| `/home/zn/QgcLogs` | `https://github.com/dAnte-9029/QgcLogs.git` | same | `385e4480e880afbb36f9f42d8aece8a1d8caa8ac` | clean; raw bytes hashed before/after |

## Entry points already verified

- GPTR: `gptr/include/GaussianProcess.hpp` declares the continuous trajectory representation; README mentions `GPUI.cpp` as UWB/inertial fusion example. Confirm exact file paths before referencing them in code.
- gnssFGO: `online_fgo/`, README describes ROS 2 continuous-time GP fusion and GNSS loose/tight coupling. It uses its own adapted GTSAM and other submodules; do not assume standard GTSAM is a drop-in substitute.

## Change policy

Start with source review and smallest upstream example. Keep any necessary adapter in this repo, not in the forks. If a fork patch is unavoidable, make the change explicitly authorized, isolated, and recorded by commit/diff. Ensure publication/reuse plans conform to each upstream license; GPTR's non-commercial ShareAlike terms deserve a separate check before redistribution.

Stage 00 source-file hashes and the 7 logged PX4 firmware revisions are in `runs/stage00_20261017/provenance_final/manifest.json`. The current PX4 checkout HEAD differs from logged firmware; semantics were inspected using `git show <logged ver_sw>:<path>`, without checking out or modifying PX4. GPTR root CC BY-NC-SA and factor GPLv3+ headers, and gnssFGO root BSD-3-Clause and core Apache-2.0 headers, require module-level attribution review before copying/distribution. No upstream source was copied into tracked files. See [method applicability](reports/00_method_applicability.md).
