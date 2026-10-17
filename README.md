# Flapping-Wing Trajectory Reconstruction

## Purpose

Reconstruct an observation-constrained, kinematically consistent **offline** 6-DoF trajectory from existing PX4 ULog recordings, then export well-defined state and derivative labels for the separate flight-dynamics modeling project.

This repository owns the reconstruction **adapters, contracts, quality analysis and exported labels**. It does not own original ULogs, the flight controller, the dynamics training pipeline, or either upstream trajectory-estimation project.

## What to read

- Human introduction and success criteria: `docs/PROJECT.md`
- The meaning, sources and limitations of the measurements: `docs/DATA_CONTRACT.md`
- Which published implementation to reuse and the decision gate: `docs/METHOD_DECISION.md`
- What counts as an acceptable reconstructed trajectory: `docs/VALIDATION.md`
- Upstream origins, licenses and pinned commits: `docs/UPSTREAM.md`
- Current evidence and the proposed next step: `docs/STATUS.md`
- Stage-specific scope: `docs/contracts/`

## Current stage

**Proposed next step: Stage 00, read-only source/observation audit.** This scaffold does not itself authorize execution. Run Stage 00 only when the user explicitly asks Codex to do so. No reconstruction algorithm has been selected or labels validated.

## Local path configuration

Copy `configs/paths.example.yaml` to `configs/paths.local.yaml` and set absolute paths after verifying they exist. The local file is ignored by Git. These paths are deliberately *not* guessed here.

## Repo layout

- `src/flap_traj_reconstruction/`: project-owned ULog, sensor-factor and GPTR integration **wrappers**, added only as stages require
- `scripts/`: small, auditable CLI entrypoints, added only as stages require
- `tests/`: mathematical, data-contract and synthetic tests, added only as stages require
- `configs/`: explicit data, method and runtime configuration
- `docs/contracts/`: drafted stage scopes and acceptance criteria (each stage needs explicit task authorization)
- `docs/reports/`: small reviewed summaries, not large binary data
- `runs/`: immutable per-run manifests, outputs and plots (ignored by Git)

## Governance

`AGENTS.md` contains the Codex operating policy. Research assumptions belong in `docs/*.md`, not in extra duplicated AGENTS files. Never silently fall back to another dataset, split, source commit, frame convention, or loss definition.
