# Agent guidance: flapping-wing trajectory reconstruction

This file sets repository-wide boundaries. Research definitions, stage plans, and detailed acceptance criteria live in `docs/`. Apply additional `AGENTS.md` files only if they exist in the directory being edited; do not create or duplicate them by default.

## Purpose

Build a reproducible **offline, observation-constrained, kinematically consistent 6-DoF trajectory reconstruction** pipeline from existing PX4/RTK flight logs. Export observation-supported position, velocity, orientation, angular velocity, linear/angular acceleration, validity, and uncertainty information for a separate dynamics-modeling project.

This is trajectory estimation and label construction, **not proof of physical truth or control-input-conditioned dynamics**. Prefer established published implementations and minimally justified adapters. GPTR is a current candidate; gnssFGO is a GNSS-factor reference, not a mandatory combined architecture. Select methods from verified observations and implementation evidence, not preference alone.

## Where to look

Read only what the task requires:

- `docs/PROJECT.md`: scientific purpose and project boundaries.
- `docs/DATA_CONTRACT.md`: ULog fields, time and frame conventions, source provenance, and label definitions.
- `docs/METHOD_DECISION.md` and `docs/UPSTREAM.md`: choosing or integrating published methods and external code.
- `docs/VALIDATION.md`: reconstruction quality, dynamics retention, and downstream evaluation.
- `docs/STATUS.md` and `docs/reports/`: continuing prior work or reusing verified findings.
- `docs/contracts/`: applicable stage plans and deliverables; consult the contract relevant to a requested stage.

Do not load all documents, papers, logs, skills, or upstream files as a startup ritual. Project documents guide the task; their proposed stage order does not by itself authorize new work. If an older document's default scope conflicts with the user's explicit current request, follow the requested scope while preserving data integrity and evaluation independence, and flag the outdated document.

## Working boundaries and initiative

Carry an authorized task through implementation, appropriate verification, and a usable result. Make reasonable reversible choices without repeatedly asking for permission. Do not stop at an initial implementation if the requested outcome includes running or checking it. Do not launch unrelated research stages or neural-network training merely because a later-stage document exists.

- Original `QgcLogs` files, the PX4/flight-controller sources, existing `flap-system-identification` data and experiments, and separately cloned GPTR/gnssFGO repositories are **read-only by default**. Ask before modifying them; keep necessary integration adapters in this repository.
- Do not overwrite established artifacts, silently change a registered dataset/split, or disguise an upstream implementation change. Record and review external licenses before copying or redistributing code.
- Routine edits, local analysis and focused tests **inside this repository** are allowed within the user's task. Seek approval for materially destructive operations, changes outside this repository, system-wide dependency changes, unusually costly builds/downloads, or heavy computation outside the agreed scope.
- Inspect existing tooling and paths as needed; `configs/paths.example.yaml` is not a verified local configuration. Use a compatible existing environment when available; do not block ordinary work over optional dependencies.
- Use delegation only where independent parallel subtasks materially help. Neither subagents nor exhaustive repository searches are mandatory.

## Scientific requirements

- Ground claims in actual raw observations or explicitly scope-qualified existing audits. Distinguish raw IMU specific force, calibrated/filtered quantities, GNSS/RTK solutions, PX4 fused EKF states, and derived targets. Do not double-count correlated measurements as independent evidence.
- Verify topic-specific `timestamp_sample`, publication `timestamp`, integration intervals, coordinate frames, sensor extrinsics, bias/gravity treatment, GNSS quality, estimator resets, and gaps when relevant. Unknown values remain unknown. Existing model conventions include local NED position/velocity and FRD body rates; verify raw topic conventions separately.
- Enforce consistent translational and rotational kinematics, including SO(3) orientation operations. Distinguish continuous derivatives, accelerometer specific force, body-frame kinematic acceleration, and downstream discrete integration. Do not fix an inaccurate integrator by distorting measured motion.
- Treat observations as uncertain; document evidence for weights and assumptions. Preserve observation-supported maneuver peaks, wing-phase/frequency structure and rapid transients. Smaller residuals or smoother plots alone do not establish greater truth; do not tune smoothing to downstream model errors.
- **Separate source discovery from model evaluation.** A user-authorized raw-log inventory may inspect all relevant logs regardless of their previous train/validation/test labels. Preserve those labels and exposure history. Do not use reserved evaluation outcomes for method selection, train/test contamination, or hyperparameter tuning. Offline labels may use future measurements; deployment-time histories, initial states, and other causal features may not.
- Judge reconstructed trajectories using applicable observation agreement (preferably independent held-out measurements), kinematic/integrator checks, uncertainty limitations, and dynamic retention. State when a motion or frequency band is not observable instead of inventing detail.

## Engineering, verification, and reporting

Reuse trustworthy existing code and evidence where applicable. Verify that an audit or result covers the actual inputs and method version before reusing it. Build the smallest meaningful change; avoid speculative frameworks, duplicate analyses, and unsupported claims.

Choose verification proportional to impact: a documentation edit may only need inspection, a code change needs relevant focused checks, and a scientific reconstruction needs the validation appropriate to its claims. Fix failures attributable to the change and rerun affected checks; broaden testing when a real risk warrants it, not automatically. No mandatory full-suite, repeated smoke-test, or subagent ritual.

For substantive data processing or experiments, write reproducible outputs with source identity/hashes, method and upstream commit, configuration, exclusions/validity, and results. Keep large data and run outputs out of Git; never present unexecuted tests as completed. Update `docs/STATUS.md` for verified project milestones, not routine edits.

Finish with a concise report of what changed or was learned, the evidence and checks actually performed, any important limitation, and the next decision only if one is needed. Distinguish observed facts, assumptions, and hypotheses.
