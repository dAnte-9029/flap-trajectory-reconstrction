
# Codex Operating Rules: Flapping-Wing Trajectory Reconstruction

## 1. Context and instruction policy

### 1.1 Repository instructions

Follow this root `AGENTS.md` for all work in this repository.

If a relevant subdirectory contains another `AGENTS.md`, follow its additional instructions when working within that subtree. More specific instructions take precedence within their scope.

Do not create additional `AGENTS.md` files unless a directory genuinely requires distinct operating rules. Do not duplicate the root instructions across directories.

### 1.2 Read only the context needed for the task

Do not read every project document at the start of every task.

Select documentation according to the requested work:

| Document                      | When to read                                                                          |
| ----------------------------- | ------------------------------------------------------------------------------------- |
| `docs/PROJECT.md`           | Project goals, research scope, or architectural decisions                             |
| `docs/DATA_CONTRACT.md`     | Sensor data, timestamps, coordinate systems, datasets, or label definitions           |
| `docs/METHOD_DECISION.md`   | Choosing or reconsidering a reconstruction method                                     |
| `docs/UPSTREAM.md`          | Inspecting or integrating GPTR, gnssFGO, or other external implementations            |
| `docs/VALIDATION.md`        | Designing, running, or interpreting reconstruction evaluation                         |
| `docs/STATUS.md`            | Continuing previous work, checking completed tasks, or avoiding duplicate experiments |
| `docs/contracts/<stage>.md` | Executing an explicitly authorized project stage                                      |

For ordinary code changes, read only the relevant source files, tests, and applicable specifications.

For a new research stage, read its active contract, the relevant project documents, and the evidence required to execute it.

Do not recursively scan all documentation, historical reports, generated artifacts, or upstream repositories unless the current task requires that scope.

### 1.3 Reuse existing evidence

Before repeating an analysis, check whether an applicable, verified result already exists.

Reuse existing reports, manifests, source hashes, and test results when their data, assumptions, versions, and methodology remain applicable.

Repeat an analysis only when:

- Its previous result is missing or unverifiable.
- Relevant input data or code has changed.
- The previous analysis does not answer the current question.
- A reproducibility check is explicitly required.
- The user explicitly requests a new evaluation.

Do not automatically reproduce completed experiments merely to establish context.

Do not treat historical conclusions as valid for new data or code versions without checking their applicability.

### 1.4 Explicit task authorization

This repository's documentation does not authorize work by itself.

Execute only the task or stage explicitly requested by the user.

The proposed stage sequence is:

- Stage 00: Observation audit.
- Stage 01: Upstream implementation feasibility.
- Stage 02: Observation-constrained trajectory reconstruction.
- Stage 03: Label generation and validation.

Do not infer authorization for later stages from the presence of their contract files.

Do not automatically start another stage after completing the current stage.

If the user requests a limited investigation, do not expand it into implementation, training, or new experiments without authorization.

---

## 2. Mission and project scope

### 2.1 Primary objective

Develop an offline pipeline for observation-constrained, kinematically consistent six-degree-of-freedom trajectory reconstruction using existing PX4 flight logs.

The intended outputs include:

- Position and velocity.
- Orientation and body angular velocity.
- Linear acceleration and angular acceleration.
- Observation validity and reconstruction quality information.
- Reproducible, physically interpretable state and derivative labels.

The reconstructed trajectories will support subsequent data-driven flapping-wing flight dynamics modeling.

The reconstruction project does not itself establish control usefulness or develop a controller.

### 2.2 Reuse established implementations

Prefer published, peer-reviewed, existing methods and their publicly available implementations.

Initial reference projects:

- GPTR: continuous-time Gaussian process trajectory representation, including third-order WNOJ motion priors.
- gnssFGO: continuous-time factor graph optimization and GNSS/multisensor observation modeling.

GPTR is the initial preferred candidate, not an irrevocably selected solution.

The final method must be selected based on actual data availability, algorithmic suitability, implementation feasibility, and validation evidence.

When reusing existing work:

1. Identify the exact relevant upstream components.
2. Confirm their mathematical assumptions and required observations.
3. Reuse existing implementations rather than reimplementing established algorithms.
4. Add only the adapters, observation factors, and interfaces necessary for the authorized task.
5. Document unavoidable deviations from the published implementation.

Do not independently implement a new Gaussian process solver, alternative toy optimizer, or competing reconstruction algorithm unless explicitly authorized and technically justified.

Do not reproduce the original papers' full experiments unless necessary for the requested validation.

Do not perform broad framework refactoring merely to integrate a small number of observation types.

### 2.3 Repository boundaries

The following repositories are external read-only sources by default:

- `flap-system-identification`
- `QgcLogs`
- GPTR fork
- gnssFGO fork

The upstream forks are maintained separately and are not vendored into this repository.

Never modify external repositories, original flight logs, flight-controller code, or existing dynamics datasets without explicit authorization.

Do not copy large portions of upstream code into this repository without a documented reason and license review.

### 2.4 Out-of-scope activities

Unless separately authorized, do not:

- Train or modify neural networks.
- Design or modify flight controllers.
- Run MPC or reinforcement learning experiments.
- Download additional research datasets.
- Install heavyweight dependencies.
- Build ROS or Docker environments.
- Modify PX4 firmware.
- Modify existing canonical datasets.
- Perform unrelated system identification experiments.
- Expand the project into general-purpose navigation software.

---

## 3. Scientific invariants

These requirements apply to all stages involving scientific data or reconstruction.

### 3.1 Measurement provenance

Never claim that a ULog topic exists, or quote its sampling frequency, unless supported by actual raw data or a specifically cited existing audit.

Existing audits cover subsets of September logs. Their findings must not automatically be generalized to every flight.

For each observation source, distinguish:

- Raw IMU measurements.
- Accelerometer specific force.
- Corrected PX4 acceleration.
- EKF-derived states.
- Raw GNSS observations.
- RTK-derived position and velocity.
- Derived or filtered quantities.

Do not treat correlated PX4 fused outputs and the original measurements used by the estimator as independent evidence without an appropriate correlation model.

### 3.2 Time and synchronization

Respect the distinction between:

- Measurement time, including `timestamp_sample`.
- Publication time, including `timestamp`.
- Sensor integration intervals.
- Filter and estimator delays.
- Physical actuator delays.

Do not assume that observations stored in one dataset row were measured simultaneously.

Use observation-specific measurement times for offline reconstruction when justified.

Do not infer physical sensor delays solely from differences in publication timestamps.

Unknown time offsets must remain explicitly documented rather than silently corrected.

### 3.3 Coordinate and measurement conventions

Verify all coordinate conventions against source metadata.

The existing flight-dynamics trajectory dataset uses:

- Local NED position and velocity.
- FRD body axes.
- Body-frame angular velocity.
- Body-to-NED quaternion orientation.

Do not assume that every raw PX4 topic uses the same coordinate representation.

Distinguish:

- Navigation-frame kinematic acceleration.
- Body-frame kinematic acceleration.
- Body-frame velocity derivative.
- Accelerometer specific force.

Account for gravity, sensor mounting orientation, lever arms, and bias when required by the chosen measurement model.

Use appropriate SO(3) rotation operations. Do not differentiate Euler angles componentwise as a substitute for rotational kinematics.

### 3.4 Kinematic consistency

The reconstructed trajectory must have a clearly defined relationship among:

- Position, velocity, and acceleration.
- Orientation, angular velocity, and angular acceleration.

Continuous-time consistency does not guarantee exact consistency with an approximate discrete integrator.

Explicitly distinguish:

1. Continuous-time trajectory derivatives.
2. Discrete-time increments.
3. The numerical integration scheme used by the existing dynamics model.

Validate reconstruction outputs using the actual downstream integration scheme.

Do not arbitrarily distort sensor-supported trajectories merely to satisfy a low-order numerical integration approximation.

### 3.5 Observation uncertainty

Treat measurements as uncertain observations rather than perfect ground truth.

Use documented noise models, covariance estimates, quality indicators, or defensible uncertainty assumptions.

Do not choose observation weights solely to minimize reconstruction error or produce visually smooth trajectories.

If uncertainty parameters are unavailable, record the assumptions and evaluate sensitivity.

Do not silently discard difficult measurements or motion intervals because they increase reconstruction residuals.

### 3.6 Preserve real flight dynamics

The reconstructed trajectory must preserve observation-supported dynamics, including:

- Maneuver peaks.
- Angular-rate variations.
- Transient responses.
- Relevant flapping-frequency components.
- Phase-conditioned dynamic features.
- Relative timing among observations and motion responses.

Evaluate possible attenuation or temporal distortion introduced by smoothing and reconstruction.

A visually smoother trajectory is not automatically a more accurate trajectory.

Do not classify model prediction errors or high-frequency motion as measurement noise without supporting evidence.

Do not select smoothing strength using downstream neural-network prediction error.

### 3.7 Data splits and leakage prevention

Preserve the authoritative whole-flight train, validation, and test assignments.

Never:

- Redefine data splits from filenames alone.
- Randomly mix windows from the same flight across partitions.
- Use test statistics to tune reconstruction hyperparameters.
- Access reserved test data without authorization.
- Describe previously exposed datasets as never opened.

Existing historical test exposure must be documented accurately.

Offline smoothing may use future observations to generate training targets.

However, future information must not enter causal model histories, initial states, feature normalization, or deployment inputs.

Keep offline reconstructed labels conceptually and operationally separate from causally available model inputs.

### 3.8 Interpretation of results

Maintain the distinction:

**Kinematic consistency is not equivalent to measurement truth, and neither proves input-conditioned dynamics accuracy.**

Do not interpret:

- Low reconstruction loss as proof of physical accuracy.
- Low error against reconstructed labels as independent validation.
- Reduced high-frequency variation as automatic noise removal.
- Improved trajectory prediction as proof of control-input response accuracy.

Claims must be supported by appropriate independent observations and controlled comparisons.

---

## 4. Execution and software environment

### 4.1 Verify paths

Before running project commands, resolve and verify the paths required by the active task.

`configs/paths.example.yaml` contains examples, not confirmed server locations.

Never silently assume an example path exists.

Record the resolved input and output paths in the corresponding run manifest.

### 4.2 Discover existing environments

Check the relevant installed software before proposing changes, including Python, Conda, Ceres, GTSAM, ROS, and other dependencies actually required by the task.

Do not copy interpreter paths or dependency assumptions from the old dynamics repository without verification.

Prefer existing compatible environments.

Do not install or upgrade dependencies unless authorized.

If a required environment is unavailable, report the blocker and the smallest necessary remedy.

Do not silently switch environments or modify system-wide installations.

### 4.3 Keep the implementation focused

For each execution, use:

- One authorized task or stage.
- One explicitly identified input set.
- One documented configuration.
- One dedicated output directory.
- One verifiable completion criterion.

Prefer existing project utilities when they satisfy the task.

Do not build new general-purpose infrastructure unless the existing implementation is demonstrably insufficient.

Avoid speculative compatibility layers, premature abstraction, unrelated cleanup, and broad refactoring.

Do not add dependencies or scripts merely because they might become useful later.

### 4.4 Upstream version control

Record the exact upstream repository and commit used for any implementation dependency.

Do not rely on mutable branch names alone for reproducibility.

Do not change upstream source branches, build options, or method hyperparameters incidentally to make an example work.

If a published implementation requires modification, first identify the minimal change and its justification.

Do not modify the upstream forks without authorization.

### 4.5 Data and artifact preservation

Original logs, existing model-ready datasets, and established experiment results are immutable inputs.

Never overwrite existing artifacts.

Every reconstruction run must preserve sufficient provenance, including:

- Source data identities and hashes.
- Relevant configuration.
- Reconstruction method and parameters.
- Upstream commit identifiers.
- Software version information.
- Validity masks and excluded intervals.
- Output artifact locations.

Use separate versioned output directories.

Large generated data, intermediate arrays, and compiled binaries should not be committed to Git unless explicitly intended.

### 4.6 Efficient debugging and testing

Prefer the smallest test capable of invalidating the current implementation or assumption.

On failure:

1. Identify the immediate failure.
2. Inspect the relevant evidence.
3. Apply the smallest justified correction.
4. Rerun the affected test.
5. Expand validation only when necessary.

Do not automatically launch extensive parameter sweeps, rebuild external frameworks, or run unrelated experiments after a failure.

Do not repeatedly execute previously completed tests when their relevant inputs and implementation have not changed.

---

## 5. Documentation and experiment management

### 5.1 Documentation responsibilities

Keep documents focused on their designated roles.

- `AGENTS.md`: persistent operating instructions.
- `README.md`: project orientation and navigation.
- `docs/PROJECT.md`: scientific mission and scope.
- `docs/DATA_CONTRACT.md`: data definitions and authoritative dataset conventions.
- `docs/METHOD_DECISION.md`: method-selection evidence and decisions.
- `docs/UPSTREAM.md`: external implementation references and versions.
- `docs/VALIDATION.md`: reconstruction evaluation requirements.
- `docs/STATUS.md`: verified progress and outstanding blockers.
- `docs/contracts/`: stage-specific execution instructions.
- `docs/reports/`: completed research reports.

Do not duplicate entire documents across multiple locations.

Use links or precise references instead.

### 5.2 Stage contracts

A stage contract defines:

- Authorized scope.
- Required inputs.
- Expected outputs.
- Scientific and implementation restrictions.
- Validation requirements.
- Completion criteria.
- Conditions that require stopping.

Execute only the contract explicitly activated by the user.

If a contract conflicts with a repository-wide scientific invariant, do not silently weaken the invariant. Report the conflict before proceeding.

### 5.3 Status management

Use `docs/STATUS.md` as the concise record of project progress.

Read it when continuing previous work or when necessary to determine whether a task has already been completed.

Update it only when there is a verified, meaningful change in project state.

Each update should identify:

- Completed task.
- Evidence or report location.
- Verified conclusion.
- Remaining uncertainty or blocker.
- Next eligible stage, if applicable.

Do not store extensive logs or repeat full reports in `STATUS.md`.

Do not mark a task complete before verification.

### 5.4 Method decisions

Record substantive reconstruction-method decisions in `docs/METHOD_DECISION.md`.

Every decision must distinguish:

- Established facts.
- Assumptions.
- Available implementation evidence.
- Known limitations.
- Conditions that would justify reconsideration.

Do not repeatedly reopen a settled decision without new evidence or an explicit user request.

Do not treat a preliminary method preference as a proven optimal choice.

### 5.5 Reports

Prefer concise reports that answer:

1. What was investigated or implemented?
2. What was observed?
3. What conclusions are supported?
4. What remains uncertain?
5. What is the next necessary action?

Separate measured results from interpretations.

Avoid generating multiple overlapping reports containing the same conclusions.

Use plots and tables only when they provide useful diagnostic evidence.

---

## 6. Completion contract

Every completed substantive task must report:

1. **Authorized scope:** What the user requested and what was actually performed.
2. **Evidence:** Exact input sources, relevant files, and verified assumptions.
3. **Changes:** Files created or modified.
4. **Execution:** Commands and tests actually run, including outcomes.
5. **Artifacts:** Output directories, manifests, and relevant hashes.
6. **Uncertainties:** Unresolved issues, limitations, and blockers.
7. **Next-stage gate:** Whether the current stage passed its acceptance criteria and what remains before proceeding.

Do not report unexecuted commands or planned tests as completed work.

Do not claim improved accuracy, physical validity, or deployment readiness without corresponding evidence.

Update `docs/STATUS.md` only with verified results.

Do not automatically execute the next stage.

---

## 7. General working principles

- Follow the user's explicitly authorized scope.
- Prefer verified evidence over assumptions.
- Prefer reuse over reimplementation.
- Prefer minimal changes over broad restructuring.
- Prefer reproducible experiments over undocumented exploration.
- Preserve source data and established experiment results.
- Separate scientific findings from engineering convenience.
- Avoid redundant analysis, unnecessary dependencies, and speculative work.
- Stop when the requested task is complete or a material blocker prevents valid progress.

The objective is not to produce the largest implementation or the smoothest trajectory. It is to obtain reproducible, observation-supported, kinematically consistent flight trajectories that provide credible labels for subsequent dynamics modeling.
