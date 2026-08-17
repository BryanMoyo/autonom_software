# Autonomous Robotics — Master Engineering Roadmap

## Objective

Develop a robust, fault-tolerant autonomy stack for an agricultural
robotics platform, with future applicability to excavation and other
heavy mobile machinery.

The development process is hardware-first and failure-driven.

The system must not be optimized merely to achieve high simulation
scores. Robustness failures are engineering evidence and must be
preserved, understood, documented and mitigated.

---

# Operating Rules

The agent must:

1. Work sequentially through the roadmap.
2. Inspect existing code before modifying it.
3. Reproduce relevant tests before making major changes.
4. State the engineering hypothesis before implementing a significant
   architectural change.
5. Make the smallest justified change.
6. Run regression tests after every meaningful change.
7. Record quantitative results.
8. Preserve previous failure cases.
9. Never weaken a robustness test to improve the score.
10. Never fabricate results.
11. Never claim simulation performance is field performance.
12. Keep architectural decisions documented.
13. Stop for human review before major safety or architectural changes.

---

# Stage 1 — Baseline Freeze

## Objective

Establish a reproducible software baseline.

## Tasks

- Inspect the complete repository.
- Identify the simulation entry points.
- Identify navigation, localization, robot and sensor modules.
- Identify all existing robustness tests.
- Identify all existing result files.
- Reproduce the nominal test.
- Reproduce the current robustness scenarios.
- Reproduce Trial 04 diagnostics.
- Record the exact software configuration.

## Required outputs

Create:

results/baseline/

results/baseline/BASELINE.md

The document must contain:

- commit hash
- Python version
- dependencies
- test commands
- nominal result
- robustness results
- Trial 04 result
- known failures
- timestamp

## Exit criterion

The current baseline can be reproduced from a clean checkout.

---

# Stage 2 — Requirements

## Objective

Convert observed failures into formal engineering requirements.

## Tasks

Review S1-S10.

For every shortfall define:

- failure mode
- trigger
- observable symptom
- affected state variable
- severity
- required mitigation
- validation method

Create:

docs/REQUIREMENTS.md

## Exit criterion

Every S1-S10 shortfall has a traceable requirement and validation method.

---

# Stage 3 — State Estimation Architecture

## Objective

Create a formal state-estimation architecture.

## Required state

### Vehicle

- x
- y
- z
- velocity
- roll
- pitch
- yaw
- angular velocity

### Tool

- boom configuration
- arm configuration
- bucket configuration
- bucket pose
- cutting-edge pose

### Environment

- terrain
- obstacles
- boundaries

### Confidence

- position uncertainty
- heading uncertainty
- sensor health
- source agreement

## Tasks

- define state interfaces
- define sensor interfaces
- define coordinate frames
- define update rates
- define timestamp requirements
- define confidence representation

Create:

docs/STATE_ESTIMATION_ARCHITECTURE.md

## Exit criterion

All major state variables have defined sources and interfaces.

---

# Stage 4 — IMU/INS Simulation

## Objective

Introduce inertial sensing.

## Tasks

Implement:

- acceleration
- angular velocity
- bias
- noise
- drift
- timestamp
- sensor failure
- sensor degradation

The simulator must support:

- nominal IMU
- noisy IMU
- biased IMU
- intermittent IMU
- complete IMU failure

## Testing

Run existing robustness tests.

Add IMU fault-injection tests.

## Required output

docs/IMU_VALIDATION.md

## Exit criterion

IMU degradation can be simulated and measured quantitatively.

---

# Stage 5 — Track/Wheel Odometry

## Objective

Introduce mechanical motion sensing.

## Tasks

Implement:

- wheel/track displacement
- velocity
- encoder noise
- slip
- bias
- encoder failure

The model must distinguish:

true vehicle movement

from

measured mechanical movement.

## Testing

Test:

- nominal odometry
- noise
- systematic bias
- slip
- complete failure

## Required output

docs/ODOMETRY_VALIDATION.md

## Exit criterion

The estimator can use odometry as an independent motion source.

---

# Stage 6 — LiDAR Localization

## Objective

Introduce environmental localization independent of GNSS.

## Tasks

Implement a simulation abstraction for:

- LiDAR scan generation
- environmental features
- relative motion estimation
- local pose correction
- localization confidence
- LiDAR dropout
- degraded LiDAR

The implementation does not need to reproduce a real LiDAR
algorithm perfectly at first.

It must provide a testable abstraction with realistic failure modes.

## Testing

Test:

- GNSS dropout with LiDAR available
- GNSS + LiDAR agreement
- LiDAR dropout
- conflicting GNSS/LiDAR estimates

## Required output

docs/LIDAR_LOCALIZATION.md

## Exit criterion

The system has a non-GNSS localization source.

---

# Stage 7 — Confidence and Sensor Health

## Objective

Prevent the autonomy system from blindly trusting degraded sensors.

## Tasks

Implement:

- sensor health
- measurement validity
- uncertainty
- source disagreement
- localization confidence
- heading confidence
- confidence thresholds

The estimator must expose both:

STATE

and

STATE CONFIDENCE.

## Example

position:

x = 10.03
y = 20.17

confidence:

position = HIGH
heading = MEDIUM

sources:

GNSS = unavailable
IMU = healthy
odometry = healthy
LiDAR = healthy

## Required output

docs/CONFIDENCE_SYSTEM.md

## Exit criterion

The system can identify degraded state confidence.

---

# Stage 8 — Degraded Autonomy

## Objective

Make sensor degradation change robot behaviour.

## Modes

A0 — Manual

A1 — Assisted

A2 — Supervised autonomy

A3 — Degraded autonomy

A4 — Safe state

## Required behaviour

Normal confidence:

FULL AUTONOMY

Reduced confidence:

REDUCED SPEED

Further degradation:

LIMITED AUTONOMY

Unsafe localization:

SAFE STOP

## Testing

Inject:

- GNSS failure
- IMU failure
- odometry failure
- LiDAR failure
- multiple sensor failures

## Required output

docs/DEGRADED_AUTONOMY.md

## Exit criterion

Sensor failures produce deliberate operating-mode changes.

---

# Stage 9 — Fault-Injection Campaign

## Objective

Systematically characterize the complete architecture.

## Test dimensions

### GNSS

- noise
- dropout
- long dropout
- intermittent dropout

### IMU

- noise
- bias
- drift
- dropout

### Odometry

- noise
- bias
- slip
- failure

### LiDAR

- noise
- localization degradation
- dropout

### Combined failures

Test combinations of:

- GNSS + IMU
- GNSS + odometry
- GNSS + LiDAR
- IMU + odometry
- multiple simultaneous failures

## Required metrics

- completion rate
- maximum position error
- maximum path error
- heading error
- localization error
- time to detect failure
- time to enter degraded mode
- time to safe stop
- recovery time
- false-positive failure detection
- mission-state consistency

## Required output

results/fault_injection/

results/fault_injection/FAULT_CAMPAIGN.md

## Exit criterion

A quantitative robustness map exists for the autonomy architecture.

---

# Stage 10 — Hardware Specification

## Objective

Convert validated software requirements into a physical robot architecture.

## Specify

### GNSS

- accuracy
- update rate
- correction method
- antenna architecture
- environmental rating

### IMU/INS

- bias stability
- noise
- update rate
- vibration tolerance
- environmental rating

### Odometry

- encoder resolution
- update rate
- slip detection
- interface

### LiDAR

- range
- field of view
- point rate
- update rate
- dust/weather considerations
- mounting architecture

### Cameras

- resolution
- frame rate
- field of view
- environmental protection
- lighting requirements

### Machine sensing

- boom sensing
- arm sensing
- bucket sensing
- hydraulic state
- CAN/CAN-FD integration

### Compute

- CPU/GPU requirements
- RAM
- storage
- real-time requirements
- thermal requirements
- environmental rating

### Communications

- CAN/CAN-FD
- Ethernet
- time synchronization
- service communication

### Safety

- emergency stop
- safe-state architecture
- watchdogs
- redundant critical signals
- operator override

## Required output

docs/HARDWARE_SPECIFICATION.md

Include:

- sensor requirements
- proposed sensor classes
- interfaces
- update rates
- accuracy
- environmental requirements
- redundancy
- estimated cost
- rationale
- traceability to S1-S10

## Exit criterion

The software robustness findings have been converted into a
traceable preliminary hardware specification.

---

# Validation Principle

Every hardware mitigation must be tested against the same failure
scenario that exposed the original weakness.

Example:

GPS dropout

↓

software baseline failure

↓

add independent localization

↓

rerun identical GPS dropout

↓

compare:

- localization error
- path error
- mission completion
- recovery
- confidence

Do not change the test merely because the hardware has been added.

---

# Decision Log

Every major architectural decision must be recorded in:

docs/DECISIONS.md

Each decision should contain:

- date
- problem
- alternatives considered
- chosen solution
- rationale
- trade-offs
- tests required

---

# Change Control

Every meaningful engineering stage should produce a coherent commit.

Recommended commit format:

stage-01: freeze baseline

stage-02: formalize requirements

stage-03: define state architecture

stage-04: add IMU simulation

stage-05: add odometry

stage-06: add LiDAR localization

stage-07: add confidence system

stage-08: add degraded autonomy

stage-09: run fault campaign

stage-10: produce hardware specification

---

# Final Principle

The objective is not:

"Make the simulator pass."

The objective is:

"Build an autonomy architecture whose failure modes are understood,
whose critical states are independently observable, whose confidence
is measurable, and whose behaviour degrades safely when sensing is
lost."

The system should ultimately progress from:

simulation

→ sensor-fusion simulation

→ hardware prototype

→ controlled vehicle testing

→ outdoor testing

→ agricultural field operation

→ expansion into other heavy-machine applications.
