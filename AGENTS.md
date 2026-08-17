# AGENTS.md

## Project

This repository contains the autonomy software and simulation environment
for an agricultural robotics platform, with future applications to
excavation and other heavy mobile machinery.

The project is being developed from first principles around:
- multi-sensor state estimation
- fault tolerance
- graceful degradation
- autonomous navigation
- machine/tool state estimation
- environmental perception
- safety-oriented autonomy

## Core Engineering Principle

Do not optimize the system merely to make robustness tests pass.

Robustness tests are deliberately designed to expose weaknesses.

A failure discovered by testing must be:
1. reproduced,
2. understood,
3. recorded,
4. classified,
5. mapped to a mitigation,
6. retested.

Do not weaken a test, remove a failure condition, or introduce
artificial assumptions solely to improve the reported score.

## Current Development Philosophy

The current software limitations are being used to identify hardware
requirements.

Software failure -> engineering requirement -> hardware mitigation ->
sensor integration -> repeated validation.

The existing S1-S10 shortfall register must be preserved.

## Current Shortfall Register

S1. GPS dropout causes localization degradation.

S2. Dead-reckoning accumulates position error during GPS loss.

S3. GPS recovery does not necessarily guarantee trajectory recovery.

S4. Waypoint transitions can become ambiguous under localization
uncertainty.

S5. Navigation can remain committed to a stale trajectory segment.

S6. Sensor uncertainty can propagate into mission-level failure.

S7. Long GPS outages substantially reduce robustness.

S8. Combined sensor noise and dropout compound failure probability.

S9. Heading uncertainty can affect navigation robustness.

S10. The current system lacks independent verification that its
estimated position/state is physically correct.

These shortfalls are engineering requirements. Do not delete them because
a software change makes the current test score better.

## Current Robustness Baseline

The current development results include:

- Nominal: 25/25
- ±0.25 m GPS noise, 5 s dropout, ±5° heading: 17/25
- ±0.50 m GPS noise, 5 s dropout, ±5° heading: 15/25
- ±0.25 m GPS noise, 5 s dropout, ±10° heading: 20/25
- ±0.50 m GPS noise, 5 s dropout, ±10° heading: 14/25
- ±0.50 m GPS noise, 10 s dropout, ±10° heading: 9/25

These are simulation-development results, not field-performance claims.

## Trial 04 Finding

Trial 04 currently exposes an important degraded-navigation failure.

The test includes:
- ±0.25 m GPS noise
- 5 second GPS dropout
- ±5° heading noise

The GPS dropout occurs from approximately 6 s to 11 s.

The system can survive the immediate dropout with bounded localization
error, but a later mission-state/navigation failure can produce
approximately 10 m of path error.

This failure must remain available for regression testing.

## Hardware-First Mitigation

The project intends to address the major robustness limitations through
hardware and sensor diversity rather than simply tuning software around
the failure.

Potential sensing architecture includes:

- RTK GNSS
- industrial IMU/INS
- track/wheel odometry
- LiDAR
- cameras
- machine/actuator sensing
- CAN/CAN-FD
- Ethernet
- rugged compute

Hardware choices must be justified by required observability and
independence of failure modes.

## First-Principles State Model

The autonomy system must estimate:

### Vehicle state
- x
- y
- z
- velocity
- roll
- pitch
- yaw
- angular rates

### Tool state
- boom configuration
- arm configuration
- bucket configuration
- bucket pose
- cutting-edge pose
- actuator state

### Environment state
- terrain
- excavation boundaries
- static obstacles
- dynamic obstacles

### Confidence state
- state uncertainty
- sensor health
- sensor agreement
- localization confidence

## Autonomy Modes

The intended architecture includes:

A0 — Manual

A1 — Assisted

A2 — Supervised autonomy

A3 — Degraded autonomy

A4 — Safe state

Loss of state confidence should cause an appropriate reduction in
autonomy, speed, operating envelope, or ultimately a controlled stop.

## Agent Workflow

For every engineering task:

1. Inspect the existing implementation.
2. Understand the current behavior.
3. State the hypothesis before making major changes.
4. Make the smallest appropriate change.
5. Run the relevant tests.
6. Compare against the previous baseline.
7. Record both improvements and regressions.
8. Do not delete useful failure evidence.
9. Update documentation when architecture changes.
10. Commit coherent changes with descriptive messages.

## Testing Rules

Before declaring a robustness improvement:

- reproduce the previous failure where possible;
- run the same test conditions;
- compare quantitative results;
- verify that nominal behavior has not regressed;
- verify that other robustness scenarios have not regressed.

A single successful trial is not sufficient to establish robustness.

## Do Not

Do not:
- fabricate test results;
- claim hardware capability that has not been implemented;
- claim field performance from simulation;
- silently change test criteria;
- remove difficult scenarios;
- delete failure logs;
- add arbitrary sensor assumptions;
- make architecture changes without documenting them.

## Current Ten-Stage Roadmap

1. Freeze and reproduce the software baseline.
2. Formalize S1-S10 into requirements.
3. Build the formal state-estimation architecture.
4. Add IMU/INS simulation.
5. Add track/wheel odometry.
6. Add LiDAR localization.
7. Add state confidence and sensor-health estimation.
8. Implement degraded-autonomy modes.
9. Run systematic fault-injection testing.
10. Produce the preliminary hardware specification.

The agent should work sequentially through these stages and stop for
human review when a major architectural or safety assumption changes.
