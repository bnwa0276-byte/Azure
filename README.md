# Autonomous Drone Platform

## V2.0 release baseline

This repository is a simulation-grade autonomous drone platform implemented in Python. It is intended to model and validate a drone control stack, mission lifecycle, guidance, environment disturbances, sensor health, analytics, and HIL-style simulation boundaries within a deterministic software environment.

This is not a certified avionics system, not a flight-ready real-world drone controller, and it is not a substitute for hardware validation or operational safety approvals. Successful simulation outcomes are evidence of software behavior in the simulated environment only.

## Purpose

The V2.0 baseline is designed to provide:
- a deterministic vehicle FSM and control loop
- a single canonical `FlightController` decision path
- altitude control using `PController` and `PDController`
- a velocity-aware TAKEOFF → HOVER handoff
- mission and waypoint progression via `NavigationSystem`
- horizontal guidance via `GuidanceSystem`
- environment and disturbance modeling
- obstacle-aware simulation logic
- sensor sampling and health reporting
- complementary estimation and telemetry capture
- analytics/reporting and visualization/GCS-style output
- HAL and HIL simulation boundaries that are intentionally separated from real hardware implementation

## Current capabilities

Core runtime modules:
- `drone.py`: `Drone` FSM, health monitor, vehicle commands, public control APIs, and simulation state management.
- `physics.py`: `PhysicsEngine`, the sole owner of motion integration for position, velocity, and acceleration.
- `flight_controller.py`: the canonical `FlightController` responsible for safety logic, altitude hold, takeoff/landing sequencing, and mission supervision.
- `navigation.py`: `Mission`, `Waypoint`, and `NavigationSystem` for waypoint progression and mission state.
- `guidance.py`: `GuidanceSystem` and `GuidanceCommand` for horizontal guidance toward the active target.
- `autopilot.py`: `PController` and `PDController` for altitude control, clamping, and rate-limited thrust output.
- `simulation_runner.py`: fixed-timestep coordination of controller update, physics step, telemetry collection, and event recording.

Supporting systems:
- `sensors/`: passive sensor readers for GPS, IMU, barometer, battery, and motors.
- `environment/`: steady wind, gusts, turbulence, drag, and noisy measurement behavior.
- `obstacles/`: obstacle representation and avoidance planning logic for the simulator.
- `fusion/`: complementary state estimation using IMU, GPS, and barometer data.
- `flight_recorder/`: event and telemetry recording with replay support.
- `analytics/`: summary and reporting consumers of recorded data without mutating the simulation state.
- `visualization/`: read-only telemetry/world rendering and GCS-style presentation utilities.
- `hal/`: interface boundaries and simulated adapters for vehicle, sensor, and navigation access.
- HIL boundary: a simulated transport/bridge layer that separates transport behavior from the core simulator but does not constitute real hardware-in-the-loop validation.

## Verification baseline

The repository is verified with the standard unittest discovery suite:

```bash
python -m unittest discover -v
```

Current automated baseline:
- 83 tests discovered
- 83 passed
- 0 failed
- 0 errors

This includes architecture-enforcement tests, controller tests, physics tests, navigation and guidance tests, environment tests, flight-recording tests, HIL/simulation tests, and full integration coverage.

## Safety boundary and release statement

This project demonstrates a simulation-grade autonomous drone platform. It is not evidence of safe real-world autonomous flight.

Simulation test success must not be interpreted as proof that the software is safe for autonomous operation outside the simulator. Real-world deployment requires hardware validation, sensor calibration, robust safety architecture, certification review, and operational testing beyond the present repository scope.

## Known limitations

The repository makes the following limitations explicit:
- `SerialTransport` is currently a placeholder and not a real serial implementation.
- `UDPTransport` is currently a placeholder and not a real network transport implementation.
- There has been no real drone hardware validation in this repository.
- There is no flight certification or airborne safety certification for the platform.
- Simulated sensors are intentionally simplified and are not hardware-grade measurement systems.
- Navigation and geometric models are simplified approximations for simulation use.
- Controller tuning is simulation-oriented rather than validated for safe real-world flight behavior.
- Obstacle avoidance is simulation-oriented and not a substitute for real sensing and avoidance certification.
- HIL here is a software simulation boundary only and is not equivalent to validated hardware-in-the-loop testing.

## Architecture summary

The implementation matches the current V2.0 structural intent:
- `Drone` owns vehicle FSM, health monitoring, and public command responsibilities.
- `FlightController` is the canonical control-decision owner.
- `PhysicsEngine` owns motion integration and state updates.
- `NavigationSystem` owns mission and waypoint progress tracking.
- `GuidanceSystem` owns horizontal guidance.
- `PController` and `PDController` own altitude-control calculation and rate limiting.
- Sensor components are read-only observers of vehicle state rather than mutators of simulation state.
- `ComplementaryEstimator` fuses telemetry from IMU, GPS, and barometer.
- `FlightRecorder` and analytics consume recorded data without mutating the system state.
- `HAL` and `HIL` remain interface boundaries rather than direct dependencies on simulator internals.

## Running the project

From the repository root:

```bash
python main.py
```

Additional examples and focused control demonstrations are also included in the repository.

## Repository status

This repository is a V2.0 simulation baseline for autonomous drone platform development and validation. It is not presented as a certified or flight-ready production avionics stack.
