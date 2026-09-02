# Architecture — Autonomous Drone Platform (Version 2.0)

## Purpose

This repository defines a simulation-grade autonomous drone platform baseline. It is intentionally designed as a deterministic software model for control logic, navigation, environment disturbances, sensor health, analytics, and HIL-style transport boundaries.

This is not a flight-certified or hardware-validated drone system. It is a simulator and engineering baseline for architecture, control, and mission-flow validation in software.

## Goals
- Maintain clear ownership boundaries between vehicle, physics, control, sensing, environment, recording, and visualization.
- Preserve deterministic simulation behavior and the existing unittest baseline.
- Explicitly separate the simulation architecture from real-world aircraft certification and hardware validation.

## Canonical ownership

The current implementation is aligned with the following ownership model:

- `Drone` owns the finite-state machine, vehicle health state, and command API.
- `FlightController` is the canonical control-decision owner and is implemented only in `flight_controller.py`.
- `PhysicsEngine` owns motion integration and physical state updates.
- `NavigationSystem` owns mission and waypoint progression.
- `GuidanceSystem` owns horizontal guidance and target-seeking logic.
- `PController` / `PDController` own altitude-control calculations and rate limiting.
- Sensors are read-only consumers of simulation state.
- `ComplementaryEstimator` performs state fusion from IMU, GPS, and barometer observations.
- `FlightRecorder` and analytics operate outside the vehicle mutation path.
- `HAL` and `HIL` remain interface boundaries rather than the authoritative implementation of flight physics or control policy.

## High-level components
- `Drone`: central vehicle object, flight-mode FSM, health monitor integration, and public command interfaces for thrust, velocity, takeoff, and landing.
- `PhysicsEngine`: sole owner of motion integration for position, velocity, and acceleration. It receives thrust commands and environment disturbances and updates vehicle state.
- `FlightController`: single canonical controller responsible for control decisions, emergency policy, altitude-hold logic, mission supervision, and staged takeoff/landing execution.
- `PController` / `PDController`: autopilot logic in `autopilot.py`, computing thrust commands and enforcing output bounds/rate limiting.
- `Mission` / `NavigationSystem` / `Waypoint`: mission ordering and waypoint progression logic.
- `GuidanceSystem`: computes desired motion toward the active navigation target, including obstacle-aware adjustments when configured.
- `Sensors`: passive readers in `sensors/` for battery, GPS, IMU, barometer, and motors; they sample simulation state and do not modify physics.
- `Environment`: models steady wind, gusts, turbulence, drag, and sensor-noise behavior.
- `ObstacleDetector` / `AvoidancePlanner`: obstacle detection and reactive avoidance planning within the simulation.
- `ComplementaryEstimator`: simple state estimator in `fusion/` that fuses IMU, GPS, and barometer information.
- `FlightRecorder`: records telemetry and events without mutating simulation state.
- `Analytics`: reads recorded data and computes summary information without mutating flight logs.
- `Visualization` / GCS: read-only rendering consumer for telemetry and world state.
- `HAL`: hardware-independent interfaces and simulated adapters.
- `HIL`: transport and timing layer between the software stack and simulated/mock hardware.
- `SimulationRunner`: orchestrates the fixed-timestep loop, controller updates, physics stepping, event logging, and optional telemetry consumers.

## Data and control flow
1. `SimulationRunner.step()` calls `FlightController.update(dt)`.
2. `FlightController` makes control decisions and issues commands to `Drone` via the public command API (`apply_thrust()`, `command_velocity()`, etc.).
3. `Drone.step_physics(dt, environment, sim_time)` delegates to `PhysicsEngine.step(...)`.
4. `PhysicsEngine` integrates position, velocity, and acceleration while incorporating environment disturbances.
5. Sensors sample from the simulated vehicle/environment and update their readings without mutating physics.
6. `ComplementaryEstimator`, `FlightRecorder`, and the visualization stack consume the resulting state and telemetry without directly changing the vehicle state.

## Design principles
- Single responsibility: the simulation modules reflect distinct concerns (physics, control, sensing, navigation, recording, visualization).
- Read-only sensors: sensor modules observe state rather than mutating simulation state.
- Deterministic simulation: the environment package supports seeded behavior for reproducible tests.
- Non-blocking control: staged takeoff and landing logic is implemented as request-driven state executed by `FlightController.update(dt)`.
- Canonical controller ownership: there is exactly one `FlightController` class, defined in `flight_controller.py`.
- Simulation boundary: the software is intentionally not presented as a certified real-world avionics implementation.

## Known limitations of the architecture
- `SerialTransport` and `UDPTransport` are placeholder interfaces and are not real hardware transports.
- HIL here is a boundary abstraction for simulation and mock communication, not real hardware-in-the-loop validation.
- The simulator does not model all real world flight certification requirements, hardware failure modes, or airframe dynamics.
- Control tuning is simulation-oriented and must be revalidated for any hardware deployment scenario.
