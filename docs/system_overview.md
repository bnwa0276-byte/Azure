# System Overview

Version: 2.0

This document summarizes the responsibilities and public APIs of each major subsystem in the current repository and defines the simulation-grade scope of the V2.0 baseline.

The software in this repository is intended to model and validate autonomous drone behavior in a deterministic simulation environment. It is not a certified or flight-ready real-world avionics system and must not be treated as proof of safe autonomous flight outside the simulator.

## Drone (`drone.py`)
- FSM for flight modes and vehicle-state tracking.
- Public APIs include `change_mode()`, `takeoff()`, `land()`, `apply_thrust()`, `command_velocity()`, and `step_physics()`.
- Owns the `PhysicsEngine` instance and the `HealthStatus` monitor.

## Physics (`physics.py`)
- Simplified motion integration for position, velocity, and acceleration.
- Public API includes `step(dt, environment=None, sim_time=0.0)`, `set_thrust_acceleration()`, and `set_altitude()` for test/setup use.
- Maintains sole authority for state integration and physical updates within the simulation.

## Flight Controller (`flight_controller.py`)
- This is the canonical `FlightController` implementation.
- Implements `update(dt)` for non-blocking staged sequences, emergency handling, battery policy, altitude-hold logic, and safety-aware controller sequencing.
- Public high-level commands include `climb()`, `hover()`, `descend()`, `enable_altitude_hold()`, and `apply_guidance()`.
- Contains the velocity-aware TAKEOFF → HOVER handoff logic used by the current validation baseline.

## Autopilot (`autopilot.py`)
- `PController` and `PDController` compute thrust accelerations and enforce safety clamping and rate limits.
- Intended for altitude acquisition and hold in the simulation environment.

## Navigation (`navigation.py`)
- `Mission`, `Waypoint`, and `NavigationSystem` manage waypoint ordering and mission progress.
- This is a simplified mission model for simulation and architecture validation, not a production navigation stack.

## Guidance (`guidance.py`)
- `GuidanceSystem` and `GuidanceCommand` compute desired horizontal motion toward the active waypoint and can incorporate obstacle-aware avoidance.
- Guidance remains a horizontal path-planning layer and does not own vehicle safety decisions.

## Sensors (`sensors/*`)
- Passive readers for battery, GPS, IMU, barometer, and motor health.
- These components read from the simulation state and do not modify physics.
- Simulated data is adequate for architecture and control validation, not hardware-grade sensing.

## Environment (`environment/model.py`)
- Provides steady wind, gusts, turbulence, drag, and measurement-noise models for deterministic simulation.

## Obstacles (`obstacles/*`)
- `ObstacleDetector` and `AvoidancePlanner` provide collision prediction and reactive path adjustments within the simulator.
- This logic is simulation-oriented and not equivalent to certified obstacle avoidance in a real aircraft system.

## Fusion (`fusion/estimator.py`)
- `ComplementaryEstimator` blends IMU prediction with GPS and barometer corrections.
- This is a lightweight sensor-fusion model for simulation use.

## Flight Recorder (`flight_recorder/recorder.py`)
- `FlightRecorder` records telemetry and events without mutating the simulation.
- Supports capture and replay of steps as data records.

## Analytics (`analytics/*`)
- Reads recorded entries and produces summary statistics and reports without mutating flight logs.

## Visualization (`visualization/*`)
- Read-only GCS/telemetry rendering stack for telemetry formatting, world mapping, and optional plotting.
- Supports headless and non-blocking rendering behavior in the simulator.

## HAL (`hal/*`)
- Hardware-independent interfaces and simulated backend/transport adapters.
- Provides abstraction boundaries between software logic and simulation transport details.

## HIL (`hal/hil.py` and related modules)
- Bridges the software stack to simulated/mock transport and timing flows.
- This is a simulation boundary, not equivalent to real hardware-in-the-loop validation.

## Simulation Runner (`simulation_runner.py`)
- Fixed-timestep orchestration layer with event logging, telemetry snapshots, visualizer hooks, metrics collection, and optional estimator/recorder integration.

## Architecture note
- `FlightController` is defined in exactly one source file: `flight_controller.py`.
- `drone.py` does not contain a second duplicate `FlightController` implementation.
- The repository intentionally distinguishes simulation logic from hardware certification and field deployment claims.
