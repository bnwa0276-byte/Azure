# Requirements

## Functional Requirements
1. Simulate vertical and horizontal motion of a drone and provide an API for commanding thrust and flight sequences (takeoff, landing, hover, mission).
2. Provide altitude-hold via autopilot controllers (`PController`, `PDController`) with configurable gains and safety clamping.
3. Allow mission definition with waypoints and mission progression tracking.
4. Support a deterministic, fixed-timestep simulation runner with event logging and telemetry.
5. Offer a modular, read-only visualization (GCS) for 2D rendering of world and telemetry.
6. Model environmental disturbances (steady wind, gusts, turbulence) and configurable sensor noise.

## Non-functional Requirements
- Deterministic behavior when a fixed seed is supplied to environment RNG.
- Clear separation-of-concerns between physics, control, sensing, and visualization.
- High unit-test coverage for core modules and simulations.
- Lightweight dependency footprint (matplotlib optional for visualization; headless tests must run without it).