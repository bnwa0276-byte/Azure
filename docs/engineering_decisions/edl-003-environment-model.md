# EDL-003: Environment Disturbance Model

Decision: Introduce a lightweight environment model to provide steady wind, gusts, turbulence, and sensor noise; make it seedable for deterministic tests.

Rationale:
- Environmental effects are necessary to validate controller robustness.
- A simple, deterministic RNG-based model is sufficient for unit tests and early-stage simulation.

Implications:
- `environment/model.py` supplies `get_external_acceleration()` and sensor noise hooks used by `PhysicsEngine` and sensors.
- `PhysicsEngine.step(dt, environment, sim_time)` now integrates external forces; `Drone.step_physics(...)` forwards the environment.
