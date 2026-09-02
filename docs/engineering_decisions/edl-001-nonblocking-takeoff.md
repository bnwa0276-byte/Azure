# EDL-001: Non-blocking Takeoff / Landing

Decision: Convert staged takeoff/landing procedures to a non-blocking, request-driven model executed incrementally by `FlightController.update(dt)`.

Rationale:
- Blocking procedures made unit testing and integration difficult; a non-blocking design allows callers to request maneuvers and continue interacting with the simulation.
- Keeps `FlightController` responsible for sequencing while allowing the simulation loop (`SimulationRunner`) to schedule physics steps.

Implications:
- `Drone.takeoff()` and `Drone.land()` now create request objects. `FlightController.update(dt)` uses these to execute staged sequences.
- Tests were rewritten to step the controller and physics iteratively until sequences complete.
