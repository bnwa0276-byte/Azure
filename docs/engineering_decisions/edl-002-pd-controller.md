# EDL-002: Add PD Altitude Controller

Decision: Extend the existing proportional altitude controller with a derivative term (`PDController`) to reduce overshoot and improve settling after disturbances.

Rationale:
- A pure P controller produced measurable overshoot and slower settling in the simplified vertical dynamics. Adding a derivative term improves damping.
- Keep the same external interface so the `FlightController` integration is minimally impacted; pass timestep `dt` when available.

Implications:
- `autopilot.py` now contains `PController` and `PDController`; both support clamping and a `max_delta_thrust` rate limiter.
- Tests added comparing P vs PD under disturbances and measuring overshoot/settling improvements.
