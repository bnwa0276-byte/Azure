# Autonomous Drone Platform — Documentation

This documentation provides a Version 2.0 engineering overview of the Autonomous Drone Platform. It describes the current subsystem boundaries, the stabilized runtime structure, testing strategy, and the current roadmap for the stabilization phase.

Top-level files:

- `architecture.md` — current system architecture and module responsibilities
- `system_overview.md` — concise description of subsystem responsibilities and public APIs
- `requirements.md` — functional and non-functional requirements
- `roadmap.md` — current stabilization roadmap
- `testing.md` — test-suite guidance and execution notes
- `engineering_metrics.md` — suggested metrics and controller-performance measurements
- `sprint_reports/` — sprint-by-sprint historical reports
- `engineering_decisions/` — Engineering Decision Logs (EDLs)

Usage
- Read `architecture.md` first for the current conceptual map.
- Use `testing.md` when running or extending the unit suite.
- Treat `debug_pd.py` and `autopilot_demo.py` as demo/debug utilities rather than core production modules.

Architecture note
- `FlightController` is defined only in `flight_controller.py`.
- `drone.py` contains the `Drone` FSM and health monitor, but it does not contain a duplicate `FlightController` implementation.
