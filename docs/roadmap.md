# Roadmap

## Current release baseline: Version 2.0

The repository is currently positioned as a simulation-grade autonomous drone platform and is not presented as a certified or flight-ready avionics implementation. The current V2.0 baseline is intended to stabilize architecture, control behavior, mission flow, and testability within a deterministic software environment.

## Current V2.0 scope
- Maintain a single canonical `FlightController` in `flight_controller.py`
- Preserve the separation between vehicle FSM, physics integration, control logic, navigation, guidance, sensors, and analytics
- Keep the simulation deterministic and testable under the existing unittest suite
- Maintain explicit documentation of known limitations and safety boundaries

## Stabilization and maintenance work
- Continue aligning documentation with the actual repository implementation and release status
- Preserve the current architecture boundary between simulation behavior and hardware validation
- Maintain the existing unittest baseline and architecture-enforcement checks without weakening them
- Keep all public documentation honest about what the repository does and does not validate

## Future directions (non-release claims)
- Refinement of simulation fidelity and environmental behavior
- Improved mission and obstacle modeling within the simulator
- Continued HIL boundary work for mock transport and software-bridge layers
- Additional engineering documentation for maintainers and release reviewers

## Explicit constraint

The roadmap does not claim that the current repository has been hardware-validated, certified, or proven safe for autonomous real-world flight. Any future extension beyond the V2.0 baseline must be treated as a separate engineering and safety validation effort.
