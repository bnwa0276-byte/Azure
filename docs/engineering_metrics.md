# Engineering Metrics

Suggested metrics to track engineering progress and platform quality:

- Test Coverage: percentage of lines / critical paths covered by unit tests.
- Determinism Score: number of end-to-end runs that are bit-identical when seeded.
- Mean Time To Fix (MTTF): average time to resolve a failing test introduced in a change.
- Simulation Fidelity: measured by integration tests that compare expected altitude profiles under given thrust profiles.
- Performance: simulation step throughput (steps / second) for the `SimulationRunner` at default timestep.

Recording metrics
- Store CI artifacts (test results, coverage reports) and compare across sprints.
