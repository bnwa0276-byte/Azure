# Testing Strategy

The project follows a unit-test-first approach. The repository includes `unittest`-based tests designed to verify both module boundaries and end-to-end simulation behavior.

Key testing principles
- Keep modules decoupled and test their public APIs.
- Use deterministic seeds for environment when comparing runs.
- Prefer short, focused tests that validate a single behavior (e.g., altitude acquisition, takeoff handoff, physics integration).

Test categories
- Unit tests: controllers (`test_autopilot.py`, `test_pd_controller.py`), physics (`test_physics.py`), sensors, navigation.
- Integration tests: takeoff/landing sequences (`test_takeoff_landing.py`), SimulationRunner behaviors (`test_simulation_runner.py`).
- Environment tests: disturbance models and sensor noise (`test_environment.py`).
- Visualization tests: coordinate mapping and telemetry formatting (`test_visualization*.py`).

Running the test suite
```bash
python -m unittest
```

CI considerations
- The visualization drawing uses a headless fallback when `matplotlib` is not installed so tests run in minimal CI environments.
