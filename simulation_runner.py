from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

from drone import Drone, FlightMode
from flight_controller import FlightController
try:
    from visualization.renderer import VisualizerProtocol
except Exception:
    VisualizerProtocol = None
from metrics.controller_metrics import ControllerMetrics
from flight_recorder import FlightRecorder
from fusion import ComplementaryEstimator


@dataclass
class SimulationRunner:
    drone: Drone
    controller: FlightController
    navigation: Optional[object] = None
    dt: float = 1.0 / 50.0
    sim_time: float = 0.0
    running: bool = False
    event_log: List[Dict[str, Any]] = field(default_factory=list)
    step_count: int = 0
    visualizer: Optional[object] = None
    metrics: Optional[ControllerMetrics] = None
    recorder: Optional[FlightRecorder] = None
    estimator: Optional[ComplementaryEstimator] = None

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def _record_event(self, name: str, info: Optional[Dict[str, Any]] = None) -> None:
        self.event_log.append({"time": self.sim_time, "event": name, "info": info or {}})

    def step(self) -> None:
        # one simulation step: controller update then physics step then sensors
        # record mode changes and request transitions
        prev_mode = getattr(self, "_last_mode", None)
        # update controller
        self.controller.update(self.dt)

        # advance physics
        self.drone.step_physics(self.dt)

        # optional navigation can be stepped here if desired
        self.sim_time += self.dt
        self.step_count += 1

        # mode change event
        if prev_mode != self.drone.mode:
            self._record_event(f"MODE:{self.drone.mode.value}")
        self._last_mode = self.drone.mode

        # detect takeoff/landing request lifecycle
        tr = getattr(self.drone, "_takeoff_request", None)
        if tr is not None:
            state = tr.get("state")
            prev = tr.get("_runner_state")
            if state == "requested" and prev is None:
                self._record_event("TAKEOFF_REQUESTED", {"target": tr.get("target_altitude")})
                tr["_runner_state"] = "started"
            if state == "completed" and tr.get("_runner_state") != "completed":
                self._record_event("TAKEOFF_COMPLETED", {"altitude": self.drone.altitude})
                tr["_runner_state"] = "completed"

        lr = getattr(self.drone, "_landing_request", None)
        if lr is not None:
            state = lr.get("state")
            prev = lr.get("_runner_state")
            if state == "requested" and prev is None:
                self._record_event("LANDING_REQUESTED", {})
                lr["_runner_state"] = "started"
            if state == "completed" and lr.get("_runner_state") != "completed":
                self._record_event("LANDING_COMPLETED", {"altitude": self.drone.altitude})
                lr["_runner_state"] = "completed"

        # update visualizer (read-only telemetry + positions)
        if self.visualizer is not None:
            telemetry = self.telemetry()
            position = tuple(self.drone.physics.position)
            waypoints = []
            if self.navigation is not None and hasattr(self.navigation, "mission"):
                try:
                    waypoints = list(getattr(self.navigation.mission, "waypoints", []))
                except Exception:
                    waypoints = []
            path = list(getattr(self, "_path_history", []))
            path.append(position)
            self._path_history = path
            try:
                self.visualizer.update(telemetry=telemetry, position=position, waypoints=waypoints, path=path)
            except Exception:
                pass
        # update estimator (if present) using latest sensor readings
        if self.estimator is not None:
            try:
                # collect sensor measurements from health monitor
                hm = self.drone.health_monitor
                gps_pos = None
                if hasattr(hm.gps, "last_position"):
                    gps_pos = tuple(getattr(hm.gps, "last_position"))
                baro_alt = None
                if hasattr(hm.barometer, "altitude"):
                    baro_alt = float(getattr(hm.barometer, "altitude"))
                imu_acc = None
                if hasattr(hm.imu, "last_accel"):
                    imu_acc = tuple(getattr(hm.imu, "last_accel"))
                gps_avail = hm.gps.is_healthy()
                self.estimator.update(self.dt, gps_pos=gps_pos, baro_alt=baro_alt, imu_accel=imu_acc, gps_available=gps_avail)
                # attach estimated state to telemetry for visualizer and recorder
                est = self.estimator.get_state()
                telemetry["estimated"] = {"position": est.position, "velocity": est.velocity, "confidence": est.confidence}
            except Exception:
                pass
        # record telemetry and events to flight recorder if present
        if self.recorder is not None:
            try:
                telemetry = self.telemetry()
                # snapshot controller output if available
                controller_snapshot = None
                if self.controller.autopilot is not None:
                    controller_snapshot = {"target_altitude": self.controller.autopilot.target_altitude}
                # snapshot environment if runner has one
                env_snapshot = None
                if hasattr(self, "environment") and getattr(self, "environment") is not None:
                    try:
                        env = getattr(self, "environment")
                        env_snapshot = {"steady_wind": getattr(env, "steady_wind", None), "turbulence_strength": getattr(env, "turbulence_strength", None)}
                    except Exception:
                        env_snapshot = None
                self.recorder.record_step(self.sim_time, telemetry, controller=controller_snapshot, environment=env_snapshot)
            except Exception:
                pass
        # record controller metrics if collector provided
        if self.metrics is not None:
            try:
                target = None
                if self.controller.autopilot is not None:
                    target = self.controller.autopilot.target_altitude
                self.metrics.sample(self.sim_time, self.drone.altitude, target)
            except Exception:
                pass

    def run(self, duration: float) -> None:
        steps = int(max(1, round(duration / self.dt)))
        self.start()
        for _ in range(steps):
            if not self.running:
                break
            self.step()
        self.stop()

    def telemetry(self) -> Dict[str, Any]:
        target = None
        if self.controller.autopilot is not None:
            target = self.controller.autopilot.target_altitude
        return {
            "sim_time": self.sim_time,
            "mode": self.drone.mode.value,
            "altitude": self.drone.altitude,
            "vz": self.drone.physics.velocity[2],
            "battery": self.drone.battery,
            "target_altitude": target,
        }
