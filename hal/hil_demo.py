"""Simple HIL simulation demo using MockTransport, MockBackend, Scheduler, and HILBridge."""
from __future__ import annotations

import os
import sys
# ensure project root is on sys.path so package imports work when running script directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hal.transports import MockTransport
from hal.backends import MockBackend
from hal.hil import HILBridge
from hal.scheduler import Scheduler
from hal.simulated import SimulatedVehicle
from drone import Drone
from flight_recorder.recorder import FlightRecorder


def run_demo(duration: float = 2.0, dt: float = 0.01) -> None:
    # deterministic setup
    transport = MockTransport(latency=0.05, jitter=0.02, loss=0.1, seed=42)
    recorder = FlightRecorder()
    drone = Drone()
    vehicle = SimulatedVehicle(drone)
    backend = MockBackend(transport=transport, recorder=recorder, name="mock_backend")
    hil = HILBridge(vehicle=vehicle, transport=transport, recorder=recorder)

    # simple logger for backend receives
    backend.register_receive_callback(lambda pkt: recorder.record_event(0.0, "backend_cb", {"pkt": pkt}))

    backend.connect()

    sched = Scheduler()

    # telemetry at 10 Hz
    sched.schedule(lambda now: hil.send_telemetry(now), rate_hz=10.0, start_time=0.0)

    # backend sends velocity command at 2 Hz
    def backend_cmd(now: float):
        backend.send_command({"cmd": "velocity", "vx": 1.0, "vy": 0.0})

    sched.schedule(backend_cmd, rate_hz=2.0, start_time=0.0)

    sim_time = 0.0
    while sim_time <= duration:
        sched.tick(sim_time)
        # process transport deliveries scheduled up to sim_time
        transport.advance_time(sim_time)
        sim_time += dt

    # flush any pending events into a final step so they appear in entries
    recorder.record_step(sim_time, {})
    # after simulation, print recorded events summary
    print("--- Recorder Events ---")
    for e in recorder.entries():
        print(f"t={e.sim_time}: events={e.events}")


if __name__ == "__main__":
    run_demo()
