from drone import Drone, FlightMode
from flight_controller import FlightController
from autopilot import PController, PDController

def run(ctrl):
    dt = 0.05
    for i in range(40):
        ctrl.update_altitude_hold(dt)
        ctrl.drone.step_physics(dt)
        lo = ctrl.autopilot.get_last_output() if ctrl.autopilot is not None else None
        print(f"t={i*dt:.2f} alt={ctrl.drone.altitude:.3f} thrust={lo if lo is not None else 'None'}")

print('P controller:')
D = Drone()
D.change_mode(FlightMode.INITIALIZING)
D.change_mode(FlightMode.IDLE)
D.change_mode(FlightMode.ARMED)
D.physics.set_altitude(0.0)
D.step_physics(0.0)
C = FlightController(D)
C.autopilot = PController(Kp=20.0, target_altitude=10.0, max_delta_thrust=100.0, max_thrust=200.0)
C.altitude_hold_enabled = True
run(C)

print('\nPD controller:')
D2 = Drone()
D2.change_mode(FlightMode.INITIALIZING)
D2.change_mode(FlightMode.IDLE)
D2.change_mode(FlightMode.ARMED)
D2.physics.set_altitude(0.0)
D2.step_physics(0.0)
C2 = FlightController(D2)
C2.autopilot = PDController(Kp=20.0, Kd=3.0, target_altitude=10.0, max_delta_thrust=100.0, max_thrust=200.0)
C2.altitude_hold_enabled = True
run(C2)
