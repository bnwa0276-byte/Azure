from drone import Drone, FlightMode
from flight_controller import FlightController

def run_demo():
    drone = Drone()
    drone.change_mode(FlightMode.INITIALIZING)
    drone.change_mode(FlightMode.IDLE)
    drone.change_mode(FlightMode.ARMED)
    drone.physics.set_altitude(0.0)
    drone.step_physics(0.0)

    controller = FlightController(drone)
    controller.enable_altitude_hold(target_altitude=20.0, Kp=1.0)

    for i in range(20):
        controller.update_altitude_hold(0.1)
        thrust = controller.autopilot.get_last_output()
        drone.step_physics(0.1)
        print(f"t={i*0.1:.1f}s alt={drone.altitude:.3f} thrust={thrust:.3f}")

if __name__ == '__main__':
    run_demo()
