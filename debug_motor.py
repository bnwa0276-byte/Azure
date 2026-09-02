from drone import Drone, FlightMode
from flight_controller import FlightController

drone = Drone()
drone.change_mode(FlightMode.INITIALIZING)
drone.change_mode(FlightMode.IDLE)
drone.change_mode(FlightMode.ARMED)
drone.physics.set_altitude(0.0)
drone.step_physics(0.0)
controller = FlightController(drone)
controller.enable_altitude_hold(target_altitude=10.0, Kp=2.0)
for i in range(60):
    controller.update_altitude_hold(0.1)
    drone.step_physics(0.1)
print('before drop', drone.altitude, drone.physics.actual_thrust_accel, drone.physics.target_thrust_accel, drone.physics.velocity)
drone.physics.set_altitude(2.0)
drone.step_physics(0.0)
post_drop_error = abs(drone.altitude - 10.0)
print('after set altitude', drone.altitude, post_drop_error, drone.physics.actual_thrust_accel, drone.physics.target_thrust_accel)
for i in range(100):
    controller.update_altitude_hold(0.1)
    drone.step_physics(0.1)
    if i in (0,1,2,9,19,49,99):
        print(i, 'alt=', drone.altitude, 'vel=', drone.vertical_velocity, 'actual=', drone.physics.actual_thrust_accel, 'target=', drone.physics.target_thrust_accel)
print('final', drone.altitude, abs(drone.altitude-10.0))
