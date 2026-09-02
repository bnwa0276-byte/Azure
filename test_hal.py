import unittest

from hal.interfaces import VehicleInterface, GPSInterface
from hal.simulated import SimulatedVehicle, SimulatedSensors
from hal.factory import create_vehicle, create_sensors
from drone import Drone


class HALTests(unittest.TestCase):
    def test_simulated_vehicle_implements_interface(self):
        d = Drone()
        sv = SimulatedVehicle(d)
        self.assertIsInstance(sv, VehicleInterface)
        sv.apply_thrust(9.81)
        sv.command_velocity(1.0, 0.0)
        pos = sv.get_position()
        self.assertIsInstance(pos, tuple)

    def test_simulated_sensors_wrap_health(self):
        d = Drone()
        sensors = SimulatedSensors(d.health_monitor)
        gpspos = sensors.last_position()
        self.assertTrue(isinstance(gpspos, (tuple, type(None))))

    def test_factory_creates_simulated(self):
        sv = create_vehicle()
        self.assertIsInstance(sv, SimulatedVehicle)


if __name__ == "__main__":
    unittest.main()
