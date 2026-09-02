import unittest

from hal.backends import GLOBAL_BACKEND_REGISTRY, MockBackend, SerialBackend, UDPBackend, Capability
from hal.transports import MockTransport
from flight_recorder.recorder import FlightRecorder


class BackendTests(unittest.TestCase):
    def test_registry_and_create(self):
        names = GLOBAL_BACKEND_REGISTRY.available()
        self.assertIn("mock", names)
        b = GLOBAL_BACKEND_REGISTRY.create("mock")
        self.assertIsInstance(b, MockBackend)

    def test_connect_disconnect_lifecycle(self):
        rec = FlightRecorder()
        mt = MockTransport()
        b = MockBackend(transport=mt, recorder=rec)
        self.assertFalse(b.is_connected())
        b.connect()
        self.assertTrue(b.is_connected())
        b.disconnect()
        self.assertFalse(b.is_connected())

    def test_capability_negotiation(self):
        b = MockBackend()
        remote = Capability(protocols={"mock", "custom"}, features={"cmd_v1", "ext"}, max_rate_hz=10.0)
        negotiated = b.negotiate(remote)
        self.assertIn("mock", negotiated.protocols)
        self.assertIn("cmd_v1", negotiated.features)
        self.assertLessEqual(negotiated.max_rate_hz, 10.0)

    def test_mock_send_command_schedules_packet(self):
        mt = MockTransport()
        b = MockBackend(transport=mt)
        b.connect()
        b.send_command({"cmd": "test", "value": 1})
        # transport outbox should have scheduled packet(s)
        self.assertTrue(len(mt._outbox) >= 1)

    def test_placeholder_backends_provide_capabilities_and_raise_on_connect(self):
        sb = SerialBackend(port="COM3")
        self.assertIn("serial", sb.capabilities().protocols)
        with self.assertRaises(NotImplementedError):
            sb.connect()
        ub = UDPBackend()
        self.assertIn("udp", ub.capabilities().protocols)
        with self.assertRaises(NotImplementedError):
            ub.connect()


if __name__ == "__main__":
    unittest.main()
