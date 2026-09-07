"""Unit and regression tests for Gazebo visualization integration."""

from __future__ import annotations

import ast
import json
import math
from pathlib import Path
import socket
from unittest.mock import MagicMock
import unittest

from gazebo import (
    GazeboBridge,
    GazeboBridgeConfig,
    GazeboPosePacket,
    compute_yaw_quaternion,
    create_gazebo_packet,
    deserialize_packet,
    serialize_packet,
)
from drone import Drone
from flight_controller import FlightController
from simulation_runner import SimulationRunner


class GazeboConfigTests(unittest.TestCase):
    def test_default_configuration(self) -> None:
        """Requirement 1: Default configuration values and validation."""
        cfg = GazeboBridgeConfig()
        self.assertEqual(cfg.host, "127.0.0.1")
        self.assertEqual(cfg.port, 9871)
        self.assertTrue(cfg.enabled)
        self.assertEqual(cfg.rate_limit_hz, 50.0)
        self.assertTrue(cfg.derive_yaw_from_velocity)

        # Validation errors
        with self.assertRaises(ValueError):
            GazeboBridgeConfig(host="")
        with self.assertRaises(ValueError):
            GazeboBridgeConfig(port=0)
        with self.assertRaises(ValueError):
            GazeboBridgeConfig(port=70000)
        with self.assertRaises(ValueError):
            GazeboBridgeConfig(rate_limit_hz=0.0)
        with self.assertRaises(ValueError):
            GazeboBridgeConfig(rate_limit_hz=-5.0)

    def test_disabled_bridge(self) -> None:
        """Requirement 2: Disabled bridge does not transmit or allocate sockets."""
        cfg = GazeboBridgeConfig(enabled=False)
        bridge = GazeboBridge(cfg)

        self.assertFalse(bridge.is_connected())
        self.assertFalse(bridge.connect())
        self.assertIsNone(bridge._socket)

        # Calling update() should safely return without creating socket
        bridge.update(
            telemetry={"sim_time": 0.0, "mode": "IDLE"},
            position=(0.0, 0.0, 0.0),
        )
        self.assertIsNone(bridge._socket)
        self.assertEqual(bridge._packets_sent, 0)


class GazeboCoordinateAndPacketTests(unittest.TestCase):
    def test_enu_coordinate_identity(self) -> None:
        """Requirement 3: Gazebo (X, Y, Z) directly matches simulator (X, Y, Z)."""
        packet = create_gazebo_packet(
            sim_time=1.5,
            position=(10.5, -20.25, 30.75),
            velocity=(1.0, 2.0, 3.0),
            flight_mode="HOVER",
        )
        self.assertEqual(packet.position, (10.5, -20.25, 30.75))

        d = packet.to_dict()
        self.assertEqual(d["position"], [10.5, -20.25, 30.75])
        self.assertEqual(d["x"], 10.5)
        self.assertEqual(d["y"], -20.25)
        self.assertEqual(d["z"], 30.75)

    def test_yaw_quaternion_for_east_velocity(self) -> None:
        """Requirement 4: East (+X) velocity produces zero yaw quaternion (1, 0, 0, 0)."""
        q = compute_yaw_quaternion(vx=5.0, vy=0.0)
        self.assertAlmostEqual(q[0], 1.0, places=6)
        self.assertAlmostEqual(q[1], 0.0, places=6)
        self.assertAlmostEqual(q[2], 0.0, places=6)
        self.assertAlmostEqual(q[3], 0.0, places=6)

    def test_yaw_quaternion_for_north_velocity(self) -> None:
        """Requirement 5: North (+Y) velocity produces +pi/2 yaw quaternion."""
        q = compute_yaw_quaternion(vx=0.0, vy=5.0)
        expected_w = math.cos(math.pi / 4.0)
        expected_z = math.sin(math.pi / 4.0)
        self.assertAlmostEqual(q[0], expected_w, places=6)
        self.assertAlmostEqual(q[1], 0.0, places=6)
        self.assertAlmostEqual(q[2], 0.0, places=6)
        self.assertAlmostEqual(q[3], expected_z, places=6)

    def test_yaw_quaternion_for_west_velocity(self) -> None:
        """Requirement 6: West (-X) velocity produces pi yaw quaternion."""
        q = compute_yaw_quaternion(vx=-5.0, vy=0.0)
        self.assertAlmostEqual(q[0], 0.0, places=6)
        self.assertAlmostEqual(q[1], 0.0, places=6)
        self.assertAlmostEqual(q[2], 0.0, places=6)
        self.assertAlmostEqual(q[3], 1.0, places=6)

    def test_yaw_quaternion_for_south_velocity(self) -> None:
        """Requirement 7: South (-Y) velocity produces -pi/2 yaw quaternion."""
        q = compute_yaw_quaternion(vx=0.0, vy=-5.0)
        expected_w = math.cos(-math.pi / 4.0)
        expected_z = math.sin(-math.pi / 4.0)
        self.assertAlmostEqual(q[0], expected_w, places=6)
        self.assertAlmostEqual(q[1], 0.0, places=6)
        self.assertAlmostEqual(q[2], 0.0, places=6)
        self.assertAlmostEqual(q[3], expected_z, places=6)

    def test_stationary_identity_quaternion(self) -> None:
        """Requirement 8: Stationary or low-speed motion defaults to identity quaternion."""
        q_zero = compute_yaw_quaternion(vx=0.0, vy=0.0)
        self.assertEqual(q_zero, (1.0, 0.0, 0.0, 0.0))

        q_slow = compute_yaw_quaternion(vx=0.05, vy=-0.05)
        self.assertEqual(q_slow, (1.0, 0.0, 0.0, 0.0))

    def test_udp_packet_serialization(self) -> None:
        """Requirement 9: Packet serialization and deserialization match JSON schema."""
        pkt = create_gazebo_packet(
            sim_time=2.5,
            position=(1.0, 2.0, 3.0),
            velocity=(4.0, 0.0, 0.0),
            acceleration=(0.1, -0.2, 9.81),
            flight_mode="MISSION",
        )
        payload = serialize_packet(pkt)
        self.assertIsInstance(payload, bytes)

        deserialized = deserialize_packet(payload)
        self.assertAlmostEqual(deserialized.sim_time, 2.5)
        self.assertEqual(deserialized.position, (1.0, 2.0, 3.0))
        self.assertEqual(deserialized.velocity, (4.0, 0.0, 0.0))
        self.assertEqual(deserialized.acceleration, (0.1, -0.2, 9.81))
        self.assertEqual(deserialized.flight_mode, "MISSION")


class GazeboBridgeNetworkTests(unittest.TestCase):
    def test_udp_transmission_using_localhost_test_socket(self) -> None:
        """Requirement 10: UDP datagram is transmitted to and received by a local listener."""
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2.0)
        test_port = receiver.getsockname()[1]

        bridge = GazeboBridge(GazeboBridgeConfig(host="127.0.0.1", port=test_port))
        try:
            telemetry = {
                "sim_time": 0.5,
                "mode": "TAKEOFF",
                "altitude": 10.0,
                "vz": 2.0,
                "velocity": (1.0, 0.0, 2.0),
                "acceleration": (0.0, 0.0, 1.5),
            }
            bridge.update(
                telemetry=telemetry,
                position=(5.0, 15.0, 10.0),
            )

            data, addr = receiver.recvfrom(4096)
            self.assertGreater(len(data), 0)

            pkt = deserialize_packet(data)
            self.assertEqual(pkt.sim_time, 0.5)
            self.assertEqual(pkt.position, (5.0, 15.0, 10.0))
            self.assertEqual(pkt.velocity, (1.0, 0.0, 2.0))
            self.assertEqual(pkt.acceleration, (0.0, 0.0, 1.5))
            self.assertEqual(pkt.flight_mode, "TAKEOFF")
        finally:
            bridge.close()
            receiver.close()

    def test_rate_limiting(self) -> None:
        """Requirement 11: Rapid updates respect rate_limit_hz without dropping below interval."""
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(0.5)
        test_port = receiver.getsockname()[1]

        # 10 Hz rate limit -> 0.10s interval
        bridge = GazeboBridge(GazeboBridgeConfig(host="127.0.0.1", port=test_port, rate_limit_hz=10.0))
        try:
            # Step 1: sim_time = 0.00 -> should send (1st packet)
            bridge.update(telemetry={"sim_time": 0.00}, position=(0.0, 0.0, 0.0))
            self.assertEqual(bridge._packets_sent, 1)

            # Step 2: sim_time = 0.02 -> throttled
            bridge.update(telemetry={"sim_time": 0.02}, position=(0.0, 0.0, 0.0))
            self.assertEqual(bridge._packets_sent, 1)

            # Step 3: sim_time = 0.08 -> throttled
            bridge.update(telemetry={"sim_time": 0.08}, position=(0.0, 0.0, 0.0))
            self.assertEqual(bridge._packets_sent, 1)

            # Step 4: sim_time = 0.11 -> exceeds 0.10s -> should send (2nd packet)
            bridge.update(telemetry={"sim_time": 0.11}, position=(0.0, 0.0, 0.0))
            self.assertEqual(bridge._packets_sent, 2)
        finally:
            bridge.close()
            receiver.close()

    def test_socket_error_resilience(self) -> None:
        """Requirement 12: Network and socket errors never raise into caller."""
        bridge = GazeboBridge()
        mock_sock = MagicMock()
        mock_sock.sendto.side_effect = OSError("Network unreachable")
        bridge._socket = mock_sock

        # Calling update() must not raise any exception
        bridge.update(
            telemetry={"sim_time": 1.0, "mode": "HOVER"},
            position=(0.0, 0.0, 5.0),
        )
        self.assertTrue(mock_sock.sendto.called)
        bridge.close()



    def test_visualizer_protocol_compatibility(self) -> None:
        """Requirement 13: GazeboBridge integrates with SimulationRunner as visualizer."""
        drone = Drone()
        controller = FlightController(drone)
        bridge = GazeboBridge(GazeboBridgeConfig(enabled=False))

        runner = SimulationRunner(
            drone=drone,
            controller=controller,
            visualizer=bridge,
            dt=0.05,
        )
        runner.step()
        self.assertEqual(runner.step_count, 1)

    def test_update_with_normal_telemetry(self) -> None:
        """Requirement 14: Standard runner telemetry payload updates velocity and mode."""
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2.0)
        test_port = receiver.getsockname()[1]

        bridge = GazeboBridge(GazeboBridgeConfig(host="127.0.0.1", port=test_port))
        try:
            # First update establishes previous position
            bridge.update(
                telemetry={"sim_time": 0.0, "mode": "IDLE", "vz": 0.0},
                position=(0.0, 0.0, 0.0),
            )
            # Second update computes finite difference velocity when not in telemetry
            bridge.update(
                telemetry={"sim_time": 0.1, "mode": "TAKEOFF", "vz": 1.0},
                position=(0.2, 0.0, 0.1),
            )
            data, _ = receiver.recvfrom(4096)
            data, _ = receiver.recvfrom(4096)
            pkt = deserialize_packet(data)
            self.assertEqual(pkt.flight_mode, "TAKEOFF")
            self.assertAlmostEqual(pkt.velocity[0], 2.0, places=5)
            self.assertAlmostEqual(pkt.velocity[1], 0.0, places=5)
        finally:
            bridge.close()
            receiver.close()

    def test_update_when_gazebo_unavailable(self) -> None:
        """Requirement 15: Sending to an offline Gazebo port succeeds cleanly."""
        # Port 59871 where nothing is listening
        bridge = GazeboBridge(GazeboBridgeConfig(host="127.0.0.1", port=59871))
        try:
            for i in range(5):
                bridge.update(
                    telemetry={"sim_time": float(i) * 0.1, "mode": "HOVER"},
                    position=(1.0, 2.0, 3.0),
                )
            self.assertEqual(bridge._packets_sent, 5)
        finally:
            bridge.close()

    def test_clean_close(self) -> None:
        """Requirement 16: close() cleanly terminates socket and allows safe re-entry."""
        bridge = GazeboBridge()
        self.assertTrue(bridge.connect())
        self.assertTrue(bridge.is_connected())

        bridge.close()
        self.assertFalse(bridge.is_connected())
        self.assertIsNone(bridge._socket)

        # Repeated close() is safe no-op
        bridge.close()
        self.assertFalse(bridge.is_connected())


class GazeboArchitectureTests(unittest.TestCase):
    def test_architecture_boundary(self) -> None:
        """Requirement 17: gazebo/ package does not import simulator mutators or unrelated subsystems."""
        gazebo_dir = Path(__file__).resolve().parent / "gazebo"
        forbidden_modules = {
            "drone",
            "physics",
            "flight_controller",
            "guidance",
            "navigation",
            "environment",
            "sensors",
            "fusion",
            "hal",
        }

        for py_file in gazebo_dir.glob("*.py"):
            tree = ast.parse(py_file.read_text(encoding="utf-8"), filename=str(py_file))
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        top_mod = alias.name.split(".")[0]
                        self.assertNotIn(
                            top_mod,
                            forbidden_modules,
                            msg=f"{py_file.name} violates architecture boundary by importing {top_mod}",
                        )
                elif isinstance(node, ast.ImportFrom) and node.module:
                    top_mod = node.module.split(".")[0]
                    self.assertNotIn(
                        top_mod,
                        forbidden_modules,
                        msg=f"{py_file.name} violates architecture boundary by importing from {top_mod}",
                    )


if __name__ == "__main__":
    unittest.main()
