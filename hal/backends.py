from __future__ import annotations

"""Hardware backend interfaces, registry, and mock/placeholder implementations."""

from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Set
from hal.transports import MockTransport
from flight_recorder.recorder import FlightRecorder


@dataclass
class Capability:
    protocols: Set[str]
    features: Set[str]
    max_rate_hz: float = 50.0


class BackendInterface:
    def name(self) -> str:
        raise NotImplementedError()

    def capabilities(self) -> Capability:
        raise NotImplementedError()

    def connect(self, timeout: float = 1.0) -> None:
        raise NotImplementedError()

    def disconnect(self) -> None:
        raise NotImplementedError()

    def is_connected(self) -> bool:
        raise NotImplementedError()

    def send_command(self, cmd: Dict[str, Any]) -> None:
        raise NotImplementedError()

    def register_receive_callback(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        raise NotImplementedError()

    def negotiate(self, remote: Capability) -> Capability:
        # default negotiation: intersect protocols and features
        cap = self.capabilities()
        prot = cap.protocols & remote.protocols
        feats = cap.features & remote.features
        return Capability(protocols=prot, features=feats, max_rate_hz=min(cap.max_rate_hz, remote.max_rate_hz))


class BackendRegistry:
    def __init__(self):
        self._factories: Dict[str, Callable[..., BackendInterface]] = {}

    def register(self, name: str, factory: Callable[..., BackendInterface]) -> None:
        self._factories[name] = factory

    def create(self, name: str, **kwargs) -> BackendInterface:
        if name not in self._factories:
            raise KeyError(f"Backend not registered: {name}")
        return self._factories[name](**kwargs)

    def available(self) -> List[str]:
        return list(self._factories.keys())


class MockBackend(BackendInterface):
    def __init__(self, transport: Optional[MockTransport] = None, recorder: Optional[FlightRecorder] = None, name: str = "mock"):
        self._name = name
        self._transport = transport if transport is not None else MockTransport()
        self._recorder = recorder
        self._connected = False
        self._recv_cb: Optional[Callable[[Dict[str, Any]], None]] = None
        # loopback: when send_command called, schedule delivery back to receive
        self._loopback = True

    def name(self) -> str:
        return self._name

    def capabilities(self) -> Capability:
        return Capability(protocols={"mock"}, features={"cmd_v1"}, max_rate_hz=100.0)

    def connect(self, timeout: float = 1.0) -> None:
        self._connected = True
        # register transport callback to translate received packets
        self._transport.register_receive_callback(self._on_transport_receive)
        if self._recorder is not None:
            self._recorder.record_event(0.0, "backend_connect", {"backend": self._name})

    def disconnect(self) -> None:
        self._connected = False
        if self._recorder is not None:
            self._recorder.record_event(0.0, "backend_disconnect", {"backend": self._name})

    def is_connected(self) -> bool:
        return bool(self._connected)

    def send_command(self, cmd: Dict[str, Any]) -> None:
        if not self._connected:
            raise RuntimeError("backend not connected")
        # send via transport; transport will schedule delivery
        self._transport.send(cmd, now=0.0)
        if self._recorder is not None:
            self._recorder.record_event(0.0, "backend_send", {"cmd": cmd})

    def register_receive_callback(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        self._recv_cb = cb

    def _on_transport_receive(self, packet: Any, delivery_time: float) -> None:
        # translate transport packet into backend-level command and invoke callback
        if self._recorder is not None:
            self._recorder.record_event(delivery_time, "backend_receive", {"packet": packet})
        if self._recv_cb and isinstance(packet, dict):
            try:
                self._recv_cb(packet)
            except Exception:
                pass


class SerialBackend(BackendInterface):
    def __init__(self, port: str, baud: int = 115200):
        self._port = port
        self._baud = baud
        self._connected = False

    def name(self) -> str:
        return f"serial:{self._port}@{self._baud}"

    def capabilities(self) -> Capability:
        return Capability(protocols={"serial"}, features={"mavlink"}, max_rate_hz=50.0)

    def connect(self, timeout: float = 1.0) -> None:
        # Placeholder: actual serial connection not available in this environment
        raise NotImplementedError("Serial backend not implemented in this environment")

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def send_command(self, cmd: Dict[str, Any]) -> None:
        raise NotImplementedError("Serial backend not implemented")

    def register_receive_callback(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        raise NotImplementedError("Serial backend not implemented")


class UDPBackend(BackendInterface):
    def __init__(self, host: str = "127.0.0.1", port: int = 14550):
        self._host = host
        self._port = port
        self._connected = False

    def name(self) -> str:
        return f"udp:{self._host}:{self._port}"

    def capabilities(self) -> Capability:
        return Capability(protocols={"udp"}, features={"mavlink"}, max_rate_hz=50.0)

    def connect(self, timeout: float = 1.0) -> None:
        raise NotImplementedError("UDP backend not implemented in this environment")

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def send_command(self, cmd: Dict[str, Any]) -> None:
        raise NotImplementedError("UDP backend not implemented")

    def register_receive_callback(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        raise NotImplementedError("UDP backend not implemented")


# convenience global registry
GLOBAL_BACKEND_REGISTRY = BackendRegistry()
GLOBAL_BACKEND_REGISTRY.register("mock", lambda **kwargs: MockBackend(**kwargs))
GLOBAL_BACKEND_REGISTRY.register("serial_placeholder", lambda **kwargs: SerialBackend(kwargs.get("port", "COM1"), kwargs.get("baud", 115200)))
GLOBAL_BACKEND_REGISTRY.register("udp_placeholder", lambda **kwargs: UDPBackend(kwargs.get("host", "127.0.0.1"), kwargs.get("port", 14550)))
