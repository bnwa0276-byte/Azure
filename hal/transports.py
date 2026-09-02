from __future__ import annotations

"""Transport interfaces and implementations for HIL bridge."""

from typing import Any, Callable, List, Tuple, Optional
import random


class TransportInterface:
    def send(self, packet: Any, now: float) -> None:
        raise NotImplementedError()

    def register_receive_callback(self, cb: Callable[[Any, float], None]) -> None:
        raise NotImplementedError()


class MockTransport(TransportInterface):
    def __init__(self, latency: float = 0.0, jitter: float = 0.0, loss: float = 0.0, seed: Optional[int] = None):
        self.latency = float(latency)
        self.jitter = float(jitter)
        self.loss = float(loss)
        self._rng = random.Random(seed)
        self._recv_cbs: List[Callable[[Any, float], None]] = []
        # scheduled deliveries: list of (delivery_time, packet)
        self._outbox: List[Tuple[float, Any]] = []

    def send(self, packet: Any, now: float) -> None:
        # simulate packet loss
        if self._rng.random() < self.loss:
            return
        jitter = (self._rng.random() * 2 - 1.0) * self.jitter
        delivery = now + max(0.0, self.latency + jitter)
        self._outbox.append((delivery, packet))

    def register_receive_callback(self, cb: Callable[[Any, float], None]) -> None:
        # allow multiple listeners (HIL bridge, backend, logger, etc.)
        self._recv_cbs.append(cb)

    def advance_time(self, now: float) -> List[Tuple[float, Any]]:
        # release due packets and invoke callback
        due = [item for item in self._outbox if item[0] <= now]
        self._outbox = [item for item in self._outbox if item[0] > now]
        for delivery_time, packet in sorted(due, key=lambda x: x[0]):
            for cb in list(self._recv_cbs):
                try:
                    cb(packet, delivery_time)
                except Exception:
                    pass
        return due


class SerialTransport(TransportInterface):
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud

    def send(self, packet: Any, now: float) -> None:
        # placeholder: real implementation would write to serial
        raise NotImplementedError("Serial transport not implemented in this environment")

    def register_receive_callback(self, cb: Callable[[Any, float], None]) -> None:
        # placeholder
        raise NotImplementedError("Serial transport not implemented in this environment")


class UDPTransport(TransportInterface):
    def __init__(self, host: str = "127.0.0.1", port: int = 14550):
        self.host = host
        self.port = port

    def send(self, packet: Any, now: float) -> None:
        raise NotImplementedError("UDP transport not implemented in this environment")

    def register_receive_callback(self, cb: Callable[[Any, float], None]) -> None:
        raise NotImplementedError("UDP transport not implemented in this environment")
