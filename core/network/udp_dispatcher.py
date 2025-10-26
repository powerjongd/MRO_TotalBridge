"""Selector-based UDP dispatcher shared by core services."""
from __future__ import annotations

import logging
import selectors
import socket
import threading
import time
from dataclasses import dataclass, field
from types import TracebackType
from typing import Callable, Dict, Optional, Tuple, Type

PacketCallback = Callable[[bytes, Tuple[str, int], float], None]
IdleCallback = Callable[[float], None]


@dataclass
class _HandlerState:
    name: str
    callback: PacketCallback
    idle_callback: Optional[IdleCallback]
    buffer_size: int
    idle_interval: Optional[float]
    last_activity: float = field(default_factory=time.monotonic)
    last_idle_check: float = field(default_factory=time.monotonic)
    packets: int = 0
    bytes: int = 0
    errors: int = 0

    def should_run_idle(self, now: float) -> bool:
        if self.idle_callback is None or self.idle_interval is None:
            return False
        return now - self.last_idle_check >= self.idle_interval


class UdpDispatcher:
    """Multiplex UDP sockets with a single selector loop."""

    def __init__(
        self,
        *,
        name: str = "udp-dispatcher",
        log: Optional[Callable[[str], None]] = None,
        default_idle: float = 0.5,
    ) -> None:
        self._selector = selectors.DefaultSelector()
        self._name = name
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._has_handlers = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._handlers: Dict[socket.socket, _HandlerState] = {}
        self._default_idle = max(0.05, float(default_idle))
        self._logger = logging.getLogger(name)
        if log is not None:
            def _proxy(message: str, *args: object) -> None:
                try:
                    if args:
                        message = message % args
                    log(message)
                except Exception:
                    self._logger.error(message, *args)

            self._log_error = _proxy
        else:
            self._log_error = self._logger.error

        self._waker_r, self._waker_w = socket.socketpair()
        for s in (self._waker_r, self._waker_w):
            s.setblocking(False)
        self._selector.register(self._waker_r, selectors.EVENT_READ, data=None)

    # -- lifecycle -----------------------------------------------------
    def start(self) -> None:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, name=self._name, daemon=True)
            self._thread.start()

    def stop(self, *, join: bool = True) -> None:
        self._stop_event.set()
        self._wakeup()
        if join:
            thread = None
            with self._lock:
                thread = self._thread
            if thread and thread.is_alive():
                thread.join(timeout=1.5)

    def close(self) -> None:
        self.stop()
        with self._lock:
            handlers = list(self._handlers.keys())
        for sock in handlers:
            try:
                self.unregister(sock)
            except Exception:
                pass
        try:
            self._selector.unregister(self._waker_r)
        except Exception:
            pass
        for s in (self._waker_r, self._waker_w):
            try:
                s.close()
            except Exception:
                pass

    # -- registration --------------------------------------------------
    def register(
        self,
        sock: socket.socket,
        callback: PacketCallback,
        *,
        name: Optional[str] = None,
        idle_callback: Optional[IdleCallback] = None,
        idle_interval: Optional[float] = None,
        buffer_size: int = 65535,
    ) -> None:
        if buffer_size <= 0:
            raise ValueError("buffer_size must be positive")
        state = _HandlerState(
            name=name or f"sock-{sock.fileno()}",
            callback=callback,
            idle_callback=idle_callback,
            buffer_size=buffer_size,
            idle_interval=idle_interval if idle_callback is not None else None,
        )
        with self._lock:
            if sock in self._handlers:
                raise ValueError("socket already registered")
            sock.setblocking(False)
            self._handlers[sock] = state
            self._selector.register(sock, selectors.EVENT_READ, data=state)
            self._has_handlers.set()
        self._wakeup()

    def unregister(self, sock: socket.socket) -> None:
        with self._lock:
            state = self._handlers.pop(sock, None)
            try:
                self._selector.unregister(sock)
            except Exception:
                pass
            if not self._handlers:
                self._has_handlers.clear()
        if state is None:
            return
        try:
            sock.setblocking(True)
        except Exception:
            pass

    # -- metrics -------------------------------------------------------
    def snapshot(self) -> Dict[str, Dict[str, int]]:
        snap: Dict[str, Dict[str, int]] = {}
        with self._lock:
            for state in self._handlers.values():
                snap[state.name] = {
                    "packets": state.packets,
                    "bytes": state.bytes,
                    "errors": state.errors,
                }
        return snap

    # -- internals -----------------------------------------------------
    def _compute_timeout(self) -> float:
        now = time.monotonic()
        timeout = self._default_idle
        with self._lock:
            for state in self._handlers.values():
                if state.idle_callback is None or state.idle_interval is None:
                    continue
                due_in = state.idle_interval - (now - state.last_idle_check)
                if due_in <= 0:
                    return 0.0
                if due_in < timeout:
                    timeout = due_in
        return max(0.0, timeout)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            if not self._has_handlers.wait(timeout=0.1):
                continue
            timeout = self._compute_timeout()
            try:
                events = self._selector.select(timeout)
            except OSError as exc:
                self._log_error("[UDP-DISPATCH] selector error: %s", exc)
                time.sleep(0.05)
                continue

            now = time.monotonic()
            if not events:
                self._run_idle_callbacks(now)
                continue

            for key, _ in events:
                if key.data is None:
                    self._drain_waker()
                    continue
                state: _HandlerState = key.data
                sock = key.fileobj
                try:
                    data, addr = sock.recvfrom(state.buffer_size)
                except BlockingIOError:
                    continue
                except OSError as exc:
                    state.errors += 1
                    self._log_error("[UDP-DISPATCH] %s recv error: %s", state.name, exc)
                    continue
                except Exception as exc:  # noqa: BLE001
                    state.errors += 1
                    self._log_error("[UDP-DISPATCH] %s unexpected error: %s", state.name, exc)
                    continue

                if not data:
                    continue

                state.packets += 1
                state.bytes += len(data)
                state.last_activity = now
                try:
                    state.callback(data, addr, now)
                except Exception as exc:  # noqa: BLE001
                    state.errors += 1
                    self._log_error("[UDP-DISPATCH] handler %s raised: %s", state.name, exc)
            self._run_idle_callbacks(time.monotonic())

    def _run_idle_callbacks(self, now: float) -> None:
        with self._lock:
            states = list(self._handlers.values())
        for state in states:
            if not state.should_run_idle(now):
                continue
            try:
                state.idle_callback(now)
            except Exception as exc:  # noqa: BLE001
                state.errors += 1
                self._log_error("[UDP-DISPATCH] idle handler %s raised: %s", state.name, exc)
            finally:
                state.last_idle_check = now

    def _wakeup(self) -> None:
        try:
            self._waker_w.send(b"\x00")
        except BlockingIOError:
            pass
        except Exception:
            pass

    def _drain_waker(self) -> None:
        try:
            while self._waker_r.recv(1024):
                pass
        except BlockingIOError:
            return
        except Exception:
            return

    def __enter__(self) -> "UdpDispatcher":
        self.start()
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc: Optional[BaseException],
        tb: Optional[TracebackType],
    ) -> Optional[bool]:
        self.stop()
        return None


_global_dispatcher: Optional[UdpDispatcher] = None
_global_lock = threading.Lock()


def get_global_dispatcher() -> UdpDispatcher:
    """Return a lazily created global dispatcher."""
    global _global_dispatcher
    with _global_lock:
        if _global_dispatcher is None:
            _global_dispatcher = UdpDispatcher()
            _global_dispatcher.start()
    return _global_dispatcher


def shutdown_global_dispatcher() -> None:
    global _global_dispatcher
    with _global_lock:
        if _global_dispatcher is None:
            return
        dispatcher = _global_dispatcher
        _global_dispatcher = None
    dispatcher.close()
