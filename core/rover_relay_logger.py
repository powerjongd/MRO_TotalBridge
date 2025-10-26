# core/rover_relay_logger.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import queue
import socket
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

from PySide6 import QtCore

from core.network import get_global_dispatcher

from .log_parsers import UNIFIED_CSV_HEADER, RoverFeedbackCsvFormatter


_FEEDBACK_LOG_RESET = object()


class _FeedbackLogWorker(QtCore.QObject):
    """Qt worker that processes queued rover feedback log entries."""

    finished = QtCore.Signal()

    def __init__(self, owner: "RoverRelayLogger") -> None:
        super().__init__()
        self._owner = owner

    @QtCore.Slot()
    def run(self) -> None:
        try:
            self._owner._feedback_log_writer_loop()
        finally:
            self.finished.emit()


class RoverRelayLogger:
    """Simple UDP relay/logger for rover feedback streams."""

    def __init__(
        self,
        log_cb: Callable[[str], None],
        status_cb: Optional[Callable[[str], None]],
        settings: Dict[str, Any],
    ) -> None:
        self.log = log_cb
        self.status_cb = status_cb
        self.s = dict(settings)

        self._lock = threading.Lock()
        self._stop_ev = threading.Event()
        self.running = False

        # Configurable options (populated via _load_settings)
        self.enabled = True
        self.autostart = False
        self.feedback_listen_ip = "0.0.0.0"
        self.feedback_listen_port = 0
        self.feedback_dest_ip = "127.0.0.1"
        self.feedback_dest_port = 0
        self.feedback_log_path = ""

        # Runtime handles/state
        self._feedback_sock: Optional[socket.socket] = None
        self._feedback_tx_sock: Optional[socket.socket] = None
        self._feedback_dest: Optional[Tuple[str, int]] = None
        self._feedback_log_file: Optional[Any] = None
        self._feedback_log_lock = threading.Lock()
        self._feedback_logged_count = 0
        self._feedback_last_ts = 0.0
        self._feedback_log_error = ""
        self._feedback_log_closing = False
        self._feedback_log_queue: "queue.Queue[object]" = queue.Queue()
        self._feedback_worker = _FeedbackLogWorker(self)
        self._feedback_thread = QtCore.QThread()
        self._feedback_thread.setObjectName("rover-csv-writer")
        self._feedback_worker.moveToThread(self._feedback_thread)
        self._feedback_thread.started.connect(self._feedback_worker.run)
        self._feedback_worker.finished.connect(self._feedback_thread.quit)
        self._feedback_worker.finished.connect(self._feedback_worker.deleteLater)
        self._feedback_thread.start()
        self._feedback_thread_shutdown = False

        self._dispatcher = get_global_dispatcher()
        self._feedback_registered = False

        self._relay: Optional["UdpRelay"] = None

        self._load_settings()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def register_relay(self, relay: Optional["UdpRelay"]) -> None:
        self._relay = relay

    def update_settings(self, values: Dict[str, Any]) -> None:
        with self._lock:
            self.s.update(values)
            self._load_settings()
            was_running = self.running
        self._log_status("settings updated")
        if was_running:
            self.stop()
            if self.enabled:
                self.start()

    def start(self) -> None:
        with self._lock:
            if self.running:
                return
            enabled = self.enabled
        if not enabled:
            raise RuntimeError("Rover relay logger disabled in settings")
        if self._relay and self._relay.is_gazebo_logging_active():
            raise RuntimeError("Gazebo logging is active. Disable it before starting rover logging")

        self._stop_ev.clear()

        try:
            self._prepare_feedback_socket()
            self._open_logs(reset_counter=True)
        except Exception:
            self.stop()
            raise

        try:
            sock = self._feedback_sock
            if sock is None:
                raise RuntimeError("feedback socket not available")
            self._dispatcher.register(
                sock,
                self._handle_feedback_packet,
                name=f"rover-feedback-{self.feedback_listen_port}",
                idle_callback=None,
                buffer_size=65535,
            )
            self._feedback_registered = True
        except Exception:
            self.stop()
            raise

        with self._lock:
            self.running = True

        if self._relay:
            self._relay.notify_rover_logging_changed()

        self._emit_status("RUNNING")
        self._log_status("started")

    def stop(self) -> None:
        self._stop_ev.set()
        with self._lock:
            was_running = self.running
            self.running = False

        sock = self._feedback_sock
        if self._feedback_registered and sock is not None:
            try:
                self._dispatcher.unregister(sock)
            except Exception:
                pass
            self._feedback_registered = False
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass
            self._feedback_sock = None

        tx_sock = self._feedback_tx_sock
        if tx_sock is not None:
            try:
                tx_sock.close()
            except Exception:
                pass
            self._feedback_tx_sock = None

        self._close_logs()

        if self._relay:
            self._relay.notify_rover_logging_changed()

        if was_running:
            self._emit_status("STOPPED")
            self._log_status("stopped")

    def close(self) -> None:
        """Stop the feedback log worker thread gracefully."""

        thread = getattr(self, "_feedback_thread", None)
        if thread is None or self._feedback_thread_shutdown:
            return
        self._feedback_thread_shutdown = True
        try:
            self._feedback_log_queue.put(None)
        except Exception:
            pass
        thread.wait(1500)
        thread.deleteLater()
        self._feedback_thread = None
        self._feedback_worker = None

    def is_logging_active(self) -> bool:
        with self._feedback_log_lock:
            return bool(self._feedback_log_file)

    def is_running(self) -> bool:
        with self._lock:
            return self.running

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            status = {
                "activated": self.running,
                "enabled": self.enabled,
                "autostart": self.autostart,
                "feedback_listen": f"{self.feedback_listen_ip}:{self.feedback_listen_port}",
                "feedback_dest": f"{self.feedback_dest_ip}:{self.feedback_dest_port}",
            }
        with self._feedback_log_lock:
            status.update(
                {
                    "feedback_log_path": self.feedback_log_path,
                    "feedback_logging_active": bool(self.running and self._feedback_log_file is not None),
                    "feedback_logged_count": self._feedback_logged_count,
                    "feedback_log_error": self._feedback_log_error,
                    "feedback_log_last_ts": self._feedback_last_ts if self._feedback_last_ts else None,
                }
            )
        status["any_logging_active"] = bool(status.get("feedback_logging_active"))
        return status

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _log_status(self, msg: str) -> None:
        try:
            self.log(f"[ROVER] {msg}")
        except Exception:
            pass

    def _emit_status(self, text: str) -> None:
        if not self.status_cb:
            return
        try:
            self.status_cb(text)
        except Exception:
            pass

    def _load_settings(self) -> None:
        self.enabled = bool(self.s.get("enabled", True))
        self.autostart = bool(self.s.get("autostart", False))

        self.feedback_listen_ip = str(self.s.get("feedback_listen_ip", "0.0.0.0"))
        self.feedback_listen_port = int(self.s.get("feedback_listen_port", 18102))
        self.feedback_dest_ip = str(self.s.get("feedback_dest_ip", "127.0.0.1"))
        self.feedback_dest_port = int(self.s.get("feedback_dest_port", 18103))
        self.feedback_log_path = str(self.s.get("feedback_log_path", "")).strip()

        self.s.update(
            {
                "enabled": self.enabled,
                "autostart": self.autostart,
                "feedback_listen_ip": self.feedback_listen_ip,
                "feedback_listen_port": self.feedback_listen_port,
                "feedback_dest_ip": self.feedback_dest_ip,
                "feedback_dest_port": self.feedback_dest_port,
                "feedback_log_path": self.feedback_log_path,
            }
        )
        for obsolete in (
            "cmd_listen_ip",
            "cmd_listen_port",
            "cmd_dest_ip",
            "cmd_dest_port",
            "cmd_log_path",
        ):
            self.s.pop(obsolete, None)

    def _prepare_feedback_socket(self) -> None:
        if self._feedback_sock:
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
        except OSError:
            pass
        sock.bind((self.feedback_listen_ip, self.feedback_listen_port))
        self._feedback_sock = sock

        dest = self._tuple_or_none(self.feedback_dest_ip, self.feedback_dest_port)
        self._feedback_dest = dest

        tx_sock: Optional[socket.socket] = None
        if dest:
            tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2 * 1024 * 1024)
            except OSError:
                pass
            try:
                tx_sock.connect(dest)
            except OSError as exc:
                self._log_status(f"feedback dest connect error: {exc}")
                try:
                    tx_sock.close()
                except Exception:
                    pass
                tx_sock = None
        self._feedback_tx_sock = tx_sock

        self._feedback_logged_count = 0
        self._feedback_last_ts = 0.0
        self._feedback_log_error = ""

    def _tuple_or_none(self, ip: str, port: int) -> Optional[Tuple[str, int]]:
        if not ip or port <= 0:
            return None
        return ip, int(port)

    def _open_logs(self, reset_counter: bool = False) -> None:
        self._open_stream_log(
            kind="feedback",
            path=self.feedback_log_path,
            file_attr="_feedback_log_file",
            lock=self._feedback_log_lock,
            counter_attr="_feedback_logged_count",
            last_ts_attr="_feedback_last_ts",
            error_attr="_feedback_log_error",
            header=UNIFIED_CSV_HEADER,
            reset_counter=reset_counter,
        )
        self._feedback_log_closing = False
        if reset_counter:
            self._request_feedback_log_reset()

    def _request_feedback_log_reset(self) -> None:
        """Notify the log writer to discard cached rover velocity state."""

        try:
            self._feedback_log_queue.put_nowait(_FEEDBACK_LOG_RESET)
        except queue.Full:
            # Dropping the reset is safe; the worker keeps previous state until
            # more packets arrive, which only affects fallback velocity smoothing.
            pass

    def _close_logs(self) -> None:
        with self._feedback_log_lock:
            closing = bool(self._feedback_log_file)
            if closing:
                self._feedback_log_closing = True
        if closing:
            self._feedback_log_queue.join()
        self._close_stream_log(
            kind="feedback",
            file_attr="_feedback_log_file",
            lock=self._feedback_log_lock,
            error_attr="_feedback_log_error",
        )
        self._feedback_log_closing = False

    def _open_stream_log(
        self,
        *,
        kind: str,
        path: str,
        file_attr: str,
        lock: threading.Lock,
        counter_attr: str,
        last_ts_attr: str,
        error_attr: str,
        header: str,
        reset_counter: bool,
    ) -> None:
        cleaned = str(path or "").strip()
        if reset_counter:
            setattr(self, counter_attr, 0)
            setattr(self, last_ts_attr, 0.0)
        with lock:
            if getattr(self, file_attr):
                try:
                    getattr(self, file_attr).close()
                except Exception:
                    pass
                setattr(self, file_attr, None)
            if not cleaned:
                setattr(self, error_attr, "")
                return
            try:
                directory = os.path.dirname(cleaned)
                if directory:
                    os.makedirs(directory, exist_ok=True)
                write_header = not os.path.exists(cleaned) or os.path.getsize(cleaned) == 0
                fh = open(cleaned, "a", encoding="utf-8")
                if write_header and header:
                    fh.write(header + "\n")
                fh.flush()
                setattr(self, file_attr, fh)
                setattr(self, error_attr, "")
                setattr(self, last_ts_attr, time.time())
            except Exception as e:
                setattr(self, error_attr, str(e))
                setattr(self, file_attr, None)
                raise

    def _close_stream_log(
        self,
        *,
        kind: str,
        file_attr: str,
        lock: threading.Lock,
        error_attr: str,
    ) -> None:
        with lock:
            fh = getattr(self, file_attr)
            if not fh:
                return
            try:
                fh.flush()
            except Exception as e:
                setattr(self, error_attr, str(e))
            finally:
                try:
                    fh.close()
                except Exception:
                    pass
                setattr(self, file_attr, None)

    def _handle_feedback_packet(
        self,
        packet: bytes,
        _addr: Tuple[str, int],
        _timestamp: float,
    ) -> None:
        if not packet:
            return

        lock = self._feedback_log_lock
        error_attr = "_feedback_log_error"

        tx_sock = self._feedback_tx_sock
        if tx_sock is not None:
            try:
                tx_sock.send(packet)
            except Exception as exc:  # noqa: BLE001
                self._log_status(f"feedback send error: {exc}")
                with lock:
                    setattr(self, error_attr, str(exc))

        self._queue_feedback_log(
            memoryview(packet),
            lock=lock,
            file_attr="_feedback_log_file",
            error_attr=error_attr,
        )

    def _queue_feedback_log(
        self,
        payload_view: memoryview,
        *,
        lock: threading.Lock,
        file_attr: str,
        error_attr: str,
    ) -> None:
        with lock:
            fh = getattr(self, file_attr)
            active = fh is not None and not self._feedback_log_closing
        if not active:
            return
        try:
            packet = bytes(payload_view)
        except MemoryError:
            with lock:
                setattr(self, error_attr, "feedback log buffer allocation failed")
            return
        self._feedback_log_queue.put(packet)

    def _feedback_log_writer_loop(self) -> None:
        formatter = RoverFeedbackCsvFormatter()
        while True:
            try:
                item = self._feedback_log_queue.get()
            except Exception:
                return
            try:
                if item is None:
                    break
                if item is _FEEDBACK_LOG_RESET:
                    formatter.reset()
                    continue
                if isinstance(item, memoryview):
                    payload = item.tobytes()
                elif isinstance(item, bytes):
                    payload = item
                elif isinstance(item, bytearray):
                    payload = bytes(item)
                else:
                    payload = bytes(item)
                try:
                    line = formatter.format_packet(payload)
                except ValueError as exc:
                    with self._feedback_log_lock:
                        self._feedback_log_error = str(exc)
                    continue
                with self._feedback_log_lock:
                    fh = self._feedback_log_file
                if not fh:
                    continue
                try:
                    fh.write(line + "\n")
                    fh.flush()
                except Exception as exc:  # noqa: BLE001
                    with self._feedback_log_lock:
                        self._feedback_log_error = str(exc)
                        current = self._feedback_log_file
                        self._feedback_log_file = None
                    if current:
                        try:
                            current.close()
                        except Exception:
                            pass
                    self._log_status(f"feedback log write error: {exc}")
                else:
                    now = time.time()
                    with self._feedback_log_lock:
                        self._feedback_logged_count += 1
                        self._feedback_last_ts = now
                        self._feedback_log_error = ""
            finally:
                self._feedback_log_queue.task_done()


# 타입 힌트를 위한 순환 참조 방지
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    from .udp_relay import UdpRelay
