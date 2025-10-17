# core/rover_relay_logger.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import socket
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple


class RoverRelayLogger:
    """Simple UDP relay/logger for rover control command & feedback streams."""

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
        self.cmd_listen_ip = "0.0.0.0"
        self.cmd_listen_port = 0
        self.cmd_dest_ip = "127.0.0.1"
        self.cmd_dest_port = 0
        self.cmd_log_path = ""
        self.feedback_listen_ip = "0.0.0.0"
        self.feedback_listen_port = 0
        self.feedback_dest_ip = "127.0.0.1"
        self.feedback_dest_port = 0
        self.feedback_log_path = ""

        # Runtime handles/state
        self._cmd_sock: Optional[socket.socket] = None
        self._cmd_dest: Optional[Tuple[str, int]] = None
        self._cmd_thread: Optional[threading.Thread] = None
        self._cmd_log_file: Optional[Any] = None
        self._cmd_log_lock = threading.Lock()
        self._cmd_logged_count = 0
        self._cmd_last_ts = 0.0
        self._cmd_log_error = ""

        self._feedback_sock: Optional[socket.socket] = None
        self._feedback_dest: Optional[Tuple[str, int]] = None
        self._feedback_thread: Optional[threading.Thread] = None
        self._feedback_log_file: Optional[Any] = None
        self._feedback_log_lock = threading.Lock()
        self._feedback_logged_count = 0
        self._feedback_last_ts = 0.0
        self._feedback_log_error = ""

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
            self._prepare_command_socket()
            self._prepare_feedback_socket()
            self._open_logs(reset_counter=True)
        except Exception:
            self.stop()
            raise

        with self._lock:
            self.running = True

        self._cmd_thread = threading.Thread(target=self._proxy_loop, args=("command",), daemon=True)
        self._cmd_thread.start()
        self._feedback_thread = threading.Thread(target=self._proxy_loop, args=("feedback",), daemon=True)
        self._feedback_thread.start()

        if self._relay:
            self._relay.notify_rover_logging_changed()

        self._emit_status("RUNNING")
        self._log_status("started")

    def stop(self) -> None:
        self._stop_ev.set()
        with self._lock:
            was_running = self.running
            self.running = False

        for sock_attr in ("_cmd_sock", "_feedback_sock"):
            sock = getattr(self, sock_attr)
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
                setattr(self, sock_attr, None)

        for thread_attr in ("_cmd_thread", "_feedback_thread"):
            thread = getattr(self, thread_attr)
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
            setattr(self, thread_attr, None)

        self._close_logs()

        if self._relay:
            self._relay.notify_rover_logging_changed()

        if was_running:
            self._emit_status("STOPPED")
            self._log_status("stopped")

    def is_logging_active(self) -> bool:
        with self._cmd_log_lock, self._feedback_log_lock:
            return bool(self._cmd_log_file) or bool(self._feedback_log_file)

    def is_running(self) -> bool:
        with self._lock:
            return self.running

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            status = {
                "activated": self.running,
                "enabled": self.enabled,
                "autostart": self.autostart,
                "cmd_listen": f"{self.cmd_listen_ip}:{self.cmd_listen_port}",
                "cmd_dest": f"{self.cmd_dest_ip}:{self.cmd_dest_port}",
                "feedback_listen": f"{self.feedback_listen_ip}:{self.feedback_listen_port}",
                "feedback_dest": f"{self.feedback_dest_ip}:{self.feedback_dest_port}",
            }
        with self._cmd_log_lock:
            status.update(
                {
                    "cmd_log_path": self.cmd_log_path,
                    "cmd_logging_active": bool(self.running and self._cmd_log_file is not None),
                    "cmd_logged_count": self._cmd_logged_count,
                    "cmd_log_error": self._cmd_log_error,
                    "cmd_log_last_ts": self._cmd_last_ts if self._cmd_last_ts else None,
                }
            )
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
        status["any_logging_active"] = bool(
            status.get("cmd_logging_active") or status.get("feedback_logging_active")
        )
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

        self.cmd_listen_ip = str(self.s.get("cmd_listen_ip", "0.0.0.0"))
        self.cmd_listen_port = int(self.s.get("cmd_listen_port", 18100))
        self.cmd_dest_ip = str(self.s.get("cmd_dest_ip", "127.0.0.1"))
        self.cmd_dest_port = int(self.s.get("cmd_dest_port", 18101))
        self.cmd_log_path = str(self.s.get("cmd_log_path", "")).strip()

        self.feedback_listen_ip = str(self.s.get("feedback_listen_ip", "0.0.0.0"))
        self.feedback_listen_port = int(self.s.get("feedback_listen_port", 18102))
        self.feedback_dest_ip = str(self.s.get("feedback_dest_ip", "127.0.0.1"))
        self.feedback_dest_port = int(self.s.get("feedback_dest_port", 18103))
        self.feedback_log_path = str(self.s.get("feedback_log_path", "")).strip()

        self.s.update(
            {
                "enabled": self.enabled,
                "autostart": self.autostart,
                "cmd_listen_ip": self.cmd_listen_ip,
                "cmd_listen_port": self.cmd_listen_port,
                "cmd_dest_ip": self.cmd_dest_ip,
                "cmd_dest_port": self.cmd_dest_port,
                "cmd_log_path": self.cmd_log_path,
                "feedback_listen_ip": self.feedback_listen_ip,
                "feedback_listen_port": self.feedback_listen_port,
                "feedback_dest_ip": self.feedback_dest_ip,
                "feedback_dest_port": self.feedback_dest_port,
                "feedback_log_path": self.feedback_log_path,
            }
        )

    def _prepare_command_socket(self) -> None:
        if self._cmd_sock:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.cmd_listen_ip, self.cmd_listen_port))
        self._cmd_sock = sock
        self._cmd_dest = self._tuple_or_none(self.cmd_dest_ip, self.cmd_dest_port)
        self._cmd_logged_count = 0
        self._cmd_last_ts = 0.0
        self._cmd_log_error = ""

    def _prepare_feedback_socket(self) -> None:
        if self._feedback_sock:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.feedback_listen_ip, self.feedback_listen_port))
        self._feedback_sock = sock
        self._feedback_dest = self._tuple_or_none(self.feedback_dest_ip, self.feedback_dest_port)
        self._feedback_logged_count = 0
        self._feedback_last_ts = 0.0
        self._feedback_log_error = ""

    def _tuple_or_none(self, ip: str, port: int) -> Optional[Tuple[str, int]]:
        if not ip or port <= 0:
            return None
        return ip, int(port)

    def _open_logs(self, reset_counter: bool = False) -> None:
        self._open_stream_log(
            kind="command",
            path=self.cmd_log_path,
            file_attr="_cmd_log_file",
            lock=self._cmd_log_lock,
            counter_attr="_cmd_logged_count",
            last_ts_attr="_cmd_last_ts",
            error_attr="_cmd_log_error",
            reset_counter=reset_counter,
        )
        self._open_stream_log(
            kind="feedback",
            path=self.feedback_log_path,
            file_attr="_feedback_log_file",
            lock=self._feedback_log_lock,
            counter_attr="_feedback_logged_count",
            last_ts_attr="_feedback_last_ts",
            error_attr="_feedback_log_error",
            reset_counter=reset_counter,
        )

    def _close_logs(self) -> None:
        self._close_stream_log(
            kind="command",
            file_attr="_cmd_log_file",
            lock=self._cmd_log_lock,
            error_attr="_cmd_log_error",
        )
        self._close_stream_log(
            kind="feedback",
            file_attr="_feedback_log_file",
            lock=self._feedback_log_lock,
            error_attr="_feedback_log_error",
        )

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
                fh = open(cleaned, "a", encoding="utf-8")
                ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                fh.write(f"# --- Rover {kind} relay session started {ts} ---\n")
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
                ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                fh.write(f"# --- Rover {kind} relay session stopped {ts} ---\n")
                fh.flush()
            except Exception as e:
                setattr(self, error_attr, str(e))
            finally:
                try:
                    fh.close()
                except Exception:
                    pass
                setattr(self, file_attr, None)

    def _proxy_loop(self, kind: str) -> None:
        sock_attr, dest_attr, lock, file_attr, counter_attr, last_ts_attr, error_attr, path_attr = {
            "command": (
                "_cmd_sock",
                "_cmd_dest",
                self._cmd_log_lock,
                "_cmd_log_file",
                "_cmd_logged_count",
                "_cmd_last_ts",
                "_cmd_log_error",
                "cmd_log_path",
            ),
            "feedback": (
                "_feedback_sock",
                "_feedback_dest",
                self._feedback_log_lock,
                "_feedback_log_file",
                "_feedback_logged_count",
                "_feedback_last_ts",
                "_feedback_log_error",
                "feedback_log_path",
            ),
        }[kind]

        while not self._stop_ev.is_set():
            sock = getattr(self, sock_attr)
            if sock is None:
                break
            try:
                data, _ = sock.recvfrom(65535)
            except OSError:
                break
            except Exception as e:  # noqa: BLE001
                self._log_status(f"{kind} recv error: {e}")
                with lock:
                    setattr(self, error_attr, str(e))
                continue

            if not data:
                continue

            self._write_log(kind, data, lock, file_attr, counter_attr, last_ts_attr, error_attr, path_attr)

            dest = getattr(self, dest_attr)
            if dest:
                try:
                    sock.sendto(data, dest)
                except Exception as e:  # noqa: BLE001
                    self._log_status(f"{kind} send error: {e}")
                    with lock:
                        setattr(self, error_attr, str(e))

    def _write_log(
        self,
        kind: str,
        data: bytes,
        lock: threading.Lock,
        file_attr: str,
        counter_attr: str,
        last_ts_attr: str,
        error_attr: str,
        path_attr: str,
    ) -> None:
        path = getattr(self, path_attr)
        if not path:
            return
        with lock:
            fh = getattr(self, file_attr)
            if fh is None:
                return
            try:
                now = time.time()
                base = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
                millis = int((now - int(now)) * 1000)
                line = f"{base}.{millis:03d}\tlen={len(data)}\t{data.hex()}\n"
                fh.write(line)
                fh.flush()
                setattr(self, counter_attr, getattr(self, counter_attr) + 1)
                setattr(self, last_ts_attr, now)
                setattr(self, error_attr, "")
            except Exception as e:
                setattr(self, error_attr, str(e))
                try:
                    fh.close()
                except Exception:
                    pass
                setattr(self, file_attr, None)
                self._log_status(f"{kind} log write error: {e}")


# 타입 힌트를 위한 순환 참조 방지
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    from .udp_relay import UdpRelay
