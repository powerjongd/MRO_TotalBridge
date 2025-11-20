"""PySide6 main window for Unified Bridge."""
from __future__ import annotations

import io
import logging
import math
import threading
import time
from collections import deque
from typing import Any, Callable, Deque, Dict, Optional

from PIL import Image
from PySide6 import QtConcurrent, QtCore, QtGui, QtWidgets

from unified_bridge.support.helpers import get_recent_log_lines
from unified_bridge.support.observers import ObservableFloat
from unified_bridge.support.zoom import BASE_ZOOM_LENS_MM


class _LogEmitter(QtCore.QObject):
    message = QtCore.Signal(str)


class QtLogHandler(logging.Handler):
    """Logging handler that forwards records to a QTextEdit in a thread-safe way."""

    def __init__(
        self,
        widget: Optional[QtWidgets.QTextEdit] = None,
        level: int = logging.NOTSET,
    ) -> None:
        super().__init__(level)
        self.widget: Optional[QtWidgets.QTextEdit] = None
        self._emitter = _LogEmitter()
        self._emitter.message.connect(self._append)
        if widget is not None:
            self.set_widget(widget)

    def set_widget(self, widget: QtWidgets.QTextEdit) -> None:
        """Attach the QTextEdit sink after construction."""

        self.widget = widget

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
        except Exception:
            self.handleError(record)
            return
        self._emitter.message.emit(msg)

    @QtCore.Slot(str)
    def _append(self, msg: str) -> None:
        if self.widget is None:
            return
        self.widget.append(msg)
        self.widget.moveCursor(QtGui.QTextCursor.End)


class _PreviewBridge(QtCore.QObject):
    frame_ready = QtCore.Signal(bytes)


class PreviewLabel(QtWidgets.QLabel):
    resized = QtCore.Signal(QtCore.QSize)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(320, 240)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet("background-color: #202020; border: 1px solid #404040;")

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # pragma: no cover - GUI callback
        super().resizeEvent(event)
        self.resized.emit(event.size())


class MainWindow(QtWidgets.QMainWindow):
    """PySide6 port of the Unified Bridge main window."""

    zoom_changed = QtCore.Signal(float)

    def __init__(
        self,
        cfg: Dict[str, Any],
        bridge,
        gimbal,
        relay,
        rover,
        log: logging.Logger,
        zoom_state: Optional[ObservableFloat] = None,
    ) -> None:
        super().__init__()
        self.setWindowTitle("Unified Bridge")
        self.resize(960, 760)

        self.cfg = cfg
        self.bridge = bridge
        self.gimbal = gimbal
        self.relay = relay
        self.rover = rover
        self.log = log
        self.zoom_state = zoom_state
        self._zoom_unsubscribe: Optional[Callable[[], None]] = None
        self._status_label_min_width = 280

        self._preview_bridge = _PreviewBridge()
        self._preview_bridge.frame_ready.connect(self._handle_preview_frame)
        self._last_image: Optional[QtGui.QPixmap] = None
        self._preview_last_ts = 0.0
        self._preview_last_size = "-"
        self._preview_last_time = "-"
        self._preview_min_interval = float(self.cfg.get("bridge", {}).get("preview_min_interval", 1.0))
        self._preview_max_queue = 5
        self._preview_queue: Deque[bytes] = deque(maxlen=self._preview_max_queue)
        self._preview_lock = threading.Lock()
        self._current_zoom_value = float(self.cfg.get("gimbal", {}).get("zoom_scale", 1.0))
        self._preview_target_size = QtCore.QSize(640, 480)
        self._disconnect_in_progress = False
        self._background_watchers: list[QtCore.QFutureWatcher] = []
        self._bridge_cfg = self.cfg.setdefault("bridge", {})
        self._initial_tcp_enabled = bool(self._bridge_cfg.get("enable_tcp", True))
        self.chk_tcp_enabled: Optional[QtWidgets.QCheckBox] = None

        gimbal_cfg = self.cfg.setdefault("gimbal", {})
        initial_method = str(gimbal_cfg.get("control_method", "tcp")).lower()
        if initial_method not in {"tcp", "mavlink"}:
            initial_method = "tcp"
        gimbal_cfg["control_method"] = initial_method

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        self._layout = QtWidgets.QVBoxLayout(central)
        self._layout.setContentsMargins(8, 8, 8, 8)
        self._layout.setSpacing(8)

        self._build_header(initial_method)
        self._build_preview_area()
        self._build_log_area()

        self.zoom_changed.connect(self._apply_zoom_value)
        self._init_zoom_subscription()
        self._install_preview_bridge()
        self._refresh_status_labels()

        self._status_timer = QtCore.QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._refresh_status_labels)
        self._status_timer.start()

        self._preview_timer = QtCore.QTimer(self)
        self._preview_timer.setInterval(50)
        self._preview_timer.timeout.connect(self._drain_preview_queue)
        self._preview_timer.start()

        self._restore_recent_logs()

    # ------------------------------------------------------------------
    def _run_background(self, func: Callable[[], object], slot: Callable[[object], None]) -> None:
        watcher = QtCore.QFutureWatcher(self)
        future = QtConcurrent.run(func)
        self._background_watchers.append(watcher)

        def _cleanup() -> None:
            try:
                result = future.result()
            except Exception as exc:  # pragma: no cover - defensive
                payload: object = exc
            else:
                payload = result
            try:
                slot(payload)
            finally:
                try:
                    self._background_watchers.remove(watcher)
                except ValueError:
                    pass
                watcher.deleteLater()

        watcher.finished.connect(_cleanup)
        watcher.setFuture(future)

    # ------------------------------------------------------------------
    # UI construction helpers
    # ------------------------------------------------------------------
    def _build_header(self, initial_method: str) -> None:
        header = QtWidgets.QWidget()
        header_layout = QtWidgets.QVBoxLayout(header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(10)

        # Image stream controls -------------------------------------------------
        bridge_box = QtWidgets.QGroupBox("Image Stream")
        bridge_layout = QtWidgets.QVBoxLayout(bridge_box)
        bridge_layout.setContentsMargins(8, 8, 8, 8)
        bridge_layout.setSpacing(6)

        bridge_buttons = QtWidgets.QHBoxLayout()
        bridge_buttons.setContentsMargins(0, 0, 0, 0)
        bridge_buttons.setSpacing(6)

        self.btn_server = QtWidgets.QPushButton("Disconnect Image Stream Module")
        self.btn_server.clicked.connect(self.on_disconnect_stream)
        bridge_buttons.addWidget(self.btn_server)

        self.btn_bridge = QtWidgets.QPushButton("Image Stream Settings")
        self.btn_bridge.clicked.connect(self.open_bridge_window)
        bridge_buttons.addWidget(self.btn_bridge)
        self.chk_tcp_enabled = QtWidgets.QCheckBox("Enable TCP/IP Connection")
        self.chk_tcp_enabled.setChecked(self._initial_tcp_enabled)
        self.chk_tcp_enabled.toggled.connect(self._on_tcp_enable_toggled)
        bridge_buttons.addWidget(self.chk_tcp_enabled)
        bridge_buttons.addStretch()
        bridge_layout.addLayout(bridge_buttons)

        self.lbl_bridge = QtWidgets.QLabel("Image Stream: Stopped · Realtime")
        self.lbl_bridge.setWordWrap(True)
        bridge_layout.addWidget(self.lbl_bridge)

        self.lbl_bridge_extra = QtWidgets.QLabel("TCP OFF / UDP OFF")
        self.lbl_bridge_extra.setWordWrap(True)
        self.lbl_bridge_extra.setStyleSheet("color: #888888;")
        bridge_layout.addWidget(self.lbl_bridge_extra)

        header_layout.addWidget(bridge_box)

        # Gimbal controls -------------------------------------------------------
        gimbal_box = QtWidgets.QGroupBox("Gimbal")
        gimbal_layout = QtWidgets.QVBoxLayout(gimbal_box)
        gimbal_layout.setContentsMargins(8, 8, 8, 8)
        gimbal_layout.setSpacing(6)

        gimbal_buttons = QtWidgets.QHBoxLayout()
        gimbal_buttons.setContentsMargins(0, 0, 0, 0)
        gimbal_buttons.setSpacing(6)
        self.btn_gimbal = QtWidgets.QPushButton("Gimbal Controls")
        self.btn_gimbal.clicked.connect(self.open_gimbal_window)
        gimbal_buttons.addWidget(self.btn_gimbal)
        gimbal_buttons.addStretch()
        gimbal_layout.addLayout(gimbal_buttons)

        mode_row = QtWidgets.QHBoxLayout()
        mode_row.setContentsMargins(0, 0, 0, 0)
        mode_row.setSpacing(6)
        mode_label = QtWidgets.QLabel("Control Mode:")
        self.rb_tcp = QtWidgets.QRadioButton("TCP/IP")
        self.rb_mav = QtWidgets.QRadioButton("MAVLink")
        mode_row.addWidget(mode_label)
        mode_row.addWidget(self.rb_tcp)
        mode_row.addWidget(self.rb_mav)
        mode_row.addStretch()
        gimbal_layout.addLayout(mode_row)
        if initial_method == "mavlink":
            self.rb_mav.setChecked(True)
        else:
            self.rb_tcp.setChecked(True)
        self.rb_tcp.toggled.connect(self._on_gimbal_method_changed)

        self.lbl_gimbal = self._make_status_label("Gimbal: Deactivated")
        gimbal_layout.addWidget(self.lbl_gimbal)

        header_layout.addWidget(gimbal_box)

        # Relay controls --------------------------------------------------------
        relay_box = QtWidgets.QGroupBox("Relay & Logging")
        relay_layout = QtWidgets.QVBoxLayout(relay_box)
        relay_layout.setContentsMargins(8, 8, 8, 8)
        relay_layout.setSpacing(6)

        relay_row = QtWidgets.QHBoxLayout()
        relay_row.setContentsMargins(0, 0, 0, 0)
        relay_row.setSpacing(6)
        self.btn_relay = QtWidgets.QPushButton("Relay Settings")
        self.btn_relay.clicked.connect(self.open_relay_window)
        relay_row.addWidget(self.btn_relay)
        relay_row.addStretch()
        self.lbl_relay = self._make_status_label("Relay: Deactivated")
        relay_row.addWidget(self.lbl_relay)
        relay_layout.addLayout(relay_row)

        rover_row = QtWidgets.QHBoxLayout()
        rover_row.setContentsMargins(0, 0, 0, 0)
        rover_row.setSpacing(6)
        self.btn_rover = QtWidgets.QPushButton("Rover Relay Logging")
        self.btn_rover.clicked.connect(self.open_rover_window)
        rover_row.addWidget(self.btn_rover)
        rover_row.addStretch()
        self.lbl_rover = self._make_status_label("Rover Logging: Idle")
        rover_row.addWidget(self.lbl_rover)
        relay_layout.addLayout(rover_row)

        self.lbl_relay_log = self._make_status_label("Gazebo Logging: Idle")
        self.lbl_relay_log.setStyleSheet("color: #888888;")
        relay_layout.addWidget(self.lbl_relay_log)

        header_layout.addWidget(relay_box)

        self._layout.addWidget(header)

    def _make_status_label(self, text: str) -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(text)
        label.setWordWrap(True)
        label.setMinimumWidth(self._status_label_min_width)
        return label

    def _format_zoom_label(self, zoom_scale: float) -> str:
        lens_mm = zoom_scale * BASE_ZOOM_LENS_MM
        return f"Zoom: x{zoom_scale:.2f} (Focal Length {lens_mm:.1f} mm)"

    def _build_preview_area(self) -> None:
        group = QtWidgets.QGroupBox("Image Preview")
        vbox = QtWidgets.QVBoxLayout(group)
        vbox.setContentsMargins(8, 8, 8, 8)

        self.preview_label = PreviewLabel()
        self.preview_label.resized.connect(self._on_preview_resized)
        vbox.addWidget(self.preview_label, stretch=1)

        info_layout = QtWidgets.QHBoxLayout()
        self.lbl_preview_info = QtWidgets.QLabel("Last: - | Size: -")
        info_layout.addWidget(self.lbl_preview_info)

        self.lbl_zoom = QtWidgets.QLabel(self._format_zoom_label(self._current_zoom_value))
        info_layout.addWidget(self.lbl_zoom)

        info_layout.addStretch()

        vbox.addLayout(info_layout)
        self._layout.addWidget(group, stretch=1)

    def _build_log_area(self) -> None:
        group = QtWidgets.QGroupBox("Log Output")
        vbox = QtWidgets.QVBoxLayout(group)
        self.txt_log = QtWidgets.QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        vbox.addWidget(self.txt_log)
        self._layout.addWidget(group, stretch=1)

        self._log_handler = QtLogHandler(self.txt_log)
        self._log_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
        self.log.addHandler(self._log_handler)

    # ------------------------------------------------------------------
    # Initialization helpers
    # ------------------------------------------------------------------
    def _init_zoom_subscription(self) -> None:
        if self.zoom_state is None:
            return

        def _on_zoom(value: float) -> None:
            self.zoom_changed.emit(value)

        self._zoom_unsubscribe = self.zoom_state.subscribe(_on_zoom)

    @QtCore.Slot(float)
    def _apply_zoom_value(self, value: float) -> None:
        self._current_zoom_value = value
        self.lbl_zoom.setText(self._format_zoom_label(value))

    def _sync_zoom_from_status(self, value: float) -> None:
        if abs(value - self._current_zoom_value) < 1e-3:
            return
        if self.zoom_state is not None:
            self.zoom_state.set(value)
        else:
            self._apply_zoom_value(value)

    def _install_preview_bridge(self) -> None:
        try:
            self.bridge.preview_cb = self.on_preview
        except Exception:
            pass

    def _restore_recent_logs(self) -> None:
        try:
            lines = get_recent_log_lines(self.log, limit=200)
        except Exception:
            lines = []
        if lines:
            self.txt_log.append("\n".join(lines))
            self.txt_log.moveCursor(QtGui.QTextCursor.End)

    # ------------------------------------------------------------------
    # Preview handling
    # ------------------------------------------------------------------
    def on_preview(self, frame: bytes, timestamp: Optional[float] = None) -> None:
        with self._preview_lock:
            self._preview_queue.append(frame)
        QtCore.QMetaObject.invokeMethod(
            self,
            "_drain_preview_queue",
            QtCore.Qt.QueuedConnection,
        )

    @QtCore.Slot()
    def _drain_preview_queue(self) -> None:
        with self._preview_lock:
            if not self._preview_queue:
                return
            frame = self._preview_queue.pop()
            self._preview_queue.clear()
        self._preview_bridge.frame_ready.emit(frame)

    @QtCore.Slot(bytes)
    def _handle_preview_frame(self, frame: bytes) -> None:
        now = time.monotonic()
        if (now - self._preview_last_ts) < max(0.0, self._preview_min_interval):
            return
        self._preview_last_ts = now
        try:
            image = QtGui.QImage.fromData(frame)
            if image.isNull():
                image = self._decode_via_pillow(frame)
        except Exception:
            image = self._decode_via_pillow(frame)
        if image.isNull():
            return

        pixmap = QtGui.QPixmap.fromImage(image)
        if not pixmap.isNull():
            scaled = pixmap.scaled(
                self._preview_target_size,
                QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation,
            )
            self.preview_label.setPixmap(scaled)
            self._last_image = pixmap
            self._preview_last_size = f"{image.width()}x{image.height()}"
            self._preview_last_time = time.strftime("%H:%M:%S")
            self.lbl_preview_info.setText(
                f"Last: {self._preview_last_time} | Size: {self._preview_last_size}"
            )

    def _decode_via_pillow(self, data: bytes) -> QtGui.QImage:
        try:
            with Image.open(io.BytesIO(data)) as img:
                rgb = img.convert("RGBA")
                ptr = rgb.tobytes("raw", "RGBA")
                qimg = QtGui.QImage(
                    ptr,
                    rgb.width,
                    rgb.height,
                    rgb.width * 4,
                    QtGui.QImage.Format_RGBA8888,
                )
                return qimg.copy()
        except Exception:
            return QtGui.QImage()

    @QtCore.Slot(QtCore.QSize)
    def _on_preview_resized(self, size: QtCore.QSize) -> None:
        self._preview_target_size = size
        if self._last_image is not None:
            scaled = self._last_image.scaled(
                size,
                QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation,
            )
            self.preview_label.setPixmap(scaled)

    # ------------------------------------------------------------------
    # Status updates
    # ------------------------------------------------------------------
    def _refresh_status_labels(self) -> None:
        try:
            running = bool(getattr(self.bridge, "is_server_running", None) and self.bridge.is_server_running.is_set())
        except Exception:
            running = False
        status = getattr(self.bridge, "get_runtime_status", None)
        if callable(status):
            try:
                st = status()
            except Exception:
                st = {}
        else:
            st = {}
        client_connected = bool(st.get("tcp_client_connected"))
        tcp_enabled = bool(st.get("tcp_enabled", True))
        if self.chk_tcp_enabled is not None and self.chk_tcp_enabled.isChecked() != tcp_enabled:
            self.chk_tcp_enabled.blockSignals(True)
            self.chk_tcp_enabled.setChecked(tcp_enabled)
            self.chk_tcp_enabled.blockSignals(False)
        if not self._disconnect_in_progress:
            self.btn_server.setEnabled(client_connected)

        mode = st.get("image_source_mode", "Realtime")
        if not tcp_enabled:
            tcp_on = "DISABLED"
        else:
            tcp_on = "ON" if st.get("tcp_listening") else "OFF"
        udp_on = "ON" if st.get("udp_listening") else "OFF"
        client_on = "ON" if client_connected else "OFF"
        mode_label = str(mode)
        stream_state = "Running" if running else "Stopped"
        self.lbl_bridge.setText(f"Image Stream: {stream_state} · {mode_label}")
        self.lbl_bridge_extra.setText(f"TCP {tcp_on} / UDP {udp_on} / Client {client_on}")

        gimbal_state = getattr(self.gimbal, "get_status", None)
        gimbal_status: Optional[Dict[str, Any]]
        if callable(gimbal_state):
            try:
                gimbal_status = gimbal_state()
            except Exception:
                gimbal_status = None
            self.lbl_gimbal.setText(self._format_gimbal_status(gimbal_status))
            if isinstance(gimbal_status, dict):
                zoom_value = gimbal_status.get("zoom_scale")
                if isinstance(zoom_value, (int, float)):
                    self._sync_zoom_from_status(float(zoom_value))
        relay_state = getattr(self.relay, "get_status", None)
        if callable(relay_state):
            try:
                status = relay_state()
            except Exception:
                status = None
            self.lbl_relay.setText(self._format_relay_status(status))
        rover_state = getattr(self.rover, "get_status", None)
        if callable(rover_state):
            try:
                rover_status = rover_state()
            except Exception:
                self.lbl_rover.setText("Rover Logging: Unknown")
            else:
                self.lbl_rover.setText(f"Rover Logging: {rover_status}")
        log_state = getattr(self.relay, "get_logging_status", None)
        if callable(log_state):
            try:
                status = log_state()
            except Exception:
                status = None
            self.lbl_relay_log.setText(self._format_logging_status(status))

    def _format_gimbal_status(self, status: Optional[Dict[str, Any]]) -> str:
        prefix = "Gimbal:"
        if not status:
            return f"{prefix} Unknown"
        if not isinstance(status, dict):
            return f"{prefix} {status}"

        active = bool(status.get("activated"))
        method = status.get("control_mode") or status.get("control_method")
        method_label = str(method).upper() if method else None
        state_text = "Active" if active else "Idle"
        if method_label:
            state_text = f"{state_text} ({method_label})"

        details: list[str] = []
        if status.get("control_method") == "tcp" and status.get("tcp_bind"):
            details.append(f"TCP {status['tcp_bind']}")
        elif status.get("serial_state"):
            details.append(f"Serial {status['serial_state']}")

        def _normalize_rpy_for_display(values: list[float]) -> list[float]:
            """Collapse Euler-angle aliases to a stable, readable form."""

            tol = 1e-3

            try:
                roll_deg, pitch_deg, yaw_deg = (float(v) for v in values)
            except Exception:
                return values

            roll_rad = math.radians(roll_deg)
            pitch_rad = math.radians(pitch_deg)
            yaw_rad = math.radians(yaw_deg)

            cy = math.cos(yaw_rad * 0.5)
            sy = math.sin(yaw_rad * 0.5)
            cp = math.cos(pitch_rad * 0.5)
            sp = math.sin(pitch_rad * 0.5)
            cr = math.cos(roll_rad * 0.5)
            sr = math.sin(roll_rad * 0.5)

            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy

            norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
            qw, qx, qy, qz = (component / norm for component in (qw, qx, qy, qz))

            sinr_cosp = 2.0 * (qw * qx + qy * qz)
            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

            sinp = 2.0 * (qw * qy - qz * qx)
            if abs(sinp) >= 1:
                pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
            else:
                pitch = math.degrees(math.asin(sinp))

            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

            wrapped = [((val + 180.0) % 360.0) - 180.0 for val in (roll, pitch, yaw)]
            normalized: list[float] = []
            for value in wrapped:
                if abs(value) < tol:
                    normalized.append(0.0)
                elif abs(abs(value) - 180.0) < tol:
                    normalized.append(0.0)
                else:
                    normalized.append(value)
            return normalized

        def _fmt_rpy(triple: object) -> Optional[str]:
            if not isinstance(triple, (list, tuple)) or len(triple) != 3:
                return None
            values: list[float] = []
            for value in triple:
                if not isinstance(value, (int, float)):
                    return None
                values.append(float(value))

            normalized = _normalize_rpy_for_display(values)
            parts = [f"{val:.1f}°" for val in normalized]
            return "/".join(parts)

        lh_rpy = _fmt_rpy(status.get("rpy_left_deg"))
        rh_rpy = _fmt_rpy(status.get("rpy_right_deg"))

        if lh_rpy:
            details.append(f"RPY(LH) {lh_rpy}")
        if rh_rpy:
            details.append(f"RPY(RH) {rh_rpy}")
        if not lh_rpy and not rh_rpy:
            roll = status.get("current_roll_deg")
            pitch = status.get("current_pitch_deg")
            yaw = status.get("current_yaw_deg")
            angles = []
            for value in (roll, pitch, yaw):
                if isinstance(value, (int, float)):
                    angles.append(f"{value:.1f}°")
            if len(angles) == 3:
                details.append(f"RPY {'/'.join(angles)}")

        zoom = status.get("zoom_scale")
        if isinstance(zoom, (int, float)):
            lens_mm = status.get("zoom_lens_mm")
            if not isinstance(lens_mm, (int, float)):
                lens_mm = zoom * BASE_ZOOM_LENS_MM
            details.append(f"Zoom x{zoom:.2f} ({lens_mm:.1f}mm)")

        if not details:
            return f"{prefix} {state_text}"
        return f"{prefix} {state_text} · {' · '.join(details)}"

    def _format_relay_status(self, status: Optional[Dict[str, Any]]) -> str:
        prefix = "Relay:"
        if not status:
            return f"{prefix} Unknown"
        if not isinstance(status, dict):
            return f"{prefix} {status}"

        active = bool(status.get("activated"))
        state_text = "Active" if active else "Stopped"

        details: list[str] = []

        listen = status.get("gazebo_listen")
        if listen:
            details.append(f"IN {listen}")
        ext = status.get("ext_dst")
        if ext:
            details.append(f"OUT {ext}")

        serial_connected = status.get("serial_connected")
        if serial_connected is not None:
            serial_label = "Serial OK" if serial_connected else "Serial OFF"
            details.append(serial_label)

        of_enabled = status.get("of_processing_enabled")
        if of_enabled is not None:
            details.append(f"OptFlow {'ON' if of_enabled else 'OFF'}")

        count = status.get("gazebo_forward_count")
        if isinstance(count, int) and count >= 0:
            details.append(f"Forwarded {count}")
        last_ts = status.get("gazebo_forward_last_ts")
        if isinstance(last_ts, (int, float)) and last_ts > 0:
            try:
                last_time = time.strftime("%H:%M:%S", time.localtime(last_ts))
            except Exception:
                last_time = None
            if last_time:
                details.append(f"Last {last_time}")

        distance = status.get("distance_m")
        if isinstance(distance, (int, float)) and distance >= 0:
            age = status.get("distance_age_s")
            if isinstance(age, (int, float)) and age >= 0:
                details.append(f"Distance {distance:.1f}m ({age:.0f}s ago)")
            else:
                details.append(f"Distance {distance:.1f}m")

        if not details:
            return f"{prefix} {state_text}"
        return f"{prefix} {state_text} · {' · '.join(details)}"

    def _format_logging_status(self, status: Optional[Dict[str, Any]]) -> str:
        prefix = "Gazebo Logging:"
        if not status:
            return f"{prefix} Unknown"
        if not isinstance(status, dict):
            return f"{prefix} {status}"

        enabled = bool(status.get("enable_gazebo_logging", True))
        active = bool(status.get("gazebo_logging_active"))
        blocked_reason = status.get("gazebo_log_block_reason")

        if not enabled:
            state_text = "Disabled"
        elif active:
            state_text = "Active"
        else:
            state_text = "Idle"

        details: list[str] = []

        if active:
            count = status.get("gazebo_logged_count")
            if isinstance(count, int):
                details.append(f"{count} entries")
            last_ts = status.get("gazebo_log_last_write_ts")
            if isinstance(last_ts, (int, float)) and last_ts > 0:
                try:
                    last_time = time.strftime("%H:%M:%S", time.localtime(last_ts))
                except Exception:
                    last_time = None
                if last_time:
                    details.append(f"Last {last_time}")
        if status.get("gazebo_log_path"):
            details.append(str(status["gazebo_log_path"]))
        if blocked_reason:
            details.append(f"Blocked: {blocked_reason}")

        if not details:
            return f"{prefix} {state_text}"
        return f"{prefix} {state_text} · {' · '.join(details)}"

    # ------------------------------------------------------------------
    # Button handlers
    # ------------------------------------------------------------------
    def _on_tcp_enable_toggled(self, checked: bool) -> None:
        desired = bool(checked)
        setter = getattr(self.bridge, "set_tcp_enabled", None)
        if not callable(setter):
            self.log.error("[UI] Bridge does not support TCP enable toggle")
            QtWidgets.QMessageBox.critical(self, "Error", "TCP/IP 연결 토글을 지원하지 않습니다.")
            if self.chk_tcp_enabled is not None:
                self.chk_tcp_enabled.blockSignals(True)
                self.chk_tcp_enabled.setChecked(not desired)
                self.chk_tcp_enabled.blockSignals(False)
            return

        try:
            result = bool(setter(desired))
        except Exception as exc:  # pragma: no cover - runtime guard
            self.log.error("[UI] Failed to toggle TCP listener: %s", exc)
            QtWidgets.QMessageBox.critical(self, "Error", f"TCP/IP 설정 변경 실패:\n{exc}")
            if self.chk_tcp_enabled is not None:
                self.chk_tcp_enabled.blockSignals(True)
                self.chk_tcp_enabled.setChecked(not desired)
                self.chk_tcp_enabled.blockSignals(False)
            return

        if result != desired:
            self.log.warning("[UI] TCP listener state mismatch: requested=%s got=%s", desired, result)
            if self.chk_tcp_enabled is not None:
                self.chk_tcp_enabled.blockSignals(True)
                self.chk_tcp_enabled.setChecked(result)
                self.chk_tcp_enabled.blockSignals(False)
            desired = result

        self.cfg.setdefault("bridge", {})["enable_tcp"] = desired
        self._refresh_status_labels()

    def on_disconnect_stream(self) -> None:
        if self._disconnect_in_progress:
            return
        self._disconnect_in_progress = True
        self.btn_server.setEnabled(False)

        def worker() -> tuple[Optional[Exception], bool]:
            disconnect = getattr(self.bridge, "disconnect_tcp_client", None)
            if not callable(disconnect):
                return RuntimeError("disconnect not supported"), False
            try:
                result = bool(disconnect(restart_listener=True))
            except Exception as exc:  # pragma: no cover - runtime guard
                return exc, False
            return None, result

        self._run_background(worker, self._disconnect_finalize)

    @QtCore.Slot(object)
    def _disconnect_finalize(self, payload: object) -> None:
        self._disconnect_in_progress = False
        error: Optional[Exception]
        disconnected = False
        if isinstance(payload, tuple) and len(payload) == 2:
            maybe_error, maybe_flag = payload
            error = maybe_error if isinstance(maybe_error, Exception) else None
            disconnected = bool(maybe_flag)
        else:
            error = payload if isinstance(payload, Exception) else None
        if error is not None:
            self.log.error("[UI] Disconnect image stream failed: %s", error)
            QtWidgets.QMessageBox.critical(self, "Error", f"Disconnect failed:\n{error}")
        elif not disconnected:
            self.log.info("[UI] No TCP client to disconnect.")
        self.btn_server.setEnabled(True)
        self._refresh_status_labels()

    def open_bridge_window(self) -> None:
        from .bridge_window import BridgeSettingsDialog

        dlg = BridgeSettingsDialog(self, self.cfg, self.bridge, self.log)
        dlg.exec()
        self._preview_min_interval = float(self.cfg.get("bridge", {}).get("preview_min_interval", 1.0))

    def open_gimbal_window(self) -> None:
        from .gimbal_window import GimbalControlsDialog

        dlg = GimbalControlsDialog(self, self.cfg, self.gimbal, self.log)
        if dlg.exec() == QtWidgets.QDialog.Accepted:
            self.cfg["gimbal"] = dlg.updated_config
            try:
                self.gimbal.update_settings(self.cfg.get("gimbal", {}))
            except Exception:
                pass

    def open_relay_window(self) -> None:
        from .relay_window import RelaySettingsDialog

        dlg = RelaySettingsDialog(self, self.cfg, self.relay, self.rover, self.log)
        dlg.exec()

    def open_rover_window(self) -> None:
        from .rover_relay_window import RoverRelaySettingsDialog

        dlg = RoverRelaySettingsDialog(self, self.cfg, self.rover, self.relay, self.log)
        dlg.exec()

    def _on_gimbal_method_changed(self) -> None:
        method = "tcp" if self.rb_tcp.isChecked() else "mavlink"
        try:
            self.gimbal.update_control_method(method)
        except Exception as exc:
            actual = getattr(self.gimbal, "control_method", "tcp")
            self.log.error("[UI] Gimbal control switch failed: %s", exc)
            QtWidgets.QMessageBox.warning(
                self,
                "Gimbal",
                f"제어 모드 전환에 실패했습니다.\n{exc}",
            )
            with QtCore.QSignalBlocker(self.rb_tcp), QtCore.QSignalBlocker(self.rb_mav):
                if actual == "mavlink":
                    self.rb_mav.setChecked(True)
                else:
                    self.rb_tcp.setChecked(True)
            self.cfg.setdefault("gimbal", {})["control_method"] = actual
            return
        self.cfg.setdefault("gimbal", {})["control_method"] = method
        self._refresh_status_labels()

    # ------------------------------------------------------------------
    # Qt overrides
    # ------------------------------------------------------------------
    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # pragma: no cover - GUI callback
        self._status_timer.stop()
        self._preview_timer.stop()
        if self._zoom_unsubscribe:
            try:
                self._zoom_unsubscribe()
            except Exception:
                pass
        self.log.removeHandler(self._log_handler)
        super().closeEvent(event)


def run_gui(
    cfg: Dict[str, Any],
    bridge,
    gimbal,
    relay,
    rover,
    log: logging.Logger,
    zoom_state: Optional[ObservableFloat] = None,
) -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MainWindow(cfg, bridge, gimbal, relay, rover, log, zoom_state=zoom_state)
    window.show()
    app.exec()
