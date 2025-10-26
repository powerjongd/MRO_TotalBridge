"""PySide6 main window for MRO TotalBridge."""
from __future__ import annotations

import io
import logging
import threading
import time
from collections import deque
from typing import Any, Callable, Deque, Dict, Optional, Tuple

from PIL import Image
from PySide6 import QtCore, QtGui, QtWidgets

from utils.helpers import get_recent_log_lines
from utils.observers import ObservableFloat


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
    frame_ready = QtCore.Signal(bytes, float)


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

        self._preview_bridge = _PreviewBridge()
        self._preview_bridge.frame_ready.connect(self._handle_preview_frame)
        self._last_image: Optional[QtGui.QPixmap] = None
        self._preview_last_ts = 0.0
        self._preview_last_size = "-"
        self._preview_last_time = "-"
        self._preview_min_interval = float(self.cfg.get("bridge", {}).get("preview_min_interval", 1.0))
        self._preview_max_queue = 5
        self._preview_queue: Deque[Tuple[bytes, float]] = deque(maxlen=self._preview_max_queue)
        self._preview_lock = threading.Lock()
        self._current_zoom_value = float(self.cfg.get("gimbal", {}).get("zoom_scale", 1.0))
        self._preview_target_size = QtCore.QSize(640, 480)
        self._server_toggle_in_progress = False

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

        self.btn_server = QtWidgets.QPushButton("Start Image Stream Module")
        self.btn_server.clicked.connect(self.on_toggle_server)
        bridge_buttons.addWidget(self.btn_server)

        self.btn_bridge = QtWidgets.QPushButton("Image Stream Settings")
        self.btn_bridge.clicked.connect(self.open_bridge_window)
        bridge_buttons.addWidget(self.btn_bridge)
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

        self.lbl_gimbal = QtWidgets.QLabel("Gimbal: Deactivated")
        self.lbl_gimbal.setWordWrap(True)
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
        self.lbl_relay = QtWidgets.QLabel("Relay: Deactivated")
        self.lbl_relay.setWordWrap(True)
        relay_row.addWidget(self.lbl_relay)
        relay_layout.addLayout(relay_row)

        rover_row = QtWidgets.QHBoxLayout()
        rover_row.setContentsMargins(0, 0, 0, 0)
        rover_row.setSpacing(6)
        self.btn_rover = QtWidgets.QPushButton("Rover Relay Logging")
        self.btn_rover.clicked.connect(self.open_rover_window)
        rover_row.addWidget(self.btn_rover)
        rover_row.addStretch()
        self.lbl_rover = QtWidgets.QLabel("Rover Logging: Idle")
        self.lbl_rover.setWordWrap(True)
        rover_row.addWidget(self.lbl_rover)
        relay_layout.addLayout(rover_row)

        self.lbl_relay_log = QtWidgets.QLabel("Gazebo Logging: Idle")
        self.lbl_relay_log.setWordWrap(True)
        self.lbl_relay_log.setStyleSheet("color: #888888;")
        relay_layout.addWidget(self.lbl_relay_log)

        header_layout.addWidget(relay_box)

        self._layout.addWidget(header)

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

        self.lbl_zoom = QtWidgets.QLabel(f"Zoom: {self._current_zoom_value:.2f}x")
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
            self._current_zoom_value = value
            self.lbl_zoom.setText(f"Zoom: {value:.2f}x")

        self._zoom_unsubscribe = self.zoom_state.subscribe(_on_zoom)

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
    def on_preview(self, frame: bytes, timestamp: float) -> None:
        with self._preview_lock:
            self._preview_queue.append((frame, timestamp))

    def _drain_preview_queue(self) -> None:
        with self._preview_lock:
            if not self._preview_queue:
                return
            frame, ts = self._preview_queue.pop()
            self._preview_queue.clear()
        self._preview_bridge.frame_ready.emit(frame, ts)

    @QtCore.Slot(bytes, float)
    def _handle_preview_frame(self, frame: bytes, timestamp: float) -> None:
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
                qimg = QtGui.QImage(ptr, rgb.width, rgb.height, QtGui.QImage.Format_RGBA8888)
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
        btn_text = "Stop Image Stream Module" if running else "Start Image Stream Module"
        self.btn_server.setText(btn_text)

        status = getattr(self.bridge, "get_runtime_status", None)
        if callable(status):
            try:
                st = status()
            except Exception:
                st = {}
        else:
            st = {}
        mode = st.get("image_source_mode", "Realtime")
        tcp_on = "ON" if st.get("tcp_listening") else "OFF"
        udp_on = "ON" if st.get("udp_listening") else "OFF"
        mode_label = str(mode)
        stream_state = "Running" if running else "Stopped"
        self.lbl_bridge.setText(f"Image Stream: {stream_state} · {mode_label}")
        self.lbl_bridge_extra.setText(f"TCP {tcp_on} / UDP {udp_on}")

        gimbal_state = getattr(self.gimbal, "get_status", None)
        if callable(gimbal_state):
            try:
                self.lbl_gimbal.setText(f"Gimbal: {gimbal_state()}")
            except Exception:
                self.lbl_gimbal.setText("Gimbal: Unknown")
        relay_state = getattr(self.relay, "get_status", None)
        if callable(relay_state):
            try:
                self.lbl_relay.setText(f"Relay: {relay_state()}")
            except Exception:
                self.lbl_relay.setText("Relay: Unknown")
        rover_state = getattr(self.rover, "get_status", None)
        if callable(rover_state):
            try:
                self.lbl_rover.setText(f"Rover Logging: {rover_state()}")
            except Exception:
                self.lbl_rover.setText("Rover Logging: Unknown")
        log_state = getattr(self.relay, "get_logging_status", None)
        if callable(log_state):
            try:
                self.lbl_relay_log.setText(f"Gazebo Logging: {log_state()}")
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Button handlers
    # ------------------------------------------------------------------
    def on_toggle_server(self) -> None:
        if self._server_toggle_in_progress:
            return
        self._server_toggle_in_progress = True
        self.btn_server.setEnabled(False)

        def worker() -> None:
            error: Optional[Exception] = None
            try:
                running = bool(getattr(self.bridge, "is_server_running", None) and self.bridge.is_server_running.is_set())
                if running:
                    self.bridge.stop()
                else:
                    self.bridge.update_settings(self.cfg.get("bridge", {}))
                    self.bridge.start()
            except Exception as exc:  # pragma: no cover - runtime guard
                error = exc
            finally:
                QtCore.QMetaObject.invokeMethod(
                    self,
                    "_toggle_finalize",
                    QtCore.Qt.QueuedConnection,
                    QtCore.Q_ARG(object, error),
                )

        threading.Thread(target=worker, name="BridgeToggle", daemon=True).start()

    @QtCore.Slot(object)
    def _toggle_finalize(self, error: Optional[Exception]) -> None:
        self._server_toggle_in_progress = False
        self.btn_server.setEnabled(True)
        if error is not None:
            self.log.error("[UI] Toggle server failed: %s", error)
            QtWidgets.QMessageBox.critical(self, "Error", f"Toggle server failed:\n{error}")
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
        self.cfg.setdefault("gimbal", {})["control_method"] = method
        try:
            self.gimbal.update_control_method(method)
        except Exception:
            pass

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
