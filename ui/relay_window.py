"""PySide6 dialog for relay configuration."""
from __future__ import annotations

import logging
from typing import Any, Callable, Dict, Optional

from PySide6 import QtConcurrent, QtCore, QtGui, QtWidgets
from serial.tools import list_ports

from utils.settings import AppConfig, ConfigManager


class RelaySettingsDialog(QtWidgets.QDialog):
    def __init__(self, parent: QtWidgets.QWidget, cfg: Dict[str, Any], relay, rover, log: logging.Logger) -> None:
        super().__init__(parent)
        self.setWindowTitle("Gazebo Relay")
        self.resize(720, 820)
        self.setModal(True)

        self.cfg = cfg
        self.relay = relay
        self.rover = rover
        self.log = log

        rconf = cfg.get("relay", {})
        self.fields: Dict[str, QtWidgets.QWidget] = {}
        self._stop_in_progress = False
        self._background_watchers: list[QtCore.QFutureWatcher] = []

        self._build_layout(rconf)
        self._status_timer = QtCore.QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start()
        self._refresh_status()

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
    def _build_layout(self, rconf: Dict[str, Any]) -> None:
        layout = QtWidgets.QVBoxLayout(self)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        content = QtWidgets.QWidget()
        scroll.setWidget(content)
        form_layout = QtWidgets.QVBoxLayout(content)

        # Gazebo input
        gazebo_group = QtWidgets.QGroupBox("Gazebo Input (UDP)")
        gz_form = QtWidgets.QFormLayout(gazebo_group)
        self.fields["gazebo_listen_ip"] = QtWidgets.QLineEdit(str(rconf.get("gazebo_listen_ip", "0.0.0.0")))
        gz_form.addRow("Listen IP", self.fields["gazebo_listen_ip"])
        ip_port = QtWidgets.QSpinBox()
        ip_port.setRange(1, 65535)
        ip_port.setValue(int(rconf.get("gazebo_listen_port", 17000)))
        self.fields["gazebo_listen_port"] = ip_port
        gz_form.addRow("Port", ip_port)
        form_layout.addWidget(gazebo_group)

        # External relay
        ext_group = QtWidgets.QGroupBox("ExternalCtrl Output (UDP Relay)")
        ext_form = QtWidgets.QFormLayout(ext_group)
        self.fields["ext_udp_ip"] = QtWidgets.QLineEdit(str(rconf.get("ext_udp_ip", "127.0.0.1")))
        ext_form.addRow("Dest IP", self.fields["ext_udp_ip"])
        ext_port = QtWidgets.QSpinBox(); ext_port.setRange(1, 65535)
        ext_port.setValue(int(rconf.get("ext_udp_port", 9091)))
        self.fields["ext_udp_port"] = ext_port
        ext_form.addRow("Port", ext_port)
        form_layout.addWidget(ext_group)

        # Distance input
        dist_group = QtWidgets.QGroupBox("Distance Sensor Input")
        dist_form = QtWidgets.QFormLayout(dist_group)
        mode = QtWidgets.QComboBox()
        mode.addItems(["raw", "mavlink"])
        current_mode = str(rconf.get("distance_mode", "raw"))
        mode.setCurrentText(current_mode if current_mode in {"raw", "mavlink"} else "raw")
        self.fields["distance_mode"] = mode
        dist_form.addRow("Mode", mode)
        self.fields["distance_udp_listen_ip"] = QtWidgets.QLineEdit(str(rconf.get("distance_udp_listen_ip", "0.0.0.0")))
        dist_form.addRow("Listen IP", self.fields["distance_udp_listen_ip"])
        dist_port = QtWidgets.QSpinBox(); dist_port.setRange(1, 65535)
        dist_port.setValue(int(rconf.get("distance_udp_listen_port", 14650)))
        self.fields["distance_udp_listen_port"] = dist_port
        dist_form.addRow("Port", dist_port)
        form_layout.addWidget(dist_group)

        # Serial settings
        serial_group = QtWidgets.QGroupBox("Shared Serial (OpticalFlow + Distance Out)")
        serial_form = QtWidgets.QFormLayout(serial_group)
        ports = [port.device for port in list_ports.comports()]
        port_combo = QtWidgets.QComboBox()
        port_combo.addItems(ports)
        default_port = str(rconf.get("serial_port", ""))
        if default_port:
            idx = port_combo.findText(default_port)
            if idx >= 0:
                port_combo.setCurrentIndex(idx)
            else:
                port_combo.insertItem(0, default_port)
                port_combo.setCurrentIndex(0)
        self.fields["serial_port"] = port_combo
        serial_form.addRow("Port", port_combo)
        baud = QtWidgets.QSpinBox(); baud.setRange(1, 1_000_000)
        baud.setValue(int(rconf.get("serial_baud", 115200)))
        self.fields["serial_baud"] = baud
        serial_form.addRow("Baud", baud)
        flow_id = QtWidgets.QSpinBox(); flow_id.setRange(0, 255)
        flow_id.setValue(int(rconf.get("flow_sensor_id", 0)))
        self.fields["flow_sensor_id"] = flow_id
        serial_form.addRow("OpticalFlow Sensor ID", flow_id)
        flags_layout = QtWidgets.QHBoxLayout()
        enable_of_serial = QtWidgets.QCheckBox("Optical Flow → MAVLink")
        enable_of_serial.setChecked(bool(rconf.get("enable_optical_flow_serial", True)))
        self.fields["enable_optical_flow_serial"] = enable_of_serial
        flags_layout.addWidget(enable_of_serial)
        enable_dist_serial = QtWidgets.QCheckBox("Distance → MAVLink")
        enable_dist_serial.setChecked(bool(rconf.get("enable_distance_serial", True)))
        self.fields["enable_distance_serial"] = enable_dist_serial
        flags_layout.addWidget(enable_dist_serial)
        flags_layout.addStretch()
        serial_form.addRow(flags_layout)
        enable_of_processing = QtWidgets.QCheckBox("Optical Flow 계산 활성화")
        enable_of_processing.setChecked(bool(rconf.get("enable_optical_flow_processing", False)))
        self.fields["enable_optical_flow_processing"] = enable_of_processing
        serial_form.addRow(enable_of_processing)
        autostart = QtWidgets.QCheckBox("Auto-Start on Launch")
        autostart.setChecked(bool(rconf.get("autostart", False)))
        self.fields["autostart"] = autostart
        serial_form.addRow(autostart)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_refresh_ports = QtWidgets.QPushButton("Refresh Ports")
        self.btn_open_serial = QtWidgets.QPushButton("Open Serial Now")
        btn_row.addWidget(self.btn_refresh_ports)
        btn_row.addWidget(self.btn_open_serial)
        btn_row.addStretch()
        serial_form.addRow(btn_row)
        form_layout.addWidget(serial_group)

        # Optical flow parameters
        of_group = QtWidgets.QGroupBox("Optical Flow Parameters")
        of_form = QtWidgets.QFormLayout(of_group)
        of_scale = QtWidgets.QDoubleSpinBox(); of_scale.setRange(0.01, 10000.0); of_scale.setDecimals(3)
        of_scale.setValue(float(rconf.get("of_scale_pix", 100.0)))
        self.fields["of_scale_pix"] = of_scale
        of_form.addRow("Pixel Scale", of_scale)
        q_box = QtWidgets.QWidget(); q_layout = QtWidgets.QHBoxLayout(q_box); q_layout.setContentsMargins(0, 0, 0, 0)
        for key in ("q_base", "q_min", "q_max"):
            spin = QtWidgets.QSpinBox(); spin.setRange(0, 255)
            spin.setValue(int(rconf.get(key, 255 if key == "q_base" else 0 if key == "q_min" else 255)))
            self.fields[key] = spin
            q_layout.addWidget(spin)
        of_form.addRow("Q Base / Min / Max", q_box)
        accel_thr = QtWidgets.QDoubleSpinBox(); accel_thr.setRange(0.0, 1000.0); accel_thr.setDecimals(3)
        accel_thr.setValue(float(rconf.get("accel_thresh", 5.0)))
        self.fields["accel_thresh"] = accel_thr
        accel_penalty = QtWidgets.QDoubleSpinBox(); accel_penalty.setRange(0.0, 1000.0); accel_penalty.setDecimals(3)
        accel_penalty.setValue(float(rconf.get("accel_penalty", 20.0)))
        self.fields["accel_penalty"] = accel_penalty
        of_form.addRow("Accel Thr / Penalty", self._pair_widget(accel_thr, accel_penalty))
        gyro_thr = QtWidgets.QDoubleSpinBox(); gyro_thr.setRange(0.0, 1000.0); gyro_thr.setDecimals(3)
        gyro_thr.setValue(float(rconf.get("gyro_thresh", 2.0)))
        self.fields["gyro_thresh"] = gyro_thr
        gyro_penalty = QtWidgets.QDoubleSpinBox(); gyro_penalty.setRange(0.0, 1000.0); gyro_penalty.setDecimals(3)
        gyro_penalty.setValue(float(rconf.get("gyro_penalty", 30.0)))
        self.fields["gyro_penalty"] = gyro_penalty
        of_form.addRow("Gyro Thr / Penalty", self._pair_widget(gyro_thr, gyro_penalty))
        form_layout.addWidget(of_group)

        # Heartbeat (Optical flow)
        hb_of_group = QtWidgets.QGroupBox("Heartbeat (Optical Flow)")
        hb_of_form = QtWidgets.QFormLayout(hb_of_group)
        hb_rate = QtWidgets.QDoubleSpinBox(); hb_rate.setRange(0.01, 100.0); hb_rate.setDecimals(3)
        hb_rate.setValue(float(rconf.get("hb_of_rate_hz", 1.0)))
        self.fields["hb_of_rate_hz"] = hb_rate
        hb_of_form.addRow("Rate (Hz)", hb_rate)
        self.fields["hb_of_sysid"] = self._spin_box(rconf.get("hb_of_sysid", 42))
        self.fields["hb_of_compid"] = self._spin_box(rconf.get("hb_of_compid", 199))
        hb_of_form.addRow("Sys / Comp", self._pair_widget(self.fields["hb_of_sysid"], self.fields["hb_of_compid"]))
        self.fields["hb_of_type"] = self._spin_box(rconf.get("hb_of_type", 18))
        self.fields["hb_of_autopilot"] = self._spin_box(rconf.get("hb_of_autopilot", 8))
        hb_of_form.addRow("Type / Autopilot", self._pair_widget(self.fields["hb_of_type"], self.fields["hb_of_autopilot"]))
        self.fields["hb_of_base_mode"] = self._spin_box(rconf.get("hb_of_base_mode", 0))
        self.fields["hb_of_custom_mode"] = self._spin_box(rconf.get("hb_of_custom_mode", 0))
        self.fields["hb_of_system_status"] = self._spin_box(rconf.get("hb_of_system_status", 4))
        row_box = self._triple_widget(
            self.fields["hb_of_base_mode"],
            self.fields["hb_of_custom_mode"],
            self.fields["hb_of_system_status"],
        )
        hb_of_form.addRow("Base / Custom / Status", row_box)
        form_layout.addWidget(hb_of_group)

        # Heartbeat (Distance)
        hb_ds_group = QtWidgets.QGroupBox("Heartbeat (Distance Sensor)")
        hb_ds_form = QtWidgets.QFormLayout(hb_ds_group)
        ds_rate = QtWidgets.QDoubleSpinBox(); ds_rate.setRange(0.01, 100.0); ds_rate.setDecimals(3)
        ds_rate.setValue(float(rconf.get("hb_ds_rate_hz", 1.0)))
        self.fields["hb_ds_rate_hz"] = ds_rate
        hb_ds_form.addRow("Rate (Hz)", ds_rate)
        self.fields["hb_ds_sysid"] = self._spin_box(rconf.get("hb_ds_sysid", 43))
        self.fields["hb_ds_compid"] = self._spin_box(rconf.get("hb_ds_compid", 200))
        hb_ds_form.addRow("Sys / Comp", self._pair_widget(self.fields["hb_ds_sysid"], self.fields["hb_ds_compid"]))
        self.fields["hb_ds_type"] = self._spin_box(rconf.get("hb_ds_type", 18))
        self.fields["hb_ds_autopilot"] = self._spin_box(rconf.get("hb_ds_autopilot", 8))
        hb_ds_form.addRow("Type / Autopilot", self._pair_widget(self.fields["hb_ds_type"], self.fields["hb_ds_autopilot"]))
        self.fields["hb_ds_base_mode"] = self._spin_box(rconf.get("hb_ds_base_mode", 0))
        self.fields["hb_ds_custom_mode"] = self._spin_box(rconf.get("hb_ds_custom_mode", 0))
        self.fields["hb_ds_system_status"] = self._spin_box(rconf.get("hb_ds_system_status", 4))
        hb_ds_form.addRow(
            "Base / Custom / Status",
            self._triple_widget(
                self.fields["hb_ds_base_mode"],
                self.fields["hb_ds_custom_mode"],
                self.fields["hb_ds_system_status"],
            ),
        )
        form_layout.addWidget(hb_ds_group)

        # Logging section
        log_group = QtWidgets.QGroupBox("Gazebo Logging")
        log_form = QtWidgets.QFormLayout(log_group)
        self.log_path_edit = QtWidgets.QLineEdit(str(rconf.get("gazebo_log_path", "")))
        browse_btn = QtWidgets.QPushButton("Browse")
        browse_btn.clicked.connect(self.on_browse_log)
        path_row = QtWidgets.QHBoxLayout()
        path_row.addWidget(self.log_path_edit)
        path_row.addWidget(browse_btn)
        log_form.addRow("Log Path", path_row)
        self.enable_gz_log = QtWidgets.QCheckBox("Enable Gazebo Logging")
        self.enable_gz_log.setChecked(bool(rconf.get("enable_gazebo_logging", True)))
        log_form.addRow(self.enable_gz_log)
        form_layout.addWidget(log_group)

        form_layout.addStretch()
        layout.addWidget(scroll)

        # Status + Buttons
        self.status_label = QtWidgets.QLabel("Status: -")
        self.detail_label = QtWidgets.QLabel("-")
        self.logging_label = QtWidgets.QLabel("Logging: Unknown")
        layout.addWidget(self.status_label)
        layout.addWidget(self.detail_label)
        layout.addWidget(self.logging_label)

        btn_box = QtWidgets.QDialogButtonBox()
        self.btn_start = QtWidgets.QPushButton("Start Relay")
        self.btn_stop = QtWidgets.QPushButton("Stop Relay")
        self.btn_start_log = QtWidgets.QPushButton("Start Logging")
        self.btn_stop_log = QtWidgets.QPushButton("Stop Logging")
        btn_box.addButton(self.btn_start, QtWidgets.QDialogButtonBox.ActionRole)
        btn_box.addButton(self.btn_stop, QtWidgets.QDialogButtonBox.ActionRole)
        btn_box.addButton(self.btn_start_log, QtWidgets.QDialogButtonBox.ActionRole)
        btn_box.addButton(self.btn_stop_log, QtWidgets.QDialogButtonBox.ActionRole)
        save_btn = btn_box.addButton("Save", QtWidgets.QDialogButtonBox.ActionRole)
        btn_box.addButton(QtWidgets.QDialogButtonBox.Ok)
        btn_box.addButton(QtWidgets.QDialogButtonBox.Cancel)
        layout.addWidget(btn_box)

        self.btn_refresh_ports.clicked.connect(self.on_refresh_ports)
        self.btn_open_serial.clicked.connect(self.on_open_serial)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_start_log.clicked.connect(lambda: self._apply_logging(True))
        self.btn_stop_log.clicked.connect(lambda: self._apply_logging(False))
        self.enable_gz_log.toggled.connect(lambda state: self._apply_logging(state, announce=False))

        save_btn.clicked.connect(self.on_save)
        btn_box.accepted.connect(self.on_apply_close)
        btn_box.rejected.connect(self.reject)

    # ------------------------------------------------------------------
    def _pair_widget(self, left: QtWidgets.QWidget, right: QtWidgets.QWidget) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(left)
        layout.addWidget(right)
        return widget

    def _triple_widget(self, a: QtWidgets.QWidget, b: QtWidgets.QWidget, c: QtWidgets.QWidget) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(a)
        layout.addWidget(b)
        layout.addWidget(c)
        return widget

    def _spin_box(self, value: int) -> QtWidgets.QSpinBox:
        spin = QtWidgets.QSpinBox()
        spin.setRange(0, 65535)
        spin.setValue(int(value))
        return spin

    # ------------------------------------------------------------------
    def _collect_values(self) -> Dict[str, Any]:
        values: Dict[str, Any] = {}
        for key, widget in self.fields.items():
            if isinstance(widget, QtWidgets.QLineEdit):
                values[key] = widget.text().strip()
            elif isinstance(widget, QtWidgets.QComboBox):
                values[key] = widget.currentText()
            elif isinstance(widget, QtWidgets.QSpinBox):
                values[key] = int(widget.value())
            elif isinstance(widget, QtWidgets.QDoubleSpinBox):
                values[key] = float(widget.value())
            elif isinstance(widget, QtWidgets.QCheckBox):
                values[key] = widget.isChecked()
        values["gazebo_log_path"] = self.log_path_edit.text().strip()
        values["enable_gazebo_logging"] = self.enable_gz_log.isChecked()
        return values

    def _apply_to_runtime(self) -> None:
        values = self._collect_values()
        self.cfg.setdefault("relay", {}).update(values)
        self.relay.update_settings(values)

    def _refresh_status(self) -> None:
        try:
            st = self.relay.get_status() if hasattr(self.relay, "get_status") else {}
        except Exception:
            st = {}
        text = st.get("status", "-") if isinstance(st, dict) else str(st)
        self.status_label.setText(f"Status: {text}")
        detail = []
        if isinstance(st, dict):
            dist = st.get("distance_m")
            if dist is not None:
                detail.append(f"Distance: {dist}")
            q = st.get("of_quality")
            if q is not None:
                detail.append(f"OF Quality: {q}")
            serial = st.get("serial")
            if serial:
                detail.append(f"Serial: {serial}")
        self.detail_label.setText(" | ".join(detail) or "-")

        try:
            log_status = self.relay.get_logging_status() if hasattr(self.relay, "get_logging_status") else {}
        except Exception:
            log_status = {}
        if isinstance(log_status, dict):
            enabled = bool(log_status.get("enable_gazebo_logging", True))
            active = bool(log_status.get("gazebo_logging_active"))
            if not enabled:
                text = "Logging: Disabled"
            else:
                text = "Logging: ON" if active else "Logging: OFF"
            if log_status.get("gazebo_log_block_reason"):
                text = f"{text} (Blocked)"
        else:
            text = "Logging: Unknown"
        self.logging_label.setText(text)

    # ------------------------------------------------------------------
    def on_browse_log(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Select Gazebo log file", "", "Text files (*.txt);;All files (*)")
        if path:
            self.log_path_edit.setText(path)

    def on_refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        combo = self.fields.get("serial_port")
        if isinstance(combo, QtWidgets.QComboBox):
            combo.clear()
            combo.addItems(ports)

    def on_open_serial(self) -> None:
        try:
            self._apply_to_runtime()
            if hasattr(self.relay, "_open_serial_shared"):
                self.relay._open_serial_shared(force=True)
            QtWidgets.QMessageBox.information(
                self,
                "Serial",
                f"Serial open attempted on {self.fields['serial_port'].currentText()} @ {self.fields['serial_baud'].value()}",
            )
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Open serial failed:\n{exc}")

    def _apply_logging(self, enable: bool, announce: bool = True) -> None:
        if enable and not self.log_path_edit.text().strip():
            QtWidgets.QMessageBox.critical(self, "Error", "로그 파일 경로를 먼저 지정하세요.")
            self.enable_gz_log.setChecked(False)
            return
        self.enable_gz_log.setChecked(enable)
        try:
            self._apply_to_runtime()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"로깅 상태 변경 실패:\n{exc}")
            return
        if announce:
            QtWidgets.QMessageBox.information(self, "Gazebo 로깅", "로그 기록을 시작했습니다." if enable else "로그 기록을 중지했습니다.")

    def on_start(self) -> None:
        try:
            self._apply_to_runtime()
            self.relay.start()
            QtWidgets.QMessageBox.information(self, "Relay", "Relay started.")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Start failed:\n{exc}")

    def on_stop(self) -> None:
        if self._stop_in_progress:
            return
        self._stop_in_progress = True
        self.btn_stop.setEnabled(False)
        self.btn_start.setEnabled(False)

        def worker() -> Optional[Exception]:
            error: Optional[Exception] = None
            try:
                self.relay.stop()
            except Exception as exc:
                error = exc
            return error

        self._run_background(worker, self._on_stop_finished)

    @QtCore.Slot(object)
    def _on_stop_finished(self, error: Optional[Exception]) -> None:
        self._stop_in_progress = False
        self.btn_stop.setEnabled(True)
        self.btn_start.setEnabled(True)
        if error is not None:
            QtWidgets.QMessageBox.critical(self, "Error", f"Stop failed:\n{error}")
        else:
            QtWidgets.QMessageBox.information(self, "Relay", "Relay stopped.")
        self._refresh_status()

    def on_save(self) -> None:
        try:
            self._apply_to_runtime()
            cm = ConfigManager(); ac = cm.load(); data = ac.to_dict()
            data.setdefault("relay", {}).update(self._collect_values())
            cm.save(AppConfig.from_dict(data))
            QtWidgets.QMessageBox.information(self, "Saved", "Relay settings saved.")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Save failed:\n{exc}")

    def on_apply_close(self) -> None:
        try:
            self._apply_to_runtime()
            self.accept()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Apply failed:\n{exc}")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # pragma: no cover - GUI callback
        self._status_timer.stop()
        super().closeEvent(event)
