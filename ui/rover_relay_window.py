"""PySide6 dialog for rover relay logging configuration."""
from __future__ import annotations

import logging
from typing import Any, Dict

from PySide6 import QtCore, QtGui, QtWidgets

from utils.settings import AppConfig, ConfigManager


class RoverRelaySettingsDialog(QtWidgets.QDialog):
    def __init__(self, parent: QtWidgets.QWidget, cfg: Dict[str, Any], rover, relay, log: logging.Logger) -> None:
        super().__init__(parent)
        self.setWindowTitle("Rover Relay Logging")
        self.setModal(True)
        self.resize(520, 360)

        self.cfg = cfg
        self.rover = rover
        self.relay = relay
        self.log = log

        rconf = cfg.get("rover", {})

        self.enabled = QtWidgets.QCheckBox("Enable rover relay logging")
        self.enabled.setChecked(bool(rconf.get("enabled", True)))
        self.autostart = QtWidgets.QCheckBox("Autostart on launch")
        self.autostart.setChecked(bool(rconf.get("autostart", False)))

        self.listen_ip = QtWidgets.QLineEdit(str(rconf.get("feedback_listen_ip", "0.0.0.0")))
        self.listen_port = QtWidgets.QSpinBox(); self.listen_port.setRange(1, 65535)
        self.listen_port.setValue(int(rconf.get("feedback_listen_port", 18102)))
        self.dest_ip = QtWidgets.QLineEdit(str(rconf.get("feedback_dest_ip", "127.0.0.1")))
        self.dest_port = QtWidgets.QSpinBox(); self.dest_port.setRange(1, 65535)
        self.dest_port.setValue(int(rconf.get("feedback_dest_port", 18103)))
        self.log_path = QtWidgets.QLineEdit(str(rconf.get("feedback_log_path", "")))

        self.status1 = QtWidgets.QLabel("-")
        self.status2 = QtWidgets.QLabel("-")

        self._build_layout()
        self._status_timer = QtCore.QTimer(self)
        self._status_timer.setInterval(1000)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start()
        self._refresh_status()

    def _build_layout(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)

        form = QtWidgets.QFormLayout()
        form.addRow("Listen IP", self.listen_ip)
        form.addRow("Listen Port", self.listen_port)
        form.addRow("Dest IP", self.dest_ip)
        form.addRow("Dest Port", self.dest_port)

        log_row = QtWidgets.QHBoxLayout()
        log_row.addWidget(self.log_path)
        browse_btn = QtWidgets.QPushButton("Browse")
        browse_btn.clicked.connect(self.on_browse)
        log_row.addWidget(browse_btn)
        form.addRow("Log Path", log_row)

        layout.addLayout(form)
        layout.addWidget(self.enabled)
        layout.addWidget(self.autostart)

        note = QtWidgets.QLabel("Gazebo logging must be stopped or disabled before starting rover logging.")
        note.setStyleSheet("color: #aa5500;")
        note.setWordWrap(True)
        layout.addWidget(note)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_apply = QtWidgets.QPushButton("Apply")
        self.btn_save = QtWidgets.QPushButton("Save")
        self.btn_close = QtWidgets.QPushButton("Close")
        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_stop)
        btn_row.addWidget(self.btn_apply)
        btn_row.addWidget(self.btn_save)
        btn_row.addWidget(self.btn_close)
        layout.addLayout(btn_row)

        layout.addWidget(self.status1)
        layout.addWidget(self.status2)

        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_apply.clicked.connect(self.on_apply)
        self.btn_save.clicked.connect(self.on_save)
        self.btn_close.clicked.connect(self.close)

    # ------------------------------------------------------------------
    def _collect_values(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled.isChecked(),
            "autostart": self.autostart.isChecked(),
            "feedback_listen_ip": self.listen_ip.text().strip(),
            "feedback_listen_port": int(self.listen_port.value()),
            "feedback_dest_ip": self.dest_ip.text().strip(),
            "feedback_dest_port": int(self.dest_port.value()),
            "feedback_log_path": self.log_path.text().strip(),
        }

    def _apply_to_runtime(self) -> None:
        values = self._collect_values()
        self.cfg.setdefault("rover", {}).update(values)
        for obsolete in (
            "cmd_listen_ip",
            "cmd_listen_port",
            "cmd_dest_ip",
            "cmd_dest_port",
            "cmd_log_path",
        ):
            self.cfg["rover"].pop(obsolete, None)
        self.rover.update_settings(values)

    # ------------------------------------------------------------------
    def on_browse(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Select feedback log file", "", "Text files (*.txt);;All files (*)")
        if path:
            self.log_path.setText(path)

    def on_start(self) -> None:
        try:
            self._apply_to_runtime()
            self.rover.start()
            QtWidgets.QMessageBox.information(self, "Rover Relay", "Rover relay logging started.")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Start failed:\n{exc}")

    def on_stop(self) -> None:
        try:
            self.rover.stop()
            QtWidgets.QMessageBox.information(self, "Rover Relay", "Rover relay logging stopped.")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Stop failed:\n{exc}")

    def on_apply(self) -> None:
        try:
            self._apply_to_runtime()
            QtWidgets.QMessageBox.information(self, "Rover Relay", "Settings applied.")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Apply failed:\n{exc}")

    def on_save(self) -> None:
        try:
            self._apply_to_runtime()
            cm = ConfigManager(); ac = cm.load(); data = ac.to_dict()
            data.setdefault("rover", {}).update(self._collect_values())
            cm.save(AppConfig.from_dict(data))
            QtWidgets.QMessageBox.information(self, "Saved", "Rover relay settings saved.")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Save failed:\n{exc}")

    def _refresh_status(self) -> None:
        try:
            st = self.rover.get_status() if hasattr(self.rover, "get_status") else {}
        except Exception:
            st = {}
        if isinstance(st, dict):
            self.status1.setText(st.get("status", "-"))
            detail = []
            if st.get("log_path"):
                detail.append(st.get("log_path"))
            if st.get("running") is not None:
                detail.append("Running" if st.get("running") else "Stopped")
            self.status2.setText(" | ".join(detail) or "-")
        else:
            self.status1.setText(str(st))
            self.status2.setText("-")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # pragma: no cover - GUI callback
        self._status_timer.stop()
        super().closeEvent(event)
