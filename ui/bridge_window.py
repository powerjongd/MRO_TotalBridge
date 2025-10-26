"""PySide6 dialog for configuring the image stream bridge."""
from __future__ import annotations

import logging
import os
from typing import Dict, Optional, Tuple

from PySide6 import QtWidgets

from utils.settings import AppConfig, ConfigManager

SENSOR_TYPES = ["Camera", "GPS", "LiDAR", "RADAR", "LRF", "IMU"]
SENSOR_COMBO_VALUES = [f"{idx}: {name}" for idx, name in enumerate(SENSOR_TYPES)]


class BridgeSettingsDialog(QtWidgets.QDialog):
    def __init__(
        self,
        parent: QtWidgets.QWidget,
        cfg: Dict[str, Dict],
        bridge,
        log: logging.Logger,
        *,
        gimbal: Optional[object] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Image Stream Module Settings")
        self.setModal(True)
        self.resize(520, 420)

        self.cfg = cfg
        self.bridge = bridge
        self.gimbal = gimbal
        self.log = log

        bconf = cfg.get("bridge", {})
        self.ip_edit = QtWidgets.QLineEdit(str(bconf.get("ip", "0.0.0.0")))
        self.tcp_edit = QtWidgets.QSpinBox()
        self.tcp_edit.setRange(1, 65535)
        self.tcp_edit.setValue(int(bconf.get("tcp_port", 9999)))
        self.udp_edit = QtWidgets.QSpinBox()
        self.udp_edit.setRange(1, 65535)
        self.udp_edit.setValue(int(bconf.get("udp_port", 9998)))

        self.mode_group = QtWidgets.QButtonGroup(self)
        self.radio_realtime = QtWidgets.QRadioButton("Use Realtime ImageSet (SaveFile)")
        self.radio_predefined = QtWidgets.QRadioButton("Use PreDefined ImageSet")
        mode = (bconf.get("image_source_mode") or "realtime").lower()
        if mode == "predefined":
            self.radio_predefined.setChecked(True)
        else:
            self.radio_realtime.setChecked(True)
        self.mode_group.addButton(self.radio_realtime)
        self.mode_group.addButton(self.radio_predefined)

        realtime_default = bconf.get("realtime_dir", bconf.get("images", "./SaveFile"))
        predefined_default = bconf.get("predefined_dir", "./PreDefinedImageSet")
        self.realtime_dir = QtWidgets.QLineEdit(realtime_default)
        self.predefined_dir = QtWidgets.QLineEdit(predefined_default)

        self.sensor_type = QtWidgets.QComboBox()
        self.sensor_type.addItems(SENSOR_COMBO_VALUES)
        self.sensor_type.setCurrentIndex(int(bconf.get("gimbal_sensor_type", 0)))
        self.sensor_id = QtWidgets.QSpinBox()
        self.sensor_id.setRange(0, 255)
        self.sensor_id.setValue(int(bconf.get("gimbal_sensor_id", 0)))

        forward_ip, forward_port = self._resolve_gimbal_endpoint()
        self.forward_ip = QtWidgets.QLineEdit(forward_ip)
        self.forward_ip.setReadOnly(True)
        self.forward_port = QtWidgets.QSpinBox()
        self.forward_port.setRange(1, 65535)
        self.forward_port.setValue(forward_port)
        self.forward_port.setReadOnly(True)

        self.status_label = QtWidgets.QLabel("Status: -")

        self._build_layout()
        self._refresh_status()

        self.mode_group.buttonToggled.connect(lambda *_: self._refresh_status())

    # ------------------------------------------------------------------
    def _build_layout(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        form = QtWidgets.QFormLayout()
        form.addRow("IP", self.ip_edit)
        form.addRow("TCP Port", self.tcp_edit)
        form.addRow("UDP Port", self.udp_edit)

        gimbal_box = QtWidgets.QGroupBox("Gimbal Forwarding")
        gimbal_form = QtWidgets.QFormLayout(gimbal_box)
        gimbal_form.addRow("Sensor Type", self.sensor_type)
        gimbal_form.addRow("Sensor ID", self.sensor_id)
        gimbal_form.addRow("Target IP", self.forward_ip)
        gimbal_form.addRow("Target Port", self.forward_port)

        mode_box = QtWidgets.QGroupBox("Image Library Source")
        mode_layout = QtWidgets.QVBoxLayout(mode_box)
        mode_layout.addWidget(self.radio_realtime)
        mode_layout.addWidget(self.radio_predefined)

        dirs_form = QtWidgets.QFormLayout()
        dirs_form.addRow("SaveFile Dir", self.realtime_dir)
        dirs_form.addRow("PreDefined Dir", self.predefined_dir)

        layout.addLayout(form)
        layout.addWidget(gimbal_box)
        layout.addWidget(mode_box)
        layout.addLayout(dirs_form)
        layout.addWidget(self.status_label)

        btn_box = QtWidgets.QDialogButtonBox()
        self.btn_save = btn_box.addButton("Save", QtWidgets.QDialogButtonBox.ActionRole)
        btn_box.addButton(QtWidgets.QDialogButtonBox.Ok)
        btn_box.addButton(QtWidgets.QDialogButtonBox.Cancel)
        layout.addWidget(btn_box)

        self.btn_save.clicked.connect(self.on_save)
        btn_box.accepted.connect(self.on_apply_close)
        btn_box.rejected.connect(self.reject)

    # ------------------------------------------------------------------
    def _collect_values(self) -> Dict[str, object]:
        ip = self.ip_edit.text().strip()
        if not ip:
            raise ValueError("IP must not be empty.")
        tcp = int(self.tcp_edit.value())
        udp = int(self.udp_edit.value())
        mode = "predefined" if self.radio_predefined.isChecked() else "realtime"
        realtime_dir = self.realtime_dir.text().strip() or "./SaveFile"
        predefined_dir = self.predefined_dir.text().strip() or "./PreDefinedImageSet"
        for path in (realtime_dir, predefined_dir):
            if not os.path.isdir(path):
                os.makedirs(path, exist_ok=True)
        values: Dict[str, object] = {
            "ip": ip,
            "tcp_port": tcp,
            "udp_port": udp,
            "image_source_mode": mode,
            "realtime_dir": realtime_dir,
            "predefined_dir": predefined_dir,
            "images": realtime_dir,
            "gimbal_sensor_type": int(self.sensor_type.currentIndex()),
            "gimbal_sensor_id": int(self.sensor_id.value()),
        }
        return values

    def _apply_to_runtime(self) -> None:
        values = self._collect_values()
        self.cfg.setdefault("bridge", {}).update(values)
        try:
            self.bridge.update_settings(self.cfg["bridge"])
            self.bridge.configure_gimbal_forwarding(
                values["gimbal_sensor_type"], values["gimbal_sensor_id"]
            )
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Apply failed:\n{exc}")
        self._refresh_status()

    def _resolve_gimbal_endpoint(self) -> Tuple[str, int]:
        if self.gimbal is not None:
            try:
                return self.gimbal.get_generator_endpoint()  # type: ignore[attr-defined]
            except Exception:
                pass
        gimbal_cfg = self.cfg.get("gimbal", {})
        ip = str(gimbal_cfg.get("generator_ip", "127.0.0.1"))
        port = int(gimbal_cfg.get("generator_port", 15020))
        return ip, port

    def _refresh_status(self) -> None:
        running = bool(getattr(self.bridge, "is_server_running", None) and self.bridge.is_server_running.is_set())
        mode = "predefined" if self.radio_predefined.isChecked() else "realtime"
        active_dir = self.predefined_dir.text().strip() if mode == "predefined" else self.realtime_dir.text().strip()
        self.status_label.setText(
            f"Status: {'Running' if running else 'Stopped'} | {self.ip_edit.text()} / "
            f"TCP:{self.tcp_edit.value()} UDP:{self.udp_edit.value()} | Mode: {mode.capitalize()} ({active_dir})"
        )
        ip, port = self._resolve_gimbal_endpoint()
        self.forward_ip.setText(ip)
        self.forward_port.setValue(port)

    # ------------------------------------------------------------------
    def on_save(self) -> None:
        try:
            values = self._collect_values()
            self.cfg.setdefault("bridge", {}).update(values)
            cm = ConfigManager()
            current = cm.load().to_dict()
            current.setdefault("bridge", {}).update(values)
            cm.save(AppConfig.from_dict(current))
            QtWidgets.QMessageBox.information(self, "Saved", "Image Stream Module settings saved.")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Save failed:\n{exc}")

    def on_apply_close(self) -> None:
        try:
            self._apply_to_runtime()
            self.accept()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Apply failed:\n{exc}")
