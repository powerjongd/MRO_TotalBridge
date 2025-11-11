"""PySide6 dialog for gimbal control configuration."""
from __future__ import annotations

from dataclasses import dataclass, field
import logging
from typing import Any, Dict, List, Optional, Tuple

from PySide6 import QtWidgets
from serial.tools import list_ports

# ❌ RPY 리맵핑 함수 임포트 제거
# from unified_bridge.support.helpers import remap_input_rpy
from unified_bridge.support.settings import AppConfig, ConfigManager


SENSOR_TYPES = ["Camera", "GPS", "LiDAR", "RADAR", "LRF", "IMU"]
SENSOR_COMBO_VALUES = [f"{idx}: {name}" for idx, name in enumerate(SENSOR_TYPES)]
MAX_SENSOR_PRESETS = 6
PRESET_VALUE_KEYS = [
    "sensor_type",
    "sensor_id",
    "pos_x",
    "pos_y",
    "pos_z",
    "init_roll_deg",
    "init_pitch_deg",
    "init_yaw_deg",
    "max_rate_dps",
    "power_on",
    "serial_port",
    "serial_baud",
    "mav_sysid",
    "mav_compid",
]


def _as_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _as_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "on", "y"}:
            return True
        if lowered in {"0", "false", "no", "off", "n", ""}:
            return False
    return default


def _as_str(value: Any, default: str = "") -> str:
    if value is None:
        return default
    return str(value)


@dataclass
class NetworkSettings:
    ip: str = "127.0.0.1"
    port: int = 15020

    @classmethod
    def from_config(cls, gimbal_cfg: Dict[str, Any]) -> "NetworkSettings":
        network = gimbal_cfg.get("network") if isinstance(gimbal_cfg.get("network"), dict) else {}
        ip = _as_str(network.get("ip", gimbal_cfg.get("generator_ip", "127.0.0.1")))
        port_src = network.get("port", gimbal_cfg.get("generator_port", 15020))
        return cls(ip=ip, port=_as_int(port_src, 15020))

    def to_config(self) -> Dict[str, Any]:
        return {"ip": self.ip, "port": self.port}


@dataclass
class SensorPresetData:
    sensor_type: int = 0
    sensor_id: int = 0
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0
    init_roll_deg: float = 0.0
    init_pitch_deg: float = 0.0
    init_yaw_deg: float = 0.0
    max_rate_dps: float = 60.0
    power_on: bool = True
    serial_port: str = ""
    serial_baud: int = 115200
    mav_sysid: int = 1
    mav_compid: int = 154

    @classmethod
    def from_dict(cls, raw: Optional[Dict[str, Any]]) -> "SensorPresetData":
        if not isinstance(raw, dict):
            return cls()
        return cls(
            sensor_type=_as_int(raw.get("sensor_type", 0), 0) & 0xFF,
            sensor_id=_as_int(raw.get("sensor_id", 0), 0) & 0xFF,
            pos_x=_as_float(raw.get("pos_x", 0.0), 0.0),
            pos_y=_as_float(raw.get("pos_y", 0.0), 0.0),
            pos_z=_as_float(raw.get("pos_z", 0.0), 0.0),
            init_roll_deg=_as_float(raw.get("init_roll_deg", 0.0), 0.0),
            init_pitch_deg=_as_float(raw.get("init_pitch_deg", 0.0), 0.0),
            init_yaw_deg=_as_float(raw.get("init_yaw_deg", 0.0), 0.0),
            max_rate_dps=_as_float(raw.get("max_rate_dps", 60.0), 60.0),
            power_on=_as_bool(raw.get("power_on", True), True),
            serial_port=_as_str(raw.get("serial_port", "")),
            serial_baud=_as_int(raw.get("serial_baud", 115200), 115200),
            mav_sysid=_as_int(raw.get("mav_sysid", 1), 1),
            mav_compid=_as_int(raw.get("mav_compid", 154), 154),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "sensor_type": self.sensor_type & 0xFF,
            "sensor_id": self.sensor_id & 0xFF,
            "pos_x": self.pos_x,
            "pos_y": self.pos_y,
            "pos_z": self.pos_z,
            "init_roll_deg": self.init_roll_deg,
            "init_pitch_deg": self.init_pitch_deg,
            "init_yaw_deg": self.init_yaw_deg,
            "max_rate_dps": self.max_rate_dps,
            "power_on": bool(self.power_on),
            "serial_port": self.serial_port,
            "serial_baud": self.serial_baud,
            "mav_sysid": self.mav_sysid,
            "mav_compid": self.mav_compid,
        }


@dataclass
class SensorPreset:
    name: str = ""
    data: SensorPresetData = field(default_factory=SensorPresetData)

    @classmethod
    def from_any(cls, raw: Any) -> Optional["SensorPreset"]:
        if not isinstance(raw, dict):
            return None
        name = _as_str(raw.get("name", "")).strip()
        payload_source: Any
        if isinstance(raw.get("values"), dict):
            payload_source = raw.get("values")
        else:
            payload_source = {k: raw.get(k) for k in PRESET_VALUE_KEYS if k in raw}
        if not payload_source:
            return None
        data = SensorPresetData.from_dict(payload_source)
        return cls(name=name, data=data)

    def to_config(self) -> Dict[str, Any]:
        return {"name": self.name, "values": self.data.to_dict()}

    def label(self, idx: int) -> str:
        base = f"Preset {idx + 1}"
        return self.name.strip() or base


@dataclass
class PresetBundle:
    network: NetworkSettings
    presets: List[Optional[SensorPreset]] = field(default_factory=list)
    selected_index: int = 0
    applicable_index: int = 0

    @classmethod
    def from_config(cls, gimbal_cfg: Dict[str, Any]) -> "PresetBundle":
        network = NetworkSettings.from_config(gimbal_cfg)
        presets: List[Optional[SensorPreset]] = []
        raw_bundle = gimbal_cfg.get("preset_bundle") if isinstance(gimbal_cfg.get("preset_bundle"), dict) else None
        raw_slots: Optional[Any] = None
        selected = _as_int(gimbal_cfg.get("selected_preset", 0), 0)
        applicable = _as_int(gimbal_cfg.get("mavlink_preset_index", selected), selected)
        if raw_bundle:
            raw_slots = raw_bundle.get("presets") or raw_bundle.get("slots")
            selected = _as_int(raw_bundle.get("selected", raw_bundle.get("selected_preset", selected)), selected)
            applicable = _as_int(
                raw_bundle.get("applicable", raw_bundle.get("applicable_preset", raw_bundle.get("mavlink_preset_index", applicable))),
                applicable,
            )
        if raw_slots is None:
            legacy = gimbal_cfg.get("presets")
            if isinstance(legacy, dict):
                raw_slots = legacy.get("slots")
                selected = _as_int(legacy.get("selected", selected), selected)
        if isinstance(raw_slots, list):
            for item in raw_slots:
                presets.append(SensorPreset.from_any(item))
        while len(presets) < MAX_SENSOR_PRESETS:
            presets.append(None)
        selected = max(0, min(selected, MAX_SENSOR_PRESETS - 1))
        applicable = max(0, min(applicable, MAX_SENSOR_PRESETS - 1))
        return cls(network=network, presets=presets, selected_index=selected, applicable_index=applicable)

    def to_config(self) -> Dict[str, Any]:
        return {
            "network": self.network.to_config(),
            "presets": [preset.to_config() if preset else None for preset in self.presets],
            "selected": self.selected_index,
            "applicable": self.applicable_index,
        }


class GimbalControlsDialog(QtWidgets.QDialog):
    def __init__(
        self,
        parent: QtWidgets.QWidget,
        cfg: Dict[str, Any],
        gimbal,
        log: logging.Logger,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Gimbal Controls")
        self.resize(820, 720)
        self.setModal(True)

        self.cfg = cfg
        self.gimbal = gimbal
        self.log = log

        gimbal_cfg = cfg.setdefault("gimbal", {})
        self.bundle = PresetBundle.from_config(gimbal_cfg)
        payload = self.bundle.presets[self.bundle.selected_index].data if self.bundle.presets[self.bundle.selected_index] else SensorPresetData.from_dict(gimbal_cfg)

        self.updated_config: Dict[str, Any] = dict(gimbal_cfg)

        self.network_ip = QtWidgets.QLineEdit(self.bundle.network.ip)
        self.network_port = QtWidgets.QSpinBox()
        self.network_port.setRange(1, 65535)
        self.network_port.setValue(self.bundle.network.port)

        self.preset_buttons: List[QtWidgets.QRadioButton] = []
        self.preset_group = QtWidgets.QButtonGroup(self)
        self.preset_name = QtWidgets.QLineEdit()
        self.btn_save_preset = QtWidgets.QPushButton("Save to Preset")
        self.btn_clear_preset = QtWidgets.QPushButton("Clear Preset")
        self.btn_apply_selected = QtWidgets.QPushButton("Apply Selected Preset")
        self.btn_apply_all = QtWidgets.QPushButton("Apply All Presets")

        self.sensor_type = QtWidgets.QComboBox()
        self.sensor_type.addItems(SENSOR_COMBO_VALUES)
        self.sensor_type.setCurrentIndex(payload.sensor_type)
        self.sensor_id = QtWidgets.QSpinBox()
        self.sensor_id.setRange(0, 255)
        self.sensor_id.setValue(payload.sensor_id)

        def _mk_spin(value: float, minimum: float = -1000.0, maximum: float = 1000.0, step: float = 0.1) -> QtWidgets.QDoubleSpinBox:
            w = QtWidgets.QDoubleSpinBox()
            w.setDecimals(3)
            w.setRange(minimum, maximum)
            w.setSingleStep(step)
            w.setValue(value)
            return w

        self.pos_x = _mk_spin(payload.pos_x)
        self.pos_y = _mk_spin(payload.pos_y)
        self.pos_z = _mk_spin(payload.pos_z)
        self.roll = _mk_spin(payload.init_roll_deg, -180.0, 180.0)
        self.pitch = _mk_spin(payload.init_pitch_deg, -180.0, 180.0)
        self.yaw = _mk_spin(payload.init_yaw_deg, -180.0, 180.0)
        self.max_rate = _mk_spin(payload.max_rate_dps, 0.0, 720.0, 1.0)
        self._power_on_flag = bool(payload.power_on)
        self.btn_power_on = QtWidgets.QPushButton("Power On")
        self.btn_power_off = QtWidgets.QPushButton("Power Off")

        self.serial_port = QtWidgets.QComboBox()
        self._refresh_serial_ports(default=payload.serial_port)
        self.serial_baud = QtWidgets.QSpinBox()
        self.serial_baud.setRange(1, 1_000_000)
        self.serial_baud.setValue(payload.serial_baud)
        self.btn_refresh_ports = QtWidgets.QPushButton("Refresh Ports")
        self.btn_open_serial = QtWidgets.QPushButton("Connect Serial")

        self.mav_sysid = QtWidgets.QSpinBox()
        self.mav_sysid.setRange(1, 255)
        self.mav_sysid.setValue(payload.mav_sysid)
        self.mav_compid = QtWidgets.QSpinBox()
        self.mav_compid.setRange(1, 255)
        self.mav_compid.setValue(payload.mav_compid)
        self.btn_apply_ids = QtWidgets.QPushButton("Apply IDs")

        self.applicable_combo = QtWidgets.QComboBox()
        self._refresh_applicable_combo()
        self.applicable_combo.setCurrentIndex(self.bundle.applicable_index)

        self.status_label = QtWidgets.QLabel("Status: -")

        self._build_layout()
        self._connect_signals()
        self._refresh_preset_buttons()
        self._update_status()

    # ------------------------------------------------------------------
    def _build_layout(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)

        net_group = QtWidgets.QGroupBox("Network Sensor Control")
        net_form = QtWidgets.QFormLayout(net_group)
        net_form.addRow("IP", self.network_ip)
        net_form.addRow("Port", self.network_port)
        layout.addWidget(net_group)

        preset_group = QtWidgets.QGroupBox("Sensor Presets")
        preset_layout = QtWidgets.QVBoxLayout(preset_group)
        preset_radio_layout = QtWidgets.QGridLayout()
        for idx in range(MAX_SENSOR_PRESETS):
            btn = QtWidgets.QRadioButton(f"Preset {idx + 1}")
            self.preset_buttons.append(btn)
            self.preset_group.addButton(btn, idx)
            preset_radio_layout.addWidget(btn, idx // 3, idx % 3)
        preset_layout.addLayout(preset_radio_layout)
        preset_form = QtWidgets.QFormLayout()
        preset_form.addRow("Name", self.preset_name)
        preset_layout.addLayout(preset_form)
        preset_btn_row = QtWidgets.QHBoxLayout()
        preset_btn_row.addWidget(self.btn_save_preset)
        preset_btn_row.addWidget(self.btn_clear_preset)
        preset_btn_row.addStretch()
        preset_layout.addLayout(preset_btn_row)
        apply_row = QtWidgets.QHBoxLayout()
        apply_row.addStretch()
        apply_row.addWidget(self.btn_apply_selected)
        apply_row.addWidget(self.btn_apply_all)
        preset_layout.addLayout(apply_row)
        layout.addWidget(preset_group)

        sensor_form = QtWidgets.QFormLayout()
        sensor_form.addRow("Sensor", self.sensor_type)
        sensor_form.addRow("ID", self.sensor_id)

        pose_grid = QtWidgets.QGridLayout()
        pose_grid.addWidget(QtWidgets.QLabel("x"), 0, 0)
        pose_grid.addWidget(self.pos_x, 0, 1)
        pose_grid.addWidget(QtWidgets.QLabel("y"), 0, 2)
        pose_grid.addWidget(self.pos_y, 0, 3)
        pose_grid.addWidget(QtWidgets.QLabel("z"), 0, 4)
        pose_grid.addWidget(self.pos_z, 0, 5)
        pose_grid.addWidget(QtWidgets.QLabel("Roll"), 1, 0)
        pose_grid.addWidget(self.roll, 1, 1)
        pose_grid.addWidget(QtWidgets.QLabel("Pitch"), 1, 2)
        pose_grid.addWidget(self.pitch, 1, 3)
        pose_grid.addWidget(QtWidgets.QLabel("Yaw"), 1, 4)
        pose_grid.addWidget(self.yaw, 1, 5)

        layout.addLayout(sensor_form)
        layout.addLayout(pose_grid)

        rate_layout = QtWidgets.QHBoxLayout()
        rate_layout.addWidget(QtWidgets.QLabel("Max Angular Rate (deg/s)"))
        rate_layout.addWidget(self.max_rate)
        rate_layout.addStretch()
        layout.addLayout(rate_layout)

        power_layout = QtWidgets.QHBoxLayout()
        power_layout.addWidget(self.btn_power_on)
        power_layout.addWidget(self.btn_power_off)
        power_layout.addStretch()
        layout.addLayout(power_layout)

        serial_group = QtWidgets.QGroupBox("MAVLink Serial")
        serial_form = QtWidgets.QFormLayout(serial_group)
        serial_form.addRow("Applicable Preset", self.applicable_combo)
        serial_form.addRow("Port", self.serial_port)
        serial_form.addRow("Baud", self.serial_baud)
        serial_form.addRow("System ID", self.mav_sysid)
        serial_form.addRow("Component ID", self.mav_compid)
        serial_btns = QtWidgets.QHBoxLayout()
        serial_btns.addWidget(self.btn_refresh_ports)
        serial_btns.addWidget(self.btn_open_serial)
        serial_btns.addWidget(self.btn_apply_ids)
        serial_form.addRow(serial_btns)
        layout.addWidget(serial_group)

        layout.addWidget(self.status_label)

        btn_box = QtWidgets.QDialogButtonBox()
        save_button = btn_box.addButton("Save", QtWidgets.QDialogButtonBox.ActionRole)
        btn_box.addButton(QtWidgets.QDialogButtonBox.Ok)
        btn_box.addButton(QtWidgets.QDialogButtonBox.Cancel)
        layout.addWidget(btn_box)

        save_button.clicked.connect(self.on_save)
        btn_box.accepted.connect(self.on_apply_close)
        btn_box.rejected.connect(self.reject)

    # ------------------------------------------------------------------
    def _connect_signals(self) -> None:
        self.preset_group.idToggled.connect(self._on_preset_selected)
        self.btn_save_preset.clicked.connect(self.on_save_preset)
        self.btn_clear_preset.clicked.connect(self.on_clear_preset)
        self.btn_apply_selected.clicked.connect(self.on_apply_selected_preset)
        self.btn_apply_all.clicked.connect(self.on_apply_all_presets)
        self.btn_refresh_ports.clicked.connect(self.on_refresh_ports)
        self.btn_open_serial.clicked.connect(self.on_connect_serial)
        self.btn_apply_ids.clicked.connect(self.on_apply_ids)
        self.applicable_combo.currentIndexChanged.connect(self.on_applicable_changed)
        self.btn_power_on.clicked.connect(self.on_power_on_clicked)
        self.btn_power_off.clicked.connect(self.on_power_off_clicked)

    # ------------------------------------------------------------------
    def _refresh_serial_ports(self, default: str = "") -> None:
        ports = [port.device for port in list_ports.comports()]
        if not ports:
            ports = [default] if default else []
        self.serial_port.clear()
        self.serial_port.addItems(ports)
        if default and default in ports:
            self.serial_port.setCurrentText(default)

    def _refresh_preset_buttons(self) -> None:
        for idx, btn in enumerate(self.preset_buttons):
            preset = self.bundle.presets[idx]
            if preset:
                btn.setText(preset.label(idx))
                btn.setEnabled(True)
            else:
                btn.setText(f"Preset {idx + 1}")
                btn.setEnabled(True)
        self.preset_group.button(self.bundle.selected_index).setChecked(True)
        selected = self.bundle.presets[self.bundle.selected_index]
        self.preset_name.setText(selected.name if selected else "")
        self._refresh_applicable_combo()

    def _refresh_applicable_combo(self) -> None:
        self.applicable_combo.blockSignals(True)
        self.applicable_combo.clear()
        for idx in range(MAX_SENSOR_PRESETS):
            preset = self.bundle.presets[idx]
            label = preset.label(idx) if preset else f"Preset {idx + 1}"
            self.applicable_combo.addItem(label, idx)
        current = max(0, min(self.bundle.applicable_index, MAX_SENSOR_PRESETS - 1))
        self.applicable_combo.setCurrentIndex(current)
        self.applicable_combo.blockSignals(False)

    def _collect_current_values(self) -> Dict[str, Any]:
        values = {
            "network": {"ip": self.network_ip.text().strip(), "port": int(self.network_port.value())},
            "sensor_type": int(self.sensor_type.currentIndex()),
            "sensor_id": int(self.sensor_id.value()),
            "pos_x": float(self.pos_x.value()),
            "pos_y": float(self.pos_y.value()),
            "pos_z": float(self.pos_z.value()),
            "init_roll_deg": float(self.roll.value()),
            "init_pitch_deg": float(self.pitch.value()),
            "init_yaw_deg": float(self.yaw.value()),
            "max_rate_dps": float(self.max_rate.value()),
            "power_on": self._power_on_flag,
            "serial_port": self.serial_port.currentText().strip(),
            "serial_baud": int(self.serial_baud.value()),
            "mav_sysid": int(self.mav_sysid.value()),
            "mav_compid": int(self.mav_compid.value()),
            "mavlink_preset_index": int(self.applicable_combo.currentIndex()),
        }
        self.bundle.network.ip = values["network"]["ip"]
        self.bundle.network.port = values["network"]["port"]
        values["generator_ip"] = self.bundle.network.ip
        values["generator_port"] = self.bundle.network.port
        return values

    def _build_preset_storage(self) -> Dict[str, Any]:
        slots = [preset.to_config() if preset else None for preset in self.bundle.presets]
        storage = {
            "preset_bundle": self.bundle.to_config(),
            "presets": {"version": 2, "slots": slots, "selected": self.bundle.selected_index, "applicable": self.bundle.applicable_index},
            "selected_preset": self.bundle.selected_index,
            "mavlink_preset_index": self.bundle.applicable_index,
        }
        return storage

    def _persist_bundle(self, values: Optional[Dict[str, Any]] = None, *, persist_to_disk: bool = True, show_error: bool = False) -> bool:
        payload = self._build_preset_storage()
        if values:
            payload.update(values)
        try:
            self.cfg.setdefault("gimbal", {}).update(payload)
            if persist_to_disk:
                cm = ConfigManager()
                app_cfg = cm.load().to_dict()
                app_cfg.setdefault("gimbal", {}).update(payload)
                cm.save(AppConfig.from_dict(app_cfg))
        except Exception as exc:
            try:
                if hasattr(self.log, "exception"):
                    self.log.exception("Failed to persist gimbal presets: %s", exc)
                elif hasattr(self.log, "error"):
                    self.log.error("Failed to persist gimbal presets: %s", exc)
            except Exception:
                pass
            if show_error:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to persist presets:\n{exc}")
            return False
        return True

    def _apply_to_runtime(self, values: Dict[str, Any]) -> None:
        try:
            if hasattr(self.gimbal, "update_settings"):
                self.gimbal.update_settings(values)
            if hasattr(self.gimbal, "set_target_pose"):
                
                # ❌ remap_input_rpy 호출 제거
                # sim_pitch, sim_yaw, sim_roll = remap_input_rpy(
                #     values.get("init_roll_deg", 0.0),
                #     values.get("init_pitch_deg", 0.0),
                #     values.get("init_yaw_deg", 0.0),
                # )
                
                # ✅ (R, P, Y) 값을 가져와서 (P, Y, R) 순서로 직접 전달
                roll_val = values.get("init_roll_deg", 0.0)
                pitch_val = values.get("init_pitch_deg", 0.0)
                yaw_val = values.get("init_yaw_deg", 0.0)
                
                self.gimbal.set_target_pose(
                    values.get("pos_x", 0.0),
                    values.get("pos_y", 0.0),
                    values.get("pos_z", 0.0),
                    pitch_val,  # sim_pitch_deg
                    yaw_val,    # sim_yaw_deg
                    roll_val,   # sim_roll_deg
                )
            if hasattr(self.gimbal, "set_max_rate"):
                self.gimbal.set_max_rate(values.get("max_rate_dps", 60.0))
            if hasattr(self.gimbal, "set_power"):
                self.gimbal.set_power(bool(values.get("power_on", True)))
            if hasattr(self.gimbal, "set_mav_ids"):
                self.gimbal.set_mav_ids(values.get("mav_sysid", 1), values.get("mav_compid", 154))
        except Exception:
            pass

    def _send_preset_pose_udp(
        self, preset: SensorPresetData, target_ip: str, target_port: int
    ) -> bool:
        if not hasattr(self.gimbal, "send_udp_preset"):
            return False
        try:
            # ❌ remap_input_rpy 호출 제거
            # sim_pitch, sim_yaw, sim_roll = remap_input_rpy(
            #     preset.init_roll_deg, preset.init_pitch_deg, preset.init_yaw_deg
            # )
            
            # ✅ (R, P, Y) 값을 (P, Y, R) 순서로 직접 전달
            self.gimbal.send_udp_preset(
                preset.sensor_type,
                preset.sensor_id,
                preset.pos_x,
                preset.pos_y,
                preset.pos_z,
                preset.init_pitch_deg,  # sim_pitch_deg
                preset.init_yaw_deg,    # sim_yaw_deg
                preset.init_roll_deg,   # sim_roll_deg
                ip=target_ip,
                port=int(target_port),
            )
            return True
        except Exception as exc:
            try:
                if hasattr(self.log, "warning"):
                    self.log.warning(
                        "Failed to send preset %d/%d over UDP: %s",
                        preset.sensor_type,
                        preset.sensor_id,
                        exc,
                    )
            except Exception:
                pass
            return False

    def _update_status(self) -> None:
        try:
            status = self.gimbal.get_status() if hasattr(self.gimbal, "get_status") else {}
        except Exception:
            status = {}
        text = status if isinstance(status, str) else status.get("text", "-") if isinstance(status, dict) else "-"
        self.status_label.setText(f"Status: {text}")

    # ------------------------------------------------------------------
    def on_save_preset(self) -> None:
        idx = self.preset_group.checkedId()
        if idx < 0:
            return
        name = self.preset_name.text().strip()
        data = SensorPresetData(
            sensor_type=int(self.sensor_type.currentIndex()),
            sensor_id=int(self.sensor_id.value()),
            pos_x=float(self.pos_x.value()),
            pos_y=float(self.pos_y.value()),
            pos_z=float(self.pos_z.value()),
            init_roll_deg=float(self.roll.value()),
            init_pitch_deg=float(self.pitch.value()),
            init_yaw_deg=float(self.yaw.value()),
            max_rate_dps=float(self.max_rate.value()),
            power_on=self._power_on_flag,
            serial_port=self.serial_port.currentText().strip(),
            serial_baud=int(self.serial_baud.value()),
            mav_sysid=int(self.mav_sysid.value()),
            mav_compid=int(self.mav_compid.value()),
        )
        preset = SensorPreset(name=name, data=data)
        self.bundle.presets[idx] = preset
        self._refresh_preset_buttons()
        if not self._persist_bundle():
            QtWidgets.QMessageBox.warning(self, "Warning", "Preset saved, but failed to persist to disk.")

    def on_clear_preset(self) -> None:
        idx = self.preset_group.checkedId()
        if idx < 0:
            return
        self.bundle.presets[idx] = None
        self._refresh_preset_buttons()
        if not self._persist_bundle():
            QtWidgets.QMessageBox.warning(self, "Warning", "Failed to persist preset removal to disk.")

    def on_apply_selected_preset(self) -> None:
        idx = self.preset_group.checkedId()
        if idx < 0:
            return
        preset = self.bundle.presets[idx]
        if not preset:
            return
        self._load_from_preset(preset)
        values = self._collect_current_values()
        self._apply_to_runtime(values)

    def on_apply_all_presets(self) -> None:
        try:
            values = self._collect_current_values()
        except Exception:
            values = {}
        network = values.get("network") if isinstance(values.get("network"), dict) else None
        target_ip = network.get("ip") if network else self.bundle.network.ip
        target_port = network.get("port") if network else self.bundle.network.port
        if not target_ip:
            target_ip = self.bundle.network.ip
        try:
            target_port = int(target_port)
        except Exception:
            target_port = self.bundle.network.port
        try:
            if hasattr(self.gimbal, "update_settings"):
                self.gimbal.update_settings(
                    {"generator_ip": target_ip, "generator_port": target_port}
                )
        except Exception:
            pass
        for preset in self.bundle.presets:
            if not preset:
                continue
            if not self._send_preset_pose_udp(preset.data, target_ip, target_port):
                values = preset.data.to_dict()
                self._apply_to_runtime(values)

    def on_refresh_ports(self) -> None:
        self._refresh_serial_ports(default=self.serial_port.currentText())

    def on_connect_serial(self) -> None:
        port = self.serial_port.currentText().strip()
        baud = int(self.serial_baud.value())
        if port and hasattr(self.gimbal, "open_serial"):
            try:
                self.gimbal.open_serial(port, baud)
            except Exception:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to open serial {port}")

    def on_apply_ids(self) -> None:
        sysid = int(self.mav_sysid.value())
        compid = int(self.mav_compid.value())
        if hasattr(self.gimbal, "set_mav_ids"):
            try:
                self.gimbal.set_mav_ids(sysid, compid)
            except Exception:
                pass

    def on_applicable_changed(self, index: int) -> None:
        self.bundle.applicable_index = index

    def on_power_on_clicked(self) -> None:
        self._apply_power_command(True)

    def on_power_off_clicked(self) -> None:
        self._apply_power_command(False)

    def _resolve_power_target_codes(self) -> Tuple[int, int]:
        idx = max(0, min(self.bundle.selected_index, MAX_SENSOR_PRESETS - 1))
        preset = self.bundle.presets[idx] if idx < len(self.bundle.presets) else None
        if preset:
            data = preset.data
            return int(data.sensor_type), int(data.sensor_id)
        return int(self.sensor_type.currentIndex()), int(self.sensor_id.value())

    def _apply_power_command(self, desired: bool) -> None:
        sensor_type, sensor_id = self._resolve_power_target_codes()
        packet: Optional[bytes] = None
        try:
            if hasattr(self.gimbal, "update_settings"):
                self.gimbal.update_settings({"sensor_type": sensor_type, "sensor_id": sensor_id})
            if hasattr(self.gimbal, "send_power"):
                packet = self.gimbal.send_power(desired, sensor_type=sensor_type, sensor_id=sensor_id)
            elif hasattr(self.gimbal, "build_power_packet"):
                packet = self.gimbal.build_power_packet(desired, sensor_type=sensor_type, sensor_id=sensor_id)  # type: ignore[attr-defined]
            elif hasattr(self.gimbal, "get_power_packet_example"):
                raw = self.gimbal.get_power_packet_example(desired, sensor_type=sensor_type, sensor_id=sensor_id)  # type: ignore[attr-defined]
                packet = bytes(raw)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Power command failed:\n{exc}")
            return

        self._power_on_flag = bool(desired)
        preset = (
            self.bundle.presets[self.bundle.selected_index]
            if 0 <= self.bundle.selected_index < len(self.bundle.presets)
            else None
        )
        if preset:
            preset.data.power_on = self._power_on_flag
        self.cfg.setdefault("gimbal", {})["power_on"] = self._power_on_flag

        message = (
            f"Sensor power {'ON' if desired else 'OFF'} applied to sensor {sensor_type}/{sensor_id}."
        )
        if packet is not None:
            hex_bytes = " ".join(f"0x{b:02X}" for b in packet)
            message += f"\nPacket bytes: {hex_bytes}"
        QtWidgets.QMessageBox.information(self, "Power Control", message)

    def _on_preset_selected(self, idx: int, checked: bool) -> None:
        if not checked:
            return
        self.bundle.selected_index = idx
        preset = self.bundle.presets[idx]
        self.preset_name.setText(preset.name if preset else "")
        if preset:
            self._load_from_preset(preset)
        self._refresh_applicable_combo()

    def _load_from_preset(self, preset: SensorPreset) -> None:
        data = preset.data
        self.sensor_type.setCurrentIndex(data.sensor_type)
        self.sensor_id.setValue(data.sensor_id)
        self.pos_x.setValue(data.pos_x)
        self.pos_y.setValue(data.pos_y)
        self.pos_z.setValue(data.pos_z)
        self.roll.setValue(data.init_roll_deg)
        self.pitch.setValue(data.init_pitch_deg)
        self.yaw.setValue(data.init_yaw_deg)
        self.max_rate.setValue(data.max_rate_dps)
        self._power_on_flag = bool(data.power_on)
        if data.serial_port:
            idx = self.serial_port.findText(data.serial_port)
            if idx >= 0:
                self.serial_port.setCurrentIndex(idx)
        self.serial_baud.setValue(data.serial_baud)
        self.mav_sysid.setValue(data.mav_sysid)
        self.mav_compid.setValue(data.mav_compid)

    def on_save(self) -> None:
        try:
            values = self._collect_current_values()
            if self._persist_bundle(values, show_error=True):
                QtWidgets.QMessageBox.information(self, "Saved", "Gimbal settings saved.")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Save failed:\n{exc}")

    def on_apply_close(self) -> None:
        try:
            values = self._collect_current_values()
            self.updated_config = dict(values)
            self.updated_config.update(self._build_preset_storage())
            self._persist_bundle(values, persist_to_disk=False)
            self._apply_to_runtime(values)
            self.accept()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", f"Apply failed:\n{exc}")