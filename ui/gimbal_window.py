# ui/gimbal_window.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
from dataclasses import dataclass, field
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Any, Dict, Iterable, List, Optional


from serial.tools import list_ports
from utils.settings import ConfigManager, AppConfig  # type: ignore

SENSOR_TYPES = ["Camera", "GPS", "LiDAR", "RADAR", "LRF", "IMU"]
SENSOR_COMBO_VALUES = [f"{i}: {name}" for i, name in enumerate(SENSOR_TYPES)]

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


def _as_str(value: Any, default: str = "") -> str:
    if value is None:
        return default
    return str(value)


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


@dataclass
class NetworkSettings:
    ip: str = "127.0.0.1"
    port: int = 15020

    @classmethod
    def from_config(cls, gimbal_cfg: Dict[str, Any]) -> "NetworkSettings":
        network = gimbal_cfg.get("network") if isinstance(gimbal_cfg.get("network"), dict) else {}
        ip = _as_str(network.get("ip", gimbal_cfg.get("generator_ip", "127.0.0.1")))
        port_source = network.get("port", gimbal_cfg.get("generator_port", 15020))
        return cls(ip=ip, port=_as_int(port_source, 15020))

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
    def from_dict(cls, data: Optional[Dict[str, Any]]) -> "SensorPresetData":
        if not isinstance(data, dict):
            return cls()
        return cls(
            sensor_type=_as_int(data.get("sensor_type", 0), 0),
            sensor_id=_as_int(data.get("sensor_id", 0), 0),
            pos_x=_as_float(data.get("pos_x", 0.0), 0.0),
            pos_y=_as_float(data.get("pos_y", 0.0), 0.0),
            pos_z=_as_float(data.get("pos_z", 0.0), 0.0),
            init_roll_deg=_as_float(data.get("init_roll_deg", 0.0), 0.0),
            init_pitch_deg=_as_float(data.get("init_pitch_deg", 0.0), 0.0),
            init_yaw_deg=_as_float(data.get("init_yaw_deg", 0.0), 0.0),
            max_rate_dps=_as_float(data.get("max_rate_dps", 60.0), 60.0),
            power_on=_as_bool(data.get("power_on", True), True),
            serial_port=_as_str(data.get("serial_port", "")),
            serial_baud=_as_int(data.get("serial_baud", 115200), 115200),
            mav_sysid=_as_int(data.get("mav_sysid", 1), 1),
            mav_compid=_as_int(data.get("mav_compid", 154), 154),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "sensor_type": self.sensor_type,
            "sensor_id": self.sensor_id,
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

    @classmethod
    def from_config(cls, gimbal_cfg: Dict[str, Any]) -> "PresetBundle":
        network = NetworkSettings.from_config(gimbal_cfg)
        presets: List[Optional[SensorPreset]] = []
        raw_bundle = gimbal_cfg.get("preset_bundle") if isinstance(gimbal_cfg.get("preset_bundle"), dict) else None
        raw_slots: Optional[Any] = None
        selected = _as_int(gimbal_cfg.get("selected_preset", 0), 0)
        if raw_bundle:
            raw_slots = raw_bundle.get("presets") or raw_bundle.get("slots")
            selected = _as_int(raw_bundle.get("selected", raw_bundle.get("selected_preset", selected)), selected)
        if raw_slots is None:
            legacy = gimbal_cfg.get("presets")
            if isinstance(legacy, dict):
                raw_slots = legacy.get("slots")
                selected = _as_int(legacy.get("selected", selected), selected)
            elif isinstance(legacy, list):
                raw_slots = legacy
        slot_items: List[Any] = []
        if isinstance(raw_slots, list):
            slot_items = raw_slots[:MAX_SENSOR_PRESETS]
        elif isinstance(raw_slots, dict):
            keys = set(raw_slots.keys())
            zero_based = any(k in keys for k in (0, "0"))
            for i in range(MAX_SENSOR_PRESETS):
                candidates: List[Any] = []
                if zero_based:
                    candidates.extend([i, str(i)])
                candidates.extend([i + 1, str(i + 1)])
                candidates.extend([i, str(i)])
                value = None
                for key in candidates:
                    if key in raw_slots:
                        value = raw_slots[key]
                        break
                slot_items.append(value)
        elif isinstance(raw_slots, Iterable):
            slot_items = list(raw_slots)[:MAX_SENSOR_PRESETS]

        for item in slot_items:
            presets.append(SensorPreset.from_any(item))
        while len(presets) < MAX_SENSOR_PRESETS:
            presets.append(None)
        selected = max(0, min(MAX_SENSOR_PRESETS - 1, selected))
        return cls(network=network, presets=presets, selected_index=selected)

    def to_config(self) -> Dict[str, Any]:
        return {
            "network": self.network.to_config(),
            "presets": [preset.to_config() if preset else None for preset in self.presets],
            "selected_preset": self.selected_index,
        }

    def set_preset(self, idx: int, preset: SensorPreset) -> None:
        if 0 <= idx < MAX_SENSOR_PRESETS:
            self.presets[idx] = preset

    def clear_preset(self, idx: int) -> None:
        if 0 <= idx < MAX_SENSOR_PRESETS:
            self.presets[idx] = None

    def get(self, idx: int) -> Optional[SensorPreset]:
        if 0 <= idx < MAX_SENSOR_PRESETS:
            return self.presets[idx]
        return None



class GimbalControlsWindow(tk.Toplevel):
    def __init__(self, master: tk.Misc, cfg: Dict[str, Any], gimbal, log: logging.Logger) -> None:
        super().__init__(master)
        self.title("Gimbal Controls")
        self.resizable(False, False)

        self.cfg = cfg
        self.gimbal = gimbal
        self.log = log

        gimbal_cfg = cfg.get("gimbal", {})
        self.bundle = PresetBundle.from_config(gimbal_cfg)

        active_preset = self.bundle.get(self.bundle.selected_index)
        if active_preset:
            payload = active_preset.data
        else:
            payload = SensorPresetData.from_dict(gimbal_cfg)

        self.v_preset_index = tk.IntVar(value=self.bundle.selected_index)
        self.v_preset_name = tk.StringVar()
        self._preset_radios: List[ttk.Radiobutton] = []

        self.v_gen_ip = tk.StringVar(value=self.bundle.network.ip)
        self.v_gen_port = tk.IntVar(value=self.bundle.network.port)

        self.v_sensor_type_combo = tk.StringVar(value=self._sensor_combo_label(payload.sensor_type))
        self.v_sensor_id = tk.IntVar(value=payload.sensor_id)

        self.v_pos_x = tk.DoubleVar(value=payload.pos_x)
        self.v_pos_y = tk.DoubleVar(value=payload.pos_y)
        self.v_pos_z = tk.DoubleVar(value=payload.pos_z)
        self.v_roll = tk.DoubleVar(value=payload.init_roll_deg)
        self.v_pitch = tk.DoubleVar(value=payload.init_pitch_deg)
        self.v_yaw = tk.DoubleVar(value=payload.init_yaw_deg)

        self.v_max_rate = tk.DoubleVar(value=payload.max_rate_dps)
        self.v_power_on = tk.BooleanVar(value=payload.power_on)

        self.v_serial_port = tk.StringVar(value=payload.serial_port)
        self.v_baud = tk.IntVar(value=payload.serial_baud)
        self.v_mav_sysid = tk.IntVar(value=payload.mav_sysid)
        self.v_mav_compid = tk.IntVar(value=payload.mav_compid)

        self.cur_x = tk.StringVar(value="-")
        self.cur_y = tk.StringVar(value="-")
        self.cur_z = tk.StringVar(value="-")
        self.cur_r = tk.StringVar(value="-")
        self.cur_p = tk.StringVar(value="-")
        self.cur_yw = tk.StringVar(value="-")
        self.cur_wx = tk.StringVar(value="-")
        self.cur_wy = tk.StringVar(value="-")
        self.cur_wz = tk.StringVar(value="-")

        self._build_layout()
        self._load_selected_preset(self.bundle.selected_index, apply_runtime=False)
        self._write_back_state()

        self._refresh_status_periodic()

    # ------------------------------------------------------------------
    # UI construction helpers
    def _build_layout(self) -> None:
        pad = dict(padx=6, pady=4)
        root = ttk.Frame(self, padding=10)
        root.grid(row=0, column=0, sticky="nsew")

        row = 0
        row = self._build_network_section(root, row, pad)
        row = self._build_presets_section(root, row, pad)
        row = self._build_sensor_section(root, row, pad)
        row = self._build_pose_section(root, row, pad)
        row = self._build_rate_section(root, row, pad)
        row = self._build_power_section(root, row, pad)
        row = self._build_serial_section(root, row, pad)
        row = self._build_status_section(root, row, pad)
        self._build_footer(root, row, pad)

    def _build_network_section(self, parent: ttk.Frame, row: int, pad: Dict[str, int]) -> int:
        box = ttk.Labelframe(parent, text="Network Sensor Control")
        box.grid(row=row, column=0, columnspan=4, sticky="ew", **pad)
        ttk.Label(box, text="IP").grid(row=0, column=0, sticky="e", padx=4, pady=2)
        ttk.Entry(box, textvariable=self.v_gen_ip, width=18).grid(row=0, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(box, text="Port").grid(row=0, column=2, sticky="e", padx=4, pady=2)
        ttk.Entry(box, textvariable=self.v_gen_port, width=8).grid(row=0, column=3, sticky="w", padx=4, pady=2)
        return row + 1

    def _build_presets_section(self, parent: ttk.Frame, row: int, pad: Dict[str, int]) -> int:
        box = ttk.Labelframe(parent, text="Sensor Presets")
        box.grid(row=row, column=0, columnspan=4, sticky="ew", **pad)
        box.columnconfigure(1, weight=1)

        rb_holder = ttk.Frame(box)
        rb_holder.grid(row=0, column=0, columnspan=4, sticky="w", padx=4, pady=2)
        for idx in range(MAX_SENSOR_PRESETS):
            btn = ttk.Radiobutton(
                rb_holder,
                text=f"Preset {idx + 1}",

                variable=self.v_preset_index,
                value=idx,
                command=lambda i=idx: self.on_select_preset(i),
            )
            btn.grid(row=0, column=idx, sticky="w", padx=2)
            self._preset_radios.append(btn)


        ttk.Label(box, text="Name").grid(row=1, column=0, sticky="e", padx=4, pady=2)
        ttk.Entry(box, textvariable=self.v_preset_name, width=24).grid(row=1, column=1, sticky="we", padx=4, pady=2)
        ttk.Button(box, text="Save to Preset", command=self.on_save_preset).grid(row=1, column=2, sticky="w", padx=4, pady=2)
        ttk.Button(box, text="Clear Preset", command=self.on_clear_preset).grid(row=1, column=3, sticky="w", padx=4, pady=2)
        return row + 1

    def _build_sensor_section(self, parent: ttk.Frame, row: int, pad: Dict[str, int]) -> int:
        ttk.Label(parent, text="Sensor").grid(row=row, column=0, sticky="e", **pad)
        ttk.Combobox(parent, textvariable=self.v_sensor_type_combo, values=SENSOR_COMBO_VALUES, state="readonly", width=16).grid(row=row, column=1, sticky="w", **pad)
        ttk.Label(parent, text="ID").grid(row=row, column=2, sticky="e", **pad)
        ttk.Entry(parent, textvariable=self.v_sensor_id, width=8).grid(row=row, column=3, sticky="w", **pad)
        row += 1
        ttk.Separator(parent, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=4, sticky="ew", pady=(6, 6))
        return row + 1

    def _build_pose_section(self, parent: ttk.Frame, row: int, pad: Dict[str, int]) -> int:
        ttk.Label(parent, text="Target Pose (x,y,z,r,p,y) [m / deg]").grid(row=row, column=0, columnspan=4, sticky="w", **pad)
        row += 1
        for label, var in (("x", self.v_pos_x), ("y", self.v_pos_y)):
            ttk.Label(parent, text=label).grid(row=row, column=(0 if label == "x" else 2), sticky="e", **pad)
            ttk.Entry(parent, textvariable=var, width=10).grid(row=row, column=(1 if label == "x" else 3), sticky="w", **pad)
        row += 1
        for label, var in (("z", self.v_pos_z), ("roll", self.v_roll)):
            ttk.Label(parent, text=label).grid(row=row, column=(0 if label == "z" else 2), sticky="e", **pad)
            ttk.Entry(parent, textvariable=var, width=10).grid(row=row, column=(1 if label == "z" else 3), sticky="w", **pad)
        row += 1
        for label, var in (("pitch", self.v_pitch), ("yaw", self.v_yaw)):
            ttk.Label(parent, text=label).grid(row=row, column=(0 if label == "pitch" else 2), sticky="e", **pad)
            ttk.Entry(parent, textvariable=var, width=10).grid(row=row, column=(1 if label == "pitch" else 3), sticky="w", **pad)

        btns_pose = ttk.Frame(parent)
        btns_pose.grid(row=row + 1, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(btns_pose, text="Apply Pose", command=self.on_apply_pose).grid(row=0, column=0, padx=6)
        tpl = ttk.Frame(parent)
        tpl.grid(row=row + 2, column=0, columnspan=4, sticky="we", **pad)
        ttk.Label(tpl, text="Templates:").pack(side=tk.LEFT)
        ttk.Button(tpl, text="Home", command=lambda: self._set_tpl(0, 0, 0, 0, 0, 0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(tpl, text="Forward", command=lambda: self._set_tpl(None, None, None, 0, 0, 0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(tpl, text="Nadir", command=lambda: self._set_tpl(None, None, None, 0, -90, 0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(tpl, text="Left", command=lambda: self._set_tpl(None, None, None, 0, 0, -90)).pack(side=tk.LEFT, padx=2)
        ttk.Button(tpl, text="Right", command=lambda: self._set_tpl(None, None, None, 0, 0, 90)).pack(side=tk.LEFT, padx=2)
        return row + 3

    def _build_rate_section(self, parent: ttk.Frame, row: int, pad: Dict[str, int]) -> int:
        ttk.Label(parent, text="Max Angular Rate (deg/s)").grid(row=row, column=0, sticky="e", **pad)
        ttk.Entry(parent, textvariable=self.v_max_rate, width=10).grid(row=row, column=1, sticky="w", **pad)
        ttk.Button(parent, text="Apply Max Rate", command=self.on_apply_max_rate).grid(row=row, column=3, sticky="e", **pad)
        row += 1
        ttk.Separator(parent, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=4, sticky="ew", pady=(6, 6))
        return row + 1

    def _build_power_section(self, parent: ttk.Frame, row: int, pad: Dict[str, int]) -> int:
        ttk.Checkbutton(parent, text="Power ON", variable=self.v_power_on).grid(row=row, column=0, columnspan=2, sticky="w", **pad)
        ttk.Button(parent, text="Apply Power", command=self.on_apply_power).grid(row=row, column=3, sticky="e", **pad)
        row += 1
        ttk.Separator(parent, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=4, sticky="ew", pady=(6, 6))
        return row + 1

    def _build_serial_section(self, parent: ttk.Frame, row: int, pad: Dict[str, int]) -> int:
        ttk.Label(parent, text="MAVLink Serial").grid(row=row, column=0, columnspan=4, sticky="w", **pad)
        row += 1
        ttk.Label(parent, text="Port").grid(row=row, column=0, sticky="e", **pad)
        self.cb_ports = ttk.Combobox(parent, textvariable=self.v_serial_port, values=self._enum_serial_ports(), width=18)
        self.cb_ports.grid(row=row, column=1, sticky="w", **pad)
        ttk.Label(parent, text="Baud").grid(row=row, column=2, sticky="e", **pad)
        ttk.Entry(parent, textvariable=self.v_baud, width=10).grid(row=row, column=3, sticky="w", **pad)
        row += 1
        ttk.Label(parent, text="System ID").grid(row=row, column=0, sticky="e", **pad)
        ttk.Entry(parent, textvariable=self.v_mav_sysid, width=10).grid(row=row, column=1, sticky="w", **pad)
        ttk.Label(parent, text="Component ID").grid(row=row, column=2, sticky="e", **pad)
        ttk.Entry(parent, textvariable=self.v_mav_compid, width=10).grid(row=row, column=3, sticky="w", **pad)
        row += 1
        btns = ttk.Frame(parent)
        btns.grid(row=row, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(btns, text="Apply IDs", command=self.on_apply_ids).grid(row=0, column=0, padx=6)
        ttk.Button(btns, text="Refresh Ports", command=self.on_refresh_ports).grid(row=0, column=1, padx=6)
        ttk.Button(btns, text="Connect Serial", command=self.on_connect_serial).grid(row=0, column=2, padx=6)
        row += 1
        ttk.Separator(parent, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=4, sticky="ew", pady=(6, 6))
        return row + 1

    def _build_status_section(self, parent: ttk.Frame, row: int, pad: Dict[str, int]) -> int:
        cur = ttk.Frame(parent)

        cur.grid(row=row, column=0, columnspan=4, sticky="we", **pad)
        ttk.Label(cur, text="Current Pose:").grid(row=0, column=0, sticky="w")
        ttk.Label(cur, text="x").grid(row=1, column=0, sticky="e")
        ttk.Label(cur, textvariable=self.cur_x, width=10).grid(row=1, column=1, sticky="w")
        ttk.Label(cur, text="y").grid(row=1, column=2, sticky="e")
        ttk.Label(cur, textvariable=self.cur_y, width=10).grid(row=1, column=3, sticky="w")
        ttk.Label(cur, text="z").grid(row=1, column=4, sticky="e")
        ttk.Label(cur, textvariable=self.cur_z, width=10).grid(row=1, column=5, sticky="w")
        ttk.Label(cur, text="roll").grid(row=2, column=0, sticky="e")
        ttk.Label(cur, textvariable=self.cur_r, width=10).grid(row=2, column=1, sticky="w")
        ttk.Label(cur, text="pitch").grid(row=2, column=2, sticky="e")
        ttk.Label(cur, textvariable=self.cur_p, width=10).grid(row=2, column=3, sticky="w")
        ttk.Label(cur, text="yaw").grid(row=2, column=4, sticky="e")
        ttk.Label(cur, textvariable=self.cur_yw, width=10).grid(row=2, column=5, sticky="w")
        ttk.Label(cur, text="ωx/ωy/ωz").grid(row=3, column=0, sticky="e")
        ttk.Label(cur, textvariable=self.cur_wx, width=10).grid(row=3, column=1, sticky="w")
        ttk.Label(cur, textvariable=self.cur_wy, width=10).grid(row=3, column=3, sticky="w")
        ttk.Label(cur, textvariable=self.cur_wz, width=10).grid(row=3, column=5, sticky="w")
        self.lbl_status = ttk.Label(parent, text="Status: -")
        self.lbl_status.grid(row=row + 1, column=0, columnspan=4, sticky="w", **pad)
        return row + 2

    def _build_footer(self, parent: ttk.Frame, row: int, pad: Dict[str, int]) -> None:
        btns = ttk.Frame(parent)
        btns.grid(row=row, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(btns, text="Save", command=self.on_save).grid(row=0, column=0, padx=6)
        ttk.Button(btns, text="Apply & Close", command=self.on_apply_close).grid(row=0, column=1, padx=6)

    # ------------------------------------------------------------------
    # Internal helpers
    def _enum_serial_ports(self) -> List[str]:
        try:
            return [p.device for p in list_ports.comports()]
        except Exception:
            return []

    def _sensor_combo_label(self, code: int) -> str:
        if 0 <= code < len(SENSOR_COMBO_VALUES):
            return SENSOR_COMBO_VALUES[code]
        return f"{code}: Sensor"

    def _sensor_code_from_combo(self) -> int:
        combo = (self.v_sensor_type_combo.get() or "0").split(":", 1)[0]
        return _as_int(combo, 0)

    def _var_get(self, variable: tk.Variable, fallback: Any) -> Any:
        try:
            return variable.get()
        except Exception:
            return fallback

    def _default_preset_name(self, idx: int) -> str:
        return f"Preset {idx + 1}"

    def _update_preset_labels(self) -> None:
        for idx, btn in enumerate(self._preset_radios):
            preset = self.bundle.get(idx)
            label = preset.label(idx) if preset else self._default_preset_name(idx)
            btn.configure(text=label)
        active = self.bundle.get(self.bundle.selected_index)
        if active:
            name = active.name.strip() or self._default_preset_name(self.bundle.selected_index)
        else:
            name = self._default_preset_name(self.bundle.selected_index)
        self.v_preset_name.set(name)

    def _collect_payload_from_form(self) -> SensorPresetData:
        return SensorPresetData(
            sensor_type=self._sensor_code_from_combo(),
            sensor_id=_as_int(self._var_get(self.v_sensor_id, 0), 0),
            pos_x=_as_float(self._var_get(self.v_pos_x, 0.0), 0.0),
            pos_y=_as_float(self._var_get(self.v_pos_y, 0.0), 0.0),
            pos_z=_as_float(self._var_get(self.v_pos_z, 0.0), 0.0),
            init_roll_deg=_as_float(self._var_get(self.v_roll, 0.0), 0.0),
            init_pitch_deg=_as_float(self._var_get(self.v_pitch, 0.0), 0.0),
            init_yaw_deg=_as_float(self._var_get(self.v_yaw, 0.0), 0.0),
            max_rate_dps=_as_float(self._var_get(self.v_max_rate, 60.0), 60.0),
            power_on=_as_bool(self._var_get(self.v_power_on, True), True),
            serial_port=_as_str(self._var_get(self.v_serial_port, "")).strip(),
            serial_baud=_as_int(self._var_get(self.v_baud, 115200), 115200),
            mav_sysid=_as_int(self._var_get(self.v_mav_sysid, 1), 1),
            mav_compid=_as_int(self._var_get(self.v_mav_compid, 154), 154),
        )

    def _apply_payload_to_form(self, payload: SensorPresetData) -> None:
        self.v_sensor_type_combo.set(self._sensor_combo_label(payload.sensor_type))
        self.v_sensor_id.set(payload.sensor_id)
        self.v_pos_x.set(payload.pos_x)
        self.v_pos_y.set(payload.pos_y)
        self.v_pos_z.set(payload.pos_z)
        self.v_roll.set(payload.init_roll_deg)
        self.v_pitch.set(payload.init_pitch_deg)
        self.v_yaw.set(payload.init_yaw_deg)
        self.v_max_rate.set(payload.max_rate_dps)
        self.v_power_on.set(bool(payload.power_on))
        self.v_serial_port.set(payload.serial_port)
        self.v_baud.set(payload.serial_baud)
        self.v_mav_sysid.set(payload.mav_sysid)
        self.v_mav_compid.set(payload.mav_compid)

    def _active_runtime_values(self) -> Dict[str, Any]:
        payload = self._collect_payload_from_form()
        self.bundle.network.ip = _as_str(self._var_get(self.v_gen_ip, self.bundle.network.ip), self.bundle.network.ip)
        self.bundle.network.port = _as_int(self._var_get(self.v_gen_port, self.bundle.network.port), self.bundle.network.port)
        values = payload.to_dict()
        values.update({
            "generator_ip": self.bundle.network.ip,
            "generator_port": self.bundle.network.port,
        })
        return values

    def _write_back_state(self) -> None:
        payload = self._collect_payload_from_form()
        self.bundle.network.ip = _as_str(self._var_get(self.v_gen_ip, self.bundle.network.ip), self.bundle.network.ip)
        self.bundle.network.port = _as_int(self._var_get(self.v_gen_port, self.bundle.network.port), self.bundle.network.port)
        bundle_dict = self.bundle.to_config()
        gimbal_cfg = self.cfg.setdefault("gimbal", {})
        gimbal_cfg.update(payload.to_dict())
        gimbal_cfg["generator_ip"] = self.bundle.network.ip
        gimbal_cfg["generator_port"] = self.bundle.network.port
        gimbal_cfg["network"] = bundle_dict["network"]
        gimbal_cfg["preset_bundle"] = {
            "network": bundle_dict["network"],
            "presets": bundle_dict["presets"],
            "selected": bundle_dict["selected_preset"],
        }
        gimbal_cfg["presets"] = {"version": 2, "slots": bundle_dict["presets"]}
        gimbal_cfg["selected_preset"] = bundle_dict["selected_preset"]

    def _apply_runtime(self, values: Dict[str, Any]) -> None:
        try:
            if hasattr(self.gimbal, "update_settings"):
                self.gimbal.update_settings(values)
        except Exception as exc:
            self.log.exception("Failed to update gimbal settings: %s", exc)
        pose_keys = ("pos_x", "pos_y", "pos_z", "init_roll_deg", "init_pitch_deg", "init_yaw_deg")
        if all(k in values for k in pose_keys) and hasattr(self.gimbal, "set_target_pose"):
            try:
                self.gimbal.set_target_pose(
                    values["pos_x"],
                    values["pos_y"],
                    values["pos_z"],
                    values["init_roll_deg"],
                    values["init_pitch_deg"],
                    values["init_yaw_deg"],
                )
            except Exception as exc:
                self.log.exception("Failed to push preset pose to gimbal: %s", exc)
        if "max_rate_dps" in values and hasattr(self.gimbal, "set_max_rate"):
            try:
                self.gimbal.set_max_rate(values["max_rate_dps"])
            except Exception as exc:
                self.log.exception("Failed to update gimbal max rate: %s", exc)
        if "power_on" in values and hasattr(self.gimbal, "set_power"):
            try:
                self.gimbal.set_power(bool(values["power_on"]))
            except Exception as exc:
                self.log.exception("Failed to sync gimbal power state: %s", exc)
        if {"mav_sysid", "mav_compid"}.issubset(values) and hasattr(self.gimbal, "set_mav_ids"):
            try:
                self.gimbal.set_mav_ids(values["mav_sysid"], values["mav_compid"])
            except Exception as exc:
                self.log.exception("Failed to set MAV IDs: %s", exc)
        port = _as_str(values.get("serial_port", "")).strip()
        if port and "serial_baud" in values and hasattr(self.gimbal, "open_serial"):
            try:
                self.gimbal.open_serial(port, _as_int(values["serial_baud"], 115200))
            except Exception as exc:
                self.log.exception("Failed to open serial from preset: %s", exc)

    # ------------------------------------------------------------------
    # Actions

    def _set_tpl(self, x, y, z, r, p, yw):
        if x is not None:
            self.v_pos_x.set(x)
        if y is not None:
            self.v_pos_y.set(y)
        if z is not None:
            self.v_pos_z.set(z)
        if r is not None:
            self.v_roll.set(r)
        if p is not None:
            self.v_pitch.set(p)
        if yw is not None:
            self.v_yaw.set(yw)

    def on_select_preset(self, idx: int) -> None:
        idx = max(0, min(MAX_SENSOR_PRESETS - 1, idx))
        self._load_selected_preset(idx, apply_runtime=True)

    def _load_selected_preset(self, idx: int, apply_runtime: bool) -> None:
        self.bundle.selected_index = idx
        self.v_preset_index.set(idx)
        preset = self.bundle.get(idx)
        if preset:
            self._apply_payload_to_form(preset.data)
            name = preset.name.strip() or self._default_preset_name(idx)
        else:
            name = self._default_preset_name(idx)
        self.v_preset_name.set(name)
        self._update_preset_labels()
        self._write_back_state()
        if apply_runtime:
            self._apply_runtime(self._active_runtime_values())

    def on_save_preset(self) -> None:
        idx = self.bundle.selected_index
        try:
            payload = self._collect_payload_from_form()
        except Exception as exc:
            messagebox.showerror("Error", f"Save preset failed:\n{exc}")
            return
        name = self.v_preset_name.get().strip() or self._default_preset_name(idx)
        self.bundle.set_preset(idx, SensorPreset(name=name, data=payload))
        self.v_preset_name.set(name)
        self._update_preset_labels()
        self._write_back_state()
        messagebox.showinfo("Saved", f"Preset {idx + 1} saved.")

    def on_clear_preset(self) -> None:
        idx = self.bundle.selected_index
        self.bundle.clear_preset(idx)
        self.v_preset_name.set(self._default_preset_name(idx))
        self._update_preset_labels()
        self._write_back_state()
        messagebox.showinfo("Cleared", f"Preset {idx + 1} cleared.")

    def on_apply_pose(self) -> None:
        try:
            values = self._active_runtime_values()
            self._write_back_state()
            if hasattr(self.gimbal, "update_settings"):
                self.gimbal.update_settings(values)
            if hasattr(self.gimbal, "set_target_pose"):
                self.gimbal.set_target_pose(
                    values["pos_x"], values["pos_y"], values["pos_z"],
                    values["init_roll_deg"], values["init_pitch_deg"], values["init_yaw_deg"],
                )
            messagebox.showinfo("OK", "Target pose applied (controller will drive to target).")
        except Exception as exc:
            messagebox.showerror("Error", f"Apply pose failed:\n{exc}")

    def on_apply_max_rate(self) -> None:
        try:
            values = self._active_runtime_values()
            self._write_back_state()
            if hasattr(self.gimbal, "update_settings"):
                self.gimbal.update_settings(values)
            if hasattr(self.gimbal, "set_max_rate"):
                self.gimbal.set_max_rate(values["max_rate_dps"])
            messagebox.showinfo("OK", "Max angular rate updated.")
        except Exception as exc:
            messagebox.showerror("Error", f"Apply max rate failed:\n{exc}")

    def on_apply_power(self) -> None:
        try:
            on = bool(self._var_get(self.v_power_on, True))
            if hasattr(self.gimbal, "send_power"):
                self.gimbal.send_power(on)
            messagebox.showinfo("OK", f"Power {'ON' if on else 'OFF'} applied.")
        except Exception as exc:
            messagebox.showerror("Error", f"Apply Power failed:\n{exc}")

    def on_connect_serial(self) -> None:
        try:
            values = self._active_runtime_values()
            self._write_back_state()
            if hasattr(self.gimbal, "update_settings"):
                self.gimbal.update_settings(values)
            if hasattr(self.gimbal, "open_serial"):
                self.gimbal.open_serial(values["serial_port"], values["serial_baud"])
            messagebox.showinfo("OK", f"Serial connected: {values['serial_port']} @ {values['serial_baud']}")
        except Exception as exc:
            messagebox.showerror("Error", f"Serial connect failed:\n{exc}")

    def on_refresh_ports(self) -> None:
        try:
            self.cb_ports["values"] = self._enum_serial_ports()
        except Exception as exc:
            messagebox.showerror("Error", f"Port refresh failed:\n{exc}")

    def on_apply_ids(self) -> None:
        try:
            sysid = _as_int(self._var_get(self.v_mav_sysid, 1), 1)
            compid = _as_int(self._var_get(self.v_mav_compid, 154), 154)
            if hasattr(self.gimbal, "set_mav_ids"):
                self.gimbal.set_mav_ids(sysid, compid)
            messagebox.showinfo("OK", f"MAV IDs applied: sys={sysid}, comp={compid}")
        except Exception as exc:
            messagebox.showerror("Error", f"Apply IDs failed:\n{exc}")

    def on_save(self) -> None:
        try:
            self._write_back_state()
            cm = ConfigManager()
            ac = cm.load()
            data = ac.to_dict()
            data.setdefault("gimbal", {}).update(self.cfg.get("gimbal", {}))
            cm.save(AppConfig.from_dict(data))
            messagebox.showinfo("Saved", "Gimbal settings saved.")
        except Exception as exc:
            messagebox.showerror("Error", f"Save failed:\n{exc}")

    def on_apply_close(self) -> None:
        try:
            values = self._active_runtime_values()
            self._write_back_state()
            if hasattr(self.gimbal, "update_settings"):
                self.gimbal.update_settings(values)
        except Exception as exc:
            messagebox.showerror("Error", f"Apply failed:\n{exc}")

            return
        self.destroy()

    # ------------------------------------------------------------------
    # Status polling
    def _refresh_status_periodic(self) -> None:
        try:
            status = self.gimbal.get_status() if hasattr(self.gimbal, "get_status") else {}
            act = status.get("activated", True)
            mode = status.get("control_mode", "CTRL")
            r = status.get("current_roll_deg", 0.0)
            p = status.get("current_pitch_deg", 0.0)
            y = status.get("current_yaw_deg", 0.0)
            x = status.get("current_x", 0.0)
            yx = status.get("current_y", 0.0)
            z = status.get("current_z", 0.0)
            wx = status.get("wx", 0.0)
            wy = status.get("wy", 0.0)
            wz = status.get("wz", 0.0)
            mrate = status.get("max_rate_dps", self.v_max_rate.get())
            ser = status.get("serial_state", "-")
            hb = status.get("hb_rx_ok", False)
            sysid = status.get("mav_sysid", self.v_mav_sysid.get())
            compid = status.get("mav_compid", self.v_mav_compid.get())

            self.cur_x.set(f"{x:.2f}")
            self.cur_y.set(f"{yx:.2f}")
            self.cur_z.set(f"{z:.2f}")
            self.cur_r.set(f"{r:.1f}")
            self.cur_p.set(f"{p:.1f}")
            self.cur_yw.set(f"{y:.1f}")
            self.cur_wx.set(f"{wx:.2f}")
            self.cur_wy.set(f"{wy:.2f}")
            self.cur_wz.set(f"{wz:.2f}")

            self.lbl_status.configure(
                text=(
                    f"Status: {'On' if act else 'Off'} | Mode={mode} | "
                    f"Sys/Comp={sysid}/{compid} | MaxRate={mrate:.1f} dps | "
                    f"Serial={ser} | HB={hb}"
                )
            )
        except Exception:
            pass
        self.after(100, self._refresh_status_periodic)
