# ui/gimbal_window.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Dict, Any, List, Optional

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


class GimbalControlsWindow(tk.Toplevel):
    def __init__(self, master: tk.Misc, cfg: Dict[str, Any], gimbal, log: logging.Logger) -> None:
        super().__init__(master)
        self.title("Gimbal Controls")
        self.resizable(False, False)

        self.cfg = cfg
        self.gimbal = gimbal
        self.log = log

        gconf = cfg.get("gimbal", {})

        self.presets: List[Optional[Dict[str, Any]]] = self._normalize_presets(gconf.get("presets"))
        try:
            init_preset_idx = int(gconf.get("selected_preset", 0) or 0)
        except Exception:
            init_preset_idx = 0
        if not (0 <= init_preset_idx < MAX_SENSOR_PRESETS):
            init_preset_idx = 0
        self.v_preset_index = tk.IntVar(value=init_preset_idx)
        self.v_preset_name = tk.StringVar()
        self._preset_radios: List[ttk.Radiobutton] = []

        # Generator
        self.v_gen_ip   = tk.StringVar(value=gconf.get("generator_ip", "127.0.0.1"))
        self.v_gen_port = tk.IntVar(value=int(gconf.get("generator_port", 15020)))

        # Sensor
        init_sensor_code = int(gconf.get("sensor_type", 0))
        if not (0 <= init_sensor_code < len(SENSOR_TYPES)): init_sensor_code = 0
        self.v_sensor_type_combo = tk.StringVar(value=f"{init_sensor_code}: {SENSOR_TYPES[init_sensor_code]}")
        self.v_sensor_id = tk.IntVar(value=int(gconf.get("sensor_id", 0)))

        # Pose target
        self.v_pos_x = tk.DoubleVar(value=float(gconf.get("pos_x", 0.0)))
        self.v_pos_y = tk.DoubleVar(value=float(gconf.get("pos_y", 0.0)))
        self.v_pos_z = tk.DoubleVar(value=float(gconf.get("pos_z", 0.0)))
        self.v_roll  = tk.DoubleVar(value=float(gconf.get("init_roll_deg", 0.0)))
        self.v_pitch = tk.DoubleVar(value=float(gconf.get("init_pitch_deg", 0.0)))
        self.v_yaw   = tk.DoubleVar(value=float(gconf.get("init_yaw_deg", 0.0)))

        self.v_max_rate = tk.DoubleVar(value=float(gconf.get("max_rate_dps", 60.0)))
        self.v_power_on = tk.BooleanVar(value=bool(gconf.get("power_on", True)))

        # MAVLink Serial + IDs
        self.v_serial_port = tk.StringVar(value=gconf.get("serial_port", ""))
        self.v_baud        = tk.IntVar(value=int(gconf.get("serial_baud", 115200)))
        self.v_mav_sysid   = tk.IntVar(value=int(gconf.get("mav_sysid", 1)))
        self.v_mav_compid  = tk.IntVar(value=int(gconf.get("mav_compid", 154)))

        # Current pose (read-only view)
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
        self._apply_preset(init_preset_idx)
        self._sync_gimbal_config_metadata()
        self._refresh_status_periodic()

    # ---------- UI ----------
    def _build_layout(self) -> None:
        pad = dict(padx=6, pady=4)
        frm = ttk.Frame(self, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")

        row = 0

        net_box = ttk.Labelframe(frm, text="Network Sensor Control")
        net_box.grid(row=row, column=0, columnspan=4, sticky="ew", **pad)
        ttk.Label(net_box, text="IP").grid(row=0, column=0, sticky="e", padx=4, pady=2)
        ttk.Entry(net_box, textvariable=self.v_gen_ip, width=18).grid(row=0, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(net_box, text="Port").grid(row=0, column=2, sticky="e", padx=4, pady=2)
        ttk.Entry(net_box, textvariable=self.v_gen_port, width=8).grid(row=0, column=3, sticky="w", padx=4, pady=2)

        row += 1

        presets_box = ttk.Labelframe(frm, text="Sensor Presets")
        presets_box.grid(row=row, column=0, columnspan=4, sticky="ew", **pad)
        presets_box.columnconfigure(1, weight=1)

        rb_holder = ttk.Frame(presets_box)
        rb_holder.grid(row=0, column=0, columnspan=4, sticky="w", padx=4, pady=2)
        for idx in range(MAX_SENSOR_PRESETS):
            btn = ttk.Radiobutton(
                rb_holder,
                text=self._format_preset_label(idx),
                variable=self.v_preset_index,
                value=idx,
                command=lambda i=idx: self.on_select_preset(i),
            )
            btn.grid(row=0, column=idx, sticky="w", padx=2)
            self._preset_radios.append(btn)

        ttk.Label(presets_box, text="Name").grid(row=1, column=0, sticky="e", padx=4, pady=(2, 2))
        ttk.Entry(presets_box, textvariable=self.v_preset_name, width=24).grid(row=1, column=1, sticky="we", padx=4, pady=(2, 2))
        ttk.Button(presets_box, text="Save to Preset", command=self.on_save_preset).grid(row=1, column=2, sticky="w", padx=4, pady=(2, 2))
        ttk.Button(presets_box, text="Clear Preset", command=self.on_clear_preset).grid(row=1, column=3, sticky="w", padx=4, pady=(2, 2))

        row += 1

        # Sensor
        ttk.Label(frm, text="Sensor").grid(row=row, column=0, sticky="e", **pad)
        ttk.Combobox(frm, textvariable=self.v_sensor_type_combo, values=SENSOR_COMBO_VALUES, state="readonly", width=16)\
            .grid(row=row, column=1, sticky="w", **pad)
        ttk.Label(frm, text="ID").grid(row=row, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_sensor_id, width=8).grid(row=row, column=3, sticky="w", **pad)

        row += 1

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=4, sticky="ew", pady=(6, 6))
        row += 1

        # Target pose
        ttk.Label(frm, text="Target Pose (x,y,z,r,p,y) [m / deg]").grid(row=row, column=0, columnspan=4, sticky="w", **pad)
        row += 1
        ttk.Label(frm, text="x").grid(row=row, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_pos_x, width=10).grid(row=row, column=1, sticky="w", **pad)
        ttk.Label(frm, text="y").grid(row=row, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_pos_y, width=10).grid(row=row, column=3, sticky="w", **pad)
        row += 1
        ttk.Label(frm, text="z").grid(row=row, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_pos_z, width=10).grid(row=row, column=1, sticky="w", **pad)
        ttk.Label(frm, text="roll").grid(row=row, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_roll, width=10).grid(row=row, column=3, sticky="w", **pad)
        row += 1
        ttk.Label(frm, text="pitch").grid(row=row, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_pitch, width=10).grid(row=row, column=1, sticky="w", **pad)
        ttk.Label(frm, text="yaw").grid(row=row, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_yaw, width=10).grid(row=row, column=3, sticky="w", **pad)

        btns_pose = ttk.Frame(frm)
        btns_pose.grid(row=row + 1, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(btns_pose, text="Apply Pose", command=self.on_apply_pose).grid(row=0, column=0, padx=6)

        row += 2

        # Pose templates
        tpl = ttk.Frame(frm)
        tpl.grid(row=row, column=0, columnspan=4, sticky="we", **pad)
        ttk.Label(tpl, text="Templates:").pack(side=tk.LEFT)
        ttk.Button(tpl, text="Home",    command=lambda: self._set_tpl(0,0,0, 0,0,0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(tpl, text="Forward", command=lambda: self._set_tpl(None,None,None, 0,0,0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(tpl, text="Nadir",   command=lambda: self._set_tpl(None,None,None, 0,-90,0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(tpl, text="Left",    command=lambda: self._set_tpl(None,None,None, 0,0,-90)).pack(side=tk.LEFT, padx=2)
        ttk.Button(tpl, text="Right",   command=lambda: self._set_tpl(None,None,None, 0,0,90)).pack(side=tk.LEFT, padx=2)

        row += 1

        # Max rate
        ttk.Label(frm, text="Max Angular Rate (deg/s)").grid(row=row, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_max_rate, width=10).grid(row=row, column=1, sticky="w", **pad)
        ttk.Button(frm, text="Apply Max Rate", command=self.on_apply_max_rate).grid(row=row, column=3, sticky="e", **pad)

        row += 1

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=4, sticky="ew", pady=(6, 6))
        row += 1

        # Power
        ttk.Checkbutton(frm, text="Power ON", variable=self.v_power_on).grid(row=row, column=0, columnspan=2, sticky="w", **pad)
        ttk.Button(frm, text="Apply Power", command=self.on_apply_power).grid(row=row, column=3, sticky="e", **pad)

        row += 1

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=4, sticky="ew", pady=(6, 6))
        row += 1

        # MAVLink Serial + IDs
        ttk.Label(frm, text="MAVLink Serial").grid(row=row, column=0, columnspan=4, sticky="w", **pad)
        row += 1
        ttk.Label(frm, text="Port").grid(row=row, column=0, sticky="e", **pad)
        self.cb_ports = ttk.Combobox(frm, textvariable=self.v_serial_port, values=self._enum_serial_ports(), width=18)
        self.cb_ports.grid(row=row, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Baud").grid(row=row, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_baud, width=10).grid(row=row, column=3, sticky="w", **pad)

        row += 1
        ttk.Label(frm, text="System ID").grid(row=row, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_mav_sysid, width=10).grid(row=row, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Component ID").grid(row=row, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_mav_compid, width=10).grid(row=row, column=3, sticky="w", **pad)

        row += 1
        ids_btns = ttk.Frame(frm)
        ids_btns.grid(row=row, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(ids_btns, text="Apply IDs", command=self.on_apply_ids).grid(row=0, column=0, padx=6)
        ttk.Button(ids_btns, text="Refresh Ports", command=self.on_refresh_ports).grid(row=0, column=1, padx=6)
        ttk.Button(ids_btns, text="Connect Serial", command=self.on_connect_serial).grid(row=0, column=2, padx=6)

        row += 1
        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=4, sticky="ew", pady=(6, 6))

        row += 1

        # Current pose (live)
        cur = ttk.Frame(frm)
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

        # Status line
        self.lbl_status = ttk.Label(frm, text="Status: -")
        self.lbl_status.grid(row=row + 1, column=0, columnspan=4, sticky="w", **pad)

        btns = ttk.Frame(frm)
        btns.grid(row=row + 2, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(btns, text="Save", command=self.on_save).grid(row=0, column=0, padx=6)
        ttk.Button(btns, text="Apply & Close", command=self.on_apply_close).grid(row=0, column=1, padx=6)

    # ---------- helpers ----------
    def _enum_serial_ports(self):
        try:
            return [p.device for p in list_ports.comports()]
        except Exception:
            return []

    def _collect_common_values(self) -> Dict[str, Any]:
        try:
            port = int(self.v_gen_port.get())
        except Exception:
            try:
                port = int(self.cfg.get("gimbal", {}).get("generator_port", 15020))
            except Exception:
                port = 15020
        return {
            "generator_ip": self.v_gen_ip.get(),
            "generator_port": port,
        }

    def _collect_values(self) -> Dict[str, Any]:
        combo = (self.v_sensor_type_combo.get() or "0: Camera").strip()
        try:
            sensor_code = int(combo.split(":")[0])
        except Exception:
            sensor_code = 0
        values = {
            "sensor_type": sensor_code,
            "sensor_id": int(self.v_sensor_id.get()),
            "pos_x": float(self.v_pos_x.get()),
            "pos_y": float(self.v_pos_y.get()),
            "pos_z": float(self.v_pos_z.get()),
            "init_roll_deg": float(self.v_roll.get()),
            "init_pitch_deg": float(self.v_pitch.get()),
            "init_yaw_deg": float(self.v_yaw.get()),
            "max_rate_dps": float(self.v_max_rate.get()),
            "power_on": bool(self.v_power_on.get()),
            "serial_port": self.v_serial_port.get().strip(),
            "serial_baud": int(self.v_baud.get()),
            "mav_sysid": int(self.v_mav_sysid.get()),
            "mav_compid": int(self.v_mav_compid.get()),
        }
        values.update(self._collect_common_values())
        return values

    def _collect_preset_values(self) -> Dict[str, Any]:
        values = self._collect_values()
        return {k: values[k] for k in PRESET_VALUE_KEYS if k in values}

    def _apply_values(self, values: Dict[str, Any]) -> None:
        if "generator_ip" in values:
            self.v_gen_ip.set(str(values["generator_ip"]))
        if "generator_port" in values:
            try:
                self.v_gen_port.set(int(values["generator_port"]))
            except Exception:
                pass
        if "sensor_type" in values:
            try:
                code = int(values["sensor_type"])
            except Exception:
                code = 0
            if 0 <= code < len(SENSOR_COMBO_VALUES):
                self.v_sensor_type_combo.set(SENSOR_COMBO_VALUES[code])
            else:
                self.v_sensor_type_combo.set(f"{code}: Sensor")
        if "sensor_id" in values:
            try:
                self.v_sensor_id.set(int(values["sensor_id"]))
            except Exception:
                pass
        if "pos_x" in values:
            self.v_pos_x.set(float(values["pos_x"]))
        if "pos_y" in values:
            self.v_pos_y.set(float(values["pos_y"]))
        if "pos_z" in values:
            self.v_pos_z.set(float(values["pos_z"]))
        if "init_roll_deg" in values:
            self.v_roll.set(float(values["init_roll_deg"]))
        if "init_pitch_deg" in values:
            self.v_pitch.set(float(values["init_pitch_deg"]))
        if "init_yaw_deg" in values:
            self.v_yaw.set(float(values["init_yaw_deg"]))
        if "max_rate_dps" in values:
            self.v_max_rate.set(float(values["max_rate_dps"]))
        if "power_on" in values:
            self.v_power_on.set(bool(values["power_on"]))
        if "serial_port" in values:
            self.v_serial_port.set(str(values["serial_port"]))
        if "serial_baud" in values:
            try:
                self.v_baud.set(int(values["serial_baud"]))
            except Exception:
                pass
        if "mav_sysid" in values:
            try:
                self.v_mav_sysid.set(int(values["mav_sysid"]))
            except Exception:
                pass
        if "mav_compid" in values:
            try:
                self.v_mav_compid.set(int(values["mav_compid"]))
            except Exception:
                pass

    def _default_preset_name(self, idx: int) -> str:
        return f"Preset {idx + 1}"

    def _format_preset_label(self, idx: int) -> str:
        preset = self.presets[idx]
        base = self._default_preset_name(idx)
        if preset and isinstance(preset, dict):
            name = str(preset.get("name", "")).strip() or base
            return name
        return base

    def _normalize_presets(self, raw: Any) -> List[Optional[Dict[str, Any]]]:
        presets: List[Optional[Dict[str, Any]]] = []
        slots: List[Any] = []
        if isinstance(raw, dict):
            slots_raw = raw.get("slots")
            if isinstance(slots_raw, list):
                slots = slots_raw
            elif isinstance(slots_raw, dict):
                keys = set(slots_raw.keys())
                zero_based = any(k in keys for k in (0, "0"))
                for i in range(MAX_SENSOR_PRESETS):
                    candidates = []
                    if zero_based:
                        candidates.extend([i, str(i)])
                    else:
                        candidates.extend([i + 1, str(i + 1)])
                    candidates.extend([i, str(i), i + 1, str(i + 1)])
                    val = None
                    for key in candidates:
                        if key in slots_raw:
                            val = slots_raw[key]
                            break
                    slots.append(val)
        elif isinstance(raw, list):
            slots = raw
        for item in slots[:MAX_SENSOR_PRESETS]:
            presets.append(self._normalize_single_preset(item))
        while len(presets) < MAX_SENSOR_PRESETS:
            presets.append(None)
        return presets

    def _normalize_single_preset(self, item: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(item, dict):
            return None
        name = str(item.get("name", "")).strip()
        values = item.get("values")
        if not isinstance(values, dict):
            values = {k: item.get(k) for k in PRESET_VALUE_KEYS if k in item}
        if not values:
            return None
        filtered = {k: values[k] for k in PRESET_VALUE_KEYS if k in values}
        if not filtered:
            return None
        return {"name": name, "values": filtered}

    def _sync_preset_widgets(self) -> None:
        for idx, btn in enumerate(self._preset_radios):
            btn.configure(text=self._format_preset_label(idx))
        idx = int(self.v_preset_index.get())
        preset = self.presets[idx]
        if preset:
            name = str(preset.get("name", "")).strip() or self._default_preset_name(idx)
        else:
            name = self._default_preset_name(idx)
        self.v_preset_name.set(name)

    def _apply_preset(self, idx: int) -> None:
        if not (0 <= idx < MAX_SENSOR_PRESETS):
            idx = 0
            self.v_preset_index.set(idx)
        preset = self.presets[idx]
        if preset and isinstance(preset.get("values"), dict):
            self._apply_values(preset["values"])
        self._sync_preset_widgets()

    def _export_presets(self) -> Dict[str, Any]:
        slots: List[Optional[Dict[str, Any]]] = []
        for preset in self.presets:
            if preset and isinstance(preset.get("values"), dict):
                slots.append({
                    "name": str(preset.get("name", "")),
                    "values": {
                        k: preset["values"].get(k) for k in PRESET_VALUE_KEYS if k in preset["values"]
                    },
                })
            else:
                slots.append(None)
        return {"version": 1, "slots": slots}

    def _sync_gimbal_config_metadata(self) -> None:
        gconf = self.cfg.setdefault("gimbal", {})
        gconf.update(self._collect_common_values())
        gconf["presets"] = self._export_presets()
        gconf["selected_preset"] = int(self.v_preset_index.get())

    def _inject_presets_into_dict(self, target: Dict[str, Any]) -> None:
        target.update(self._collect_common_values())
        target["presets"] = self._export_presets()
        target["selected_preset"] = int(self.v_preset_index.get())

    # ---------- actions ----------
    def _set_tpl(self, x, y, z, r, p, yw):
        # None 인 값은 현 값을 유지
        if x is not None:  self.v_pos_x.set(x)
        if y is not None:  self.v_pos_y.set(y)
        if z is not None:  self.v_pos_z.set(z)
        if r is not None:  self.v_roll.set(r)
        if p is not None:  self.v_pitch.set(p)
        if yw is not None: self.v_yaw.set(yw)

    def on_select_preset(self, idx: int) -> None:
        self.v_preset_index.set(idx)
        self._apply_preset(idx)
        self.cfg.setdefault("gimbal", {}).update(self._collect_values())
        self._sync_gimbal_config_metadata()

    def on_save_preset(self) -> None:
        idx = int(self.v_preset_index.get())
        try:
            preset_values = self._collect_preset_values()
        except Exception as e:
            messagebox.showerror("Error", f"Save preset failed:\n{e}")
            return
        name = self.v_preset_name.get().strip() or self._default_preset_name(idx)
        self.presets[idx] = {"name": name, "values": preset_values}
        self.v_preset_name.set(name)
        self.cfg.setdefault("gimbal", {}).update(self._collect_values())
        self._sync_preset_widgets()
        self._sync_gimbal_config_metadata()
        messagebox.showinfo("Saved", f"Preset {idx + 1} saved.")

    def on_clear_preset(self) -> None:
        idx = int(self.v_preset_index.get())
        self.presets[idx] = None
        self._sync_preset_widgets()
        self.cfg.setdefault("gimbal", {}).update(self._collect_values())
        self._sync_gimbal_config_metadata()
        messagebox.showinfo("Cleared", f"Preset {idx + 1} cleared.")

    def on_apply_pose(self) -> None:
        try:
            v = self._collect_values()
            self.cfg.setdefault("gimbal", {}).update(v)
            self._sync_gimbal_config_metadata()
            self.gimbal.update_settings(v)
            self.gimbal.set_target_pose(v["pos_x"], v["pos_y"], v["pos_z"],
                                        v["init_roll_deg"], v["init_pitch_deg"], v["init_yaw_deg"])
            messagebox.showinfo("OK", "Target pose applied (controller will drive to target).")
        except Exception as e:
            messagebox.showerror("Error", f"Apply pose failed:\n{e}")

    def on_apply_max_rate(self) -> None:
        try:
            v = self._collect_values()
            self.cfg.setdefault("gimbal", {}).update(v)
            self._sync_gimbal_config_metadata()
            self.gimbal.update_settings(v)
            self.gimbal.set_max_rate(v["max_rate_dps"])
            messagebox.showinfo("OK", "Max angular rate updated.")
        except Exception as e:
            messagebox.showerror("Error", f"Apply max rate failed:\n{e}")

    def on_apply_power(self) -> None:
        try:
            on = bool(self.v_power_on.get())
            self.gimbal.send_power(on)

            # (선택) 현재 UI 값을 설정에 반영해 두고 싶다면:
            # v = self._collect_values()
            # self.cfg.setdefault("gimbal", {}).update(v)

            messagebox.showinfo("OK", f"Power {'ON' if on else 'OFF'} applied.")
        except Exception as e:
            messagebox.showerror("Error", f"Apply Power failed:\n{e}")

    def on_send_power_once(self) -> None:
        try:
            on = bool(self.v_power_on.get())
            self.gimbal.send_power(on)
        except Exception as e:
            messagebox.showerror("Error", f"Send power failed:\n{e}")

    def on_connect_serial(self) -> None:
        try:
            v = self._collect_values()
            self.cfg.setdefault("gimbal", {}).update(v)
            self._sync_gimbal_config_metadata()
            self.gimbal.update_settings(v)
            self.gimbal.open_serial(v["serial_port"], v["serial_baud"])
            messagebox.showinfo("OK", f"Serial connected: {v['serial_port']} @ {v['serial_baud']}")
        except Exception as e:
            messagebox.showerror("Error", f"Serial connect failed:\n{e}")

    def on_refresh_ports(self) -> None:
        try:
            self.cb_ports["values"] = self._enum_serial_ports()
        except Exception as e:
            messagebox.showerror("Error", f"Port refresh failed:\n{e}")

    def on_apply_ids(self) -> None:
        try:
            sysid = int(self.v_mav_sysid.get())
            compid = int(self.v_mav_compid.get())
            self.gimbal.set_mav_ids(sysid, compid)
            messagebox.showinfo("OK", f"MAV IDs applied: sys={sysid}, comp={compid}")
        except Exception as e:
            messagebox.showerror("Error", f"Apply IDs failed:\n{e}")

    def on_save(self) -> None:
        try:
            v = self._collect_values()
            self.cfg.setdefault("gimbal", {}).update(v)
            self._sync_gimbal_config_metadata()
            cm = ConfigManager(); ac = cm.load(); d = ac.to_dict()
            d.setdefault("gimbal", {}).update(v)
            self._sync_gimbal_config_metadata()
            self._inject_presets_into_dict(d.setdefault("gimbal", {}))
            cm.save(AppConfig.from_dict(d))
            messagebox.showinfo("Saved", "Gimbal settings saved.")
        except Exception as e:
            messagebox.showerror("Error", f"Save failed:\n{e}")

    def on_apply_close(self) -> None:
        try:
            v = self._collect_values()
            self.cfg.setdefault("gimbal", {}).update(v)
            self._sync_gimbal_config_metadata()
            self.gimbal.update_settings(v)
        except Exception as e:
            messagebox.showerror("Error", f"Apply failed:\n{e}")
            return
        self.destroy()

    # ---------- status (live) ----------
    def _refresh_status_periodic(self) -> None:
        try:
            st = self.gimbal.get_status() if hasattr(self.gimbal, "get_status") else {}
            act = st.get("activated", True)
            mode = st.get("control_mode", "CTRL")
            r = st.get("current_roll_deg", 0.0); p = st.get("current_pitch_deg", 0.0); y = st.get("current_yaw_deg", 0.0)
            x = st.get("current_x", 0.0); yx = st.get("current_y", 0.0); z = st.get("current_z", 0.0)
            wx = st.get("wx", 0.0); wy = st.get("wy", 0.0); wz = st.get("wz", 0.0)
            mrate = st.get("max_rate_dps", self.v_max_rate.get())
            ser = st.get("serial_state", "-"); hb = st.get("hb_rx_ok", False)
            sysid = st.get("mav_sysid", self.v_mav_sysid.get()); compid = st.get("mav_compid", self.v_mav_compid.get())

            # 라벨 갱신
            self.cur_x.set(f"{x:.2f}"); self.cur_y.set(f"{yx:.2f}"); self.cur_z.set(f"{z:.2f}")
            self.cur_r.set(f"{r:.1f}"); self.cur_p.set(f"{p:.1f}");  self.cur_yw.set(f"{y:.1f}")
            self.cur_wx.set(f"{wx:.2f}"); self.cur_wy.set(f"{wy:.2f}"); self.cur_wz.set(f"{wz:.2f}")

            self.lbl_status.configure(
                text=f"Status: {'On' if act else 'Off'} | Mode={mode} | Sys/Comp={sysid}/{compid} | MaxRate={mrate:.1f} dps | Serial={ser} | HB={hb}"
            )
        except Exception:
            pass
        self.after(100, self._refresh_status_periodic)
