# ui/relay_window.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Dict, Any

from serial.tools import list_ports
from utils.settings import ConfigManager, AppConfig  # type: ignore


class RelaySettingsWindow(tk.Toplevel):
    """
    Gazebo UDP → ExternalCtrl UDP로 릴레이
               → Optical Flow 변환 후 공용 Serial로 MAVLink OPTICAL_FLOW 송신
    Image Generator의 Distance를 RAW UDP(기본) 또는 MAVLink로 수신 → 동일 Serial로 중계
    - Serial 포트는 OpticalFlow/Distance 모두 ‘공유’
    - 상태표시 2줄
    - Auto-Start on Launch
    - Optical Flow 스케일/품질 파라미터
    - OpticalFlow / Distance 각각의 Heartbeat 파라미터
    - Distance 수신 모드 선택(raw/mavlink)
    """

    def __init__(self, master: tk.Misc, cfg: Dict[str, Any], relay, log: logging.Logger) -> None:
        super().__init__(master)
        self.title("Gazebo Relay")
        self.resizable(False, False)

        self.cfg = cfg
        self.relay = relay
        self.log = log

        rconf = cfg.get("relay", {})

        # --- Gazebo 입력 (수신)
        self.v_gz_ip   = tk.StringVar(value=rconf.get("gazebo_listen_ip", "0.0.0.0"))
        self.v_gz_port = tk.IntVar(value=int(rconf.get("gazebo_listen_port", 17000)))

        # --- ExternalCtrl 출력 (원본 릴레이) — 기본 포트 9091
        self.v_ext_ip   = tk.StringVar(value=rconf.get("ext_udp_ip", "127.0.0.1"))
        self.v_ext_port = tk.IntVar(value=int(rconf.get("ext_udp_port", 9091)))

        # --- Distance 입력 (RAW 기본, 또는 MAVLink)
        self.v_dist_mode = tk.StringVar(value=(rconf.get("distance_mode", "raw")))
        self.v_dist_ip   = tk.StringVar(value=rconf.get("distance_udp_listen_ip", "0.0.0.0"))
        self.v_dist_port = tk.IntVar(value=int(rconf.get("distance_udp_listen_port", 14650)))

        # --- 공용 Serial (OpticalFlow + Distance out)
        self.v_serial_port = tk.StringVar(value=rconf.get("serial_port", ""))
        self.v_baud        = tk.IntVar(value=int(rconf.get("serial_baud", 115200)))
        self.v_flow_id     = tk.IntVar(value=int(rconf.get("flow_sensor_id", 0)))

        # --- Auto-start
        self.v_autostart = tk.BooleanVar(value=bool(rconf.get("autostart", False)))

        # --- Optical Flow 스케일 & 품질 모델 파라미터
        self.v_of_scale_pix   = tk.DoubleVar(value=float(rconf.get("of_scale_pix", 100.0)))
        self.v_q_base         = tk.IntVar(value=int(rconf.get("q_base", 255)))
        self.v_q_min          = tk.IntVar(value=int(rconf.get("q_min", 0)))
        self.v_q_max          = tk.IntVar(value=int(rconf.get("q_max", 255)))
        self.v_accel_thr      = tk.DoubleVar(value=float(rconf.get("accel_thresh", 5.0)))
        self.v_gyro_thr       = tk.DoubleVar(value=float(rconf.get("gyro_thresh", 2.0)))
        self.v_accel_penalty  = tk.DoubleVar(value=float(rconf.get("accel_penalty", 20.0)))  # per (|a|-thr)
        self.v_gyro_penalty   = tk.DoubleVar(value=float(rconf.get("gyro_penalty", 30.0)))   # per (|w|-thr)

        # --- Heartbeat 파라미터 (Optical Flow)
        self.v_hb_of_rate     = tk.DoubleVar(value=float(rconf.get("hb_of_rate_hz", 1.0)))
        self.v_hb_of_sysid    = tk.IntVar(value=int(rconf.get("hb_of_sysid", 42)))
        self.v_hb_of_compid   = tk.IntVar(value=int(rconf.get("hb_of_compid", 199)))
        self.v_hb_of_type     = tk.IntVar(value=int(rconf.get("hb_of_type", 18)))
        self.v_hb_of_autop    = tk.IntVar(value=int(rconf.get("hb_of_autopilot", 8)))
        self.v_hb_of_mode     = tk.IntVar(value=int(rconf.get("hb_of_base_mode", 0)))
        self.v_hb_of_cus      = tk.IntVar(value=int(rconf.get("hb_of_custom_mode", 0)))
        self.v_hb_of_stat     = tk.IntVar(value=int(rconf.get("hb_of_system_status", 4)))

        # --- Heartbeat 파라미터 (Distance Sensor)
        self.v_hb_ds_rate     = tk.DoubleVar(value=float(rconf.get("hb_ds_rate_hz", 1.0)))
        self.v_hb_ds_sysid    = tk.IntVar(value=int(rconf.get("hb_ds_sysid", 43)))
        self.v_hb_ds_compid   = tk.IntVar(value=int(rconf.get("hb_ds_compid", 200)))
        self.v_hb_ds_type     = tk.IntVar(value=int(rconf.get("hb_ds_type", 18)))
        self.v_hb_ds_autop    = tk.IntVar(value=int(rconf.get("hb_ds_autopilot", 8)))
        self.v_hb_ds_mode     = tk.IntVar(value=int(rconf.get("hb_ds_base_mode", 0)))
        self.v_hb_ds_cus      = tk.IntVar(value=int(rconf.get("hb_ds_custom_mode", 0)))
        self.v_hb_ds_stat     = tk.IntVar(value=int(rconf.get("hb_ds_system_status", 4)))

        self._build_layout()
        self._refresh_status_periodic()

    # ---------------- UI ----------------
    def _build_layout(self) -> None:
        pad = dict(padx=6, pady=4)
        frm = ttk.Frame(self, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")

        # --- Gazebo (input)
        ttk.Label(frm, text="Gazebo Input (UDP)").grid(row=0, column=0, columnspan=4, sticky="w", **pad)
        ttk.Label(frm, text="Listen IP").grid(row=1, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_gz_ip, width=16).grid(row=1, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Port").grid(row=1, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_gz_port, width=10).grid(row=1, column=3, sticky="w", **pad)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=2, column=0, columnspan=4, sticky="ew", pady=(6,6))

        # --- ExternalCtrl (output relay)
        ttk.Label(frm, text="ExternalCtrl Output (UDP Relay)").grid(row=3, column=0, columnspan=4, sticky="w", **pad)
        ttk.Label(frm, text="Dest IP").grid(row=4, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_ext_ip, width=16).grid(row=4, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Port").grid(row=4, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_ext_port, width=10).grid(row=4, column=3, sticky="w", **pad)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=5, column=0, columnspan=4, sticky="ew", pady=(6,6))

        # --- Distance (input from generator)
        ttk.Label(frm, text="Distance Sensor Input").grid(row=6, column=0, columnspan=4, sticky="w", **pad)
        ttk.Label(frm, text="Mode").grid(row=7, column=0, sticky="e", **pad)
        ttk.Combobox(frm, textvariable=self.v_dist_mode, values=["raw", "mavlink"], width=10, state="readonly").grid(row=7, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Listen IP").grid(row=8, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_dist_ip, width=16).grid(row=8, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Port").grid(row=8, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_dist_port, width=10).grid(row=8, column=3, sticky="w", **pad)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=9, column=0, columnspan=4, sticky="ew", pady=(6,6))

        # --- Shared Serial (OF + Distance out)
        ttk.Label(frm, text="Shared Serial (OpticalFlow + Distance Out)").grid(row=10, column=0, columnspan=4, sticky="w", **pad)
        ttk.Label(frm, text="Port").grid(row=11, column=0, sticky="e", **pad)
        self.cb_ports = ttk.Combobox(frm, textvariable=self.v_serial_port, values=self._enum_serial_ports(), width=18)
        self.cb_ports.grid(row=11, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Baud").grid(row=11, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_baud, width=10).grid(row=11, column=3, sticky="w", **pad)

        ttk.Label(frm, text="OpticalFlow Sensor ID").grid(row=12, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_flow_id, width=10).grid(row=12, column=1, sticky="w", **pad)

        # Auto-start
        ttk.Checkbutton(frm, text="Auto-Start on Launch", variable=self.v_autostart).grid(row=12, column=2, columnspan=2, sticky="w", **pad)

        btns_serial = ttk.Frame(frm)
        btns_serial.grid(row=13, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(btns_serial, text="Refresh Ports", command=self.on_refresh_ports).grid(row=0, column=0, padx=6)
        ttk.Button(btns_serial, text="Open Serial Now", command=self.on_open_serial).grid(row=0, column=1, padx=6)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=14, column=0, columnspan=4, sticky="ew", pady=(6,6))

        # --- Optical Flow Scale & Quality Model
        ttk.Label(frm, text="Optical Flow Parameters").grid(row=15, column=0, columnspan=4, sticky="w", **pad)
        ttk.Label(frm, text="Pixel Scale").grid(row=16, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_of_scale_pix, width=10).grid(row=16, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Q Base / Min / Max").grid(row=16, column=2, sticky="e", **pad)
        row16 = ttk.Frame(frm); row16.grid(row=16, column=3, sticky="w")
        ttk.Entry(row16, textvariable=self.v_q_base, width=4).pack(side=tk.LEFT, padx=(0,2))
        ttk.Entry(row16, textvariable=self.v_q_min,  width=4).pack(side=tk.LEFT, padx=(0,2))
        ttk.Entry(row16, textvariable=self.v_q_max,  width=4).pack(side=tk.LEFT)

        ttk.Label(frm, text="Accel Thr / Penalty").grid(row=17, column=0, sticky="e", **pad)
        row17 = ttk.Frame(frm); row17.grid(row=17, column=1, sticky="w")
        ttk.Entry(row17, textvariable=self.v_accel_thr,     width=8).pack(side=tk.LEFT, padx=(0,4))
        ttk.Entry(row17, textvariable=self.v_accel_penalty, width=8).pack(side=tk.LEFT)

        ttk.Label(frm, text="Gyro Thr / Penalty").grid(row=17, column=2, sticky="e", **pad)
        row17b = ttk.Frame(frm); row17b.grid(row=17, column=3, sticky="w")
        ttk.Entry(row17b, textvariable=self.v_gyro_thr,     width=8).pack(side=tk.LEFT, padx=(0,4))
        ttk.Entry(row17b, textvariable=self.v_gyro_penalty, width=8).pack(side=tk.LEFT)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=18, column=0, columnspan=4, sticky="ew", pady=(6,6))

        # --- Heartbeat params (Optical Flow)
        ttk.Label(frm, text="Heartbeat (Optical Flow)").grid(row=19, column=0, columnspan=4, sticky="w", **pad)
        ttk.Label(frm, text="Rate (Hz)").grid(row=20, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_hb_of_rate, width=8).grid(row=20, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Sys / Comp").grid(row=20, column=2, sticky="e", **pad)
        row20 = ttk.Frame(frm); row20.grid(row=20, column=3, sticky="w")
        ttk.Entry(row20, textvariable=self.v_hb_of_sysid,  width=5).pack(side=tk.LEFT, padx=(0,4))
        ttk.Entry(row20, textvariable=self.v_hb_of_compid, width=5).pack(side=tk.LEFT)

        ttk.Label(frm, text="Type / Autopilot").grid(row=21, column=0, sticky="e", **pad)
        row21 = ttk.Frame(frm); row21.grid(row=21, column=1, sticky="w")
        ttk.Entry(row21, textvariable=self.v_hb_of_type,  width=5).pack(side=tk.LEFT, padx=(0,4))
        ttk.Entry(row21, textvariable=self.v_hb_of_autop, width=5).pack(side=tk.LEFT)
        ttk.Label(frm, text="Base / Custom / Status").grid(row=21, column=2, sticky="e", **pad)
        row21b = ttk.Frame(frm); row21b.grid(row=21, column=3, sticky="w")
        ttk.Entry(row21b, textvariable=self.v_hb_of_mode, width=5).pack(side=tk.LEFT, padx=(0,2))
        ttk.Entry(row21b, textvariable=self.v_hb_of_cus,  width=5).pack(side=tk.LEFT, padx=(0,2))
        ttk.Entry(row21b, textvariable=self.v_hb_of_stat, width=5).pack(side=tk.LEFT)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=22, column=0, columnspan=4, sticky="ew", pady=(6,6))

        # --- Heartbeat params (Distance Sensor)
        ttk.Label(frm, text="Heartbeat (Distance Sensor)").grid(row=23, column=0, columnspan=4, sticky="w", **pad)
        ttk.Label(frm, text="Rate (Hz)").grid(row=24, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_hb_ds_rate, width=8).grid(row=24, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Sys / Comp").grid(row=24, column=2, sticky="e", **pad)
        row24 = ttk.Frame(frm); row24.grid(row=24, column=3, sticky="w")
        ttk.Entry(row24, textvariable=self.v_hb_ds_sysid,  width=5).pack(side=tk.LEFT, padx=(0,4))
        ttk.Entry(row24, textvariable=self.v_hb_ds_compid, width=5).pack(side=tk.LEFT)

        ttk.Label(frm, text="Type / Autopilot").grid(row=25, column=0, sticky="e", **pad)
        row25 = ttk.Frame(frm); row25.grid(row=25, column=1, sticky="w")
        ttk.Entry(row25, textvariable=self.v_hb_ds_type,  width=5).pack(side=tk.LEFT, padx=(0,4))
        ttk.Entry(row25, textvariable=self.v_hb_ds_autop, width=5).pack(side=tk.LEFT)
        ttk.Label(frm, text="Base / Custom / Status").grid(row=25, column=2, sticky="e", **pad)
        row25b = ttk.Frame(frm); row25b.grid(row=25, column=3, sticky="w")
        ttk.Entry(row25b, textvariable=self.v_hb_ds_mode, width=5).pack(side=tk.LEFT, padx=(0,2))
        ttk.Entry(row25b, textvariable=self.v_hb_ds_cus,  width=5).pack(side=tk.LEFT, padx=(0,2))
        ttk.Entry(row25b, textvariable=self.v_hb_ds_stat, width=5).pack(side=tk.LEFT)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=26, column=0, columnspan=4, sticky="ew", pady=(6,6))

        # --- Control
        ctrl = ttk.Frame(frm)
        ctrl.grid(row=27, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(ctrl, text="Start Relay", command=self.on_start).grid(row=0, column=0, padx=6)
        ttk.Button(ctrl, text="Stop Relay",  command=self.on_stop).grid(row=0, column=1, padx=6)

        # Save/Apply
        save = ttk.Frame(frm)
        save.grid(row=28, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(save, text="Save", command=self.on_save).grid(row=0, column=0, padx=6)
        ttk.Button(save, text="Apply & Close", command=self.on_apply_close).grid(row=0, column=1, padx=6)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=29, column=0, columnspan=4, sticky="ew", pady=(6,6))

        # --- Status (2 lines)
        self.lbl_status1 = ttk.Label(frm, text="Status: -")
        self.lbl_status1.grid(row=30, column=0, columnspan=4, sticky="w", **pad)
        self.lbl_status2 = ttk.Label(frm, text="-")
        self.lbl_status2.grid(row=31, column=0, columnspan=4, sticky="w", **pad)

    # ---------------- Helpers ----------------
    def _enum_serial_ports(self):
        try:
            return [p.device for p in list_ports.comports()]
        except Exception:
            return []

    def _collect_values(self) -> Dict[str, Any]:
        return {
            # Gazebo/Ext/Distance
            "gazebo_listen_ip": self.v_gz_ip.get(),
            "gazebo_listen_port": int(self.v_gz_port.get()),
            "ext_udp_ip": self.v_ext_ip.get(),
            "ext_udp_port": int(self.v_ext_port.get()),
            "distance_mode": self.v_dist_mode.get(),
            "distance_udp_listen_ip": self.v_dist_ip.get(),
            "distance_udp_listen_port": int(self.v_dist_port.get()),
            # Serial & Flow ID
            "serial_port": self.v_serial_port.get().strip(),
            "serial_baud": int(self.v_baud.get()),
            "flow_sensor_id": int(self.v_flow_id.get()),
            # Autostart
            "autostart": bool(self.v_autostart.get()),
            # OF scale & quality
            "of_scale_pix": float(self.v_of_scale_pix.get()),
            "q_base": int(self.v_q_base.get()),
            "q_min": int(self.v_q_min.get()),
            "q_max": int(self.v_q_max.get()),
            "accel_thresh": float(self.v_accel_thr.get()),
            "gyro_thresh": float(self.v_gyro_thr.get()),
            "accel_penalty": float(self.v_accel_penalty.get()),
            "gyro_penalty": float(self.v_gyro_penalty.get()),
            # HB OF
            "hb_of_rate_hz": float(self.v_hb_of_rate.get()),
            "hb_of_sysid": int(self.v_hb_of_sysid.get()),
            "hb_of_compid": int(self.v_hb_of_compid.get()),
            "hb_of_type": int(self.v_hb_of_type.get()),
            "hb_of_autopilot": int(self.v_hb_of_autop.get()),
            "hb_of_base_mode": int(self.v_hb_of_mode.get()),
            "hb_of_custom_mode": int(self.v_hb_of_cus.get()),
            "hb_of_system_status": int(self.v_hb_of_stat.get()),
            # HB Distance
            "hb_ds_rate_hz": float(self.v_hb_ds_rate.get()),
            "hb_ds_sysid": int(self.v_hb_ds_sysid.get()),
            "hb_ds_compid": int(self.v_hb_ds_compid.get()),
            "hb_ds_type": int(self.v_hb_ds_type.get()),
            "hb_ds_autopilot": int(self.v_hb_ds_autop.get()),
            "hb_ds_base_mode": int(self.v_hb_ds_mode.get()),
            "hb_ds_custom_mode": int(self.v_hb_ds_cus.get()),
            "hb_ds_system_status": int(self.v_hb_ds_stat.get()),
        }

    def _apply_to_runtime(self) -> None:
        v = self._collect_values()
        self.cfg.setdefault("relay", {}).update(v)
        self.relay.update_settings(v)

    # ---------------- Actions ----------------
    def on_refresh_ports(self) -> None:
        try:
            self.cb_ports["values"] = self._enum_serial_ports()
        except Exception as e:
            messagebox.showerror("Error", f"Port refresh failed:\n{e}")

    def on_open_serial(self) -> None:
        try:
            self._apply_to_runtime()
            self.relay._open_serial_shared()
            messagebox.showinfo("OK", f"Serial open attempted on {self.v_serial_port.get()} @ {self.v_baud.get()}")
        except Exception as e:
            messagebox.showerror("Error", f"Open serial failed:\n{e}")

    def on_start(self) -> None:
        try:
            self._apply_to_runtime()
            self.relay.start()
            messagebox.showinfo("Relay", "Relay started.")
        except Exception as e:
            messagebox.showerror("Error", f"Start failed:\n{e}")

    def on_stop(self) -> None:
        try:
            self.relay.stop()
            messagebox.showinfo("Relay", "Relay stopped.")
        except Exception as e:
            messagebox.showerror("Error", f"Stop failed:\n{e}")

    def on_save(self) -> None:
        try:
            self._apply_to_runtime()
            cm = ConfigManager(); ac = cm.load(); d = ac.to_dict()
            d.setdefault("relay", {}).update(self._collect_values())
            cm.save(AppConfig.from_dict(d))
            messagebox.showinfo("Saved", "Relay settings saved.")
        except Exception as e:
            messagebox.showerror("Error", f"Save failed:\n{e}")

    def on_apply_close(self) -> None:
        try:
            self._apply_to_runtime()
        except Exception as e:
            messagebox.showerror("Error", f"Apply failed:\n{e}")
            return
        self.destroy()

    # ---------------- Status (2 lines) ----------------
    def _refresh_status_periodic(self) -> None:
        try:
            st = self.relay.get_status() if hasattr(self.relay, "get_status") else {}
            act = st.get("activated", False)
            dist = st.get("distance_m", 0.0)
            dage = st.get("distance_age_s", -1.0)
            q = st.get("of_quality", 0)
            h = st.get("of_ground_dist", 0.0)
            of_age = st.get("of_last_send_age_s", -1.0)
            serial = st.get("serial", "-")
            gz = st.get("gazebo_listen", "-")
            ext = st.get("ext_dst", "-")
            dup = st.get("dist_udp", "-")

            line1 = (
                f"Status: {'Running' if act else 'Stopped'} | "
                f"Dist={dist:.2f} m (age {dage:.1f}s) | "
                f"OF q={q} h={h:.2f} (age {of_age:.1f}s)"
            )
            line2 = f"GZ={gz} → EXT={ext} | UDPdist={dup} | COM={serial}"
            self.lbl_status1.configure(text=line1)
            self.lbl_status2.configure(text=line2)
        except Exception:
            pass
        self.after(1000, self._refresh_status_periodic)
