# ui/bridge_window.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
import os
import tkinter as tk
from tkinter import ttk, messagebox, StringVar, IntVar
from typing import Tuple, Optional

from ui.gimbal_window import SENSOR_COMBO_VALUES

from utils.settings import ConfigManager, AppConfig  # type: ignore


class BridgeSettingsWindow(tk.Toplevel):
    """
    영상 스트리밍(ImageStreamBridge) 모듈 설정 팝업.
    - IP, TCP Port, UDP Port
    - 이미지 라이브러리 선택 (Realtime SaveFile / PreDefinedImageSet)
    - Save/Apply
    """
    def __init__(
        self,
        master: tk.Misc,
        cfg: dict,
        bridge,
        log: logging.Logger,
        *,
        gimbal: Optional[object] = None,
    ) -> None:
        super().__init__(master)
        self.title("Image Stream Module Settings")
        self.resizable(False, False)

        self.cfg = cfg
        self.bridge = bridge
        self.gimbal = gimbal
        self.log = log

        bconf = cfg.get("bridge", {})
        self.v_ip = StringVar(value=bconf.get("ip", "0.0.0.0"))
        self.v_tcp = IntVar(value=int(bconf.get("tcp_port", 9999)))
        self.v_udp = IntVar(value=int(bconf.get("udp_port", 9998)))
        self.v_mode = StringVar(value=(bconf.get("image_source_mode") or "realtime").lower())
        self.v_realtime_dir = StringVar(
            value=bconf.get("realtime_dir", bconf.get("images", "./SaveFile"))
        )
        self.v_predefined_dir = StringVar(
            value=bconf.get("predefined_dir", "./PreDefinedImageSet")
        )
        self.v_sensor_type = StringVar(value=self._sensor_combo_label(int(bconf.get("gimbal_sensor_type", 0))))
        self.v_sensor_id = IntVar(value=int(bconf.get("gimbal_sensor_id", 0)))
        forward_ip, forward_port = self._resolve_gimbal_endpoint()
        self.v_forward_ip = StringVar(value=forward_ip)
        self.v_forward_port = IntVar(value=forward_port)

        self._build_layout()
        self._bind_events()
        self._refresh_status()

    # ---------------- UI ----------------
    def _build_layout(self) -> None:
        pad = dict(padx=6, pady=4)
        frm = ttk.Frame(self, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frm, text="Image Stream Module Network").grid(row=0, column=0, columnspan=4, sticky="w", **pad)

        ttk.Label(frm, text="IP").grid(row=1, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_ip, width=18).grid(row=1, column=1, sticky="w", **pad)

        ttk.Label(frm, text="TCP Port").grid(row=1, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_tcp, width=10).grid(row=1, column=3, sticky="w", **pad)

        ttk.Label(frm, text="UDP Port").grid(row=2, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_udp, width=10).grid(row=2, column=3, sticky="w", **pad)

        gimbal_box = ttk.Labelframe(frm, text="Gimbal Forwarding")
        gimbal_box.grid(row=3, column=0, columnspan=4, sticky="ew", padx=6, pady=(12, 0))
        ttk.Label(gimbal_box, text="Sensor Type").grid(row=0, column=0, sticky="e", padx=4, pady=2)
        ttk.Combobox(
            gimbal_box,
            textvariable=self.v_sensor_type,
            values=SENSOR_COMBO_VALUES,
            state="readonly",
            width=20,
        ).grid(row=0, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(gimbal_box, text="Sensor ID").grid(row=0, column=2, sticky="e", padx=4, pady=2)
        ttk.Entry(gimbal_box, textvariable=self.v_sensor_id, width=8).grid(row=0, column=3, sticky="w", padx=4, pady=2)

        ttk.Label(gimbal_box, text="Target IP").grid(row=1, column=0, sticky="e", padx=4, pady=2)
        ttk.Entry(gimbal_box, textvariable=self.v_forward_ip, width=18, state="readonly").grid(row=1, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(gimbal_box, text="Target Port").grid(row=1, column=2, sticky="e", padx=4, pady=2)
        ttk.Entry(gimbal_box, textvariable=self.v_forward_port, width=8, state="readonly").grid(row=1, column=3, sticky="w", padx=4, pady=2)

        ttk.Label(
            gimbal_box,
            text="IP/Port 변경은 Gimbal Controls 창에서 수행해주세요.",
            foreground="#555555",
            wraplength=320,
        ).grid(row=2, column=0, columnspan=4, sticky="w", padx=4, pady=(0, 4))

        ttk.Label(frm, text="Image Library Source").grid(row=4, column=0, columnspan=4, sticky="w", pady=(12, 0), padx=6)
        mode_frame = ttk.Frame(frm)
        mode_frame.grid(row=5, column=0, columnspan=4, sticky="w", padx=6)
        ttk.Radiobutton(
            mode_frame,
            text="Use Realtime ImageSet (SaveFile)",
            value="realtime",
            variable=self.v_mode,
        ).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(
            mode_frame,
            text="Use PreDefined ImageSet",
            value="predefined",
            variable=self.v_mode,
        ).grid(row=1, column=0, sticky="w", pady=(2, 0))

        ttk.Label(frm, text="SaveFile Dir").grid(row=6, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_realtime_dir, width=30).grid(row=6, column=1, sticky="w", **pad)
        ttk.Label(frm, text="PreDefined Dir").grid(row=6, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_predefined_dir, width=30).grid(row=6, column=3, sticky="w", **pad)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=7, column=0, columnspan=4, sticky="ew", pady=(8, 8))
        self.lbl_status = ttk.Label(frm, text="Status: -")
        self.lbl_status.grid(row=8, column=0, columnspan=4, sticky="w", **pad)

        btns = ttk.Frame(frm)
        btns.grid(row=9, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(btns, text="Save", command=self.on_save).grid(row=0, column=0, padx=6)
        ttk.Button(btns, text="Apply & Close", command=self.on_apply_close).grid(row=0, column=1, padx=6)

    def _bind_events(self) -> None:
        self.bind("<Return>", lambda _e: self.on_apply_close())
        try:
            self.v_mode.trace_add("write", lambda *_: self._refresh_status())
        except Exception:
            pass

    # ---------------- 상태 ----------------
    def _refresh_status(self) -> None:
        running = getattr(self.bridge, "is_server_running", None) and self.bridge.is_server_running.is_set()
        ip = self.v_ip.get()
        tcp = self.v_tcp.get()
        udp = self.v_udp.get()
        mode = self.v_mode.get() or "realtime"
        active_dir = self.v_realtime_dir.get() if mode == "realtime" else self.v_predefined_dir.get()
        self._refresh_gimbal_endpoint()
        self.lbl_status.configure(
            text=(
                f"Status: {'Running' if running else 'Stopped'} | {ip} / TCP:{tcp} UDP:{udp}"
                f" | Mode: {mode.capitalize()} ({active_dir})"
            )
        )

    # ---------------- 설정 I/O ----------------
    def _collect_values(self) -> dict:
        # 간단한 검증
        ip = self.v_ip.get().strip()
        if not ip:
            raise ValueError("IP must not be empty.")
        tcp = int(self.v_tcp.get())
        udp = int(self.v_udp.get())
        if not (0 < tcp < 65536 and 0 < udp < 65536):
            raise ValueError("TCP/UDP port must be 1..65535")

        mode = (self.v_mode.get() or "realtime").lower()
        if mode not in {"realtime", "predefined"}:
            mode = "realtime"

        realtime_dir = self.v_realtime_dir.get().strip() or "./SaveFile"
        predefined_dir = self.v_predefined_dir.get().strip() or "./PreDefinedImageSet"
        for p in (realtime_dir, predefined_dir):
            if p and not os.path.isdir(p):
                try:
                    os.makedirs(p, exist_ok=True)
                except Exception as e:
                    raise ValueError(f"Failed to create directory {p}: {e}")

        sensor_type = self._sensor_code_from_combo(self.v_sensor_type.get())
        try:
            sensor_id = int(self.v_sensor_id.get())
        except Exception:
            raise ValueError("Sensor ID must be an integer value.")

        return {
            "ip": ip,
            "tcp_port": tcp,
            "udp_port": udp,
            "realtime_dir": realtime_dir,
            "predefined_dir": predefined_dir,
            "image_source_mode": mode,
            "images": realtime_dir,  # legacy 호환
            "gimbal_sensor_type": sensor_type,
            "gimbal_sensor_id": sensor_id,
        }

    def _apply_to_runtime(self) -> None:
        values = self._collect_values()
        self.cfg.setdefault("bridge", {}).update(values)
        try:
            self.bridge.update_settings(self.cfg["bridge"])
            self.bridge.configure_gimbal_forwarding(
                values["gimbal_sensor_type"], values["gimbal_sensor_id"]
            )
            self._refresh_status()
        except Exception as e:
            messagebox.showerror("Error", f"Image Stream Module apply failed:\n{e}")

    # ---------------- 버튼 ----------------
    def on_save(self) -> None:
        try:
            values = self._collect_values()
            self.cfg.setdefault("bridge", {}).update(values)
            cm = ConfigManager()
            ac = cm.load()
            d = ac.to_dict()
            d.setdefault("bridge", {}).update(values)
            cm.save(AppConfig.from_dict(d))
            messagebox.showinfo("Saved", "Image Stream Module settings saved.")
        except Exception as e:
            messagebox.showerror("Error", f"Save failed:\n{e}")

    def on_apply_close(self) -> None:
        self._apply_to_runtime()
        self.destroy()

    def _sensor_combo_label(self, code: int) -> str:
        if 0 <= code < len(SENSOR_COMBO_VALUES):
            return SENSOR_COMBO_VALUES[code]
        return SENSOR_COMBO_VALUES[0]

    def _sensor_code_from_combo(self, combo: str) -> int:
        try:
            token = (combo or "0").split(":", 1)[0]
            return int(token)
        except Exception:
            return 0

    def _resolve_gimbal_endpoint(self) -> Tuple[str, int]:
        if self.gimbal is not None:
            try:
                return self.gimbal.get_generator_endpoint()
            except Exception:
                pass
        gimbal_cfg = self.cfg.get("gimbal", {})
        ip = str(gimbal_cfg.get("generator_ip", "127.0.0.1"))
        port = int(gimbal_cfg.get("generator_port", 15020))
        return ip, port

    def _refresh_gimbal_endpoint(self) -> None:
        ip, port = self._resolve_gimbal_endpoint()
        self.v_forward_ip.set(ip)
        self.v_forward_port.set(port)
