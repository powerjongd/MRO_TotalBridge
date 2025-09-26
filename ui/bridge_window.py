# ui/bridge_window.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
import os
import tkinter as tk
from tkinter import ttk, messagebox, StringVar, IntVar

from utils.settings import ConfigManager, AppConfig  # type: ignore


class BridgeSettingsWindow(tk.Toplevel):
    """
    카메라 브릿지(기본 TCP/UDP 서버) 네트워킹 설정 팝업.
    - IP, TCP Port, UDP Port, Images 경로
    - Save: 디스크 저장
    - Apply & Close: 런타임 bridge.update_settings() 반영 후 닫기
    """
    def __init__(self, master: tk.Misc, cfg: dict, bridge, log: logging.Logger) -> None:
        super().__init__(master)
        self.title("Bridge Settings")
        self.resizable(False, False)

        self.cfg = cfg
        self.bridge = bridge
        self.log = log

        bconf = cfg.get("bridge", {})
        self.v_ip = StringVar(value=bconf.get("ip", "0.0.0.0"))
        self.v_tcp = IntVar(value=int(bconf.get("tcp_port", 9999)))
        self.v_udp = IntVar(value=int(bconf.get("udp_port", 9998)))
        self.v_images = StringVar(value=bconf.get("images", "./images"))

        self._build_layout()
        self._bind_events()
        self._refresh_status()

    # ---------------- UI ----------------
    def _build_layout(self) -> None:
        pad = dict(padx=6, pady=4)
        frm = ttk.Frame(self, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frm, text="Bridge Network").grid(row=0, column=0, columnspan=4, sticky="w", **pad)

        ttk.Label(frm, text="IP").grid(row=1, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_ip, width=18).grid(row=1, column=1, sticky="w", **pad)

        ttk.Label(frm, text="TCP Port").grid(row=1, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_tcp, width=10).grid(row=1, column=3, sticky="w", **pad)

        ttk.Label(frm, text="UDP Port").grid(row=2, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_udp, width=10).grid(row=2, column=3, sticky="w", **pad)

        ttk.Label(frm, text="Images Dir").grid(row=2, column=0, sticky="e", **pad)
        ent_images = ttk.Entry(frm, textvariable=self.v_images, width=30)
        ent_images.grid(row=2, column=1, sticky="w", **pad)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=3, column=0, columnspan=4, sticky="ew", pady=(8, 8))
        self.lbl_status = ttk.Label(frm, text="Status: -")
        self.lbl_status.grid(row=4, column=0, columnspan=4, sticky="w", **pad)

        btns = ttk.Frame(frm)
        btns.grid(row=5, column=0, columnspan=4, sticky="e", **pad)
        ttk.Button(btns, text="Save", command=self.on_save).grid(row=0, column=0, padx=6)
        ttk.Button(btns, text="Apply & Close", command=self.on_apply_close).grid(row=0, column=1, padx=6)

    def _bind_events(self) -> None:
        self.bind("<Return>", lambda _e: self.on_apply_close())

    # ---------------- 상태 ----------------
    def _refresh_status(self) -> None:
        running = getattr(self.bridge, "is_server_running", None) and self.bridge.is_server_running.is_set()
        ip = self.v_ip.get()
        tcp = self.v_tcp.get()
        udp = self.v_udp.get()
        self.lbl_status.configure(text=f"Status: {'Running' if running else 'Stopped'} | {ip} / TCP:{tcp} UDP:{udp}")

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

        images = self.v_images.get().strip()
        if images and not os.path.isdir(images):
            try:
                os.makedirs(images, exist_ok=True)
            except Exception as e:
                raise ValueError(f"Failed to create images dir: {e}")

        return {
            "ip": ip,
            "tcp_port": tcp,
            "udp_port": udp,
            "images": images or "./images",
        }

    def _apply_to_runtime(self) -> None:
        values = self._collect_values()
        self.cfg.setdefault("bridge", {}).update(values)
        try:
            self.bridge.update_settings(self.cfg["bridge"])
            self._refresh_status()
        except Exception as e:
            messagebox.showerror("Error", f"Bridge apply failed:\n{e}")

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
            messagebox.showinfo("Saved", "Bridge settings saved.")
        except Exception as e:
            messagebox.showerror("Error", f"Save failed:\n{e}")

    def on_apply_close(self) -> None:
        self._apply_to_runtime()
        self.destroy()
