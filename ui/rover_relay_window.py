# ui/rover_relay_window.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from typing import Dict, Any

from utils.settings import ConfigManager, AppConfig  # type: ignore


class RoverRelaySettingsWindow(tk.Toplevel):
    """Configure rover feedback relay logging."""

    def __init__(self, master: tk.Misc, cfg: Dict[str, Any], rover, relay, log) -> None:
        super().__init__(master)
        self.title("Rover Relay Logging")
        self.resizable(False, False)

        self.cfg = cfg
        self.rover = rover
        self.relay = relay
        self.log = log

        rconf = cfg.get("rover", {})

        self.v_enabled = tk.BooleanVar(value=bool(rconf.get("enabled", True)))
        self.v_autostart = tk.BooleanVar(value=bool(rconf.get("autostart", False)))

        self.v_fb_listen_ip = tk.StringVar(value=rconf.get("feedback_listen_ip", "0.0.0.0"))
        self.v_fb_listen_port = tk.IntVar(value=int(rconf.get("feedback_listen_port", 18102)))
        self.v_fb_dest_ip = tk.StringVar(value=rconf.get("feedback_dest_ip", "127.0.0.1"))
        self.v_fb_dest_port = tk.IntVar(value=int(rconf.get("feedback_dest_port", 18103)))
        self.v_fb_log = tk.StringVar(value=rconf.get("feedback_log_path", ""))

        self._build_layout()
        self._refresh_status_periodic()

    # ------------------------------------------------------------------
    def _build_layout(self) -> None:
        pad = dict(padx=6, pady=4)
        frm = ttk.Frame(self, padding=12)
        frm.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frm, text="Feedback Relay").grid(row=0, column=0, columnspan=4, sticky="w", **pad)
        ttk.Label(frm, text="Listen IP").grid(row=1, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_fb_listen_ip, width=16).grid(row=1, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Port").grid(row=1, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_fb_listen_port, width=10).grid(row=1, column=3, sticky="w", **pad)

        ttk.Label(frm, text="Dest IP").grid(row=2, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_fb_dest_ip, width=16).grid(row=2, column=1, sticky="w", **pad)
        ttk.Label(frm, text="Port").grid(row=2, column=2, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_fb_dest_port, width=10).grid(row=2, column=3, sticky="w", **pad)

        ttk.Label(frm, text="Log Path").grid(row=3, column=0, sticky="e", **pad)
        ttk.Entry(frm, textvariable=self.v_fb_log, width=32).grid(row=3, column=1, columnspan=2, sticky="we", **pad)
        ttk.Button(frm, text="Browse", command=self.on_browse_fb_log).grid(row=3, column=3, sticky="w", **pad)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(row=4, column=0, columnspan=4, sticky="ew", pady=(8, 8))

        ttk.Checkbutton(frm, text="Enable rover relay logging", variable=self.v_enabled).grid(
            row=5, column=0, columnspan=2, sticky="w", **pad
        )
        ttk.Checkbutton(frm, text="Autostart on launch", variable=self.v_autostart).grid(
            row=5, column=2, columnspan=2, sticky="w", **pad
        )

        note = ttk.Label(
            frm,
            text="Gazebo logging must be stopped or disabled before starting rover logging.",
            foreground="#aa5500",
            wraplength=380,
            justify="left",
        )
        note.grid(row=6, column=0, columnspan=4, sticky="w", pady=(4, 8))

        btns = ttk.Frame(frm)
        btns.grid(row=7, column=0, columnspan=4, sticky="ew", pady=(4, 4))
        btns.columnconfigure(0, weight=1)
        ttk.Button(btns, text="Start", command=self.on_start).grid(row=0, column=0, padx=4)
        ttk.Button(btns, text="Stop", command=self.on_stop).grid(row=0, column=1, padx=4)
        ttk.Button(btns, text="Apply", command=self.on_apply).grid(row=0, column=2, padx=4)
        ttk.Button(btns, text="Save", command=self.on_save).grid(row=0, column=3, padx=4)
        ttk.Button(btns, text="Close", command=self.destroy).grid(row=0, column=4, padx=4)

        self.lbl_status1 = ttk.Label(frm, text="-")
        self.lbl_status1.grid(row=8, column=0, columnspan=4, sticky="w", **pad)
        self.lbl_status2 = ttk.Label(frm, text="-")
        self.lbl_status2.grid(row=9, column=0, columnspan=4, sticky="w", **pad)

    # ------------------------------------------------------------------
    def _collect_values(self) -> Dict[str, Any]:
        return {
            "enabled": bool(self.v_enabled.get()),
            "autostart": bool(self.v_autostart.get()),
            "feedback_listen_ip": self.v_fb_listen_ip.get().strip(),
            "feedback_listen_port": int(self.v_fb_listen_port.get()),
            "feedback_dest_ip": self.v_fb_dest_ip.get().strip(),
            "feedback_dest_port": int(self.v_fb_dest_port.get()),
            "feedback_log_path": self.v_fb_log.get().strip(),
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
    def on_browse_fb_log(self) -> None:
        self._browse_log(self.v_fb_log, "Select feedback log file")

    def _browse_log(self, var: tk.StringVar, title: str) -> None:
        current = var.get().strip()
        initial_dir = os.path.dirname(current) if current else ""
        initial_file = os.path.basename(current) if current else ""
        path = filedialog.asksaveasfilename(
            parent=self,
            title=title,
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            initialdir=initial_dir or None,
            initialfile=initial_file or None,
        )
        if path:
            var.set(path)

    def on_start(self) -> None:
        try:
            self._apply_to_runtime()
            self.rover.start()
            messagebox.showinfo("Rover Relay", "Rover relay logging started.")
        except Exception as e:
            messagebox.showerror("Error", f"Start failed:\n{e}")

    def on_stop(self) -> None:
        try:
            self.rover.stop()
            messagebox.showinfo("Rover Relay", "Rover relay logging stopped.")
        except Exception as e:
            messagebox.showerror("Error", f"Stop failed:\n{e}")

    def on_apply(self) -> None:
        try:
            self._apply_to_runtime()
            messagebox.showinfo("Rover Relay", "Settings applied.")
        except Exception as e:
            messagebox.showerror("Error", f"Apply failed:\n{e}")

    def on_save(self) -> None:
        try:
            self._apply_to_runtime()
            cm = ConfigManager()
            ac = cm.load()
            data = ac.to_dict()
            data.setdefault("rover", {}).update(self._collect_values())
            for obsolete in (
                "cmd_listen_ip",
                "cmd_listen_port",
                "cmd_dest_ip",
                "cmd_dest_port",
                "cmd_log_path",
            ):
                data["rover"].pop(obsolete, None)
            cm.save(AppConfig.from_dict(data))
            messagebox.showinfo("Saved", "Rover relay settings saved.")
        except Exception as e:
            messagebox.showerror("Error", f"Save failed:\n{e}")

    # ------------------------------------------------------------------
    def _refresh_status_periodic(self) -> None:
        try:
            status = self.rover.get_status() if hasattr(self.rover, "get_status") else {}
            activated = status.get("activated", False)
            enabled = status.get("enabled", self.v_enabled.get())
            fb_log_path = status.get("feedback_log_path") or self.v_fb_log.get()
            fb_active = bool(status.get("feedback_logging_active", False))
            fb_count = status.get("feedback_logged_count")
            fb_err = status.get("feedback_log_error") or ""

            parts = [
                f"Status: {'Running' if activated else 'Stopped'}",
                f"Enabled: {'Yes' if enabled else 'No'}",
            ]
            if fb_active:
                if isinstance(fb_count, (int, float)):
                    parts.append(f"FB logged {int(fb_count)}")
                else:
                    parts.append("FB logging")
            self.lbl_status1.configure(text=" | ".join(parts))

            msg = []
            if fb_err:
                msg.append(f"FB error: {fb_err}")
            if not msg:
                paths = []
                if fb_log_path:
                    paths.append(f"FB→{os.path.basename(fb_log_path) or fb_log_path}")
                if paths:
                    msg.append(", ".join(paths))
                else:
                    msg.append("No log files configured")
            self.lbl_status2.configure(text=" | ".join(msg))
        except Exception:
            pass
        self.after(1000, self._refresh_status_periodic)
