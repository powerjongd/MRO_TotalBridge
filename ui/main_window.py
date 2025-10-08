# ui/main_window.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from PIL import Image, ImageTk
import io
import time

import logging
import tkinter as tk
from tkinter import ttk, messagebox

try:
    from typing import Optional, Dict, Any, Callable
except Exception:
    Optional = Dict = Any = Callable = object  # type: ignore

from utils.observers import ObservableFloat



class MainWindow(tk.Tk):
    """
    메인 앱 창.
    - Gimbal Controls 팝업 열기
    - Gazebo Relay Settings 팝업 열기
    - 영상 스트리밍 모듈(ImageStreamBridge) 토글
    - 주기적 상태 표시 업데이트
    """

    def __init__(
        self,
        cfg,
        bridge,
        gimbal,
        relay,
        log,
        zoom_state: Optional[ObservableFloat] = None,
    ):
        super().__init__()
        self.title("Unified Bridge")
        self.geometry("1000x700")

        self.cfg = cfg
        self.bridge = bridge
        self.gimbal = gimbal
        self.relay = relay
        self.log = log
        self.zoom_state = zoom_state
        self._zoom_unsubscribe: Optional[Callable[[], None]] = None

        # 프리뷰 상태 변수
        self._last_photo = None
        self._last_img_ts = 0.0
        self._preview_paused = False

        self.bridge_status_var = tk.StringVar(value="Image Stream Module: Stopped (Realtime)")
        self.gimbal_status_var = tk.StringVar(value="Gimbal: Deactivated")
        self.relay_status_var = tk.StringVar(value="Relay: Deactivated")
        self.preview_info_var = tk.StringVar(value="Last: - | Size: -")
        self.zoom_var = tk.StringVar(value="Zoom: 1.00x")

        self._build_layout()
        self._bind_events()
        self._init_zoom_subscription()

        # ✅ 브릿지에 프리뷰 콜백 등록 (GUI 모드에서만)
        try:
            self.bridge.preview_cb = self.on_preview
        except Exception:
            pass

        # 주기 상태 갱신
        self._refresh_status_periodic()

    # ---------------- UI 구성 ----------------

    def _build_layout(self):
        root = self

        # 상단 컨트롤 바 (기존 버튼들)
        top = ttk.Frame(root, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        self.btn_server = ttk.Button(top, text="Start Image Stream Module", command=self.on_toggle_server)
        self.btn_server.grid(row=0, column=0, padx=(0, 8), pady=4, sticky="w")

        self.btn_bridge = ttk.Button(top, text="Image Stream Module Settings", command=self.open_bridge_window)
        self.btn_bridge.grid(row=0, column=1, padx=(0, 16), pady=4, sticky="w")

        self.lbl_bridge = ttk.Label(top, textvariable=self.bridge_status_var)
        self.lbl_bridge.grid(row=0, column=2, padx=(0, 16), sticky="w")

        self.btn_gimbal = ttk.Button(top, text="Gimbal Controls", command=self.open_gimbal_window)
        self.btn_gimbal.grid(row=1, column=0, padx=(0, 8), pady=4, sticky="w")

        self.lbl_gimbal = ttk.Label(top, textvariable=self.gimbal_status_var)
        self.lbl_gimbal.grid(row=1, column=2, padx=(0, 16), sticky="w")

        self.btn_relay = ttk.Button(top, text="Gazebo Relay Settings", command=self.open_relay_window)
        self.btn_relay.grid(row=2, column=0, padx=(0, 8), pady=4, sticky="w")

        self.lbl_relay = ttk.Label(top, textvariable=self.relay_status_var)
        self.lbl_relay.grid(row=2, column=2, padx=(0, 16), sticky="w")

        # ───────── Preview 영역 ─────────
        mid = ttk.Frame(root, padding=8)
        mid.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.preview_label = ttk.Label(mid, text="Waiting for UDP image...", anchor="center")
        self.preview_label.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 우측 로그/상태 영역(선택)
        right = ttk.Frame(mid, padding=8, width=280)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)

        self.lbl_preview_info = ttk.Label(right, textvariable=self.preview_info_var, anchor="w")
        self.lbl_preview_info.pack(fill=tk.X, pady=(0, 2))

        ttk.Label(right, textvariable=self.zoom_var, anchor="w").pack(fill=tk.X, pady=(0, 6))

        btns = ttk.Frame(right)
        btns.pack(fill=tk.X, pady=6)
        ttk.Button(btns, text="Pause/Resume Preview", command=self.toggle_preview_pause).pack(fill=tk.X)
        ttk.Button(btns, text="Save Latest", command=self.save_latest_image).pack(fill=tk.X, pady=(6,0))

        self.log_text = tk.Text(right, height=20, wrap="word")
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=(8,0))
        self.log_text.insert("end", "Logs will appear in console.\n")

    def _init_zoom_subscription(self) -> None:
        if not self.zoom_state:
            return

        def _callback(value: float) -> None:
            self._queue_zoom_update(value)

        self._zoom_unsubscribe = self.zoom_state.subscribe(_callback)

    def _queue_zoom_update(self, value: float) -> None:
        def _apply() -> None:
            self._update_zoom_label(value)

        # Tkinter calls must run on the main thread
        self.after(0, _apply)

    def _update_zoom_label(self, value: float) -> None:
        try:
            zoom = float(value)
        except Exception:
            zoom = 1.0
        self.zoom_var.set(f"Zoom: {zoom:.2f}x")

    def on_preview(self, jpeg_bytes: bytes):
        """
        ImageStreamBridge가 UDP로 최신 JPEG을 수신할 때 호출됨 (백그라운드 스레드).
        Tk 위젯 업데이트는 메인스레드에서 after로 처리.
        """
        if self._preview_paused:
            return

        def _update():
            try:
                img = Image.open(io.BytesIO(jpeg_bytes))
                # 라벨 크기에 맞춰 썸네일
                w = max(self.preview_label.winfo_width(), 1)
                h = max(self.preview_label.winfo_height(), 1)
                img.thumbnail((w, h), Image.Resampling.LANCZOS)

                photo = ImageTk.PhotoImage(img)
                self.preview_label.configure(image=photo, text="")
                # 참조 유지
                self.preview_label.image = photo
                self._last_photo = (photo, jpeg_bytes)
                self._last_img_ts = time.time()

                kb = len(jpeg_bytes) / 1024.0
                tstr = time.strftime("%H:%M:%S", time.localtime(self._last_img_ts))
                self.preview_info_var.set(f"Last: {tstr} | Size: {kb:.1f} KB")
            except Exception as e:
                self.preview_label.configure(text=f"Preview error: {e}", image="")
                self.preview_label.image = None
                self._last_photo = None

        # GUI 스레드에서 실행
        self.after(0, _update)

    def toggle_preview_pause(self):
        self._preview_paused = not self._preview_paused
        if self._preview_paused:
            self.preview_label.configure(text="Preview paused", image="")
            self.preview_label.image = None
        else:
            self.preview_label.configure(text="Waiting for UDP image..." if not self._last_photo else "")
            # 재갱신은 다음 프레임 수신 때 반영

    def save_latest_image(self):
        if not self._last_photo:
            return
        try:
            _, jpeg = self._last_photo
            # Image stream 모듈의 SaveFile 디렉터리에 타임스탬프명으로 저장
            bridge_cfg = self.cfg.get("bridge", {})
            images_dir = (
                bridge_cfg.get("realtime_dir")
                or bridge_cfg.get("images")
                or "./SaveFile"
            )
            import os
            os.makedirs(images_dir, exist_ok=True)
            fn = os.path.join(images_dir, f"preview_{int(self._last_img_ts)}.jpg")
            with open(fn, "wb") as f:
                f.write(jpeg)
            self.log.info("[UI] Saved latest preview to %s", fn)
        except Exception as e:
            self.log.error("[UI] Save latest preview failed: %s", e)

    def _bind_events(self) -> None:
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- 상태 갱신 ----------------

    def _refresh_status_periodic(self) -> None:
        try:
            running = getattr(self.bridge, "is_server_running", None) and self.bridge.is_server_running.is_set()
            try:
                mode = self.bridge.get_runtime_status().get("image_source_mode")
            except Exception:
                mode = None
            self._set_bridge_status("Running" if running else "Stopped", mode)
            self.btn_server.configure(
                text="Stop Image Stream Module" if running else "Start Image Stream Module"
            )

            # Gimbal 상태
            if hasattr(self.gimbal, "get_status"):
                st = self.gimbal.get_status()
                act = st.get("activated", False)
                mode = st.get("control_mode", "IDLE")
                self._set_gimbal_status(f"{'Activated' if act else 'Deactivated'} ({mode})")
                zoom_val = st.get("zoom_scale")
                if isinstance(zoom_val, (int, float)):
                    self._update_zoom_label(float(zoom_val))
            else:
                self._set_gimbal_status("Unknown")

            # Relay 상태
            act_relay = bool(self.cfg.get("relay", {}).get("activated", False))
            self._set_relay_status("Activated" if act_relay else "Deactivated")

        except Exception as e:
            self.log.error("[UI] status refresh error: %s", e)

        # 500ms 주기
        self.after(500, self._refresh_status_periodic)

    def _set_bridge_status(self, text: str, mode: Optional[str] = None) -> None:
        if mode:
            display = f"{text} ({mode.capitalize()})"
        else:
            display = text
        self.bridge_status_var.set(f"Image Stream Module: {display}")

    def _set_gimbal_status(self, text: str) -> None:
        self.gimbal_status_var.set(f"Gimbal: {text}")

    def _set_relay_status(self, text: str) -> None:
        self.relay_status_var.set(f"Relay: {text}")

    # ---------------- 버튼 동작 ----------------

    def on_toggle_server(self) -> None:
        try:
            running = getattr(self.bridge, "is_server_running", None) and self.bridge.is_server_running.is_set()
            if running:
                self.bridge.stop()
                self.btn_server.configure(text="Start Image Stream Module")
            else:
                # 설정에서 네트워크 파라미터 읽어와 재시작 가능
                bs = self.cfg.get("bridge", {})
                self.bridge.update_settings(bs)
                self.bridge.start()
                self.btn_server.configure(text="Stop Image Stream Module")
        except Exception as e:
            messagebox.showerror("Error", f"Toggle server failed:\n{e}")

    def open_bridge_window(self) -> None:
        try:
            from ui.bridge_window import BridgeSettingsWindow
            BridgeSettingsWindow(
                self,
                self.cfg,
                self.bridge,
                self.log,
                gimbal=self.gimbal,
            )
        except Exception as e:
            messagebox.showerror("Error", f"Open Image Stream Module window failed:\n{e}")

    def open_gimbal_window(self) -> None:
        try:
            from ui.gimbal_window import GimbalControlsWindow
            GimbalControlsWindow(self, self.cfg, self.gimbal, self.log)
        except Exception as e:
            messagebox.showerror("Error", f"Open Gimbal window failed:\n{e}")

    def open_relay_window(self) -> None:
        try:
            from ui.relay_window import RelaySettingsWindow
            RelaySettingsWindow(self, self.cfg, self.relay, self.log)
        except Exception as e:
            messagebox.showerror("Error", f"Open Relay window failed:\n{e}")

    # ---------------- 종료 ----------------

    def on_close(self) -> None:
        if self._zoom_unsubscribe:
            try:
                self._zoom_unsubscribe()
            except Exception:
                pass
        try:
            self.destroy()
        except Exception:
            pass


def run_gui(
    cfg: dict,
    bridge,
    gimbal,
    relay,
    log: logging.Logger,
    zoom_state: Optional[ObservableFloat] = None,
) -> None:
    app = MainWindow(cfg, bridge, gimbal, relay, log, zoom_state=zoom_state)
    app.mainloop()
