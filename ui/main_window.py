# ui/main_window.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from PIL import Image, ImageTk
import io
import os
import time

import logging
import threading
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox

try:
    from typing import Optional, Dict, Any, Callable, Deque, Tuple
except Exception:
    Optional = Dict = Any = Callable = Deque = Tuple = object  # type: ignore

from utils.observers import ObservableFloat


class TkHeartbeatMonitor:
    """Background watchdog that monitors Tk `after` callbacks.

    If the main loop stops scheduling heartbeats for longer than the configured
    timeout, the monitor attempts to recover the image bridge module by
    restarting it.  This helps when the UI thread is starved and the operator
    perceives the entire bridge as frozen.
    """

    def __init__(
        self,
        app: tk.Tk,
        *,
        log: logging.Logger,
        bridge,
        bridge_settings_getter: Optional[Callable[[], Dict[str, Any]]] = None,
        enabled: bool = True,
        interval: float = 1.0,
        timeout: float = 5.0,
        cooldown: float = 30.0,
        restart_delay: float = 1.0,
    ) -> None:
        self._app = app
        self._log = log
        self._bridge = bridge
        self._bridge_settings_getter = bridge_settings_getter
        self._enabled = bool(enabled)
        self._interval = max(0.1, float(interval))
        self._timeout = max(0.5, float(timeout))
        self._cooldown = max(1.0, float(cooldown))
        self._restart_delay = max(0.0, float(restart_delay))

        self._stop_event = threading.Event()
        self._last_pulse = time.monotonic()
        self._last_recovery = 0.0
        self._recovery_lock = threading.Lock()
        self._monitor_thread: Optional[threading.Thread] = None
        self._initialized = False

        if not self._enabled:
            return

        self._schedule_pulse()
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            name="TkHeartbeatMonitor",
            daemon=True,
        )
        self._monitor_thread.start()

    def stop(self) -> None:
        if not self._enabled:
            return
        self._stop_event.set()
        if self._monitor_thread is not None:
            self._monitor_thread.join(timeout=1.0)
            self._monitor_thread = None

    # ----- internal helpers -----

    def _schedule_pulse(self) -> None:
        try:
            self._app.after(int(self._interval * 1000), self._record_pulse)
        except tk.TclError:
            # Tk has already been destroyed.
            self._stop_event.set()

    def _record_pulse(self) -> None:
        self._last_pulse = time.monotonic()
        self._initialized = True
        if self._stop_event.is_set() or not self._enabled:
            return
        self._schedule_pulse()

    def _monitor_loop(self) -> None:
        while not self._stop_event.wait(self._interval):
            if not self._initialized:
                continue
            now = time.monotonic()
            elapsed = now - self._last_pulse
            if elapsed < self._timeout:
                continue
            if now - self._last_recovery < self._cooldown:
                continue
            self._trigger_recovery(elapsed)

    def _trigger_recovery(self, elapsed: float) -> None:
        if not self._recovery_lock.acquire(blocking=False):
            return
        try:
            self._last_recovery = time.monotonic()
            try:
                self._log.warning(
                    "[WATCHDOG] Tk heartbeat stalled for %.1fs (timeout %.1fs). Restarting image bridge.",
                    elapsed,
                    self._timeout,
                )
            except Exception:
                pass
            self._restart_bridge()
        finally:
            self._recovery_lock.release()

    def _restart_bridge(self) -> None:
        if not self._bridge:
            return
        try:
            if hasattr(self._bridge, "stop"):
                self._bridge.stop()
        except Exception as exc:
            try:
                self._log.error("[WATCHDOG] Bridge stop failed: %s", exc)
            except Exception:
                pass
        if self._restart_delay > 0.0:
            time.sleep(self._restart_delay)
        if callable(getattr(self._bridge, "update_settings", None)) and self._bridge_settings_getter:
            try:
                settings = self._bridge_settings_getter() or {}
                self._bridge.update_settings(settings)
            except Exception as exc:
                try:
                    self._log.error("[WATCHDOG] Bridge settings refresh failed: %s", exc)
                except Exception:
                    pass
        try:
            if hasattr(self._bridge, "start"):
                self._bridge.start()
        except Exception as exc:
            try:
                self._log.error("[WATCHDOG] Bridge restart failed: %s", exc)
            except Exception:
                pass


class TkTextHandler(logging.Handler):
    """Logging handler that forwards records to a Tk Text widget."""

    def __init__(self, widget: tk.Text) -> None:
        super().__init__()
        self.widget = widget
        self._buffer: Deque[str] = deque()
        self._lock = threading.Lock()

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
        except Exception:
            self.handleError(record)
            return
        with self._lock:
            self._buffer.append(msg)
        try:
            self.widget.after(0, self._flush)
        except tk.TclError:
            # Widget destroyed; drop the buffered logs.
            with self._lock:
                self._buffer.clear()

    def _flush(self) -> None:
        with self._lock:
            if not self._buffer:
                return
            lines = list(self._buffer)
            self._buffer.clear()
        try:
            self.widget.configure(state="normal")
            for line in lines:
                self.widget.insert("end", line + "\n")
            self.widget.see("end")
            self.widget.configure(state="disabled")
        except tk.TclError:
            # Ignore widget update errors after destruction.
            pass

    def close(self) -> None:
        with self._lock:
            self._buffer.clear()
        super().close()



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
        rover,
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
        self.rover = rover
        self.log = log
        self.zoom_state = zoom_state
        self._zoom_unsubscribe: Optional[Callable[[], None]] = None
        self._heartbeat_monitor: Optional[TkHeartbeatMonitor] = None

        gimbal_cfg = self.cfg.setdefault("gimbal", {})
        initial_method = str(gimbal_cfg.get("control_method", "tcp")).lower()
        if initial_method not in ("tcp", "mavlink"):
            initial_method = "tcp"
        gimbal_cfg["control_method"] = initial_method
        self.gimbal_control_method_var = tk.StringVar(value=initial_method)

        # 프리뷰 상태 변수
        self._last_photo = None
        self._last_img_ts = 0.0
        self._preview_paused = False
        self._preview_lock = threading.Lock()
        self._preview_last_monotonic = 0.0
        self._preview_gui_min_interval = 1.0
        try:
            preview_interval_cfg = float(self.cfg.get("bridge", {}).get("preview_min_interval", 1.0))
        except Exception:
            preview_interval_cfg = 1.0
        self.preview_interval_var = tk.StringVar()
        try:
            self._set_preview_interval(
                preview_interval_cfg,
                persist=False,
                sync_bridge=True,
                announce_bridge=False,
                reset_gate=False,
            )
        except RuntimeError:
            self._preview_gui_min_interval = max(0.0, preview_interval_cfg)
            self.preview_interval_var.set(f"{self._preview_gui_min_interval:.2f}")
        self._preview_pending_frame: Optional[Tuple[bytes, float]] = None
        self._preview_worker_event = threading.Event()
        self._preview_worker_stop = threading.Event()
        self._preview_worker: Optional[threading.Thread] = None
        self._preview_target_size: Tuple[int, int] = (640, 480)
        self._server_toggle_in_progress = False

        self.bridge_status_var = tk.StringVar(value="Image Stream Module: Stopped (Realtime)")
        self.gimbal_status_var = tk.StringVar(value="Gimbal: Deactivated")
        self.relay_status_var = tk.StringVar(value="Relay: Deactivated")
        self.relay_log_status_var = tk.StringVar(value="Gazebo Logging: Idle")
        self.rover_log_status_var = tk.StringVar(value="Rover Logging: Idle")
        self.preview_info_var = tk.StringVar(value="Last: - | Size: -")
        self.zoom_var = tk.StringVar(value="Zoom: 1.00x")

        self._preview_last_time = "-"
        self._preview_last_size = "-"
        self._current_zoom_value = 1.0
        self._refresh_preview_info_label()

        self._build_layout()
        self.preview_label.bind("<Configure>", self._on_preview_label_resize)
        self._bind_events()
        self._init_zoom_subscription()

        # ✅ 브릿지에 프리뷰 콜백 등록 (GUI 모드에서만)
        try:
            self.bridge.preview_cb = self.on_preview
        except Exception:
            pass

        self._start_preview_worker()
        self._init_heartbeat_monitor()

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

        mode_frame = ttk.LabelFrame(top, text="Gimbal Control")
        mode_frame.grid(row=1, column=1, padx=(0, 16), pady=4, sticky="w")
        ttk.Radiobutton(
            mode_frame,
            text="TCP/IP",
            value="tcp",
            variable=self.gimbal_control_method_var,
            command=self._on_gimbal_control_method_changed,
        ).pack(side=tk.LEFT, padx=(4, 4))
        ttk.Radiobutton(
            mode_frame,
            text="MAVLink",
            value="mavlink",
            variable=self.gimbal_control_method_var,
            command=self._on_gimbal_control_method_changed,
        ).pack(side=tk.LEFT, padx=(0, 4))

        self.lbl_gimbal = ttk.Label(top, textvariable=self.gimbal_status_var)
        self.lbl_gimbal.grid(row=1, column=2, padx=(0, 16), sticky="w")

        self.btn_relay = ttk.Button(top, text="Gazebo Relay Settings", command=self.open_relay_window)
        self.btn_relay.grid(row=2, column=0, padx=(0, 8), pady=4, sticky="w")
        self.btn_rover = ttk.Button(top, text="Rover Relay Settings", command=self.open_rover_relay_window)
        self.btn_rover.grid(row=2, column=1, padx=(0, 16), pady=4, sticky="w")

        self.lbl_relay = ttk.Label(top, textvariable=self.relay_status_var)
        self.lbl_relay.grid(row=2, column=2, padx=(0, 16), sticky="w")
        self.lbl_relay_log = ttk.Label(top, textvariable=self.relay_log_status_var)
        self.lbl_relay_log.grid(row=2, column=3, padx=(0, 16), sticky="w")
        self.lbl_rover_log = ttk.Label(top, textvariable=self.rover_log_status_var)
        self.lbl_rover_log.grid(row=2, column=4, padx=(0, 0), sticky="w")

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

        interval_frame = ttk.Frame(right)
        interval_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(interval_frame, text="Preview interval (s)").pack(side=tk.LEFT, padx=(0, 4))
        interval_entry = ttk.Entry(interval_frame, textvariable=self.preview_interval_var, width=6)
        interval_entry.pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(interval_frame, text="Apply", command=self._on_apply_preview_interval).pack(side=tk.LEFT)

        btns = ttk.Frame(right)
        btns.pack(fill=tk.X, pady=6)
        ttk.Button(btns, text="Pause/Resume Preview", command=self.toggle_preview_pause).pack(fill=tk.X)
        ttk.Button(btns, text="Save Latest", command=self.save_latest_image).pack(fill=tk.X, pady=(6,0))

        self._log_handler: Optional[logging.Handler] = None

        self.log_text = tk.Text(right, height=20, wrap="word")
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=(8,0))
        self.log_text.configure(state="disabled")

        handler = TkTextHandler(self.log_text)
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        try:
            self.log.addHandler(handler)
            self._log_handler = handler
        except Exception:
            handler.close()

    def _on_gimbal_control_method_changed(self) -> None:
        method = str(self.gimbal_control_method_var.get()).lower()
        gimbal_cfg = self.cfg.setdefault("gimbal", {})
        prev_method = str(gimbal_cfg.get("control_method", "tcp")).lower()
        prev_valid = prev_method if prev_method in ("tcp", "mavlink") else "tcp"
        if method not in ("tcp", "mavlink"):
            self.gimbal_control_method_var.set(prev_valid)
            return
        if method == "mavlink":
            serial_port = str(gimbal_cfg.get("serial_port", "") or "").strip()
            if not serial_port:
                messagebox.showerror(
                    "Serial 필요",
                    "MAVLink 제어를 사용하려면 먼저 시리얼 포트를 연결하세요.",
                )
                self.gimbal_control_method_var.set(prev_valid)
                return
        if prev_method == method:
            return
        try:
            if hasattr(self.gimbal, "update_settings"):
                self.gimbal.update_settings({"control_method": method})
        except ValueError as exc:
            try:
                self.log.warning("[UI] MAVLink control activation blocked: %s", exc)
            except Exception:
                pass
            messagebox.showerror(
                "MAVLink 제어 실패",
                "시리얼 포트가 설정되지 않아 MAVLink 제어를 활성화할 수 없습니다.",
            )
            self.gimbal_control_method_var.set(prev_valid)
            return

        except Exception as exc:
            try:
                self.log.error("[UI] Failed to apply gimbal control method: %s", exc)
            except Exception:
                pass
            messagebox.showerror("오류", f"제어 모드 변경 실패:\n{exc}")
            self.gimbal_control_method_var.set(prev_valid)
            return
        gimbal_cfg["control_method"] = method

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
        self._current_zoom_value = zoom
        self.zoom_var.set(f"Zoom: {zoom:.2f}x")
        self._refresh_preview_info_label()

    def _refresh_preview_info_label(self) -> None:
        info = f"Last: {self._preview_last_time} | Size: {self._preview_last_size}"
        info += f" | Zoom: {self._current_zoom_value:.2f}x"
        self.preview_info_var.set(info)

    def _on_preview_label_resize(self, event) -> None:
        with self._preview_lock:
            self._preview_target_size = (max(event.width, 1), max(event.height, 1))

    def _start_preview_worker(self) -> None:
        if self._preview_worker is not None and self._preview_worker.is_alive():
            return

        def _worker_loop() -> None:
            self._preview_worker_loop()

        self._preview_worker = threading.Thread(
            target=_worker_loop,
            name="PreviewDecoder",
            daemon=True,
        )
        self._preview_worker.start()

    def _preview_worker_loop(self) -> None:
        while not self._preview_worker_stop.is_set():
            self._preview_worker_event.wait()
            if self._preview_worker_stop.is_set():
                break
            while True:
                with self._preview_lock:
                    frame = self._preview_pending_frame
                    if frame is None:
                        self._preview_worker_event.clear()
                        break
                    self._preview_pending_frame = None
                    target_size = self._preview_target_size
                jpeg_bytes, wall_time = frame
                try:
                    img = Image.open(io.BytesIO(jpeg_bytes))
                    img.load()
                    if img.mode not in ("RGB", "RGBA", "L"):
                        img = img.convert("RGB")
                    if target_size[0] > 0 and target_size[1] > 0:
                        img.thumbnail(target_size, Image.Resampling.LANCZOS)
                    processed_img = img.copy()
                    img.close()
                except Exception as exc:
                    try:
                        self.after(0, lambda err=exc: self._handle_preview_error(err))
                    except tk.TclError:
                        pass
                    continue
                kb = len(jpeg_bytes) / 1024.0
                try:
                    self.after(
                        0,
                        lambda p_img=processed_img, raw=jpeg_bytes, ts=wall_time, kb_val=kb: self._apply_preview_image(
                            p_img, raw, ts, kb_val
                        ),
                    )
                except tk.TclError:
                    return

    def _handle_preview_error(self, err: Exception) -> None:
        self.preview_label.configure(text=f"Preview error: {err}", image="")
        self.preview_label.image = None
        self._last_photo = None

    def _apply_preview_image(
        self,
        processed_img: Image.Image,
        jpeg_bytes: bytes,
        wall_time: float,
        kb_val: float,
    ) -> None:
        if self._preview_paused:
            return
        try:
            photo = ImageTk.PhotoImage(processed_img)
        except Exception as exc:
            self._handle_preview_error(exc)
            return

        self.preview_label.configure(image=photo, text="")
        self.preview_label.image = photo
        self._last_photo = (photo, jpeg_bytes)
        self._last_img_ts = wall_time

        tstr = time.strftime("%H:%M:%S", time.localtime(wall_time))
        self._preview_last_time = tstr
        self._preview_last_size = f"{kb_val:.1f} KB"
        self._refresh_preview_info_label()

    def on_preview(self, jpeg_bytes: bytes):
        """
        ImageStreamBridge가 UDP로 최신 JPEG을 수신할 때 호출됨 (백그라운드 스레드).
        Tk 위젯 업데이트는 메인스레드에서 after로 처리.
        """
        if self._preview_paused:
            return

        wall_time = time.time()
        with self._preview_lock:
            if self._preview_paused:
                return
            now = time.monotonic()
            if (
                self._preview_gui_min_interval > 0.0
                and now - self._preview_last_monotonic < self._preview_gui_min_interval
            ):
                return
            self._preview_last_monotonic = now
            self._preview_pending_frame = (jpeg_bytes, wall_time)
        self._preview_worker_event.set()

    def toggle_preview_pause(self):
        self._preview_paused = not self._preview_paused
        if self._preview_paused:
            with self._preview_lock:
                self._preview_pending_frame = None
            self.preview_label.configure(text="Preview paused", image="")
            self.preview_label.image = None
        else:
            with self._preview_lock:
                self._preview_last_monotonic = 0.0
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

    def _set_preview_interval(
        self,
        interval: float,
        *,
        persist: bool,
        sync_bridge: bool,
        announce_bridge: bool,
        reset_gate: bool = True,
    ) -> float:
        try:
            normalized = max(0.0, float(interval))
        except (TypeError, ValueError) as exc:
            raise RuntimeError("Invalid preview interval") from exc

        applied = normalized
        if sync_bridge:
            configure = getattr(self.bridge, "configure_preview_interval", None)
            if callable(configure):
                try:
                    result = configure(
                        normalized,
                        reset_gate=reset_gate,
                        announce=announce_bridge,
                    )
                    if isinstance(result, (int, float)):
                        applied = float(result)
                except Exception as exc:
                    raise RuntimeError("Bridge rejected preview interval") from exc

        self._preview_gui_min_interval = applied
        self.preview_interval_var.set(f"{applied:.2f}")
        if reset_gate:
            with self._preview_lock:
                self._preview_last_monotonic = 0.0
        if persist:
            bridge_cfg = self.cfg.setdefault("bridge", {})
            bridge_cfg["preview_min_interval"] = applied
        return applied

    def _on_apply_preview_interval(self) -> None:
        value = self.preview_interval_var.get().strip()
        try:
            interval = float(value)
        except ValueError:
            messagebox.showerror("잘못된 값", "숫자 값을 입력하세요.")
            self.preview_interval_var.set(f"{self._preview_gui_min_interval:.2f}")
            return

        if interval < 0:
            messagebox.showerror("잘못된 값", "0초 이상으로 입력하세요.")
            self.preview_interval_var.set(f"{self._preview_gui_min_interval:.2f}")
            return

        previous = self._preview_gui_min_interval
        try:
            applied = self._set_preview_interval(
                interval,
                persist=True,
                sync_bridge=True,
                announce_bridge=True,
            )
        except RuntimeError as exc:
            self.preview_interval_var.set(f"{previous:.2f}")
            try:
                self.log.error("[UI] Preview interval update failed: %s", exc)
            except Exception:
                pass
            messagebox.showerror("오류", "브릿지에 적용하지 못했습니다.")
            return

        try:
            self.log.info("[UI] Preview interval updated to %.2fs", applied)
        except Exception:
            pass

    # ---------------- 상태 갱신 ----------------

    def _refresh_status_periodic(self) -> None:
        try:
            running = getattr(self.bridge, "is_server_running", None) and self.bridge.is_server_running.is_set()
            bridge_status: Dict[str, Any] = {}
            try:
                status_obj = self.bridge.get_runtime_status()
                if isinstance(status_obj, dict):
                    bridge_status = status_obj
            except Exception:
                bridge_status = {}
            mode = bridge_status.get("image_source_mode")
            self._set_bridge_status("Running" if running else "Stopped", mode)
            self.btn_server.configure(
                text="Stop Image Stream Module" if running else "Start Image Stream Module"
            )

            if not self.zoom_state:
                bridge_zoom = bridge_status.get("zoom_scale")
                if isinstance(bridge_zoom, (int, float)):
                    zoom_value = float(bridge_zoom)
                    if abs(zoom_value - self._current_zoom_value) > 1e-3:
                        self._update_zoom_label(zoom_value)

            # Gimbal 상태
            if hasattr(self.gimbal, "get_status"):
                st = self.gimbal.get_status()
                act = st.get("activated", False)
                method = str(st.get("control_method", "")).lower()
                display_mode = st.get("control_mode")
                if method:
                    display = method.upper()
                    if method in ("tcp", "mavlink") and self.gimbal_control_method_var.get() != method:
                        self.gimbal_control_method_var.set(method)
                    gimbal_cfg = self.cfg.setdefault("gimbal", {})
                    if gimbal_cfg.get("control_method") != method:
                        gimbal_cfg["control_method"] = method
                else:
                    display = str(display_mode or "IDLE").upper()
                    cfg_method = str(self.cfg.get("gimbal", {}).get("control_method", "")).lower()
                    if cfg_method in ("tcp", "mavlink") and self.gimbal_control_method_var.get() != cfg_method:
                        self.gimbal_control_method_var.set(cfg_method)
                self._set_gimbal_status(f"{'Activated' if act else 'Deactivated'} ({display})")
                if not self.zoom_state:
                    zoom_val = st.get("zoom_scale")
                    if isinstance(zoom_val, (int, float)):
                        self._update_zoom_label(float(zoom_val))
            else:
                self._set_gimbal_status("Unknown")

            # Relay 상태 + Gazebo 로깅
            relay_cfg = self.cfg.setdefault("relay", {})
            relay_status = {}
            try:
                if hasattr(self.relay, "get_status"):
                    relay_status = self.relay.get_status() or {}
            except Exception:
                relay_status = {}

            act_relay = bool(relay_status.get("activated", relay_cfg.get("activated", False)))
            relay_cfg["activated"] = act_relay
            self._set_relay_status("Activated" if act_relay else "Deactivated")

            if "gazebo_log_path" in relay_status and relay_status["gazebo_log_path"] is not None:
                relay_cfg["gazebo_log_path"] = relay_status["gazebo_log_path"]
            log_path = str(relay_cfg.get("gazebo_log_path", "") or "")
            log_active = bool(relay_status.get("gazebo_logging_active", False))
            relay_cfg["gazebo_logging_active"] = log_active
            log_count_raw = relay_status.get("gazebo_logged_count")
            relay_cfg["gazebo_logged_count"] = log_count_raw
            log_error = relay_status.get("gazebo_log_error")
            enable_log = bool(relay_status.get("enable_gazebo_logging", relay_cfg.get("enable_gazebo_logging", True)))
            relay_cfg["enable_gazebo_logging"] = enable_log
            block_reason = str(relay_status.get("gazebo_log_block_reason") or "")
            relay_cfg["gazebo_log_block_reason"] = block_reason

            if not enable_log:
                text = "Gazebo Logging: Disabled (Settings)"
            elif block_reason:
                text = f"Gazebo Logging: Blocked ({block_reason})"
            elif log_error:
                text = f"Gazebo Logging: Error ({log_error})"
            elif log_active and log_path:
                display_path = os.path.basename(log_path) or log_path
                if isinstance(log_count_raw, (int, float)):
                    count_disp = int(log_count_raw)
                    text = f"Gazebo Logging: Recording ({count_disp}) → {display_path}"
                else:
                    text = f"Gazebo Logging: Recording → {display_path}"
            elif log_path:
                display_path = os.path.basename(log_path) or log_path
                text = f"Gazebo Logging: Ready → {display_path}"
            else:
                text = "Gazebo Logging: Disabled"
            self.relay_log_status_var.set(text)

            # Rover logging status
            rover_cfg = self.cfg.setdefault("rover", {})
            rover_status = {}
            try:
                if hasattr(self.rover, "get_status"):
                    rover_status = self.rover.get_status() or {}
            except Exception:
                rover_status = {}

            rover_active = bool(rover_status.get("activated", rover_cfg.get("activated", False)))
            rover_cfg["activated"] = rover_active
            rover_enabled = bool(rover_status.get("enabled", rover_cfg.get("enabled", True)))
            rover_cfg["enabled"] = rover_enabled

            if "feedback_log_path" in rover_status and rover_status["feedback_log_path"] is not None:
                rover_cfg["feedback_log_path"] = rover_status["feedback_log_path"]

            fb_path = str(rover_cfg.get("feedback_log_path", "") or "")
            fb_active = bool(rover_status.get("feedback_logging_active", False))
            fb_count = rover_status.get("feedback_logged_count")
            fb_error = rover_status.get("feedback_log_error") or ""

            rover_cfg.pop("cmd_log_path", None)

            if not rover_enabled:
                rover_text = "Rover Logging: Disabled (Settings)"
            elif fb_active:
                if isinstance(fb_count, (int, float)):
                    rover_text = f"Rover Logging: Recording (FB {int(fb_count)})"
                else:
                    rover_text = "Rover Logging: Recording (FB)"
            elif fb_error:
                rover_text = f"Rover Logging: Error (FB {fb_error})"
            elif rover_active and not fb_path:
                rover_text = "Rover Logging: Forwarding (no log path)"
            elif fb_path:
                target = f"FB→{os.path.basename(fb_path) or fb_path}"
                rover_text = f"Rover Logging: Ready ({target})"
            else:
                rover_text = "Rover Logging: Idle"
            self.rover_log_status_var.set(rover_text)

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
        if self._server_toggle_in_progress:
            return

        self._server_toggle_in_progress = True
        try:
            self.btn_server.configure(state=tk.DISABLED)
        except tk.TclError:
            self._server_toggle_in_progress = False
            return

        def _toggle_worker() -> None:
            error: Optional[Exception] = None
            try:
                running_event = getattr(self.bridge, "is_server_running", None)
                running = bool(running_event and running_event.is_set())
                if running:
                    self.bridge.stop()
                else:
                    bs = self.cfg.get("bridge", {})
                    self.bridge.update_settings(bs)
                    self.bridge.start()
            except Exception as exc:
                error = exc
            finally:
                def _finalize() -> None:
                    try:
                        running_now = bool(
                            getattr(self.bridge, "is_server_running", None)
                            and self.bridge.is_server_running.is_set()
                        )
                    except Exception:
                        running_now = False

                    btn_text = (
                        "Stop Image Stream Module"
                        if running_now
                        else "Start Image Stream Module"
                    )
                    try:
                        self.btn_server.configure(text=btn_text, state=tk.NORMAL)
                    except tk.TclError:
                        pass
                    self._server_toggle_in_progress = False

                    if error is not None:
                        try:
                            if hasattr(self, "log") and self.log:
                                self.log.error("[UI] Toggle server failed: %s", error)
                        except Exception:
                            pass
                        try:
                            messagebox.showerror("Error", f"Toggle server failed:\n{error}")
                        except tk.TclError:
                            pass

                try:
                    self.after(0, _finalize)
                except tk.TclError:
                    self._server_toggle_in_progress = False

        threading.Thread(target=_toggle_worker, name="BridgeToggle", daemon=True).start()

    def open_bridge_window(self) -> None:
        try:
            from ui.bridge_window import BridgeSettingsWindow
            BridgeSettingsWindow(self, self.cfg, self.bridge, self.log)
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

    def open_rover_relay_window(self) -> None:
        try:
            from ui.rover_relay_window import RoverRelaySettingsWindow
            RoverRelaySettingsWindow(self, self.cfg, self.rover, self.relay, self.log)
        except Exception as e:
            messagebox.showerror("Error", f"Open Rover Relay window failed:\n{e}")

    # ---------------- 종료 ----------------

    def on_close(self) -> None:
        if self._zoom_unsubscribe:
            try:
                self._zoom_unsubscribe()
            except Exception:
                pass
        self._preview_worker_stop.set()
        self._preview_worker_event.set()
        worker = getattr(self, "_preview_worker", None)
        if isinstance(worker, threading.Thread):
            worker.join(timeout=1.0)
        handler = getattr(self, "_log_handler", None)
        if handler is not None:
            try:
                self.log.removeHandler(handler)
            except Exception:
                pass
            try:
                handler.close()
            except Exception:
                pass
            self._log_handler = None
        if self._heartbeat_monitor is not None:
            try:
                self._heartbeat_monitor.stop()
            except Exception:
                pass
            self._heartbeat_monitor = None
        try:
            self.destroy()
        except Exception:
            pass


    def _init_heartbeat_monitor(self) -> None:
        ui_cfg = self.cfg.get("ui")
        if not isinstance(ui_cfg, dict):
            ui_cfg = {}
            self.cfg["ui"] = ui_cfg
        else:
            self.cfg["ui"] = ui_cfg
        heartbeat_cfg = ui_cfg.setdefault("heartbeat", {})
        if not isinstance(heartbeat_cfg, dict):
            heartbeat_cfg = {}
            ui_cfg["heartbeat"] = heartbeat_cfg

        enabled = bool(heartbeat_cfg.get("enabled", True))
        if not enabled:
            return

        def _float(cfg_key: str, default: float) -> float:
            try:
                return float(heartbeat_cfg.get(cfg_key, default))
            except (TypeError, ValueError):
                return default

        interval = max(0.1, _float("interval_sec", 1.0))
        timeout = max(interval * 2.0, _float("timeout_sec", 5.0))
        cooldown = max(timeout, _float("recovery_cooldown_sec", 30.0))
        restart_delay = max(0.0, _float("restart_delay_sec", 1.0))
        restart_bridge = bool(heartbeat_cfg.get("restart_bridge", True))

        bridge_obj = self.bridge if restart_bridge else None

        self._heartbeat_monitor = TkHeartbeatMonitor(
            self,
            log=self.log,
            bridge=bridge_obj,
            bridge_settings_getter=lambda: self.cfg.get("bridge", {}),
            enabled=enabled,
            interval=interval,
            timeout=timeout,
            cooldown=cooldown,
            restart_delay=restart_delay,
        )


def run_gui(
    cfg: dict,
    bridge,
    gimbal,
    relay,
    rover,
    log: logging.Logger,
    zoom_state: Optional[ObservableFloat] = None,
) -> None:
    app = MainWindow(cfg, bridge, gimbal, relay, rover, log, zoom_state=zoom_state)
    app.mainloop()
