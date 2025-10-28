# core/image_stream_bridge.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import io
import os
import queue
import shutil
import socket
import struct
import threading
import time
import datetime
from typing import Callable, Optional, Dict, Any, Tuple, List

from core.network import get_global_dispatcher

try:
    from PIL import Image  # 미리보기 썸네일 생성에 사용될 수 있음(옵션)
    _HAS_PIL = True
except Exception:
    _HAS_PIL = False


# ---- Command IDs (MroCameraControl) ----
from network.image_stream_icd import (
    CMD_REQ_CAPTURE,
    CMD_SET_GIMBAL,
    CMD_SET_COUNT,
    CMD_GET_IMG_NUM,
    CMD_REQ_SEND_IMG,
    CMD_SET_ZOOM_RATIO,
    CMD_GET_ZOOM_RATIO,
    CMD_IMG_NUM_RESPONSE,
    CMD_FILE_IMG_TRANSFER,
    CMD_ACK_ZOOM_RATIO,
    COMMAND_NAMES,
    UDP_HEADER_SIZE as ICD_UDP_HEADER_SIZE,
    parse_udp_header,
    pack_tcp_response,
)


__all__ = ["ImageStreamBridge", "ImageBridgeCore"]


class ImageStreamBridge:
    UDP_HEADER_SIZE = ICD_UDP_HEADER_SIZE
    MAX_UDP_FRAME_SIZE = 64_970
    MAX_TOTAL_IMAGE_SIZE = 128 * 1024 * 1024  # 128 MB safety cap
    MAX_FRAMES_PER_IMAGE = 4096
    UDP_REASSEMBLY_TIMEOUT = 3.0

    TCP_MAX_PAYLOAD = 1 * 1024 * 1024  # 1 MB safety cap for control commands
    TCP_SOCKET_TIMEOUT = 1.0
    TCP_MIN_PAYLOAD_TIMEOUT = 1.5
    TCP_BYTES_PER_SEC = 256_000  # ~250 KB/s lower bound expectation
    MIN_COMMAND_PAYLOAD = 9

    """Image stream module for MRO UnifiedBridge (통합브릿지).

    MRO UnifiedBridge를 구성하는 세 모듈 중 영상 스트리밍 담당 파트로,
    실시간 UDP 프레임을 조립하고 TCP 명령(캡처/파일 전송 등)에 대응한다.
    SaveFile(실시간 캡처) 또는 PreDefinedImageSet(사전 이미지) 중 선택된
    라이브러리를 사용해 TCP 클라이언트에 이미지를 제공한다.

    - TCP 서버: MroCameraControl 명령 송수신
    - UDP 수신: New ICD(30B 헤더) 기반 JPEG 조립
    - 이미지 라이브러리 선택: realtime(SaveFile) / predefined(PreDefinedImageSet)
    - 콜백: log_cb(text), preview_cb(jpeg_bytes), status_cb(text)
    """

    def __init__(
        self,
        log_cb: Callable[[str], None],
        preview_cb: Optional[Callable[[bytes], None]],
        status_cb: Optional[Callable[[str], None]],
        settings: Dict[str, Any],
        zoom_update_cb: Optional[Callable[[float], None]] = None,
    ) -> None:
        self.log = log_cb
        self.preview_cb = preview_cb
        self.status_cb = status_cb

        # settings
        self.ip = settings.get("ip", "0.0.0.0")
        self.tcp_port = int(settings.get("tcp_port", 9999))
        self.udp_port = int(settings.get("udp_port", 9998))

        legacy_images = settings.get("images")
        self.realtime_dir = str(
            settings.get("realtime_dir")
            or legacy_images
            or "./SaveFile"
        )
        self.predefined_dir = str(settings.get("predefined_dir", "./PreDefinedImageSet"))
        self.image_source_mode = self._sanitize_mode(settings.get("image_source_mode", "realtime"))
        # Backward compatibility: maintain images_dir alias used by legacy callers/UI.
        self.images_dir = self.realtime_dir

        # Cached metadata for predefined set enumeration.
        self._predefined_numbers: list[int] = []

        self._zoom_scale = max(1.0, float(settings.get("zoom_scale", 1.0)))
        self._zoom_requires_processing = self._zoom_scale > 1.0001
        self._logged_zoom_warn = False

        self._zoom_cb_lock = threading.Lock()
        self._zoom_callbacks: List[Callable[[float], None]] = []

        self._lock = threading.Lock()
        self._latest_raw_jpeg: Optional[bytes] = None
        self._latest_jpeg: Optional[bytes] = None
        self._next_image_number: int = 0  # 다음 저장 번호 (000..999 롤링)

        self._latest_frame_token: int = 0
        self._last_captured_frame_token: int = -1
        self._capture_inflight: bool = False
        self._last_capture_completed_at: float = 0.0

        self._last_preview_monotonic = 0.0
        try:
            preview_interval_cfg = settings.get("preview_min_interval", 1.0)
        except Exception:
            preview_interval_cfg = 1.0
        try:
            self.configure_preview_interval(preview_interval_cfg, reset_gate=False)
        except Exception:
            self._preview_min_interval = 1.0

        self._gimbal: Optional["GimbalControl"] = None
        try:
            self._gimbal_sensor_type = int(settings.get("gimbal_sensor_type", 0))
        except Exception:
            self._gimbal_sensor_type = 0
        try:
            self._gimbal_sensor_id = int(settings.get("gimbal_sensor_id", 0))
        except Exception:
            self._gimbal_sensor_id = 0

        # runtime
        self.is_server_running = threading.Event()
        self._tcp_sock: Optional[socket.socket] = None
        self._udp_sock: Optional[socket.socket] = None
        self._client_lock = threading.Lock()
        self._client_conn: Optional[socket.socket] = None

        self._tcp_thread: Optional[threading.Thread] = None
        self._udp_registered = False
        self._dispatcher = get_global_dispatcher()
        self._udp_queue: Optional[queue.Queue[bytes]] = None
        self._udp_worker: Optional[threading.Thread] = None
        self._udp_queue_drop_count = 0
        self._udp_queue_last_log = 0.0

        # UDP reassembly buffer
        self._reasm: Optional[Dict[str, Any]] = None
        self._last_image_meta = {
            "kb": 0.0,
            "received_at": None,   # datetime or None
            "saved_path": None,    # 마지막으로 디스크에 쓴 파일 경로
        }

        if zoom_update_cb is not None:
            self.register_zoom_update_callback(zoom_update_cb)

        self._prepare_dirs()
        self._sync_next_number()

        if self.image_source_mode == "predefined":
            self._predefined_numbers = self._scan_predefined_numbers()


    # --------------- lifecycle ---------------

    def start(self) -> None:
        if self.is_server_running.is_set():
            return
        self.is_server_running.set()
        self._open_tcp()
        self._open_udp()
        self._tcp_thread = threading.Thread(target=self._tcp_server_thread, daemon=True)
        self._tcp_thread.start()
        if self._udp_sock:
            try:
                self._start_udp_worker()
                self._dispatcher.register(
                    self._udp_sock,
                    self._handle_udp_packet,
                    name=f"image-stream-{self.udp_port}",
                    idle_callback=None,
                    buffer_size=65535,
                )
                self._udp_registered = True
            except Exception as exc:
                self.log(f"[BRIDGE] UDP dispatcher registration failed: {exc}")
                self._udp_registered = False
                self.is_server_running.clear()
                self._stop_udp_worker()
                self._close_udp_socket()
                raise
        self._emit_status("RUNNING")
        self.log(f"[BRIDGE] started at {self.ip} TCP:{self.tcp_port} UDP:{self.udp_port}")

    def stop(self) -> None:
        if not self.is_server_running.is_set():
            return
        self.is_server_running.clear()

        # close client first
        client: Optional[socket.socket]
        with self._client_lock:
            client = self._client_conn
            self._client_conn = None
        if client:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                client.close()
            except Exception:
                pass

        # wake up accept()
        if self._tcp_sock:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.2)
                s.connect((self.ip, self.tcp_port))
                s.close()
            except Exception:
                pass

        self._close_sockets()
        self._stop_udp_worker()
        thread = self._tcp_thread
        self._tcp_thread = None
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.5)
        self._emit_status("STOPPED")
        self.log("[BRIDGE] stopped")

    def disconnect_tcp_client(self) -> bool:
        """Force-close the current TCP client connection if present."""

        with self._client_lock:
            conn = self._client_conn
            self._client_conn = None
        if not conn:
            return False
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            conn.close()
        except Exception:
            pass
        self.log("[BRIDGE] TCP client disconnect requested")
        return True

    def update_settings(self, settings: Dict[str, Any]) -> None:
        self.ip = settings.get("ip", self.ip)
        self.tcp_port = int(settings.get("tcp_port", self.tcp_port))
        self.udp_port = int(settings.get("udp_port", self.udp_port))

        if "gimbal_sensor_type" in settings:
            try:
                self._gimbal_sensor_type = int(settings["gimbal_sensor_type"])
            except Exception:
                pass
        if "gimbal_sensor_id" in settings:
            try:
                self._gimbal_sensor_id = int(settings["gimbal_sensor_id"])
            except Exception:
                pass
        if "preview_min_interval" in settings:
            try:
                self.configure_preview_interval(
                    settings["preview_min_interval"], announce=True
                )
            except Exception:
                pass

        legacy_images = settings.get("images")
        if "realtime_dir" in settings or legacy_images is not None:
            new_realtime = str(settings.get("realtime_dir", legacy_images or self.realtime_dir))
            if new_realtime:
                self.realtime_dir = new_realtime

        if "predefined_dir" in settings:
            new_predefined = settings.get("predefined_dir")
            if new_predefined:
                self.predefined_dir = str(new_predefined)

        # Keep legacy alias in sync for older callers.
        self.images_dir = self.realtime_dir

        if "image_source_mode" in settings:
            self.set_image_source_mode(settings.get("image_source_mode", self.image_source_mode))
        elif self.image_source_mode == "predefined":
            # Directory change without explicit mode update -> rescan predefined list.
            self._predefined_numbers = self._scan_predefined_numbers()

        self._prepare_dirs()
        self._sync_next_number()
        self.log("[BRIDGE] settings updated")

    def attach_gimbal_controller(self, gimbal: Optional["GimbalControl"]) -> None:
        self._gimbal = gimbal

    def configure_gimbal_forwarding(self, sensor_type: int, sensor_id: int) -> None:
        self._gimbal_sensor_type = int(sensor_type)
        self._gimbal_sensor_id = int(sensor_id)

    def configure_preview_interval(
        self,
        interval: float,
        *,
        reset_gate: bool = True,
        announce: bool = False,
    ) -> float:
        """Update the minimum interval between preview callbacks.

        Returns the normalized interval that will be used by the bridge."""

        try:
            value = max(0.0, float(interval))
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid preview interval") from exc

        with self._lock:
            previous = self._preview_min_interval if hasattr(self, "_preview_min_interval") else None
            self._preview_min_interval = value
            if reset_gate:
                self._last_preview_monotonic = 0.0

        if announce and (previous is None or abs(value - previous) > 1e-6):
            self.log(f"[BRIDGE] preview interval -> {value:.2f}s")

        return self._preview_min_interval

    def register_zoom_update_callback(
        self, callback: Optional[Callable[[float], None]]
    ) -> Callable[[], None]:
        """Register *callback* to be invoked when the zoom scale changes.

        Returns an unsubscribe function. If *callback* is ``None`` a no-op
        unsubscribe is returned for convenience.
        """

        if callback is None:
            return lambda: None

        with self._zoom_cb_lock:
            self._zoom_callbacks.append(callback)

        try:
            callback(self._zoom_scale)
        except Exception:
            pass

        def _unsubscribe() -> None:
            with self._zoom_cb_lock:
                self._zoom_callbacks = [cb for cb in self._zoom_callbacks if cb is not callback]

        return _unsubscribe

    # --------------- TCP Server ---------------

    def _open_tcp(self) -> None:
        self._tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._tcp_sock.bind((self.ip, self.tcp_port))
        self._tcp_sock.listen(1)
        self.log(f"[BRIDGE] TCP listening: {self.ip}:{self.tcp_port}")

    def _tcp_server_thread(self) -> None:
        while self.is_server_running.is_set():
            try:
                conn, addr = self._tcp_sock.accept()
                with self._client_lock:
                    self._client_conn = conn
                self.log(f"[BRIDGE] TCP client connected: {addr}")
                t = threading.Thread(target=self._handle_client_thread, args=(conn,), daemon=True)
                t.start()
            except OSError:
                break
            except Exception as e:
                self.log(f"[BRIDGE] TCP accept error: {e}")
                time.sleep(0.1)

    def _handle_client_thread(self, conn: socket.socket) -> None:
        try:
            conn.settimeout(self.TCP_SOCKET_TIMEOUT)
            while self.is_server_running.is_set():
                header = self._recv_all(
                    conn,
                    4,
                    total_timeout=None,
                    allow_idle=True,
                )
                if header is None:
                    break
                if header == b"":
                    continue
                payload_len = struct.unpack("<I", header)[0]
                if payload_len < self.MIN_COMMAND_PAYLOAD:
                    self.log(
                        f"[BRIDGE] TCP payload too small ({payload_len} bytes); closing connection"
                    )
                    break
                if payload_len > self.TCP_MAX_PAYLOAD:
                    self.log(
                        f"[BRIDGE] TCP payload too large ({payload_len} bytes); closing connection"
                    )
                    break
                payload_timeout = self._compute_payload_timeout(payload_len)
                payload = self._recv_all(
                    conn,
                    payload_len,
                    total_timeout=payload_timeout,
                )
                if not payload or len(payload) != payload_len:
                    self.log("[BRIDGE] TCP payload timeout or length mismatch; closing connection")
                    break
                self._process_command(payload, conn)
        except Exception as e:
            self.log(f"[BRIDGE] client error: {e}")
        finally:
            try:
                conn.close()
            except Exception:
                pass
            with self._client_lock:
                if self._client_conn is conn:
                    self._client_conn = None
            self.log("[BRIDGE] TCP client disconnected")
            
    def _process_command(self, payload: bytes, conn: socket.socket) -> None:
        # [ts_sec(4)][ts_nsec(4)][cmd_id(1)] + payload...
        if len(payload) < 9:
            self.log("[BRIDGE] malformed cmd payload")
            return
        ts_sec, ts_nsec, cmd_id = struct.unpack("<IIB", payload[:9])
        cmd_payload = payload[9:]

        cmd_name = COMMAND_NAMES.get(cmd_id, f"0x{cmd_id:02X}")

        def _expect_uint8_one(p: bytes, name: str) -> bool:
            """uint8==1 규격을 검증. 없거나 값이 1이 아니면 경고 로그."""
            if len(p) < 1:
                self.log(f"[BRIDGE] {name}: missing uint8 payload; treating as 1 for backward-compat")
                return True  # 하위호환을 위해 허용
            (flag,) = struct.unpack("<B", p[:1])
            if flag != 1:
                self.log(f"[BRIDGE] {name}: expected uint8==1, got {flag}; ignoring but continuing")
            return True

        if cmd_id == CMD_REQ_CAPTURE:
            # 기대 페이로드: uint8==1 (1바이트)
            _expect_uint8_one(cmd_payload, cmd_name)
            self._handle_req_capture()

        elif cmd_id == CMD_SET_COUNT:
            if len(cmd_payload) >= 4:
                (count_num,) = struct.unpack("<I", cmd_payload[:4])
                self._handle_set_count(count_num)
            else:
                self.log(f"[BRIDGE] {cmd_name}: missing uint32 count_num")

        elif cmd_id == CMD_GET_IMG_NUM:
            # 기대 페이로드: uint8==1 (1바이트)
            _expect_uint8_one(cmd_payload, cmd_name)
            self._handle_get_img_num(conn)

        elif cmd_id == CMD_REQ_SEND_IMG:
            if len(cmd_payload) >= 4:
                (img_num,) = struct.unpack("<I", cmd_payload[:4])
                self._handle_req_send_img(conn, img_num)
            else:
                self.log(f"[BRIDGE] {cmd_name}: missing uint32 img_num")

        elif cmd_id == CMD_SET_GIMBAL:
            self._handle_set_gimbal(cmd_payload)

        elif cmd_id == CMD_SET_ZOOM_RATIO:
            if len(cmd_payload) < 4:
                self.log(f"[BRIDGE] {cmd_name}: missing float zoom_ratio")
                return
            (zoom_ratio,) = struct.unpack("<f", cmd_payload[:4])
            self.set_zoom_scale(zoom_ratio)
            with self._lock:
                applied = self._zoom_scale
            self.log(f"[BRIDGE] {cmd_name} -> request={zoom_ratio:.3f}, applied={applied:.3f}")
            self._send_zoom_ratio_ack(conn, applied)

        elif cmd_id == CMD_GET_ZOOM_RATIO:
            _expect_uint8_one(cmd_payload, cmd_name)
            with self._lock:
                applied = self._zoom_scale
            self.log(f"[BRIDGE] {cmd_name} -> {applied:.3f}")
            self._send_zoom_ratio_ack(conn, applied)

        else:
            self.log(f"[BRIDGE] unknown cmd_id: {cmd_name}")

    def _handle_req_capture(self) -> None:
        current_frame_token = -1
        next_num = 0
        fn = ""
        latest_jpeg: Optional[bytes]
        fallback_src: Optional[str]

        with self._lock:
            if self._capture_inflight:
                self.log("[BRIDGE] capture request ignored: capture already in progress")
                return
            current_frame_token = self._latest_frame_token
            if (
                self._last_captured_frame_token >= 0
                and current_frame_token <= self._last_captured_frame_token
                and self._last_capture_completed_at > 0.0
            ):
                self.log(
                    "[BRIDGE] capture request ignored: waiting for a new frame before saving"
                )
                return
            self._capture_inflight = True
            next_num = self._next_image_number
            fn = os.path.join(self.realtime_dir, f"{next_num:03d}.jpg")
            latest_jpeg = self._latest_jpeg
            fallback_src = self._last_image_meta.get("saved_path")

        saved_kb: Optional[float] = None
        saved_from = "live"
        write_error: Optional[Exception] = None
        capture_success = False

        try:
            if latest_jpeg:
                try:
                    with open(fn, "wb") as f:
                        f.write(latest_jpeg)
                    saved_kb = len(latest_jpeg) / 1024.0
                except Exception as exc:
                    write_error = exc
            else:
                write_error = RuntimeError("no in-memory frame to capture")

            if write_error:
                if fallback_src and os.path.exists(fallback_src):
                    try:
                        shutil.copy2(fallback_src, fn)
                        saved_kb = os.path.getsize(fn) / 1024.0
                        saved_from = "fallback"
                        self.log(
                            f"[BRIDGE] capture fallback: reused {fallback_src} due to {write_error}"
                        )
                    except Exception as copy_exc:
                        self.log(
                            f"[BRIDGE] capture fallback failed: {copy_exc} (original: {write_error})"
                        )
                        return
                else:
                    self.log(
                        f"[BRIDGE] capture failed ({write_error}); no previous image to copy"
                    )
                    return

            capture_success = True
        finally:
            with self._lock:
                self._capture_inflight = False
                if capture_success:
                    self._last_captured_frame_token = current_frame_token
                    self._last_capture_completed_at = time.monotonic()
                    self._last_image_meta["saved_path"] = fn
                    if saved_kb is not None:
                        self._last_image_meta["kb"] = saved_kb
                    self.log(
                        f"[BRIDGE] saved image ({saved_from}): {fn} | next_num(before)={next_num:03d}"
                    )
                    self._next_image_number = (self._next_image_number + 1) % 1000
                    self.log(
                        f"[BRIDGE] next image number -> {self._next_image_number:03d}"
                    )

    def _handle_set_count(self, count_num: int) -> None:
        with self._lock:
            self._next_image_number = count_num % 1000
        self.log(f"[BRIDGE] next image number set to {self._next_image_number:03d}")

    def _handle_get_img_num(self, conn: socket.socket) -> None:
        with self._lock:
            last_saved = (self._next_image_number - 1 + 1000) % 1000
        ack_uuid = 0
        data = struct.pack("<II", ack_uuid, last_saved)
        ts_sec, ts_nsec = int(time.time()), time.time_ns() % 1_000_000_000
        frame = pack_tcp_response(CMD_IMG_NUM_RESPONSE, data, ts_sec=ts_sec, ts_nsec=ts_nsec)
        try:
            conn.sendall(frame)
            self.log(f"[BRIDGE] Img_Num_Response: {last_saved:03d}")
        except Exception as e:
            self.log(f"[BRIDGE] send Get_ImgNum failed: {e}")

    def _handle_req_send_img(self, conn: socket.socket, img_num: int) -> None:
        fp = self._resolve_image_path(img_num)
        ack_uuid = 0
        try:
            with open(fp, "rb") as f:
                img = f.read()
        except FileNotFoundError:
            img = b""
        if img:
            img = self._apply_zoom_to_jpeg(img)
        data = struct.pack("<III", ack_uuid, img_num, len(img)) + img
        ts_sec, ts_nsec = int(time.time()), time.time_ns() % 1_000_000_000
        frame = pack_tcp_response(CMD_FILE_IMG_TRANSFER, data, ts_sec=ts_sec, ts_nsec=ts_nsec)
        try:
            conn.sendall(frame)
            self.log(f"[BRIDGE] File_ImgTransfer sent: num={img_num:03d}, size={len(img)}")
        except Exception as e:
            self.log(f"[BRIDGE] send File_ImgTransfer failed: {e}")

    def _handle_set_gimbal(self, payload: bytes) -> None:
        if not payload:
            self.log("[BRIDGE] Set_Gimbal payload missing")
            return

        values: Optional[Tuple[float, float, float, float, float, float]] = None
        if len(payload) >= struct.calcsize("<3d3f"):
            try:
                values = struct.unpack("<3d3f", payload[: struct.calcsize("<3d3f")])
            except struct.error:
                values = None
        if values is None and len(payload) >= struct.calcsize("<6f"):
            try:
                values = struct.unpack("<6f", payload[: struct.calcsize("<6f")])
            except struct.error:
                values = None

        if values is None:
            self.log("[BRIDGE] Set_Gimbal malformed payload; expected <3d3f> or <6f>")
            return

        x, y, z, roll_legacy, pitch_legacy, yaw_legacy = values
        sim_pitch = float(pitch_legacy)
        sim_yaw = float(yaw_legacy)
        sim_roll = float(roll_legacy)
        sensor_type = self._gimbal_sensor_type
        sensor_id = self._gimbal_sensor_id

        gimbal = self._gimbal
        if not gimbal:
            self.log(
                "[BRIDGE] Set_Gimbal ignored because no gimbal controller is attached"
            )
            return

        try:
            gimbal.apply_external_pose(
                sensor_type,
                sensor_id,
                x,
                y,
                z,
                sim_pitch,
                sim_yaw,
                sim_roll,
            )
            self.log(
                "[BRIDGE] Set_Gimbal forwarded to gimbal -> sensor=%d/%d "
                "xyz=(%.2f,%.2f,%.2f) sim_rpy(P,Y,R)=(%.2f,%.2f,%.2f)"
                % (sensor_type, sensor_id, x, y, z, sim_pitch, sim_yaw, sim_roll)
            )
        except Exception as exc:
            self.log(f"[BRIDGE] forwarding Set_Gimbal failed: {exc}")

    # --------------- UDP Receiver (New ICD) ---------------

    def _open_udp(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
            sock.bind((self.ip, self.udp_port))
            sock.settimeout(0.3)
        except OSError:
            sock.close()
            raise
        self._udp_sock = sock
        self.log(f"[BRIDGE] UDP listening: {self.ip}:{self.udp_port}")

    def _start_udp_worker(self) -> None:
        if self._udp_worker and self._udp_worker.is_alive():
            return
        self._udp_queue = queue.Queue(maxsize=512)
        self._udp_queue_drop_count = 0
        self._udp_queue_last_log = 0.0
        self._udp_worker = threading.Thread(
            target=self._udp_worker_loop,
            name=f"image-udp-worker-{self.udp_port}",
            daemon=True,
        )
        self._udp_worker.start()

    def _stop_udp_worker(self) -> None:
        worker = self._udp_worker
        queue_obj = self._udp_queue
        self._udp_worker = None
        if queue_obj is not None:
            # Drop any queued packets so the worker can exit promptly.
            try:
                while not queue_obj.empty():
                    queue_obj.get_nowait()
                    queue_obj.task_done()
            except Exception:
                pass
        if worker and worker.is_alive():
            worker.join(timeout=1.5)
        self._udp_queue = None
        if self._udp_queue_drop_count:
            self.log(
                f"[BRIDGE] UDP queue dropped {self._udp_queue_drop_count} packets before shutdown"
            )
            self._udp_queue_drop_count = 0

    def _udp_worker_loop(self) -> None:
        queue_obj = self._udp_queue
        if queue_obj is None:
            return
        while True:
            running = self.is_server_running.is_set()
            try:
                packet = queue_obj.get(timeout=0.2)
            except queue.Empty:
                if not running and queue_obj.empty():
                    break
                self._expire_stale_reassembly(now=time.monotonic())
                continue
            if packet is None:
                continue
            try:
                self._process_udp_datagram(packet)
            except Exception as exc:  # noqa: BLE001
                self.log(f"[BRIDGE] UDP worker error: {exc}")
            finally:
                queue_obj.task_done()
        self._expire_stale_reassembly(force=True)

    def _record_udp_queue_drop(self, now: float) -> None:
        self._udp_queue_drop_count += 1
        if now - self._udp_queue_last_log >= 1.0:
            dropped = self._udp_queue_drop_count
            self.log(f"[BRIDGE] UDP queue overflow; dropped {dropped} packets")
            self._udp_queue_drop_count = 0
            self._udp_queue_last_log = now

    def _handle_udp_packet(self, packet: bytes, _addr: Tuple[str, int], _now: float) -> None:
        queue_obj = self._udp_queue
        if queue_obj is None or not packet:
            return
        try:
            queue_obj.put_nowait(packet)
        except queue.Full:
            self._record_udp_queue_drop(time.monotonic())

    def _process_udp_datagram(self, packet: bytes) -> None:
        header = parse_udp_header(packet)
        if not header or header.get("msg_type") != 9:
            return

        now = time.monotonic()
        self._expire_stale_reassembly(now=now)

        fi = header.get("frame_index", 0)
        frames = header.get("msg_frames", 0)
        fsize = header.get("frame_size", 0)
        fpos = header.get("frame_pos", 0)
        msg_size = header.get("msg_size", 0)

        if fi == 1:
            if self._reasm is not None:
                self._drop_reassembly("new frame start")
            if not self._start_new_reassembly(header):
                self._reasm = None
                return

        reasm = self._reasm
        if reasm is None:
            return

        if frames <= 0 or msg_size <= 0:
            self._drop_reassembly("invalid header values")
            return

        if reasm["msg_size"] != msg_size or reasm["total"] != frames:
            if fi == 1 and self._start_new_reassembly(header):
                reasm = self._reasm
            else:
                self._drop_reassembly("metadata mismatch")
                return

        if fi < 1 or fi > reasm["total"]:
            self._drop_reassembly(f"frame index out of range ({fi}/{reasm['total']})")
            return

        if fsize <= 0 or fsize > self.MAX_UDP_FRAME_SIZE:
            self._drop_reassembly(f"invalid frame_size {fsize}")
            return

        if fpos < 0 or fpos > reasm["msg_size"]:
            self._drop_reassembly(f"invalid frame_pos {fpos}")
            return

        payload = packet[self.UDP_HEADER_SIZE : self.UDP_HEADER_SIZE + fsize]
        if len(payload) != fsize:
            self._drop_reassembly("payload truncated")
            return

        endpos = fpos + fsize
        if endpos > reasm["msg_size"]:
            self._drop_reassembly("payload exceeds msg_size")
            return

        reasm["buf"][fpos:endpos] = payload
        reasm["recv"].add(fi)
        reasm["frame_index"] = fi
        if endpos > reasm["maxpos"]:
            reasm["maxpos"] = endpos
        reasm["last_update"] = now

        if len(reasm["recv"]) == reasm["total"]:
            data = bytes(reasm["buf"][: reasm["maxpos"]])
            if len(data) >= 4 and data.startswith(b"\xff\xd8") and data.endswith(b"\xff\xd9"):
                zoomed = self._publish_zoomed_frame(data, update_timestamp=True)
                elapsed = now - reasm["start_monotonic"]
                size_kb = len(zoomed) / 1024.0 if zoomed else len(data) / 1024.0
                self.log(
                    f"[BRIDGE] image received: {size_kb:.1f} KB in {elapsed:.2f}s | next={self._next_image_number:03d}"
                )
            else:
                self.log("[BRIDGE] invalid JPEG received")
            self._reasm = None

    def _start_new_reassembly(self, header: Dict[str, int]) -> bool:
        msg_size = header.get("msg_size", 0)
        msg_frames = header.get("msg_frames", 0)

        if msg_frames <= 0 or msg_frames > self.MAX_FRAMES_PER_IMAGE:
            self.log(f"[BRIDGE] invalid msg_frames value: {msg_frames}")
            return False
        if msg_size <= 0 or msg_size > self.MAX_TOTAL_IMAGE_SIZE:
            self.log(f"[BRIDGE] invalid msg_size value: {msg_size}")
            return False
        if msg_frames * self.MAX_UDP_FRAME_SIZE < msg_size:
            self.log(
                f"[BRIDGE] msg_size/msg_frames inconsistent: size={msg_size} frames={msg_frames}"
            )
            return False
        try:
            buf = bytearray(msg_size)
        except MemoryError:
            self.log(
                f"[BRIDGE] cannot allocate image buffer ({msg_size / (1024 * 1024):.1f} MB)"
            )
            return False

        now = time.monotonic()
        self._reasm = {
            "buf": buf,
            "recv": set(),
            "total": msg_frames,
            "frame_index": 0,
            "maxpos": 0,
            "start_monotonic": now,
            "last_update": now,
            "msg_size": msg_size,
        }
        return True

    def _drop_reassembly(self, reason: str) -> None:
        if not self._reasm:
            return
        frames = len(self._reasm.get("recv", ()))
        total = self._reasm.get("total", 0)
        self.log(
            f"[BRIDGE] dropping partial image ({reason}); received {frames}/{total} frames"
        )
        self._reasm = None

    def _expire_stale_reassembly(
        self,
        *,
        force: bool = False,
        now: Optional[float] = None,
    ) -> None:
        if not self._reasm:
            return
        now_ts = time.monotonic() if now is None else now
        last_update = self._reasm.get("last_update", now_ts)
        if not force and now_ts - last_update <= self.UDP_REASSEMBLY_TIMEOUT:
            return
        elapsed = now_ts - self._reasm.get("start_monotonic", now_ts)
        frames = len(self._reasm.get("recv", ()))
        total = self._reasm.get("total", 0)
        reason = "timeout" if not force else "reset"
        self.log(
            f"[BRIDGE] dropping partial image ({reason}: {elapsed:.2f}s, frames {frames}/{total})"
        )
        self._reasm = None

    def _compute_payload_timeout(self, size: int) -> float:
        base = size / float(self.TCP_BYTES_PER_SEC)
        return max(self.TCP_MIN_PAYLOAD_TIMEOUT, base + 1.0)
    
    def get_runtime_status(self) -> Dict[str, Any]:
        with self._lock:
            status = {
                "next_image_number": self._next_image_number,
                "last_image_kb": self._last_image_meta["kb"],
                "last_image_received_at": (
                    self._last_image_meta["received_at"].strftime("%Y-%m-%d %H:%M:%S")
                    if self._last_image_meta["received_at"] else None
                ),
                "last_saved_path": self._last_image_meta["saved_path"],
                "udp_listening": bool(self._udp_sock),
                "tcp_listening": bool(self._tcp_sock),
                "image_source_mode": self.image_source_mode,
                "realtime_dir": self.realtime_dir,
                "predefined_dir": self.predefined_dir,
                "active_library_dir": self.realtime_dir if self.image_source_mode == "realtime" else self.predefined_dir,
                "zoom_scale": self._zoom_scale,
            }
        with self._client_lock:
            status["tcp_client_connected"] = self._client_conn is not None
        return status

    def set_zoom_scale(self, zoom: float) -> None:
        value = max(1.0, float(zoom))
        with self._lock:
            if abs(value - self._zoom_scale) < 1e-3:
                return
            self._zoom_scale = value
            self._zoom_requires_processing = value > 1.0001
            latest_raw = self._latest_raw_jpeg
        self.log(f"[BRIDGE] zoom scale -> {self._zoom_scale:.2f}x")
        if latest_raw:
            self._publish_zoomed_frame(latest_raw, update_timestamp=False)
        with self._zoom_cb_lock:
            callbacks = list(self._zoom_callbacks)
        for cb in callbacks:
            try:
                cb(self._zoom_scale)
            except Exception:
                pass

    def _send_zoom_ratio_ack(self, conn: socket.socket, zoom_ratio: float) -> None:
        ts_sec, ts_nsec = int(time.time()), time.time_ns() % 1_000_000_000
        data = struct.pack("<f", float(zoom_ratio))
        frame = pack_tcp_response(CMD_ACK_ZOOM_RATIO, data, ts_sec=ts_sec, ts_nsec=ts_nsec)
        try:
            conn.sendall(frame)
            self.log(f"[BRIDGE] Ack_Zoomratio sent: {zoom_ratio:.3f}")
        except Exception as e:
            self.log(f"[BRIDGE] send Ack_Zoomratio failed: {e}")

    def _get_zoom_state(self) -> tuple[float, bool]:
        with self._lock:
            return self._zoom_scale, self._zoom_requires_processing

    def _apply_zoom_to_jpeg(self, jpeg: bytes) -> bytes:
        zoom, needs_processing = self._get_zoom_state()
        if not needs_processing:
            return jpeg
        if not _HAS_PIL:
            if not self._logged_zoom_warn:
                self.log("[BRIDGE] zoom requested but Pillow is unavailable; skipping digital zoom")
                self._logged_zoom_warn = True
            return jpeg
        try:
            with Image.open(io.BytesIO(jpeg)) as img:
                width, height = img.size
                if width <= 0 or height <= 0:
                    return jpeg
                crop_w = max(1, min(width, int(round(width / zoom))))
                crop_h = max(1, min(height, int(round(height / zoom))))
                left = max(0, (width - crop_w) // 2)
                top = max(0, (height - crop_h) // 2)
                right = min(width, left + crop_w)
                bottom = min(height, top + crop_h)
                img = img.crop((left, top, right, bottom))
                if (right - left) != width or (bottom - top) != height:
                    img = img.resize((width, height), Image.BICUBIC)
                buf = io.BytesIO()
                img.save(buf, format="JPEG", quality=90)
                return buf.getvalue()
        except Exception as exc:
            self.log(f"[BRIDGE] zoom processing failed: {exc}")
        return jpeg

    def _publish_zoomed_frame(self, raw: bytes, *, update_timestamp: bool) -> Optional[bytes]:
        if not raw:
            return None

        zoomed = self._apply_zoom_to_jpeg(raw)
        should_emit_preview = False
        with self._lock:
            if update_timestamp:
                self._latest_frame_token += 1
                self._last_image_meta["received_at"] = datetime.datetime.now()
            self._latest_raw_jpeg = raw
            self._latest_jpeg = zoomed
            self._last_image_meta["kb"] = len(zoomed) / 1024.0
            now = time.monotonic()
            if self._preview_min_interval <= 0.0:
                should_emit_preview = True
                self._last_preview_monotonic = now
            elif now - self._last_preview_monotonic >= self._preview_min_interval:
                should_emit_preview = True
                self._last_preview_monotonic = now
        if should_emit_preview and self.preview_cb:
            try:
                self.preview_cb(zoomed)
            except Exception:
                pass
        return zoomed

    # --------------- utils ---------------

    def _prepare_dirs(self) -> None:
        for label, path in ("SaveFile", self.realtime_dir), ("PreDefinedImageSet", self.predefined_dir):
            if not path:
                continue
            try:
                os.makedirs(path, exist_ok=True)
            except Exception as e:
                self.log(f"[BRIDGE] mkdir {label} failed: {e}")

    def _sync_next_number(self) -> None:
        try:
            digits = [
                int(os.path.splitext(name)[0])
                for name in os.listdir(self.realtime_dir)
                if os.path.splitext(name)[0].isdigit()
            ]
            if digits:
                self._next_image_number = (max(digits) + 1) % 1000
        except Exception:
            pass

    def _resolve_image_path(self, img_num: int) -> str:
        base_dir = self.realtime_dir if self.image_source_mode == "realtime" else self.predefined_dir
        return os.path.join(base_dir, f"{img_num:03d}.jpg")

    def _scan_predefined_numbers(self) -> list[int]:
        numbers: list[int] = []
        try:
            for name in os.listdir(self.predefined_dir):
                stem, ext = os.path.splitext(name)
                if stem.isdigit() and ext.lower() in {".jpg", ".jpeg"}:
                    try:
                        numbers.append(int(stem))
                    except ValueError:
                        continue
            numbers.sort()
        except Exception as e:
            self.log(f"[BRIDGE] predefined scan failed: {e}")
        return numbers

    def _sanitize_mode(self, mode: str) -> str:
        if isinstance(mode, str) and mode.lower() in {"realtime", "predefined"}:
            return mode.lower()
        return "realtime"

    def set_image_source_mode(self, mode: str) -> None:
        new_mode = self._sanitize_mode(mode)
        if new_mode == self.image_source_mode:
            return
        self.image_source_mode = new_mode
        if new_mode == "predefined":
            self._predefined_numbers = self._scan_predefined_numbers()
        self.log(f"[BRIDGE] image source mode -> {self.image_source_mode}")

    def _close_sockets(self) -> None:
        self._close_tcp_socket()
        self._close_udp_socket()

    def _close_tcp_socket(self) -> None:
        sock = self._tcp_sock
        if not sock:
            return
        self._tcp_sock = None
        try:
            sock.close()
        except Exception:
            pass

    def _close_udp_socket(self) -> None:
        sock = self._udp_sock
        if not sock:
            return
        if self._udp_registered:
            try:
                self._dispatcher.unregister(sock)
            except Exception:
                pass
            self._udp_registered = False
        self._udp_sock = None
        try:
            sock.close()
        except Exception:
            pass

    def _recv_all(
        self,
        conn: socket.socket,
        n: int,
        *,
        total_timeout: Optional[float] = None,
        allow_idle: bool = False,
    ) -> Optional[bytes]:
        """Return exactly *n* bytes.

        Returns ``None`` on EOF/errors and ``b""`` when *allow_idle* is True and the
        call timed out without reading any bytes within *total_timeout*."""

        if n <= 0:
            return b""

        buf = bytearray()
        start = time.monotonic()
        while len(buf) < n and self.is_server_running.is_set():
            try:
                chunk = conn.recv(n - len(buf))
            except socket.timeout:
                now = time.monotonic()
                if not buf and allow_idle:
                    if total_timeout is not None and now - start >= total_timeout:
                        return b""
                    continue
                if total_timeout is not None and now - start >= total_timeout:
                    return None
                continue
            except OSError:
                return None

            if not chunk:
                return None

            buf.extend(chunk)

            if total_timeout is not None:
                now = time.monotonic()
                if now - start >= total_timeout and len(buf) < n:
                    return None

        if len(buf) < n:
            return None

        return bytes(buf)

    def _emit_status(self, text: str) -> None:
        try:
            if self.status_cb:
                self.status_cb(text)
        except Exception:
            pass


class ImageBridgeCore(ImageStreamBridge):

    """Concrete alias retained for backwards compatibility."""

    pass

