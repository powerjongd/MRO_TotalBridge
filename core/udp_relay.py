# core/udp_relay.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import os
import queue
import socket
import struct
import threading
import time
from typing import Callable, Optional, Dict, Any, Tuple

from pymavlink import mavutil
try:
    from serial import SerialException
except ImportError:
    from serial.serialutil import SerialException

from network import gazebo_relay_icd as relay_icd
from .log_parsers import UNIFIED_CSV_HEADER, format_drone_csv_row


class UdpRelay:
    """
    Gazebo UDP → (1) 그대로 ImageGenerator ExternalCtrl UDP로 Relay
                → (2) Optical Flow 센서(가정)로 변환하여 MAVLink OPTICAL_FLOW(100) 시리얼 송출
    또한, ImageGenerator가 UDP로 뿌리는 Distance Sensor를 'RAW UDP ICD'로 수신하여
    동일 시리얼 포트로 MAVLink DISTANCE_SENSOR(132)로 중계하고, Current_Distance를 상태로 유지/표시한다.

    스레드:
      - Gazebo 수신/릴레이/OF 변환 스레드
      - Distance RAW UDP 수신 → 시리얼 중계 스레드 (또는 MAVLink 모드 수신)
      - 센서 Heartbeat 송신 스레드(OF/DS 각각 주기)
    """

    # 자세한 ICD 정의는 :mod:`network.gazebo_relay_icd` 참조

    # MAVLink Heartbeat 기본(센서)
    # (값들은 UI에서 변경 가능)
    DEF_HB_TYPE = 18         # MAV_TYPE_ONBOARD_CONTROLLER
    DEF_HB_AUTOPILOT = 8     # MAV_AUTOPILOT_INVALID
    DEF_HB_BASE_MODE = 0
    DEF_HB_CUSTOM_MODE = 0
    DEF_HB_SYS_STATUS = 4

    SERIAL_RETRY_INTERVAL = 5.0
    SERIAL_RETRY_SUCCESS_HOLD = 1.0

    def __init__(
        self,
        log_cb: Callable[[str], None],
        status_cb: Optional[Callable[[str], None]],
        settings: Dict[str, Any],
    ):
        self.log = log_cb
        self.status_cb = status_cb
        self.s = dict(settings)

        # ---- 런타임 상태값 (UI 폴링용) ----
        self._lock = threading.Lock()
        self.running = False

        # Gazebo 로그 상태
        self.gazebo_log_path = str(self.s.get("gazebo_log_path", "")).strip()
        self._gazebo_log_file: Optional[Any] = None
        self._gazebo_log_lock = threading.Lock()
        self._gazebo_log_active = False
        self._gazebo_log_count = 0
        self._gazebo_log_error = ""
        self._gazebo_log_last_ts = 0.0
        self._gazebo_log_start_ts = 0.0
        self._gazebo_logging_enabled = bool(self.s.get("enable_gazebo_logging", True))
        self.s["enable_gazebo_logging"] = self._gazebo_logging_enabled
        self._gazebo_log_block_reason = ""
        self._gazebo_log_closing = False
        self._gazebo_log_queue: "queue.Queue[Optional[str]]" = queue.Queue()
        self._gazebo_log_writer = threading.Thread(
            target=self._gazebo_log_worker_loop,
            name="gazebo-csv-writer",
            daemon=True,
        )
        self._gazebo_log_writer.start()

        self._rover_logger: Optional["RoverRelayLogger"] = None

        # 거리 센서 현재값
        self.current_distance_m: float = 0.0
        self.last_distance_update: float = 0.0

        # 마지막 OF 품질/고도/각속도
        self.of_quality: int = 0
        self.last_ground_distance: float = 0.0
        self.last_av = (0.0, 0.0, 0.0)
        self.last_of_send_ts: float = 0.0

        # Sockets / MAV
        self.sock_gazebo: Optional[socket.socket] = None
        self.sock_ext: Optional[socket.socket] = None

        # Distance 입력 (둘 중 하나 사용)
        self.sock_dist: Optional[socket.socket] = None      # RAW UDP
        self.mav_udp: Optional[mavutil.mavfile] = None      # MAVLink UDP

        # 공용 시리얼 (OF + Distance 중계)
        self.mav_ser: Optional[mavutil.mavfile] = None
        self._serial_lock = threading.Lock()
        self._serial_retry_at = 0.0
        self.enable_of_serial = bool(self.s.get("enable_optical_flow_serial", True))
        self.s["enable_optical_flow_serial"] = self.enable_of_serial
        self.enable_distance_serial = bool(self.s.get("enable_distance_serial", True))
        self.s["enable_distance_serial"] = self.enable_distance_serial

        # Threads
        self.stop_ev = threading.Event()
        self.th_gazebo: Optional[threading.Thread] = None
        self.th_dist_udp: Optional[threading.Thread] = None
        self.th_hb: Optional[threading.Thread] = None

        # ---- Optical Flow 스케일/품질/하트비트 파라미터 (UI로부터 설정) ----
        # 스케일/품질
        self.of_scale_pix   = float(self.s.get("of_scale_pix", 100.0))
        self.q_base         = int(self.s.get("q_base", 255))
        self.q_min          = int(self.s.get("q_min", 0))
        self.q_max          = int(self.s.get("q_max", 255))
        self.accel_thresh   = float(self.s.get("accel_thresh", 5.0))
        self.gyro_thresh    = float(self.s.get("gyro_thresh", 2.0))
        self.accel_penalty  = float(self.s.get("accel_penalty", 20.0))
        self.gyro_penalty   = float(self.s.get("gyro_penalty", 30.0))

        # Heartbeat (OF)
        self.hb_of_rate_hz  = float(self.s.get("hb_of_rate_hz", 1.0))
        self.hb_of_sysid    = int(self.s.get("hb_of_sysid", 42))
        self.hb_of_compid   = int(self.s.get("hb_of_compid", 199))
        self.hb_of_type     = int(self.s.get("hb_of_type", self.DEF_HB_TYPE))
        self.hb_of_autop    = int(self.s.get("hb_of_autopilot", self.DEF_HB_AUTOPILOT))
        self.hb_of_mode     = int(self.s.get("hb_of_base_mode", self.DEF_HB_BASE_MODE))
        self.hb_of_cus      = int(self.s.get("hb_of_custom_mode", self.DEF_HB_CUSTOM_MODE))
        self.hb_of_stat     = int(self.s.get("hb_of_system_status", self.DEF_HB_SYS_STATUS))

        # Heartbeat (Distance)
        self.hb_ds_rate_hz  = float(self.s.get("hb_ds_rate_hz", 1.0))
        self.hb_ds_sysid    = int(self.s.get("hb_ds_sysid", 43))
        self.hb_ds_compid   = int(self.s.get("hb_ds_compid", 200))
        self.hb_ds_type     = int(self.s.get("hb_ds_type", self.DEF_HB_TYPE))
        self.hb_ds_autop    = int(self.s.get("hb_ds_autopilot", self.DEF_HB_AUTOPILOT))
        self.hb_ds_mode     = int(self.s.get("hb_ds_base_mode", self.DEF_HB_BASE_MODE))
        self.hb_ds_cus      = int(self.s.get("hb_ds_custom_mode", self.DEF_HB_CUSTOM_MODE))
        self.hb_ds_stat     = int(self.s.get("hb_ds_system_status", self.DEF_HB_SYS_STATUS))

        # Distance 수신 모드: 'raw' (기본) or 'mavlink'
        self.distance_mode = str(self.s.get("distance_mode", "raw")).lower()

    # ------------- 설정 업데이트 -------------
    def update_settings(self, s: Dict[str, Any]) -> None:
        log_path_changed = False
        logging_state_changed = False
        serial_changed = False
        baud_changed = False
        serial_flags_changed = False
        with self._lock:
            prev_port = str(self.s.get("serial_port", "")).strip()
            prev_baud = int(self.s.get("serial_baud", 115200))
            prev_flags = (self.enable_of_serial, self.enable_distance_serial)
            self.s.update(s)
            new_port = str(self.s.get("serial_port", prev_port)).strip()
            self.s["serial_port"] = new_port
            if new_port != prev_port:
                serial_changed = True
            new_baud = int(self.s.get("serial_baud", prev_baud))
            self.s["serial_baud"] = new_baud
            if new_baud != prev_baud:
                baud_changed = True
            # 스케일/품질
            self.of_scale_pix  = float(self.s.get("of_scale_pix", self.of_scale_pix))
            self.q_base        = int(self.s.get("q_base", self.q_base))
            self.q_min         = int(self.s.get("q_min", self.q_min))
            self.q_max         = int(self.s.get("q_max", self.q_max))
            self.accel_thresh  = float(self.s.get("accel_thresh", self.accel_thresh))
            self.gyro_thresh   = float(self.s.get("gyro_thresh", self.gyro_thresh))
            self.accel_penalty = float(self.s.get("accel_penalty", self.accel_penalty))
            self.gyro_penalty  = float(self.s.get("gyro_penalty", self.gyro_penalty))
            # HB OF
            self.hb_of_rate_hz = float(self.s.get("hb_of_rate_hz", self.hb_of_rate_hz))
            self.hb_of_sysid   = int(self.s.get("hb_of_sysid", self.hb_of_sysid))
            self.hb_of_compid  = int(self.s.get("hb_of_compid", self.hb_of_compid))
            self.hb_of_type    = int(self.s.get("hb_of_type", self.hb_of_type))
            self.hb_of_autop   = int(self.s.get("hb_of_autopilot", self.hb_of_autop))
            self.hb_of_mode    = int(self.s.get("hb_of_base_mode", self.hb_of_mode))
            self.hb_of_cus     = int(self.s.get("hb_of_custom_mode", self.hb_of_cus))
            self.hb_of_stat    = int(self.s.get("hb_of_system_status", self.hb_of_stat))
            # HB DS
            self.hb_ds_rate_hz = float(self.s.get("hb_ds_rate_hz", self.hb_ds_rate_hz))
            self.hb_ds_sysid   = int(self.s.get("hb_ds_sysid", self.hb_ds_sysid))
            self.hb_ds_compid  = int(self.s.get("hb_ds_compid", self.hb_ds_compid))
            self.hb_ds_type    = int(self.s.get("hb_ds_type", self.hb_ds_type))
            self.hb_ds_autop   = int(self.s.get("hb_ds_autopilot", self.hb_ds_autop))
            self.hb_ds_mode    = int(self.s.get("hb_ds_base_mode", self.hb_ds_mode))
            self.hb_ds_cus     = int(self.s.get("hb_ds_custom_mode", self.hb_ds_cus))
            self.hb_ds_stat    = int(self.s.get("hb_ds_system_status", self.hb_ds_stat))
            # Distance 모드 및 플래그
            self.distance_mode = str(self.s.get("distance_mode", self.distance_mode)).lower()
            self.enable_of_serial = bool(self.s.get("enable_optical_flow_serial", self.enable_of_serial))
            self.s["enable_optical_flow_serial"] = self.enable_of_serial
            self.enable_distance_serial = bool(self.s.get("enable_distance_serial", self.enable_distance_serial))
            self.s["enable_distance_serial"] = self.enable_distance_serial
            if (self.enable_of_serial, self.enable_distance_serial) != prev_flags:
                serial_flags_changed = True
            new_log_path = str(self.s.get("gazebo_log_path", self.gazebo_log_path)).strip()
            if new_log_path != self.gazebo_log_path:
                log_path_changed = True
                self.gazebo_log_path = new_log_path
            self.s["gazebo_log_path"] = self.gazebo_log_path
            new_enabled = bool(self.s.get("enable_gazebo_logging", self._gazebo_logging_enabled))
            if new_enabled != self._gazebo_logging_enabled:
                logging_state_changed = True
                self._gazebo_logging_enabled = new_enabled
            self.s["enable_gazebo_logging"] = self._gazebo_logging_enabled
        self.log("[RELAY] settings updated")
        if serial_changed or baud_changed or serial_flags_changed:
            self._drop_serial_connection(schedule_retry=False)
            if self.running and (self.enable_of_serial or self.enable_distance_serial):
                self._open_serial_shared(force=True)
        if log_path_changed or logging_state_changed:
            if self.running:
                self._open_gazebo_log(reset_counter=True)
            else:
                self._close_gazebo_log()

    # ------------- 라이프사이클 -------------
    def start(self) -> None:
        with self._lock:
            if self.running:
                return
            self.running = True

        self.stop_ev.clear()

        # 외부 UDP 송신 소켓 (ExternalCtrl)
        self.sock_ext = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_ext.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock_ext.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2 * 1024 * 1024)
        except OSError:
            pass

        # Gazebo 수신 소켓
        self.sock_gazebo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_gazebo.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_gazebo.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)

        try:
            g_ip = self.s.get("gazebo_listen_ip", "0.0.0.0")
            g_port = int(self.s.get("gazebo_listen_port", 17000))
            self.sock_gazebo.bind((g_ip, g_port))
            self.log(f"[RELAY] Gazebo listen on {g_ip}:{g_port}")
        except Exception as e:
            self.log(f"[RELAY] Gazebo bind error: {e}")
            self.stop()
            return

        # Distance 입력 열기 (RAW 우선)
        try:
            d_ip = self.s.get("distance_udp_listen_ip", "0.0.0.0")
            d_port = int(self.s.get("distance_udp_listen_port", 14650))

            if self.distance_mode == "raw":
                # RAW UDP 바인드
                self.sock_dist = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock_dist.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.sock_dist.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 * 1024 * 1024)
                self.sock_dist.bind((d_ip, d_port))
                self.log(f"[RELAY] Distance RAW UDP listen on {d_ip}:{d_port}")
            else:
                # MAVLink 수신
                self.mav_udp = mavutil.mavlink_connection(
                    f"udpin:{d_ip}:{d_port}",
                    dialect="common",
                    autoreconnect=True,
                    robust_parsing=True,
                )
                try:
                    self.mav_udp.mav.mavlink20 = True
                except Exception:
                    pass
                self.log(f"[RELAY] Distance MAVLink UDP listen on {d_ip}:{d_port}")
        except Exception as e:
            self.log(f"[RELAY] Distance input open error: {e}")

        # 공용 시리얼 오픈
        self._open_serial_shared(force=True)

        # Gazebo 로그 오픈
        self._open_gazebo_log(reset_counter=True)

        # 스레드 가동
        self.th_gazebo = threading.Thread(target=self._gazebo_loop, daemon=True)
        self.th_gazebo.start()

        if self.sock_dist:
            self.th_dist_udp = threading.Thread(target=self._distance_raw_udp_loop, daemon=True)
            self.th_dist_udp.start()
        elif self.mav_udp:
            self.th_dist_udp = threading.Thread(target=self._distance_mav_udp_loop, daemon=True)
            self.th_dist_udp.start()

        self.th_hb = threading.Thread(target=self._sensor_heartbeat_loop, daemon=True)
        self.th_hb.start()

        self._emit_status("RUNNING")
        self.log("[RELAY] started")

    def stop(self) -> None:
        self.stop_ev.set()
        with self._lock:
            self.running = False

        # 소켓/링크 정리
        try:
            if self.sock_gazebo:
                self.sock_gazebo.close()
        except Exception:
            pass
        self.sock_gazebo = None

        try:
            if self.sock_ext:
                self.sock_ext.close()
        except Exception:
            pass
        self.sock_ext = None

        try:
            if self.sock_dist:
                self.sock_dist.close()
        except Exception:
            pass
        self.sock_dist = None

        try:
            if self.mav_udp:
                self.mav_udp.close()
        except Exception:
            pass
        self.mav_udp = None

        self._drop_serial_connection(schedule_retry=False)

        self._close_gazebo_log()

        self._emit_status("STOPPED")
        self.log("[RELAY] stopped")

    # ------------- 상태 -------------
    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            status = {
                "activated": self.running,
                "distance_m": self.current_distance_m,
                "distance_age_s": (time.time() - self.last_distance_update) if self.last_distance_update else -1.0,
                "of_quality": self.of_quality,
                "of_ground_dist": self.last_ground_distance,
                "of_last_send_age_s": (time.time() - self.last_of_send_ts) if self.last_of_send_ts else -1.0,
                "serial": self.s.get("serial_port", ""),
                "serial_connected": bool(self.mav_ser),
                "of_serial_enabled": bool(self.enable_of_serial),
                "distance_serial_enabled": bool(self.enable_distance_serial),
                "gazebo_listen": f"{self.s.get('gazebo_listen_ip','0.0.0.0')}:{int(self.s.get('gazebo_listen_port',17000))}",
                "ext_dst": f"{self.s.get('ext_udp_ip','127.0.0.1')}:{int(self.s.get('ext_udp_port',9091))}",
                "dist_udp": f"{self.s.get('distance_udp_listen_ip','0.0.0.0')}:{int(self.s.get('distance_udp_listen_port',14650))}",
            }
        with self._gazebo_log_lock:
            status.update({
                "gazebo_log_path": self.gazebo_log_path,
                "gazebo_logging_active": bool(self.running and self._gazebo_log_file is not None),
                "gazebo_logged_count": self._gazebo_log_count,
                "gazebo_log_error": self._gazebo_log_error,
                "gazebo_log_last_write_ts": self._gazebo_log_last_ts if self._gazebo_log_last_ts else None,
                "enable_gazebo_logging": self._gazebo_logging_enabled,
                "gazebo_log_block_reason": self._gazebo_log_block_reason,
            })
        return status

    # ------------- 내부 유틸 -------------
    def _open_serial_shared(self, force: bool = False) -> None:
        port = str(self.s.get("serial_port", "")).strip()
        baud = int(self.s.get("serial_baud", 115200))
        with self._serial_lock:
            if not port:
                if self.mav_ser:
                    self.log("[RELAY] serial_port cleared; closing serial")
                self._drop_serial_connection_locked(schedule_retry=False)
                self.log("[RELAY] serial_port empty (opticalflow/dist relay disabled)")
                return
            if self.mav_ser and not force:
                return
            self._drop_serial_connection_locked(schedule_retry=False)
            try:
                conn = mavutil.mavlink_connection(
                    port,
                    baud=baud,
                    source_system=self.hb_of_sysid,
                    source_component=self.hb_of_compid,
                )
                try:
                    conn.mav.mavlink20 = True
                except Exception:
                    pass
                try:
                    port_handle = getattr(conn, "port", None)
                    if port_handle and hasattr(port_handle, "reset_input_buffer"):
                        port_handle.reset_input_buffer()
                    if port_handle and hasattr(port_handle, "reset_output_buffer"):
                        port_handle.reset_output_buffer()
                except Exception:
                    pass
                self.mav_ser = conn
                self._serial_retry_at = time.time() + self.SERIAL_RETRY_SUCCESS_HOLD
                self.log(f"[RELAY] serial open: {port} @ {baud}")
            except SerialException as e:
                self._serial_retry_at = time.time() + self.SERIAL_RETRY_INTERVAL
                self.log(f"[RELAY] serial error: {e}")
            except Exception as e:
                self._serial_retry_at = time.time() + self.SERIAL_RETRY_INTERVAL
                self.log(f"[RELAY] mavlink open error: {e}")

    def _drop_serial_connection(self, *, schedule_retry: bool = True) -> None:
        with self._serial_lock:
            self._drop_serial_connection_locked(schedule_retry=schedule_retry)

    def _drop_serial_connection_locked(self, *, schedule_retry: bool) -> None:
        if self.mav_ser:
            try:
                port_handle = getattr(self.mav_ser, "port", None)
                if port_handle and hasattr(port_handle, "reset_input_buffer"):
                    port_handle.reset_input_buffer()
                if port_handle and hasattr(port_handle, "reset_output_buffer"):
                    port_handle.reset_output_buffer()
            except Exception:
                pass
            try:
                self.mav_ser.close()
            except Exception:
                pass
        self.mav_ser = None
        if schedule_retry:
            self._serial_retry_at = time.time() + self.SERIAL_RETRY_INTERVAL
        else:
            self._serial_retry_at = 0.0

    def _ensure_serial_connection(self) -> None:
        if not (self.enable_of_serial or self.enable_distance_serial):
            return
        port = str(self.s.get("serial_port", "")).strip()
        if not port:
            return
        now = time.time()
        with self._serial_lock:
            if self.mav_ser:
                return
            if now < self._serial_retry_at:
                return
        self._open_serial_shared()

    def _handle_serial_send_error(self, context: str, exc: Exception) -> None:
        self.log(f"[RELAY] {context} error: {exc}")
        self._drop_serial_connection(schedule_retry=True)

    def _emit_status(self, text: str) -> None:
        try:
            if self.status_cb:
                self.status_cb(text)
        except Exception:
            pass

    def register_rover_logger(self, rover_logger: Optional["RoverRelayLogger"]) -> None:
        self._rover_logger = rover_logger

    def notify_rover_logging_changed(self) -> None:
        if self.running:
            self._open_gazebo_log(reset_counter=False)
        else:
            _, reason = self._should_enable_gazebo_log()
            with self._gazebo_log_lock:
                self._gazebo_log_block_reason = reason or ""
                if reason:
                    self._gazebo_log_error = reason
                else:
                    self._gazebo_log_error = ""

    def is_gazebo_logging_active(self) -> bool:
        with self._gazebo_log_lock:
            return bool(self._gazebo_log_file)

    def is_gazebo_logging_enabled(self) -> bool:
        return bool(self._gazebo_logging_enabled)

    def _is_rover_logging_active(self) -> bool:
        rover = self._rover_logger
        if rover is None:
            return False
        try:
            return bool(rover.is_logging_active())
        except Exception:
            return False

    def _should_enable_gazebo_log(self) -> Tuple[bool, Optional[str]]:
        if not self._gazebo_logging_enabled:
            return False, "Disabled by settings"
        if self._is_rover_logging_active():
            return False, "Disabled: Rover logging active"
        return True, None

    def _open_gazebo_log(self, reset_counter: bool = True) -> None:
        # 최신 설정 적용
        self.gazebo_log_path = str(self.s.get("gazebo_log_path", self.gazebo_log_path)).strip()
        path = self.gazebo_log_path
        allow, reason = self._should_enable_gazebo_log()
        with self._gazebo_log_lock:
            if not allow:
                if self._gazebo_log_file:
                    try:
                        self._gazebo_log_file.close()
                    except Exception:
                        pass
                    self._gazebo_log_file = None
                if reset_counter:
                    self._gazebo_log_count = 0
                self._gazebo_log_active = False
                self._gazebo_log_error = reason or ""
                self._gazebo_log_block_reason = reason or ""
                self._gazebo_log_closing = False
                return

            self._gazebo_log_block_reason = ""
            if not path:
                if self._gazebo_log_file:
                    try:
                        self._gazebo_log_file.close()
                    except Exception:
                        pass
                    self._gazebo_log_file = None
                if reset_counter:
                    self._gazebo_log_count = 0
                self._gazebo_log_active = False
                self._gazebo_log_error = ""
                self._gazebo_log_closing = False
                return

            if self._gazebo_log_file and not reset_counter:
                # 이미 열린 파일을 계속 사용
                self._gazebo_log_closing = False
                return

            if self._gazebo_log_file:
                try:
                    self._gazebo_log_file.close()
                except Exception:
                    pass
                self._gazebo_log_file = None
            if reset_counter:
                self._gazebo_log_count = 0
            try:
                directory = os.path.dirname(path)
                if directory:
                    os.makedirs(directory, exist_ok=True)
                write_header = not os.path.exists(path) or os.path.getsize(path) == 0
                self._gazebo_log_file = open(path, "a", encoding="utf-8")
                if write_header:
                    self._gazebo_log_file.write(UNIFIED_CSV_HEADER + "\n")
                self._gazebo_log_file.flush()
                self._gazebo_log_active = True
                self._gazebo_log_error = ""
                self._gazebo_log_start_ts = time.time()
                self._gazebo_log_last_ts = self._gazebo_log_start_ts
                self._gazebo_log_closing = False
            except Exception as e:
                self._gazebo_log_error = str(e)
                self._gazebo_log_active = False
                self._gazebo_log_file = None
                self._gazebo_log_closing = False
                self.log(f"[RELAY] Gazebo log open error: {e}")

    def _close_gazebo_log(self) -> None:
        with self._gazebo_log_lock:
            closing = bool(self._gazebo_log_file)
            if closing:
                self._gazebo_log_closing = True
            else:
                self._gazebo_log_closing = False
        if closing:
            self._gazebo_log_queue.join()
        with self._gazebo_log_lock:
            fh = self._gazebo_log_file
            if fh:
                try:
                    fh.flush()
                except Exception:
                    pass
                try:
                    fh.close()
                except Exception:
                    pass
            self._gazebo_log_file = None
            self._gazebo_log_active = False
            self._gazebo_log_block_reason = ""
            self._gazebo_log_closing = False

    def _write_gazebo_log(
        self,
        *,
        time_usec: int,
        position: tuple[float, float, float],
        orientation_wxyz: tuple[float, float, float, float],
        angular_velocity: tuple[float, float, float],
    ) -> None:
        with self._gazebo_log_lock:
            active = self._gazebo_log_file is not None and not self._gazebo_log_closing
        if not active:
            return
        self._gazebo_log_queue.put((
            int(time_usec),
            tuple(float(v) for v in position),
            tuple(float(v) for v in orientation_wxyz),
            tuple(float(v) for v in angular_velocity),
        ))

    def _gazebo_log_worker_loop(self) -> None:
        while True:
            try:
                item = self._gazebo_log_queue.get()
            except Exception:
                return
            try:
                if item is None:
                    break
                try:
                    time_usec, position, orientation_wxyz, angular_velocity = item
                except (TypeError, ValueError):
                    continue
                try:
                    line = format_drone_csv_row(
                        time_usec=time_usec,
                        position=position,
                        orientation_wxyz=orientation_wxyz,
                        angular_velocity=angular_velocity,
                    )
                except ValueError as exc:
                    with self._gazebo_log_lock:
                        self._gazebo_log_error = str(exc)
                        self._gazebo_log_active = False
                    continue
                with self._gazebo_log_lock:
                    fh = self._gazebo_log_file
                if not fh:
                    continue
                try:
                    fh.write(line + "\n")
                    fh.flush()
                except Exception as exc:  # noqa: BLE001
                    with self._gazebo_log_lock:
                        self._gazebo_log_error = str(exc)
                        self._gazebo_log_active = False
                        current = self._gazebo_log_file
                        self._gazebo_log_file = None
                    if current:
                        try:
                            current.close()
                        except Exception:
                            pass
                    self.log(f"[RELAY] Gazebo log write error: {exc}")
                else:
                    now = time.time()
                    with self._gazebo_log_lock:
                        self._gazebo_log_count += 1
                        self._gazebo_log_last_ts = now
                        self._gazebo_log_active = True
                        self._gazebo_log_error = ""
            finally:
                self._gazebo_log_queue.task_done()

    # ------------- Gazebo 루프 -------------
    def _gazebo_loop(self) -> None:
        """
        - Gazebo UDP 패킷 수신:
            * 그대로 ExternalCtrl 목적지로 relay (지연 최소)
            * 동시에 OPTICAL_FLOW(#100) 값 생성해 시리얼 송신
        """
        ext_ip = self.s.get("ext_udp_ip", "127.0.0.1")
        ext_port = int(self.s.get("ext_udp_port", 9091))  # 기본 9091
        dst_tuple = (ext_ip, ext_port)

        buffer = bytearray(4096)
        view = memoryview(buffer)

        while not self.stop_ev.is_set():
            sock = self.sock_gazebo
            if not sock:
                break
            try:
                size, _ = sock.recvfrom_into(view)
            except OSError:
                break
            except Exception as e:
                self.log(f"[RELAY] gazebo loop error: {e}")
                continue

            if size <= 0:
                continue

            payload_view = view[:size]

            ext_sock = self.sock_ext
            if ext_sock:
                try:
                    ext_sock.sendto(payload_view, dst_tuple)
                except Exception as e:
                    self.log(f"[RELAY] ext send error: {e}")

            if size < relay_icd.GAZEBO_STRUCT_SIZE:
                continue

            try:
                (
                    _header_type, _msg_type, _msg_size, _reserved,
                    _unique_id, time_usec,
                    px, py, pz,
                    qw, qx, qy, qz,
                    ax, ay, az,
                    wx, wy, wz
                ) = struct.unpack_from(
                    relay_icd.GAZEBO_STRUCT_FMT,
                    payload_view,
                    0,
                )
            except struct.error:
                continue

            time_usec_int = int(time_usec)
            px_f, py_f, pz_f = float(px), float(py), float(pz)
            qw_f, qx_f, qy_f, qz_f = float(qw), float(qx), float(qy), float(qz)
            ax_f, ay_f, az_f = float(ax), float(ay), float(az)
            wx_f, wy_f, wz_f = float(wx), float(wy), float(wz)

            ground_distance = max(0.0, pz_f)

            flow_comp_m_x = wy_f * ground_distance   # +Y rate → +X flow(m/s) (가정)
            flow_comp_m_y = -wx_f * ground_distance  # +X rate → -Y flow(m/s)

            quality = self._estimate_quality(ax_f, ay_f, az_f, wx_f, wy_f, wz_f)

            scale_pix = self.of_scale_pix
            flow_x = int(max(-32768, min(32767, flow_comp_m_x * scale_pix)))
            flow_y = int(max(-32768, min(32767, flow_comp_m_y * scale_pix)))

            sent = self._send_optical_flow(
                time_usec=time_usec_int,
                sensor_id=int(self.s.get("flow_sensor_id", 0)),
                flow_x=flow_x,
                flow_y=flow_y,
                flow_comp_m_x=float(flow_comp_m_x),
                flow_comp_m_y=float(flow_comp_m_y),
                quality=int(quality),
                ground_distance=float(ground_distance),
            )

            with self._lock:
                self.last_ground_distance = ground_distance
                self.last_av = (wx_f, wy_f, wz_f)
                self.of_quality = quality
                if sent:
                    self.last_of_send_ts = time.time()

            self._write_gazebo_log(
                time_usec=time_usec_int,
                position=(px_f, py_f, pz_f),
                orientation_wxyz=(qw_f, qx_f, qy_f, qz_f),
                angular_velocity=(wx_f, wy_f, wz_f),
            )

    # ------------- Distance RAW UDP 루프 -------------
    def _distance_raw_udp_loop(self) -> None:
        """
        ImageGenerator → RAW UDP 거리센서 수신
        - 39B payload-only (헤더 없음) 자동 인식
        - 또는 10B 헤더 + 39B payload (msg_type=10708) 호환
        → 상태 업데이트 + 공용 시리얼로 MAVLink DISTANCE_SENSOR 재송신
        """
        if not self.sock_dist:
            return

        last_log = time.time()
        rx_cnt = 0

        while not self.stop_ev.is_set():
            try:
                pkt, _ = self.sock_dist.recvfrom(2048)
                if not pkt:
                    continue

                # 케이스 1: 39B 페이로드 단독
                if len(pkt) == relay_icd.DIST_PAYLOAD_SIZE:
                    off = 0

                # 케이스 2: 10B 헤더 + 39B 페이로드
                elif len(pkt) >= (relay_icd.DIST_HDR_SIZE + relay_icd.DIST_PAYLOAD_SIZE):
                    try:
                        hdr_type, msg_type, msg_size, reserved = struct.unpack_from(relay_icd.DIST_HDR_FMT, pkt, 0)
                    except struct.error:
                        continue
                    if msg_type != relay_icd.DIST_MSG_TYPE or msg_size != relay_icd.DIST_PAYLOAD_SIZE:
                        # 다른 타입이면 스킵
                        continue
                    off = relay_icd.DIST_HDR_SIZE
                else:
                    # 지원 길이 아님
                    continue

                # payload 파싱
                try:
                    (time_boot_ms,
                     min_cm, max_cm, cur_cm,
                     type_, sid, orientation, covariance,
                     hfov, vfov,
                     qw, qx, qy, qz,
                     sigq) = struct.unpack_from(relay_icd.DIST_PAYLOAD_FMT, pkt, off)
                except struct.error:
                    continue

                # 상태 업데이트 (m 단위)
                with self._lock:
                    self.current_distance_m = float(cur_cm) / 100.0
                    self.last_distance_update = time.time()

                # 시리얼로 MAVLink DISTANCE_SENSOR 표준 전송 (확장 필드는 시리얼 전송 생략)
                self._ensure_serial_connection()
                if self.mav_ser and self.enable_distance_serial:
                    try:
                        self.mav_ser.mav.distance_sensor_send(
                            int(time_boot_ms), int(min_cm), int(max_cm), int(cur_cm),
                            int(type_), int(sid), int(orientation), int(covariance)
                        )
                    except Exception as e:
                        self._handle_serial_send_error("distance serial send", e)

                rx_cnt += 1
                if time.time() - last_log > 3.0:
                    self.log(f"[RELAY] DIST RAW UDP rx {rx_cnt} msgs in last 3s")
                    rx_cnt = 0
                    last_log = time.time()

            except OSError:
                break
            except Exception as e:
                self.log(f"[RELAY] distance raw udp error: {e}")

    # ------------- Distance MAVLink UDP 루프 (옵션) -------------
    def _distance_mav_udp_loop(self) -> None:
        """
        ImageGenerator가 UDP로 송출하는 MAVLink DISTANCE_SENSOR를 수신 → 시리얼로 재송신
        (distance_mode='mavlink'일 때만 사용)
        """
        if not self.mav_udp:
            return
        last_log = time.time()
        rx_cnt = 0
        while not self.stop_ev.is_set():
            try:
                m = self.mav_udp.recv_match(blocking=True, timeout=0.2)
                if not m:
                    if time.time() - last_log > 3.0:
                        self.log("[RELAY] DIST UDP(MAV) no msgs in last 3s")
                        last_log = time.time()
                    continue
                if m.get_type() == "DISTANCE_SENSOR":
                    rx_cnt += 1
                    min_cm = int(getattr(m, "min_distance", 0))
                    max_cm = int(getattr(m, "max_distance", 0))
                    cur     = getattr(m, "current_distance", 0)

                    # 단위 휴리스틱
                    if isinstance(cur, float):
                        cur_cm = int(cur * 100.0)
                    else:
                        cur_cm = int(cur if cur > 10 else cur * 100)

                    self._ensure_serial_connection()
                    if self.mav_ser and self.enable_distance_serial:
                        try:
                            self.mav_ser.mav.distance_sensor_send(
                                int(getattr(m, "time_boot_ms", 0)),
                                int(min_cm), int(max_cm), int(cur_cm),
                                int(getattr(m, "type", 0)),
                                int(getattr(m, "id", 0)),
                                int(getattr(m, "orientation", 0)),
                                int(getattr(m, "covariance", 0)),
                            )
                        except Exception as e:
                            self._handle_serial_send_error("distance serial send", e)

                    with self._lock:
                        self.current_distance_m = cur_cm / 100.0
                        self.last_distance_update = time.time()

                if time.time() - last_log > 3.0:
                    self.log(f"[RELAY] DIST UDP(MAV) rx {rx_cnt} msgs in last 3s")
                    rx_cnt = 0
                    last_log = time.time()

            except Exception as e:
                self.log(f"[RELAY] distance udp(mav) error: {e}")

    # ------------- Heartbeat 루프 -------------
    def _sensor_heartbeat_loop(self) -> None:
        """
        Optical Flow / Distance 센서 Heartbeat를 각자 설정된 Hz로 송신
        같은 시리얼 포트를 공유하며, 송신 전 srcSystem/srcComponent를 변경해 보냄
        """
        next_of = 0.0
        next_ds = 0.0
        while not self.stop_ev.is_set():
            now = time.time()
            try:
                self._ensure_serial_connection()
                mav_conn = self.mav_ser
                if mav_conn:
                    # Optical Flow HB
                    if self.enable_of_serial and self.hb_of_rate_hz > 0 and now >= next_of:
                        mav_conn.mav.srcSystem = self.hb_of_sysid
                        mav_conn.mav.srcComponent = self.hb_of_compid
                        mav_conn.mav.heartbeat_send(
                            self.hb_of_type, self.hb_of_autop,
                            self.hb_of_mode, self.hb_of_cus, self.hb_of_stat
                        )
                        next_of = now + 1.0 / max(0.001, self.hb_of_rate_hz)

                    # Distance HB
                    if self.enable_distance_serial and self.hb_ds_rate_hz > 0 and now >= next_ds:
                        mav_conn.mav.srcSystem = self.hb_ds_sysid
                        mav_conn.mav.srcComponent = self.hb_ds_compid
                        mav_conn.mav.heartbeat_send(
                            self.hb_ds_type, self.hb_ds_autop,
                            self.hb_ds_mode, self.hb_ds_cus, self.hb_ds_stat
                        )
                        next_ds = now + 1.0 / max(0.001, self.hb_ds_rate_hz)
            except Exception as e:
                self._handle_serial_send_error("HB", e)
            time.sleep(0.01)

    # ------------- MAV 송신 유틸 -------------
    def _send_optical_flow(
        self,
        time_usec: int,
        sensor_id: int,
        flow_x: int,
        flow_y: int,
        flow_comp_m_x: float,
        flow_comp_m_y: float,
        quality: int,
        ground_distance: float,
    ) -> bool:
        """MAVLink OPTICAL_FLOW (#100) 전송"""
        self._ensure_serial_connection()
        mav_conn = self.mav_ser
        if not mav_conn or not self.enable_of_serial:
            return False
        try:
            mav_conn.mav.optical_flow_send(
                int(time_usec),
                int(sensor_id) & 0xFF,
                int(flow_x), int(flow_y),
                float(flow_comp_m_x), float(flow_comp_m_y),
                int(max(0, min(255, quality))),
                float(ground_distance)
            )
            return True
        except Exception as e:
            self._handle_serial_send_error("optical_flow send", e)
            return False

    # ------------- 품질 추정(설정 반영) -------------
    def _estimate_quality(self, ax: float, ay: float, az: float, wx: float, wy: float, wz: float) -> int:
        """
        간단 품질 추정:
          - 가속/각속도가 과도하면 품질 저하
          - 0~255 범위 (UI 설정 반영)
        """
        a_mag = math.sqrt(ax*ax + ay*ay + az*az)
        w_mag = math.sqrt(wx*wx + wy*wy + wz*wz)
        q = float(self.q_base)
        if a_mag > self.accel_thresh:
            q -= min(1000.0, (a_mag - self.accel_thresh) * self.accel_penalty)
        if w_mag > self.gyro_thresh:
            q -= min(1000.0, (w_mag - self.gyro_thresh) * self.gyro_penalty)
        q = max(self.q_min, min(self.q_max, q))
        return int(q)
