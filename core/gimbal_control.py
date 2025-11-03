# core/gimbal_control.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Dict, Any, Iterable, List, Set, Tuple

from pymavlink import mavutil
try:
    from serial import SerialException
except ImportError:
    from serial.serialutil import SerialException

# ❌ utils.helpers 의존성 없음
from utils.zoom import zoom_scale_to_lens_mm
from network.bridge_tcp import parse_bridge_tcp_command
from network.gimbal_icd import (
    MC_HB_TYPE,
    GIMBAL_STATUS_FLAGS,
    PARAM_TABLE,
    PARAM_COUNT,
    TCP_CMD_GET_STATUS,
    TCP_CMD_SET_TARGET,
    TCP_CMD_SET_ZOOM,
)
from network.gimbal_messages import (
    StatusSnapshot,
    build_gimbal_ctrl_packet,
    build_power_ctrl_packet,
    build_status_frame,
    parse_set_target,
    parse_set_zoom,
)

def _format_value(
    value: Any,
    *,
    to_degrees: bool = False,
    precision: int = 1,
    suffix: str = "",
) -> str:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "--"
    numeric = float(value)
    if to_degrees:
        numeric = math.degrees(numeric)
    return f"{numeric:.{precision}f}{suffix}"


def _format_sequence(
    values: Iterable[Any],
    *,
    to_degrees: bool = False,
    precision: int = 1,
    suffix: str = "",
) -> str:
    return ", ".join(
        _format_value(v, to_degrees=to_degrees, precision=precision, suffix=suffix)
        for v in values
    )

def _quat_to_frotator_deg(qw: float, qx: float, qy: float, qz: float) -> Tuple[float, float, float]:
    """(w, x, y, z) 쿼터니언을 Unreal/Sim (Pitch, Yaw, Roll) 각도로 변환합니다.
    (이 함수는 MAVLink 수신부에서 RPY 리맵핑 요구사항을 구현하기 위해 유지됩니다.)
    """
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    w, x, y, z = qw / n, qx / n, qy / n, qz / n

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return pitch, yaw, roll

class GimbalControl:
    def __init__(
        self,
        log_cb,
        status_cb,
        settings: Dict[str, Any],
        *,
        zoom_update_cb: Optional[Callable[[float], None]] = None,
    ) -> None:
        self.log = log_cb
        self.status_cb = status_cb
        self.s = dict(settings)

        self.control_method = self._normalize_control_method(self.s.get("control_method", "tcp"))
        self.s["control_method"] = self.control_method

        self._zoom_update_cb = zoom_update_cb

        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.settimeout(0.2)

        self._lock = threading.Lock()
        self.pos = [float(self.s.get("pos_x", 0.0)),
                    float(self.s.get("pos_y", 0.0)),
                    float(self.s.get("pos_z", 0.0))]
        
        # RPY (섀도우 상태 - 입력 목표 저장용)
        # ❌ RPY 리맵핑이 제거되었으므로 'Internal RPY'와 'Sim RPY'는 동일합니다.
        #    (R, P, Y) 순서로 저장합니다.
        self.rpy_tgt = [float(self.s.get("init_roll_deg", 0.0)),
                        float(self.s.get("init_pitch_deg", 0.0)),
                        float(self.s.get("init_yaw_deg", 0.0))]
        self.max_rate_dps = float(self.s.get("max_rate_dps", 60.0))

        # 쿼터니언 제어기 상태 변수 (x,y,z,w)
        try:
            self.q_tgt = self._rpy_to_quat(self.rpy_tgt[0], self.rpy_tgt[1], self.rpy_tgt[2])
            self.q_cur = self.q_tgt[:]
        except Exception as e:
            self.log(f"[GIMBAL] euler_to_quat_init failed: {e}. Defaulting.")
            self.q_tgt = [0.0, 0.0, 0.0, 1.0]
            self.q_cur = [0.0, 0.0, 0.0, 1.0]
        
        # 쿼터니언 기반 각속도 계산용 변수
        self.q_prev = self.q_cur[:] # 이전 쿼터니언 상태
        self.w_cur = [0.0, 0.0, 0.0]  # 현재 몸체 각속도 (wx, wy, wz)
        
        self.power_on = bool(self.s.get("power_on", True))
        self.zoom_scale = max(1.0, float(self.s.get("zoom_scale", 1.0)))

        self.sensor_type = int(self.s.get("sensor_type", 0)) & 0xFF
        self.sensor_id   = int(self.s.get("sensor_id", 0)) & 0xFF
        self.s["sensor_type"] = self.sensor_type
        self.s["sensor_id"] = self.sensor_id

        self.mavlink_preset_index = int(self.s.get("mavlink_preset_index", 0))
        self.mavlink_sensor_type = int(self.s.get("mavlink_sensor_type", self.sensor_type)) & 0xFF
        self.mavlink_sensor_id = int(self.s.get("mavlink_sensor_id", self.sensor_id)) & 0xFF
        self.s["mavlink_preset_index"] = self.mavlink_preset_index
        self.s["mavlink_sensor_type"] = self.mavlink_sensor_type
        self.s["mavlink_sensor_id"] = self.mavlink_sensor_id

        self.mav_sys_id  = int(self.s.get("mav_sysid", 1))
        self.mav_comp_id = int(self.s.get("mav_compid", 154))

        self.rx_ip = self.s.get("bind_ip", "0.0.0.0")
        self.rx_port = int(self.s.get("bind_port", 16060))

        self.debug_dump_packets = bool(self.s.get("debug_dump_packets", False))
        self.s["debug_dump_packets"] = self.debug_dump_packets

        self.stop_ev = threading.Event()
        self.ctrl_thread = None
        self.mav_rx_thread = None
        self.mav_tx_thread = None
        self._tcp_thread: Optional[threading.Thread] = None
        self._tcp_sock: Optional[socket.socket] = None
        self._tcp_clients: Set[socket.socket] = set()
        self._tcp_clients_lock = threading.Lock()
        self._tcp_stop = threading.Event()
        self._mav_stop = threading.Event()

        self._mav_lock = threading.Lock()
        self.mav: Optional[mavutil.mavfile] = None
        self.serial_port = str(self.s.get("serial_port", "") or "").strip()
        self.s["serial_port"] = self.serial_port
        self.serial_baud = int(self.s.get("serial_baud", 115200))
        self.s["serial_baud"] = self.serial_baud
        if self.control_method == "mavlink" and not self.serial_port:
            self.log("[GIMBAL] No serial port configured; falling back to TCP control")
            self.control_method = "tcp"
            self.s["control_method"] = self.control_method
        self.hb_rx_ok = False
        self.last_hb_rx = 0.0

        self._last_ts = time.time()
        
        self._last_sent_snapshot: Optional[
            Tuple[int, int, Tuple[float, float, float], Tuple[float, float, float, float]]
        ] = None
        
        if self._zoom_update_cb:
            try:
                self._zoom_update_cb(self.zoom_scale)
            except Exception as exc:
                self.log(f"[GIMBAL] zoom callback init error: {exc}")

    # -------- lifecycle --------
    @staticmethod
    def _sanitize_sensor_code(value: Any) -> int:
        try:
            return int(value) & 0xFF
        except Exception:
            return 0

    def _active_sensor_codes_locked(self) -> Tuple[int, int]:
        if self.control_method == "mavlink":
            sensor_type = self.mavlink_sensor_type
            sensor_id = self.mavlink_sensor_id
        else:
            sensor_type = self.sensor_type
            sensor_id = self.sensor_id
        return sensor_type & 0xFF, sensor_id & 0xFF

    def _active_sensor_codes(self) -> Tuple[int, int]:
        with self._lock:
            return self._active_sensor_codes_locked()

    @staticmethod
    def _normalize_control_method(value: Any) -> str:
        if isinstance(value, str):
            lowered = value.lower()
            if lowered in ("tcp", "mavlink"):
                return lowered
        return "tcp"

    def start(self) -> None:
        self.stop_ev.clear()
        if not self.ctrl_thread or not self.ctrl_thread.is_alive():
            self.ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
            self.ctrl_thread.start()
        self._apply_control_method_runtime(restart=True)
        self._emit_status("RUNNING")
        self.log("[GIMBAL] started")

    def stop(self) -> None:
        self.stop_ev.set()
        try: self.tx_sock.close()
        except Exception: pass
        self._stop_tcp()
        self._stop_mavlink_threads()
        self._emit_status("STOPPED")
        self.log("[GIMBAL] stopped")

    def update_settings(self, settings: Dict[str, Any]) -> None:
        restart_tcp = False
        method_changed = False
        serial_changed = False
        baud_changed = False
        new_zoom: Optional[float] = None
        with self._lock:
            requested_method = self._normalize_control_method(
                settings.get("control_method", self.control_method)
            )
            requested_serial = str(settings.get("serial_port", self.serial_port) or "").strip()
            requested_baud = self.serial_baud
            if "serial_baud" in settings:
                try:
                    requested_baud = int(settings.get("serial_baud", self.serial_baud))
                except (TypeError, ValueError):
                    requested_baud = self.serial_baud
            if requested_method == "mavlink" and not requested_serial:
                raise ValueError("MAVLink control requires a serial port before activation")
            self.s.update(settings)
            self.sensor_type = self._sanitize_sensor_code(self.s.get("sensor_type", self.sensor_type))
            self.sensor_id   = self._sanitize_sensor_code(self.s.get("sensor_id", self.sensor_id))
            self.s["sensor_type"] = self.sensor_type
            self.s["sensor_id"] = self.sensor_id
            self.max_rate_dps = float(self.s.get("max_rate_dps", self.max_rate_dps))
            self.pos = [float(self.s.get("pos_x", self.pos[0])),
                        float(self.s.get("pos_y", self.pos[1])),
                        float(self.s.get("pos_z", self.pos[2]))]
            self.power_on = bool(self.s.get("power_on", self.power_on))
            if "zoom_scale" in self.s:
                new_zoom = max(1.0, float(self.s.get("zoom_scale", self.zoom_scale)))
            if "mav_sysid" in self.s:  self.mav_sys_id  = int(self.s["mav_sysid"])
            if "mav_compid" in self.s: self.mav_comp_id = int(self.s["mav_compid"])
            if requested_method != self.control_method:
                self.control_method = requested_method
                method_changed = True
            self.s["control_method"] = self.control_method
            if "bind_ip" in self.s:
                new_ip = self.s.get("bind_ip", self.rx_ip)
                if new_ip != self.rx_ip:
                    self.rx_ip = new_ip
                    restart_tcp = True
            if "bind_port" in self.s:
                new_port = int(self.s.get("bind_port", self.rx_port))
                if new_port != self.rx_port:
                    self.rx_port = new_port
                    restart_tcp = True
            if "debug_dump_packets" in settings:
                self.debug_dump_packets = bool(self.s.get("debug_dump_packets", self.debug_dump_packets))
                self.s["debug_dump_packets"] = self.debug_dump_packets
            
            if {"sensor_type", "sensor_id", "mavlink_sensor_type", "mavlink_sensor_id"}.intersection(settings.keys()):
                self._invalidate_cached_orientation_locked()
            
            if "mavlink_preset_index" in settings:
                self.mavlink_preset_index = int(self.s.get("mavlink_preset_index", self.mavlink_preset_index))
                self.s["mavlink_preset_index"] = self.mavlink_preset_index
            if "mavlink_sensor_type" in settings:
                self.mavlink_sensor_type = self._sanitize_sensor_code(settings.get("mavlink_sensor_type", self.mavlink_sensor_type))
                self.s["mavlink_sensor_type"] = self.mavlink_sensor_type
            if "mavlink_sensor_id" in settings:
                self.mavlink_sensor_id = self._sanitize_sensor_code(settings.get("mavlink_sensor_id", self.mavlink_sensor_id))
                self.s["mavlink_sensor_id"] = self.mavlink_sensor_id

            if requested_serial != self.serial_port:
                self.serial_port = requested_serial
                serial_changed = True
            self.s["serial_port"] = self.serial_port
            if requested_baud != self.serial_baud:
                self.serial_baud = requested_baud
                baud_changed = True
            self.s["serial_baud"] = self.serial_baud

        self.log("[GIMBAL] settings updated")
        if method_changed:
            self.log(f"[GIMBAL] control method -> {self.control_method.upper()}")
        if restart_tcp and self.control_method == "tcp":
            self.log("[GIMBAL] restarting TCP listener due to bind change")
            self._stop_tcp()
        if self.control_method == "tcp":
            if restart_tcp or method_changed:
                self._start_tcp()
            self._stop_mavlink_threads()
        else:
            if method_changed or serial_changed or baud_changed:
                self._stop_mavlink_threads()
            self._stop_tcp()
            self._ensure_mavlink_running(force_restart=method_changed or serial_changed or baud_changed)
        if new_zoom is not None:
            self._set_zoom_scale(new_zoom)

    # -------- public controls --------

    def update_control_method(self, method: str) -> None:
        """Switch between TCP and MAVLink control modes at runtime."""
        requested = self._normalize_control_method(method)
        if requested == self.control_method:
            return
        self.update_settings({"control_method": requested})

    def _apply_control_method_runtime(self, restart: bool = False) -> None:
        if self.control_method == "mavlink":
            if restart:
                self._stop_mavlink_threads()
            self._stop_tcp()
            self._ensure_mavlink_running(force_restart=restart)
        else:
            if restart:
                self._stop_tcp()
            self._start_tcp()
            self._stop_mavlink_threads()

    def _ensure_mavlink_running(self, force_restart: bool = False) -> None:
        if self.control_method != "mavlink":
            return
        if not self.serial_port:
            if force_restart:
                self.log("[GIMBAL] MAVLink mode requested without a serial port; skipping activation")
            return
        if force_restart:
            self._stop_mavlink_threads()
        if self.mav_rx_thread and self.mav_rx_thread.is_alive() and not self._mav_stop.is_set():
            return
        self._open_serial()

    def _stop_mavlink_threads(self) -> None:
        self._mav_stop.set()
        with self._mav_lock:
            mav = self.mav
            self.mav = None
        if mav:
            try: mav.close()
            except Exception: pass
        for th in (self.mav_rx_thread, self.mav_tx_thread):
            if th and th.is_alive():
                try: th.join(timeout=0.5)
                except Exception: pass
        self.mav_rx_thread = None
        self.mav_tx_thread = None
        self._mav_stop.clear()
        self.hb_rx_ok = False

    def set_target_pose(
        self,
        x: float,
        y: float,
        z: float,
        sim_pitch_deg: float, # ✅ 이 함수는 (P, Y, R) 입력을 받는 것을 유지
        sim_yaw_deg: float,
        sim_roll_deg: float,
        *,
        persist: bool = False,
        log: bool = True,
    ) -> None:
        """
        (TCP/UI용) Sim RPY (P,Y,R) 입력을 받아 내부 목표 쿼터니언을 설정합니다.
        """
        # ❌ 1. RPY 리맵핑 제거
        # internal_roll, internal_pitch, internal_yaw = self._sim_to_internal_rpy(
        #     sim_pitch_deg, sim_yaw_deg, sim_roll_deg
        # )
        
        # ✅ 1. 입력받은 (R, P, Y)를 직접 사용
        roll_deg = sim_roll_deg
        pitch_deg = sim_pitch_deg
        yaw_deg = sim_yaw_deg
        
        # 2. (R,P,Y) -> 새로운 쿼터니언 목표
        try:
            q_new_target = self._rpy_to_quat(roll_deg, pitch_deg, yaw_deg)
        except Exception as e:
            self.log(f"[Gimbal] euler_to_quat_apply failed: {e}")
            q_new_target = self.q_tgt[:] # 실패 시 이전 목표 유지

        with self._lock:
            # ✅ 3. Anti-Flip: 현재 쿼터니언(q_cur)과 비교해 최단 경로 쿼터니언을 q_tgt로 설정
            dot = self._q_dot(q_new_target, self.q_cur)
            
            if dot < 1e-9: 
                self.q_tgt = [-val for val in q_new_target]
            else:
                self.q_tgt = q_new_target[:]
            
            # 4. 위치 및 섀도우 RPY 목표 설정 (R, P, Y 순서로 저장)
            self.pos[:] = [float(x), float(y), float(z)]
            self.rpy_tgt[:] = [roll_deg, pitch_deg, yaw_deg]
            
            if persist:
                self.s["pos_x"], self.s["pos_y"], self.s["pos_z"] = self.pos
                self.s["init_roll_deg"], self.s["init_pitch_deg"], self.s["init_yaw_deg"] = self.rpy_tgt
            
            self._invalidate_cached_orientation_locked()

        if log:
            self.log(
                f"[GIMBAL] target pose set → xyz=({x:.2f},{y:.2f},{z:.2f}), "
                f"rpy(R,P,Y)=({roll_deg:.1f},{pitch_deg:.1f},{yaw_deg:.1f})"
            )

    def set_max_rate(self, rate_dps: float) -> None:
        with self._lock:
            self.max_rate_dps = float(rate_dps)
        self.log(f"[GIMBAL] max rate = {rate_dps:.1f} dps")

    def set_power(self, on: bool) -> None:
        with self._lock:
            self.power_on = bool(on)
        self.log(f"[GIMBAL] power state updated (no send) -> {'ON' if on else 'OFF'}")

    def build_power_packet(
        self,
        on: bool,
        *,
        sensor_type: Optional[int] = None,
        sensor_id: Optional[int] = None,
    ) -> bytes:
        """Return the raw SensorPowerCtrl packet for the requested sensor."""
        with self._lock:
            if sensor_type is None or sensor_id is None:
                sensor_type_i, sensor_id_i = self._active_sensor_codes_locked()
            else:
                sensor_type_i = self._sanitize_sensor_code(sensor_type)
                sensor_id_i = self._sanitize_sensor_code(sensor_id)
        return self._pack_power_ctrl(sensor_type_i, sensor_id_i, int(bool(on)))

    def get_power_packet_example(
        self,
        on: bool,
        *,
        sensor_type: Optional[int] = None,
        sensor_id: Optional[int] = None,
    ) -> bytearray:
        """Provide a bytearray sample for UI inspection."""
        return bytearray(
            self.build_power_packet(on, sensor_type=sensor_type, sensor_id=sensor_id)
        )

    def send_power(
        self,
        on: bool,
        *,
        sensor_type: Optional[int] = None,
        sensor_id: Optional[int] = None,
    ) -> bytes:
        sensor_type_override = None if sensor_type is None else self._sanitize_sensor_code(sensor_type)
        sensor_id_override = None if sensor_id is None else self._sanitize_sensor_code(sensor_id)
        packet = self.build_power_packet(
            on,
            sensor_type=sensor_type_override,
            sensor_id=sensor_id_override,
        )
        with self._lock:
            self.power_on = bool(on)
            if sensor_type_override is not None:
                self.sensor_type = sensor_type_override
                self.s["sensor_type"] = self.sensor_type
            if sensor_id_override is not None:
                self.sensor_id = sensor_id_override
                self.s["sensor_id"] = self.sensor_id
            sensor_type, sensor_id = self._active_sensor_codes_locked()
            target_ip = str(self.s.get("generator_ip", "127.0.0.1"))
            target_port = int(self.s.get("generator_port", 15020))
        try:
            self.tx_sock.sendto(packet, (target_ip, target_port))
            self.log(
                f"[GIMBAL] POWER {'ON' if on else 'OFF'} sent -> "
                f"sensor={sensor_type}/{sensor_id} target={target_ip}:{target_port}"
            )
            if self.debug_dump_packets:
                self._dump_packet_bytes("POWER", packet)
        except Exception as e:
            self.log(f"[GIMBAL] power send error: {e}")
        return packet

    def send_udp_preset(
        self,
        sensor_type: int,
        sensor_id: int,
        pos_x: float,
        pos_y: float,
        pos_z: float,
        sim_pitch_deg: float, # ✅ 이 함수는 (P, Y, R) 입력을 받는 것을 유지
        sim_yaw_deg: float,
        sim_roll_deg: float,
        *,
        ip: Optional[str] = None,
        port: Optional[int] = None,
    ) -> None:
        """Send a one-shot gimbal control packet for a specific sensor preset."""

        target_ip = ip or self.s.get("generator_ip", "127.0.0.1")
        target_port = int(port or self.s.get("generator_port", 15020))

        # ❌ 1. RPY 리맵핑 제거
        # internal_roll, internal_pitch, internal_yaw = self._sim_to_internal_rpy(
        #     sim_pitch_deg, sim_yaw_deg, sim_roll_deg
        # )
        
        # ✅ 1. (R, P, Y)로 변환
        roll_deg = sim_roll_deg
        pitch_deg = sim_pitch_deg
        yaw_deg = sim_yaw_deg
        
        # 2. 내부 (R,P,Y) -> 내부 (x,y,z,w) 쿼터니언
        base_quat = self._rpy_to_quat(roll_deg, pitch_deg, yaw_deg)
        
        # ✅ 3. 현재 쿼터니언(q_cur)과 비교하여 최단 경로 선택
        with self._lock:
            dot = self._q_dot(base_quat, self.q_cur)
        
        if dot < 1e-9:
            base_quat = [-x for x in base_quat]
            
        # 4. 시뮬레이터용 쿼터니언 리맵핑 적용 (요구사항)
        #remapped_udp_quat = self._remap_quat_for_simulator(base_quat)
        remapped_udp_quat = base_quat
        
        pkt = self._pack_gimbal_ctrl(
            int(sensor_type),
            int(sensor_id),
            [float(pos_x), float(pos_y), float(pos_z)],
            remapped_udp_quat, # 리맵핑된 쿼터니언 전달
        )
        try:
            self.tx_sock.sendto(pkt, (target_ip, target_port))
            self.log(
                f"[GIMBAL] preset UDP sent -> sensor={sensor_type}/{sensor_id}, "
                f"target={target_ip}:{target_port}"
            )
            if self.debug_dump_packets:
                self._dump_packet_bytes("PRESET", pkt)
        except Exception as exc:
            self.log(f"[GIMBAL] preset UDP send error: {exc}")

    def get_generator_endpoint(self) -> Tuple[str, int]:
        with self._lock:
            ip = str(self.s.get("generator_ip", "127.0.0.1"))
            port = int(self.s.get("generator_port", 15020))
        return ip, port

    def apply_external_pose(
        self,
        sensor_type: int,
        sensor_id: int,
        pos_x: float,
        pos_y: float,
        pos_z: float,
        sim_pitch_deg: float, # ✅ 이 함수는 (P, Y, R) 입력을 받는 것을 유지
        sim_yaw_deg: float,
        sim_roll_deg: float,
    ) -> None:
        """Handle an external Set_Gimbal request."""

        sensor_type_i = self._sanitize_sensor_code(sensor_type)
        sensor_id_i = self._sanitize_sensor_code(sensor_id)
        with self._lock:
            self.sensor_type = sensor_type_i
            self.sensor_id = sensor_id_i
            self.s["sensor_type"] = sensor_type_i
            self.s["sensor_id"] = sensor_id_i
            self._invalidate_cached_orientation_locked()

        self.set_target_pose(
            pos_x,
            pos_y,
            pos_z,
            sim_pitch_deg,
            sim_yaw_deg,
            sim_roll_deg,
            log=False,
        )
        self.send_udp_preset(
            sensor_type_i,
            sensor_id_i,
            pos_x,
            pos_y,
            pos_z,
            sim_pitch_deg,
            sim_yaw_deg,
            sim_roll_deg,
        )
        self.log(
            "[GIMBAL] external pose applied -> sensor=%d/%d xyz=(%.2f,%.2f,%.2f) "
            "sim_rpy(P,Y,R)=(%.2f,%.2f,%.2f)",
            (
                sensor_type_i,
                sensor_id_i,
                pos_x,
                pos_y,
                pos_z,
                sim_pitch_deg,
                sim_yaw_deg,
                sim_roll_deg,
            ),
        )

    def set_mavlink_target(self, preset_index: int, sensor_type: int, sensor_id: int) -> None:
        idx = max(0, int(preset_index))
        sensor_type_i = self._sanitize_sensor_code(sensor_type)
        sensor_id_i = self._sanitize_sensor_code(sensor_id)
        with self._lock:
            self.mavlink_preset_index = idx
            self.mavlink_sensor_type = sensor_type_i
            self.mavlink_sensor_id = sensor_id_i
            self.s["mavlink_preset_index"] = self.mavlink_preset_index
            self.s["mavlink_sensor_type"] = self.mavlink_sensor_type
            self.s["mavlink_sensor_id"] = self.mavlink_sensor_id
            self._invalidate_cached_orientation_locked()
        self.log(
            "[GIMBAL] MAVLink preset target -> preset=%d sensor=%d/%d",
            idx + 1,
            sensor_type_i,
            sensor_id_i,
        )

    def set_mav_ids(self, sys_id: int, comp_id: int) -> None:
        with self._lock:
            self.mav_sys_id  = int(sys_id)
            self.mav_comp_id = int(comp_id)
            self.s["mav_sysid"]  = self.mav_sys_id
            self.s["mav_compid"] = self.mav_comp_id
        self.log(f"[GIMBAL] MAV IDs updated: sys={self.mav_sys_id}, comp={self.mav_comp_id}")

    # -------- serial / MAVLink --------
    def open_serial(self, port: str, baud: int) -> None:
        cleaned_port = str(port or "").strip()
        self.serial_port = cleaned_port
        self.s["serial_port"] = self.serial_port
        self.serial_baud = int(baud)
        self.s["serial_baud"] = self.serial_baud

        if self.control_method == "mavlink":
            self._ensure_mavlink_running(force_restart=True)
        else:
            self.log("[GIMBAL] Serial configured but control method is TCP; switch to MAVLink to activate")

    def _open_serial(self) -> None:
        self._stop_mavlink_threads()
        if not self.serial_port:
            return
        try:
            mav = mavutil.mavlink_connection(
                self.serial_port,
                baud=self.serial_baud,
                source_system=self.mav_sys_id,
                source_component=self.mav_comp_id,
            )
            self.log(f"[GIMBAL] MAVLink serial: {self.serial_port} @ {self.serial_baud} (sys={self.mav_sys_id}, comp={self.mav_comp_id})")
            with self._mav_lock:
                self.mav = mav
            self._mav_stop.clear()
            self.mav_rx_thread = threading.Thread(target=self._mav_rx_loop, daemon=True)
            self.mav_tx_thread = threading.Thread(target=self._mav_tx_loop, daemon=True)
            self.mav_rx_thread.start()
            self.mav_tx_thread.start()
        except SerialException as e:
            self.log(f"[GIMBAL] serial error: {e}")
        except Exception as e:
            self.log(f"[GIMBAL] mavlink open error: {e}")

    # -------- status --------
    def _get_mavlink(self) -> Optional[mavutil.mavfile]:
        with self._mav_lock:
            return self.mav

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            method_display = self.control_method.upper() if self.control_method else "CTRL"
            
            # ✅ RPY 상태를 q_cur에서 즉시 계산 (R, P, Y)
            r_cur, p_cur, y_cur = self._q_to_rpy(self.q_cur)
            
            return {
                "activated": True,
                "control_mode": method_display,
                "control_method": self.control_method,
                "current_roll_deg": r_cur,   # R
                "current_pitch_deg": p_cur,  # P
                "current_yaw_deg": y_cur,    # Y
                "current_x": self.pos[0],
                "current_y": self.pos[1],
                "current_z": self.pos[2],
                "wx": self.w_cur[0], # 몸체 각속도
                "wy": self.w_cur[1], 
                "wz": self.w_cur[2], 
                "max_rate_dps": self.max_rate_dps,
                "serial_state": (self.serial_port or "-"),
                "hb_rx_ok": self.hb_rx_ok,
                "mav_sysid": self.mav_sys_id,
                "mav_compid": self.mav_comp_id,
                "zoom_scale": self.zoom_scale,
                "zoom_lens_mm": zoom_scale_to_lens_mm(self.zoom_scale, clamp=False),
                "tcp_bind": f"{self.rx_ip}:{self.rx_port}",
            }

    # -------- TCP control server --------
    def _start_tcp(self) -> None:
        if self.control_method != "tcp":
            return
        if self._tcp_thread and self._tcp_thread.is_alive() and self._tcp_sock:
            return
        self._tcp_stop.clear()
        try:
            self._open_tcp()
        except Exception as exc:
            self.log(f"[GIMBAL] TCP listen failed: {exc}")
            return
        self._tcp_thread = threading.Thread(target=self._tcp_accept_loop, daemon=True)
        self._tcp_thread.start()

    def _stop_tcp(self) -> None:
        self._tcp_stop.set()
        try:
            if self._tcp_sock:
                try:
                    wake = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    wake.settimeout(0.2)
                    wake.connect((("127.0.0.1" if self.rx_ip in ("0.0.0.0", "") else self.rx_ip), self.rx_port))
                    wake.close()
                except Exception:
                    pass
                self._tcp_sock.close()
        except Exception:
            pass
        self._tcp_sock = None
        with self._tcp_clients_lock:
            clients = list(self._tcp_clients)
            self._tcp_clients.clear()
        for conn in clients:
            try:
                conn.close()
            except Exception:
                pass
        self._tcp_thread = None

    def _open_tcp(self) -> None:
        self._tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSE_ADDR, 1)
        self._tcp_sock.bind((self.rx_ip, self.rx_port))
        self._tcp_sock.listen(5)
        self.log(f"[GIMBAL] TCP listening: {self.rx_ip}:{self.rx_port}")

    def _tcp_accept_loop(self) -> None:
        while not self._tcp_stop.is_set():
            try:
                conn, addr = self._tcp_sock.accept()
            except OSError:
                break
            except Exception as exc:
                self.log(f"[GIMBAL] TCP accept error: {exc}")
                time.sleep(0.1)
                continue
            if self._tcp_stop.is_set():
                try:
                    conn.close()
                except Exception:
                    pass
                break
            with self._tcp_clients_lock:
                self._tcp_clients.add(conn)
            self.log(f"[GIMBAL] TCP client connected: {addr}")
            t = threading.Thread(target=self._handle_tcp_client, args=(conn, addr), daemon=True)
            t.start()

    def _handle_tcp_client(self, conn: socket.socket, addr) -> None:
        try:
            while not self._tcp_stop.is_set():
                header = self._recv_all(conn, 4)
                if not header:
                    break
                (payload_len,) = struct.unpack("<I", header)
                payload = self._recv_all(conn, payload_len)
                if not payload:
                    break
                self._process_tcp_command(payload, conn)
        except Exception as exc:
            self.log(f"[GIMBAL] TCP client error from {addr}: {exc}")
        finally:
            try:
                conn.close()
            except Exception:
                pass
            with self._tcp_clients_lock:
                self._tcp_clients.discard(conn)
            self.log(f"[GIMBAL] TCP client disconnected: {addr}")

    @staticmethod
    def _recv_all(conn: socket.socket, n: int) -> Optional[bytes]:
        buf = bytearray()
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _process_tcp_command(self, payload: bytes, conn: socket.socket) -> None:
        command = parse_bridge_tcp_command(payload)
        if command is None:
            self.log("[GIMBAL] TCP payload too short")
            return

        if command.cmd_id == TCP_CMD_SET_TARGET:
            target = parse_set_target(command)
            if target is None:
                self.log("[GIMBAL] TCP SET_TARGET invalid payload")
                return
            sensor_type = self._sanitize_sensor_code(target.sensor_type)
            sensor_id = self._sanitize_sensor_code(target.sensor_id)
            px, py, pz = (
                float(target.position_xyz[0]),
                float(target.position_xyz[1]),
                float(target.position_xyz[2]),
            )
            # ✅ TCP 패킷은 (P, Y, R) 순서로 값을 제공
            sim_pitch, sim_yaw, sim_roll = (
                float(target.sim_rpy[0]),
                float(target.sim_rpy[1]),
                float(target.sim_rpy[2]),
            )
            with self._lock:
                self.sensor_type = sensor_type
                self.sensor_id = sensor_id
                self.s["sensor_type"] = self.sensor_type
                self.s["sensor_id"] = self.sensor_id
            
            # ✅ (P, Y, R) 입력을 set_target_pose로 전달
            self.set_target_pose(
                px,
                py,
                pz,
                sim_pitch,
                sim_yaw,
                sim_roll,
                persist=True,
                log=True, 
            )
            self._send_status_message(conn)
            
        elif command.cmd_id == TCP_CMD_SET_ZOOM:
            zoom = parse_set_zoom(command)
            if zoom is None:
                self.log("[GIMBAL] TCP SET_ZOOM invalid payload")
                return
            self._set_zoom_scale(zoom.zoom_scale)
            lens_mm = zoom_scale_to_lens_mm(self.zoom_scale, clamp=False)
            self.log(f"[GIMBAL] TCP zoom -> x{self.zoom_scale:.2f} ({lens_mm:.1f}mm)")
            self._send_status_message(conn)
        elif command.cmd_id == TCP_CMD_GET_STATUS:
            self._send_status_message(conn)
        else:
            self.log(f"[GIMBAL] TCP unknown cmd: 0x{command.cmd_id:02X}")

    def _send_status_message(self, conn: socket.socket) -> None:
        with self._lock:
            sensor_type = self.sensor_type
            sensor_id = self.sensor_id
            px, py, pz = self.pos
            
            # ✅ 현재 RPY는 q_cur에서 즉시 계산 (R, P, Y)
            r_cur, p_cur, y_cur = self._q_to_rpy(self.q_cur)
            
            # ✅ 목표 RPY는 저장된 rpy_tgt 사용 (R, P, Y)
            r_tgt, p_tgt, y_tgt = self.rpy_tgt 
            
            zoom = self.zoom_scale
            max_rate = self.max_rate_dps
        
        # ✅ TCP 상태 메시지는 (P, Y, R) 순서를 기대함
        sim_rpy_cur = (p_cur, y_cur, r_cur)
        sim_rpy_tgt = (p_tgt, y_tgt, r_tgt)
        
        snapshot = StatusSnapshot(
            sensor_type=sensor_type,
            sensor_id=sensor_id,
            position_xyz=(px, py, pz),
            sim_rpy_current=sim_rpy_cur, # (P, Y, R)
            sim_rpy_target=sim_rpy_tgt, # (P, Y, R)
            zoom_scale=zoom,
            max_rate_dps=max_rate,
        )
        ts_sec = int(time.time())
        ts_nsec = time.time_ns() % 1_000_000_000
        frame = build_status_frame(snapshot, ts_sec=ts_sec, ts_nsec=ts_nsec)
        try:
            conn.sendall(frame)
        except Exception as exc:
            self.log(f"[GIMBAL] TCP send status failed: {exc}")

    def _set_zoom_scale(self, value: float) -> None:
        zoom = max(1.0, float(value))
        with self._lock:
            if abs(zoom - self.zoom_scale) < 1e-3:
                return
            self.zoom_scale = zoom
            self.s["zoom_scale"] = self.zoom_scale
        if self._zoom_update_cb:
            try:
                self._zoom_update_cb(self.zoom_scale)
            except Exception as exc:
                self.log(f"[GIMBAL] zoom callback error: {exc}")

    def set_zoom_scale(self, value: float) -> None:
        self._set_zoom_scale(value)

    # -------- 쿼터니언 제어 헬퍼 시작 --------
    
    @staticmethod
    def _rpy_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> List[float]:
        """
        RPY 각도(도)를 (x, y, z, w) 쿼터니언으로 변환합니다.
        (ZYX 오일러 각 순서 적용)
        """
        try:
            r = math.radians(roll_deg)
            p = math.radians(pitch_deg)
            y = math.radians(yaw_deg)

            cr = math.cos(r * 0.5)
            sr = math.sin(r * 0.5)
            cp = math.cos(p * 0.5)
            sp = math.sin(p * 0.5)
            cy = math.cos(y * 0.5)
            sy = math.sin(y * 0.5)

            # ZYX 순서 (Yaw-Pitch-Roll)
            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy
            
            return [x, y, z, w]
        except Exception as e:
            print(f"[STATIC_HELPER] _rpy_to_quat error: {e}")
            return [0.0, 0.0, 0.0, 1.0]

    @staticmethod
    def _q_dot(q1: List[float], q2: List[float]) -> float:
        """두 쿼터니언의 내적 (dot product)"""
        return q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]

    @staticmethod
    def _q_normalize(q: List[float]) -> List[float]:
        """쿼터니언 정규화"""
        norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        if norm < 1e-9:
            return [0.0, 0.0, 0.0, 1.0]
        return [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]

    @staticmethod
    def _q_conjugate(q: List[float]) -> List[float]:
        """쿼터니언의 켤레(conjugate)"""
        return [-q[0], -q[1], -q[2], q[3]]

    @staticmethod
    def _q_multiply(q1: List[float], q2: List[float]) -> List[float]:
        """쿼터니언 곱셈 (q1 * q2)"""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return [x, y, z, w]
    


    @staticmethod
    def _remap_quat_for_simulator(
        quat_xyzw: List[float] | Tuple[float, float, float, float]
    ) -> Tuple[float, float, float, float]:
        """
        [2안] (X,Y,Z) -> (Y,Z,X) 매핑 (Intrinsic)
        (R,P,Y) -> (Y,R,P) 변환
        """
        
        # ✅ (X,Y,Z)->(Y,Z,X) 변환 쿼터니언 (q_transform)
        q_transform = [0.5, 0.5, 0.5, 0.5]
        
        q_bridge = quat_xyzw
        
        # ✅ q_sim = q_bridge * q_transform (Intrinsic)
        x1, y1, z1, w1 = q_bridge
        x2, y2, z2, w2 = q_transform
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return (x, y, z, w)

    def _q_slerp_step(
        self,
        q_start: List[float],
        q_end: List[float],
        max_rate_dps: float,
        dt: float,
    ) -> List[float]:
        """쿼터니언 SLERP(구면 선형 보간)을 사용하여 현재 상태를 목표로 이동"""
        
        q_start_norm = self._q_normalize(q_start)
        q_end_norm = self._q_normalize(q_end)

        dot = self._q_dot(q_start_norm, q_end_norm)
        if dot < 0.0:
            q_end_norm = [-x for x in q_end_norm]
            dot = -dot

        dot = max(min(dot, 1.0), -1.0)
        angle_error_rad = math.acos(dot) * 2.0

        max_step_rad = max_rate_dps * (math.pi / 180.0) * dt

        if angle_error_rad < 1e-6 or angle_error_rad < max_step_rad:
            return q_end_norm

        t = max_step_rad / angle_error_rad
        theta = math.acos(dot)
        sin_theta = math.sin(theta)
        
        if sin_theta < 1e-6:
            return q_start_norm

        w1 = math.sin((1.0 - t) * theta) / sin_theta
        w2 = math.sin(t * theta) / sin_theta

        q_res = [
            (w1 * q_start_norm[0] + w2 * q_end_norm[0]),
            (w1 * q_start_norm[1] + w2 * q_end_norm[1]),
            (w1 * q_start_norm[2] + w2 * q_end_norm[2]),
            (w1 * q_start_norm[3] + w2 * q_end_norm[3]),
        ]
        return self._q_normalize(q_res)

    # ✅ _q_to_internal_rpy -> _q_to_rpy로 변경
    def _q_to_rpy(self, q: List[float]) -> List[float]:
        """
        내부 쿼터니언(x,y,z,w)을 RPY(r,p,y)로 변환 (상태 보고용)
        (ZYX 오일러 각 기준)
        """
        qx, qy, qz, qw = q
        
        # 1. (x,y,z,w) -> MAVLink (w,x,y,z)
        w, x, y, z = self._remap_quat_for_mavlink(q)
        
        # 2. MAVLink (w,x,y,z) -> (P,Y,R)
        #    (_quat_to_frotator_deg는 ZYX 오일러 각을 P,Y,R 순서로 반환합니다)
        pitch, yaw, roll = _quat_to_frotator_deg(w, x, y, z)
        
        # ❌ 3. RPY 리맵핑 제거
        # internal_roll, internal_pitch, internal_yaw = self._sim_to_internal_rpy(
        #     sim_pitch, sim_yaw, sim_roll
        # )
        
        # ✅ 3. (R, P, Y) 순서로 반환
        return [roll, pitch, yaw]
    
    def _calculate_angular_velocity(
        self, q_start: List[float], q_end: List[float], dt: float
    ) -> List[float]:
        """
        두 쿼터니언 사이의 변화(q_delta)를 기반으로
        몸체 기준 각속도(wx, wy, wz)를 라디안/초 단위로 계산합니다.
        """
        if dt < 1e-6:
            return self.w_cur 
        
        q_start_conj = self._q_conjugate(q_start)
        q_delta = self._q_multiply(q_end, q_start_conj)
        
        dw = max(min(q_delta[3], 1.0), -1.0) 
        angle_rad = 2.0 * math.acos(dw)
        
        sin_half_angle = math.sin(angle_rad / 2.0)
        
        if abs(sin_half_angle) < 1e-6:
            return [0.0, 0.0, 0.0]
        
        dx, dy, dz = q_delta[0], q_delta[1], q_delta[2]
        scale = angle_rad / (sin_half_angle * dt)
        
        wx = dx * scale
        wy = dy * scale
        wz = dz * scale
        
        return [wx, wy, wz]


    @staticmethod
    def _remap_quat_for_mavlink(
        quat_xyzw: List[float] | Tuple[float, float, float, float]
    ) -> List[float]:
        """
        내부 (x, y, z, w) 쿼터니언을 MAVLink (w, x, y, z) 포맷으로 변환합니다.
        """
        x, y, z, w = quat_xyzw
        return [w, x, y, z]

    # -------- 쿼터니언 제어 헬퍼 끝 --------
    
    # ❌❌❌ RPY 리맵핑 함수 제거 ❌❌❌
    # @staticmethod
    # def _internal_to_sim_rpy(...)
    # @staticmethod
    # def _sim_to_internal_rpy(...)

    # -------- control loop & ICD send --------
    def _control_loop(self) -> None:
        period = 0.01  # 100 Hz
        while not self.stop_ev.is_set():
            t0 = time.time()
            pkt: Optional[bytes] = None
            target: Optional[Tuple[str, int]] = None
            dump_ctrl = False
            with self._lock:
                dt = max(1e-3, t0 - self._last_ts)
                self._last_ts = t0
                
                q_old = self.q_cur[:]

                self.q_cur = self._q_slerp_step(
                    self.q_cur, self.q_tgt, self.max_rate_dps, dt
                )
                
                self.w_cur = self._calculate_angular_velocity(q_old, self.q_cur, dt)

                sensor_type, sensor_id = self._active_sensor_codes_locked()
                
                snapshot = (
                    sensor_type,
                    sensor_id,
                    (float(self.pos[0]), float(self.pos[1]), float(self.pos[2])),
                    (float(self.q_cur[0]), float(self.q_cur[1]), float(self.q_cur[2]), float(self.q_cur[3])),
                )
                
                should_send = (
                    self._last_sent_snapshot is None
                    or self._has_pose_delta(self._last_sent_snapshot, snapshot)
                )
                
                if should_send:
                    q_to_send = self.q_cur
                    
                    #remapped_udp_quat = self._remap_quat_for_simulator(q_to_send)
                    remapped_udp_quat = q_to_send 

                    pkt = self._pack_gimbal_ctrl(
                        sensor_type, 
                        sensor_id, 
                        self.pos, 
                        remapped_udp_quat
                    )

                    target = (
                        str(self.s.get("generator_ip", "127.0.0.1")),
                        int(self.s.get("generator_port", 15020)),
                    )
                    dump_ctrl = self.debug_dump_packets
                    self._last_sent_snapshot = snapshot

            if pkt and target:
                try:
                    self.tx_sock.sendto(pkt, target)
                    if dump_ctrl:
                        self._dump_packet_bytes("CTRL", pkt)
                except Exception as e:
                    self.log(f"[GIMBAL] send 10706 error: {e}")

            time.sleep(max(0.0, period - (time.time() - t0)))

    # -------- MAVLink RX/TX --------
    def _mav_rx_loop(self) -> None:
        while not self.stop_ev.is_set() and not self._mav_stop.is_set():
            mav = self._get_mavlink()
            if not mav:
                time.sleep(0.1)
                continue
            try:
                m = mav.recv_match(blocking=True, timeout=0.2)
                if not m:
                    continue
                t = m.get_type()
                if t == "HEARTBEAT":
                    if int(getattr(m, "type", -1)) == MC_HB_TYPE:
                        self.hb_rx_ok = True
                        self.last_hb_rx = time.time()
                elif t == "GIMBAL_DEVICE_SET_ATTITUDE":
                    raw_q = getattr(m, "q", [float("nan")] * 4)
                    if not isinstance(raw_q, (list, tuple)):
                        raw_q = [float("nan")] * 4
                    q_list = list(raw_q)[:4]
                    while len(q_list) < 4:
                        q_list.append(float("nan"))
                    
                    if not any(math.isnan(v) for v in q_list):
                        
                        # 1. MAVLink (w,x,y,z) -> Sim (P,Y,R)
                        sim_pitch, sim_yaw, sim_roll = _quat_to_frotator_deg(
                            q_list[0], q_list[1], q_list[2], q_list[3]
                        )
                        
                        # ❌ 2. RPY 리맵핑 제거
                        # internal_roll, internal_pitch, internal_yaw = self._sim_to_internal_rpy(
                        #     sim_pitch, sim_yaw, sim_roll
                        # )
                        
                        # ✅ 2. (R, P, Y) 값을 직접 사용
                        roll_deg = sim_roll
                        pitch_deg = sim_pitch
                        yaw_deg = sim_yaw
                        
                        # 3. 내부 (R,P,Y) -> 새로운 쿼터니언 목표
                        try:
                            q_new_target = self._rpy_to_quat(roll_deg, pitch_deg, yaw_deg)
                        except Exception as e:
                            self.log(f"[Gimbal] euler_to_quat_apply (mav) failed: {e}")
                            q_new_target = self.q_tgt[:] 

                        with self._lock:
                            # ✅ 4. Anti-Flip
                            dot = self._q_dot(q_new_target, self.q_cur)
                            
                            if dot < 1e-9:
                                self.q_tgt = [-val for val in q_new_target]
                            else:
                                self.q_tgt = q_new_target[:]
                            
                            # 5. 섀도우 RPY 목표 설정 (R, P, Y)
                            self.rpy_tgt[:] = [roll_deg, pitch_deg, yaw_deg]
                            
                            self._invalidate_cached_orientation_locked()
                        self.log(
                            f"[GIMBAL] RX target RPY(R,P,Y)=({roll_deg:.1f},{pitch_deg:.1f},{yaw_deg:.1f})"
                        )
                elif t == "PARAM_REQUEST_LIST":
                    self._send_param_list()
                elif t == "PARAM_REQUEST_READ":
                    pid = getattr(m, "param_id", "").strip("\x00")
                    pidx = int(getattr(m, "param_index", -1))
                    self._send_param_read(pid, pidx)
            except Exception as e:
                self.log(f"[GIMBAL] MAV RX error: {e}")

    def _mav_tx_loop(self) -> None:
        next_hb = 0.0
        period_status = 0.1
        last_status = 0.0
        t0 = time.time()
        while not self.stop_ev.is_set() and not self._mav_stop.is_set():
            mav = self._get_mavlink()
            if not mav:
                time.sleep(0.1)
                continue
            now = time.time()
            try:
                if now >= next_hb:
                    mav.mav.heartbeat_send(26, 8, 0, 100, 4)
                    next_hb = now + 1.0

                if now - last_status >= period_status:
                    with self._lock:
                        q_current_internal = list(self.q_cur)
                        wx, wy, wz = self.w_cur # (몸체 각속도 rad/s)
                        gimbal_id = int(self.mavlink_sensor_id) & 0xFF
                    
                    q_mavlink = self._remap_quat_for_mavlink(q_current_internal)
                    
                    time_boot_ms = int((now - t0) * 1000.0)
                    # ✅ MAVLink는 (w,x,y,z) 쿼터니언과 (wx, wy, wz) 몸체 각속도를 전송
                    mav.mav.gimbal_device_attitude_status_send(
                        self.mav_sys_id,
                        self.mav_comp_id,
                        time_boot_ms,
                        GIMBAL_STATUS_FLAGS,
                        q_mavlink, # (w,x,y,z)
                        wx,        # (wx)
                        wy,        # (wy)
                        wz,        # (wz)
                        0,
                        float("nan"),
                        float("nan"),
                        gimbal_id,
                    )
                    last_status = now
            except Exception as e:
                self.log(f"[GIMBAL] MAV TX error: {e}")
            time.sleep(0.01)

    # -------- PARAM helpers --------
    def _send_param_list(self) -> None:
        mav = self._get_mavlink()
        if not mav:
            return
        for idx, (pid, val, ptype) in enumerate(PARAM_TABLE):
            try:
                mav.mav.param_value_send(pid.encode("ascii"), float(val), ptype, PARAM_COUNT, idx)
            except Exception as e:
                self.log(f"[GIMBAL] PARAM_VALUE idx={idx} error: {e}")
            time.sleep(0.02)

    def _send_param_read(self, pid: str, pidx: int) -> None:
        mav = self._get_mavlink()
        if not mav:
            return
        if 0 <= pidx < PARAM_COUNT:
            name, val, ptype = PARAM_TABLE[pidx]
        else:
            hit = next(((i, t) for i, t in enumerate(PARAM_TABLE) if t[0] == pid), None)
            if not hit: return
            pidx, (name, val, ptype) = hit
        try:
            mav.mav.param_value_send(name.encode("ascii"), float(val), ptype, PARAM_COUNT, pidx)
        except Exception as e:
            self.log(f"[GIMBAL] PARAM_VALUE(read) error: {e}")

    # -------- packers (ICD) --------
    def _has_pose_delta(
        self,
        prev: Tuple[int, int, Tuple[float, float, float], Tuple[float, float, float, float]],
        curr: Tuple[int, int, Tuple[float, float, float], Tuple[float, float, float, float]],
    ) -> bool:
        if prev[0] != curr[0] or prev[1] != curr[1]:
            return True
        pos_prev, pos_curr = prev[2], curr[2]
        q_prev, q_curr = prev[3], curr[3]
        
        for a, b in zip(pos_prev, pos_curr):
            if abs(a - b) > 1e-6:
                return True
        
        dot = abs(self._q_dot(list(q_prev), list(q_curr)))
        
        if dot > (1.0 - 1e-7):
             return False
        
        return True

    def _invalidate_cached_orientation_locked(self) -> None:
        """Reset cached pose/quaternion bookkeeping while ``self._lock`` is held."""
        self._last_sent_snapshot = None

    def _pack_gimbal_ctrl(
        self,
        sensor_type: int,
        sensor_id: int,
        xyz: List[float],
        remapped_quat: Tuple[float, float, float, float], # ✅ 리맵핑된 쿼터니언
    ) -> bytes:
        return build_gimbal_ctrl_packet(
            sensor_type,
            sensor_id,
            (float(xyz[0]), float(xyz[1]), float(xyz[2])),
            remapped_quat, # ✅ 리맵핑된 쿼터니언을 그대로 전달
        )

    def _pack_power_ctrl(self, sensor_type: int, sensor_id: int, power_on: int) -> bytes:
        return build_power_ctrl_packet(sensor_type, sensor_id, power_on)

    def _dump_packet_bytes(self, label: str, pkt: bytes) -> None:
        hex_str = " ".join(f"{b:02X}" for b in pkt)
        self.log(f"[GIMBAL] {label} packet ({len(pkt)} bytes): {hex_str}")

    # -------- utils --------
    def _emit_status(self, text: str) -> None:
        try:
            if self.status_cb: self.status_cb(text)
        except Exception: pass