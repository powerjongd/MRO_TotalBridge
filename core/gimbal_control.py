# core/gimbal_control.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import socket
import struct
import threading
import time
from typing import Callable, Optional, Dict, Any, List, Tuple, Set

from pymavlink import mavutil
try:
    from serial import SerialException
except ImportError:
    from serial.serialutil import SerialException

from utils.helpers import euler_to_quat

TYPE_GIMBAL_CTRL = 10706
TYPE_POWER_CTRL  = 10707

# 임무컴퓨터 Heartbeat 기대값
MC_HB_TYPE = 18

GIMBAL_STATUS_FLAGS = 2  # 요구사항

PARAM_TABLE: List[Tuple[str, float, int]] = [
    ("VERSION_X", 7, 9), ("SRL_NUMBER", 100, 9), ("STIFF_TILT", 50, 9),
    ("RC_CHAN_STILT", 2, 9), ("STIFF_ROLL", 50, 9), ("STIFF_PAN", 50, 9),
    ("FILTER_OUT", 1, 9), ("RC_CHAN_SPAN", 2, 9), ("PWR_TILT", 30, 9),
    ("PWR_ROLL", 30, 9), ("PWR_PAN", 30, 9), ("FLW_SP_TILT", 50, 9),
    ("GMB_HOME_PAN", 900, 9), ("FLW_SP_PAN", 80, 9), ("FLW_LPF_TILT", 50, 9),
    ("MAPPING_ANGLE", 0, 9), ("FLW_LPF_PAN", 50, 9), ("GYRO_TRUST", 100, 9),
    ("RC_DZONE_TILT", 40, 9), ("RC_DZONE_ROLL", 40, 9), ("RC_DZONE_PAN", 40, 9),
    ("RC_TYPE", 15, 9), ("GYRO_LPF", 5, 9), ("RC_LIM_MIN_TILT", -90, 9),
    ("RC_LIM_MAX_TILT", 90, 9), ("RC_LIM_MIN_ROLL", -20, 9),
    ("RC_LIM_MAX_ROLL", 20, 9), ("RC_LPF_TILT", 20, 9), ("RC_LPF_ROLL", 20, 9),
    ("RC_LPF_PAN", 20, 9), ("RC_CHAN_TILT", 1, 9), ("RC_CHAN_ROLL", 7, 9),
    ("RC_CHAN_PAN", 0, 9), ("RC_CHAN_MODE", 6, 9), ("RC_TRIM_TILT", 0, 9),
    ("RC_TRIM_ROLL", 0, 9), ("RC_TRIM_PAN", 0, 9), ("RC_MODE_TILT", 1, 9),
    ("RC_MODE_ROLL", 0, 9), ("RC_MODE_PAN", 1, 9), ("FLW_WD_TILT", 10, 9),
    ("FLW_WD_PAN", 10, 9), ("RC_SPD_TILT", 50, 9), ("RC_SPD_ROLL", 50, 9),
    ("RC_SPD_PAN", 50, 9), ("RC_REVERSE_AXIS", 0, 9), ("VERSION_Y", 8, 9),
    ("VERSION_Z", 3, 9), ("RC_LIM_MIN_PAN", -45, 9), ("RC_LIM_MAX_PAN", 45, 9),
    ("MAV_EMIT_HB", 1, 9), ("MAV_RATE_ST", 1, 9), ("MAV_RATE_ENCCNT", 10, 9),
    ("MAV_TS_ENCNT", 1, 9), ("MAV_RATE_ORIEN", 10, 9), ("MAV_RATE_IMU", 15, 9),
    ("BAUDRATE_COM2", 1152, 9), ("BAUDRATE_COM4", 0, 9), ("GIMBAL_COMPID", 154, 9),
    ("TILT_DAMPING", 20, 9), ("ROLL_DAMPING", 15, 9), ("PAN_DAMPING", 20, 9),
]
PARAM_COUNT = len(PARAM_TABLE)


def _quat_to_euler_deg(qx: float, qy: float, qz: float, qw: float):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) or 1.0
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    sinp = 2.0 * (w*y - z*x)
    pitch = math.degrees(math.copysign(math.pi/2, sinp)) if abs(sinp) >= 1 else math.degrees(math.asin(sinp))
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


TCP_CMD_SET_TARGET = 0x01
TCP_CMD_SET_ZOOM = 0x02
TCP_CMD_GET_STATUS = 0x80
TCP_CMD_STATUS = 0x81


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

        self._zoom_update_cb = zoom_update_cb

        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.settimeout(0.2)

        self._lock = threading.Lock()
        self.pos = [float(self.s.get("pos_x", 0.0)),
                    float(self.s.get("pos_y", 0.0)),
                    float(self.s.get("pos_z", 0.0))]
        self.rpy_cur = [float(self.s.get("init_roll_deg", 0.0)),
                        float(self.s.get("init_pitch_deg", 0.0)),
                        float(self.s.get("init_yaw_deg", 0.0))]
        self.rpy_tgt = self.rpy_cur[:]
        self.max_rate_dps = float(self.s.get("max_rate_dps", 60.0))
        self.power_on = bool(self.s.get("power_on", True))
        self.zoom_scale = max(1.0, float(self.s.get("zoom_scale", 1.0)))

        self.sensor_type = int(self.s.get("sensor_type", 0))
        self.sensor_id   = int(self.s.get("sensor_id", 0))

        # ✅ MAVLink IDs를 설정에서 읽어오도록
        self.mav_sys_id  = int(self.s.get("mav_sysid", 1))
        self.mav_comp_id = int(self.s.get("mav_compid", 154))

        self.rx_ip = self.s.get("bind_ip", "0.0.0.0")
        self.rx_port = int(self.s.get("bind_port", 16060))

        self.stop_ev = threading.Event()
        self.ctrl_thread = None
        self.mav_rx_thread = None
        self.mav_tx_thread = None
        self._tcp_thread: Optional[threading.Thread] = None
        self._tcp_sock: Optional[socket.socket] = None
        self._tcp_clients: Set[socket.socket] = set()
        self._tcp_stop = threading.Event()

        self.mav: Optional[mavutil.mavfile] = None
        self.serial_port = self.s.get("serial_port", "")
        self.serial_baud = int(self.s.get("serial_baud", 115200))
        self.hb_rx_ok = False
        self.last_hb_rx = 0.0

        self._last_rpy = self.rpy_cur[:]
        self._last_ts = time.time()
        self._rpy_rate = [0.0, 0.0, 0.0]

        if self._zoom_update_cb:
            try:
                self._zoom_update_cb(self.zoom_scale)
            except Exception as exc:
                self.log(f"[GIMBAL] zoom callback init error: {exc}")

    # -------- lifecycle --------
    def start(self) -> None:
        self.stop_ev.clear()
        self.ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.ctrl_thread.start()
        if self.serial_port:
            self._open_serial()
        self._start_tcp()
        self._emit_status("RUNNING")
        self.log("[GIMBAL] started")

    def stop(self) -> None:
        self.stop_ev.set()
        try: self.tx_sock.close()
        except Exception: pass
        self._stop_tcp()
        try:
            if self.mav: self.mav.close()
        except Exception: pass
        self._emit_status("STOPPED")
        self.log("[GIMBAL] stopped")

    def update_settings(self, settings: Dict[str, Any]) -> None:
        restart_tcp = False
        new_zoom: Optional[float] = None
        with self._lock:
            self.s.update(settings)
            self.sensor_type = int(self.s.get("sensor_type", self.sensor_type))
            self.sensor_id   = int(self.s.get("sensor_id", self.sensor_id))
            self.max_rate_dps = float(self.s.get("max_rate_dps", self.max_rate_dps))
            self.pos = [float(self.s.get("pos_x", self.pos[0])),
                        float(self.s.get("pos_y", self.pos[1])),
                        float(self.s.get("pos_z", self.pos[2]))]
            self.power_on = bool(self.s.get("power_on", self.power_on))
            if "zoom_scale" in self.s:
                new_zoom = max(1.0, float(self.s.get("zoom_scale", self.zoom_scale)))
            # ✅ 시스템/컴포넌트 ID 반영
            if "mav_sysid" in self.s:  self.mav_sys_id  = int(self.s["mav_sysid"])
            if "mav_compid" in self.s: self.mav_comp_id = int(self.s["mav_compid"])
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
        self.log("[GIMBAL] settings updated")
        if restart_tcp:
            self.log("[GIMBAL] restarting TCP listener due to bind change")
            self._stop_tcp()
            self._start_tcp()
        if new_zoom is not None:
            self._set_zoom_scale(new_zoom)

    # -------- public controls --------
    def set_target_pose(self, x, y, z, roll_deg, pitch_deg, yaw_deg) -> None:
        with self._lock:
            self.pos[:] = [x, y, z]
            self.rpy_tgt[:] = [roll_deg, pitch_deg, yaw_deg]
        self.log(f"[GIMBAL] target pose set → xyz=({x:.2f},{y:.2f},{z:.2f}), rpy=({roll_deg:.1f},{pitch_deg:.1f},{yaw_deg:.1f})")

    def set_max_rate(self, rate_dps: float) -> None:
        with self._lock:
            self.max_rate_dps = float(rate_dps)
        self.log(f"[GIMBAL] max rate = {rate_dps:.1f} dps")

    def set_power(self, on: bool) -> None:
        with self._lock:
            self.power_on = bool(on)
        self.log(f"[GIMBAL] power state updated (no send) -> {'ON' if on else 'OFF'}")

    def send_power(self, on: bool) -> None:
        pkt = self._pack_power_ctrl(self.sensor_type, self.sensor_id, int(bool(on)))
        try:
            self.tx_sock.sendto(pkt, (self.s.get("generator_ip", "127.0.0.1"),
                                      int(self.s.get("generator_port", 15020))))
            self.log(f"[GIMBAL] POWER {'ON' if on else 'OFF'} sent")
        except Exception as e:
            self.log(f"[GIMBAL] power send error: {e}")

    def send_udp_preset(
        self,
        sensor_type: int,
        sensor_id: int,
        pos_x: float,
        pos_y: float,
        pos_z: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        *,
        ip: Optional[str] = None,
        port: Optional[int] = None,
    ) -> None:
        """Send a one-shot gimbal control packet for a specific sensor preset."""

        target_ip = ip or self.s.get("generator_ip", "127.0.0.1")
        target_port = int(port or self.s.get("generator_port", 15020))
        pkt = self._pack_gimbal_ctrl(
            int(sensor_type),
            int(sensor_id),
            [float(pos_x), float(pos_y), float(pos_z)],
            [float(roll_deg), float(pitch_deg), float(yaw_deg)],
        )
        try:
            self.tx_sock.sendto(pkt, (target_ip, target_port))
            self.log(
                f"[GIMBAL] preset UDP sent -> sensor={sensor_type}/{sensor_id}, "
                f"target={target_ip}:{target_port}"
            )
        except Exception as exc:
            self.log(f"[GIMBAL] preset UDP send error: {exc}")

    def set_mav_ids(self, sys_id: int, comp_id: int) -> None:
        with self._lock:
            self.mav_sys_id  = int(sys_id)
            self.mav_comp_id = int(comp_id)
            self.s["mav_sysid"]  = self.mav_sys_id
            self.s["mav_compid"] = self.mav_comp_id
        self.log(f"[GIMBAL] MAV IDs updated: sys={self.mav_sys_id}, comp={self.mav_comp_id}")

    # -------- serial / MAVLink --------
    def open_serial(self, port: str, baud: int) -> None:
        self.serial_port = port
        self.serial_baud = int(baud)
        self._open_serial()

    def _open_serial(self) -> None:
        try:
            if self.mav:
                try: self.mav.close()
                except Exception: pass
            self.mav = mavutil.mavlink_connection(
                self.serial_port, baud=self.serial_baud,
                source_system=self.mav_sys_id, source_component=self.mav_comp_id
            )
            self.log(f"[GIMBAL] MAVLink serial: {self.serial_port} @ {self.serial_baud} (sys={self.mav_sys_id}, comp={self.mav_comp_id})")
            self.mav_rx_thread = threading.Thread(target=self._mav_rx_loop, daemon=True)
            self.mav_tx_thread = threading.Thread(target=self._mav_tx_loop, daemon=True)
            self.mav_rx_thread.start()
            self.mav_tx_thread.start()
        except SerialException as e:
            self.log(f"[GIMBAL] serial error: {e}")
        except Exception as e:
            self.log(f"[GIMBAL] mavlink open error: {e}")

    # -------- status --------
    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "activated": True,
                "control_mode": "CTRL",
                "current_roll_deg": self.rpy_cur[0],
                "current_pitch_deg": self.rpy_cur[1],
                "current_yaw_deg": self.rpy_cur[2],
                "current_x": self.pos[0],
                "current_y": self.pos[1],
                "current_z": self.pos[2],
                "wx": self._rpy_rate[0],
                "wy": self._rpy_rate[1],
                "wz": self._rpy_rate[2],
                "max_rate_dps": self.max_rate_dps,
                "serial_state": (self.serial_port or "-"),
                "hb_rx_ok": self.hb_rx_ok,
                "mav_sysid": self.mav_sys_id,
                "mav_compid": self.mav_comp_id,
                "zoom_scale": self.zoom_scale,
                "tcp_bind": f"{self.rx_ip}:{self.rx_port}",
            }

    # -------- TCP control server --------
    def _start_tcp(self) -> None:
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
        for conn in list(self._tcp_clients):
            try:
                conn.close()
            except Exception:
                pass
        self._tcp_clients.clear()

    def _open_tcp(self) -> None:
        self._tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
        if len(payload) < 9:
            self.log("[GIMBAL] TCP payload too short")
            return
        ts_sec, ts_nsec, cmd_id = struct.unpack("<IIB", payload[:9])
        body = payload[9:]
        if cmd_id == TCP_CMD_SET_TARGET:
            fmt = "<hh3d3f"
            if len(body) != struct.calcsize(fmt):
                self.log("[GIMBAL] TCP SET_TARGET malformed payload")
                return
            sensor_type, sensor_id, px, py, pz, r, p, y = struct.unpack(fmt, body)
            with self._lock:
                self.sensor_type = int(sensor_type)
                self.sensor_id = int(sensor_id)
                self.pos[:] = [px, py, pz]
                self.rpy_tgt[:] = [r, p, y]
                self.s["sensor_type"] = self.sensor_type
                self.s["sensor_id"] = self.sensor_id
                self.s["pos_x"], self.s["pos_y"], self.s["pos_z"] = self.pos
                self.s["init_roll_deg"], self.s["init_pitch_deg"], self.s["init_yaw_deg"] = self.rpy_tgt
            self.log(
                f"[GIMBAL] TCP target -> sensor={sensor_type}/{sensor_id} "
                f"xyz=({px:.2f},{py:.2f},{pz:.2f}) rpy=({r:.2f},{p:.2f},{y:.2f})"
            )
            self._send_status_message(conn)
        elif cmd_id == TCP_CMD_SET_ZOOM:
            if len(body) < 4:
                self.log("[GIMBAL] TCP SET_ZOOM missing float")
                return
            (zoom_val,) = struct.unpack("<f", body[:4])
            self._set_zoom_scale(float(zoom_val))
            self.log(f"[GIMBAL] TCP zoom scale -> {self.zoom_scale:.2f}")
            self._send_status_message(conn)
        elif cmd_id == TCP_CMD_GET_STATUS:
            self._send_status_message(conn)
        else:
            self.log(f"[GIMBAL] TCP unknown cmd: 0x{cmd_id:02X}")

    def _send_status_message(self, conn: socket.socket) -> None:
        with self._lock:
            sensor_type = self.sensor_type
            sensor_id = self.sensor_id
            px, py, pz = self.pos
            r_cur, p_cur, y_cur = self.rpy_cur
            r_tgt, p_tgt, y_tgt = self.rpy_tgt
            zoom = self.zoom_scale
            max_rate = self.max_rate_dps
        payload = struct.pack(
            "<hh3d3f3ff",
            sensor_type,
            sensor_id,
            px,
            py,
            pz,
            r_cur,
            p_cur,
            y_cur,
            r_tgt,
            p_tgt,
            y_tgt,
            zoom,
            max_rate,
        )
        ts_sec = int(time.time())
        ts_nsec = time.time_ns() % 1_000_000_000
        full = struct.pack("<IIB", ts_sec, ts_nsec, TCP_CMD_STATUS) + payload
        header = struct.pack("<I", len(full))
        try:
            conn.sendall(header + full)
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

    # -------- control loop & ICD send --------
    def _control_loop(self) -> None:
        period = 0.01  # 100 Hz
        while not self.stop_ev.is_set():
            t0 = time.time()
            with self._lock:
                dt = max(1e-3, t0 - self._last_ts)
                self._last_ts = t0

                for i in range(3):
                    err = self.rpy_tgt[i] - self.rpy_cur[i]
                    step = math.copysign(min(abs(err), self.max_rate_dps * dt), err)
                    self.rpy_cur[i] += step
                    self._rpy_rate[i] = (self.rpy_cur[i] - self._last_rpy[i]) / dt
                    self._last_rpy[i] = self.rpy_cur[i]

                pkt = self._pack_gimbal_ctrl(self.sensor_type, self.sensor_id, self.pos, self.rpy_cur)
                try:
                    self.tx_sock.sendto(pkt, (self.s.get("generator_ip", "127.0.0.1"),
                                              int(self.s.get("generator_port", 15020))))
                except Exception as e:
                    self.log(f"[GIMBAL] send 10706 error: {e}")

            time.sleep(max(0.0, period - (time.time() - t0)))

    # -------- MAVLink RX/TX --------
    def _mav_rx_loop(self) -> None:
        while not self.stop_ev.is_set() and self.mav:
            try:
                m = self.mav.recv_match(blocking=True, timeout=0.2)
                if not m:
                    continue
                t = m.get_type()
                if t == "HEARTBEAT":
                    if int(getattr(m, "type", -1)) == MC_HB_TYPE:
                        self.hb_rx_ok = True
                        self.last_hb_rx = time.time()
                elif t == "GIMBAL_DEVICE_SET_ATTITUDE":
                    q = getattr(m, "q", [float("nan")]*4)
                    avx = getattr(m, "angular_velocity_x", float("nan"))
                    # mode b: q 유효 → 목표 각도 설정
                    if not any(math.isnan(v) for v in q):
                        r, p, y = _quat_to_euler_deg(q[0], q[1], q[2], q[3])
                        with self._lock:
                            self.rpy_tgt[:] = [r, p, y]
                        self.log(f"[GIMBAL] RX target RPY=({r:.1f},{p:.1f},{y:.1f})")
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
        while not self.stop_ev.is_set() and self.mav:
            now = time.time()
            try:
                if now >= next_hb:
                    # type=26, autopilot=8, base_mode=0, custom_mode=100, system_status=4
                    self.mav.mav.heartbeat_send(26, 8, 0, 100, 4)
                    next_hb = now + 1.0

                if now - last_status >= period_status:
                    with self._lock:
                        r, p, y = self.rpy_cur
                        qx, qy, qz, qw = euler_to_quat(r, p, y)
                        wx, wy, wz = self._rpy_rate
                    time_boot_ms = int((now - t0) * 1000.0)
                    self.mav.mav.gimbal_device_attitude_status_send(
                        self.mav_sys_id, self.mav_comp_id,
                        time_boot_ms, GIMBAL_STATUS_FLAGS,
                        [qx, qy, qz, qw], wx, wy, wz, 0
                    )
                    last_status = now
            except Exception as e:
                self.log(f"[GIMBAL] MAV TX error: {e}")
            time.sleep(0.01)

    # -------- PARAM helpers --------
    def _send_param_list(self) -> None:
        if not self.mav:
            return
        for idx, (pid, val, ptype) in enumerate(PARAM_TABLE):
            try:
                self.mav.mav.param_value_send(pid.encode("ascii"), float(val), ptype, PARAM_COUNT, idx)
            except Exception as e:
                self.log(f"[GIMBAL] PARAM_VALUE idx={idx} error: {e}")
            time.sleep(0.02)

    def _send_param_read(self, pid: str, pidx: int) -> None:
        if not self.mav:
            return
        if 0 <= pidx < PARAM_COUNT:
            name, val, ptype = PARAM_TABLE[pidx]
        else:
            hit = next(((i, t) for i, t in enumerate(PARAM_TABLE) if t[0] == pid), None)
            if not hit: return
            pidx, (name, val, ptype) = hit
        try:
            self.mav.mav.param_value_send(name.encode("ascii"), float(val), ptype, PARAM_COUNT, pidx)
        except Exception as e:
            self.log(f"[GIMBAL] PARAM_VALUE(read) error: {e}")

    # -------- packers (ICD) --------
    def _pack_gimbal_ctrl(self, sensor_type: int, sensor_id: int, xyz: List[float], rpy_deg: List[float]) -> bytes:
        header = struct.pack("<BiIB", 1, TYPE_GIMBAL_CTRL, 42, 0)
        qx, qy, qz, qw = euler_to_quat(rpy_deg[0], rpy_deg[1], rpy_deg[2])
        payload = struct.pack("<BBdddffff",
                              sensor_type & 0xFF, sensor_id & 0xFF,
                              float(xyz[0]), float(xyz[1]), float(xyz[2]),
                              float(qx), float(qy), float(qz), float(qw))
        return header + payload

    def _pack_power_ctrl(self, sensor_type: int, sensor_id: int, power_on: int) -> bytes:
        header = struct.pack("<BiIB", 1, TYPE_POWER_CTRL, 3, 0)
        payload = struct.pack("<BBB", sensor_type & 0xFF, sensor_id & 0xFF, power_on & 0xFF)
        return header + payload

    # -------- utils --------
    def _emit_status(self, text: str) -> None:
        try:
            if self.status_cb: self.status_cb(text)
        except Exception: pass
