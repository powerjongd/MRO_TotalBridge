# core/bridge_core.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import io
import os
import socket
import struct
import threading
import time
import datetime
from typing import Callable, Optional, Dict, Any

try:
    from PIL import Image  # 미리보기 썸네일 생성에 사용될 수 있음(옵션)
    _HAS_PIL = True
except Exception:
    _HAS_PIL = False


# ---- Command IDs (MroCameraControl) ----
CMD_REQ_CAPTURE      = 0x01
CMD_SET_GIMBAL       = 0x02  # 예약
CMD_SET_COUNT        = 0x03
CMD_GET_IMG_NUM      = 0x04
CMD_REQ_SEND_IMG     = 0x05
CMD_IMG_NUM_RESPONSE = 0x11
CMD_FILE_IMG_TRANSFER= 0x12


class ImageBridgeCore:
    """
    - TCP 서버: MroCameraControl 명령 송수신
    - UDP 수신: New ICD(30B 헤더) 기반 JPEG 조립
    - 콜백:
        log_cb(text), preview_cb(jpeg_bytes), status_cb(text)
    """

    def __init__(
        self,
        log_cb: Callable[[str], None],
        preview_cb: Optional[Callable[[bytes], None]],
        status_cb: Optional[Callable[[str], None]],
        settings: Dict[str, Any],
    ) -> None:
        self.log = log_cb
        self.preview_cb = preview_cb
        self.status_cb = status_cb

        # settings
        self.ip = settings.get("ip", "0.0.0.0")
        self.tcp_port = int(settings.get("tcp_port", 9999))
        self.udp_port = int(settings.get("udp_port", 9998))
        self.images_dir = str(settings.get("images", "./images"))

        # runtime
        self.is_server_running = threading.Event()
        self._tcp_sock: Optional[socket.socket] = None
        self._udp_sock: Optional[socket.socket] = None
        self._client_conn: Optional[socket.socket] = None

        self._tcp_thread: Optional[threading.Thread] = None
        self._udp_thread: Optional[threading.Thread] = None

        self._lock = threading.Lock()
        self._latest_jpeg: Optional[bytes] = None
        self._next_image_number: int = 0  # 다음 저장 번호 (000..999 롤링)

        # UDP reassembly buffer
        self._reasm: Optional[Dict[str, Any]] = None
        self._last_image_meta = {
            "kb": 0.0,
            "received_at": None,   # datetime or None
            "saved_path": None,    # 마지막으로 디스크에 쓴 파일 경로
        }

        self._prepare_dirs()

    # --------------- lifecycle ---------------

    def start(self) -> None:
        if self.is_server_running.is_set():
            return
        self.is_server_running.set()
        self._open_tcp()
        self._open_udp()
        self._tcp_thread = threading.Thread(target=self._tcp_server_thread, daemon=True)
        self._udp_thread = threading.Thread(target=self._udp_receiver_thread, daemon=True)
        self._tcp_thread.start()
        self._udp_thread.start()
        self._emit_status("RUNNING")
        self.log(f"[BRIDGE] started at {self.ip} TCP:{self.tcp_port} UDP:{self.udp_port}")

    def stop(self) -> None:
        if not self.is_server_running.is_set():
            return
        self.is_server_running.clear()

        # close client first
        if self._client_conn:
            try:
                self._client_conn.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self._client_conn.close()
            except Exception:
                pass
            self._client_conn = None

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
        self._emit_status("STOPPED")
        self.log("[BRIDGE] stopped")

    def update_settings(self, settings: Dict[str, Any]) -> None:
        self.ip = settings.get("ip", self.ip)
        self.tcp_port = int(settings.get("tcp_port", self.tcp_port))
        self.udp_port = int(settings.get("udp_port", self.udp_port))
        self.images_dir = str(settings.get("images", self.images_dir))
        self._prepare_dirs()
        self.log("[BRIDGE] settings updated")

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
            while self.is_server_running.is_set():
                header = self._recv_all(conn, 4)
                if not header:
                    break
                payload_len = struct.unpack("<I", header)[0]
                payload = self._recv_all(conn, payload_len)
                if not payload or len(payload) != payload_len:
                    self.log("[BRIDGE] TCP payload length mismatch")
                    break
                self._process_command(payload, conn)
        except Exception as e:
            self.log(f"[BRIDGE] client error: {e}")
        finally:
            try:
                conn.close()
            except Exception:
                pass
            self._client_conn = None
            self.log("[BRIDGE] TCP client disconnected")
            
    def _process_command(self, payload: bytes, conn: socket.socket) -> None:
        # [ts_sec(4)][ts_nsec(4)][cmd_id(1)] + payload...
        if len(payload) < 9:
            self.log("[BRIDGE] malformed cmd payload")
            return
        ts_sec, ts_nsec, cmd_id = struct.unpack("<IIB", payload[:9])
        cmd_payload = payload[9:]

        def _expect_uint8_one(p: bytes, cmd_name: str) -> bool:
            """uint8==1 규격을 검증. 없거나 값이 1이 아니면 경고 로그."""
            if len(p) < 1:
                self.log(f"[BRIDGE] {cmd_name}: missing uint8 payload; treating as 1 for backward-compat")
                return True  # 하위호환을 위해 허용
            (flag,) = struct.unpack("<B", p[:1])
            if flag != 1:
                self.log(f"[BRIDGE] {cmd_name}: expected uint8==1, got {flag}; ignoring but continuing")
            return True

        if cmd_id == CMD_REQ_CAPTURE:
            # 기대 페이로드: uint8==1 (1바이트)
            _expect_uint8_one(cmd_payload, "Req_Capture")
            self._handle_req_capture()

        elif cmd_id == CMD_SET_COUNT:
            if len(cmd_payload) >= 4:
                (count_num,) = struct.unpack("<I", cmd_payload[:4])
                self._handle_set_count(count_num)
            else:
                self.log("[BRIDGE] Set_Count: missing uint32 count_num")

        elif cmd_id == CMD_GET_IMG_NUM:
            # 기대 페이로드: uint8==1 (1바이트)
            _expect_uint8_one(cmd_payload, "Get_ImgNum")
            self._handle_get_img_num(conn)

        elif cmd_id == CMD_REQ_SEND_IMG:
            if len(cmd_payload) >= 4:
                (img_num,) = struct.unpack("<I", cmd_payload[:4])
                self._handle_req_send_img(conn, img_num)
            else:
                self.log("[BRIDGE] Req_SendImg: missing uint32 img_num")

        elif cmd_id == CMD_SET_GIMBAL:
            self.log("[BRIDGE] Set_Gimbal received (reserved)")

        else:
            self.log(f"[BRIDGE] unknown cmd_id: 0x{cmd_id:02X}")

    def _handle_req_capture(self) -> None:
        with self._lock:
            if not self._latest_jpeg:
                self.log("[BRIDGE] no image to capture (UDP not received yet)")
                return
            fn = os.path.join(self.images_dir, f"{self._next_image_number:03d}.jpg")
            try:
                with open(fn, "wb") as f:
                    f.write(self._latest_jpeg)
                # 로그/상태 갱신
                self._last_image_meta["saved_path"] = fn
                self.log(f"[BRIDGE] saved image: {fn} | next_num(before)={self._next_image_number:03d}")
                self._next_image_number = (self._next_image_number + 1) % 1000
                self.log(f"[BRIDGE] next image number -> {self._next_image_number:03d}")
            except Exception as e:
                self.log(f"[BRIDGE] file write error: {e}")

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
        full = struct.pack("<IIB", ts_sec, ts_nsec, CMD_IMG_NUM_RESPONSE) + data
        header = struct.pack("<I", len(full))
        try:
            conn.sendall(header + full)
            self.log(f"[BRIDGE] Img_Num_Response: {last_saved:03d}")
        except Exception as e:
            self.log(f"[BRIDGE] send Get_ImgNum failed: {e}")

    def _handle_req_send_img(self, conn: socket.socket, img_num: int) -> None:
        fp = os.path.join(self.images_dir, f"{img_num:03d}.jpg")
        ack_uuid = 0
        try:
            with open(fp, "rb") as f:
                img = f.read()
        except FileNotFoundError:
            img = b""
        data = struct.pack("<III", ack_uuid, img_num, len(img)) + img
        ts_sec, ts_nsec = int(time.time()), time.time_ns() % 1_000_000_000
        full = struct.pack("<IIB", ts_sec, ts_nsec, CMD_FILE_IMG_TRANSFER) + data
        header = struct.pack("<I", len(full))
        try:
            conn.sendall(header + full)
            self.log(f"[BRIDGE] File_ImgTransfer sent: num={img_num:03d}, size={len(img)}")
        except Exception as e:
            self.log(f"[BRIDGE] send File_ImgTransfer failed: {e}")

    # --------------- UDP Receiver (New ICD) ---------------

    def _open_udp(self) -> None:
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
        self._udp_sock.bind((self.ip, self.udp_port))
        self._udp_sock.settimeout(0.3)
        self.log(f"[BRIDGE] UDP listening: {self.ip}:{self.udp_port}")

    def _udp_receiver_thread(self) -> None:
        while self.is_server_running.is_set():
            try:
                packet, _ = self._udp_sock.recvfrom(65535)
                header = self._parse_udp_header(packet)
                if not header:
                    continue
                if header.get("msg_type") != 9:
                    continue

                fi = header["frame_index"]
                frames = header["msg_frames"]
                fsize = header["frame_size"]
                fpos = header["frame_pos"]
                msg_size = header["msg_size"]

                # 새 이미지 시작
                if fi == 1 and (self._reasm is None or self._reasm.get("frame_index", 0) > 1):
                    self._reasm = {
                        "buf": bytearray(msg_size),
                        "recv": set(),
                        "total": frames,
                        "frame_index": fi,
                        "maxpos": 0,
                        "start": time.time(),
                    }

                if self._reasm is None:
                    continue

                # 데이터 복사
                frame = packet[30:30 + fsize]
                endpos = fpos + len(frame)
                if endpos > len(self._reasm["buf"]):
                    # 방어: 헤더가 잘못된 경우
                    continue
                self._reasm["buf"][fpos:endpos] = frame
                self._reasm["recv"].add(fi)
                self._reasm["frame_index"] = fi
                if endpos > self._reasm["maxpos"]:
                    self._reasm["maxpos"] = endpos

                # 완성 검사
                if len(self._reasm["recv"]) == self._reasm["total"]:
                    data = bytes(self._reasm["buf"][: self._reasm["maxpos"]])
                    if len(data) >= 4 and data.startswith(b"\xff\xd8") and data.endswith(b"\xff\xd9"):
                        with self._lock:
                            self._latest_jpeg = data
                            self._last_image_meta["kb"] = len(data) / 1024.0
                            self._last_image_meta["received_at"] = datetime.datetime.now()
                        if self.preview_cb:
                            try: self.preview_cb(data)
                            except Exception: pass
                        elapsed = time.time() - self._reasm["start"]
                        self.log(f"[BRIDGE] image received: {len(data)/1024:.1f} KB in {elapsed:.2f}s | next={self._next_image_number:03d}")
                    else:
                        self.log("[BRIDGE] invalid JPEG received")
                    self._reasm = None

            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                self.log(f"[BRIDGE] UDP error: {e}")
                time.sleep(0.05)
    
    def get_runtime_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
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
            }

    @staticmethod
    def _parse_udp_header(data: bytes) -> Optional[Dict[str, int]]:
        """
        30 bytes header: <IBBHIIIIIBB>
        keys: header_version, msg_type, protocol_type, send_count,
              msg_frames, frame_size, frame_pos, frame_index, msg_size,
              time_sec(1B), time_nano_sec(1B)
        """
        if len(data) < 30:
            return None
        try:
            header_format = "<IBBHIIIIIBB"
            keys = [
                "header_version", "msg_type", "protocol_type", "send_count",
                "msg_frames", "frame_size", "frame_pos", "frame_index",
                "msg_size", "time_sec", "time_nano_sec",
            ]
            values = struct.unpack(header_format, data[:30])
            return dict(zip(keys, values))
        except struct.error:
            return None

    # --------------- utils ---------------

    def _prepare_dirs(self) -> None:
        for label, path in ("SaveFile", self.realtime_dir), ("PreDefinedImageSet", self.predefined_dir):
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
        try:
            if self._tcp_sock:
                self._tcp_sock.close()
        except Exception:
            pass
        try:
            if self._udp_sock:
                self._udp_sock.close()
        except Exception:
            pass
        self._tcp_sock = None
        self._udp_sock = None

    @staticmethod
    def _recv_all(conn: socket.socket, n: int) -> Optional[bytes]:
        """exactly n bytes or None on EOF"""
        buf = bytearray()
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _emit_status(self, text: str) -> None:
        try:
            if self.status_cb:
                self.status_cb(text)
        except Exception:
            pass

