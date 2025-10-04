# utils/settings.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import os
import sys
import tempfile
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


# ------------------------------------------------------------
# 경로 유틸
# ------------------------------------------------------------

def _is_writable_dir(path: str) -> bool:
    try:
        os.makedirs(path, exist_ok=True)
        test = os.path.join(path, ".write_test.tmp")
        with open(test, "w", encoding="utf-8") as f:
            f.write("ok")
        os.remove(test)
        return True
    except Exception:
        return False

def get_program_dir() -> str:
    """
    실행 기준 폴더를 반환.
    - one-file(PyInstaller)에서도 '원래 exe가 있는 폴더'를 우선 사용
      (sys.argv[0] 또는 sys.orig_argv[0] 기반)
    - 쓰기 불가하면 사용자 홈 아래 AppData 로 폴백
    """
    # 1) 원래 exe 경로 우선
    cand = None
    try:
        if hasattr(sys, "orig_argv") and sys.orig_argv:
            cand = os.path.dirname(os.path.abspath(sys.orig_argv[0]))
        elif sys.argv and sys.argv[0]:
            cand = os.path.dirname(os.path.abspath(sys.argv[0]))
    except Exception:
        cand = None

    # 2) 일반 venv/스크립트 실행일 때의 후보
    if not cand:
        if getattr(sys, "frozen", False) and hasattr(sys, "executable"):
            cand = os.path.dirname(os.path.abspath(sys.executable))
        else:
            # utils/settings.py → utils → 프로젝트 루트
            cand = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # 3) 쓰기 가능 여부 확인, 불가하면 사용자 폴더로 폴백
    if not _is_writable_dir(cand):
        # Windows: %LOCALAPPDATA%\MroUnifiedBridge
        # 기타 OS: ~/.local/share/MroUnifiedBridge
        from pathlib import Path
        if os.name == "nt":
            base = os.environ.get("LOCALAPPDATA", str(Path.home()))
            fallback = os.path.join(base, "MroUnifiedBridge")
        else:
            fallback = os.path.join(str(Path.home()), ".local", "share", "MroUnifiedBridge")

        os.makedirs(fallback, exist_ok=True)
        return fallback

    return cand


def get_default_save_dir() -> str:
    """
    저장 폴더는 실행파일/메인 스크립트 바로 아래의 'savedata' 폴더로 고정.
    예) <exe 또는 main.py 있는 곳>/savedata
    """
    return os.path.join(get_program_dir(), "savedata")


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _atomic_write(path: str, data: str, encoding: str = "utf-8") -> None:
    """
    데이터 손상을 방지하기 위한 원자적 파일 저장.
    """
    d = os.path.dirname(path)
    ensure_dir(d)
    fd, tmp = tempfile.mkstemp(prefix=".tmp_", dir=d)
    try:
        with os.fdopen(fd, "w", encoding=encoding) as f:
            f.write(data)
        # Windows에서도 덮어쓰기 안전하게
        if os.path.exists(path):
            os.replace(tmp, path)
        else:
            os.rename(tmp, path)
    finally:
        try:
            if os.path.exists(tmp):
                os.remove(tmp)
        except Exception:
            pass


def load_json(path: str, default: Any) -> Any:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return default
    except Exception:
        # 깨진 파일 대비 백업 후 기본값 리턴
        try:
            ts = time.strftime("%Y%m%d-%H%M%S")
            os.replace(path, f"{path}.corrupt.{ts}.bak")
        except Exception:
            pass
        return default


def save_json(path: str, obj: Any) -> None:
    _atomic_write(path, json.dumps(obj, ensure_ascii=False, indent=2))


# ------------------------------------------------------------
# 설정 데이터 클래스
# ------------------------------------------------------------

@dataclass
class AppConfig:
    """
    전체 앱 설정의 루트. 내부는 자유롭게 확장 가능.
    - bridge / gimbal / relay 섹션을 표준화해 기본값 주입.
    """
    bridge: Dict[str, Any] = field(default_factory=dict)
    gimbal: Dict[str, Any] = field(default_factory=dict)
    relay:  Dict[str, Any] = field(default_factory=dict)
    # 기타 루트 레벨 키도 허용
    _extras: Dict[str, Any] = field(default_factory=dict, repr=False)

    # ---------- 직렬화 도우미 ----------

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "bridge": dict(self.bridge),
            "gimbal": dict(self.gimbal),
            "relay":  dict(self.relay),
        }
        # extras 병합 (충돌 없을 때만)
        for k, v in self._extras.items():
            if k not in d:
                d[k] = v
        return d

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "AppConfig":
        # 알려진 섹션 추출
        bridge = dict(d.get("bridge", {}))
        gimbal = dict(d.get("gimbal", {}))
        relay  = dict(d.get("relay", {}))
        # 나머지 extras
        extras = {k: v for k, v in d.items() if k not in ("bridge", "gimbal", "relay")}
        ac = AppConfig(bridge=bridge, gimbal=gimbal, relay=relay, _extras=extras)
        # 기본값 주입/마이그레이션
        ac._inject_defaults()
        return ac

    # ---------- 섹션별 기본값 주입 ----------

    def _inject_defaults(self) -> None:
        # Bridge 기본값
        b = self.bridge
        b.setdefault("ip", "0.0.0.0")
        b.setdefault("tcp_port", 9999)
        b.setdefault("udp_port", 9998)
        default_realtime = os.path.join(get_program_dir(), "SaveFile")
        default_predefined = os.path.join(get_program_dir(), "PreDefinedImageSet")

        if "realtime_dir" not in b:
            if b.get("images"):
                b["realtime_dir"] = b["images"]
            else:
                b["realtime_dir"] = default_realtime
        if "predefined_dir" not in b:
            b["predefined_dir"] = default_predefined

        ensure_dir(b["realtime_dir"])
        ensure_dir(b["predefined_dir"])

        b.setdefault("image_source_mode", "realtime")
        b["images"] = b["realtime_dir"]
        # GUI 미리보기 등
        b.setdefault("console_echo", True)         # 콘솔 로그 echo
        b.setdefault("show_hud", True)             # (옵션) 간단 상태 표시
        # req_capture / get_imgnum 페이로드 정책
        b.setdefault("use_1byte_payload_for_rcv", True)
        b.setdefault("zoom_scale", 1.0)

        # Gimbal 기본값
        g = self.gimbal
        g.setdefault("enabled", True)
        g.setdefault("sensor_type", 0)             # 0: Camera
        g.setdefault("sensor_id", 0)
        g.setdefault("max_rate_deg_s", 60.0)       # 최대 각속도
        # 제너레이터(시뮬레이터) UDP 목적지
        g.setdefault("sim_ip", "127.0.0.1")
        g.setdefault("sim_port", 10706)            # gimbal ctrl 포트 가정 시 명시 사용
        g.setdefault("sim_power_port", 10707)      # power ctrl 포트 가정 시 명시 사용
        g.setdefault("bind_ip", "0.0.0.0")
        g.setdefault("bind_port", 16060)
        # system/component id (MAVLink 상호작용용 UI에서 사용)
        g.setdefault("sys_id", 1)
        g.setdefault("comp_id", 154)
        # 초기 포즈 (xyzrpy)
        g.setdefault("pose_xyzrpy", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        g.setdefault("power_on", True)
        g.setdefault("zoom_scale", 1.0)
        # Serial (임무컴퓨터용 MAVLink)
        g.setdefault("serial_port", "")
        g.setdefault("serial_baud", 115200)
        g.setdefault("generator_ip", "127.0.0.1")
        g.setdefault("generator_port", 15020)
        g.setdefault("presets", {"version": 2, "slots": [None] * 6})
        g.setdefault(
            "preset_bundle",
            {
                "network": {
                    "ip": g.get("generator_ip", "127.0.0.1"),
                    "port": g.get("generator_port", 15020),
                },
                "presets": [None] * 6,
                "selected": 0,
            },
        )

        g.setdefault("selected_preset", 0)

        # Relay 기본값
        r = self.relay
        # Gazebo 입력
        r.setdefault("gazebo_listen_ip", "0.0.0.0")
        r.setdefault("gazebo_listen_port", 17000)
        # ExternalCtrl 출력 (원본 릴레이)
        r.setdefault("ext_udp_ip", "127.0.0.1")
        r.setdefault("ext_udp_port", 9091)         # 요구사항: 기본 9091
        # Distance 입력 (RAW 기본)
        r.setdefault("distance_mode", "raw")       # "raw" | "mavlink"
        r.setdefault("distance_udp_listen_ip", "0.0.0.0")
        r.setdefault("distance_udp_listen_port", 14650)
        # 공용 Serial (OpticalFlow + Distance out)
        r.setdefault("serial_port", "")
        r.setdefault("serial_baud", 115200)
        r.setdefault("flow_sensor_id", 0)
        # Auto-start
        r.setdefault("autostart", False)
        # Optical Flow 스케일/품질 모델
        r.setdefault("of_scale_pix", 100.0)
        r.setdefault("q_base", 255)
        r.setdefault("q_min", 0)
        r.setdefault("q_max", 255)
        r.setdefault("accel_thresh", 5.0)
        r.setdefault("gyro_thresh", 2.0)
        r.setdefault("accel_penalty", 20.0)
        r.setdefault("gyro_penalty", 30.0)
        # Heartbeat (Optical Flow)
        r.setdefault("hb_of_rate_hz", 1.0)
        r.setdefault("hb_of_sysid", 42)
        r.setdefault("hb_of_compid", 199)
        r.setdefault("hb_of_type", 18)
        r.setdefault("hb_of_autopilot", 8)
        r.setdefault("hb_of_base_mode", 0)
        r.setdefault("hb_of_custom_mode", 0)
        r.setdefault("hb_of_system_status", 4)
        # Heartbeat (Distance)
        r.setdefault("hb_ds_rate_hz", 1.0)
        r.setdefault("hb_ds_sysid", 43)
        r.setdefault("hb_ds_compid", 200)
        r.setdefault("hb_ds_type", 18)
        r.setdefault("hb_ds_autopilot", 8)
        r.setdefault("hb_ds_base_mode", 0)
        r.setdefault("hb_ds_custom_mode", 0)
        r.setdefault("hb_ds_system_status", 4)

    # ---------- 편의 메서드 ----------

    def get_save_dir(self) -> str:
        """
        현재 설정의 저장 폴더(루트 savedata) 반환.
        """
        return get_default_save_dir()

    def get_config_path(self) -> str:
        return os.path.join(self.get_save_dir(), "config.json")


# ------------------------------------------------------------
# 설정 매니저
# ------------------------------------------------------------

class ConfigManager:
    """
    설정 파일 로드/저장 및 기본값 부여. 파일 위치는:
      <exe 또는 main.py 있는 곳>/savedata/config.json
    """
    def __init__(self, base_dir: Optional[str] = None):
        self.base_dir = base_dir or get_default_save_dir()
        ensure_dir(self.base_dir)
        self.config_path = os.path.join(self.base_dir, "config.json")

    def load(self) -> AppConfig:
        raw = load_json(self.config_path, default={})
        # 마이그레이션 훅 (필요시 확장)
        migrated = self._migrate_if_needed(raw)
        ac = AppConfig.from_dict(migrated)
        # 저장 경로가 바뀌었거나 기본값 주입되었으면 저장(선택)
        try:
            self.save(ac)
        except Exception:
            # 저장 실패는 치명적이지 않음
            pass
        return ac

    def save(self, cfg: AppConfig) -> None:
        ensure_dir(self.base_dir)
        save_json(self.config_path, cfg.to_dict())

    # ---------- 마이그레이션 훅 ----------
    def _migrate_if_needed(self, d: Dict[str, Any]) -> Dict[str, Any]:
        """
        예전 버전에서 appdata 등 다른 경로를 쓰던 설정을
        현재의 savedata 구조로 자연스럽게 옮기고 싶은 경우 사용.
        지금은 pass-through로 두되, 추후 키 이름 변경/이동에 대응.
        """
        if not isinstance(d, dict):
            return {}
        out = dict(d)

        # (예시) 예전 키명을 새 키로 옮기는 샘플
        # if "relay_ext_port" in out.get("relay", {}):
        #     out["relay"]["ext_udp_port"] = out["relay"].pop("relay_ext_port")

        return out


# ------------------------------------------------------------
# 모듈 외부에서 바로 쓰기 편한 헬퍼
# ------------------------------------------------------------

def load_config() -> AppConfig:
    """
    단순 로드 헬퍼.
    """
    cm = ConfigManager()
    return cm.load()


def save_config(cfg: AppConfig) -> None:
    """
    단순 저장 헬퍼.
    """
    cm = ConfigManager()
    cm.save(cfg)
