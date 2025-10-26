# utils/helpers.py
# -*- coding: utf-8 -*-
"""Utility helpers for the desktop bridge suite.

The module intentionally keeps dependencies light so it can be imported by
both GUI and headless entry points without side effects.  The available
helpers are:

* :func:`has_display` - Detect whether a display is available (mainly for
  Linux headless deployments).
* :func:`get_logger` - Provide a lazily configured logger without duplicating
  handlers when the module is imported multiple times.
* :func:`euler_to_quat` - Convert roll, pitch, yaw degrees into a quaternion
  tuple (x, y, z, w) using the ZYX convention.
* :func:`clamp` - Clamp a value to optional minimum/maximum bounds.
* :func:`rate_limit` - Limit the absolute value of a signal to the provided
  threshold.
"""

from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from collections import deque
from threading import Lock
import math
import os
import sys
from typing import Optional, Tuple, List


class _ConsoleEchoFilter(logging.Filter):
    """Filter that respects the record's ``console`` flag.

    Tk GUI 실행 시 PyInstaller로 패키징하면 콘솔 창이 보이지 않는 경우가 많아,
    ``console`` 속성이 False인 레코드는 스트림 핸들러에서 숨기고 파일 로그에는
    그대로 남겨 두기 위해 사용한다.
    """

    def filter(self, record: logging.LogRecord) -> bool:  # type: ignore[override]
        return bool(getattr(record, "console", True))


class _RecentBufferHandler(logging.Handler):
    """Keep a rolling buffer of formatted log records."""

    def __init__(self, capacity: int = 500) -> None:
        super().__init__()
        self._buffer: deque[str] = deque(maxlen=max(10, int(capacity)))
        self._lock = Lock()

    def emit(self, record: logging.LogRecord) -> None:  # type: ignore[override]
        try:
            message = self.format(record)
        except Exception:
            self.handleError(record)
            return
        with self._lock:
            self._buffer.append(message)

    def get_lines(self) -> List[str]:
        with self._lock:
            return list(self._buffer)

    def clear(self) -> None:
        with self._lock:
            self._buffer.clear()


def _resolve_runtime_root() -> str:
    """현재 실행 중인 바이너리/스크립트의 루트 디렉터리를 반환."""

    if getattr(sys, "frozen", False):  # PyInstaller
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def has_display() -> bool:
    """
    단순 디스플레이 유무 체크.
    - Linux: DISPLAY 또는 WAYLAND_DISPLAY 환경변수
    - macOS/Windows: 기본 True (대부분 GUI 사용 가능)
    """
    if sys.platform.startswith("linux"):
        return ("DISPLAY" in os.environ) or ("WAYLAND_DISPLAY" in os.environ)
    return True


def get_logger(
    name: str,
    *,
    enable_file_log: bool = False,
    log_file_path: Optional[str] = None,
) -> logging.Logger:
    """
    콘솔 핸들러는 기본으로, 파일 핸들러는 옵션으로 구성한 로거를 반환한다.

    동일 로거를 여러 번 요청해도 핸들러가 중복 추가되지 않도록 내부 플래그로
    상태를 추적한다.
    """
    log = logging.getLogger(name)
    if not getattr(log, "_bridge_logger_configured", False):
        log.setLevel(logging.INFO)
        fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
        setattr(log, "_bridge_formatter", fmt)

        ch = logging.StreamHandler()
        ch.setFormatter(fmt)
        ch.addFilter(_ConsoleEchoFilter())
        log.addHandler(ch)

        buffer_handler = _RecentBufferHandler(capacity=500)
        buffer_handler.setFormatter(fmt)
        log.addHandler(buffer_handler)

        setattr(log, "_bridge_logger_configured", True)
        setattr(log, "_bridge_file_log_enabled", False)
        setattr(log, "_bridge_file_handler", None)
        setattr(log, "_bridge_recent_handler", buffer_handler)

    if enable_file_log and not getattr(log, "_bridge_file_log_enabled", False):
        if not log_file_path:
            base_dir = _resolve_runtime_root()
            log_dir = os.path.join(base_dir, "savedata", "logs")
            os.makedirs(log_dir, exist_ok=True)
            log_file_path = os.path.join(log_dir, "bridge.log")
        else:
            os.makedirs(os.path.dirname(os.path.abspath(log_file_path)), exist_ok=True)

        fmt = getattr(log, "_bridge_formatter", logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        fh = RotatingFileHandler(log_file_path, maxBytes=2_000_000, backupCount=5, encoding="utf-8")
        fh.setFormatter(fmt)
        log.addHandler(fh)

        setattr(log, "log_file_path", log_file_path)
        setattr(log, "_bridge_file_log_enabled", True)
        setattr(log, "_bridge_file_handler", fh)
    elif not enable_file_log and getattr(log, "_bridge_file_log_enabled", False):
        handler = getattr(log, "_bridge_file_handler", None)
        if handler is not None:
            log.removeHandler(handler)
            handler.close()
        if hasattr(log, "log_file_path"):
            delattr(log, "log_file_path")
        setattr(log, "_bridge_file_log_enabled", False)
        setattr(log, "_bridge_file_handler", None)

    return log


def get_recent_log_lines(log: logging.Logger, limit: Optional[int] = None) -> List[str]:
    """Return the recent log backlog recorded by :func:`get_logger`."""

    handler = getattr(log, "_bridge_recent_handler", None)
    if handler is None:
        return []
    try:
        lines = handler.get_lines()
    except Exception:
        return []
    if limit is None:
        return lines
    if limit <= 0:
        return []
    return lines[-limit:]


def wrap_angle_deg(angle: float) -> float:
    """[-180°, 180°] 범위로 각도를 정규화."""

    if not math.isfinite(angle):
        return 0.0
    wrapped = math.fmod(angle, 360.0)
    if wrapped <= -180.0:
        wrapped += 360.0
    elif wrapped > 180.0:
        wrapped -= 360.0
    if wrapped == -0.0:
        return 0.0
    return wrapped


def euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float, float]:
    """
    ZYX(roll=X, pitch=Y, yaw=Z) 회전 순서 기준 쿼터니언 변환.
    입력 deg는 내부에서 [-180°, 180°]로 래핑되어 180° 이상 입력도 안정적으로 처리한다.
    반환: (x, y, z, w)
    """
    r = math.radians(wrap_angle_deg(roll_deg))
    p = math.radians(wrap_angle_deg(pitch_deg))
    y = math.radians(wrap_angle_deg(yaw_deg))

    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    qw /= norm
    qx /= norm
    qy /= norm
    qz /= norm

    def _zero_if_close(value: float) -> float:
        return 0.0 if abs(value) < 1e-12 else value

    return (
        _zero_if_close(qx),
        _zero_if_close(qy),
        _zero_if_close(qz),
        _zero_if_close(qw),
    )


def clamp(v: float, vmin: Optional[float], vmax: Optional[float]) -> float:
    """
    v를 [vmin, vmax] 영역으로 클램프.
    vmin 또는 vmax가 None이면 해당 제약을 무시.
    """
    if vmin is not None and v < vmin:
        return vmin
    if vmax is not None and v > vmax:
        return vmax
    return v


def rate_limit(x: float, limit: Optional[float]) -> float:
    """
    절대값이 limit를 넘지 않도록 제한. limit가 None이면 무제한.
    """
    if limit is None:
        return x
    if x > limit:
        return limit
    if x < -limit:
        return -limit
    return x
