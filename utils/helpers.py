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


def remap_input_rpy(
    roll_deg: float, pitch_deg: float, yaw_deg: float
) -> Tuple[float, float, float]:
    """Remap UI/TCP/MAV roll/pitch/yaw inputs into simulator order."""

    # 각 채널(UI, TCP/IP, MAVLink)에서 전달되는 오일러각은 레이블과 실제 축이
    # 순환(rotated) 관계를 가진다. 입력 ``roll`` 값은 시뮬레이터의 Pitch, 입력
    # ``pitch`` 값은 시뮬레이터의 Yaw, 입력 ``yaw`` 값은 시뮬레이터의 Roll에
    # 대응하므로 이를 (Pitch, Yaw, Roll) 순서로 재배열한다.
    sim_pitch = float(roll_deg)
    sim_yaw = float(pitch_deg)
    sim_roll = float(yaw_deg)
    return sim_pitch, sim_yaw, sim_roll


def euler_to_quat(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    *,
    order: str = "ZYX",
) -> Tuple[float, float, float, float]:
    """Convert Euler angles (degrees) into a quaternion following the desired order.

    Parameters
    ----------
    roll_deg, pitch_deg, yaw_deg:
        Bridge-native roll/pitch/yaw angles expressed in degrees.
    order:
        Rotation order string.  Only ``"ZYX"`` (intrinsic yaw → pitch → roll) is
        supported at the moment because the Unreal Engine ``FRotator`` applies
        rotations in that order.  Passing another value raises ``ValueError`` so
        call sites never accidentally fall back to a different convention.

    Returns
    -------
    Tuple[float, float, float, float]
        Quaternion in ``(x, y, z, w)`` form that matches the supplied rotation.
        The scalar component ``w`` is always non-negative so downstream
        interpolation picks the shortest arc.
    """

    order = order.upper()
    if order != "ZYX":
        raise ValueError(f"Unsupported Euler order '{order}'. Only 'ZYX' is allowed.")

    # Unreal Engine는 좌표계를 Z(위) → Y(우) → X(앞) 축 기준 내적 회전(intrinsic ZYX)
    # 순서로 적용한다.  따라서 입력된 roll/pitch/yaw를 순서대로 roll(X), pitch(Y),
    # yaw(Z) 축 회전에 대응시킨 뒤 yaw → pitch → roll 순으로 조합한다.

    r = math.radians(wrap_angle_deg(roll_deg))
    p = math.radians(wrap_angle_deg(pitch_deg))
    y = math.radians(wrap_angle_deg(yaw_deg))

    # Build individual axis quaternions.  Each quaternion is normalized so the
    # final multiplication stays numerically stable even near 180° inputs.
    sr, cr = math.sin(r * 0.5), math.cos(r * 0.5)
    sp, cp = math.sin(p * 0.5), math.cos(p * 0.5)
    sy, cy = math.sin(y * 0.5), math.cos(y * 0.5)

    q_roll = (sr, 0.0, 0.0, cr)
    q_pitch = (0.0, sp, 0.0, cp)
    q_yaw = (0.0, 0.0, sy, cy)

    def _quat_mul(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    # Intrinsic rotations multiply in reverse axis order (Z → Y → X).
    qx, qy, qz, qw = _quat_mul(_quat_mul(q_yaw, q_pitch), q_roll)

    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    qw /= norm
    qx /= norm
    qy /= norm
    qz /= norm

    # ``q``와 ``-q``는 동일한 회전을 나타내므로, 스칼라부(w)를 양수로 맞춰
    # 표현을 고정해 둔다. 이후 보간 시 음수 스칼라로 인해 긴 경로를 선택하는
    # 일을 피할 수 있다.
    if qw < 0.0:
        qw = -qw
        qx = -qx
        qy = -qy
        qz = -qz

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
