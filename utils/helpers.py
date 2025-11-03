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

    # UI/TCP/MAVLink 스택은 전통적으로 Roll, Pitch, Yaw 순서로 값을 노출하지만
    # 언리얼 엔진 시뮬레이터는 (Pitch, Yaw, Roll) 순서의 ``FRotator`` 값을
    # 기대한다. 후속 코드가 축을 혼동하지 않도록 시뮬레이터 순서로 재배열한다.
    sim_pitch = float(pitch_deg)
    sim_yaw = float(yaw_deg)
    sim_roll = float(roll_deg)
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

    # Intrinsic rotations multiply in reverse axis order (Z → Y → X).
    qx, qy, qz, qw = quat_multiply_xyzw(quat_multiply_xyzw(q_yaw, q_pitch), q_roll)

    return quat_normalize_xyzw((qx, qy, qz, qw))


def quat_multiply_xyzw(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Multiply two quaternions expressed in ``(x, y, z, w)`` order."""

    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate_xyzw(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Return the quaternion conjugate while preserving ``(x, y, z, w)`` ordering."""

    x, y, z, w = q
    return (-x, -y, -z, w)


def _zero_small_components(values: Tuple[float, ...], *, threshold: float = 1e-12) -> Tuple[float, ...]:
    return tuple(0.0 if abs(v) < threshold else v for v in values)


def quat_normalize_xyzw(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Normalize and canonicalize a quaternion in ``(x, y, z, w)`` form."""

    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-15:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / norm
    x *= inv
    y *= inv
    z *= inv
    w *= inv
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    return _zero_small_components((x, y, z, w))


def quat_to_euler(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> Tuple[float, float, float]:
    """Convert a quaternion into roll, pitch, yaw angles (degrees)."""

    w = float(qw)
    x = float(qx)
    y = float(qy)
    z = float(qz)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return (
        wrap_angle_deg(roll),
        wrap_angle_deg(pitch),
        wrap_angle_deg(yaw),
    )


def quat_to_axis_angle(
    q: Tuple[float, float, float, float]
) -> Tuple[Tuple[float, float, float], float]:
    """Return the unit rotation axis and rotation angle (radians)."""

    qx, qy, qz, qw = quat_normalize_xyzw(q)
    qw = max(-1.0, min(1.0, qw))
    angle = 2.0 * math.acos(qw)
    sin_half = math.sqrt(max(0.0, 1.0 - qw * qw))
    if sin_half < 1e-8:
        return (0.0, 0.0, 0.0), 0.0
    inv = 1.0 / sin_half
    axis = (qx * inv, qy * inv, qz * inv)
    return (_zero_small_components(axis, threshold=1e-9), angle)


def quat_from_axis_angle(
    axis: Tuple[float, float, float], angle_rad: float
) -> Tuple[float, float, float, float]:
    """Construct a quaternion from an axis-angle rotation."""

    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < 1e-12 or abs(angle_rad) < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / norm
    ax *= inv
    ay *= inv
    az *= inv
    half = angle_rad * 0.5
    s = math.sin(half)
    c = math.cos(half)
    return quat_normalize_xyzw((ax * s, ay * s, az * s, c))


def quat_from_wxyz(
    w: float, x: float, y: float, z: float
) -> Tuple[float, float, float, float]:
    """Convert a ``(w, x, y, z)`` quaternion into canonical ``(x, y, z, w)`` form."""

    return quat_normalize_xyzw((float(x), float(y), float(z), float(w)))


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
