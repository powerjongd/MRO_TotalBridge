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
import math
import os
import sys
from typing import Optional, Tuple


def has_display() -> bool:
    """
    단순 디스플레이 유무 체크.
    - Linux: DISPLAY 또는 WAYLAND_DISPLAY 환경변수
    - macOS/Windows: 기본 True (대부분 GUI 사용 가능)
    """
    if sys.platform.startswith("linux"):
        return ("DISPLAY" in os.environ) or ("WAYLAND_DISPLAY" in os.environ)
    return True


def get_logger(name: str) -> logging.Logger:
    """
    콘솔 스트림 핸들러 1개만 가진 로거를 반환.
    """
    log = logging.getLogger(name)
    if not log.handlers:
        log.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
        ch.setFormatter(fmt)
        log.addHandler(ch)
    return log


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
