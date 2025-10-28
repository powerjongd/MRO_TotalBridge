"""Zoom/Focal-length helper utilities."""
from __future__ import annotations

from math import isnan
from typing import Tuple

__all__ = [
    "ALLOWED_ZOOM_LENS_MM",
    "BASE_ZOOM_LENS_MM",
    "MAX_ZOOM_SCALE",
    "normalize_zoom_lens_dist",
    "zoom_scale_to_lens_mm",
    "lens_mm_to_zoom_scale",
]

ALLOWED_ZOOM_LENS_MM: Tuple[float, ...] = (20.0, 24.0, 28.0, 35.0, 50.0, 75.0)
BASE_ZOOM_LENS_MM: float = ALLOWED_ZOOM_LENS_MM[0]
MAX_ZOOM_SCALE: float = ALLOWED_ZOOM_LENS_MM[-1] / BASE_ZOOM_LENS_MM


def _sanitize_value(value: float) -> float:
    try:
        numeric = float(value)
    except Exception:
        return BASE_ZOOM_LENS_MM
    if isnan(numeric):
        return BASE_ZOOM_LENS_MM
    return numeric


def normalize_zoom_lens_dist(lens_mm: float) -> tuple[float, float]:
    """Clamp *lens_mm* to the supported list and return ``(mm, scale)``.

    The TCP protocol only accepts the discrete focal lengths described in
    :data:`ALLOWED_ZOOM_LENS_MM`. Values outside the range are clamped and then
    rounded to the nearest supported entry (round-half-up semantics are
    achieved by picking the smallest candidate when the distance matches).
    """

    value = _sanitize_value(lens_mm)
    lower, upper = ALLOWED_ZOOM_LENS_MM[0], ALLOWED_ZOOM_LENS_MM[-1]
    value = max(lower, min(upper, value))
    mm = min(ALLOWED_ZOOM_LENS_MM, key=lambda candidate: (abs(candidate - value), candidate))
    scale = mm / BASE_ZOOM_LENS_MM
    return mm, scale


def lens_mm_to_zoom_scale(lens_mm: float) -> float:
    value = _sanitize_value(lens_mm)
    return max(1.0, min(MAX_ZOOM_SCALE, value / BASE_ZOOM_LENS_MM))


def zoom_scale_to_lens_mm(scale: float, *, clamp: bool = False) -> float:
    value = _sanitize_value(scale) * BASE_ZOOM_LENS_MM
    if not clamp:
        return value
    lower, upper = ALLOWED_ZOOM_LENS_MM[0], ALLOWED_ZOOM_LENS_MM[-1]
    return max(lower, min(upper, value))
