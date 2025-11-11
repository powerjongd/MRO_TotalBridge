# -*- coding: utf-8 -*-
"""Support utilities: settings, helpers, and shared math."""

from __future__ import annotations

__version__ = "0.1.0"

# settings
try:
    from .settings import (  # noqa: F401
        AppConfig,
        BridgeSettings,
        GimbalSettings,
        RelaySettings,
        ConfigManager,
    )
except Exception:  # pragma: no cover
    AppConfig = BridgeSettings = GimbalSettings = RelaySettings = ConfigManager = None  # type: ignore

# helpers
try:
    from .helpers import (  # noqa: F401
        has_display,
        get_logger,
        euler_to_quat,
        wrap_angle_deg,
        remap_input_rpy,
        quat_conjugate_xyzw,
        quat_from_axis_angle,
        quat_from_wxyz,
        quat_multiply_xyzw,
        quat_normalize_xyzw,
        quat_to_axis_angle,
        quat_to_euler,
        clamp,
        rate_limit,
    )
except Exception:  # pragma: no cover
    has_display = (
        get_logger
    ) = (
        euler_to_quat
    ) = (
        wrap_angle_deg
    ) = remap_input_rpy = (
        quat_conjugate_xyzw
    ) = (
        quat_from_axis_angle
    ) = (
        quat_from_wxyz
    ) = (
        quat_multiply_xyzw
    ) = (
        quat_normalize_xyzw
    ) = (
        quat_to_axis_angle
    ) = (
        quat_to_euler
    ) = clamp = rate_limit = None  # type: ignore


__all__ = [
    "__version__",
    # settings
    "AppConfig",
    "BridgeSettings",
    "GimbalSettings",
    "RelaySettings",
    "ConfigManager",
    # helpers
    "has_display",
    "get_logger",
    "euler_to_quat",
    "wrap_angle_deg",
    "remap_input_rpy",
    "quat_conjugate_xyzw",
    "quat_from_axis_angle",
    "quat_from_wxyz",
    "quat_multiply_xyzw",
    "quat_normalize_xyzw",
    "quat_to_axis_angle",
    "quat_to_euler",
    "clamp",
    "rate_limit",
]
