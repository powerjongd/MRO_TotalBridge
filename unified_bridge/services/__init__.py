# -*- coding: utf-8 -*-
"""Service layer public API.

This package contains UI-agnostic services (Unified Bridge modules):
- ImageStreamBridge: 영상 스트리밍(TCP 카메라 명령 + UDP 조립)
- GimbalControl:     짐벌 제어 모듈
- DroneRelay:        Gazebo/가상 센서 릴레이 모듈
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable, Dict, Any


@runtime_checkable
class CoreService(Protocol):
    """Common lifecycle interface for headless services."""

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def update_settings(self, settings: Dict[str, Any]) -> None: ...


# Re-export concrete services (lazy-safe)
try:
    from .image_stream_bridge import ImageStreamBridge  # noqa: F401
except Exception:  # pragma: no cover
    ImageStreamBridge = None  # type: ignore

try:
    from .gimbal_control import GimbalControl  # noqa: F401
except Exception:  # pragma: no cover
    GimbalControl = None  # type: ignore

try:
    from .drone_relay import DroneRelay  # noqa: F401
except Exception:  # pragma: no cover
    DroneRelay = None  # type: ignore


__all__ = [
    "CoreService",
    "ImageStreamBridge",
    "GimbalControl",
    "DroneRelay",
]
