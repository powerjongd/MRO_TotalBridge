# core/__init__.py
# -*- coding: utf-8 -*-
"""
Core layer public API.

This package contains UI-agnostic services:
- ImageBridgeCore: UDP image reassembly + TCP command server
- GimbalControl:   MAVLink(Command Long) intake -> 10706 out (angle/rate controller)
- UdpRelay:        UDP relay (raw/proc) with simple parse/transform hooks
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
    from .bridge_core import ImageBridgeCore  # noqa: F401
except Exception:  # pragma: no cover
    ImageBridgeCore = None  # type: ignore

try:
    from .gimbal_control import GimbalControl  # noqa: F401
except Exception:  # pragma: no cover
    GimbalControl = None  # type: ignore

try:
    from .udp_relay import UdpRelay  # noqa: F401
except Exception:  # pragma: no cover
    UdpRelay = None  # type: ignore


__all__ = [
    "CoreService",
    "ImageBridgeCore",
    "GimbalControl",
    "UdpRelay",
]
