# ui/__init__.py
# -*- coding: utf-8 -*-
"""
UI layer public API (Tkinter-based).

Expose:
- MainWindow:            main application window
- GimbalControlsWindow:  gimbal/sensor control popup
- RelaySettingsWindow:   UDP relay settings popup

Utilities:
- is_ui_available(): whether Tk is importable and a display is (likely) present
"""

from __future__ import annotations

from typing import Any

# Tk availability check (import-level)
try:
    import tkinter as _tk  # noqa: F401
    _TK_AVAILABLE = True
except Exception:
    _TK_AVAILABLE = False


def is_ui_available() -> bool:
    """Return True if Tkinter is importable (display check is app-level)."""
    return _TK_AVAILABLE


# Re-export windows (guarded to avoid import errors in headless builds)
try:
    from .main_window import MainWindow  # noqa: F401
except Exception:  # pragma: no cover
    MainWindow = None  # type: ignore

try:
    from .gimbal_window import GimbalControlsWindow  # noqa: F401
except Exception:  # pragma: no cover
    GimbalControlsWindow = None  # type: ignore

try:
    from .relay_window import RelaySettingsWindow  # noqa: F401
except Exception:  # pragma: no cover
    RelaySettingsWindow = None  # type: ignore

try:
    from .rover_relay_window import RoverRelaySettingsWindow  # noqa: F401
except Exception:  # pragma: no cover
    RoverRelaySettingsWindow = None  # type: ignore


__all__ = [
    "is_ui_available",
    "MainWindow",
    "GimbalControlsWindow",
    "RelaySettingsWindow",
    "RoverRelaySettingsWindow",
]
