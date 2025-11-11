"""PySide6 UI bootstrap for Unified Bridge."""
from __future__ import annotations

from typing import TYPE_CHECKING

try:
    from PySide6 import QtWidgets  # type: ignore

    _QT_AVAILABLE = True
except Exception:  # pragma: no cover - import guard for headless envs
    QtWidgets = None  # type: ignore
    _QT_AVAILABLE = False


def is_ui_available() -> bool:
    """Return True if PySide6 can be imported."""
    return _QT_AVAILABLE


if TYPE_CHECKING or _QT_AVAILABLE:
    from .main_window import MainWindow, run_gui  # noqa: F401
    from .gimbal_window import GimbalControlsDialog  # noqa: F401
    from .bridge_window import BridgeSettingsDialog  # noqa: F401
    from .relay_window import RelaySettingsDialog  # noqa: F401
    from .rover_relay_window import RoverRelaySettingsDialog  # noqa: F401
else:  # pragma: no cover - headless fallback
    MainWindow = None  # type: ignore
    run_gui = None  # type: ignore
    GimbalControlsDialog = None  # type: ignore
    BridgeSettingsDialog = None  # type: ignore
    RelaySettingsDialog = None  # type: ignore
    RoverRelaySettingsDialog = None  # type: ignore


__all__ = [
    "is_ui_available",
    "MainWindow",
    "run_gui",
    "GimbalControlsDialog",
    "BridgeSettingsDialog",
    "RelaySettingsDialog",
    "RoverRelaySettingsDialog",
]
