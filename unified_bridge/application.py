"""Application bootstrap and orchestration helpers for Unified Bridge."""
from __future__ import annotations

import sys
from typing import Any, Dict, Sequence

from PySide6 import QtCore, QtWidgets

from .services.gimbal_control import GimbalControl
from .services.image_stream_bridge import ImageStreamBridge
from .services.rover_relay_logger import RoverRelayLogger
from .services.drone_relay import DroneRelay
from .support.helpers import get_logger
from .support.observers import ObservableFloat
from .support.settings import AppConfig, ConfigManager


def _make_log_cb(logger, prefix: str, enabled: bool):
    def _log(msg: str, *args, **kwargs) -> None:
        extra = dict(kwargs.pop("extra", {}) or {})
        extra.setdefault("console", enabled)
        if args:
            logger.info("%s " + msg, prefix, *args, extra=extra, **kwargs)
        else:
            logger.info("%s %s", prefix, msg, extra=extra, **kwargs)

    return _log


def _make_status_cb(logger, name: str, enabled: bool):
    def _status(message: str) -> None:
        logger.info("[%s][STATUS] %s", name, message, extra={"console": enabled})

    return _status


class ServiceController(QtCore.QObject):
    """Qt-aware orchestrator for bridge services and shared state."""

    def __init__(self, cfg_mgr: ConfigManager, cfg: Dict[str, Any], parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.cfg_mgr = cfg_mgr
        self.cfg = cfg
        self.log = get_logger("unified-bridge")

        log_file_path = getattr(self.log, "log_file_path", None)
        if log_file_path:
            self.log.info("[MAIN] Writing logs to %s", log_file_path)

        bridge_cfg = self.cfg.setdefault("bridge", {})
        console_enabled = bool(bridge_cfg.get("console_echo", True))

        self.bridge_log = _make_log_cb(self.log, "[BRIDGE]", console_enabled)
        self.gimbal_log = _make_log_cb(self.log, "[GIMBAL]", console_enabled)
        self.relay_log = _make_log_cb(self.log, "[RELAY]", console_enabled)
        self.rover_log = _make_log_cb(self.log, "[ROVER]", console_enabled)

        gimbal_cfg = self.cfg.setdefault("gimbal", {})
        method = str(gimbal_cfg.get("control_method", "tcp")).lower()
        if method not in {"tcp", "mavlink"}:
            method = "tcp"
        gimbal_cfg["control_method"] = method

        try:
            initial_zoom = float(gimbal_cfg.get("zoom_scale", 1.0))
        except (TypeError, ValueError):
            initial_zoom = 1.0
        self.zoom_state = ObservableFloat(initial_zoom)

        self.bridge = ImageStreamBridge(
            log_cb=self.bridge_log,
            preview_cb=None,
            status_cb=_make_status_cb(self.log, "BRIDGE", console_enabled),
            settings=bridge_cfg,
            zoom_update_cb=self.zoom_state.set,
        )

        self.gimbal = GimbalControl(
            log_cb=self.gimbal_log,
            status_cb=_make_status_cb(self.log, "GIMBAL", console_enabled),
            settings=gimbal_cfg,
            zoom_update_cb=self.zoom_state.set,
        )

        self.zoom_state.subscribe(self.bridge.set_zoom_scale)
        self.zoom_state.subscribe(self.gimbal.set_zoom_scale)
        self.bridge.attach_gimbal_controller(self.gimbal)

        try:
            sensor_type = int(bridge_cfg.get("gimbal_sensor_type", 0))
        except (TypeError, ValueError):
            sensor_type = 0
        try:
            sensor_id = int(bridge_cfg.get("gimbal_sensor_id", 0))
        except (TypeError, ValueError):
            sensor_id = 0
        self.bridge.configure_gimbal_forwarding(sensor_type, sensor_id)

        self.relay = DroneRelay(
            log_cb=self.relay_log,
            status_cb=_make_status_cb(self.log, "RELAY", console_enabled),
            settings=self.cfg.get("relay", {}),
        )
        self.rover = RoverRelayLogger(
            log_cb=self.rover_log,
            status_cb=_make_status_cb(self.log, "ROVER", console_enabled),
            settings=self.cfg.get("rover", {}),
        )
        self.relay.register_rover_logger(self.rover)
        self.rover.register_relay(self.relay)

        self._shutdown_called = False

    def start_services(self) -> None:
        self.log.info("[MAIN] Starting services")
        self.bridge.start()
        self.gimbal.start()
        self.relay.start()
        if self.cfg.get("rover", {}).get("autostart", False):
            try:
                self.rover.start()
            except Exception as exc:
                self.log.error("[MAIN] Rover relay autostart failed: %s", exc)

    @QtCore.Slot()
    def shutdown(self) -> None:
        if self._shutdown_called:
            return
        self._shutdown_called = True
        self.log.info("[MAIN] Shutting down services")
        for svc in (self.bridge, self.gimbal, self.relay, self.rover):
            try:
                svc.stop()
            except Exception as exc:
                self.log.error("[MAIN] Failed to stop %s: %s", svc.__class__.__name__, exc)
        try:
            self.rover.close()
        except Exception as exc:
            self.log.warning("[MAIN] Rover logger close failed: %s", exc)
        try:
            self.cfg_mgr.save(AppConfig.from_dict(self.cfg))
            self.log.info("[MAIN] Config saved to %s", self.cfg_mgr.config_path)
        except Exception as exc:
            self.log.error("[MAIN] Failed to save config: %s", exc)


def create_application(argv: Sequence[str] | None = None) -> tuple[QtWidgets.QApplication, ServiceController]:
    """Create the Qt application and service controller."""

    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(list(argv or sys.argv))

    cfg_mgr = ConfigManager()
    cfg = cfg_mgr.load().to_dict()
    controller = ServiceController(cfg_mgr, cfg)

    return app, controller


def start_ui(app: QtWidgets.QApplication, controller: ServiceController) -> int:
    """Launch the Qt UI and return the exit code."""

    from .ui.main_window import MainWindow

    window = MainWindow(
        controller.cfg,
        controller.bridge,
        controller.gimbal,
        controller.relay,
        controller.rover,
        controller.log,
        zoom_state=controller.zoom_state,
    )
    app.aboutToQuit.connect(controller.shutdown)
    window.show()

    exit_code = app.exec()
    controller.shutdown()
    return exit_code


def run(argv: Sequence[str] | None = None) -> int:
    """Convenience runner used by the CLI entry point."""

    app, controller = create_application(argv)
    try:
        controller.start_services()
    except Exception as exc:
        controller.log.error("[MAIN] Fatal startup error: %s", exc)
        QtWidgets.QMessageBox.critical(None, "Startup failed", f"서비스 시작 실패:\n{exc}")
        controller.shutdown()
        return 1

    return start_ui(app, controller)
