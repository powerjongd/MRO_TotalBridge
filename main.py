# main.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import Dict, Any, Optional

from utils.helpers import has_display, get_logger
from utils.settings import ConfigManager, AppConfig
from utils.observers import ObservableFloat
from core.image_stream_bridge import ImageStreamBridge
from core.gimbal_control import GimbalControl
from core.udp_relay import UdpRelay
from core.rover_relay_logger import RoverRelayLogger


#사용법
# GUI(표준): 디스플레이 있는 환경에서 그냥 실행
# python main.py
# 헤드리스 강제 + 콘솔 HUD on: 디스플레이 없는 환경에서는 아래와 같이 실행해주세요
# python main.py --no-gui --console-hud --bridge-ip 0.0.0.0 --bridge-tcp 9999 --bridge-udp 9998



# -----------------------------
# CLI
# -----------------------------
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="unified-bridge",
        description=(
            "Unified Bridge (GUI/Headless) with Image Stream, Gimbal, and Sensor Relay modules."
        ),
    )
    # mode
    p.add_argument("--no-gui", action="store_true", help="Force headless mode (ignore display).")

    # console HUD
    g = p.add_mutually_exclusive_group()
    g.add_argument("--console-hud", action="store_true", help="Enable periodic console HUD.")
    g.add_argument("--no-console-hud", action="store_true", help="Disable periodic console HUD.")
    p.add_argument("--hud-interval", type=float, default=1.0, help="Console HUD interval seconds (default: 1.0)")

    # console logging toggle
    log_group = p.add_mutually_exclusive_group()
    log_group.add_argument("--console-log", dest="console_log", action="store_true", help="Force-enable console log output.")
    log_group.add_argument("--no-console-log", dest="console_log", action="store_false", help="Disable console log output.")
    p.set_defaults(console_log=None)

    # optional file logging
    file_log_group = p.add_mutually_exclusive_group()
    file_log_group.add_argument(
        "--log-file",
        dest="log_file",
        action="store_true",
        help="Enable rotating file logging to savedata/logs/bridge.log.",
    )
    file_log_group.add_argument(
        "--no-log-file",
        dest="log_file",
        action="store_false",
        help="Disable file logging (default).",
    )
    p.set_defaults(log_file=False)
    p.add_argument(
        "--log-file-path",
        type=str,
        help="Custom file path used when --log-file is specified.",
    )

    # Bridge overrides
    p.add_argument("--bridge-ip", type=str, help="Bridge bind IP (TCP/UDP).")
    p.add_argument("--bridge-tcp", type=int, help="Bridge TCP port.")
    p.add_argument("--bridge-udp", type=int, help="Bridge UDP port.")
    p.add_argument("--images", type=str, help="(Legacy) images directory alias for realtime SaveFile.")
    p.add_argument("--realtime-dir", type=str, help="SaveFile directory for realtime captures.")
    p.add_argument("--predefined-dir", type=str, help="PreDefined image set directory.")
    p.add_argument(
        "--image-source-mode",
        type=str,
        choices=["realtime", "predefined"],
        help="Select TCP image source library.",
    )

    # (Optional) Gimbal overrides
    p.add_argument("--gimbal-bind-ip", type=str, help="Gimbal RX bind IP.")
    p.add_argument("--gimbal-bind-port", type=int, help="Gimbal RX bind port.")
    p.add_argument("--gen-ip", type=str, help="Generator IP for 10706.")
    p.add_argument("--gen-port", type=int, help="Generator port for 10706.")
    p.add_argument("--sensor-type", type=int, help="Sensor type code (0:Camera,1:GPS,2:LiDAR,3:RADAR,4:LRF,5:IMU).")
    p.add_argument("--sensor-id", type=int, help="Sensor ID.")
    p.add_argument(
        "--gimbal-control-method",
        type=str,
        choices=["tcp", "mavlink"],
        help="Select gimbal control transport (TCP or MAVLink).",
    )
    p.add_argument(
        "--show-gimbal-packets",
        action="store_true",
        help="Display the packed 10706 gimbal control bytes in the GUI log window.",
    )

    # (Optional) Relay overrides
    p.add_argument("--relay-bind-ip", type=str, help="Relay input bind IP.")
    p.add_argument("--relay-port", type=int, help="Relay input port.")
    p.add_argument("--relay-raw-ip", type=str, help="Relay RAW dst IP.")
    p.add_argument("--relay-raw-port", type=int, help="Relay RAW dst port.")
    p.add_argument("--relay-proc-ip", type=str, help="Relay PROC dst IP.")
    p.add_argument("--relay-proc-port", type=int, help="Relay PROC dst port.")

    return p.parse_args()


# -----------------------------
# Helpers
# -----------------------------
def prompt_if_missing_headless(args: argparse.Namespace, cfg: Dict[str, Any], log) -> None:
    """
    Headless 모드에서 Bridge IP/TCP/UDP 가 인자로 오지 않았다면 콘솔에서 입력받아 채움.
    """
    b = cfg.setdefault("bridge", {})
    def _ask(txt: str, default: str) -> str:
        try:
            s = input(f"{txt} [{default}]: ").strip()
            return s or default
        except EOFError:
            return default

    if args.bridge_ip is None:
        b["ip"] = _ask("Bridge IP", b.get("ip", "0.0.0.0"))
    else:
        b["ip"] = args.bridge_ip

    if args.bridge_tcp is None:
        try:
            b["tcp_port"] = int(_ask("Bridge TCP port", str(b.get("tcp_port", 9999))))
        except ValueError:
            log.warning("Invalid input; using default TCP 9999")
            b["tcp_port"] = 9999
    else:
        b["tcp_port"] = int(args.bridge_tcp)

    if args.bridge_udp is None:
        try:
            b["udp_port"] = int(_ask("Bridge UDP port", str(b.get("udp_port", 9998))))
        except ValueError:
            log.warning("Invalid input; using default UDP 9998")
            b["udp_port"] = 9998
    else:
        b["udp_port"] = int(args.bridge_udp)

    if args.images is not None:
        b["images"] = args.images


def apply_cli_overrides(args: argparse.Namespace, cfg: Dict[str, Any]) -> None:
    """
    CLI 인자를 config(dict)에 병합 (지정된 값만 반영).
    """
    # Bridge
    b = cfg.setdefault("bridge", {})
    if args.bridge_ip is not None:   b["ip"] = args.bridge_ip
    if args.bridge_tcp is not None:  b["tcp_port"] = int(args.bridge_tcp)
    if args.bridge_udp is not None:  b["udp_port"] = int(args.bridge_udp)
    if args.images is not None:
        b["images"] = args.images
        b["realtime_dir"] = args.images
    if args.realtime_dir is not None:
        b["realtime_dir"] = args.realtime_dir
        b["images"] = args.realtime_dir
    if args.predefined_dir is not None:
        b["predefined_dir"] = args.predefined_dir
    if args.image_source_mode is not None:
        b["image_source_mode"] = args.image_source_mode
    if getattr(args, "console_log", None) is not None:
        b["console_echo"] = bool(args.console_log)

    # Gimbal
    g = cfg.setdefault("gimbal", {})
    if args.gimbal_bind_ip is not None:  g["bind_ip"] = args.gimbal_bind_ip
    if args.gimbal_bind_port is not None:g["bind_port"] = int(args.gimbal_bind_port)
    if args.gen_ip is not None:          g["generator_ip"] = args.gen_ip
    if args.gen_port is not None:        g["generator_port"] = int(args.gen_port)
    if args.sensor_type is not None:     g["sensor_type"] = int(args.sensor_type)
    if args.sensor_id is not None:       g["sensor_id"] = int(args.sensor_id)
    if args.show_gimbal_packets:         g["debug_dump_packets"] = True
    if getattr(args, "gimbal_control_method", None) is not None:
        g["control_method"] = str(args.gimbal_control_method).lower()
    if args.sensor_type is not None:
        b["gimbal_sensor_type"] = int(args.sensor_type)
    if args.sensor_id is not None:
        b["gimbal_sensor_id"] = int(args.sensor_id)

    # Relay
    r = cfg.setdefault("relay", {})
    if args.relay_bind_ip is not None:   r["input_bind_ip"] = args.relay_bind_ip
    if args.relay_port is not None:      r["input_port"] = int(args.relay_port)
    if args.relay_raw_ip is not None:    r["dst_raw_ip"] = args.relay_raw_ip
    if args.relay_raw_port is not None:  r["dst_raw_port"] = int(args.relay_raw_port)
    if args.relay_proc_ip is not None:   r["dst_proc_ip"] = args.relay_proc_ip
    if args.relay_proc_port is not None: r["dst_proc_port"] = int(args.relay_proc_port)


def make_log_cb(logger, prefix: str, enabled: bool):
    def _log(msg: str, *args, **kwargs) -> None:
        """Proxy logger that enforces a prefix while supporting %-style args."""

        extra = dict(kwargs.pop("extra", {}) or {})
        extra.setdefault("console", enabled)
        if args:
            logger.info("%s " + msg, prefix, *args, extra=extra, **kwargs)
        else:
            # msg 자체에 prefix가 있을 수 있지만, 통일된 앞머리 보장을 위해 강제 prefix
            logger.info("%s %s", prefix, msg, extra=extra, **kwargs)

    return _log


def make_status_cb(logger, name: str, enabled: bool):
    def _status(s: str) -> None:
        logger.info("[%s][STATUS] %s", name, s, extra={"console": enabled})

    return _status


# -----------------------------
# Main
# -----------------------------
def main() -> None:
    args = parse_args()
    log_file_override = getattr(args, "log_file_path", None)
    enable_file_log = bool(getattr(args, "log_file", False)) or bool(log_file_override)
    log = get_logger(
        "unified-bridge",
        enable_file_log=enable_file_log,
        log_file_path=log_file_override,
    )
    log_file_path = getattr(log, "log_file_path", None)
    if log_file_path:
        log.info("[MAIN] Writing logs to %s", log_file_path)

    # 1) 설정 로드
    cfg_mgr = ConfigManager()
    appcfg: AppConfig = cfg_mgr.load()
    cfg_dict: Dict[str, Any] = appcfg.to_dict()

    # 2) CLI override 반영
    apply_cli_overrides(args, cfg_dict)

    # 3) 모드 결정 (디스플레이 유무 + --no-gui)
    gui_possible = has_display() and (not args.no_gui)
    headless = not gui_possible

    console_enabled = bool(cfg_dict.get("bridge", {}).get("console_echo", True))

    # 4) 헤드리스면 Bridge IP/TCP/UDP 없을 때 콘솔에서 입력
    if headless:
        prompt_if_missing_headless(args, cfg_dict, log)

    # 5) 인스턴스 생성
    bridge_log = make_log_cb(log, "[BRIDGE]", console_enabled)
    gimbal_log = make_log_cb(log, "[GIMBAL]", console_enabled)
    relay_log  = make_log_cb(log, "[RELAY]", console_enabled)
    rover_log  = make_log_cb(log, "[ROVER]", console_enabled)

    gimbal_cfg = cfg_dict.get("gimbal", {})
    method = str(gimbal_cfg.get("control_method", "tcp")).lower()
    if method not in ("tcp", "mavlink"):
        method = "tcp"
    gimbal_cfg["control_method"] = method
    try:
        initial_zoom = float(gimbal_cfg.get("zoom_scale", 1.0))
    except (TypeError, ValueError):
        initial_zoom = 1.0
    zoom_state = ObservableFloat(initial_zoom)

    bridge_cfg = cfg_dict.get("bridge", {})

    bridge = ImageStreamBridge(
        log_cb=bridge_log,
        preview_cb=None,  # GUI 미리보기는 UI 쪽에서 연결
        status_cb=make_status_cb(log, "BRIDGE", console_enabled),
        settings=bridge_cfg,
        zoom_update_cb=zoom_state.set,
    )

    zoom_state.subscribe(bridge.set_zoom_scale)

    gimbal = GimbalControl(
        log_cb=gimbal_log,
        status_cb=make_status_cb(log, "GIMBAL", console_enabled),
        settings=gimbal_cfg,
        zoom_update_cb=zoom_state.set,
    )

    bridge.attach_gimbal_controller(gimbal)
    try:
        sensor_type = int(bridge_cfg.get("gimbal_sensor_type", 0))
    except (TypeError, ValueError):
        sensor_type = 0
    try:
        sensor_id = int(bridge_cfg.get("gimbal_sensor_id", 0))
    except (TypeError, ValueError):
        sensor_id = 0
    bridge.configure_gimbal_forwarding(sensor_type, sensor_id)
    relay = UdpRelay(
        log_cb=relay_log,
        status_cb=make_status_cb(log, "RELAY", console_enabled),
        settings=cfg_dict.get("relay", {}),
    )
    rover = RoverRelayLogger(
        log_cb=rover_log,
        status_cb=make_status_cb(log, "ROVER", console_enabled),
        settings=cfg_dict.get("rover", {}),
    )
    relay.register_rover_logger(rover)
    rover.register_relay(relay)

    # 6) 콘솔 HUD 설정
    def hud_enabled_default() -> bool:
        # 기본값: headless -> on / GUI -> off
        return headless

    if args.console_hud:
        hud_enabled = True
    elif args.no_console_hud:
        hud_enabled = False
    else:
        hud_enabled = hud_enabled_default()

    hud_stop = threading.Event()

    def hud_loop():
        while not hud_stop.is_set():
            try:
                st = bridge.get_runtime_status()
                mode = st.get("image_source_mode", "-")
                active_dir = st.get("active_library_dir") or "-"
                log.info(
                    "[HUD] mode=%s | dir=%s | next=%03d | last=%.1fKB @ %s | saved=%s | TCP=%s UDP=%s",
                    mode,
                    active_dir,
                    st.get("next_image_number", -1),
                    st.get("last_image_kb", 0.0),
                    st.get("last_image_received_at") or "-",
                    st.get("last_saved_path") or "-",
                    "ON" if st.get("tcp_listening") else "OFF",
                    "ON" if st.get("udp_listening") else "OFF",
                )
            except Exception as e:
                log.error("[HUD] error: %s", e)
            hud_stop.wait(max(0.1, float(args.hud_interval)))

    hud_thread: Optional[threading.Thread] = None
    if hud_enabled:
        hud_thread = threading.Thread(target=hud_loop, daemon=True)
        hud_thread.start()

    # 7) 공통 종료 루틴
    def shutdown():
        log.info("[MAIN] Shutting down...")
        try:
            bridge.stop()
        except Exception:
            pass
        try:
            gimbal.stop()
        except Exception:
            pass
        try:
            relay.stop()
        except Exception:
            pass
        try:
            rover.stop()
        except Exception:
            pass
        try:
            hud_stop.set()
        except Exception:
            pass
        # 설정 저장 (메모리 cfg_dict → AppConfig → 파일)
        try:
            cfg_mgr.save(AppConfig.from_dict(cfg_dict))
            log.info("[MAIN] Config saved to %s", cfg_mgr.config_path)
        except Exception as e:
            log.error("[MAIN] Failed to save config: %s", e)

    # 8) 실행
    try:
        # 세 서비스 시작 (활성 플래그는 각 모듈에서 사용)
        bridge.start()
        gimbal.start()
        relay.start()
        if cfg_dict.get("rover", {}).get("autostart", False):
            try:
                rover.start()
            except Exception as e:
                log.error("[MAIN] Rover relay autostart failed: %s", e)

        if gui_possible:
            log.info("[MAIN] GUI mode")
            from ui.main_window import run_gui
            # UI에 dict를 넘겨 변경 사항을 반영하게 함 (팝업에서 저장/적용 시 cfg_dict를 수정)
            run_gui(
                cfg=cfg_dict,
                bridge=bridge,
                gimbal=gimbal,
                relay=relay,
                rover=rover,
                log=log,
                zoom_state=zoom_state,
            )
            shutdown()
        else:
            log.info("[MAIN] Headless mode (Ctrl+C to stop)")
            stop_ev = threading.Event()
            try:
                while not stop_ev.wait(0.5):
                    pass
            except KeyboardInterrupt:
                pass
            finally:
                shutdown()

    except Exception as e:
        log.error("[MAIN] Fatal error: %s", e)
        try:
            shutdown()
        finally:
            # 비정상 종료 시 non-zero
            sys.exit(1)
    
    try:
        if cfg.get("relay", {}).get("autostart", False):
            relay.update_settings(cfg.get("relay", {}))
            relay.start()
    except Exception as e:
        log.error("[MAIN] Relay autostart failed: %s", e)


if __name__ == "__main__":
    main()
