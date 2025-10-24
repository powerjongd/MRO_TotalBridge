# Contributor Guide for MRO TotalBridge

This document captures the functional expectations, configuration layout, and
coding conventions for the entire repository. Any change inside this repo must
follow the rules below unless a more specific `AGENTS.md` is present in a
subdirectory.

## Application Overview

MRO TotalBridge (a.k.a. Unified Bridge) is a desktop relay hub that links the
MORAI simulator, custom sensor payloads, and external control stacks. The
application exposes both a Tkinter-based GUI and a headless CLI and stores all
runtime state beside the executable (`savedata/`, `SaveFile/`, etc.). The
high-level features are:

- **Image Stream Bridge** – Receives UDP JPEG frames, provides an interactive
  preview, serves images via the `MroCameraControl` TCP service, and manages
  real-time vs. predefined image sources including digital zoom handling.
- **Gimbal Control** – Sends sensor pose/power updates over UDP (ports 10706 and
  10707), exposes presets in the GUI, and mirrors updates to the
  `GimbalControl` TCP endpoint. Zoom commands are shared with the image bridge.
- **Sensor Relay (Gazebo)** – Listens for Gazebo simulator packets, forwards
  them to an external controller, converts to MAVLink optical-flow messages,
  and handles generator distance telemetry including 1 Hz heartbeats.
- **Rover Relay Logging** – Bridges rover control/feedback UDP streams through
  configurable IP/port pairs while optionally recording payloads to timestamped
  log files. Gazebo and rover logging are mutually exclusive by design.
- **Serial/MAVLink Stack** – Manages a bidirectional MAVLink link, exposing
  specific message types and supporting parameter query/response workflows.
- **Configuration Store** – Persists every GUI/CLI setting atomically to
  `savedata/config.json`, including logging toggles and relay endpoints.

## Directory Structure

- `main.py` – Entry point. Parses CLI arguments, bootstraps settings, launches
  the GUI event loop or headless services, and coordinates shutdown.
- `core/`
  - `image_stream_bridge.py` – Core logic for the image pipeline (UDP reception,
    JPEG buffering, TCP camera server, digital zoom processing).
  - `gimbal_control.py` – UDP/TCP gimbal control loop, preset management, and
    synchronization with the UI.
  - `udp_relay.py` – Gazebo relay processes, distance/heartbeat handling, and
    logging guards shared with rover relay logging.
  - `rover_relay_logger.py` – Rover relay logging orchestrator. Manages Tk
    windows, async loops, log writers, and exclusivity with Gazebo logging.
- `ui/`
  - `main_window.py` – Tk main window composition, binding of core modules,
    logging status widgets, and navigation to configuration dialogs.
  - `rover_relay_window.py` – Rover relay settings/monitor UI with Tk variable
    bindings, validation, and live status updates.
  - `__init.py` – GUI bootstrap helpers (thread coordination, style, dialogs).
- `utils/`
  - `settings.py` – Settings dataclasses, schema evolution helpers, atomic JSON
    persistence, and defaults for every feature.
- `network/` – Low-level socket helpers and shared protocol definitions.

## Functional Expectations

1. **Mutually Exclusive Logging** – Gazebo and rover relay logging features must
   remain mutually exclusive. UI toggles and backend checks should prevent both
   from being active simultaneously and surface clear status messages.
2. **Atomic Configuration Writes** – Continue using write-to-temp-file then
   rename semantics in `utils.settings.save_settings`. Never introduce partial
   writes.
3. **Thread & Async Safety** – Tk operations must run on the main thread. When
   background threads interact with Tk variables/widgets, schedule via
   `ui.__init.run_on_main_thread` (or equivalent). Long-running IO must avoid
   blocking the GUI event loop.
4. **UDP/TCP Protocol Stability** – Do not change command IDs, binary layouts,
   or default ports without clear backward compatibility notes. Sensor control,
   MAVLink, and camera protocols are relied upon by external systems.
5. **Log File Format** – Rover and Gazebo log files store one line per packet
   with `YYYY-MM-DD HH:MM:SS.mmm\tlen=<bytes>\t<hex payload>` formatting.
   Preserve this layout so downstream analyzers continue to parse the logs.
6. **Saved Paths** – Respect relative directories (`SaveFile/`,
   `PreDefinedImageSet/`, `savedata/`). Any new assets should default to
   locations beside `main.py` to preserve portability.

## Coding Conventions

- Python 3.10+ syntax is available; prefer type hints on new functions and
  dataclasses.
- Keep GUI strings user-facing in Korean where already localized; follow
  existing tone when introducing new labels.
- Avoid catching broad exceptions unless re-raising with context. Use the
  existing logging helpers for error reporting.
- Tests: the project currently uses manual/functional testing. When feasible,
  run `python -m compileall .` to verify syntax.
- Follow Black-ish formatting (4-space indents, trailing commas where natural),
  but do not introduce a dependency on Black.

## Documentation & Comments

- Update this file when introducing notable subsystems or altering workflows.
- Inline comments should explain *why* rather than *what* unless the protocol is
  non-obvious.
- Keep README.md in sync with major feature additions to aid operators.

## Network Resiliency Notes (2024-06)

- **UDP 이미지 재조립 안정화** – `core/image_stream_bridge.py` 의 UDP 루프는
  이제 프레임마다 헤더 필드를 검증하고, 조각 수/총 크기/오프셋이 사양을
  벗어나면 해당 세션을 폐기합니다. 최대 조각 크기(64,970바이트)와 총
  이미지 크기(128MB) 상한을 초과하는 값은 즉시 거부하며, 3초 동안 갱신이
  없으면 부분 프레임을 타임아웃 처리합니다. 소켓 오류가 발생하면
  자동으로 소켓을 닫고 짧은 백오프 후 재오픈하여 수신이 중단되지 않게
  했습니다.
- **TCP 명령 경로 보호** – 헤더 길이 값은 최소/최대 범위를 검사하고,
  페이로드는 길이에 따라 적응형 타임아웃으로 수신합니다. 타임아웃이나
  길이 불일치가 발생하면 연결을 정리하고 클라이언트가 재연결할 수 있게
  하여 장시간 블로킹을 방지합니다. 명령 헤더를 기다리는 동안에는 타임아웃
  없이 소켓 타임아웃(1초)만 반복해서 polling 하므로, 장시간 명령이 없는
  유휴 연결도 끊지 않고 유지합니다.

By contributing to this repository, you agree to follow these guidelines to
ensure consistent behavior across the Unified Bridge tooling.
