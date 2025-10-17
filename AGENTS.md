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

By contributing to this repository, you agree to follow these guidelines to
ensure consistent behavior across the Unified Bridge tooling.
