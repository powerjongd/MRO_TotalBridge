"""CSV helpers for the unified drone/rover log format."""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Iterable, Sequence


UNIFIED_CSV_HEADER = (
    "time_usec,position_x,position_y,position_z,"
    "orientation_w,orientation_x,orientation_y,orientation_z,"
    "angular_velocity_x,angular_velocity_y,angular_velocity_z"
)


def _format_float(value: float) -> str:
    """Format floats for CSV output, preserving NaN semantics."""

    if math.isnan(value):
        return "NaN"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return f"{value:.9f}".rstrip("0").rstrip(".") or "0"


def _utc_timestamp_usec() -> int:
    """Return the current UTC timestamp in microseconds."""

    return time.time_ns() // 1000


def format_unified_row(
    *,
    time_usec: int,
    position: Sequence[float],
    orientation_wxyz: Sequence[float],
    angular_velocity: Sequence[float],
) -> str:
    """Format a row that matches :data:`UNIFIED_CSV_HEADER`."""

    px, py, pz = position
    ow, ox, oy, oz = orientation_wxyz
    avx, avy, avz = angular_velocity
    fields: Iterable[str] = (
        str(int(time_usec)),
        _format_float(float(px)),
        _format_float(float(py)),
        _format_float(float(pz)),
        _format_float(float(ow)),
        _format_float(float(ox)),
        _format_float(float(oy)),
        _format_float(float(oz)),
        _format_float(float(avx)),
        _format_float(float(avy)),
        _format_float(float(avz)),
    )
    return ",".join(fields)


def format_drone_csv_row(
    *,
    time_usec: int,
    position: Sequence[float],
    orientation_wxyz: Sequence[float],
    angular_velocity: Sequence[float],
) -> str:
    """Expose the drone ghost-mode payload using the unified CSV layout."""

    return format_unified_row(
        time_usec=time_usec,
        position=position,
        orientation_wxyz=orientation_wxyz,
        angular_velocity=angular_velocity,
    )


@dataclass
class RoverFeedbackCsvFormatter:
    """Stateful formatter that maps rover feedback packets to CSV rows."""

    _last_time_usec: int | None = None
    _last_position: tuple[float, float, float] | None = None

    def reset(self) -> None:
        """Clear cached state so subsequent packets start fresh."""

        self._last_time_usec = None
        self._last_position = None

    def format_packet(self, data: bytes) -> str:
        """Convert a rover feedback packet into the unified CSV layout."""

        if len(data) < 26:
            raise ValueError("rover feedback packet too short")
        if data[0:2] != b"PU":
            raise ValueError("invalid rover feedback packet signature")

        # Parse payload per ICD (little-endian floats/ints).
        x, y, z, heading_deg = struct.unpack_from("<ffff", data, 5)
        velocity_raw = struct.unpack_from("<b", data, 21)[0]

        # Convert timestamp immediately to support derivative fallbacks.
        time_usec = _utc_timestamp_usec()

        # Position expressed in metres in the aircraft-aligned frame.
        position = (float(x), float(y), float(z))

        # Orientation: heading is defined clockwise from north with +Z up.
        heading_rad_clockwise = math.radians(float(heading_deg) % 360.0)
        yaw_rad = -heading_rad_clockwise  # Convert to right-handed CCW convention.
        half_yaw = yaw_rad / 2.0
        orientation = (
            math.cos(half_yaw),  # w
            0.0,  # x
            0.0,  # y
            math.sin(half_yaw),  # z
        )

        # Velocity is supplied in 0.1 KPH units.
        speed_mps = float(velocity_raw) * 0.1 * (1000.0 / 3600.0)

        if speed_mps == 0.0 and self._last_time_usec and self._last_position:
            # Derive a fallback speed from the position delta when the encoded
            # speed reports zero. This honours the instruction to combine
            # position and heading when possible.
            dt = (time_usec - self._last_time_usec) / 1_000_000.0
            if dt > 0:
                last_x, last_y, last_z = self._last_position
                dx = x - last_x
                dy = y - last_y
                dz = z - last_z
                distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                speed_mps = distance / dt

        # Resolve the horizontal components using the project coordinate system
        # (+X north, +Y west). The z-component is not provided by the ICD and
        # must remain NaN as requested.
        angular_velocity = (
            speed_mps * math.cos(heading_rad_clockwise),
            -speed_mps * math.sin(heading_rad_clockwise),
            math.nan,
        )

        self._last_time_usec = time_usec
        self._last_position = position

        return format_unified_row(
            time_usec=time_usec,
            position=position,
            orientation_wxyz=orientation,
            angular_velocity=angular_velocity,
        )


__all__ = [
    "UNIFIED_CSV_HEADER",
    "format_drone_csv_row",
    "format_unified_row",
    "RoverFeedbackCsvFormatter",
]
