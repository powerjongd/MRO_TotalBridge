"""Helpers for serialising and parsing gimbal control payloads."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional, Tuple

from .bridge_tcp import BridgeTcpCommand, pack_bridge_tcp_frame
from .gimbal_icd import (
    TYPE_GIMBAL_CTRL,
    TYPE_POWER_CTRL,
    TCP_CMD_SET_TARGET,
    TCP_CMD_SET_ZOOM,
    TCP_CMD_STATUS,
)

_GIMBAL_CTRL_HEADER_FMT = "<BiIB"
_GIMBAL_CTRL_PAYLOAD_FMT = "<BB3d4f"
_POWER_CTRL_HEADER_FMT = "<BiIB"
_POWER_CTRL_PAYLOAD_FMT = "<BBB"
_STATUS_PAYLOAD_FMT = "<hh3d3f3ff"
_SET_TARGET_FMT = "<hh3d3f"


@dataclass(slots=True)
class SetTargetPayload:
    sensor_type: int
    sensor_id: int
    position_xyz: Tuple[float, float, float]
    sim_rpy: Tuple[float, float, float]


@dataclass(slots=True)
class SetZoomPayload:
    zoom_scale: float


@dataclass(slots=True)
class StatusSnapshot:
    sensor_type: int
    sensor_id: int
    position_xyz: Tuple[float, float, float]
    sim_rpy_current: Tuple[float, float, float]
    sim_rpy_target: Tuple[float, float, float]
    zoom_scale: float
    max_rate_dps: float


def build_gimbal_ctrl_packet(
    sensor_type: int,
    sensor_id: int,
    position_xyz: Tuple[float, float, float],
    quat_xyzw: Tuple[float, float, float, float],
) -> bytes:
    """Pack the UDP control packet consumed by the ImageGenerator.

    The quaternion must be supplied in ``(x, y, z, w)`` order to match the
    :func:`utils.helpers.euler_to_quat` helper used by the bridge.
    """

    payload = struct.pack(
        _GIMBAL_CTRL_PAYLOAD_FMT,
        sensor_type & 0xFF,
        sensor_id & 0xFF,
        float(position_xyz[0]),
        float(position_xyz[1]),
        float(position_xyz[2]),
        float(quat_xyzw[0]),
        float(quat_xyzw[1]),
        float(quat_xyzw[2]),
        float(quat_xyzw[3]),
    )
    header = struct.pack(_GIMBAL_CTRL_HEADER_FMT, 1, TYPE_GIMBAL_CTRL, len(payload), 0)
    return header + payload


def build_power_ctrl_packet(sensor_type: int, sensor_id: int, power_on: int) -> bytes:
    payload = struct.pack(
        _POWER_CTRL_PAYLOAD_FMT,
        sensor_type & 0xFF,
        sensor_id & 0xFF,
        power_on & 0xFF,
    )
    header = struct.pack(_POWER_CTRL_HEADER_FMT, 1, TYPE_POWER_CTRL, len(payload), 0)
    return header + payload


def build_status_frame(snapshot: StatusSnapshot, *, ts_sec: int, ts_nsec: int) -> bytes:
    payload = struct.pack(
        _STATUS_PAYLOAD_FMT,
        int(snapshot.sensor_type),
        int(snapshot.sensor_id),
        float(snapshot.position_xyz[0]),
        float(snapshot.position_xyz[1]),
        float(snapshot.position_xyz[2]),
        float(snapshot.sim_rpy_current[0]),
        float(snapshot.sim_rpy_current[1]),
        float(snapshot.sim_rpy_current[2]),
        float(snapshot.sim_rpy_target[0]),
        float(snapshot.sim_rpy_target[1]),
        float(snapshot.sim_rpy_target[2]),
        float(snapshot.zoom_scale),
        float(snapshot.max_rate_dps),
    )
    return pack_bridge_tcp_frame(TCP_CMD_STATUS, payload, ts_sec=ts_sec, ts_nsec=ts_nsec)


def parse_set_target(command: BridgeTcpCommand) -> Optional[SetTargetPayload]:
    if command.cmd_id != TCP_CMD_SET_TARGET or len(command.body) < struct.calcsize(_SET_TARGET_FMT):
        return None
    try:
        sensor_type, sensor_id, px, py, pz, r_sim, p_sim, y_sim = struct.unpack(
            _SET_TARGET_FMT, command.body[: struct.calcsize(_SET_TARGET_FMT)]
        )
    except struct.error:
        return None
    return SetTargetPayload(
        sensor_type=int(sensor_type),
        sensor_id=int(sensor_id),
        position_xyz=(float(px), float(py), float(pz)),
        sim_rpy=(float(r_sim), float(p_sim), float(y_sim)),
    )


def parse_set_zoom(command: BridgeTcpCommand) -> Optional[SetZoomPayload]:
    if command.cmd_id != TCP_CMD_SET_ZOOM or len(command.body) < 4:
        return None
    try:
        (zoom_val,) = struct.unpack("<f", command.body[:4])
    except struct.error:
        return None
    return SetZoomPayload(zoom_scale=float(zoom_val))


__all__ = [
    "SetTargetPayload",
    "SetZoomPayload",
    "StatusSnapshot",
    "build_gimbal_ctrl_packet",
    "build_power_ctrl_packet",
    "build_status_frame",
    "parse_set_target",
    "parse_set_zoom",
]
