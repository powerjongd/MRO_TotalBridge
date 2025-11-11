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
# ❌ RPY 리맵핑 함수 임포트 제거
# from unified_bridge.support.helpers import remap_input_rpy

_GIMBAL_CTRL_HEADER_FMT = "<BiIB"
_GIMBAL_CTRL_PAYLOAD_FMT = "<BB3d4f"
_POWER_CTRL_HEADER_FMT = "<BiIB"
_POWER_CTRL_PAYLOAD_FMT = "<BBB"
_STATUS_PAYLOAD_FMT = "<hh3d3f3ff"
_SET_TARGET_FMT = "<hh3d3f"


@dataclass
class SetTargetPayload:
    """Decoded payload for :data:`TCP_CMD_SET_TARGET`.

    The underlying TCP command still transmits angles in the legacy
    roll-pitch-yaw order for backward compatibility.  The ``sim_rpy`` tuple
    normalizes those angles into the Unreal ``FRotator`` ordering of
    (Pitch, Yaw, Roll) so downstream code can reason about the simulator
    convention without worrying about the on-wire layout.
    """
    sensor_type: int
    sensor_id: int
    position_xyz: Tuple[float, float, float]
    sim_rpy: Tuple[float, float, float]


@dataclass
class SetZoomPayload:
    zoom_scale: float


@dataclass
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

    The quaternion should be supplied in canonical ``(x, y, z, w)`` order as
    produced by :func:`unified_bridge.support.helpers.euler_to_quat`.  The values are forwarded
    without axis remapping so downstream consumers interpret them using the
    standard roll/pitch/yaw convention.
    """

    qx, qy, qz, qw = quat_xyzw
    payload = struct.pack(
        _GIMBAL_CTRL_PAYLOAD_FMT,
        sensor_type & 0xFF,
        sensor_id & 0xFF,
        float(position_xyz[0]),
        float(position_xyz[1]),
        float(position_xyz[2]),
        float(qx),
        float(qy),
        float(qz),
        float(qw),
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
        # ✅ TCP 패킷은 (R, P, Y) 순서로 값을 전송 (roll_sim, pitch_sim, yaw_sim)
        sensor_type, sensor_id, px, py, pz, roll_sim, pitch_sim, yaw_sim = struct.unpack(
            _SET_TARGET_FMT, command.body[: struct.calcsize(_SET_TARGET_FMT)]
        )
    except struct.error:
        return None
    
    # ❌ remap_input_rpy 호출 제거
    # sim_pitch, sim_yaw, sim_roll = remap_input_rpy(
    #     float(roll_sim),
    #     float(pitch_sim),
    #     float(yaw_sim),
    # )
    
    # ✅ (R, P, Y) 값을 (P, Y, R) 순서로 struct에 맞게 단순 할당
    sim_pitch = float(pitch_sim)
    sim_yaw = float(yaw_sim)
    sim_roll = float(roll_sim)
    
    return SetTargetPayload(
        sensor_type=int(sensor_type),
        sensor_id=int(sensor_id),
        position_xyz=(float(px), float(py), float(pz)),
        sim_rpy=(sim_pitch, sim_yaw, sim_roll), # (P, Y, R) 튜플
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