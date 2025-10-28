"""ICD constants and helpers for the Image Stream module."""

from __future__ import annotations

import struct
from typing import Dict, Optional

from .bridge_tcp import pack_bridge_tcp_frame


UDP_HEADER_FORMAT = "<IBBHIIIIIBB"
UDP_HEADER_FIELDS = [
    "header_version",
    "msg_type",
    "protocol_type",
    "send_count",
    "msg_frames",
    "frame_size",
    "frame_pos",
    "frame_index",
    "msg_size",
    "time_sec",
    "time_nano_sec",
]
UDP_HEADER_SIZE = struct.calcsize(UDP_HEADER_FORMAT)

CMD_REQ_CAPTURE = 0x01
CMD_SET_GIMBAL = 0x02  # Reserved
CMD_SET_COUNT = 0x03
CMD_GET_IMG_NUM = 0x04
CMD_REQ_SEND_IMG = 0x05
CMD_SET_ZOOM_LENS_DIST = 0x06
CMD_GET_ZOOM_LENS_DIST = 0x07

CMD_IMG_NUM_RESPONSE = 0x11
CMD_FILE_IMG_TRANSFER = 0x12
CMD_ACK_ZOOM_LENS_DIST = 0x13


COMMAND_NAMES: Dict[int, str] = {
    CMD_REQ_CAPTURE: "Req_Capture",
    CMD_SET_GIMBAL: "Set_Gimbal",
    CMD_SET_COUNT: "Set_Count",
    CMD_GET_IMG_NUM: "Get_Img_Num",
    CMD_REQ_SEND_IMG: "Req_Send_Img",
    CMD_SET_ZOOM_LENS_DIST: "Set_ZoomLensDist",
    CMD_GET_ZOOM_LENS_DIST: "Get_ZoomLensDist",
    CMD_IMG_NUM_RESPONSE: "Img_Num_Response",
    CMD_FILE_IMG_TRANSFER: "File_Img_Transfer",
    CMD_ACK_ZOOM_LENS_DIST: "Ack_ZoomLensDist",
}


def parse_udp_header(data: bytes) -> Optional[Dict[str, int]]:
    """Decode the 30-byte UDP header emitted by the image generator."""

    if len(data) < UDP_HEADER_SIZE:
        return None
    try:
        values = struct.unpack(UDP_HEADER_FORMAT, data[:UDP_HEADER_SIZE])
    except struct.error:
        return None
    return {key: int(value) for key, value in zip(UDP_HEADER_FIELDS, values)}


def pack_tcp_response(
    cmd_id: int,
    payload: bytes,
    *,
    ts_sec: int,
    ts_nsec: int,
) -> bytes:
    """Helper to produce the TCP frame used by the bridge camera service."""

    return pack_bridge_tcp_frame(cmd_id, payload, ts_sec=ts_sec, ts_nsec=ts_nsec)


__all__ = [
    "UDP_HEADER_FORMAT",
    "UDP_HEADER_FIELDS",
    "UDP_HEADER_SIZE",
    "CMD_REQ_CAPTURE",
    "CMD_SET_GIMBAL",
    "CMD_SET_COUNT",
    "CMD_GET_IMG_NUM",
    "CMD_REQ_SEND_IMG",
    "CMD_SET_ZOOM_LENS_DIST",
    "CMD_GET_ZOOM_LENS_DIST",
    "CMD_IMG_NUM_RESPONSE",
    "CMD_FILE_IMG_TRANSFER",
    "CMD_ACK_ZOOM_LENS_DIST",
    "COMMAND_NAMES",
    "parse_udp_header",
    "pack_tcp_response",
]
