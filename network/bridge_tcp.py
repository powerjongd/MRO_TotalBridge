"""Bridge-wide TCP framing helpers.

The gimbal control server and the image stream bridge both use the same
TCP transport layout: a 4-byte little-endian length prefix followed by a
command envelope ``<IIB>`` (seconds, nanoseconds, command id) and the
command-specific payload.  Centralising the helpers keeps the struct
layouts documented in one place so that protocol tweaks only require a
single update.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Optional

TCP_LENGTH_FMT = "<I"
TCP_ENVELOPE_FMT = "<IIB"
TCP_ENVELOPE_SIZE = struct.calcsize(TCP_ENVELOPE_FMT)


@dataclass(slots=True)
class BridgeTcpCommand:
    """Parsed representation of a bridge TCP command frame."""

    ts_sec: int
    ts_nsec: int
    cmd_id: int
    body: bytes


def parse_bridge_tcp_command(data: bytes) -> Optional[BridgeTcpCommand]:
    """Return a :class:`BridgeTcpCommand` extracted from *data*.

    ``None`` is returned when the payload is shorter than the envelope or
    when unpacking fails.  The command body is returned as a ``bytes``
    view without further interpretation so that the caller can keep the
    higher level parsing in its own module.
    """

    if len(data) < TCP_ENVELOPE_SIZE:
        return None
    try:
        ts_sec, ts_nsec, cmd_id = struct.unpack(TCP_ENVELOPE_FMT, data[:TCP_ENVELOPE_SIZE])
    except struct.error:
        return None
    body = data[TCP_ENVELOPE_SIZE:]
    return BridgeTcpCommand(ts_sec=int(ts_sec), ts_nsec=int(ts_nsec), cmd_id=int(cmd_id), body=bytes(body))


def pack_bridge_tcp_frame(
    cmd_id: int,
    payload: bytes,
    *,
    ts_sec: Optional[int] = None,
    ts_nsec: Optional[int] = None,
) -> bytes:
    """Pack a TCP frame for the bridge command transport.

    ``ts_sec``/``ts_nsec`` default to ``time.time_ns()`` when omitted so
    that callers that do not care about the exact timestamp can simply
    provide the command id and payload.  The return value contains both
    the 4-byte length prefix and the envelope+payload bytes so it is
    ready to be written to a socket.
    """

    if ts_sec is None or ts_nsec is None:
        now_ns = time.time_ns()
        ts_sec = now_ns // 1_000_000_000
        ts_nsec = now_ns % 1_000_000_000
    envelope = struct.pack(TCP_ENVELOPE_FMT, int(ts_sec), int(ts_nsec), int(cmd_id)) + payload
    header = struct.pack(TCP_LENGTH_FMT, len(envelope))
    return header + envelope


__all__ = [
    "BridgeTcpCommand",
    "TCP_LENGTH_FMT",
    "TCP_ENVELOPE_FMT",
    "TCP_ENVELOPE_SIZE",
    "parse_bridge_tcp_command",
    "pack_bridge_tcp_frame",
]
