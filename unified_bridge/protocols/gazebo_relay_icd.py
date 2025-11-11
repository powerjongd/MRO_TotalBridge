"""ICD constants for the Gazebo relay module.

The Gazebo ghost-mode payload follows Gazebo's quaternion convention of
``(w, x, y, z)``. This intentionally differs from the Unreal-oriented
``(x, y, z, w)`` order used by the gimbal control module.
"""

from __future__ import annotations

import struct

GAZEBO_HEADER_FMT = "<BiIB"
GAZEBO_HEADER_SIZE = struct.calcsize(GAZEBO_HEADER_FMT)

GAZEBO_PAYLOAD_FMT = "<IQdddffffffffff"
GAZEBO_PAYLOAD_SIZE = struct.calcsize(GAZEBO_PAYLOAD_FMT)

GAZEBO_PAYLOAD_NO_HEADER_FMT = "<Qdddffffffffff"
GAZEBO_PAYLOAD_NO_HEADER_SIZE = struct.calcsize(GAZEBO_PAYLOAD_NO_HEADER_FMT)

GAZEBO_STRUCT_FMT = "<BiIBIQdddffffffffff"
GAZEBO_STRUCT_SIZE = struct.calcsize(GAZEBO_STRUCT_FMT)

DIST_MSG_TYPE = 10708
DIST_HDR_FMT = "<BiIB"
DIST_HDR_SIZE = struct.calcsize(DIST_HDR_FMT)
DIST_PAYLOAD_FMT = "<I H H H B B B B f f f f f f B"
DIST_PAYLOAD_SIZE = struct.calcsize(DIST_PAYLOAD_FMT)
DIST_TOTAL_MIN = DIST_HDR_SIZE + DIST_PAYLOAD_SIZE

__all__ = [
    "GAZEBO_HEADER_FMT",
    "GAZEBO_HEADER_SIZE",
    "GAZEBO_PAYLOAD_FMT",
    "GAZEBO_PAYLOAD_SIZE",
    "GAZEBO_PAYLOAD_NO_HEADER_FMT",
    "GAZEBO_PAYLOAD_NO_HEADER_SIZE",
    "GAZEBO_STRUCT_FMT",
    "GAZEBO_STRUCT_SIZE",
    "DIST_MSG_TYPE",
    "DIST_HDR_FMT",
    "DIST_HDR_SIZE",
    "DIST_PAYLOAD_FMT",
    "DIST_PAYLOAD_SIZE",
    "DIST_TOTAL_MIN",
]
