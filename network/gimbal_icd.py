"""ICD constants for the Gimbal control module."""

from __future__ import annotations

from typing import List, Tuple

TYPE_GIMBAL_CTRL = 10706
TYPE_POWER_CTRL = 10707

MC_HB_TYPE = 18
GIMBAL_STATUS_FLAGS = 2

TCP_CMD_SET_TARGET = 0x01
TCP_CMD_SET_ZOOM = 0x02
TCP_CMD_GET_STATUS = 0x80
TCP_CMD_STATUS = 0x81

PARAM_TABLE: List[Tuple[str, float, int]] = [
    ("VERSION_X", 7, 9), ("SRL_NUMBER", 100, 9), ("STIFF_TILT", 50, 9),
    ("RC_CHAN_STILT", 2, 9), ("STIFF_ROLL", 50, 9), ("STIFF_PAN", 50, 9),
    ("FILTER_OUT", 1, 9), ("RC_CHAN_SPAN", 2, 9), ("PWR_TILT", 30, 9),
    ("PWR_ROLL", 30, 9), ("PWR_PAN", 30, 9), ("FLW_SP_TILT", 50, 9),
    ("GMB_HOME_PAN", 900, 9), ("FLW_SP_PAN", 80, 9), ("FLW_LPF_TILT", 50, 9),
    ("MAPPING_ANGLE", 0, 9), ("FLW_LPF_PAN", 50, 9), ("GYRO_TRUST", 100, 9),
    ("RC_DZONE_TILT", 40, 9), ("RC_DZONE_ROLL", 40, 9), ("RC_DZONE_PAN", 40, 9),
    ("RC_TYPE", 15, 9), ("GYRO_LPF", 5, 9), ("RC_LIM_MIN_TILT", -90, 9),
    ("RC_LIM_MAX_TILT", 90, 9), ("RC_LIM_MIN_ROLL", -20, 9),
    ("RC_LIM_MAX_ROLL", 20, 9), ("RC_LPF_TILT", 20, 9), ("RC_LPF_ROLL", 20, 9),
    ("RC_LPF_PAN", 20, 9), ("RC_CHAN_TILT", 1, 9), ("RC_CHAN_ROLL", 7, 9),
    ("RC_CHAN_PAN", 0, 9), ("RC_CHAN_MODE", 6, 9), ("RC_TRIM_TILT", 0, 9),
    ("RC_TRIM_ROLL", 0, 9), ("RC_TRIM_PAN", 0, 9), ("RC_MODE_TILT", 1, 9),
    ("RC_MODE_ROLL", 0, 9), ("RC_MODE_PAN", 1, 9), ("FLW_WD_TILT", 10, 9),
    ("FLW_WD_PAN", 10, 9), ("RC_SPD_TILT", 50, 9), ("RC_SPD_ROLL", 50, 9),
    ("RC_SPD_PAN", 50, 9), ("RC_REVERSE_AXIS", 0, 9), ("VERSION_Y", 8, 9),
    ("VERSION_Z", 3, 9), ("RC_LIM_MIN_PAN", -45, 9), ("RC_LIM_MAX_PAN", 45, 9),
    ("MAV_EMIT_HB", 1, 9), ("MAV_RATE_ST", 1, 9), ("MAV_RATE_ENCCNT", 10, 9),
    ("MAV_TS_ENCNT", 1, 9), ("MAV_RATE_ORIEN", 10, 9), ("MAV_RATE_IMU", 15, 9),
    ("BAUDRATE_COM2", 1152, 9), ("BAUDRATE_COM4", 0, 9), ("GIMBAL_COMPID", 154, 9),
    ("TILT_DAMPING", 20, 9), ("ROLL_DAMPING", 15, 9), ("PAN_DAMPING", 20, 9),
]

PARAM_COUNT = len(PARAM_TABLE)

__all__ = [
    "TYPE_GIMBAL_CTRL",
    "TYPE_POWER_CTRL",
    "MC_HB_TYPE",
    "GIMBAL_STATUS_FLAGS",
    "TCP_CMD_SET_TARGET",
    "TCP_CMD_SET_ZOOM",
    "TCP_CMD_GET_STATUS",
    "TCP_CMD_STATUS",
    "PARAM_TABLE",
    "PARAM_COUNT",
]
