"""ICD constants for the Image Stream module."""

from __future__ import annotations

from typing import Dict

CMD_REQ_CAPTURE = 0x01
CMD_SET_GIMBAL = 0x02  # Reserved
CMD_SET_COUNT = 0x03
CMD_GET_IMG_NUM = 0x04
CMD_REQ_SEND_IMG = 0x05
CMD_SET_ZOOM_RATIO = 0x06
CMD_GET_ZOOM_RATIO = 0x07

CMD_IMG_NUM_RESPONSE = 0x11
CMD_FILE_IMG_TRANSFER = 0x12
CMD_ACK_ZOOM_RATIO = 0x13


COMMAND_NAMES: Dict[int, str] = {
    CMD_REQ_CAPTURE: "Req_Capture",
    CMD_SET_GIMBAL: "Set_Gimbal",
    CMD_SET_COUNT: "Set_Count",
    CMD_GET_IMG_NUM: "Get_Img_Num",
    CMD_REQ_SEND_IMG: "Req_Send_Img",
    CMD_SET_ZOOM_RATIO: "Set_Zoomratio",
    CMD_GET_ZOOM_RATIO: "Get_Zoomratio",
    CMD_IMG_NUM_RESPONSE: "Img_Num_Response",
    CMD_FILE_IMG_TRANSFER: "File_Img_Transfer",
    CMD_ACK_ZOOM_RATIO: "Ack_Zoomratio",
}


__all__ = [
    "CMD_REQ_CAPTURE",
    "CMD_SET_GIMBAL",
    "CMD_SET_COUNT",
    "CMD_GET_IMG_NUM",
    "CMD_REQ_SEND_IMG",
    "CMD_SET_ZOOM_RATIO",
    "CMD_GET_ZOOM_RATIO",
    "CMD_IMG_NUM_RESPONSE",
    "CMD_FILE_IMG_TRANSFER",
    "CMD_ACK_ZOOM_RATIO",
    "COMMAND_NAMES",
]
