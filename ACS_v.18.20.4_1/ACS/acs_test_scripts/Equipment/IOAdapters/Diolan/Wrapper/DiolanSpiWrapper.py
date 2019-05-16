from ctypes import c_uint8, c_uint16, c_uint32, byref, create_string_buffer, Structure

from DiolanDef import DLN_MSG_HEADER


DLN_PIN_ROLE_SPI_SLAVE_MISO = 1
DLN_PIN_ROLE_SPI_SLAVE_MOSI = 0
DLN_PIN_ROLE_SPI_SLAVE_SCLK = 0
DLN_PIN_ROLE_SPI_SLAVE_SS = 0


def DLN_MSG_ID_SPI_SLAVE_GET_PORT_COUNT(): DLN_BUILD_MSG_ID(0x00, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_ENABLE(): DLN_BUILD_MSG_ID(0x01, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_DISABLE(): DLN_BUILD_MSG_ID(0x02, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_IS_ENABLED(): DLN_BUILD_MSG_ID(0x03, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_SET_MODE(): DLN_BUILD_MSG_ID(0x04, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_MODE(): DLN_BUILD_MSG_ID(0x05, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_SET_FRAME_SIZE(): DLN_BUILD_MSG_ID(0x06, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_FRAME_SIZE(): DLN_BUILD_MSG_ID(0x07, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_LOAD_REPLY(): DLN_BUILD_MSG_ID(0x08, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_SET_REPLY_SHORTAGE_ACTION(): DLN_BUILD_MSG_ID(0x0B, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_REPLY_SHORTAGE_ACTION(): DLN_BUILD_MSG_ID(0x0C, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_ENQUEUE_REPLY(): DLN_BUILD_MSG_ID(0x0D, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_SET_REPLY_MODE(): DLN_BUILD_MSG_ID(0x0E, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_REPLY_MODE(): DLN_BUILD_MSG_ID(0x0F, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_DATA_RECEIVED_EV(): DLN_BUILD_MSG_ID(0x10, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_TRANSFER_EV(): DLN_MSG_ID_SPI_SLAVE_DATA_RECEIVED_EV()


def DLN_MSG_ID_SPI_SLAVE_ENABLE_EVENT(): DLN_BUILD_MSG_ID(0x11, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_DISABLE_EVENT(): DLN_BUILD_MSG_ID(0x12, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_IS_EVENT_ENABLED(): DLN_BUILD_MSG_ID(0x13, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_SET_EVENT_SIZE(): DLN_BUILD_MSG_ID(0x14, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_EVENT_SIZE(): DLN_BUILD_MSG_ID(0x15, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_SET_CPOL(): DLN_BUILD_MSG_ID(0x16, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_CPOL(): DLN_BUILD_MSG_ID(0x17, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_SET_CPHA(): DLN_BUILD_MSG_ID(0x18, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_CPHA(): DLN_BUILD_MSG_ID(0x19, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_ENABLE_SS_RISE_EVENT(): DLN_BUILD_MSG_ID(0x1A, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_DISABLE_SS_RISE_EVENT(): DLN_BUILD_MSG_ID(0x1B, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_IS_SS_RISE_EVENT_ENABLED(): DLN_BUILD_MSG_ID(0x1C, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_SET_SS_IDLE_TIMEOUT(): DLN_BUILD_MSG_ID(0x1D, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_SS_IDLE_TIMEOUT(): DLN_BUILD_MSG_ID(0x1E, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_SUPPORTED_MODES(): DLN_BUILD_MSG_ID(0x40, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_SUPPORTED_FRAME_SIZES(): DLN_BUILD_MSG_ID(0x41, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_SUPPORTED_CPOL_VALUES(): DLN_BUILD_MSG_ID(0x42, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_SUPPORTED_CPHA_VALUES(): DLN_BUILD_MSG_ID(0x43, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_SUPPORTED_SHORTAGE_ACTIONS(): DLN_BUILD_MSG_ID(0x44, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_SUPPORTED_EVENT_TYPES(): DLN_BUILD_MSG_ID(0x45, DLN_MODULE_SPI_SLAVE)


def DLN_MSG_ID_SPI_SLAVE_GET_SUPPORTED_REPLY_MODES(): DLN_BUILD_MSG_ID(0x46, DLN_MODULE_SPI_SLAVE)


DLN_SPI_SLAVE_BUFFER_SIZE = 256

DLN_SPI_SLAVE_ENABLED = 1
DLN_SPI_SLAVE_DISABLED = 0
DLN_SPI_SLAVE_CANCEL_TRANSFERS = 0
DLN_SPI_SLAVE_WAIT_FOR_TRANSFERS = 1

DLN_SPI_SLAVE_CPHA_BIT = (1 << 0)
DLN_SPI_SLAVE_CPHA_0 = (0 << 0)
DLN_SPI_SLAVE_CPHA_1 = (1 << 0)
DLN_SPI_SLAVE_CPOL_BIT = (1 << 1)
DLN_SPI_SLAVE_CPOL_0 = (0 << 1)
DLN_SPI_SLAVE_CPOL_1 = (1 << 1)

DLN_SPI_SLAVE_FRAME_SIZE_8 = 8
DLN_SPI_SLAVE_FRAME_SIZE_9 = 9
DLN_SPI_SLAVE_FRAME_SIZE_10 = 10
DLN_SPI_SLAVE_FRAME_SIZE_11 = 11
DLN_SPI_SLAVE_FRAME_SIZE_12 = 12
DLN_SPI_SLAVE_FRAME_SIZE_13 = 13
DLN_SPI_SLAVE_FRAME_SIZE_14 = 14
DLN_SPI_SLAVE_FRAME_SIZE_15 = 15
DLN_SPI_SLAVE_FRAME_SIZE_16 = 16

DLN_SPI_SLAVE_EVENT_ENABLED = 1
DLN_SPI_SLAVE_EVENT_DISABLED = 0

DLN_SPI_SLAVE_EVENT_NONE = 0
DLN_SPI_SLAVE_EVENT_SS_RISE = 1
DLN_SPI_SLAVE_EVENT_BUFFER_FULL = 2

DLN_SPI_SLAVE_REPLY_SHORTAGE_SEND_ZEROES = 0
DLN_SPI_SLAVE_REPLY_SHORTAGE_REUSE = 1

DLN_SPI_SLAVE_REPLY_MODE_OFF = 0
DLN_SPI_SLAVE_REPLY_MODE_SS_BASED = 1
DLN_SPI_SLAVE_REPLY_MODE_COUNT_BASED = 2

DLN_SPI_SLAVE_SS_IDLE_TIMEOUT_MIN = 1
DLN_SPI_SLAVE_SS_IDLE_TIMEOUT_MAX = 1000


class DLN_SPI_SLAVE_GET_PORT_COUNT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_SPI_SLAVE_GET_PORT_COUNT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("count", c_uint8)]


class DLN_SPI_SLAVE_ENABLE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_ENABLE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("conflict", c_uint16)]


class DLN_SPI_SLAVE_DISABLE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("waitForTransferCompletion", c_uint8)]


class DLN_SPI_SLAVE_DISABLE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_IS_ENABLED_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_IS_ENABLED_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("enabled", c_uint8)]


class DLN_SPI_SLAVE_SET_MODE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("mode", c_uint8)]


class DLN_SPI_SLAVE_SET_MODE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_GET_MODE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_MODE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("mode", c_uint8)]


class DLN_SPI_SLAVE_SET_FRAME_SIZE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("frameSize", c_uint8)]


class DLN_SPI_SLAVE_SET_FRAME_SIZE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_GET_FRAME_SIZE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_FRAME_SIZE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("frameSize", c_uint8)]


class DLN_SPI_SLAVE_ENQUEUE_REPLY_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("size", c_uint16),
                ("attribute", c_uint8),  # reserved
                ("buffer", c_uint8 * DLN_SPI_SLAVE_BUFFER_SIZE)]


class DLN_SPI_SLAVE_ENQUEUE_REPLY_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


DLN_SPI_SLAVE_LOAD_REPLY_CMD = DLN_SPI_SLAVE_ENQUEUE_REPLY_CMD
DLN_SPI_SLAVE_LOAD_REPLY_RSP = DLN_SPI_SLAVE_ENQUEUE_REPLY_RSP


class DLN_SPI_SLAVE_SET_REPLY_SHORTAGE_ACTION_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("action", c_uint8)]


class DLN_SPI_SLAVE_SET_REPLY_SHORTAGE_ACTION_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_GET_REPLY_SHORTAGE_ACTION_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_REPLY_SHORTAGE_ACTION_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("action", c_uint8)]


class DLN_SPI_SLAVE_SET_REPLY_MODE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("replyMode", c_uint8)]


class DLN_SPI_SLAVE_SET_REPLY_MODE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_GET_REPLY_MODE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_REPLY_MODE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("replyMode", c_uint8)]


class DLN_SPI_SLAVE_DATA_RECEIVED_EV(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("eventCount", c_uint16),
                ("eventType", c_uint8),
                ("port", c_uint8),
                ("size", c_uint16),
                ("buffer", c_uint8 * DLN_SPI_SLAVE_BUFFER_SIZE)]


# define DLN_SPI_SLAVE_TRANSFER_EV DLN_SPI_SLAVE_DATA_RECEIVED_EV


class DLN_SPI_SLAVE_ENABLE_EVENT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_ENABLE_EVENT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_DISABLE_EVENT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_DISABLE_EVENT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_IS_EVENT_ENABLED_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_IS_EVENT_ENABLED_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("enabled", c_uint8)]


class DLN_SPI_SLAVE_SET_EVENT_SIZE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("size", c_uint16)]


class DLN_SPI_SLAVE_SET_EVENT_SIZE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_GET_EVENT_SIZE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_EVENT_SIZE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("size", c_uint16)]


class DLN_SPI_SLAVE_SET_CPOL_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("cpol", c_uint8)]


class DLN_SPI_SLAVE_SET_CPOL_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_GET_CPOL_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_CPOL_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("cpol", c_uint8)]


class DLN_SPI_SLAVE_SET_CPHA_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("cpha", c_uint8)]


class DLN_SPI_SLAVE_SET_CPHA_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_GET_CPHA_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_CPHA_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("cpha", c_uint8)]


class DLN_SPI_SLAVE_ENABLE_SS_RISE_EVENT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_ENABLE_SS_RISE_EVENT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_DISABLE_SS_RISE_EVENT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_DISABLE_SS_RISE_EVENT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_IS_SS_RISE_EVENT_ENABLED_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_IS_SS_RISE_EVENT_ENABLED_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("enabled", c_uint8)]


class DLN_SPI_SLAVE_SET_SS_IDLE_TIMEOUT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("timeout", c_uint32)]


class DLN_SPI_SLAVE_SET_SS_IDLE_TIMEOUT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_SPI_SLAVE_GET_SS_IDLE_TIMEOUT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_SS_IDLE_TIMEOUT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("timeout", c_uint32)]


class DLN_SPI_SLAVE_MODE_VALUES(Structure):
    _pack_ = 1
    _fields_ = [("count", c_uint8),
                ("values", c_uint8 * 4)]


class DLN_SPI_SLAVE_GET_SUPPORTED_MODES_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_SUPPORTED_MODES_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("values", DLN_SPI_SLAVE_MODE_VALUES)]


class DLN_SPI_SLAVE_CPOL_VALUES(Structure):
    _pack_ = 1
    _fields_ = [("count", c_uint8),
                ("values", c_uint8 * 2)]


class DLN_SPI_SLAVE_GET_SUPPORTED_CPOL_VALUES_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_SUPPORTED_CPOL_VALUES_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("values", DLN_SPI_SLAVE_CPOL_VALUES)]


class DLN_SPI_SLAVE_CPHA_VALUES(Structure):
    _pack_ = 1
    _fields_ = [("count", c_uint8),
                ("values", c_uint8 * 2)]


class DLN_SPI_SLAVE_GET_SUPPORTED_CPHA_VALUES_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_SUPPORTED_CPHA_VALUES_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("values", DLN_SPI_SLAVE_CPHA_VALUES)]


class DLN_SPI_SLAVE_FRAME_SIZES(Structure):
    _pack_ = 1
    _fields_ = [("count", c_uint8),
                ("frameSizes", c_uint8 * 36)]


class DLN_SPI_SLAVE_GET_SUPPORTED_FRAME_SIZES_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_SUPPORTED_FRAME_SIZES_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("supportedSizes", DLN_SPI_SLAVE_FRAME_SIZES)]


class DLN_SPI_SLAVE_SHORTAGE_ACTIONS(Structure):
    _pack_ = 1
    _fields_ = [("count", c_uint8),
                ("actions", c_uint8 * 10)]


class DLN_SPI_SLAVE_GET_SUPPORTED_SHORTAGE_ACTIONS_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_SUPPORTED_SHORTAGE_ACTIONS_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("supportedActions", DLN_SPI_SLAVE_SHORTAGE_ACTIONS)]


class DLN_SPI_SLAVE_EVENT_TYPES(Structure):
    _pack_ = 1
    _fields_ = [("count", c_uint8),
                ("eventTypes", c_uint8 * 8)]


class DLN_SPI_SLAVE_GET_SUPPORTED_EVENT_TYPES_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_SUPPORTED_EVENT_TYPES_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("supportedEventTypes", DLN_SPI_SLAVE_EVENT_TYPES)]


class DLN_SPI_SLAVE_REPLY_MODES(Structure):
    _pack_ = 1
    _fields_ = [("count", c_uint8),
                ("replyModes", c_uint8 * 8)]


class DLN_SPI_SLAVE_GET_SUPPORTED_REPLY_MODES_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_SPI_SLAVE_GET_SUPPORTED_REPLY_MODES_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("supportedReplyModes", DLN_SPI_SLAVE_REPLY_MODES)]


class DiolanSpiSlaveWrapper():
    """

    """

    def __init__(self, dll_api):
        self.__full_payload = "00"
        self.__dll_api = dll_api

    def dln_spislave_get_port_count(self, handle):
        """
        :return DLN_RES_SUCCESS - the port count has been successfully retrieved.
        """
        count = c_uint8()
        result = self.__dll_api.DlnSpiSlaveGetPortCount(handle, byref(count))
        return result, count.value

    def dln_spislave_enable(self, handle, port):
        """
        :return DLN_RES_SUCCESS - the SPI slave module has been successfully enabled
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        :return DLN_RES_PIN_IN_USE - the SPI pins are assigned to another module of the adapter and cannot be enabled as SPI.
        """
        conflict = c_uint8()
        result = self.__dll_api.DlnSpiSlaveEnable(handle, c_uint8(port), byref(conflict))
        return result, conflict

    def dln_spislave_disable(self, handle, port, waitForTransferCompletion):
        """
        :return DLN_RES_SUCCESS - the SPI slave module has been successfully disabled
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        :return DLN_RES_TRANSFER_CANCELLED - the pending transfers were cancelled.
        """
        result = self.__dll_api.DlnSpiSlaveDisable(handle, port, waitForTransferCompletion)
        return result

    def dln_spislave_is_enabled(self, handle, port):
        """
        :return DLN_RES_SUCCESS - the SPI slave state has been successfully retrieved
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range.
        """
        enabled = c_uint8()
        result = self.__dll_api.DlnSpiSlaveIsEnabled(handle, port, byref(enabled))
        return result, enabled.value

    def dln_spislave_set_mode(self, handle, port, mode):
        """
        :return DLN_RES_SUCCESS - the SPI slave port mode has been successfully set
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        :return DLN_RES_MUST_BE_DISABLED - could not change mode while the SPI slave module is enabled.
        :return DLN_RES_INVALID_MODE - invalid value is provided as mode parameter.
            Use DlnSpiSlaveGetSupportedModes() function to check supported modes for
            current SPI Slave port.
        """
        result = self.__dll_api.DlnSpiSlaveSetMode(handle, port, mode)
        return result

    def dln_spislave_get_mode(self, handle, port):
        """
        :return DLN_RES_SUCCESS - the SPI slave port mode has been successfully retrieved
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range.
        """
        mode = c_uint8()
        result = self.__dll_api.DlnSpiSlaveGetMode(handle, port, byref(mode))
        return result, mode.value

    def dln_spislave_set_framesize(self, handle, port, frameSize):
        """
        :return DLN_RES_SUCCESS - the SPI slave port frame size has been successfully set
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        :return DLN_RES_SPI_INVALID_FRAME_SIZE - the frame size is out of range
        :return DLN_RES_BUSY - could not change frame size while the SPI slave module is enabled.
        """
        result = self.__dll_api.DlnSpiSlaveSetFrameSize(handle, port, frameSize)
        return result

    def dln_spislave_get_framesize(self, handle, port):
        """
        :return DLN_RES_SUCCESS - the SPI slave port frame size has been successfully retrieved.
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range.
        """
        frameSize = c_uint8()
        result = self.__dll_api.DlnSpiSlaveGetFrameSize(handle, port, byref(frameSize))
        return result, frameSize

    def dln_spislave_get_supported_framesizes(self, handle, port):
        """
        :return DLN_RES_SUCCESS - the SPI slave port frame size has been successfully retrieved.
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range.
        """
        supportedFrameSize = DLN_SPI_SLAVE_FRAME_SIZES()
        result = self.__dll_api.DlnSpiSlaveGetSupportedFrameSizes(handle, port, byref(supportedFrameSize))
        count = supportedFrameSize.count
        returnTab = supportedFrameSize.frameSizes[:count]
        return result, returnTab

    def dln_spislave_load_reply(self, handle, port, size, buffer):
        """
        :return DLN_RES_SUCCESS - the SPI slave reply has been successfully loaded
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        :return DLN_RES_BUSY - could not load reply while the SPI slave module is enabled
        :return DLN_RES_INVALID_BUFFER_SIZE - the buffer size is out of range.
        """
        self.__full_payload = buffer
        result = self.__dll_api.DlnSpiSlaveLoadReply(handle, port, size, create_string_buffer(buffer))
        return result

    def dln_spislave_enable_event(self, handle, port):
        result = self.__dll_api.DlnSpiSlaveEnableEvent(handle, port)
        return result

    def dln_spislave_disable_event(self, handle, port):
        result = self.__dll_api.DlnSpiSlaveDisableEvent(handle, port)
        return result

    def dln_spislave_is_event_enabled(self, handle, port):
        enabled = c_uint8()
        result = self.__dll_api.DlnSpiSlaveIsEventEnabled(handle, port, byref(enabled))
        return result, enabled.value

    def dln_spislave_set_event_size(self, handle, port, size):
        result = self.__dll_api.DlnSpiSlaveSetEventSize(handle, port, size)
        return result

    def dln_spislave_get_event_size(self, handle, port):
        event_size = c_uint16()
        result = self.__dll_api.DlnSpiSlaveGetEventSize(handle, port, byref(event_size))
        return result, event_size.value

    def dln_spislave_set_cpol(self, handle, port, cpol):
        """
        :return DLN_RES_SUCCESS - the SPI slave CPOL configuratin values has been successfully set
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        :return DLN_RES_MUST_BE_DISABLED - could not change CPOL value while the SPI slave module is enabled.
        :return DLN_RES_INVALID_CPOL- - invalid value is provided as cpol parameter.
            Use DlnSpiSlaveGetSupportedCpolValues() function to check supported CPOL values for
            current SPI Slave port.
        """
        result = self.__dll_api.DlnSpiSlaveSetCpol(handle, port, cpol)
        return result

    def dln_spislave_get_cpol(self, handle, port):
        """
        :return DLN_RES_SUCCESS
        """
        cpol = c_uint8()
        result = self.__dll_api.DlnSpiSlaveGetCpol(handle, port, byref(cpol))
        return result, cpol.value

    def dln_spislave_set_cpha(self, handle, port, cpha):
        """
        :return DLN_RES_SUCCESS - the SPI slave port mode has been successfully set
        :return DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        :return DLN_RES_MUST_BE_DISABLED - could not change CPHA value while the SPI slave module is enabled.
        :return DLN_RES_INVALID_CPHA - - invalid value is provided as cpha parameter.
            Use DlnSpiSlaveGetSupportedCphaValues() function to check supported CPHA values for
            current SPI Slave port.
        """
        result = self.__dll_api.DlnSpiSlaveSetCpha(handle, port, cpha)
        return result

    def dln_spislave_get_cpha(self, handle, port):
        """
        :return DLN_RES_SUCCESS
        """
        cpha = c_uint8()
        result = self.__dll_api.DlnSpiSlaveGetCpha(handle, port, byref(cpha))
        return result, cpha.value

    def dln_spislave_enable_ss_rise_event(self, handle, port):
        result = self.__dll_api.DlnSpiSlaveEnableSSRiseEvent(handle, port)
        return result, cpha.value

    def dln_spislave_disable_ss_rise_event(self, handle, port):
        result = self.__dll_api.DlnSpiSlaveDisableSSRiseEvent(handle, port)
        return result

    def dln_spislave_is_ss_rise_event_enabled(self, handle, port):
        enabled = c_uint8()
        result = self.__dll_api.DlnSpiSlaveIsSSRiseEventEnabled(handle, port, byref(enabled))
        return result, enabled.value

    def dln_spislave_set_ss_idle_timeout(self, handle, port, timeout):
        result = self.__dll_api.DlnSpiSlaveSetSSIdleTimeout(handle, port, c_uint32(timeout))
        return result

    def dln_spislave_get_ss_idle_timeout(self, handle, port):
        timeout = c_uint32()
        result = self.__dll_api.DlnSpiSlaveGetSSIdleTimeout(handle, port, byref(timeout))
        return result, timeout.value

    def dln_spislave_set_replyshortage_action(self, handle, port, action):
        result = self.__dll_api.DlnSpiSlaveSetReplyShortageAction(handle, port, action)
        return result

    def dln_spislave_get_replyshortage_action(self, handle, port):
        action = c_uint8()
        result = self.__dll_api.DlnSpiSlaveGetReplyShortageAction(handle, port, byref(action))
        return result, action.value

    def dln_spislave_enqueue_reply(self, handle, port, size, buffer):
        result = self.__dll_api.DlnSpiSlaveEnqueueReply(handle, port, size, create_string_buffer(buffer, size))
        return result

    def dln_spislave_get_supported_modes(self, handle, port):
        values = DLN_SPI_SLAVE_MODE_VALUES()
        result = self.__dll_api.DlnSpiSlaveGetSupportedModes(handle, port, byref(values))
        count = values.count
        returnTab = values.values[:count]
        return result, returnTab

    def dln_spislave_get_supported_cpol_values(self, handle, port):
        values = DLN_SPI_SLAVE_CPOL_VALUES()
        result = self.__dll_api.DlnSpiSlaveGetSupportedCpolValues(handle, port, byref(values))
        return result, values

    def dln_spislave_get_supported_cpha_values(self, handle, port):
        values = DLN_SPI_SLAVE_CPHA_VALUES()
        result = self.__dll_api.DlnSpiSlaveGetSupportedCphaValues(handle, port, byref(values))
        return result, values

    def dln_spislave_get_supported_shortage_actions(self, handle, port):
        supported_sizes = DLN_SPI_SLAVE_SHORTAGE_ACTIONS()
        result = self.__dll_api.DlnSpiSlaveGetSupportedShortageActions(handle, port, byref(supported_sizes))
        return result, supported_sizes.value

    def dln_spislave_set_reply_mode(self, handle, port, replyMode):
        result = self.__dll_api.DlnSpiSlaveSetReplyMode(handle, port, replyMode)
        return result

    def dln_spislave_get_reply_mode(self, handle, port):
        reply_mode = c_uint8()
        result = self.__dll_api.DlnSpiSlaveGetReplyMode(handle, port, byref(reply_mode))
        return result, reply_mode.value

    def dln_spislave_get_supported_reply_modes(self, handle, port):
        supported_reply_modes = DLN_SPI_SLAVE_REPLY_MODES()
        result = self.__dll_api.DlnSpiSlaveGetSupportedReplyModes(handle, port, byref(supported_reply_modes))
        return result, supported_reply_modes.value

    def trt_event_notification(self, header, buffer):
        """
        This function handle the I2C Slave events.
        We handle only the write event to modify the reply registry.
        :param header: header structure
        :param buffer: full buffer structure
        :return:
        """
        if header.msgId == DLN_MSG_ID_SPI_SLAVE_DATA_RECEIVED_EV():
            event_struct = DLN_SPI_SLAVE_DATA_RECEIVED_EV.from_buffer_copy(buffer)
            pass
        else:
            # unknown event
            pass
