"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL NDG SW
:summary: wrapper for Diolan IO adapter module, contain Generic functions definitions import from the C header file
:since: 08/08/2014
:author: dpierrex
"""
from ctypes import Structure, c_uint8, c_uint16, c_char, c_void_p, CFUNCTYPE, c_uint32, c_char_p, c_ushort, \
    c_uint, byref

from acs_test_scripts.Equipment.IOAdapters.Diolan.Wrapper.DiolanWrapper import DLN_BUILD_MSG_ID, DLN_MODULE_GENERIC, DLN_MSG_HEADER


DLN_PIN_ROLE_NOT_IN_USE = 0


def DLN_MSG_ID_REGISTER_NOTIFICATION():
    DLN_BUILD_MSG_ID(0x00, DLN_MODULE_GENERIC)


def DLN_MSG_ID_UNREGISTER_NOTIFICATION():
    DLN_BUILD_MSG_ID(0x01, DLN_MODULE_GENERIC)


def DLN_MSG_ID_CONNECT():
    DLN_BUILD_MSG_ID(0x10, DLN_MODULE_GENERIC)


def DLN_MSG_ID_DISCONNECT():
    DLN_BUILD_MSG_ID(0x11, DLN_MODULE_GENERIC)


def DLN_MSG_ID_DISCONNECT_ALL():
    DLN_BUILD_MSG_ID(0x12, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_SRV_UUID():
    DLN_BUILD_MSG_ID(0x13, DLN_MODULE_GENERIC)


def DLN_MSG_ID_CLEANUP():
    DLN_BUILD_MSG_ID(0x14, DLN_MODULE_GENERIC)


def DLN_MSG_ID_CONNECTION_LOST_EV():
    DLN_BUILD_MSG_ID(0x1F, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_DEVICE_COUNT():
    DLN_BUILD_MSG_ID(0x20, DLN_MODULE_GENERIC)


def DLN_MSG_ID_OPEN_DEVICE():
    DLN_BUILD_MSG_ID(0x21, DLN_MODULE_GENERIC)


def DLN_MSG_ID_OPEN_STREAM():
    DLN_BUILD_MSG_ID(0x22, DLN_MODULE_GENERIC)


def DLN_MSG_ID_CLOSE_HANDLE():
    DLN_BUILD_MSG_ID(0x23, DLN_MODULE_GENERIC)


def DLN_MSG_ID_CLOSE_ALL_HANDLES():
    DLN_BUILD_MSG_ID(0x24, DLN_MODULE_GENERIC)


def DLN_MSG_ID_DEVICE_REMOVED_EV():
    DLN_BUILD_MSG_ID(0x2E, DLN_MODULE_GENERIC)


def DLN_MSG_ID_DEVICE_ADDED_EV():
    DLN_BUILD_MSG_ID(0x2F, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_VER():
    DLN_BUILD_MSG_ID(0x30, DLN_MODULE_GENERIC)  # Get HW, SW and protocol version


def DLN_MSG_ID_GET_DEVICE_SN():
    DLN_BUILD_MSG_ID(0x31, DLN_MODULE_GENERIC)


def DLN_MSG_ID_SET_DEVICE_ID():
    DLN_BUILD_MSG_ID(0x32, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_DEVICE_ID():
    DLN_BUILD_MSG_ID(0x33, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_HARDWARE_TYPE():
    DLN_BUILD_MSG_ID(0x34, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_HARDWARE_VERSION():
    DLN_BUILD_MSG_ID(0x35, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_FIRMWARE_VERSION():
    DLN_BUILD_MSG_ID(0x36, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_SERVER_VERSION():
    DLN_BUILD_MSG_ID(0x37, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_LIBRARY_VERSION():
    DLN_BUILD_MSG_ID(0x38, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_PIN_CFG():
    DLN_BUILD_MSG_ID(0x40, DLN_MODULE_GENERIC)


def DLN_MSG_ID_GET_COMMAND_RESTRICTION():
    DLN_BUILD_MSG_ID(0x41, DLN_MODULE_GENERIC)


def DLN_MSG_ID_DELAY():
    DLN_BUILD_MSG_ID(0x42, DLN_MODULE_GENERIC)


def DLN_MSG_ID_RESTART():
    DLN_BUILD_MSG_ID(0x43, DLN_MODULE_GENERIC)


# ////////////////////////////////////////////////////////////////////////////
# Device types
# ////////////////////////////////////////////////////////////////////////////
from ctypes import c_uint32 as DLN_HW_TYPE

DLN_HW_TYPE_DLN5 = 0x0500
DLN_HW_TYPE_DLN4M = 0x0401
DLN_HW_TYPE_DLN4S = 0x0402
DLN_HW_TYPE_DLN3 = 0x0300
DLN_HW_TYPE_DLN2 = 0x0200
DLN_HW_TYPE_DLN1 = 0x0100

#call back(handler of the asynchronous operations)
#typedef void (* PDLN_CALLBACK)(HDLN handle, void* context);
PDLN_CALLBACK = CFUNCTYPE(None, c_uint16, c_void_p)

DLN_NOTIFICATION_TYPE_NO_NOTIFICATION = 0x00
DLN_NOTIFICATION_TYPE_CALLBACK = 0x01


class _DLN_NOTIFICATION_CALLBACK_STRUCT(Structure):
    _pack_ = 1
    _fields_ = [("function", PDLN_CALLBACK),
                ("context", c_void_p)]


class DLN_NOTIFICATION(Structure):
    _pack_ = 1
    _fields_ = [("type", c_uint16),
                ("callback", _DLN_NOTIFICATION_CALLBACK_STRUCT)]


#////////////////////////////////////////////////////////////////////////////////
#//// Common commands
#////////////////////////////////////////////////////////////////////////////////
class DLN_REGISTER_NOTIFICATION_CMD(Structure):
    """
    The command registers notification settings
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("notification", DLN_NOTIFICATION)]


class DLN_REGISTER_NOTIFICATION_RSP(Structure):
    """
    The response notifies whether the settings were successfully registered.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_UNREGISTER_NOTIFICATION_CMD(Structure):
    """
    The command unregisters notification settings.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_UNREGISTER_NOTIFICATION_RSP(Structure):
    """
    The response notifies whether the settings were successfully unregistered.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)];


DLN_MAX_HOST_LENGTH = 50
DLN_DEFAULT_SERVER_PORT = 9656


class DLN_CONNECT_CMD(Structure):
    """
    The command establishes the connection to the DLN server.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("host", c_char * (DLN_MAX_HOST_LENGTH + 1)),
                ("port", c_uint16)]


class DLN_CONNECT_RSP(Structure):
    """
    The response notifies if the connection was successfully established.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_DISCONNECT_CMD(Structure):
    """
    The closes the connection to the specified DLN server.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("host", c_char * (DLN_MAX_HOST_LENGTH + 1)),
                ("port", c_uint16)]


class DLN_DISCONNECT_RSP(Structure):
    """
    The response notifies if the connection was successfully closed.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_DISCONNECT_ALL_CMD(Structure):
    """
    The command closes connections to all servers at once.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_DISCONNECT_ALL_RSP(Structure):
    """
    The response notifies if all the connections were successfully closed.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_GET_SRV_UUID_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_GET_SRV_UUID_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("uuid", c_uint8 * 16)]


class DLN_CLEANUP_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_CLEANUP_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_CONNECTION_LOST_EV(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("host", c_char * (DLN_MAX_HOST_LENGTH + 1)),
                ("port", c_uint16)]


DLN_DEVICE_FILTER_NUMBER = (1 << 0)
DLN_DEVICE_FILTER_HW_TYPE = (1 << 1)
DLN_DEVICE_FILTER_SN = (1 << 2)
DLN_DEVICE_FILTER_ID = (1 << 3)


class DLN_GET_DEVICE_COUNT_CMD(Structure):
    """
    The command retrieves the total number of DLN-devices available.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("filter", c_uint16),
                ("hardwareType", c_uint32),
                ("sn", c_uint32),
                ("id", c_uint32)]


class DLN_GET_DEVICE_COUNT_RSP(Structure):
    """
    The response notifies if the device was successfully opened.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("deviceCount", c_uint32)]


class DLN_OPEN_DEVICE_CMD(Structure):
    """
    The command opens the specified device.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("filter", c_uint16 ),
                ("number", c_uint32 ),
                ("hardwareType", DLN_HW_TYPE),
                ("sn", c_uint32 ),
                ("id", c_uint32)]


class DLN_OPEN_DEVICE_RSP(Structure):
    """
    The response notifies if the device was successfully opened.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("device_handle", c_uint16)]


class DLN_OPEN_STREAM_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_OPEN_STREAM_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("streamHandle", c_uint16)]


class DLN_CLOSE_HANDLE_CMD(Structure):
    """
    The command closes the handle to the opened DLN device (stream).
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_CLOSE_HANDLE_RSP(Structure):
    """
    The response notifies if the connection was successfully closed.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_CLOSE_ALL_HANDLES_CMD(Structure):
    """
    The command closes all handles to opened DLN devices (stream).
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_CLOSE_ALL_HANDLES_RSP(Structure):
    """
    The response notifies if all connections were successfully closed.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16 )]


class DLN_DEVICE_REMOVED_EV(Structure):
    """
    The event notifies about a device having being disconnected from a server.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_DEVICE_ADDED_EV(Structure):
    """
    The event notifies about a device being connected to a server.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("hardwareType", DLN_HW_TYPE),
                ("id", c_uint32),
                ("sn", c_uint32)]


class DLN_VERSION(Structure):
    """
    The structure is used to store the DLN device and software version data.
    """
    _pack_ = 1
    _fields_ = [("hardwareType", DLN_HW_TYPE),
                ("hardwareVersion", c_uint32),
                ("firmwareVersion", c_uint32),
                ("serverVersion", c_uint32),
                ("libraryVersion", c_uint32)]


class DLN_GET_VER_CMD(Structure):
    """
    The command retrieves the DLN device and software version data.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_GET_VER_RSP(Structure):
    """
    The response contains the retrieved information.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("version", DLN_VERSION)]


class DLN_GET_DEVICE_SN_CMD(Structure):
    """
    The command retrieves a device serial number.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_GET_DEVICE_SN_RSP(Structure):
    """
    The response contains a device serial number.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("sn", c_uint32)]


class DLN_SET_DEVICE_ID_CMD(Structure):
    """
    The command sets a new ID to the DLN device.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("id", c_uint32)]


class DLN_SET_DEVICE_ID_RSP(Structure):
    """
    The response notifies if the ID was successfully set.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_GET_DEVICE_ID_CMD(Structure):
    """
    The command retrieves the device ID number.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_GET_DEVICE_ID_RSP(Structure):
    """
    The response contains the retrieved device ID number.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("id", c_uint32)]


class DLN_GET_HARDWARE_TYPE_CMD(Structure):
    """
    The command retrieves the device hardware type (e.g DLN_HW_TYPE_DLN4M, DLN_HW_TYPE_DLN2)
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_GET_HARDWARE_TYPE_RSP(Structure):
    """
    The response contains the retrieved device hardware type.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("type", DLN_HW_TYPE)]


class DLN_GET_HARDWARE_VERSION_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_GET_HARDWARE_VERSION_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("version", c_uint32 )]


class DLN_GET_FIRMWARE_VERSION_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_GET_FIRMWARE_VERSION_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("version", c_uint32 )]


class DLN_GET_SERVER_VERSION_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_GET_SERVER_VERSION_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("version", c_uint32 )]


class DLN_GET_LIBRARY_VERSION_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_GET_LIBRARY_VERSION_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("version", c_uint32 )]


class DLN_PIN_CFG(Structure):
    """
    The structure is used to store the configuration of a single DLN device pin.
    """
    _pack_ = 1
    _fields_ = [("module", c_uint8),
                ("role", c_uint8)]


class DLN_DMA_CFG(Structure):
    _pack_ = 1
    _fields_ = [("module", c_uint8),
                ("role", c_uint8)]


class DLN_GET_PIN_CFG_CMD(Structure):
    """
    The command retrieves current configuration of the DLN device pin.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("pin", c_uint16)]


class DLN_GET_PIN_CFG_RSP(Structure):
    """
    The response contains current configuration of the specified DLN device pin.
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("cfg", DLN_PIN_CFG)]


class DLN_GET_COMMAND_RESTRICTION_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("msgId", c_uint16),
                ("entity", c_uint16 )]


class DLN_GET_COMMAND_RESTRICTION_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("restriction", c_uint8)]


class DLN_DELAY_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("delay", c_uint32)]


class DLN_DELAY_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_RESTART_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_RESTART_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DiolanGenWrapper():
    """

    """

    def __init__(self, dll_api):
        self.__dll_api = dll_api

    def dln_register_notification(self, handle, notification_type, notification_callback):
        """
        Registers notification settings
        :param handle - A handle to the DLN device.
        :param notification - Defines the notification settings.
        :return:
        """
        notification = DLN_NOTIFICATION()
        notification.type = notification_type
        notification.callback.function = notification_callback
        result = self.__dll_api.DlnRegisterNotification(handle, notification)
        return result

    def dln_unregister_notification(self, handle):
        """
        Unregisters notification settings.
        :param handle - A handle to the DLN device.
        :return:
        """
        result = self.__dll_api.DlnUnregisterNotification(handle)
        return result

    def dln_connect(self, host, port):
        """
        Establishes the connection to the DLN server.
        :param host - A server to establish the connection to.
        :param port - A port number of the DLN server.
        :return DLN_RES_INSUFFICIENT_RESOURCES
        :return DLN_RES_SOCKET_INITIALIZATION_FAILED
        :return:
        """
        result = self.__dll_api.DlnConnect(c_char_p(host), c_ushort(port))
        return result

    def dln_disconnect(self, host, port):
        """
        Closes the connection to the specified DLN server.
        :param host - A server to close the connection to.
        :param port - A port of the DLN server.
        :return:
        """
        result = self.__dll_api.DlnDisconnect(c_char_p(host), c_ushort(port))
        return result

    def dln_disconnect_all(self):
        """
        Closes connections to all servers at once.
        :return DLN_RES_SUCCESS - connections to all servers were successfully closed;
        :return DLN_RES_NOT_CONNECTED - no connections were present during the command execution.
        """
        result = self.__dll_api.DlnDisconnectAll()
        return result

    #c_uint16 DlnCleanup();
    """
    Closes all connections and frees the resources used.
    :return DLN_RES_SUCCESS
    """

    def dln_get_device_count(self):
        """
        Retrieves the total number of DLN-devices available.
        :param deviceCount - A pointer to an unsigned 32-bit integer.
        This integer will be filled with the total number of available DLN devices.
        :return:
        """
        deviceCount = c_uint()
        result = self.__dll_api.DlnGetDeviceCount(byref(deviceCount))
        return result, deviceCount

    def dln_open_device(self, device_number):
        """
        Opens the specified device corresponding to the specified deviceNumber.
        If successful, returns the handle of the opened device as device_handle.
        :param device_number - A number of the device to be opened.
        :param device_handle - A pointer to the variable that receives the device handle after the function execution.
        :return DLN_RES_SUCCESS - The device was successfully opened. The device_handle parameter contains a valid handle.
        :return DLN_RES_NOT_CONNECTED - The library was not connected to any server.
        :return DLN_RES_MEMORY_ERROR - Not enough memory to process this command.
        :return DLN_RES_HARDWARE_NOT_FOUND - The number of available devices is less than deviceNumber+1
        retval DLN_RES_DEVICE_REMOVED - The device was disconnected while opening.
        """
        device_handle = c_ushort()
        result = self.__dll_api.DlnOpenDevice(c_uint(device_number), byref(device_handle))
        return result, device_handle

    def dln_open_device_by_sn(self, device_sn):
        """
        Opens a specified defined by its serial number.
        :param sn - A serial number of the DLN device.
        :param device_handle - A pointer to the variable that receives the device handle after the function execution.
        :return:
        """
        device_handle = c_ushort()
        result = self.__dll_api.DlnOpenDeviceBySn(c_uint(device_sn), byref(device_handle))
        return result, device_handle

    def dln_open_device_by_id(self, deviceId):
        """
        Opens a specified defined by its ID number.
        :param id - An ID number of the DLN device.
        :param device_handle - A pointer to the variable that receives the device handle after the function execution.
        :return:
        """
        device_handle = c_ushort()
        result = self.__dll_api.DlnOpenDeviceById(c_uint(deviceId), byref(device_handle))
        return result, device_handle

    #c_uint16    DlnOpenDeviceByHwType(DLN_HW_TYPE    hwType, HDLN * device_handle);

    def dln_open_stream(self, device_handle):
        stream_handle = c_ushort()
        result = self.__dll_api.DlnOpenStream(device_handle, byref(stream_handle))
        return result, stream_handle

    def dln_close_handle(self, handle):
        """
        Closes the handle to an opened DLN device (stream).
        :param handle - A handle to the DLN device.
        :return:
        """
        result = self.__dll_api.DlnCloseHandle(handle)
        return result

    def dln_close_all_handles(self):
        """
        Closes handles to all opened DLN devices and streams.
        :return:
        """
        result = self.__dll_api.DlnCloseAllHandles()
        return result

    def DlnGetVersion(self, handle):
        """
        Retrieves the DLN device and software version data.
        :param handle - A handle to the DLN device.
        :param version - A pointer to a DLN_VERSION structure that receives version information after the function execution.
        :return:
        """
        version = DLN_VERSION()
        result = self.__dll_api.DlnGetVersion(handle, byref(version))
        return result, version

    def dln_get_device_sn(self, handle):
        """
        Retrieves the device serial number.
        :param handle - A handle to the DLN device.
        :param sn - A pointer to the variable that receives the device serial number after the function execution.
        :return:
        """
        device_sn = c_uint()
        result = self.__dll_api.dln_get_device_sn(handle, byref(device_sn))
        return result, device_sn

    def dln_set_device_id(self, handle, deviceId):
        """
        Sets a new ID number to the DLN device.
        :param handle - A handle to the DLN device.
        :param id - An ID number to be set.
        :return:
        """
        result = self.__dll_api.DlnSetDeviceId(handle, c_uint(deviceId))
        return result

    def dln_get_device_id(self, handle):
        """
        Retrieves the device ID number.
        :param handle - A handle to the DLN device.
        :param id - A pointer to an unsigned 32-bit integer. This integer will be filled with the ID number after the function execution.
        :return:
        """
        device_id = c_uint()
        result = self.__dll_api.DlnGetDeviceId(handle, byref(device_id))
        return result, device_id
