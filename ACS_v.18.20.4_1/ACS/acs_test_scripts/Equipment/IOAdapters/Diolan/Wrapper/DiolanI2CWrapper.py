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
:summary: wrapper for Diolan IO adapter module I2C, contain I2C definition import from the C header file
:since: 08/08/2014
:author: dpierrex
"""
from ctypes import Structure, byref, c_uint8, c_uint16, create_string_buffer

from acs_test_scripts.Equipment.IOAdapters.Diolan.Wrapper.DiolanResult import DLN_SUCCEEDED
from acs_test_scripts.Equipment.IOAdapters.Diolan.Wrapper.DiolanDef import DLN_BUILD_MSG_ID, DLN_MODULE_I2C_SLAVE, DLN_MSG_HEADER


"""
Commands group and macros
"""
DLN_PIN_ROLE_I2C_SLAVE_SDA = 0
DLN_PIN_ROLE_I2C_SLAVE_SCL = 1

DLN_MSG_ID_I2C_SLAVE_GET_PORT_COUNT = DLN_BUILD_MSG_ID(0x00, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_ENABLE = DLN_BUILD_MSG_ID(0x01, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_DISABLE = DLN_BUILD_MSG_ID(0x02, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_IS_ENABLED = DLN_BUILD_MSG_ID(0x03, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_GET_ADDRESS_COUNT = DLN_BUILD_MSG_ID(0x04, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_SET_ADDRESS = DLN_BUILD_MSG_ID(0x05, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_GET_ADDRESS = DLN_BUILD_MSG_ID(0x06, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_GENERAL_CALL_ENABLE = DLN_BUILD_MSG_ID(0x07, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_GENERAL_CALL_DISABLE = DLN_BUILD_MSG_ID(0x08, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_GENERAL_CALL_IS_ENABLED = DLN_BUILD_MSG_ID(0x09, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_LOAD_REPLY = DLN_BUILD_MSG_ID(0x0A, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_SET_EVENT = DLN_BUILD_MSG_ID(0x0B, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_GET_EVENT = DLN_BUILD_MSG_ID(0x0C, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_READ_EV = DLN_BUILD_MSG_ID(0x10, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_WRITE_EV = DLN_BUILD_MSG_ID(0x11, DLN_MODULE_I2C_SLAVE)
DLN_MSG_ID_I2C_SLAVE_GET_SUPPORTED_EVENT_TYPES = DLN_BUILD_MSG_ID(0x40, DLN_MODULE_I2C_SLAVE)

DLN_I2C_SLAVE_BUFFER_SIZE = 256

DLN_I2C_SLAVE_ENABLED = 1
DLN_I2C_SLAVE_DISABLED = 0
DLN_I2C_SLAVE_CANCEL_TRANSFERS = 0
DLN_I2C_SLAVE_WAIT_FOR_TRANSFERS = 1

DLN_I2C_SLAVE_ADDRESS_DISABLED = 0
DLN_I2C_SLAVE_GENERAL_CALL_ENABLED = 1
DLN_I2C_SLAVE_GENERAL_CALL_DISABLED = 0

DLN_I2C_SLAVE_EVENT_NONE = 0
DLN_I2C_SLAVE_EVENT_READ = 1
DLN_I2C_SLAVE_EVENT_WRITE = 2
DLN_I2C_SLAVE_EVENT_READ_WRITE = 3

"""
CType definitions
"""


class DLN_I2C_SLAVE_GET_PORT_COUNT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]


class DLN_I2C_SLAVE_GET_PORT_COUNT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("count", c_uint8)]


class DLN_I2C_SLAVE_ENABLE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_I2C_SLAVE_ENABLE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("conflict", c_uint16)]


class DLN_I2C_SLAVE_DISABLE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("waitForTransferCompletion", c_uint8)]


class DLN_I2C_SLAVE_DISABLE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_I2C_SLAVE_IS_ENABLED_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_I2C_SLAVE_IS_ENABLED_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("enabled", c_uint8)]


class DLN_I2C_SLAVE_GET_ADDRESS_COUNT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_I2C_SLAVE_GET_ADDRESS_COUNT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("count", c_uint8)]


class DLN_I2C_SLAVE_SET_ADDRESS_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("slaveAddressNumber", c_uint8),
                ("address", c_uint8)]


class DLN_I2C_SLAVE_SET_ADDRESS_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_I2C_SLAVE_GET_ADDRESS_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("slaveAddressNumber", c_uint8)]


class DLN_I2C_SLAVE_GET_ADDRESS_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("address", c_uint8)]


class DLN_I2C_SLAVE_GENERAL_CALL_ENABLE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_I2C_SLAVE_GENERAL_CALL_ENABLE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_I2C_SLAVE_GENERAL_CALL_DISABLE_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_I2C_SLAVE_GENERAL_CALL_DISABLE_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_I2C_SLAVE_GENERAL_CALL_IS_ENABLED_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8)]


class DLN_I2C_SLAVE_GENERAL_CALL_IS_ENABLED_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("enabled", c_uint8)]


class DLN_I2C_SLAVE_LOAD_REPLY_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("size", c_uint16),
                ("buffer", c_uint8 * DLN_I2C_SLAVE_BUFFER_SIZE)]


class DLN_I2C_SLAVE_LOAD_REPLY_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_I2C_SLAVE_SET_EVENT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("slaveAddressNumber", c_uint8),
                ("eventType", c_uint8)]


class DLN_I2C_SLAVE_SET_EVENT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_I2C_SLAVE_GET_EVENT_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("port", c_uint8),
                ("slaveAddressNumber", c_uint8)]


class DLN_I2C_SLAVE_GET_EVENT_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("eventType", c_uint8)]


class DLN_I2C_SLAVE_READ_EV(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("eventCount", c_uint16),
                ("eventType", c_uint8),
                ("port", c_uint8),
                ("slaveAddress", c_uint8),
                ("size", c_uint16)]


class DLN_I2C_SLAVE_WRITE_EV(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("eventCount", c_uint16),
                ("eventType", c_uint8),
                ("port", c_uint8),
                ("slaveAddress", c_uint8),
                ("size", c_uint16),
                ("buffer", c_uint8 * DLN_I2C_SLAVE_BUFFER_SIZE)]


class DLN_I2C_SLAVE_EVENT_TYPES(Structure):
    _pack_ = 1
    _fields_ = [("count", c_uint8),
                ("eventTypes", c_uint8 * 8)]


class DLN_I2C_SLAVE_GET_SUPPORTED_EVENT_TYPES_CMD(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_I2C_SLAVE_GET_SUPPORTED_EVENT_TYPES_RSP(Structure):
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16),
                ("supportedEventTypes", DLN_I2C_SLAVE_EVENT_TYPES)]


class DiolanI2CSlaveWrapper():
    """

    """

    def __init__(self, dll_api):
        self.__dll_api = dll_api
        self.__fullPayload = "00"

    def dln_i2cslave_get_port_count(self, handle):
        """

        :param handle:
        :return:DLN_RES_SUCCESS - the port count has been successfully retrieved.
        """
        count = c_uint8()
        result = self.__dll_api.DlnI2cSlaveGetPortCount(handle, byref(count))
        return result, count.value

    def dln_i2cslave_Enable(self, handle, port):
        """
        :param handle:
        :param port:
        :return:DLN_RES_SUCCESS - the I2C slave module has been successfully enabled
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        DLN_RES_PIN_IN_USE - the I2C pins are assigned to another module of the adapter and cannot be enabled as I2C
        DLN_RES_I2C_SLAVE_ADDRESS_NEEDED - I2C slave address must be set before enabling.
        """
        conflict = c_uint8()
        result = self.__dll_api.DlnI2cSlaveEnable(handle, c_uint8(port), byref(conflict))
        return result, conflict

    def dln_i2cslave_disable(self, handle, port, wait_for_transfer_completion):
        """

        :param handle:
        :param port:
        :param wait_for_transfer_completion:
        :return:DLN_RES_SUCCESS - the I2C slave module has been successfully disabled
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        DLN_RES_TRANSFER_CANCELLED - the pending transfers were cancelled.
        """
        result = self.__dll_api.DlnI2cSlaveDisable(handle, c_uint8(port), c_uint8(wait_for_transfer_completion))
        return result

    def dln_i2cslave_is_enabled(self, handle, port):
        """

        :param handle:
        :param port:
        :return:DLN_RES_SUCCESS - the I2C slave state has been successfully retrieved
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range.
        """
        enabled = c_uint8()
        result = self.__dll_api.DlnI2cSlaveIsEnabled(handle, port, byref(enabled))
        return result, enabled.value

    def dln_i2cslave_get_address_count(self, handle, port):
        """

        :param handle:
        :param port:
        :return:DLN_RES_SUCCESS - the I2C slave address count has been successfully retrieved
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range.
        """
        count = c_uint8()
        result = self.__dll_api.DlnI2cSlaveGetAddressCount(handle, port, byref(count))
        return result, count.value

    def dln_i2cslave_set_address(self, handle, port, slave_address_number, address):
        """

        :param handle:
        :param port:
        :param slave_address_number:
        :param address:
        :return:DLN_RES_SUCCESS - the I2C slave address has been successfully set
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        DLN_RES_BUSY - could not change address while the I2C slave module is enabled
        DLN_RES_INVALID_ADDRESS - the address is out of range.
        """
        result = self.__dll_api.DlnI2cSlaveSetAddress(handle, c_uint8(port), c_uint8(slave_address_number), c_uint8(address))
        return result

    def dln_i2cslave_get_address(self, handle, port, slave_address_number):
        """

        :param handle:
        :param port:
        :param slave_address_number:
        :return:DLN_RES_SUCCESS - the I2C slave address has been successfully retrieved
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        """
        address = c_uint8()
        result = self.__dll_api.DlnI2cSlaveGetAddress(handle, c_uint8(port), c_uint8(slave_address_number), byref(address))
        return result, "{:X}".format(address.value,)

    def dln_i2cslave_general_call_enable(self, handle, port):
        """

        :param handle:
        :param port:
        :return:DLN_RES_SUCCESS - the I2C slave general call has been successfully enabled
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range.
        """
        result = self.__dll_api.DlnI2cSlaveGeneralCallEnable(handle, c_uint8(port))
        return result

    def dln_i2cslave_general_call_disable(self, handle, port):
        """

        :param handle:
        :param port:
        :return:DLN_RES_SUCCESS - the I2C slave general call has been successfully disabled
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range.
        """
        result = self.__dll_api.DlnI2cSlaveGeneralCallDisable(handle, c_uint8(port))
        return result

    def dln_i2cslave_general_call_is_enabled(self, handle, port):
        """

        :param handle:
        :param port:
        :return:DLN_RES_SUCCESS - the I2C slave general call state has been successfully retrieved
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        """
        enabled = c_uint8()
        result = self.__dll_api.DlnI2cSlaveGeneralCallIsEnabled(handle, c_uint8(port), byref(enabled))
        return result, enabled.value

    def dln_i2cslave_load_reply(self, handle, port, size, buffer):
        """

        :param handle:
        :param port:
        :param size:
        :param buffer:
        :return:DLN_RES_SUCCESS - the I2C slave reply has been successfully loaded
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        DLN_RES_BUSY - could not load reply while the I2C slave module is enabled
        DLN_RES_INVALID_BUFFER_SIZE - the buffer size is out of range.
        """
        self.__fullPayload = buffer
        result = self.__dll_api.DlnI2cSlaveLoadReply(handle, c_uint8(port), c_uint16(size), create_string_buffer(buffer))
        return result

    def dln_i2cslave_set_event(self, handle, port, slave_address_number, event_type):
        """

        :param handle:
        :param port:
        :param slave_address_number:
        :param event_type:
        :return:DLN_RES_SUCCESS - the I2C slave event has been successfully configured
        DLN_RES_INVALID_PORT_NUMBER - the port number is out of range
        DLN_RES_BUSY - could not configure event while the I2C slave module is enabled
        DLN_RES_INVALID_EVENT_TYPE - the event type is out of range.
        """
        result = self.__dll_api.DlnI2cSlaveSetEvent(handle, c_uint8(port), c_uint8(slave_address_number), c_uint8(event_type))
        return result

    def dln_i2cslave_get_event(self, handle, port, slave_address_number, event_type):
        """
        :param handle:
        :param port:
        :param slave_address_number:
        :param eventType:
        :return: DLN_RES_SUCCESS - the I2C slave event state has been successfully retrieved
        :return: DLN_RES_INVALID_PORT_NUMBER - the port number is out of range.
        """
        result = self.__dll_api.DlnI2cSlaveGetEvent(handle, c_uint8(port), c_uint8(slave_address_number), byref(event_type))
        return result, event_type

    def dln_i2cslave_get_supported_event_types(self, handle, port, supported_event_types):
        result = self.__dll_api.DlnI2cSlaveGetSupportedEventTypes(handle, c_uint8(port), byref(supported_event_types))

        return result,

    def trt_event_notification(self, header, buffer):
        """
        This function handle the I2C Slave events.
        We handle only the write event to modify the reply registry.
        :param header: header structure
        :param buffer: full buffer structure
        :return:
        """
        if header.msgId == DLN_MSG_ID_I2C_SLAVE_WRITE_EV:
            event_struct = DLN_I2C_SLAVE_WRITE_EV.from_buffer_copy(buffer)
            buf = bytearray(event_struct.buffer)
            buf = buf[:event_struct.size]
            write = buf[1:].decode('utf-8')
            # address_index = struct.unpack_from("B", buf[:1])[0] - 1

            if event_struct.size == 1:
                pass
                #master want writing at address
            else:
                # ajust the payload size
                if len(write) > len(self.__fullPayload):
                    to_add = len(write) - len(self.__fullPayload)
                    self.__fullPayload += '\x00' * to_add

                # modify the reply registry
                for count, byte in enumerate(write):
                    payload = list(self.__fullPayload)
                    payload[count] = byte.decode('utf-8')
                    self.__fullPayload = ''.join(payload)
                # load the new reply registry
                enable = self.dln_i2cslave_is_enabled(event_struct.header.handle, 0)
                if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
                    self.dln_i2cslave_disable(event_struct.header.handle, 0, 0)
                self.dln_i2cslave_load_reply(event_struct.header.handle, event_struct.port, len(self.__fullPayload),
                                             self.__fullPayload)
                if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
                    self.dln_i2cslave_Enable(event_struct.header.handle, 0)
        else:
            # unknown event
            pass
