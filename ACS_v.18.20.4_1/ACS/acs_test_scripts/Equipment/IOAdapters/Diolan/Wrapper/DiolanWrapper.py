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
:summary: wrapper for Diolan IO adapter
:since: 07/08/2014
:author: dpierrex
"""
import binascii
from ctypes import create_string_buffer, c_uint16

from ErrorHandling.TestEquipmentException import TestEquipmentException
from DiolanDef import *
from acs_test_scripts.Equipment.IOAdapters.Diolan.Wrapper.DiolanResult import DLN_SUCCEEDED, DLN_FAILED, DLN_RES_FAIL, DLN_RES_SUCCESS

# If you want to connect to another server specify its IP or url
# in serverHost variable.
SERVER_HOST = "localhost"
# You can change the server port, changing the serverPort variable.
SERVER_PORT = DLN_DEFAULT_SERVER_PORT


class DiolanWrapper():
    """
    Abstract Class that implements wrapper that will drive Diolan io adapter
    """

    def __init__(self, logger, api):
        """
        Constructor
        :type logger: Logger
        :param logger: the logger of the io adapter instance

        :type api: DllLoader
        :param api: the handle to the dll api
        """
        self._logger = logger
        self._api = api
        self._serial_number = "---"
        self.__handle = None
        self._dlnGen = None
        self._dlnI2cSlave = None
        self._dlnSpiSlave = None
        self._dlnCallback = None

    def _import_module_in_wrapper(self, *args):
        """

        :param args: list of modules defined in DiolanDef.py
        :return: True if module correctly imported
        """
        for module in args:
            if module == DLN_MODULE_GENERIC and self._dlnGen is None:
                from acs_test_scripts.Equipment.IOAdapters.Diolan.Wrapper.DiolanGenWrapper import DiolanGenWrapper

                self._dlnGen = DiolanGenWrapper(self._api)

            if module == DLN_MODULE_I2C_SLAVE and self._dlnI2cSlave is None:
                from acs_test_scripts.Equipment.IOAdapters.Diolan.Wrapper.DiolanI2CWrapper import DiolanI2CSlaveWrapper

                self._dlnI2cSlave = DiolanI2CSlaveWrapper(self._api)
                self.enable_i2cslave_notifications()

            if module == DLN_MODULE_SPI_SLAVE and self._dlnSpiSlave is None:
                from acs_test_scripts.Equipment.IOAdapters.Diolan.Wrapper.DiolanSpiWrapper import DiolanSpiSlaveWrapper

                self._dlnSpiSlave = DiolanSpiSlaveWrapper(self._api)

            if module > DLN_MODULE_ANALYZER:
                TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                       "Diolan module with value {0} not exist".format(module))

    def _decimal_octets_as_string(self, n, byteCount):
        s = ""
        for i in range(byteCount - 1):
            s = "." + str(n % 256) + s
            n /= 256
        return str(n) + s

    # Printing the version information
    def _format_version(self, version):
        hardware_type_id = self._decimal_octets_as_string(version.hardwareType, 2)
        hwtype = version.hardwareType / 256
        if hwtype == DLN_HW_TYPE_DLN1:
            hardware_model = "DLN-1"
        elif hwtype == DLN_HW_TYPE_DLN2:
            hardware_model = "DLN-2"
        elif hwtype == DLN_HW_TYPE_DLN3:
            hardware_model = "DLN-3"
        elif hwtype == DLN_HW_TYPE_DLN4:
            submodel = version.hardwareType % 256
            if submodel == 1:
                hardware_model = "DLN-4M"
            elif submodel == 2:
                hardware_model = "DLN-4S"
            else:
                hardware_model = "DLN-4x (%d)" % submodel
        elif hwtype == DLN_HW_TYPE_DLN5:
            hardware_model = "DLN-5"
        else:
            hardware_model = "Unknown"

        hardware_version = self._decimal_octets_as_string(version.hardwareVersion, 4)
        firmware_version = self._decimal_octets_as_string(version.firmwareVersion, 4)
        server_version = self._decimal_octets_as_string(version.serverVersion, 4)
        library_version = self._decimal_octets_as_string(version.libraryVersion, 4)

        self._logger.info("Hardware type ID: " + hardware_type_id)
        self._logger.info("Hardware model:   " + hardware_model)
        self._logger.info("Hardware version: " + hardware_version)
        self._logger.info("Firmware version: " + firmware_version)
        self._logger.info("Server version:   " + server_version)
        self._logger.info("Library version:  " + library_version)

        version_dict = {"Hardware_Type_ID": hardware_type_id,
                        "Hardware_model": hardware_model,
                        "Hardware_version": hardware_version,
                        "Firmware_version": firmware_version,
                        "Server_version": server_version,
                        "Library_Version": library_version}

        return version_dict

    def serial_number(self):
        """
        :rtype: int
        :return: the io adapter serial number
        """
        return self._serial_number

    def dln_server_connection(self):
        self._import_module_in_wrapper(DLN_MODULE_GENERIC)
        self._logger.info("start connection")
        result = dlnGen.dln_connect(SERVER_HOST, SERVER_PORT)
        self._logger.info("connection return {0}".format(result))
        if DLN_FAILED(result):
            TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Diolan Connection fail")

    def reset_board(self):
        """
        cleanup all remain connection and get a new one
        :rtype: bool
        :return: True if all is ok else False
        """
        self._import_module_in_wrapper(DLN_MODULE_GENERIC)
        self._dlnGen.dln_close_all_handles()
        self._dlnGen.dln_disconnect_all()
        self._logger.info("start connection")
        result = self._dlnGen.dln_connect(SERVER_HOST, SERVER_PORT)
        self._logger.info("connection return {0}".format(result))
        if DLN_FAILED(result):
            TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Diolan Connection fail")
        else:
            return True

    def open_device(self, deviceId):
        """
        Open the Diolan io adapter interface
        :return: handle to the device
        """
        self._import_module_in_wrapper(DLN_MODULE_GENERIC)
        if deviceId is not None:
            res = self._dlnGen.dln_open_device_by_id(deviceId)
            if DLN_SUCCEEDED(res[0]):
                self.__handle = res[1]

        else:
            TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Diolan Connection fail")

        return self.__handle

    def close_device(self):
        """
        Close the Diolan io adapter interface
        """
        self._import_module_in_wrapper(DLN_MODULE_GENERIC)
        self._dlnGen.dln_close_all_handles()
        self._dlnGen.dln_disconnect_all()

    def get_version(self):
        """
        get the versions of the Diolan io adapter
        Hardware_Type_ID | Hardware_model | Hardware_version | Firmware_version | Server_version | Library_Version
        :return: dictionary containing versions information
        """
        self._import_module_in_wrapper(DLN_MODULE_GENERIC)
        res = self._dlnGen.DlnGetVersion(self.__handle)
        if DLN_SUCCEEDED(res[0]):
            self._format_version(res[1])

    def get_serial_number(self):
        """
        Return the device serial number
        :return int: serial number
        """
        self._import_module_in_wrapper(DLN_MODULE_GENERIC)
        res = self._dlnGen.DlnGetDeviceSn(self.__handle)
        if DLN_SUCCEEDED(res[0]):
            self._serial_number = res[1]
        return self._serial_number

    def get_i2cslave_address(self):
        """
        Retreive the I2C address of the module
        :return: string containing address in hexa
        """

        self._import_module_in_wrapper(DLN_MODULE_I2C_SLAVE)
        res = self._dlnI2cSlave.dln_i2cslave_get_address(self.__handle, 0, 1)
        if DLN_SUCCEEDED(res[0]):
            return res[1]

    def set_i2cslave_address(self, address):
        """
        Set the I2C slave address
        :param address: hexa string containing the address to set
        :return: True if the command was successfully send else False
        """
        self._import_module_in_wrapper(DLN_MODULE_I2C_SLAVE)
        enable = self._dlnI2cSlave.dln_i2cslave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            self._dlnI2cSlave.dln_i2cslave_disable(self.__handle, 0, 0)
        res = self._dlnI2cSlave.dln_i2cslave_set_address(self.__handle, 0, 1, int(address, 16))
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            enable = self._dlnI2cSlave.dln_i2cslave_Enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0]) and DLN_SUCCEEDED(res)

    def set_i2cslave_payload(self, payload):
        """
        Set the payload value of the device witch is read on i2cget
        :param payload: String containing the hex value to load
        :return: True if the payload was successfully set else false
        """
        self._import_module_in_wrapper(DLN_MODULE_I2C_SLAVE)
        enable = self._dlnI2cSlave.dln_i2cslave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            self._dlnI2cSlave.dln_i2cslave_disable(self.__handle, 0, 0)
        buffer = binascii.unhexlify(payload)
        res = self._dlnI2cSlave.dln_i2cslave_load_reply(self.__handle, 0, len(buffer), buffer)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            enable = self._dlnI2cSlave.dln_i2cslave_Enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0]) and DLN_SUCCEEDED(res)

    def enable_i2cslave(self):
        """
        Enable the I2C Slave
        :return: True if success, else False
        """
        self._import_module_in_wrapper(DLN_MODULE_I2C_SLAVE)
        enable = self._dlnI2cSlave.dln_i2cslave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 0:
            enable = self._dlnI2cSlave.dln_i2cslave_Enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0])

    def disable_i2cslave(self):
        """
        Disable the I2C Slave
        :return: True if success, else False
        """
        self._import_module_in_wrapper(DLN_MODULE_I2C_SLAVE)
        enable = self._dlnI2cSlave.dln_i2cslave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            res = self._dlnI2cSlave.dln_i2cslave_disable(self.__handle, 0, 5)
            return DLN_SUCCEEDED(res)
        else:
            return DLN_SUCCEEDED(enable[0])

    def py_dlnCallback(self, handle, context_p):
        """
        Callback for the Diolan Events
        :param handle:
        :param context_p:
        :return:
        """
        res = self.__dln_get_message(HDLN_ALL_DEVICES)
        if DLN_SUCCEEDED(res[0]):
            return_buf = res[1]
            header = DLN_MSG_HEADER.from_buffer_copy(return_buf)
            # DLN_DEVICE_ADDED_EV event doesn't contain device handle - device is not opened yet.
            # To show device ID and SN in the events list, we open the device and change the handle
            # in the DLN_DEVICE_ADDED_EV with the new one.
            # This way the DLN_DEVICE_ADDED_EV can be processed as any other event.
            if self._dlnI2cSlave is not None:
                self._dlnI2cSlave.trt_event_notification(header, return_buf)
            if self._dlnSpiSlave is not None:
                self._dlnSpiSlave.trt_event_notification(header, return_buf)

    def enable_notification_callback(self):
        """
        Enable the Events from Diolan
        :return:
        """
        self._import_module_in_wrapper(DLN_MODULE_GENERIC)
        from acs_test_scripts.Equipment.IOAdapters.Diolan.Wrapper.DiolanGenWrapper import PDLN_CALLBACK, DLN_NOTIFICATION_TYPE_CALLBACK

        self._dlnCallback = PDLN_CALLBACK(self.py_dlnCallback)
        res = self._dlnGen.dln_register_notification(HDLN_ALL_DEVICES, notification_type=DLN_NOTIFICATION_TYPE_CALLBACK,
                                                   notification_callback=self._dlnCallback)
        self._logger.info("DLN register notification ret = " + str(DLN_SUCCEEDED(res)))

    def enable_i2cslave_notifications(self):
        """
        Enable the I2C Slave notifications
        :return:
        """
        if self._dlnI2cSlave is not None:
            enable = self._dlnI2cSlave.dln_i2cslave_is_enabled(self.__handle, 0)
            if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
                self._dlnI2cSlave.dln_i2cslave_disable(self.__handle, 0, 0)
            from acs_test_scripts.Equipment.IOAdapters.Diolan.Wrapper.DiolanI2CWrapper import DLN_I2C_SLAVE_EVENT_WRITE

            res = self._dlnI2cSlave.dln_i2cslave_set_event(self.__handle, 0, 1, DLN_I2C_SLAVE_EVENT_WRITE)
            if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
                enable = self._dlnI2cSlave.dln_i2cslave_Enable(self.__handle, 0)
            return DLN_SUCCEEDED(enable[0]) and DLN_SUCCEEDED(res)
        else:
            return False

    def enable_spislave(self):
        """
        Enable the SPI Slave
        :return: True if success, else False
        """
        self._import_module_in_wrapper(DLN_MODULE_SPI_SLAVE)
        enable = self._dlnSpiSlave.dln_spislave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 0:
            enable = self._dlnSpiSlave.dln_spislave_enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0])

    def disable_spislave(self):
        """
        Disable the SPI Slave
        :return: True if success, else False
        """
        self._import_module_in_wrapper(DLN_MODULE_SPI_SLAVE)
        enable = self._dlnSpiSlave.dln_spislave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            res = self._dlnSpiSlave.dln_spislave_disable(self.__handle, 0, 5)
            return DLN_SUCCEEDED(res)
        else:
            return DLN_SUCCEEDED(enable[0])

    def set_spislave_payload(self, payload):
        """
        Set the payload value of the device witch is read on i2cget
        :param payload: String containing the hex value to load
        :return: True if the payload was successfully set else false
        """
        self._import_module_in_wrapper(DLN_MODULE_SPI_SLAVE)
        enable = self._dlnSpiSlave.dln_spislave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            self._dlnSpiSlave.dln_spislave_disable(self.__handle, 0, 0)
        bin_payload = binascii.unhexlify(payload)
        res = self._dlnSpiSlave.dln_spislave_load_reply(self.__handle, 0, len(bin_payload), bin_payload)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            enable = self._dlnSpiSlave.dln_spislave_enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0]) and DLN_SUCCEEDED(res)

    def dln_spislave_get_supported_modes(self):
        self._import_module_in_wrapper(DLN_MODULE_SPI_SLAVE)
        enable = self._dlnSpiSlave.dln_spislave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            self._dlnSpiSlave.dln_spislave_disable(self.__handle, 0, 0)

        res = self._dlnSpiSlave.dln_spislave_get_supported_modes(self.__handle, 0)

        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            enable = self._dlnSpiSlave.dln_spislave_enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0]) and DLN_SUCCEEDED(res[0]), res[1]

    def dln_spislave_set_mode(self, mode):
        """
        Set the SPI mode of the SPI Slave
        :param mode: possible values are
         Mode 0 => DLN_SPI_SLAVE_CPOL_0 | DLN_SPI_SLAVE_CPHA_0
         Mode 1 => DLN_SPI_SLAVE_CPOL_0 | DLN_SPI_SLAVE_CPHA_1
         Mode 2 => DLN_SPI_SLAVE_CPOL_1 | DLN_SPI_SLAVE_CPHA_0
         Mode 3 => DLN_SPI_SLAVE_CPOL_1 | DLN_SPI_SLAVE_CPHA_1
        :return:
        """
        cpol = 0
        cpha = 0
        if mode == 1:
            cpol = 0
            cpha = 1
        if mode == 2:
            cpol = 1
            cpha = 0
        if mode == 3:
            cpol = 1
            cpha = 1
        self._import_module_in_wrapper(DLN_MODULE_SPI_SLAVE)
        enable = self._dlnSpiSlave.dln_spislave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            self._dlnSpiSlave.dln_spislave_disable(self.__handle, 0, 0)

        if DLN_SUCCEEDED(self._dlnSpiSlave.dln_spislave_set_cpol(self.__handle, 0, cpol)) and DLN_SUCCEEDED(
                self._dlnSpiSlave.dln_spislave_set_cpha(self.__handle, 0, cpha)):
            res = DLN_RES_SUCCESS
        else:
            res = DLN_RES_FAIL

        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            enable = self._dlnSpiSlave.dln_spislave_enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0]) and DLN_SUCCEEDED(res)

    def dln_spislave_get_mode(self):
        """
        Set the SPI mode of the SPI Slave
        :return mode: possible values are
         Mode 0 => DLN_SPI_SLAVE_CPOL_0 | DLN_SPI_SLAVE_CPHA_0
         Mode 1 => DLN_SPI_SLAVE_CPOL_0 | DLN_SPI_SLAVE_CPHA_1
         Mode 2 => DLN_SPI_SLAVE_CPOL_1 | DLN_SPI_SLAVE_CPHA_0
         Mode 3 => DLN_SPI_SLAVE_CPOL_1 | DLN_SPI_SLAVE_CPHA_1
        """
        self._import_module_in_wrapper(DLN_MODULE_SPI_SLAVE)
        enable = self._dlnSpiSlave.dln_spislave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            self._dlnSpiSlave.dln_spislave_disable(self.__handle, 0, 0)

        res = self._dlnSpiSlave.dln_spislave_get_mode(self.__handle, 0)

        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            enable = self._dlnSpiSlave.dln_spislave_enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0]) and DLN_SUCCEEDED(res[0]), res[1]

    def dln_spislave_set_framesize(self, framesize):
        """
        Set the frame size of the SPI slave
        """
        self._import_module_in_wrapper(DLN_MODULE_SPI_SLAVE)
        enable = self._dlnSpiSlave.dln_spislave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            self._dlnSpiSlave.dln_spislave_disable(self.__handle, 0, 0)

        res = self._dlnSpiSlave.dln_spislave_set_framesize(self.__handle, 0, framesize)

        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            enable = self._dlnSpiSlave.dln_spislave_enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0]) and DLN_SUCCEEDED(res)

    def dln_spislave_get_framesize(self):
        """
        :return the SPI slave port frame size
        """
        self._import_module_in_wrapper(DLN_MODULE_SPI_SLAVE)
        enable = self._dlnSpiSlave.dln_spislave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            self._dlnSpiSlave.dln_spislave_disable(self.__handle, 0, 0)

        res = self._dlnSpiSlave.dln_spislave_get_framesize(self.__handle, 0)

        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            enable = self._dlnSpiSlave.dln_spislave_enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0]) and DLN_SUCCEEDED(res[0]), res[1]

    def dln_spislave_get_supported_framesizes(self):
        self._import_module_in_wrapper(DLN_MODULE_SPI_SLAVE)
        enable = self._dlnSpiSlave.dln_spislave_is_enabled(self.__handle, 0)
        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            self._dlnSpiSlave.dln_spislave_disable(self.__handle, 0, 0)

        res = self._dlnSpiSlave.dln_spislave_get_supported_framesizes(self.__handle, 0)

        if DLN_SUCCEEDED(enable[0]) and enable[1] == 1:
            enable = self._dlnSpiSlave.dln_spislave_enable(self.__handle, 0)
        return DLN_SUCCEEDED(enable[0]) and DLN_SUCCEEDED(res[0]), res[1]

    # global function for work with library

    def __dln_send_message(self, message):
        """
        Sends a specified message (an asynchronous command) to the device.
        :param message - a pointer to a variable that contains a message to be sent.
        """
        result = self._api.DlnSendMessage(message)
        return result

    def __dln_transaction(self, command):
        """
        Sends a synchronous command, waits for a response and returns the response details.
        :param command - a pointer to a variable that contains a command to be sent;
        :param responseBuffer - a pointer to the buffer that receives the response information;
        :param responseBufferSize - the maximum number of bytes to be retrieved.
        """
        response_buffer = create_string_buffer(DLN_MAX_MSG_SIZE)
        result = DLN_RES_FAIL
        if self._api is not None:
            result = self._api.DlnTransaction(command, response_buffer, c_uint16(DLN_MAX_MSG_SIZE))
        return result, response_buffer

    def __dln_get_message(self, handle):
        """
        Retrieves a message (response or event) sent by the device.
        :param handle - a handle to the the DLN device;
        :param messageBuffer - a pointer to the buffer that receives the message information;
        :param messageSize - the maximum number of bytes to be retrieved.
        """
        message_buffer = create_string_buffer(DLN_MAX_MSG_SIZE)
        result = DLN_RES_FAIL
        if self._api is not None:
            result = self._api.DlnGetMessage(handle, message_buffer, c_uint16(DLN_MAX_MSG_SIZE))

        return result, message_buffer
