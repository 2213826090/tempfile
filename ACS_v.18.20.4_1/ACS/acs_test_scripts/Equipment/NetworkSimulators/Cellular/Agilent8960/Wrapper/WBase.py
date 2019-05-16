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

:organization: INTEL MCG PSI
:summary: wrapper for basic function of Agilent 8960
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def Connect(eqt, board_id, gpib_address):
    """
    Wraps to Connect function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type board_id: integer
    :param board_id: GPIB board ID
    :type gpib_address: integer
    :param gpib_address: GPIB address (between 0 and 15).
    :rtype: tuple
    :return:
        - long: error code of driver function
        - unsigned long: handle value
        - str: log message
    """
    eqt.get_logger().debug("Connection")
    dll = eqt.get_dll()
    handle = ctypes.c_ulong()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.Connect(
        board_id,
        gpib_address,
        ctypes.byref(handle),
        ctypes.byref(err_msg))
    return err, handle.value, err_msg.value


def Disconnect(eqt):
    """
    Wraps to Disconnect function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().debug("Disconnection")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.Disconnect(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def StartLogging(eqt):
    """
    Wraps to StartLogging function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Start equipment logging option")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.StartLogging(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def StopLogging(eqt):
    """
    Wraps to StopLogging function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Stop equipment logging option")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    logfile = ctypes.c_char_p('\x00' * 256)
    err = dll.StopLogging(handle, ctypes.byref(logfile), ctypes.byref(err_msg))
    return err, err_msg.value


def GetEqId(eqt):
    """
    Wraps to GetEqtId function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: the identification str of the equipment
        - str: log message
    """
    eqt.get_logger().info("Get identification string")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    eq_id = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetEqId(handle, eq_id, ctypes.byref(err_msg))
    return err, eq_id.value, err_msg.value


def PerformFullPreset(eqt):
    """
    Wraps to PerformFullPreset function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Perform full preset")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.PerformFullPreset(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetAppFormat(eqt, app_format):
    """
    Wraps to SetAppFormat function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type app_format: str
    :param app_format: the application format to set
        - "1xEV-DO"
        - "AMPS/136"
        - "GSM/GPRS"
        - "IS-2000/IS-95/AMPS"
        - "IS-856"
        - "WCDMA"
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Set application format to %s", app_format)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetAppFormat(
        handle,
        ctypes.c_char_p(app_format),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetAppFormat(eqt):
    """
    Wraps to GetAppFormat driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: the str representing the format of the current running application.
            Possible values:
                - "1xEV-DO"
                - "AMPS/136"
                - "GSM/GPRS"
                - "IS-2000/IS-95/AMPS"
                - "IS-856"
                - "WCDMA"
        - str: log message
    """
    eqt.get_logger().info("Get application format")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    app_format = ctypes.c_char_p('\x00' * 32)
    err = dll.GetAppFormat(handle, 32, app_format, ctypes.byref(err_msg))
    return err, app_format.value, err_msg.value


def SetCorrectionFrequency(eqt, corr_table):
    """
    Wraps to SetCorrectionFrequency driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type corr_table: str
    :param corr_table: the str of (maximum 20) frequencies between
    292.5 MHz and 2700 MHz to set for frequency correction. Empty table
    sets the state of all frequencies to OFF. The units (GHz, MHz, kHz, Hz)
    are optional. If no units are specified, units default to MHz.
    Resolution: 1 Hz
    :raise: TestEquipmentException if driver function failed
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Set frequency correction")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCorrectionFrequency(
        handle,
        ctypes.c_char_p(corr_table),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCorrectionGain(eqt, corr_table):
    """
    Wraps to SetCorrectionGain driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type corr_table: str
    :param corr_table: the array of (maximum 20) gain values between
    -100 and +100 (dB) to set for gain correction. Empty table sets
    all offsets state to OFF. The units, dB, are optional. If no units
    are specified, units default to dB. Resolution: 0.01 dB
    :raise TestEquipmentException: failed to call driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Set gain correction")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCorrectionGain(
        handle,
        ctypes.c_char_p(corr_table),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCorrectionState(eqt, state):
    """
    Wraps to SetCorrectionFrequency driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: desired state for system correction. Possible values:
        - "ON"
        - "OFF"
    :raise TestEquipmentException: failed to call driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Set %s correction state", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCorrectionState(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetGpibAddress(eqt, gpib_addr):
    """
    Wraps to SetGpibAddress driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type gpib_addr: integer
    :param gpib_addr: the GPIB address to set. An integer
    from 0 to 30.
    :raise TestEquipmentException: failed to call SetGpibAddress driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Set GPIB address to %d", gpib_addr)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetGpibAddress(
        handle,
        ctypes.c_int(gpib_addr),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDebugState(eqt, state):
    """
    Wraps to SetDebugState driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: desired state for the debugger. Possible values:
        - "ON"
        - "OFF"
    :raise TestEquipmentException: failed to call SetDebugState driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Set %s debugger", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDebugState(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetIp4LanAddress(eqt, ip_addr):
    """
    Wraps to SetIp4LanAddress driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip_addr: str
    :param ip_addr: the IP address to set. 15 characters formatted
    as follows: A.B.C.D. The range of values for A = 0 to 126
    and 128 to 223. The range of values for B,C,D = 0 to 255
    (no embedded spaces).
    :raise TestEquipmentException: failed to call SetIp4LanAddress driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Set IPv4 address to %s", ip_addr)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetIp4LanAddress(
        handle,
        ctypes.c_char_p(ip_addr),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetIp4LanAddress2(eqt, ip_addr):
    """
    Wraps to SetIp4LanAddress2 driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip_addrr: str
    :param ip_addr: the IP address to set. 15 characters formatted
    as follows: A.B.C.D. The range of values for A = 0 to 126
    and 128 to 223. The range of values for B,C,D = 0 to 255
    (no embedded spaces).
    :raise TestEquipmentException: failed to call SetIp4LanAddress2 driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Set IPv4 address 2 to %s", ip_addr)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetIp4LanAddress2(
        handle,
        ctypes.c_char_p(ip_addr),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetIp4DefaultGateway(eqt, gateway):
    """
    Wraps to SetIp4DefaultGateway driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type gateway: str
    :param gateway: the gateway to set. 15 characters formatted
    as follows: A.B.C.D where A = 0 to 223 B,C,D = 0 to 255
    (no embedded spaces).
    :raise TestEquipmentException: failed to call SetIp4DefaultGateway driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Set default gateway to %s", gateway)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetIp4DefaultGateway(
        handle,
        ctypes.c_char_p(gateway),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetIp4SubnetMask(eqt, mask):
    """
    Wraps to SetIp4SubnetMask driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type mask: str
    :param mask: the subnet mask to set. 15 characters formatted
    as follows: A.B.C.D where A,B,C,D are between = 0 to 255
    (no embedded spaces).
    :raise TestEquipmentException: failed to call SetIp4SubnetMask driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().info("Set subnet mask to %s", mask)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetIp4SubnetMask(
        handle,
        ctypes.c_char_p(mask),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetExternalDeviceConnectionStatus(eqt, device_number):
    """
    Wraps to GetExternalDeviceConnectionStatus driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type device_number: integer
    :param device_number: Number of the device to get connection status
    (1 or 2 because 8960 manages max 2 devices)
    :raise TestEquipmentException: failed to call GetExternalDeviceConnectionStatus driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: The connection status of the device to check (1 or 2)
                    Status :
                    - CONG : Connecting
                    - CONN : Connected
                    - DISC : Disconnecting
                    - NCON : Not Connected
        - str: error message of the driver function
    """
    eqt.get_logger().debug("Get external connection status for device %d",
                           device_number)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    conn_status = ctypes.c_char_p('\x00' * 32)
    err = dll.GetExternalDeviceConnectionStatus(
        handle,
        device_number,
        conn_status,
        ctypes.byref(err_msg))
    return err, conn_status.value, err_msg.value


def GetExternalDeviceConnectedIpAddress(eqt, device_number):
    """
    Wraps to GetExternalDeviceConnectedIpAddress driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type device_number: integer
    :param device_number: Number of the device to get connection status
    (1 or 2 because 8960 manages max 2 devices)
    :raise TestEquipmentException: failed to call GetExternalDeviceConnectedIpAddress driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: The connected device IP address
        - str: error message of the driver function
    """
    eqt.get_logger().debug("Get external connection status for device %d",
                           device_number)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ip = ctypes.c_char_p('\x00' * 32)
    err = dll.GetExternalDeviceConnectedIpAddress(
        handle,
        device_number,
        ip,
        ctypes.byref(err_msg))
    return err, ip.value, err_msg.value


def SetExternalIpAddress(eqt, ip):
    """
    Wraps to SetExternalIpAddress driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip: str
    :param ip: The external IP address to set
    :raise TestEquipmentException: failed to call SetExternalIpAddress driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set external IP address to %s", ip)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetExternalIpAddress(
        handle,
        ctypes.c_char_p(ip),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConnectToExternalDevice(eqt):
    """
    Wraps to ConnectToExternalDevice driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    device if good connection parameters are set
    :raise TestEquipmentException: failed to call ConnectToExternalDevice driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Connect to external device")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConnectToExternalDevice(
        handle,
        ctypes.byref(err_msg))
    return err, err_msg.value


def DisconnectFromExternalDevice(eqt):
    """
    Wraps to DisconnectFromExternalDevice driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: failed to call DisconnectFromExternalDevice
    driver function
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Disconnect from external device")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.DisconnectFromExternalDevice(
        handle,
        ctypes.byref(err_msg))
    return err, err_msg.value


def SendCommand(eqt, command):
    """
    Wraps to SendCommand driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type command: str
    :param command: the gpib command to execute

    :raise TestEquipmentException: failed to execute gpib command.
    :rtype: tuple
    :return:
        - long: error code of driver function
        - str: log message
    """
    eqt.get_logger().debug("Send GPIB command: %s", command)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SendCommand(
        handle,
        ctypes.c_char_p(command),
        ctypes.byref(err_msg))
    return err, err_msg.value
