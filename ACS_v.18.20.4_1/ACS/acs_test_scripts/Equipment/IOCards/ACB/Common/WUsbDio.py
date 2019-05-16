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
:summary: wrapper for USB DIO devices
:author: ssavrimoutou
:since: 22/11/2010
"""

import ctypes
from ErrorHandling.TestEquipmentException import TestEquipmentException


def Connect(eqt, serial_number=None):
    """
    Do the connection and retrieve device index.
    The connection will be done on first usb_dio device connected
    on the computer (don't support multiple usb_dio device)
    :type eqt: DllLoader
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the device index of the connected equipment, -1 if no device found
    :raise: raises TestEquipmentException (error code, error message) in case of failure
    """
    dll = eqt.get_dll()
    # Get directly device if the serial number is passed as input
    if serial_number not in [None, ""]:
        eqt.get_logger().info(
            "Connecting to device with serial number : %s",
            serial_number)
        device_index = \
            dll.GetDeviceBySerialNumber(ctypes.c_ulonglong(serial_number))

    # In others cases the device index is the first one found
    else:
        eqt.get_logger().info(
            "No serial number given... Connecting to first found device !")
        devices = dll.GetDevices()

        if devices != 0:
            device_index = 0
            while device_index < 31:
                if (1 << device_index) & devices:
                    # Get one device, this is the one
                    eqt.get_logger().info(
                        "Connection to device %s",
                        str(device_index))
                    break
                device_index += 1

        else:
            device_index = -1

    return device_index


def Disconnect(eqt):
    """
    Resets device index
    :type eqt: ACBN or ACBE or ACBP
    :param eqt: the equipment that uses the wrapper
    """
    device_index = eqt.get_dev_idx()
    dll = eqt.get_dll()
    if device_index != -1:
        dll.ClearDevices()
        eqt.get_logger().info("Disconnection from device %s", device_index)


def ShowInfo(eqt):
    """
    Displays device info
    :type eqt: ACBN or ACBE or ACBP
    :param eqt: the equipment that uses the wrapper
    :type device_index: integer
    :param device_index: the device index of the equipment
    """
    device_index = eqt.get_dev_idx()
    dll = eqt.get_dll()
    eqt.get_logger().info("status:")
    if device_index != -1:
        pid = ctypes.c_ulong()

        buffer_size = 254
        name_buffer_size = ctypes.c_ulong(buffer_size)
        name = ctypes.create_string_buffer('\000' * buffer_size)
        dio_bytes_supported = ctypes.c_ulong()
        counters_available = ctypes.c_ulong()
        device_index = ctypes.c_ulong(device_index)

        dll.QueryDeviceInfo(device_index,  # DeviceIndex
                            ctypes.byref(pid),  # PID
                            ctypes.byref(name_buffer_size),  # size of name buffer
                            ctypes.byref(name),  # name
                            ctypes.byref(dio_bytes_supported),  # DIO bytes # supported
                            ctypes.byref(counters_available))  # nb of counters available

        log = eqt.get_logger()
        log.info("           Device connected: %s", str(device_index))
        log.info("           Name: %s", repr(name.value))
        log.info("           PID: 0x%X", pid.value)
        log.info("           Nb DIO bytes: %s", str(dio_bytes_supported.value))
        log.info("           Nb counters: %s", str(counters_available.value))


def __WriteBit(eqt, bit, enable):
    """
    Write single bit on USBDIO
    :type eqt: ACBN or ACBE or ACBP
    :param eqt: the equipment that uses the wrapper
    :type bit: unsigned long
    :param bit:
    :type enable: boolean
    :param enable:
    """
    dll = eqt.get_dll()
    device_index = eqt.get_dev_idx()
    # bit_index = ctypes.c_ulong(int(bit, 16))
    bit_index = ctypes.c_ulong(bit)
    value = ctypes.c_bool(enable)
    dll.DIO_Write1(device_index, bit_index, value)


def Enable(eqt, line, log=True):
    """
    Enable line
    :type eqt: ACBN or ACBE or ACBP
    :param eqt: the equipment that uses the wrapper
    :type log: boolean
    :param log: disable the log if log = False
    :type line:
    :param line:
    """
    device_index = eqt.get_dev_idx()
    if device_index == -1:
        eqt.get_logger().error("No device connected!")
        raise TestEquipmentException(
            TestEquipmentException.CONNECTION_ERROR,
            "No device connected!")

    line_pos = line.Value
    if line_pos != -1:
        __WriteBit(eqt, line_pos, True)
        if log:
            eqt.get_logger().debug("           Enable line: %s", str(line))
    elif line_pos == -1:
        eqt.get_logger().error("Try to enable unknown line: %s", str(line))


def Disable(eqt, line, log=True):
    """
    Disable line
    :type eqt: ACBN or ACBE or ACBP
    :param eqt: the equipment that uses the wrapper
    :type log: boolean
    :param log: disable the log if log = False
    :type line:
    :param line:
    """
    device_index = eqt.get_dev_idx()
    if device_index == -1:
        raise TestEquipmentException(
            TestEquipmentException.CONNECTION_ERROR,
            "No device connected!")

    line_pos = line.Value
    if line_pos != -1:
        __WriteBit(eqt, line_pos, False)
        if log:
            eqt.get_logger().debug("           Disable line: %s", str(line))
    elif line_pos == -1:
        eqt.get_logger().error("Try to disable unknown line: %s", str(line))


def __ReadBit(eqt, bit):
    """
    Read single bit on USBDIO
    :type eqt: ACBN or ACBE or ACBP
    :param eqt: the equipment that uses the wrapper
    :type bit: unsigned long
    :param bit:
    :rtype:
    :return:
    """
    dll = eqt.get_dll()
    device_index = eqt.get_dev_idx()
    # bit_index = ctypes.c_ulong(int(bit, 16))
    bit_index = ctypes.c_ulong(bit)
    value = ctypes.c_byte()

    dll.DIO_Read1(device_index, bit_index, ctypes.byref(value))
    return value.value


def GetState(eqt, line):
    """
    Get line state
    :type eqt: ACBN or ACBE or ACBP
    :param eqt: the equipment that uses the wrapper
    :type line:
    :param line:
    :rtype:
    :return:
    """
    device_index = eqt.get_dev_idx()
    if device_index == -1:
        raise TestEquipmentException(
            TestEquipmentException.CONNECTION_ERROR,
            "No device connected!")

    line_enable = False
    line_pos = line.Value

    if line_pos != -1:
        if __ReadBit(eqt, line_pos) != 0:
            line_enable = True
    else:
        eqt.get_logger().error("Try to read state of an unknown line: %s",
                               str(line))

    return line_enable


def Reset(eqt):
    """
    Set all lines to output & to off
    :type eqt: ACBN or ACBE or ACBP
    :param eqt: the equipment that uses the wrapper
    """
    dll = eqt.get_dll()
    device_index = eqt.get_dev_idx()
    if device_index == -1:
        raise TestEquipmentException(
            TestEquipmentException.CONNECTION_ERROR,
            "No device connected!")

    tristate = ctypes.c_bool(False)

    mask = 0
    mask |= 1 << 0  # Put 1st slot (of 8 bit) to output
    mask |= 1 << 1  # Put 2sd slot (of 8 bit) to output
    mask |= 1 << 2  # Put 3td slot (of 8 bit) to output
    mask |= 1 << 3  # Put 4th slot (of 8 bit) to output
    #    print("out mask  = " + dec2bin(mask, 32))
    out_mask = ctypes.c_byte(mask)
    data = 0x00000000
#    data = 0xFFFFFFFF
# data = data | (0xFF << (0 * 8)) # Set 1st slot output byte high
# data = data | (0xFF << (1 * 8)) # Set 2sd slot output byte high
# data = data | (0xFF << (2 * 8)) # Set 3td slot output byte high
# data = data | (0xFF << (3 * 8)) # Set 4th slot output byte high
    dio_data = ctypes.c_ulong(data)

    dll.DIO_Configure(device_index, tristate,
                      ctypes.byref(out_mask), ctypes.byref(dio_data))

    # TO DO: to debug selective tristate line setup
    # the function DIO_ConfigureEx doesn't update the line status?!?
    # Disable tristate for our lines
#    mask = 0xFFFFFFFF
#    for line in self.LINES:
#        mask &= ~(1 << line.Value)

#        print("tristate mask  = " + dec2bin(mask))
#        tristate_mask = ctypes.c_ulong(mask)
#
#        dll.DIO_ConfigureEx(self.__device_index,
#            ctypes.byref(out_mask),
#            ctypes.byref(dio_data),
#            ctypes.byref(tristate_mask))

#        rtristate_mask = ctypes.c_ulong()
#        rout_mask = ctypes.c_byte()
#        dll.DIO_ConfigurationQuery(self.__device_index,
#            ctypes.byref(rout_mask), ctypes.byref(rtristate_mask))
#        print("rout mask = " + dec2bin(rout_mask.value, 32))
#        print("rtristate mask = " + dec2bin(rtristate_mask.value, 32))

#        self.__write_bit(0, False)
#        self.__write_bit(0, True)

#        self.disable(self.LINES.battery)
#        self.enable(self.LINES.battery)

# Disable all the lines
#        print("Disable used lines:")
#        for line in self.LINES:
#            self.disable(line)

    # Init all the line to the good status
    off_state = 0x00000000
#    off_state |= (1 << self.LINES.batt_id_glitch.Value)
    # set to inactive status (1 and not 0), ready to glitch
    off_state = ctypes.c_ulong(off_state)
    dll.DIO_WriteAll(device_index,
                     ctypes.byref(off_state))
