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
:summary: wrapper for Diolan IO adapter module definitions, import from the C header file
:since: 08/08/2014
:author: dpierrex
"""
from ctypes import Structure, c_uint16

HDLN_ALL_DEVICES = 0
HDLN_INVALID_HANDLE = 0xFFFF

DLN_MAX_MSG_SIZE = 288

DLN_DEFAULT_SERVER_PORT = 9656
"""
Commands group and macros
"""
DLN_MODULE_GENERIC = 0x00  # Common commands
DLN_MODULE_GPIO = 0x01  # Command for GPIO module
DLN_MODULE_SPI_MASTER = 0x02  # Command for SPI module
DLN_MODULE_I2C_MASTER = 0x03  # Command for I2C module
DLN_MODULE_LED = 0x04  # Command for LED module
DLN_MODULE_BOOT = 0x05  # Command for BOOT module
DLN_MODULE_ADC = 0x06  # Command for ADC module
DLN_MODULE_PWM = 0x07  # Command for PWM module
DLN_MODULE_FREQ = 0x08  # Command for Freq. counter module
DLN_MODULE_I2S = 0x09
DLN_MODULE_SDIO = 0x0A  # Command for SDIO module
DLN_MODULE_SPI_SLAVE = 0x0B
DLN_MODULE_I2C_SLAVE = 0x0C
DLN_MODULE_PLS_CNT = 0x0D  # Command for pulse counter module
DLN_MODULE_UART = 0x0E
DLN_MODULE_SPI_SLAVE_SYNC = 0x0F
DLN_MODULE_I2C_EEPROM = 0x10
DLN_MODULE_SPI_EEPROM = 0x11
DLN_MODULE_SPI_FLASH = 0x12
DLN_MODULE_I2C_DATAFLASH = 0x13
DLN_MODULE_ANALYZER = 0x14

DLN_MSG_MODULE_POSITION = 8  # bit position of group code


def DLN_BUILD_MSG_ID(id, module):
    """

    :param id: message id
    :param module: diolan module
    :return: True or False
    """
    return (id) | ((module) << DLN_MSG_MODULE_POSITION)


class DLN_MSG_HEADER(Structure):
    """
    struct DLN_MSG_HEADER
    The message header is the first field of each message, sent from a host to a device or vice versa.
    It is used to identify and route the message correctly.
    """
    _pack_ = 1
    _fields_ = [("size", c_uint16),  # The size of the message.
                ("msgId", c_uint16),  # The code defining the message.
                ("echoCounter", c_uint16),  # Used to establish a one-one link between a command/response pair.
                # In case the message is an event, this is a freerunning counter.
                ("handle", c_uint16)]  # A handle to the DLN device.


class DLN_BASIC_RSP(Structure):
    """
    struct DLN_BASIC_RSP
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER),
                ("result", c_uint16)]


class DLN_BASIC_CMD(Structure):
    """
    struct DLN_BASIC_CMD
    """
    _pack_ = 1
    _fields_ = [("header", DLN_MSG_HEADER)]
