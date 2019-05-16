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
:summary: This file implements low speed IO UECmds
:since: 02/09/2014
:author: dpierrex
"""
from acs_test_scripts.Device.UECmd.Interface.LowSpeedIO.ISpi import ISpi
from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base
from ErrorHandling.DeviceException import DeviceException


class Spi(Base, ISpi):
    """
    :summary: Ipi UEcommands operations for Linux platforms
    """

    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        ISpi.__init__(self, device)
        self.__spi_python_message_cmd = 'python -c "from spi import SPIDevice, reading, writing, duplex; ' \
                                     'ret = SPIDevice(1,5).transaction({0}); ' \
                                     'print ret"'

    def read_spi_message(self, byte_count):
        """
        This method read a message from the SPI bus
        !! doesn't work on Edioson !! use duplex
        :return: execution status and read result
        """
        cmd = self.__spi_python_message_cmd.format("reading({0})".format(byte_count))
        return self._internal_exec(cmd)

    def write_spi_message(self, message):
        """
        This method write a message on the SPI bus
        :return: execution status
        """
        cmd = self.__spi_python_message_cmd.format("writing(\'{0}\')".format(message))
        return self._internal_exec(cmd)


    def duplex_spi_message(self, message):
        """
        this method read/write a message in full duplex mode
        :param message:
        :return: execution status and read result
        """
        cmd = self.__spi_python_message_cmd.format("duplex(\'{0}\')".format(message))
        return self._internal_exec(cmd)

    def set_spi_speed(self, speed_hz):
        """
        this method set the speed of the SPI bus
        :param speed_hz: requested speed
        :return: execution status
        """
        cmd = "python -c \"from spi import SPIDevice; " \
              "SPIDevice(1,5).speed_hz = {0};\"".format(speed_hz)
        return self._internal_exec(cmd)

    def get_spi_speed(self):
        """
        get the SPI speed
        :return: bus speed in herz
        """
        cmd = "python -c \"from spi import SPIDevice; " \
              "print SPIDevice(1,5).speed_hz\""
        return self._internal_exec(cmd)

    def set_spi_mode(self, mode):
        """
        set the spi mode to [0-3]
        :param mode:
        :return: execution status
        """
        cmd = "python -c \"from spi import SPIDevice; " \
              "SPIDevice(1,5).clock_mode = {0}\"".format(mode)
        return self._internal_exec(cmd)

    def get_spi_mode(self):
        """
        return the current SPI mode
        :return: execution status and spi mode
        """
        cmd = "python -c \"from spi import SPIDevice; " \
              "print SPIDevice(1,5).clock_mode\""
        return self._internal_exec(cmd)

    def set_spi_bits_per_word(self, bits_per_word):
        """
        set the spi bits per word property
        :param mode:
        :return: execution status
        """
        cmd = "python -c \"from spi import SPIDevice; " \
              "SPIDevice(1,5).bits_per_word = {0}\"".format(bits_per_word)
        return self._internal_exec(cmd)

    def get_spi_bits_per_word(self):
        """
        return the current SPI bits per word property
        :return: execution status and spi bpw
        """
        cmd = "python -c \"from spi import SPIDevice; " \
              "print SPIDevice(1,5).bits_per_word\""
        return self._internal_exec(cmd)

    def set_spi_lsb_first(self, lsb_first):
        """
        set the spi lsb_first property
        :param mode:
        :return: execution status
        """
        cmd = "python -c \"from spi import SPIDevice; " \
              "SPIDevice(1,5).lsb_first = {0}\"".format(lsb_first)
        return self._internal_exec(cmd)

    def get_spi_lsb_first(self):
        """
        return the current SPI lsb_first property
        :return: execution status and spi lsb first
        """
        cmd = "python -c \"from spi import SPIDevice; " \
              "print SPIDevice(1,5).lsb_first\""
        return self._internal_exec(cmd)
