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
from ErrorHandling.DeviceException import DeviceException


class ISpi():
    """
    Abstract class that defines the interface to be implemented
    by io sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def read_spi_message(self):
        """
        This method read a message from the SPI bus
        :rtype: tuple
        :return: execution status and read result
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def write_spi_message(self, message):
        """
        This method write a message on the SPI bus
        :type message: String
        :param message: the message to send on the SPI bus
        :return: execution status
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def duplex_spi_message(self, message):
        """
        this method read/write a message in full duplex mode
        :type message: String
        :param message: the message to send on the SPI bus
        :rtype: Boolean
        :return: execution status and read result
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_spi_speed(self, speed_hz):
        """
        this method set the speed of the SPI bus
        :type speed_hz: Integer
        :param speed_hz: requested speed in Herz
        :rtype: Boolean
        :return: execution status
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_spi_speed(self):
        """
        get the SPI speed
        :rtype: tuple
        :return: Execution status and bus speed as Integer
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_spi_mode(self, mode):
        """
        set the spi mode to [0-3]
        :type mode: Integer
        :param mode: the requested SPI mode
        :return: execution status
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_spi_mode(self):
        """
        return the current SPI mode
        :rtype: tuple
        :return: execution status and spi mode as Integer
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_spi_bits_per_word(self, bits_per_word):
        """
        set the spi bits per word property
        :type bits_per_word: Integer
        :param bits_per_word: The requested bits per word
        :return: execution status
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_spi_bits_per_word(self):
        """
        return the current SPI bits per word property
        :rtype: tuple
        :return: execution status and spi bpw as Integer
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_spi_lsb_first(self, lsb_first):
        """
        set the spi lsb_first property
        :type lsb_first: Integer
        :param lsb_first: 0 or 1 => true or false
        :return: execution status
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_spi_lsb_first(self):
        """
        return the current SPI lsb_first property
        :rtype: tuple
        :return: execution status and spi bpw
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
