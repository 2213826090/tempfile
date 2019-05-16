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
:summary: This file implements the Modem Flashing interface for an Mfld Android device
:since: 06/09/2012
:author: asebbanx
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class FlashResult(object):

    """
    A class that represent the result of a flash operation.
    This class is used as a facility to use strings in
    order to describe numeric return values.
    """

    def __init__(self):
        """
        Initializes this instance.
        """
        pass

    def to_string(self, numeric_value):
        """
        Returns the C{str} equivalent of the given value.
        C{numeric_value} is expected to be the return value
        of a flash operation.

        :type numeric_value: int
        :param numeric_value: the numeric value corresponding
            to a flash operation result.

        :rtype: str
        :return: the str value corresponding to C{numeric_value}
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def from_string(self, string_value):
        """
        Returns the C{int} equivalent of the given value.
        C{string_value} is expected to be the textual representaion
        of a flash operation return value.

        :type string_value: str
        :param string_value: the str value explaining the flash operation result.

        :rtype: int
        :return: the integer value corresponding to C{string_value}
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)


class IModemFlashing(object):

    """
    :summary: System UEcommands operations for Android platforms.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def flash_nvm_mos(self, nvm_config_file_path, flashing_tty, flashing_app_path, target_directory):
        """
        Flashes the device modem's I{NVM} when in I{MOS} (Main OS) mode.
        In order to flash the modem's {NVM} we need to provide a raw text
        file containing the commands to send.

        The application used to flash is given by the C{flashing_app_path} parameter.

        :param nvm_config_file_path: the path to the raw text file
        :type nvm_config_file_path: str

        :param flashing_tty: the name of the I{tty} to use
        :type flashing_tty: str

        :param flashing_app_path: the absolute path the the flashing application
        :type flashing_app_path: str

        :param target_directory: the absolute path to the
            temporary directory where the flash file will be
            uploaded before the flash operation.
        :type target_directory: str

        :rtype: list
        :return: operation status & output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def flash_nvm_pos(self, nvm_config_file_path, target_directory):
        """
        Flashes the device modem's I{NVM} when in I{POS} (Provisioning OS) mode.
        In order to flash the modem's {NVM} we need to provide a raw text
        file containing the commands to send.

        :param nvm_config_file_path: the path to the raw text file
        :type nvm_config_file_path: str

        :param target_directory: the absolute path to the
            temporary directory where the flash file will be
            uploaded before the flash operation.
        :type target_directory: str

        :rtype: list
        :return: operation status & output log & stdout & stderr
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def flash_fw(self, fw_flash_file_path, flash_timeout, flash_app_path):
        """
        Flashes the device modem's firmware when in I{MOS} (Main OS) mode.
        :param fw_flash_file_path: the path to the flash file
        :type fw_flash_file_path: str

        :param flashing_app_path: the absolute path the the flashing application.
        :type flashing_app_path: str

        :param flash_timeout: the flash timeout
        :type flash_timeout: int

        :rtype: list
        :return: operation status & output log & stdout & stderr
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_at_proxy_from_pos(self):
        """
        Starts the I{AT proxy} tool on the device
        and returns the I{tty} associated to the
        tool.

        Note that when this method is called (and
        the device is booted in I{POS} mode)
        I{AT proxy} is started in I{tunneling} mode.

        We assume that the device is booted in I{POS}
        mode when this method is called.

        :rtype: str
        :return: the name of the I{tty} to use.

        :raise DeviceException: if the command failed.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_at_proxy_from_pos(self):
        """
        Stops the I{AT proxy} tool on the device.

        We assume that the device is booted in I{POS}
        mode when this method is called.

        :rtype: None

        :raise DeviceException: if the command failed.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_at_proxy_from_mos(self, mode):
        """
        Starts the I{AT proxy} tool on the device
        and returns the I{tty} associated to the
        tool.

        We assume that the device is booted in I{MOS}
        mode when this method is called.

        :type mode: int
        :param mode: the at proxy mode (normal or tunneling)

        :rtype: str
        :return: the name of the I{tty} to use.

        :raise DeviceException: if the command failed.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_at_proxy_from_mos(self):
        """
        Stops the I{AT proxy} tool on the device.

        We assume that the device is booted in I{POS}
        mode when this method is called.

        :rtype: None

        :raise DeviceException: if the command failed.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def ping_modem_from_serial(self, serial_instance):
        """
        Pings the modem from the provided C{serial.Serial} instance.
        The C{serial.Serial} instance has to be open when
        calling this method and the provided instance will
        be left untouched (no call to C{close} method).

        :param serial_instance: the C{serial.Serial} instance to/from wich
            the commands and answers will be written/read.
        :type serial_instance: serial.Serial

        :rtype: bool
        :return: C{True} if the ping to the mode succeeded, C{False} otherwise.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def ping_modem(self, tty, baudrate, timeout):
        """
        Pings the modem through the provided C{tty} name
        using the given C{baudrate}.
        This method creates a C{serial.Serial} instance
        then calls the C{ping_modem_from_serial} method.

        :param tty: the I{TTY} name to use in order to talk with the mode.
        :type tty: str

        :param baudrate: the baudrate to use for the serial communication.
        :type baudrate: int

        :param timeout: the timeout to use for each I{read} operation from the mode.
        :type timeout: int

        :rtype: bool
        :return: C{True} if the ping to the mode succeeded, C{False} otherwise.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def change_at_proxy_property_value(self, mode=1):
        """
        Change the at proxy property value and check the result
        Possible modes: 1 Normal Mode ; 2 Tunneling Mode

        We assume that the device is booted in I{MOS}
        mode when this method is called.

        @type mode: int
        @param mode: the at proxy mode (normal or tunneling).
                     Defaults to 1 (normal).

        @rtype: None

        @raise DeviceException: if the command failed.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
