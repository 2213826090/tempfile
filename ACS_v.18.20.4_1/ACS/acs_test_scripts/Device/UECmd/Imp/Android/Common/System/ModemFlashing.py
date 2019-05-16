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
:summary: This file implements the Modem Flashing UEcmds for an Mfld Android device
:since: 06/09/2012
:author: asebbanx
"""

import os
import serial
import subprocess
import sys
import tempfile
import time

from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.Utilities import internal_shell_exec
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.IModemFlashing import IModemFlashing, FlashResult
from acs_test_scripts.Device.UECmd.UECmdDecorator import need, atproxy
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException

# Import winreg only when running on Windows system
if "win" in sys.platform.lower():
    import _winreg


class ModemFlashing(Base, IModemFlashing):

    """
    :summary: Modem Flashing UEcommands operations for Android platforms.
    """

    AT_PROXY_NORMAL_MODE = 1
    """
    The C{integer} value corresponding to AT proxy
    in normal mode.
    """

    AT_PROXY_TUNNELING_MODE = 2
    """
    The C{integer} value corresponding to AT proxy
    in tunneling mode.
    """

    AT_PROXY_PROPERTY_NAME = "persist.system.at-proxy.mode"
    """
    The name of the I{Android} system property that steers
    AT proxy's behavior.
    """

    AT_PROXY_TTY = "/dev/ttyACM0"
    """
    The name of the AT proxy TTY.
    """

    @need('modem')
    def __init__(self, device):
        """
        Constructor.
        """
        Base.__init__(self, device)
        IModemFlashing.__init__(self, device)
        self._logger = device.get_logger()
        self._serial_number = device.get_serial_number()
        self._at_proxy_supported = device.get_config("isATProxySupported", "True", "str_to_bool")
        self._at_cmd_port = device.get_config("ATCmdPort", "True", "str_to_bool")

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

        :param flashing_app_path: (optional) the absolute path the the flashing application.
        :type flashing_app_path: str

        :param target_directory: (optional) the absolute path to the
            temporary directory where the flash file will be
            uploaded before the flash operation.
        :type target_directory: str

        :rtype: list
        :return: operation status & output log
        """
        # Compute the file to the NVM config file on the device
        file_name = os.path.basename(nvm_config_file_path)
        target_file_path = "%s/%s" % (target_directory, file_name)

        # Push the file on the target
        # self._device.push(nvm_config_file_path, target_file_path, 10)
        command = "adb push %s %s" % (nvm_config_file_path, target_file_path)
        self._logger.debug("Pushing NVM file to target: %s" % command)
        (fd_out, outname) = tempfile.mkstemp()
        (fd_err, errname) = tempfile.mkstemp()
        process = subprocess.Popen(
            args=command,
            shell=True,
            stdout=fd_out,
            stderr=fd_err,
        )
        # pylint: disable=E1101
        exit_code = process.wait()
        os.close(fd_out)
        os.close(fd_err)
        output = file(outname).read()
        errors = file(errname).read()
        os.unlink(outname)
        os.unlink(errname)
        sys.stdout.write(output)
        sys.stderr.write(errors)
        if exit_code != 0:
            return exit_code, output, output, errors

        # Flash the modem
        # pylint: disable=W0212
        exec_status, output = internal_shell_exec(
            "adb shell %s -p %s -k %s" % (flashing_app_path, flashing_tty, target_file_path),
            30)
        # pylint: enable=W0212

        # Check whether previous step completed successfully
        if exec_status == Global.FAILURE:
            message = "Could not flash modem's NVM: %s" % output
            self._logger.error("%s" % message)
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, message)

        # Return the result
        # When using _internal_shell_exec_command the stderr
        # is concatenated to sdtout. We simply return stdout
        # twice.
        return exec_status, output, output, output

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

        :rtype: tuple
        :return: operation status & output log & stdout & stderr
        """
        # Compute the file to the NVM config file on the device
        file_name = os.path.basename(nvm_config_file_path)
        target_file_path = "%s/%s" % (target_directory, file_name)

        # Push the configuration file to the target
        # (adb not available in POS)
        # pylint: disable=W0212
        exec_status, output = internal_shell_exec(
            "fastboot flash %s %s" % (target_file_path, nvm_config_file_path),
            10)
        # pylint: enable=W0212

        # Check whether previous step completed successfully
        if exec_status != NvmFastbootResult.NVM_ERR_SUCESS:
            message = "Could not push configuration file to target: %s" % output
            self._logger.error("%s" % message)
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, message)

        # Run Intel's OEM hooks to flash the modem's NVM
        exec_status, stdout = internal_shell_exec("fastboot oem nvm apply %s" % target_file_path, 30)  # pylint: disable=W0212

        # Return the result
        # When using _internal_shell_exec_command the stderr
        # is concatenated to sdtout. We simply return stdout.
        return exec_status, stdout

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
        # Flash the modem
        command = "adb shell %s -f %s" % (
            str(flash_app_path),
            str(fw_flash_file_path))
        exec_status, stdout = internal_shell_exec(command, flash_timeout)  # pylint: disable=W0212

        # Check whether previous step completed successfully
        if exec_status == Global.FAILURE:
            message = "Could not flash modem's firmware: %s" % stdout
            self._logger.error("%s" % message)
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, message)

        # Return the result
        # When using _internal_shell_exec_command the stderr
        # is concatenated to sdtout. We simply return stdout.
        return exec_status, stdout

    @atproxy
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
        # Inform user that the next command will fail
        # but that is what we expect.
        self._logger.info("The command that stops AT proxy always fail. The coming error is expected.")

        # Run Intel's OEM hooks to flash the modem's NVM
        _exec_status, stdout = internal_shell_exec("fastboot oem proxy start", 30)  # pylint: disable=W0212

        # We expect a failure but let's check the
        # output content just to make sure that the
        # error is actually the one we expect.
        expected_error = "FAILED (status read failed (No such device))"
        if stdout.find(expected_error) == -1:
            message = "Unexpected failure cause when starting AT proxy: %s" % \
                      stdout
            self._logger.error(message)
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, message)

        # Return the TTY
        return ModemFlashing.AT_PROXY_TTY

    @atproxy
    def stop_at_proxy_from_pos(self):
        """
        Stops the I{AT proxy} tool on the device.

        We assume that the device is booted in I{POS}
        mode when this method is called.

        :rtype: None

        :raise DeviceException: if the command failed.
        """
        # Inform user that the next command will fail
        # but that is what we expect.
        self._logger.info("The command that stops AT proxy always fail. The coming error is expected.")

        # Run Intel's OEM hooks to flash the modem's NVM
        _exec_status, stdout = internal_shell_exec("fastboot oem proxy stop", 30)  # pylint: disable=W0212

        # We expect a failure but let's check the
        # output content just to make sure that the
        # error is actually the one we expect.
        expected_error = "FAILED (status read failed (No such device))"
        if stdout.find(expected_error) == -1:
            message = "Unexpected failure cause when starting AT proxy: %s" % \
                      stdout
            self._logger.error(message)
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, message)

    @atproxy
    def start_at_proxy_from_mos(self, mode=1):
        """
        Starts the I{AT proxy} tool on the device
        and returns the I{tty} associated to the
        tool.

        We assume that the device is booted in I{MOS}
        mode when this method is called.

        :type mode: int
        :param mode: the at proxy mode (normal or tunneling).
            Defaults to 1 (normal).

        :rtype: str
        :return: the name of the I{tty} to use.

        :raise DeviceException: if the command failed.
        """
        # Check the mode parameter's value
        if mode not in (ModemFlashing.AT_PROXY_NORMAL_MODE, ModemFlashing.AT_PROXY_TUNNELING_MODE):
            message = "Unsupported value: %s" % (str(mode))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        if sys.platform == "linux2":
            # Change the AT proxy property value
            self.change_at_proxy_property_value(mode)
            # Return the AT proxy TTY
            return ModemFlashing.AT_PROXY_TTY
        else:
            if sys.platform == "win32":
                # check if AT proxy is already started
                self._logger.info("Check AT proxy state")
                property_value = self._device.get_property_value(ModemFlashing.AT_PROXY_PROPERTY_NAME)
                self._logger.info("property_value = " + str(property_value))
                if property_value is None:
                    raise TestEquipmentException(TestEquipmentException.UNKNOWN_EQT, "Could not retrieve AT proxy mode value (ADB failure).")
                if str(property_value) != "":
                    if int(property_value) in (ModemFlashing.AT_PROXY_NORMAL_MODE, ModemFlashing.AT_PROXY_TUNNELING_MODE):
                        self._logger.info("AT Proxy will be restarted to retrieve the Win COM port")
                        self.stop_at_proxy_from_mos()
                        time.sleep(10)
                    else:
                        self._logger.info("AT Proxy is not started")
                else:
                    self._logger.info("AT Proxy is not started")
                # Start AT proxy, according to LAUNCH_MODE
                self._logger.info("Starting AT proxy")

                # Check initial Ports in Windows before starting DUT AT proxy
                initial_win_com_list = self._enumerate_windows_ports()

                # Change the AT proxy property value
                self.change_at_proxy_property_value(mode)

                # List Windows Com Ports after AT proxy is started
                win_com_list_ports_after_proxy_start = self._enumerate_windows_ports()

                if len(initial_win_com_list) < len(win_com_list_ports_after_proxy_start):
                    self.at_proxy_com_port = win_com_list_ports_after_proxy_start[len(initial_win_com_list)]
                else:
                    raise TestEquipmentException(TestEquipmentException.UNKNOWN_EQT, "AT Proxy Windows Virtual COM Port could not be created")
                return self.at_proxy_com_port
            else:
                raise TestEquipmentException(TestEquipmentException.UNKNOWN_EQT, "Unknown PC Operation System")

    @atproxy
    def stop_at_proxy_from_mos(self):
        """
        Stops the I{AT proxy} tool on the device.

        We assume that the device is booted in I{MOS}
        mode when this method is called.

        :rtype: None

        :raise DeviceException: if the command failed.
        """
        # Change the AT proxy property value
        self.change_at_proxy_property_value(0)

        # ADB shell always exits with 0,
        # so we check the AT proxy property value
        # to make sure our command succeeded.
        time.sleep(10)
        property_value = self._device.get_property_value(
            ModemFlashing.AT_PROXY_PROPERTY_NAME)
        property_value = str(property_value)
        if property_value != "0":
            message = "Property value %s was not changed. Expected 0, got %s." % (
                      ModemFlashing.AT_PROXY_PROPERTY_NAME,
                      property_value)
            raise DeviceException(DeviceException.OPERATION_FAILED, message)

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
        # Log a simple message
        self._logger.debug("Pinging the modem")
        # Initialize the return value
        read_something = False
        # Initialize local variables
        max_tries = 4
        current_try = 0
        current_line = None
        # Send the 'at' command
        serial_instance.write("at\r\n")
        while current_try < max_tries:
            # Try and read a line from the modem
            current_try += 1
            current_line = serial_instance.readline()
            current_line = current_line.translate(None, "\r\n")
            # Try to find at least one line with including
            # a token answer
            if current_line == "OK":
                read_something = True
        # Return the ping status
        return read_something

    def ping_modem(self, tty, baudrate, timeout=2):
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
            Defaults to 2.
        :type timeout: int

        :rtype: bool
        :return: C{True} if the ping to the mode succeeded, C{False} otherwise.
        """

        # Initialize the serial instance
        # and open the serial port
        serial_instance = serial.Serial(
            port=tty,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout)
        # Pint the modem
        ping_status = self.ping_modem_from_serial(serial_instance)
        # Close the serial port
        serial_instance.close()

        # Return the ping status
        return ping_status

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
        # Change the AT proxy property value
        command = "adb -s %s shell setprop %s %s" % (
            self._serial_number,
            ModemFlashing.AT_PROXY_PROPERTY_NAME,
            str(mode))
        internal_shell_exec(command, 5)  # pylint: disable=W0212

        # ADB shell always exits with 0,
        # so we check the AT proxy property value
        # to make sure our command succeeded.
        time.sleep(10)
        property_value = self._device.get_property_value(ModemFlashing.AT_PROXY_PROPERTY_NAME)
        property_value = str(property_value)
        if property_value != str(mode):
            message = "Property value %s was not changed. Expected %s, got %s." % (
                      ModemFlashing.AT_PROXY_PROPERTY_NAME,
                      str(mode),
                      property_value)
            raise DeviceException(DeviceException.OPERATION_FAILED, message)
        else:
            return

    def _get_nvm_fastboot_result(self):
        """
        Returns a new C{NvmFastbootResult} instance.

        Note that this method is not exposed in the interface
        as it is tightly linked to a specific
        C{FlashResult} implementation.

        :rtype: NvmFastbootResult
        :return: a new C{NvmFastbootResult} instance.
        """
        return NvmFastbootResult()

    def _get_cmfwdl_result(self):
        """
        Returns a new C{CmfwdlResult} instance.

        Note that this method is not exposed in the interface
        as it is tightly linked to a specific
        C{FlashResult} implementation.

        :rtype: CmfwdlResult
        :return: a new C{CmfwdlResult} instance.
        """
        return CmfwdlResult()

    def _get_fw_fastboot_result(self):
        """
        Returns a new C{FwFastbootResult} instance.

        Note that this method is not exposed in the interface
        as it is tightly linked to a specific
        C{FlashResult} implementation.

        :rtype: FwFastbootResult
        :return: a new C{FwFastbootResult} instance.
        """
        return FwFastbootResult()

    def _enumerate_windows_ports(self):
        """
        Returns a list with the COM ports available on Windows System.

        Note that this method is not exposed in the interface
        as it is tightly linked to a specific
        C{ModemFlashing} implementation.

        @rtype: list
        @return: list of current available COM ports on Windows System
        """
        ports = []
        # Iterate through registry because WMI does not show virtual serial ports
        try:
            key = _winreg.OpenKey(_winreg.HKEY_LOCAL_MACHINE, r'HARDWARE\DEVICEMAP\SERIALCOMM')
        except WindowsError:
            return []
        i = 0
        while True:
            try:
                ports.append(str(_winreg.EnumValue(key, i)[1]))
                i = i + 1
            except WindowsError:
                # return a list of current available COM ports on Windows System
                return ports
        # if no COM ports available, return empty list
        return []


class NvmFastbootResult(FlashResult):

    """
    A class that represent the result of a flash operation.
    This class is used as a facility to use strings in
    order to describe numeric return values.
    This class is the implementation of I{NVM} flash operation
    when using 'fastboot'.
    """

    NVM_ERR_SUCESS = 0
    """
    The return code corresponding to a success.
    """

    NVM_ERR_ERROR = -1
    """
    The return code corresponding to a failure.
    """

    def __init__(self):
        """
        Initializes this instance.
        """
        FlashResult.__init__(self)

    def to_string(self, numeric_value):
        """
        Returns the C{str} equivalent of the given value.
        C{numeric_value} is expected to be the return value
        of a flash operation.

        :type numeric_value: int
        :param numeric_value: the numeric value corresponding
            to a flash operation result.

        :rtype: str
        :return: the str value corresponding to C{numeric_value} or C{None}.
        """
        for k, v in vars(self.__class__).iteritems():
            if v == numeric_value:
                return k
        return None

    def from_string(self, string_value):
        """
        Returns the C{int} equivalent of the given value.
        C{string_value} is expected to be the textual representaion
        of a flash operation return value.

        :type string_value: str
        :param string_value: the str value explaining the flash operation result.

        :rtype: int
        :return: the integer value corresponding to C{string_value} or C{None}
        """
        upper_case_value = str(string_value).upper()
        return getattr(self, upper_case_value, None)


class FwFastbootResult(FlashResult):

    """
    A class that represent the result of a flash operation.
    This class is used as a facility to use strings in
    order to describe numeric return values.
    This class is the implementation of firmware flash operation
    when using 'fastboot'.
    """

    FW_ERR_SUCESS = 0
    """
    The return code corresponding to a success.
    """

    FW_ERR_ERROR = -1
    """
    The return code corresponding to a failure.
    """

    def __init__(self):
        """
        Initializes this instance.
        """
        FlashResult.__init__(self)

    def to_string(self, numeric_value):
        """
        Returns the C{str} equivalent of the given value.
        C{numeric_value} is expected to be the return value
        of a flash operation.

        :type numeric_value: int
        :param numeric_value: the numeric value corresponding
            to a flash operation result.

        :rtype: str
        :return: the str value corresponding to C{numeric_value} or C{None}.
        """
        for k, v in vars(self.__class__).iteritems():
            if v == numeric_value:
                return k
        return None

    def from_string(self, string_value):
        """
        Returns the C{int} equivalent of the given value.
        C{string_value} is expected to be the textual representaion
        of a flash operation return value.

        :type string_value: str
        :param string_value: the str value explaining the flash operation result.

        :rtype: int
        :return: the integer value corresponding to C{string_value} or C{None}
        """
        upper_case_value = str(string_value).upper()
        return getattr(self, upper_case_value, None)


class CmfwdlResult(FlashResult):

    """
    A class that represent the result of a flash operation.
    This class is used as a facility to use strings in
    order to describe numeric return values.
    This class is the implementation of flash operation
    when using 'cmfwdl'.
    """

    NVM_ERR_SUCESS = 0
    """
    Operation completed successfully
    """

    NVM_ERR_INTERNAL_AP_ERROR = 1
    """
    An internal error occurred (memory allocation error, etc.)
    """

    NVM_ERR_MODEM_OPEN = 2
    """
    The specified modem port could not be open
    """

    NVM_ERR_MODEM_WRITE = 3
    """
    The write to the modem failed
    """

    NVM_ERR_MODEM_READ = 4
    """
    The read from the modem failed
    """

    NVM_ERR_DELTA_FILE_NOT_FOUND = 5
    """
    The specified TLV file was not found
    """

    NVM_ERR_SET_SCRIPT_ERROR = 6
    """
    Sending the script to the modem was refused by the modem
    """

    NVM_ERR_RUN_SCRIPT_ERROR = 7
    """
    The specified TLV file contains script errors
    (invalid or not supported commands, etc.)
    """

    NVM_ERR_ID_READ_ERROR = 8
    """
    The read ID request was unsuccessful
    """

    def __init__(self):
        """
        Initializes this instance.
        """
        FlashResult.__init__(self)

    def to_string(self, numeric_value):
        """
        Returns the C{str} equivalent of the given value.
        C{numeric_value} is expected to be the return value
        of a flash operation.

        :type numeric_value: int
        :param numeric_value: the numeric value corresponding
            to a flash operation result.

        :rtype: str
        :return: the str value corresponding to C{numeric_value} or C{None}.
        """
        for k, v in vars(self.__class__).iteritems():
            if v == numeric_value:
                return k
        return None

    def from_string(self, string_value):
        """
        Returns the C{int} equivalent of the given value.
        C{string_value} is expected to be the textual representaion
        of a flash operation return value.

        :type string_value: str
        :param string_value: the str value explaining the flash operation result.

        :rtype: int
        :return: the integer value corresponding to C{string_value} or C{None}
        """
        upper_case_value = str(string_value).upper()
        return getattr(self, upper_case_value, None)
