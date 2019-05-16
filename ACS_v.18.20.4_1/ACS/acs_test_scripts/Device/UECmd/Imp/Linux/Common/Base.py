"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: UECmd based file for all Linux Application Framework Base
:since: 13/03/2014
:author: jreynaux
"""
from acs_test_scripts.Device.UECmd.Imp.UECmdBase import UECmdBase
from acs_test_scripts.Utilities.SshUtilities import SshUtilities
from acs_test_scripts.Utilities.UartUtilities import UartUtilities
from UtilitiesFWK.Utilities import str_to_bool_ex
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException

class Base(UECmdBase):
    """
    Base class containing interactions methods for Linux device.

    Commonly use ssh connection and UART
    """

    def __init__(self, device):

        super(Base, self).__init__()

        self._device = device
        self._logger = self._device.get_logger()
        self._uecmd_default_timeout = int(self._device.get_uecmd_timeout())
        self._device_ip = self._device.get_device_ip()

        self.__ssh_user = "root"

        # Path to used binaries on Linux device
        self.__path_env = "PATH=/usr/sbin/:/usr/bin:/usr/local/bin:/bin"

        self._ssh_api = SshUtilities(self._logger)
        self._uart_api = None
        # Configure SSH client
        self._ssh_api.configure_client()
        self._ssh_api.configure_credentials(self._device_ip, "root", "")

        if self._is_serial_port_used(silent_mode=True):
            self._uart_api = UartUtilities(self._logger)
            if not self._uart_api.configured:
                self._uart_api.configure_port(port=self._device.get_config("serialPort", "/dev/ttyUSB0"),
                                              baudrate=self._device.get_config("serialBaudRate", 115200, int))

            if not self._uart_api.logged:
                self._device.get_device_logger().stop("SERIAL")
                self._uart_api.log_in(self._device.get_config("serialLogin", ""),
                                      self._device.get_config("serialPassword", ""))
                self._device.get_device_logger().start("SERIAL")

    def get_ssh_api(self):
        """
        Get the ssh api from the base
        :rtype: SshUtilities
        :return: ssh_api from the base
        """
        return self._ssh_api

    def get_uart_api(self):
        """
        Get the uart api from the base
        :rtype: UartUtilities
        :return: uart_api from the base
        """
        return self._uart_api

    def _internal_uart_exec(self, cmd, timeout=5, silent_mode=False, raise_exception=True, wait_for_response=True):
        """
        Internal method that execute std command on device using the UART
        :type  cmd: str
        :param cmd: cmd to be executed

        :type timeout: int
        :param timeout: timeout for execution

        :type silent_mode: bool
        :param silent_mode: Whether verbose or not

        :type raise_exception: bool
        :param raise_exception: Whether raise a DeviceException when a failure occurs or not.

        :raise DeviceException if an error occurs during command execution
        :raise AcsConfigException if try to run command via UART but no serial connection configured

        :rtype: tuple (bool, str)
        :return Output status and output log
        """
        status = False
        output = ""

        if not silent_mode:
            self._logger.debug("Try to run command via UART")
            if self._is_serial_port_used():
                if not silent_mode:
                    self._logger.debug("Suspend UART log while handling the UART cmd")

                self._device.get_device_logger().stop("SERIAL")

                self._uart_api.log_in(self._device.get_config("serialLogin", ""),
                                  self._device.get_config("serialPassword", ""))
                status, output = self._uart_api.uart_run_cmd(cmd, timeout=timeout, silent_mode=silent_mode, wait_for_response=wait_for_response)

                self._device.get_device_logger().start("SERIAL")
            else:
                raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, "No serial connection configured")

        if not status and raise_exception:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, str(output))

        return status, output

    def _internal_exec(self, cmd, timeout=5, silent_mode=False, raise_exception=True, use_uart=True, wait_for_response=True):
        """
        Internal method that execute std command on device
        :type  cmd: str
        :param cmd: cmd to be executed

        :type timeout: int
        :param timeout: timeout for execution

        :type silent_mode: bool
        :param silent_mode: Whether verbose or not

        :type raise_exception: bool
        :param raise_exception: Whether raise a DeviceException when a failure occurs or not.

        :type use_uart: bool
        :param use_uart: Whether attempting to send the command via UART if failed via SSH

        :raise DeviceException if an error occurs during command execution
        :raise AcsConfigException if try to run command via UART but no serial connection configured

        :rtype: tuple (bool, str)
        :return Output status and output log
        """
        if not silent_mode:
            self._logger.debug("Try to run command via SSH primary")
        # Try via SSH
        status, output = self._ssh_api.ssh_run_cmd(cmd, timeout=timeout, silent_mode=silent_mode, wait_for_response=wait_for_response)

        if not status and SshUtilities.SSH_CONNECTION_ESTABLISHMENT_FAILED_ERROR_MSG in output and use_uart:
            if not silent_mode:
                self._logger.warning("Command via ssh failed, trying via UART")
            status, output = self._internal_uart_exec(cmd, timeout, silent_mode, raise_exception)

        if not status and raise_exception:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, str(output))

        return status, output

    def _is_serial_port_used(self, silent_mode=False):
        """
        Internal method that retrieves the parameters indicating if serial is used.
        :type silent_mode: bool
        :param silent_mode: Flag indicating some information are display as log.

        :rtype: boolean
        :return: true if serial enabled and configured in the config files
        """
        status = False

        if (str_to_bool_ex(self._device.get_config("retrieveSerialTrace", "False"))
           and self._device.get_config("serialPort", "") != ""
           and self._device.get_config("serialBaudRate") is not None):
            status = True
        else:
            if not silent_mode:
                self._logger.warning("Serial connection undefined in the config")

        return status
