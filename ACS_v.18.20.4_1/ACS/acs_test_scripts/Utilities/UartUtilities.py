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

:organization: INTEL NDG SW DEV
:summary: Utilities class for UART implementation
:author: jreynaux
:since: 19/08/2010
"""
import serial
import time

from ErrorHandling.DeviceException import DeviceException


class UartUtilities (object):
    """
    Utility class containing interactions methods with UART to device.
    """

    __instance = None

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(UartUtilities, cls).__new__(cls, *args, **kwargs)
            cls.__instance.__initialized = False
        return cls.__instance

    def __init__(self, logger=None):
        if self.__initialized: return

        self._port = None
        self._logger = logger
        self._logged = False
        self.__initialized = True

    @property
    def logged(self):
        """
        Indicate uart logged or not.

        :rtype: bool
        :return: Bool
        """
        return self._logged

    @logged.setter
    def logged(self, logged):
        """
        Set uart logged trigger to given value.

        :type logged: bool
        :param logged: New logged trigger value

        :rtype: None
        """
        self._logged = logged

    @property
    def configured(self):
        """
        Indicate serial port configured or not.

        :rtype: bool
        :return: Bool
        """
        return True if self._port else False

    @configured.setter
    def configured(self, configured):
        """
        Reset uart port configuration.

        :type configured: bool
        :param configured: Whether reset configuration or not

        :rtype: None
        """
        if not configured:
            self._port = None

    def log_in(self, user, password=""):
        """
        Log in as 'user' and using 'password'

        :type user: str
        :param user: User Id to use for login

        :type password: str
        :param password: Password to use for login

        :rtype: None
        """
        if not self._logged:
            if not self._port.isOpen():
                self._port.open()

            # Send login to establish connection
            if user != "":
                cmd = user + "\r\n"
                self._port.write(cmd)

            if password != "":
                cmd = password + "\r\n"
                self._port.write(cmd)

            if self._port.isOpen():
                self._port.close()

            status, ret_msg = self.uart_run_cmd("echo {0}".format(id(self)))
            if status and str(id(self)) in ret_msg:
                self._logger.debug("LOGIN OK")
                self._logged = True
            else:
                self._logger.debug("LOGIN FAIL")
                self._logged = False
        return self._logged

    def configure_port(self, port='/dev/ttyUSB0', baudrate=115200, timeout=0,
                       bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE,
                       parity=serial.PARITY_NONE, xonxoff=False):
        """
        Configure a client that will be used for SSH interaction

        :type port: str
        :param port: Port to be used for communication (tty or COM)

        :type baudrate: int
        :param baudrate: Baudrate speed of communication

        :type timeout: int
        :param timeout: Timeout for command execution

        :type bytesize: int (use serial variables)
        :param bytesize: Size of octets send to channel

        :type stopbits: int
        :param stopbits: Stop bit to use.

        :type parity: str
        :param parity: Define on which parity data are send.

        :type xonxoff: bool
        :param xonxoff: Define flow control or not.

        :return: None
        """
        self._port = serial.Serial(port=port,
                                   baudrate=baudrate,
                                   timeout=timeout,
                                   bytesize=bytesize,
                                   stopbits=stopbits,
                                   parity=parity,
                                   xonxoff=xonxoff)

    def uart_run_cmd(self, command, timeout=5, silent_mode=False, wait_for_response=True):
        """
        Run a cmd via UART to interact with Linux device

        :type command: str
        :param command: The command to execute on remote device

        :type silent_mode: bool
        :param silent_mode: Flag indicating some information are display as log.

        :rtype: tuple (bool, str)
        :return: Execution status & output string
        """
        cmd_succeed = False
        output = "Generic error"

        # If no logger force silent_mode to True
        if self._logger is None:
            silent_mode = True

        if not silent_mode:
            self._logger.debug("Execute UART command: " + str(command))

        if self._port is None:
            self.configure_port()
        else:
            if not silent_mode:
                self._logger.debug("Port already configured :  " + str(self._port))

        try:
            cmd_succeed, output = self.__do_uart_command(command, timeout, silent_mode,
                                                         wait_for_response=wait_for_response)
        except Exception as ex:
            error_msg = str(ex)
            if not silent_mode:
                self._logger.error(error_msg)
            cmd_succeed = False
            output = error_msg

        finally:
            if self._port is not None:
                if not silent_mode:
                    self._logger.debug("Close tty")
                self._port.close()
                if not silent_mode:
                    self._logger.debug("Closed: " + str(not self._port.isOpen()))

        return cmd_succeed, output

    def __do_uart_command(self, command, timeout, silent_mode=False, wait_for_response=True):
        """
            Send command to given UART port and compute result.

        :type command: str
        :param command: The (linux standard) command to be sent

        :type timeout: int
        :param timeout: Time Out to wait for sending ssh command

        :type silent_mode: bool
        :param silent_mode: Flag indicating some information are display as log.

        :rtype: tuple (int, str)
        :return: Execution cmd_succeed & output string
        """
        cmd_succeed = True
        # If no logger force silent_mode to True
        if self._logger is None:
            silent_mode = True

        if not silent_mode:
            self._logger.debug("Opened at start: " + str(self._port.isOpen()))
        if not self._port.isOpen():
            if not silent_mode:
                self._logger.debug("Open the port")
            self._port.open()
            if not silent_mode:
                self._logger.debug("Opened: " + str(self._port.isOpen()))

        # Flush input and output
        self._port.flushInput()
        self._port.flushOutput()

        time.sleep(0.5)

        if not silent_mode:
            self._logger.debug("Send command : {0}".format(command))
        self._port.write(command + "\r\n")

        uart_std_out_data = ""
        # Detect arrival of data on serial port for a while (let's give device time to answer) before reading data
        # When data starting to come, then continue to read
        end_time = time.time() + float(timeout)
        exec_time = time.time()
        start_time = exec_time
        buffer = ""
        if wait_for_response:
            while exec_time < end_time and buffer == "":
                buffer = str(self._port.read(1))
                exec_time = time.time()
                time.sleep(0.1)

            if exec_time > end_time:
                if not silent_mode:
                    self._logger.error('Uu UART Timeout!')
                msg = "UART Timeout, no data received on serial port"
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            uart_std_out_data += buffer
            # while something to read or timeout
            end_time = time.time() + float(timeout)
            exec_time = time.time()
            while exec_time < end_time and self._port.inWaiting() > 0:
                uart_std_out_data += str(self._port.read(1))
                exec_time = time.time()

            if exec_time > end_time:
                if not silent_mode:
                    self._logger.warning('Uu UART Timeout!')
                msg = "UART Timeout while reading data on serial port"
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if cmd_succeed:
            if not silent_mode:
                self._logger.debug("Uu RAW OUTPUT: \n" + uart_std_out_data + "<<")

            # remove trailing and leading whitespace
            uart_std_out_data.strip()

            # Remove first line corresponding to command sent and useless empty prompts as artefacts
            uart_list = uart_std_out_data.split('\r\n')
            uart_std_out_data = ""
            bad_words = [str(command), "root@edison:~#"]
            for line in uart_list:
                if not any(bad_word in line for bad_word in bad_words):
                    uart_std_out_data += line + '\n'

            if not silent_mode:
                self._logger.debug("Uu Clean output: \n>>\n" + uart_std_out_data + "<<")

        return cmd_succeed, uart_std_out_data

